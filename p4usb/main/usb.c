#include "usb.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_mac.h"
#include "esp_private/usb_phy.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "device/dcd.h"
#include "tusb.h"

#include "bridge.h"
#include "console.h"
#include "video.h"

#define USB_VID 0x303a
/* Not the ROM loader's 0x0012 or USB-Serial/JTAG's 0x1001, so esptool resets it with DTR and RTS. */
#define USB_PID 0x8000
#define EP_CDC_NOTIF 0x81
#define EP_CDC_OUT 0x02
#define EP_CDC_IN 0x82
#define UVC_CLOCK_HZ 27000000
#define TERM_CAMERA 1
#define TERM_OUTPUT 2
/* A closed or stalled reader should cost the console a moment, not hang it; any progress restarts it. */
#define WRITE_TIMEOUT_US (200 * 1000)

enum { ITF_CDC, ITF_CDC_DATA, ITF_VIDEO_CONTROL, ITF_VIDEO_STREAMING, ITF_COUNT };
enum { STR_LANG, STR_MANUFACTURER, STR_PRODUCT, STR_SERIAL, STR_CONSOLE, STR_VIDEO };

#define FRAME_INTERVAL (10000000 / VIDEO_FPS)
#define VIDEO_DESC_LEN                                                                                         \
    (TUD_VIDEO_DESC_IAD_LEN + TUD_VIDEO_DESC_STD_VC_LEN + TUD_VIDEO_DESC_CS_VC_LEN + 1 +                       \
     TUD_VIDEO_DESC_CAMERA_TERM_LEN + TUD_VIDEO_DESC_OUTPUT_TERM_LEN + TUD_VIDEO_DESC_STD_VS_LEN +            \
     TUD_VIDEO_DESC_CS_VS_IN_LEN + 1 + TUD_VIDEO_DESC_CS_VS_FMT_MJPEG_LEN +                                   \
     TUD_VIDEO_DESC_CS_VS_FRM_MJPEG_CONT_LEN + TUD_VIDEO_DESC_CS_VS_COLOR_MATCHING_LEN + 7)
#define CONFIG_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + VIDEO_DESC_LEN)

static const tusb_desc_device_t s_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = STR_MANUFACTURER,
    .iProduct = STR_PRODUCT,
    .iSerialNumber = STR_SERIAL,
    .bNumConfigurations = 1,
};

static const uint8_t s_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_LEN, 0, 500),
    TUD_CDC_DESCRIPTOR(ITF_CDC, STR_CONSOLE, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 512),
    TUD_VIDEO_DESC_IAD(ITF_VIDEO_CONTROL, 2, STR_VIDEO),
    TUD_VIDEO_DESC_STD_VC(ITF_VIDEO_CONTROL, 0, STR_VIDEO),
    TUD_VIDEO_DESC_CS_VC(0x0150, TUD_VIDEO_DESC_CAMERA_TERM_LEN + TUD_VIDEO_DESC_OUTPUT_TERM_LEN, UVC_CLOCK_HZ,
                         ITF_VIDEO_STREAMING),
    TUD_VIDEO_DESC_CAMERA_TERM(TERM_CAMERA, 0, 0, 0, 0, 0, 0),
    TUD_VIDEO_DESC_OUTPUT_TERM(TERM_OUTPUT, VIDEO_TT_STREAMING, 0, TERM_CAMERA, 0),
    TUD_VIDEO_DESC_STD_VS(ITF_VIDEO_STREAMING, 0, 1, STR_VIDEO),
    TUD_VIDEO_DESC_CS_VS_INPUT(1,
                               TUD_VIDEO_DESC_CS_VS_FMT_MJPEG_LEN + TUD_VIDEO_DESC_CS_VS_FRM_MJPEG_CONT_LEN +
                                   TUD_VIDEO_DESC_CS_VS_COLOR_MATCHING_LEN,
                               USB_EP_VIDEO_IN, 0, TERM_OUTPUT, 0, 0, 0, 0),
    TUD_VIDEO_DESC_CS_VS_FMT_MJPEG(1, 1, 0, 1, 0, 0, 0, 0),
    TUD_VIDEO_DESC_CS_VS_FRM_MJPEG_CONT(1, 0, VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_WIDTH * VIDEO_HEIGHT * 16,
                                        VIDEO_WIDTH * VIDEO_HEIGHT * 16 * VIDEO_FPS, VIDEO_WIDTH * VIDEO_HEIGHT * 2,
                                        FRAME_INTERVAL, FRAME_INTERVAL, FRAME_INTERVAL, FRAME_INTERVAL),
    TUD_VIDEO_DESC_CS_VS_COLOR_MATCHING(VIDEO_COLOR_PRIMARIES_BT709, VIDEO_COLOR_XFER_CH_BT709,
                                        VIDEO_COLOR_COEF_SMPTE170M),
    TUD_VIDEO_DESC_EP_BULK(USB_EP_VIDEO_IN, 512, 1),
};
_Static_assert(sizeof s_config == CONFIG_LEN, "configuration descriptor length");

static char s_serial[13];
static const char *const s_strings[] = {
    [STR_MANUFACTURER] = "C5VRX",
    [STR_PRODUCT] = "C5VRX Receiver",
    [STR_SERIAL] = s_serial,
    [STR_CONSOLE] = "C5VRX Console",
    [STR_VIDEO] = "P4USB",
};

/* The P4's high-speed DWC2 controller: each IN endpoint's DIEPCTL, whose USBAEP bit the core clears on bus reset. */
#define DWC2_DIEPCTL(n) (*(volatile uint32_t *)(0x50000900 + 0x20 * (n)))
#define DIEPCTL_USBAEP (1u << 15)

bool __real_dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *desc);
void __real_dcd_edpt_close_all(uint8_t rhport);

/* The video class reopens its bulk endpoint on every SET_INTERFACE and the DWC2 port has no close, so each
 * reopen took another TX FIFO until allocation failed; reactivate an active IN endpoint in place instead. */
bool __wrap_dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *desc)
{
    uint8_t ep = desc->bEndpointAddress;
    if (tu_edpt_dir(ep) == TUSB_DIR_IN && (DWC2_DIEPCTL(tu_edpt_number(ep)) & DIEPCTL_USBAEP))
        return dcd_edpt_iso_activate(rhport, desc);
    return __real_dcd_edpt_open(rhport, desc);
}

/* Closing all re-initializes the FIFOs, so every endpoint must allocate again. */
void __wrap_dcd_edpt_close_all(uint8_t rhport)
{
    __real_dcd_edpt_close_all(rhport);
    for (int n = 1; n < 16; ++n) DWC2_DIEPCTL(n) &= ~DIEPCTL_USBAEP;
}

static usb_phy_handle_t s_phy;
static SemaphoreHandle_t s_rx_ready;

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&s_device;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    return s_config;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    static uint16_t desc[32];
    size_t n;
    if (index == STR_LANG) {
        desc[1] = 0x0409;
        n = 1;
    } else if (index < sizeof s_strings / sizeof s_strings[0] && s_strings[index]) {
        const char *s = s_strings[index];
        for (n = 0; s[n] && n < 31; ++n) desc[1 + n] = s[n];
    } else {
        return NULL;
    }
    desc[0] = TUSB_DESC_STRING << 8 | (2 * n + 2);
    return desc;
}

void tud_cdc_rx_cb(uint8_t itf)
{
    xSemaphoreGive(s_rx_ready);
}

/* esptool's reset holds RTS (EN) with DTR (IO0) released, then releases RTS with DTR held. */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    static bool in_reset;
    if (bridge_active()) return;
    if (rts && !dtr) {
        in_reset = true;
    } else if (!rts) {
        if (in_reset && dtr) console_reboot(true);
        in_reset = false;
    }
}

static void usb_task(void *arg)
{
    for (;;) tud_task();
}

void usb_init(void)
{
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(s_serial, sizeof s_serial, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    s_rx_ready = xSemaphoreCreateBinary();

    const usb_phy_config_t phy = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT,
        .otg_speed = USB_PHY_SPEED_HIGH,
    };
    ESP_ERROR_CHECK(usb_new_phy(&phy, &s_phy));
    if (!tusb_init()) abort();
    xTaskCreate(usb_task, "usb", 4096, NULL, 5, NULL);
}

void usb_console_info(void)
{
    say("usb console: mounted %d, suspended %d, DTR %d, %lu bytes free to send\n", tud_mounted(), tud_suspended(),
        tud_cdc_connected(), (unsigned long)tud_cdc_write_available());
}

bool usb_console_connected(void)
{
    return tud_cdc_connected();
}

void usb_console_write(const void *data, size_t len)
{
    const uint8_t *p = data;
    int64_t deadline = esp_timer_get_time() + WRITE_TIMEOUT_US;
    while (len && tud_cdc_connected()) {
        uint32_t n = tud_cdc_write(p, len);
        tud_cdc_write_flush();
        p += n;
        len -= n;
        if (n) deadline = esp_timer_get_time() + WRITE_TIMEOUT_US;
        if (len) {
            if (esp_timer_get_time() > deadline) return;
            vTaskDelay(1);
        }
    }
}

size_t usb_console_read(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    if (!tud_cdc_available()) xSemaphoreTake(s_rx_ready, pdMS_TO_TICKS(timeout_ms));
    return tud_cdc_read(buf, len);
}
