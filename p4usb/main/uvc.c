#include "uvc.h"

#include <string.h>

#include "device/usbd_pvt.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tusb.h"

#include "console.h"
#include "usb.h"
#include "video.h"

#define UVC_BUFFER_BYTES (128 * 1024)
#define PAYLOAD_HEADER 2
#define USB_HS_PACKET 512
/* Frames arrive every 16.7 ms, so this only runs out if the encoder fails. */
#define NEXT_WAIT_MS 100
/* A frame takes about 1 ms on the bus; a host that stops reading leaves it unfinished. */
#define XFER_WAIT_MS 500

static uint8_t *s_buffer;
static TaskHandle_t s_task;
static uint32_t s_starts;
static uint32_t s_sent;
static uint32_t s_skipped;
static uint32_t s_oversize;
static volatile bool s_restart;

int tud_video_commit_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx, const video_probe_and_commit_control_t *parameters)
{
    ++s_starts;
    s_restart = true;
    /* A host that closed without SET_INTERFACE can leave the last frame's transfer queued. */
    usbd_edpt_stall(TUD_OPT_RHPORT, USB_EP_VIDEO_IN);
    usbd_edpt_clear_stall(TUD_OPT_RHPORT, USB_EP_VIDEO_IN);
    usbd_edpt_release(TUD_OPT_RHPORT, USB_EP_VIDEO_IN);
    xTaskNotifyGive(s_task);
    return VIDEO_ERROR_NONE;
}

void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx)
{
    xTaskNotifyGive(s_task);
}

/* Blocks on the producer rather than a timer, so each frame goes out once and none repeat. */
static void uvc_task(void *arg)
{
    uint32_t last_seq = 0;
    for (;;) {
        if (!tud_video_n_streaming(0, 0)) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
            continue;
        }
        if (s_restart) {
            s_restart = false;
            last_seq = 0;
        }
        video_frame_t f;
        if (!video_next(VIDEO_READER_UVC, &f, pdMS_TO_TICKS(NEXT_WAIT_MS))) continue;
        if (last_seq && f.seq != last_seq + 1) s_skipped += f.seq - last_seq - 1;
        last_seq = f.seq;
        size_t len = f.len;
        if (len >= UVC_BUFFER_BYTES) {
            ++s_oversize;
            video_release(VIDEO_READER_UVC);
            continue;
        }
        memcpy(s_buffer, f.jpeg, len);
        video_release(VIDEO_READER_UVC);
        /* The class sends no zero-length packet, so a last payload filling whole packets would run into the
         * next one; a byte after EOI ends it short, and decoders ignore it. */
        size_t data = CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE - PAYLOAD_HEADER;
        if (len % data && (len % data + PAYLOAD_HEADER) % USB_HS_PACKET == 0) s_buffer[len++] = 0;
        ulTaskNotifyTake(pdTRUE, 0);
        if (!tud_video_n_frame_xfer(0, 0, s_buffer, len)) continue;
        ++s_sent;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(XFER_WAIT_MS));
    }
}

void uvc_init(void)
{
    s_buffer = heap_caps_aligned_alloc(64, UVC_BUFFER_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    assert(s_buffer);
    xTaskCreate(uvc_task, "uvc", 4096, NULL, 5, &s_task);
}

void uvc_counts(uint32_t *sent, uint32_t *skipped)
{
    *sent = s_sent;
    *skipped = s_skipped;
}

void uvc_info(void)
{
    say("usb %s%s, uvc %s, %lu start(s), %lu sent, %lu skipped, %lu too large\n",
        tud_mounted() ? "mounted" : "not mounted", tud_suspended() ? " (suspended)" : "",
        tud_video_n_streaming(0, 0) ? "streaming" : "idle", (unsigned long)s_starts, (unsigned long)s_sent,
        (unsigned long)s_skipped, (unsigned long)s_oversize);
}
