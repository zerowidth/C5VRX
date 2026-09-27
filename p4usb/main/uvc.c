#include "uvc.h"

#include <sys/time.h>

#include "esp_heap_caps.h"
#include "tusb.h"
#include "usb_device_uvc.h"

#include "console.h"
#include "video.h"

#define UVC_BUFFER_BYTES (128 * 1024)

static uvc_fb_t s_fb;
static uint32_t s_starts;

static esp_err_t on_start(uvc_format_t format, int width, int height, int rate, void *ctx)
{
    ++s_starts;
    return ESP_OK;
}

/* Required by usb_device_uvc; nothing to tear down since the pipeline runs regardless. */
static void on_stop(void *ctx) {}

static uvc_fb_t *on_get(void *ctx)
{
    size_t len;
    const uint8_t *buf = video_acquire(&len);
    if (!buf) return NULL;
    s_fb = (uvc_fb_t){
        .buf = (uint8_t *)buf,
        .len = len,
        .width = VIDEO_WIDTH,
        .height = VIDEO_HEIGHT,
        .format = UVC_FORMAT_JPEG,
    };
    gettimeofday(&s_fb.timestamp, NULL);
    return &s_fb;
}

static void on_return(uvc_fb_t *fb, void *ctx)
{
    video_release();
}

void uvc_init(void)
{
    uvc_device_config_t cfg = {
        .uvc_buffer = heap_caps_aligned_alloc(64, UVC_BUFFER_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
        .uvc_buffer_size = UVC_BUFFER_BYTES,
        .start_cb = on_start,
        .fb_get_cb = on_get,
        .fb_return_cb = on_return,
        .stop_cb = on_stop,
    };
    assert(cfg.uvc_buffer);
    ESP_ERROR_CHECK(uvc_device_config(0, &cfg));
    ESP_ERROR_CHECK(uvc_device_init());
}

void uvc_info(void)
{
    say("usb %s%s, uvc %s, %lu start(s)\n", tud_mounted() ? "mounted" : "not mounted",
        tud_suspended() ? " (suspended)" : "", tud_video_n_streaming(0, 0) ? "streaming" : "idle", (unsigned long)s_starts);
}
