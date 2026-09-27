#include "uvc.h"

#include "esp_heap_caps.h"
#include "tusb.h"
#include "usb_device_uvc.h"

#include "console.h"
#include "video.h"

#define UVC_BUFFER_BYTES (128 * 1024)
/* Frames arrive every 16.7 ms, so this only runs out if the encoder fails. */
#define NEXT_WAIT_MS 100

static uvc_fb_t s_fb;
static uint32_t s_starts;
static uint32_t s_sent;
static uint32_t s_skipped;
static uint32_t s_last_seq;

static esp_err_t on_start(uvc_format_t format, int width, int height, int rate, void *ctx)
{
    ++s_starts;
    s_last_seq = 0;
    return ESP_OK;
}

/* Required by usb_device_uvc; nothing to tear down since the pipeline runs regardless. */
static void on_stop(void *ctx) {}

/* usb_device_uvc polls on a whole-millisecond interval shorter than a frame, so blocking here
 * until the next frame paces the stream by the producer: each frame goes out once. */
static uvc_fb_t *on_get(void *ctx)
{
    video_frame_t f;
    if (!video_next(VIDEO_READER_UVC, &f, pdMS_TO_TICKS(NEXT_WAIT_MS))) return NULL;
    if (s_last_seq && f.seq != s_last_seq + 1) s_skipped += f.seq - s_last_seq - 1;
    s_last_seq = f.seq;
    ++s_sent;
    s_fb = (uvc_fb_t){
        .buf = (uint8_t *)f.jpeg,
        .len = f.len,
        .width = VIDEO_WIDTH,
        .height = VIDEO_HEIGHT,
        .format = UVC_FORMAT_JPEG,
        .timestamp = {.tv_sec = f.t_us / 1000000, .tv_usec = f.t_us % 1000000},
    };
    return &s_fb;
}

static void on_return(uvc_fb_t *fb, void *ctx)
{
    video_release(VIDEO_READER_UVC);
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

void uvc_counts(uint32_t *sent, uint32_t *skipped)
{
    *sent = s_sent;
    *skipped = s_skipped;
}

void uvc_info(void)
{
    say("usb %s%s, uvc %s, %lu start(s), %lu sent, %lu skipped\n", tud_mounted() ? "mounted" : "not mounted",
        tud_suspended() ? " (suspended)" : "", tud_video_n_streaming(0, 0) ? "streaming" : "idle",
        (unsigned long)s_starts, (unsigned long)s_sent, (unsigned long)s_skipped);
}
