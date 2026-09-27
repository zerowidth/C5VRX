#include "video.h"

#include <stdio.h>
#include <string.h>

#include "driver/jpeg_encode.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bridge.h"
#include "console.h"
#include "overlay.h"

/* One being sent, the newest finished, and one being encoded, so the producer never waits on USB. */
#define JPEG_SLOTS 3
#define JPEG_SLOT_BYTES (128 * 1024)
#define JPEG_QUALITY 80
#define TEXT_SCALE 3
#define MARKER 24

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t len;
} slot_t;

static slot_t s_slots[JPEG_SLOTS];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_newest = -1;
static int s_held = -1;
static uint8_t *s_raw;
static size_t s_raw_size;
static jpeg_encoder_handle_t s_enc;
static TaskHandle_t s_task;

static uint32_t s_frames;
static uint32_t s_late;
static uint32_t s_acquired;
static uint32_t s_errors;
static uint32_t s_render_us;
static uint32_t s_encode_us;
static uint32_t s_fps_tenths;

static void tick(void *arg)
{
    xTaskNotifyGive(s_task);
}

static int writable_slot(void)
{
    int w = 0;
    taskENTER_CRITICAL(&s_lock);
    while (w == s_newest || w == s_held) ++w;
    taskEXIT_CRITICAL(&s_lock);
    return w;
}

static size_t newest_len(void)
{
    taskENTER_CRITICAL(&s_lock);
    size_t len = s_newest < 0 ? 0 : s_slots[s_newest].len;
    taskEXIT_CRITICAL(&s_lock);
    return len;
}

static void render(void)
{
    int64_t now = esp_timer_get_time();
    uint32_t ms = now / 1000;
    char lines[9][32];
    snprintf(lines[0], sizeof lines[0], "C5VRX P4 TEST PATTERN");
    snprintf(lines[1], sizeof lines[1], "uptime %02lu:%02lu:%02lu.%03lu", (unsigned long)(ms / 3600000),
             (unsigned long)(ms / 60000 % 60), (unsigned long)(ms / 1000 % 60), (unsigned long)(ms % 1000));
    snprintf(lines[2], sizeof lines[2], "frame  %lu", (unsigned long)s_frames);
    snprintf(lines[3], sizeof lines[3], "rate   %lu.%lu fps", (unsigned long)(s_fps_tenths / 10),
             (unsigned long)(s_fps_tenths % 10));
    snprintf(lines[4], sizeof lines[4], "jpeg   %u bytes", (unsigned)newest_len());
    snprintf(lines[5], sizeof lines[5], "encode %lu us", (unsigned long)s_encode_us);
    snprintf(lines[6], sizeof lines[6], "sent   %lu late %lu", (unsigned long)s_acquired, (unsigned long)s_late);
    snprintf(lines[7], sizeof lines[7], "psram  %u KB free",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
    snprintf(lines[8], sizeof lines[8], "bridge %s", bridge_active() ? "active" : "idle");

    memset(s_raw, 0, s_raw_size);
    for (int i = 0; i < 9; ++i) {
        overlay_text(s_raw, VIDEO_WIDTH, 48, 48 + i * 10 * TEXT_SCALE, TEXT_SCALE, lines[i]);
    }
    /* A sweeping block makes dropped or repeated frames visible. */
    int span = VIDEO_WIDTH - MARKER;
    int x = (int)(s_frames * 8 % (2 * span));
    if (x > span) x = 2 * span - x;
    overlay_box(s_raw, VIDEO_WIDTH, x, VIDEO_HEIGHT - 2 * MARKER, MARKER, MARKER, 0xff);
}

static void encode(void)
{
    static const jpeg_encode_cfg_t cfg = {
        .width = VIDEO_WIDTH,
        .height = VIDEO_HEIGHT,
        .src_type = JPEG_ENCODE_IN_FORMAT_GRAY,
        .sub_sample = JPEG_DOWN_SAMPLING_GRAY,
        .image_quality = JPEG_QUALITY,
    };
    int w = writable_slot();
    uint32_t len = 0;
    if (jpeg_encoder_process(s_enc, &cfg, s_raw, s_raw_size, s_slots[w].buf, s_slots[w].cap, &len) != ESP_OK) {
        ++s_errors;
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    s_slots[w].len = len;
    s_newest = w;
    taskEXIT_CRITICAL(&s_lock);
}

static void video_task(void *arg)
{
    int64_t window_start = esp_timer_get_time();
    uint32_t window_frames = 0;
    for (;;) {
        uint32_t ticks = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ticks > 1) s_late += ticks - 1;

        int64_t t0 = esp_timer_get_time();
        render();
        int64_t t1 = esp_timer_get_time();
        encode();
        int64_t t2 = esp_timer_get_time();
        s_render_us = t1 - t0;
        s_encode_us = t2 - t1;
        ++s_frames;

        ++window_frames;
        if (t2 - window_start >= 1000000) {
            s_fps_tenths = window_frames * 10000000LL / (t2 - window_start);
            window_start = t2;
            window_frames = 0;
        }
    }
}

void video_init(void)
{
    const jpeg_encode_engine_cfg_t eng = {.timeout_ms = 100};
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&eng, &s_enc));

    const jpeg_encode_memory_alloc_cfg_t in = {.buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER};
    const jpeg_encode_memory_alloc_cfg_t out = {.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER};
    s_raw = jpeg_alloc_encoder_mem(VIDEO_WIDTH * VIDEO_HEIGHT, &in, &s_raw_size);
    assert(s_raw);
    for (int i = 0; i < JPEG_SLOTS; ++i) {
        s_slots[i].buf = jpeg_alloc_encoder_mem(JPEG_SLOT_BYTES, &out, &s_slots[i].cap);
        assert(s_slots[i].buf);
    }

    xTaskCreate(video_task, "video", 4096, NULL, 4, &s_task);
    const esp_timer_create_args_t timer = {.callback = tick, .name = "video"};
    esp_timer_handle_t h;
    ESP_ERROR_CHECK(esp_timer_create(&timer, &h));
    ESP_ERROR_CHECK(esp_timer_start_periodic(h, 1000000 / VIDEO_FPS));
}

const uint8_t *video_acquire(size_t *len)
{
    const uint8_t *buf = NULL;
    taskENTER_CRITICAL(&s_lock);
    if (s_newest >= 0) {
        s_held = s_newest;
        buf = s_slots[s_held].buf;
        *len = s_slots[s_held].len;
    }
    taskEXIT_CRITICAL(&s_lock);
    if (buf) ++s_acquired;
    return buf;
}

void video_release(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_held = -1;
    taskEXIT_CRITICAL(&s_lock);
}

static void grab(void)
{
    static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t len;
    const uint8_t *p = video_acquire(&len);
    if (!p) {
        say("no frame yet\n");
        return;
    }
    say("-----BEGIN JPEG %u-----\n", (unsigned)len);
    char line[80];
    size_t n = 0;
    for (size_t i = 0; i < len; i += 3) {
        uint32_t v = p[i] << 16 | (i + 1 < len ? p[i + 1] << 8 : 0) | (i + 2 < len ? p[i + 2] : 0);
        line[n++] = b64[v >> 18 & 63];
        line[n++] = b64[v >> 12 & 63];
        line[n++] = i + 1 < len ? b64[v >> 6 & 63] : '=';
        line[n++] = i + 2 < len ? b64[v & 63] : '=';
        if (n == 76 || i + 3 >= len) {
            line[n++] = '\n';
            host_write(line, n);
            n = 0;
        }
    }
    video_release();
    say("-----END JPEG-----\n");
}

void video_command(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "grab") == 0) {
        grab();
        return;
    }
    say("%lu frames at %lu.%lu fps, %lu late, %lu errors, %lu acquired\n", (unsigned long)s_frames,
        (unsigned long)(s_fps_tenths / 10), (unsigned long)(s_fps_tenths % 10), (unsigned long)s_late,
        (unsigned long)s_errors, (unsigned long)s_acquired);
    say("render %lu us, encode %lu us, jpeg %u bytes\n", (unsigned long)s_render_us,
        (unsigned long)s_encode_us, (unsigned)newest_len());
}
