#include "video.h"

#include <stdio.h>
#include <string.h>

#include "driver/jpeg_encode.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "bridge.h"
#include "console.h"
#include "overlay.h"
#include "uvc.h"

/* One held per reader, the newest finished, and one being encoded, so the producer never waits. */
#define JPEG_SLOTS (VIDEO_READERS + 2)
#define JPEG_SLOT_BYTES (128 * 1024)
/* Room in front of the encoder's output for a COM segment; a multiple of the cache line. */
#define HEAD_ROOM 128
/* Noise fields overflow a slot at the normal quality, so quality drops on overflow and climbs back
 * while frames stay small. */
#define JPEG_QUALITY 70
#define JPEG_QUALITY_MIN 20
#define JPEG_QUALITY_STEP 10
#define JPEG_RAISE_AFTER 30
#define TEXT_SCALE 3
#define MARKER 24

typedef struct {
    uint8_t *base;
    size_t cap;
    video_frame_t frame;
} slot_t;

static slot_t s_slots[JPEG_SLOTS];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_newest = -1;
static int s_held[VIDEO_READERS] = {-1, -1};
static uint32_t s_taken[VIDEO_READERS];
static SemaphoreHandle_t s_published;
static uint8_t *s_raw;
static size_t s_raw_size;
static size_t s_field_size;
/* The decoder fills one while the encoder reads the other. */
static uint8_t *s_field[2];
static int s_fill;
static volatile int s_ready = -1;
static int64_t s_last_field_us;
static jpeg_encoder_handle_t s_enc;
static TaskHandle_t s_task;

static uint32_t s_frames;
static uint32_t s_errors;
static esp_err_t s_last_error;
static uint32_t s_render_us;
static uint32_t s_encode_us;
static uint32_t s_fps_tenths;
static int s_quality = JPEG_QUALITY;
static int s_small_frames;

#define NOTIFY_TICK 1
#define NOTIFY_FIELD 2
/* Without a decoded field for this long, the test pattern takes over. */
#define PATTERN_AFTER_US (100 * 1000)

static void tick(void *arg)
{
    xTaskNotify(s_task, NOTIFY_TICK, eSetBits);
}

static int writable_slot(void)
{
    taskENTER_CRITICAL(&s_lock);
    int w = 0;
    for (;; ++w) {
        bool busy = w == s_newest;
        for (int r = 0; r < VIDEO_READERS; ++r) busy |= w == s_held[r];
        if (!busy) break;
    }
    taskEXIT_CRITICAL(&s_lock);
    return w;
}

static size_t newest_len(void)
{
    taskENTER_CRITICAL(&s_lock);
    size_t len = s_newest < 0 ? 0 : s_slots[s_newest].frame.len;
    taskEXIT_CRITICAL(&s_lock);
    return len;
}

static void render(uint32_t seq)
{
    uint32_t ms = esp_timer_get_time() / 1000;
    uint32_t sent, skipped;
    uvc_counts(&sent, &skipped);
    char lines[9][32];
    snprintf(lines[0], sizeof lines[0], "C5VRX P4 TEST PATTERN");
    snprintf(lines[1], sizeof lines[1], "uptime %02lu:%02lu:%02lu.%03lu", (unsigned long)(ms / 3600000),
             (unsigned long)(ms / 60000 % 60), (unsigned long)(ms / 1000 % 60), (unsigned long)(ms % 1000));
    snprintf(lines[2], sizeof lines[2], "frame  %lu", (unsigned long)seq);
    snprintf(lines[3], sizeof lines[3], "rate   %lu.%lu fps", (unsigned long)(s_fps_tenths / 10),
             (unsigned long)(s_fps_tenths % 10));
    snprintf(lines[4], sizeof lines[4], "jpeg   %u bytes", (unsigned)newest_len());
    snprintf(lines[5], sizeof lines[5], "encode %lu us", (unsigned long)s_encode_us);
    snprintf(lines[6], sizeof lines[6], "usb    %lu sent %lu skip", (unsigned long)sent, (unsigned long)skipped);
    snprintf(lines[7], sizeof lines[7], "video  no decoded fields");
    snprintf(lines[8], sizeof lines[8], "bridge %s", bridge_active() ? "active" : "idle");

    memset(s_raw, 0, s_raw_size);
    for (int i = 0; i < 9; ++i) {
        overlay_text(s_raw, VIDEO_WIDTH, 48, 48 + i * 10 * TEXT_SCALE, TEXT_SCALE, lines[i]);
    }
    /* A sweeping block makes dropped or repeated frames visible. */
    int span = VIDEO_WIDTH - MARKER;
    int x = (int)(seq * 8 % (2 * span));
    if (x > span) x = 2 * span - x;
    overlay_box(s_raw, VIDEO_WIDTH, x, VIDEO_HEIGHT - 2 * MARKER, MARKER, MARKER, 0xff);
}

/* Moves SOI and APP0 back into the head room and puts a COM segment after them, so every JPEG
 * names its own frame number and capture time even after it leaves the P4. */
static uint8_t *add_comment(uint8_t *jpeg, uint32_t *len, uint32_t seq, int64_t t_us)
{
    size_t keep = 2;
    if (jpeg[2] == 0xff && jpeg[3] == 0xe0) keep += 2 + (jpeg[4] << 8 | jpeg[5]);
    char text[HEAD_ROOM - 8];
    int n = snprintf(text, sizeof text, "C5VRX frame=%lu t_us=%lld", (unsigned long)seq, (long long)t_us);
    uint8_t *start = jpeg - (4 + n);
    memmove(start, jpeg, keep);
    uint8_t *com = start + keep;
    com[0] = 0xff;
    com[1] = 0xfe;
    com[2] = (n + 2) >> 8;
    com[3] = (n + 2) & 0xff;
    memcpy(com + 4, text, n);
    *len += 4 + n;
    return start;
}

static void encode(const uint8_t *raw, size_t size, bool color, uint32_t seq, int64_t t_us)
{
    const jpeg_encode_cfg_t cfg = {
        .width = VIDEO_WIDTH,
        .height = VIDEO_HEIGHT,
        .src_type = color ? JPEG_ENCODE_IN_FORMAT_YUV422 : JPEG_ENCODE_IN_FORMAT_GRAY,
        .sub_sample = color ? JPEG_DOWN_SAMPLING_YUV422 : JPEG_DOWN_SAMPLING_GRAY,
        .image_quality = s_quality,
    };
    int w = writable_slot();
    slot_t *s = &s_slots[w];
    uint8_t *out = s->base + HEAD_ROOM;
    uint32_t len = 0;
    esp_err_t err = jpeg_encoder_process(s_enc, &cfg, raw, size, out, s->cap - HEAD_ROOM, &len);
    if (err != ESP_OK) {
        ++s_errors;
        s_last_error = err;
        if (s_quality > JPEG_QUALITY_MIN) s_quality -= JPEG_QUALITY_STEP;
        s_small_frames = 0;
        return;
    }
    if (len > (s->cap - HEAD_ROOM) / 3 || s_quality >= JPEG_QUALITY) {
        s_small_frames = 0;
    } else if (++s_small_frames >= JPEG_RAISE_AFTER) {
        s_quality += JPEG_QUALITY_STEP;
        s_small_frames = 0;
    }
    const uint8_t *jpeg = add_comment(out, &len, seq, t_us);
    taskENTER_CRITICAL(&s_lock);
    s->frame = (video_frame_t){.jpeg = jpeg, .len = len, .seq = seq, .t_us = t_us};
    s_newest = w;
    taskEXIT_CRITICAL(&s_lock);
    xSemaphoreGive(s_published);
}

static void video_task(void *arg)
{
    int64_t window_start = esp_timer_get_time();
    uint32_t window_frames = 0;
    for (;;) {
        uint32_t bits = 0;
        xTaskNotifyWait(0, UINT32_MAX, &bits, portMAX_DELAY);
        uint32_t seq = s_frames + 1;
        int64_t t0 = esp_timer_get_time();
        const uint8_t *raw;
        bool field = bits & NOTIFY_FIELD;
        if (field) {
            raw = s_field[s_ready];
            s_last_field_us = t0;
        } else if (t0 - s_last_field_us > PATTERN_AFTER_US) {
            render(seq);
            raw = s_raw;
        } else {
            continue;
        }
        int64_t t1 = esp_timer_get_time();
        encode(raw, field ? s_field_size : s_raw_size, field, seq, t0);
        int64_t t2 = esp_timer_get_time();
        s_render_us = t1 - t0;
        s_encode_us = t2 - t1;
        s_frames = seq;

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
    /* It logs every overflow, which the quality control handles. */
    esp_log_level_set("jpeg.encoder", ESP_LOG_NONE);
    const jpeg_encode_engine_cfg_t eng = {.timeout_ms = 100};
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&eng, &s_enc));

    const jpeg_encode_memory_alloc_cfg_t in = {.buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER};
    const jpeg_encode_memory_alloc_cfg_t out = {.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER};
    s_raw = jpeg_alloc_encoder_mem(VIDEO_WIDTH * VIDEO_HEIGHT, &in, &s_raw_size);
    assert(s_raw);
    for (int i = 0; i < 2; ++i) {
        s_field[i] = jpeg_alloc_encoder_mem(VIDEO_WIDTH * VIDEO_HEIGHT * 2, &in, &s_field_size);
        assert(s_field[i]);
    }
    for (int i = 0; i < JPEG_SLOTS; ++i) {
        s_slots[i].base = jpeg_alloc_encoder_mem(JPEG_SLOT_BYTES, &out, &s_slots[i].cap);
        assert(s_slots[i].base);
    }
    s_published = xSemaphoreCreateBinary();

    /* On core 1, below the demodulator: the encoder driver's cache write-back of each frame is the
     * biggest CPU cost here, and core 0 draws the decoder's lines. */
    xTaskCreatePinnedToCore(video_task, "video", 4096, NULL, 4, &s_task, 1);
    const esp_timer_create_args_t timer = {.callback = tick, .name = "video"};
    esp_timer_handle_t h;
    ESP_ERROR_CHECK(esp_timer_create(&timer, &h));
    ESP_ERROR_CHECK(esp_timer_start_periodic(h, 1000000 / VIDEO_FPS));
}

uint8_t *video_field_buffer(void)
{
    return s_field[s_fill];
}

void video_field_done(void)
{
    s_ready = s_fill;
    s_fill ^= 1;
    xTaskNotify(s_task, NOTIFY_FIELD, eSetBits);
}

static bool take(video_reader_t who, video_frame_t *f, bool only_new)
{
    bool ok = false;
    taskENTER_CRITICAL(&s_lock);
    if (s_newest >= 0 && (!only_new || s_slots[s_newest].frame.seq != s_taken[who])) {
        s_held[who] = s_newest;
        *f = s_slots[s_newest].frame;
        s_taken[who] = f->seq;
        ok = true;
    }
    taskEXIT_CRITICAL(&s_lock);
    return ok;
}

bool video_next(video_reader_t who, video_frame_t *f, TickType_t wait)
{
    TickType_t start = xTaskGetTickCount();
    while (!take(who, f, true)) {
        TickType_t waited = xTaskGetTickCount() - start;
        if (waited >= wait || xSemaphoreTake(s_published, wait - waited) != pdTRUE) return false;
    }
    return true;
}

bool video_newest(video_reader_t who, video_frame_t *f)
{
    return take(who, f, false);
}

void video_release(video_reader_t who)
{
    taskENTER_CRITICAL(&s_lock);
    s_held[who] = -1;
    taskEXIT_CRITICAL(&s_lock);
}

static void grab(void)
{
    static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    video_frame_t f;
    if (!video_newest(VIDEO_READER_CONSOLE, &f)) {
        say("no frame yet\n");
        return;
    }
    const uint8_t *p = f.jpeg;
    size_t len = f.len;
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
    video_release(VIDEO_READER_CONSOLE);
    say("-----END JPEG-----\n");
}

void video_command(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "grab") == 0) {
        grab();
        return;
    }
    say("%lu frames at %lu.%lu fps, %lu errors", (unsigned long)s_frames, (unsigned long)(s_fps_tenths / 10),
        (unsigned long)(s_fps_tenths % 10), (unsigned long)s_errors);
    say(s_errors ? " (last %s)\n" : "\n", esp_err_to_name(s_last_error));
    say("render %lu us, encode %lu us, jpeg %u bytes at quality %d\n", (unsigned long)s_render_us,
        (unsigned long)s_encode_us, (unsigned)newest_len(), s_quality);
    uvc_info();
}
