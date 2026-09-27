#include "decode.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/parlio_rx.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "console.h"
#include "iq.h"
#include "video.h"

/* The C5 sends every 6th sample of its 80 MS/s bus; the carrier plus deviation stays inside the
 * +-6.67 MHz this can follow without wrapping. */
#define EVERY 6
#define FS_HZ (80000000 / EVERY)
/* One unit of demodulated frequency is FS_HZ / 256, about 52 kHz. */
#define KHZ_PER_UNIT 52

/* NTSC timing in demodulated samples; the line is 286 / 4.5 MHz. */
#define LINE_Q16 ((uint32_t)(80000000ull * 286 * 65536 / EVERY / 4500000))
#define US(x) ((int32_t)((x) * FS_HZ / 1000000))
#define HSYNC_MIN US(3.4)
#define HSYNC_MAX US(6.4)
/* Broad vertical-sync pulses are 27.1 us; anything past 18 us counts. */
#define BROAD_MIN US(18)
/* Measured from the sync's leading edge as the detector sees it. */
#define ACTIVE_START US(9.4)
#define ACTIVE_Q16 ((uint32_t)(52.66 * FS_HZ / 1e6 * 65536))
#define STEP_Q16 (ACTIVE_Q16 / VIDEO_WIDTH)
#define RENDER_AT (ACTIVE_START + US(52.66) + 4)
/* A detected hsync this close to the predicted one steers the line clock. */
#define LOCK_WINDOW US(3)
#define LOST_AFTER_LINES 30
#define FIELD_LINES 262
#define FIRST_ACTIVE 20
#define ACTIVE_LINES (VIDEO_HEIGHT / 2)
/* Broad pulses begin on line 4. */
#define VSYNC_LINE 3

/* Demodulated samples waiting for the sync task on the other core: 2.5 ms. */
#define HIST 32768
/* The sync task keeps this far behind the demodulator at most, dropping older samples if it falls back. */
#define SYNC_MAX_LAG (HIST - 4096)
#define BOX 4

#define RING_BYTES (96 * 1024)
#define NODES 64
#define NOTIFY_EVERY 4
#define EOF_BYTES 0xfc00
/* Enough raw words per DMA node for a steady DC estimate at little cost. */
#define DC_WORDS 32
/* The top 6 bits of I and Q index the phase table: at a working gain that costs about 0.5% of the
 * sync-to-white range, and 4 KB fits the zero-wait SPM, where a lookup is several times faster than
 * from L2 memory through the L1 cache. */
#define LUT_SIZE (1 << 12)

typedef struct {
    const uint16_t *data;
    uint32_t words;
} node_t;

static parlio_rx_unit_handle_t s_rx;
static parlio_rx_delimiter_handle_t s_delim;
static uint16_t *s_ring;
static uint32_t s_ring_nodes;
static node_t s_nodes[NODES];
static volatile uint32_t s_head;
static uint32_t s_tail;
static TaskHandle_t s_task;
static TaskHandle_t s_sync_task;
static volatile bool s_running;

SPM_DRAM_ATTR static uint8_t s_lut[LUT_SIZE];
/* Built with a new DC estimate and copied into s_lut by the decode task between DMA batches. */
static uint8_t s_lut_next[LUT_SIZE];
static volatile bool s_lut_ready;
static int s_lut_dc_i = 1000, s_lut_dc_q = 1000;
static volatile int32_t s_dc_sum_i, s_dc_sum_q, s_dc_n;

/* Demodulator and sync state, owned by the decode task. */
/* Demodulated samples; the sync detector reads them four at a time. */
static int8_t s_hist[HIST] __attribute__((aligned(4)));
/* Sums of each aligned group of four demodulated samples, for the sync detector. */
static int16_t s_blocks[HIST / 4];
/* Next sample the sync detector looks at, always a multiple of 4. */
static uint32_t s_nb;
static uint32_t s_n;
static uint8_t s_prev_phase;
static bool s_low;
static uint32_t s_low_start;
static int32_t s_thr_lo, s_thr_hi;
static uint32_t s_histogram[256];

static uint32_t s_line;
static uint32_t s_line_frac;
static uint32_t s_period = LINE_Q16;
static uint32_t s_render_at;
static bool s_hit;
static int s_missed = LOST_AFTER_LINES;
static int s_vline;
static int s_parity;
static uint32_t s_last_broad;

static int s_tip = -57, s_blank = -17;
/* Demodulated sample, as a uint8_t, to pixel level. */
static uint8_t s_map[256];
static uint8_t *s_frame;

/* Per-field counters for the status line. */
static uint32_t s_fields, s_field_hits, s_field_vsync;
static uint32_t s_last_hits, s_last_vsync_ok;
static uint32_t s_overruns, s_sync_overruns;
typedef struct {
    uint32_t busy_cycles, permille;
    int64_t start;
} load_t;
static load_t s_demod_load, s_sync_load;

static bool IRAM_ATTR on_partial(parlio_rx_unit_handle_t rx, const parlio_rx_event_data_t *e, void *arg)
{
    uint32_t h = s_head;
    s_nodes[h % NODES] = (node_t){e->data, e->recv_bytes / 2};
    __atomic_store_n(&s_head, h + 1, __ATOMIC_RELEASE);
    BaseType_t woken = pdFALSE;
    if (h % NOTIFY_EVERY == NOTIFY_EVERY - 1) vTaskNotifyGiveFromISR(s_task, &woken);
    return woken == pdTRUE;
}

/* Words carry I and Q as 2v+1 for signed 7-bit v; the table sees the top 6 bits of each, so it takes
 * the midpoint of the dropped bit, 4u+2 for signed 6-bit u. */
#define IDX(x) ((((x) >> 2) & 0x3f) | (((x) >> 4) & 0xfc0))

static void build_lut(uint8_t *lut, float dc_i, float dc_q)
{
    for (int k = 0; k < LUT_SIZE; ++k) {
        int i = ((int8_t)(k << 2) >> 2) * 4 + 2, q = ((int8_t)((k >> 6) << 2) >> 2) * 4 + 2;
        float turns = atan2f(q - dc_q, i - dc_i) * (float)(0.5 / M_PI);
        lut[k] = (uint8_t)(int)lrintf(turns * 256);
    }
}

static void update_lut(void)
{
    int32_t n = s_dc_n;
    if (n < 4096) return;
    float dc_i = (float)s_dc_sum_i / n, dc_q = (float)s_dc_sum_q / n;
    s_dc_sum_i = s_dc_sum_q = s_dc_n = 0;
    /* Quarter-LSB steps, so small drift doesn't rebuild the table every time. */
    int qi = lrintf(dc_i * 4), qq = lrintf(dc_q * 4);
    if (abs(qi - s_lut_dc_i) < 2 && abs(qq - s_lut_dc_q) < 2) return;
    if (s_lut_ready) return;
    build_lut(s_lut_next, qi / 4.0f, qq / 4.0f);
    s_lut_ready = true;
    s_lut_dc_i = qi;
    s_lut_dc_q = qq;
}

/* Levels come from each field's histogram: the sync tip is the 2nd percentile and blanking is the
 * median of what lies 1-3 MHz above it, so they work before anything is locked. */
static void set_thresholds(void);

static void update_levels(void)
{
    uint32_t total = 0;
    for (int k = 0; k < 256; ++k) total += s_histogram[k];
    if (total < 1000) return;
    int tip = 0;
    for (uint32_t acc = 0; tip < 256 && (acc += s_histogram[tip]) < total / 50; ++tip) {
    }
    int lo = tip + 1000 / KHZ_PER_UNIT, hi = tip + 3000 / KHZ_PER_UNIT;
    if (hi > 255) hi = 255;
    uint32_t band = 0;
    for (int k = lo; k <= hi; ++k) band += s_histogram[k];
    int blank = lo;
    for (uint32_t acc = 0; blank < hi && (acc += s_histogram[blank]) < band / 2; ++blank) {
    }
    memset(s_histogram, 0, sizeof s_histogram);
    s_tip = (3 * s_tip + tip - 128) / 4;
    s_blank = (3 * s_blank + blank - 128) / 4;
    set_thresholds();
}

static void set_thresholds(void)
{
    int span = s_blank - s_tip;
    if (span < 8) span = 8;
    int thr = (s_tip + s_blank) / 2, hyst = span / 8;
    s_thr_lo = BOX * (thr - hyst);
    s_thr_hi = BOX * (thr + hyst);
    /* White is 100 IRE above blanking, and sync 40 below. */
    for (int v = -128; v < 128; ++v) {
        int pix = (v - s_blank) * 255 * 2 / (span * 5);
        s_map[(uint8_t)v] = pix < 0 ? 0 : pix > 255 ? 255 : pix;
    }
}

static void emit_field(void)
{
    video_field_done();
    s_frame = video_field_buffer();
    update_levels();
    ++s_fields;
    s_last_hits = s_field_hits;
    s_last_vsync_ok = s_field_vsync;
    s_field_hits = s_field_vsync = 0;
}

static bool s_skip_render, s_skip_detect;

static void render_line(void)
{
    int row = (s_vline - FIRST_ACTIVE) * 2 + s_parity;
    if (!s_skip_render && s_vline >= FIRST_ACTIVE && s_vline < FIRST_ACTIVE + ACTIVE_LINES && row < VIDEO_HEIGHT) {
        uint8_t *dst = s_frame + row * VIDEO_WIDTH;
        /* Nearest sample, rounded; interpolating costs more than the core has to spare. */
        uint32_t p = ((uint32_t)ACTIVE_START << 16) + s_line_frac + 0x8000;
        const uint8_t *map = s_map;
        const int8_t *hist = s_hist;
        const uint32_t line = s_line;
        for (int x = 0; x < VIDEO_WIDTH; ++x, p += STEP_Q16) dst[x] = map[(uint8_t)hist[(line + (p >> 16)) % HIST]];
        if (row + 1 < VIDEO_HEIGHT) memcpy(dst + VIDEO_WIDTH, dst, VIDEO_WIDTH);
    }
    if (s_vline == FIRST_ACTIVE + ACTIVE_LINES - 1) emit_field();

    /* Every 13th sample of each line is plenty for the level histogram. */
    for (uint32_t i = 0; i < (LINE_Q16 >> 16); i += 13) ++s_histogram[(uint8_t)(s_hist[(s_line + i) % HIST] + 128)];
    if (s_hit) ++s_field_hits;
    s_missed = s_hit ? 0 : s_missed + 1;
    s_hit = false;
    /* Coast through a missing vertical sync at 262.5 lines per field. */
    if (++s_vline >= FIELD_LINES + s_parity) {
        s_vline = 0;
        s_parity ^= 1;
    }
    uint32_t next = s_line_frac + s_period;
    s_line += next >> 16;
    s_line_frac = next & 0xffff;
    s_render_at = s_line + RENDER_AT;
}

static void on_pulse(uint32_t start, uint32_t width)
{
    if (width >= HSYNC_MIN && width <= HSYNC_MAX) {
        int32_t err = (int32_t)(start - s_line);
        if (s_missed >= LOST_AFTER_LINES) {
            s_line = start;
            s_line_frac = 0;
            s_period = LINE_Q16;
            s_render_at = s_line + RENDER_AT;
            s_hit = true;
        } else if (err >= -LOCK_WINDOW && err <= LOCK_WINDOW) {
            int32_t e16 = err * 65536 - (int32_t)s_line_frac;
            int32_t pos = (int32_t)s_line_frac + e16 / 8;
            s_line += pos >> 16;
            s_line_frac = pos & 0xffff;
            s_render_at = s_line + RENDER_AT;
            int32_t period = (int32_t)s_period + e16 / 256;
            const int32_t slack = LINE_Q16 / 100;
            if (period < (int32_t)LINE_Q16 - slack) period = LINE_Q16 - slack;
            if (period > (int32_t)LINE_Q16 + slack) period = LINE_Q16 + slack;
            s_period = period;
            s_hit = true;
        }
    } else if (width >= BROAD_MIN) {
        /* The first broad pulse of a vertical interval: it starts a whole line after an even field's
         * last hsync and half a line into the line for an odd field. */
        if (start - s_last_broad > 4 * (LINE_Q16 >> 16)) {
            int32_t into = (int32_t)(start - s_line);
            int32_t half = LINE_Q16 >> 17;
            s_parity = into > half / 2 && into < half * 3 / 2;
            s_vline = VSYNC_LINE;
            ++s_field_vsync;
        }
        s_last_broad = start;
    }
}

static int32_t box_at(uint32_t m)
{
    return s_hist[m % HIST] + s_hist[(m - 1) % HIST] + s_hist[(m - 2) % HIST] + s_hist[(m - 3) % HIST];
}

/* The first sample near a block crossing where the sliding 4-sample sum crosses too. */
static uint32_t refine(uint32_t block, bool falling, int32_t thr)
{
    for (uint32_t m = block - 4; m != block + 4; ++m) {
        int32_t b = box_at(m);
        if (falling ? b < thr : b > thr) return m;
    }
    return block;
}

static void detect(uint32_t until)
{
    const int16_t *blocks = s_blocks;
    uint32_t nb = s_nb, render_at = s_render_at;
    int32_t thr_lo = s_thr_lo, thr_hi = s_thr_hi;
    bool low = s_low;
    for (; (int32_t)(until - nb) >= 4; nb += 4) {
        int32_t b = blocks[(nb % HIST) / 4];
        if (low) {
            if (b > thr_hi) {
                low = false;
                uint32_t end = refine(nb + 3, false, thr_hi);
                on_pulse(s_low_start, end - s_low_start);
                render_at = s_render_at;
            }
        } else if (b < thr_lo) {
            low = true;
            s_low_start = refine(nb + 3, true, thr_lo);
        }
        if ((int32_t)(nb + 4 - render_at) >= 0) {
            render_line();
            render_at = s_render_at;
            thr_lo = s_thr_lo;
            thr_hi = s_thr_hi;
        }
    }
    s_nb = nb;
    s_low = low;
}

static void process(const uint16_t *w, uint32_t words)
{
    int32_t si = 0, sq = 0;
    for (int k = 0; k < DC_WORDS && k < (int)words; ++k) {
        si += (int8_t)w[k];
        sq += (int8_t)(w[k] >> 8);
    }
    s_dc_sum_i += si;
    s_dc_sum_q += sq;
    s_dc_n += DC_WORDS;

    const uint8_t *lut = s_lut;
    int8_t *hist = s_hist;
    uint32_t prev = s_prev_phase;
    uint32_t *out = (uint32_t *)hist;
    int16_t *blocks = s_blocks;
    uint32_t n = s_n;
    /* Unrolled by four so the loads overlap; nodes are whole multiples of four words. */
    for (uint32_t k = 0; k + 4 <= words; k += 4) {
        uint32_t x0 = w[k], x1 = w[k + 1], x2 = w[k + 2], x3 = w[k + 3];
        uint32_t p0 = lut[IDX(x0)], p1 = lut[IDX(x1)], p2 = lut[IDX(x2)], p3 = lut[IDX(x3)];
        int32_t d0 = (int8_t)(p0 - prev), d1 = (int8_t)(p1 - p0), d2 = (int8_t)(p2 - p1), d3 = (int8_t)(p3 - p2);
        prev = p3;
        uint32_t i = (n % HIST) / 4;
        out[i] = (d0 & 0xff) | (d1 & 0xff) << 8 | (d2 & 0xff) << 16 | (uint32_t)d3 << 24;
        blocks[i] = d0 + d1 + d2 + d3;
        n += 4;
    }
    __atomic_store_n(&s_n, n, __ATOMIC_RELEASE);
    s_prev_phase = prev;
    if (s_sync_task) xTaskNotifyGive(s_sync_task);
    else if (!s_skip_detect) detect(n);
}

/* Rebuilding the table takes a couple of milliseconds, too long to hold up the decode task. */
static void lut_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (s_running) update_lut();
    }
}

static void account(load_t *l, uint32_t t0)
{
    l->busy_cycles += esp_cpu_get_cycle_count() - t0;
    int64_t now = esp_timer_get_time();
    if (now - l->start >= 1000000) {
        l->permille = l->busy_cycles / (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * (uint32_t)((now - l->start) / 1000));
        l->busy_cycles = 0;
        l->start = now;
    }
}

/* Sync detection, the line clock and rendering run here, on core 0, behind the demodulator on core 1. */
static void sync_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        uint32_t t0 = esp_cpu_get_cycle_count();
        uint32_t n = __atomic_load_n(&s_n, __ATOMIC_ACQUIRE);
        if (n - s_nb > SYNC_MAX_LAG) {
            ++s_sync_overruns;
            s_nb = n - SYNC_MAX_LAG / 2;
        }
        detect(n);
        account(&s_sync_load, t0);
    }
}

static void decode_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        uint32_t t0 = esp_cpu_get_cycle_count();
        uint32_t head;
        while (s_running && s_tail != (head = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE))) {
            /* The DMA is refilling the oldest node; skip to the newest complete one. */
            if (head - s_tail >= s_ring_nodes - 1) {
                ++s_overruns;
                s_tail = head - 1;
            }
            node_t node = s_nodes[s_tail % NODES];
            ++s_tail;
            process(node.data, node.words);
        }
        account(&s_demod_load, t0);
        if (s_lut_ready) {
            memcpy(s_lut, s_lut_next, LUT_SIZE);
            s_lut_ready = false;
        }
    }
}

bool decode_running(void)
{
    return s_running;
}

void decode_stop(void)
{
    if (!s_rx) return;
    s_running = false;
    parlio_rx_soft_delimiter_start_stop(s_rx, s_delim, false);
    parlio_rx_unit_disable(s_rx);
    parlio_del_rx_delimiter(s_delim);
    parlio_del_rx_unit(s_rx);
    s_rx = NULL;
    s_delim = NULL;
}

static esp_err_t start_rx(void)
{
    const parlio_rx_soft_delimiter_config_t dcfg = {
        .sample_edge = iq_edge(),
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = EOF_BYTES,
    };
    const parlio_rx_event_callbacks_t cbs = {.on_partial_receive = on_partial};
    esp_err_t err = iq_new_rx(RING_BYTES, &s_rx);
    if (err != ESP_OK) return err;
    if ((err = parlio_new_rx_soft_delimiter(&dcfg, &s_delim)) != ESP_OK) return err;
    if ((err = parlio_rx_unit_register_event_callbacks(s_rx, &cbs, NULL)) != ESP_OK) return err;
    if ((err = parlio_rx_unit_enable(s_rx, true)) != ESP_OK) return err;
    s_tail = s_head;
    s_running = true;
    const parlio_receive_config_t rcfg = {.delimiter = s_delim, .flags.partial_rx_en = 1};
    if ((err = parlio_rx_unit_receive(s_rx, s_ring, RING_BYTES, &rcfg)) != ESP_OK) return err;
    return parlio_rx_soft_delimiter_start_stop(s_rx, s_delim, true);
}

static void start(const char *channel)
{
    if (!s_ring) {
        s_ring = heap_caps_aligned_calloc(128, 1, RING_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        if (!s_ring) {
            say("decode: no memory for the ring\n");
            return;
        }
        /* The driver splits the ring into nodes of at most 4032 bytes. */
        s_ring_nodes = RING_BYTES / 4032;
        build_lut(s_lut, 0, 0);
        xTaskCreatePinnedToCore(lut_task, "decode_lut", 3072, NULL, 1, NULL, 0);
        set_thresholds();
        s_frame = video_field_buffer();
        /* Core 1, above everything else there; the DMA interrupt stays on core 0 with the console. */
        xTaskCreatePinnedToCore(sync_task, "decode_sync", 4096, NULL, 9, &s_sync_task, 0);
        xTaskCreatePinnedToCore(decode_task, "decode", 4096, NULL, 10, &s_task, 1);
    }
    if (!iq_start(channel, EVERY)) return;
    esp_err_t err = start_rx();
    if (err != ESP_OK) {
        say("decode: %s\n", esp_err_to_name(err));
        decode_stop();
        return;
    }
    say("decoding\n");
}

static void status(void)
{
    say("%s, %lu fields, last field %lu/%d hsyncs%s, line %lu.%03lu samples\n",
        s_running ? "running" : "stopped", (unsigned long)s_fields, (unsigned long)s_last_hits, FIELD_LINES,
        s_last_vsync_ok ? ", vsync" : ", no vsync", (unsigned long)(s_period >> 16),
        (unsigned long)((s_period & 0xffff) * 1000 >> 16));
    say("sync tip %d kHz, blanking %d kHz, I/Q DC %d.%02d %d.%02d\n", s_tip * KHZ_PER_UNIT, s_blank * KHZ_PER_UNIT,
        s_lut_dc_i / 4, abs(s_lut_dc_i % 4) * 25, s_lut_dc_q / 4, abs(s_lut_dc_q % 4) * 25);
    say("demod on core 1 %lu.%lu%% with %lu overruns, sync on core 0 %lu.%lu%% with %lu\n",
        (unsigned long)(s_demod_load.permille / 10), (unsigned long)(s_demod_load.permille % 10),
        (unsigned long)s_overruns, (unsigned long)(s_sync_load.permille / 10),
        (unsigned long)(s_sync_load.permille % 10), (unsigned long)s_sync_overruns);
}

/* Runs the stopped decoder over whatever the ring holds, to price each stage per demodulated sample. */
static void bench(void)
{
    if (!s_ring || s_running) {
        say("decode: bench needs a stopped decoder that has run\n");
        return;
    }
    TaskHandle_t sync = s_sync_task;
    s_sync_task = NULL;
    const uint32_t small = 2048, small_rounds = 200;
    s_skip_detect = true;
    uint32_t c0 = esp_cpu_get_cycle_count();
    for (uint32_t r = 0; r < small_rounds; ++r) process(s_ring, small);
    uint32_t cs = esp_cpu_get_cycle_count() - c0;
    say("demod, 4 KB cached: %lu.%02lu cycles per sample\n", (unsigned long)(cs / (small * small_rounds)),
        (unsigned long)(cs * 100ull / (small * small_rounds) % 100));
    const uint32_t words = RING_BYTES / 2, rounds = 20;
    for (int pass = 0; pass < 3; ++pass) {
        s_skip_detect = pass == 0;
        s_skip_render = pass <= 1;
        uint32_t t0 = esp_cpu_get_cycle_count();
        for (uint32_t r = 0; r < rounds; ++r) process(s_ring, words);
        uint32_t cycles = esp_cpu_get_cycle_count() - t0;
        say("%s: %lu.%02lu cycles per demodulated sample\n", pass == 0 ? "demod" : pass == 1 ? "demod and sync" : "with rendering",
            (unsigned long)(cycles / (words * rounds)),
            (unsigned long)(cycles * 100ull / (words * rounds) % 100));
    }
    s_skip_render = s_skip_detect = false;
    s_sync_task = sync;
    say("budget at %d MHz: %lu cycles per sample\n", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        (unsigned long)(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000ull / FS_HZ));
}

void decode_command(int argc, char **argv)
{
    const char *sub = argc > 1 ? argv[1] : "";
    if (strcmp(sub, "on") == 0) {
        decode_stop();
        start(argc > 2 ? argv[2] : "R3");
    } else if (strcmp(sub, "off") == 0) {
        decode_stop();
    } else if (strcmp(sub, "bench") == 0) {
        bench();
    } else if (strcmp(sub, "") == 0) {
        status();
    } else {
        say("decode [on [channel] | off]\n");
    }
}
