#include "decode.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/parlio_rx.h"
#include "esp_async_memcpy.h"
#include "esp_cache.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/axi_dma_struct.h"

#include "c5.h"
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
/* Broad pulses begin half-way through line 3 or at the start of line 4, depending on the field. */
#define VSYNC_LINE 3

/* Demodulated samples waiting for the sync task on the other core: 2.5 ms. */
#define HIST 32768
/* The sync task keeps this far behind the demodulator at most, dropping older samples if it falls back. */
#define SYNC_MAX_LAG (HIST - 4096)
#define BOX 4

/* The driver fills the ring in 4032-byte DMA nodes. Each soft-delimiter EOF ends the node it lands in
 * early, so the EOF period and the ring are whole numbers of nodes and every node comes back full. */
#define NODE_BYTES 4032
#define RING_BYTES (24 * NODE_BYTES)
/* How long the DMA takes to fill the ring. */
#define RING_US ((int64_t)RING_BYTES / 2 * 1000000 / FS_HZ)
#define NOTIFY_EVERY 4
#define EOF_BYTES (8 * NODE_BYTES)
/* Enough raw words per DMA node for a steady DC estimate at little cost. */
#define DC_WORDS 32
/* The raw word shifted right by two indexes the phase table: the top 6 bits of I, Q's constant bit and
 * all 7 of Q. One shift per sample instead of five operations, and 16 KB still stays in the L1 cache. */
#define LUT_SIZE (1 << 14)

/* Gain keeps the I/Q RMS (in the lanes' 2v+1 units, full scale 127) in this range. Around 20 the
 * phase table's rounding adds a third to the picture's noise; around 60 it adds almost nothing. */
#define RMS_LOW 40
#define RMS_HIGH 72
#define CLIP_PERMILLE_MAX 5
#define GAIN_MIN 30
#define GAIN_MAX 80
#define GAIN_STEP 2

/* PARLIO RX's DMA request line on the P4's AXI-GDMA. */
#define PARLIO_DMA_PERIPH 3

static parlio_rx_unit_handle_t s_rx;
static parlio_rx_delimiter_handle_t s_delim;
static uint16_t *s_ring;
static int s_dma_ch = -1;
/* Byte offset in the ring the decoder has read up to. */
static uint32_t s_read;
static int64_t s_read_us;
static int64_t s_max_gap_us;
static TaskHandle_t s_task;
static TaskHandle_t s_sync_task;
static volatile bool s_running;

static uint8_t s_luts[2][LUT_SIZE];
static const uint8_t *volatile s_lut = s_luts[0];
static int s_lut_dc_i = 1000, s_lut_dc_q = 1000;
static volatile int32_t s_dc_sum_i, s_dc_sum_q, s_dc_n;
static volatile uint32_t s_dc_power, s_dc_clipped;
static int s_gain = 60;
static bool s_agc = true;
static int s_rms, s_clip_permille;

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
/* Vertical syncs that moved the line count, beyond the first. */
static uint32_t s_vjumps;
typedef struct {
    uint32_t busy_cycles, permille;
    int64_t start;
} load_t;
static load_t s_demod_load, s_sync_load;

/* Only a wake-up: the driver counts one node per interrupt, so it falls behind when interrupts are late
 * and two nodes finish together. The decoder reads the DMA's position from the hardware instead. */
static bool IRAM_ATTR on_partial(parlio_rx_unit_handle_t rx, const parlio_rx_event_data_t *e, void *arg)
{
    static uint32_t count;
    BaseType_t woken = pdFALSE;
    if (++count % NOTIFY_EVERY == 0) vTaskNotifyGiveFromISR(s_task, &woken);
    return woken == pdTRUE;
}

static int find_dma_channel(void)
{
    for (int ch = 0; ch < 3; ++ch) {
        if (AXI_DMA.in[ch].conf.in_peri_sel.peri_in_sel_chn == PARLIO_DMA_PERIPH) return ch;
    }
    return -1;
}

/* The ring offset of the node the DMA finished last or is still filling; everything before it is done.
 * A descriptor's second word is its buffer address. */
static uint32_t dma_offset(void)
{
    uint32_t desc = AXI_DMA.in[s_dma_ch].conf.in_dscr_bf0.val;
    if (!desc) return s_read;
    uint32_t off = ((const uint32_t *)desc)[1] - (uint32_t)s_ring;
    return off < RING_BYTES ? off : s_read;
}

/* Words carry I and Q as 2v+1 for signed 7-bit v. The table sees the top 6 bits of I, so it takes the
 * midpoint of the dropped bit, 4u+2 for signed 6-bit u. */
#define IDX(x) ((x) >> 2)

static void build_lut(uint8_t *lut, float dc_i, float dc_q)
{
    for (int k = 0; k < LUT_SIZE; ++k) {
        int i = ((int8_t)(k << 2) >> 2) * 4 + 2, q = ((int8_t)((k >> 7) << 1) >> 1) * 2 + 1;
        float turns = atan2f(q - dc_q, i - dc_i) * (float)(0.5 / M_PI);
        lut[k] = (uint8_t)(int)lrintf(turns * 256);
    }
}

static void set_gain(int gain)
{
    char cmd[16], reply[32];
    snprintf(cmd, sizeof cmd, "gain %d", gain);
    if (c5_request(cmd, reply, sizeof reply, 1000)) s_gain = gain;
}

/* Twice a second: re-center the phase table on the I/Q DC offset and steer the C5's gain. */
static void control(void)
{
    int32_t n = s_dc_n;
    if (n < 4096) return;
    float dc_i = (float)s_dc_sum_i / n, dc_q = (float)s_dc_sum_q / n;
    float power = (float)s_dc_power / n;
    uint32_t clipped = s_dc_clipped;
    s_dc_sum_i = s_dc_sum_q = s_dc_n = 0;
    s_dc_power = s_dc_clipped = 0;
    s_rms = (int)lrintf(sqrtf(fmaxf(power - dc_i * dc_i - dc_q * dc_q, 0)));
    s_clip_permille = clipped * 1000 / n;
    if (s_agc) {
        int gain = s_gain;
        if (s_clip_permille > CLIP_PERMILLE_MAX || s_rms > RMS_HIGH) gain -= GAIN_STEP;
        else if (s_rms < RMS_LOW) gain += GAIN_STEP;
        gain = gain < GAIN_MIN ? GAIN_MIN : gain > GAIN_MAX ? GAIN_MAX : gain;
        if (gain != s_gain) set_gain(gain);
    }

    /* Quarter-LSB steps, so small drift doesn't rebuild the table every time. */
    int qi = lrintf(dc_i * 4), qq = lrintf(dc_q * 4);
    if (abs(qi - s_lut_dc_i) < 2 && abs(qq - s_lut_dc_q) < 2) return;
    uint8_t *spare = s_lut == s_luts[0] ? s_luts[1] : s_luts[0];
    build_lut(spare, qi / 4.0f, qq / 4.0f);
    s_lut = spare;
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

static void flush_strip(void);
static void wait_strip(int k);

static void emit_field(void)
{
    flush_strip();
    wait_strip(0);
    wait_strip(1);
    video_field_done();
    s_frame = video_field_buffer();
    update_levels();
    ++s_fields;
    s_last_hits = s_field_hits;
    s_last_vsync_ok = s_field_vsync;
    s_field_hits = s_field_vsync = 0;
}

static bool s_skip_render, s_skip_detect;

/* Lines are rendered, doubled, into internal RAM and DMA'd to the PSRAM frame a strip at a time: the
 * core writing PSRAM through its cache costs several cycles a byte. */
#define STRIP_ROWS 16
static uint8_t s_strips[2][STRIP_ROWS * VIDEO_WIDTH] __attribute__((aligned(64)));
static int s_strip, s_strip_rows, s_strip_row0;
static volatile bool s_strip_busy[2];
static async_memcpy_handle_t s_mcp;
static uint32_t s_strip_waits, s_strip_errors;

static bool IRAM_ATTR strip_done(async_memcpy_handle_t mcp, async_memcpy_event_t *e, void *arg)
{
    s_strip_busy[(int)arg] = false;
    return false;
}

static void wait_strip(int k)
{
    if (!s_strip_busy[k]) return;
    int64_t give_up = esp_timer_get_time() + 2000;
    while (s_strip_busy[k] && esp_timer_get_time() < give_up) {
    }
    s_strip_busy[k] = false;
}

static void flush_strip(void)
{
    if (!s_strip_rows) return;
    int rows = s_strip_rows;
    if (s_strip_row0 + rows > VIDEO_HEIGHT) rows = VIDEO_HEIGHT - s_strip_row0;
    s_strip_busy[s_strip] = true;
    if (esp_async_memcpy(s_mcp, s_frame + s_strip_row0 * VIDEO_WIDTH, s_strips[s_strip], rows * VIDEO_WIDTH, strip_done,
                         (void *)s_strip) != ESP_OK) {
        s_strip_busy[s_strip] = false;
        ++s_strip_errors;
    }
    s_strip ^= 1;
    s_strip_rows = 0;
}

/* Room for rows row and row + 1 of the frame. */
static uint8_t *strip_rows(int row)
{
    if (s_strip_rows && (row != s_strip_row0 + s_strip_rows || s_strip_rows == STRIP_ROWS)) flush_strip();
    if (!s_strip_rows) {
        s_strip_waits += s_strip_busy[s_strip];
        wait_strip(s_strip);
        s_strip_row0 = row;
    }
    uint8_t *dst = s_strips[s_strip] + s_strip_rows * VIDEO_WIDTH;
    s_strip_rows += 2;
    return dst;
}

/* Which sample each pixel takes, from the line's start, for four steps of the line clock's fraction. */
#define PHASES 4
static uint16_t s_offs[PHASES][VIDEO_WIDTH];
#define LINE_SPAN (ACTIVE_START + US(52.66) + 2)
static uint8_t s_wrapped[LINE_Q16 >> 16] __attribute__((aligned(4)));

static void build_offsets(void)
{
    for (int f = 0; f < PHASES; ++f) {
        uint32_t p = ((uint32_t)ACTIVE_START << 16) + (2 * f + 1) * 65536 / (2 * PHASES) + 0x8000;
        for (int x = 0; x < VIDEO_WIDTH; ++x, p += STEP_Q16) s_offs[f][x] = p >> 16;
    }
}

static void render_line(void)
{
    int row = (s_vline - FIRST_ACTIVE) * 2 + s_parity;
    if (!s_skip_render && s_vline >= FIRST_ACTIVE && s_vline < FIRST_ACTIVE + ACTIVE_LINES && row < VIDEO_HEIGHT) {
        uint32_t *dst = (uint32_t *)strip_rows(row);
        uint32_t start = s_line % HIST;
        const uint8_t *src = (const uint8_t *)s_hist + start;
        if (start + LINE_SPAN > HIST) {
            memcpy(s_wrapped, src, HIST - start);
            memcpy(s_wrapped + HIST - start, s_hist, LINE_SPAN - (HIST - start));
            src = s_wrapped;
        }
        const uint16_t *off = s_offs[s_line_frac * PHASES >> 16];
        const uint8_t *map = s_map;
        /* Four pixels per store, so the loads needn't wait on the byte stores they might alias. */
        for (int x = 0; x < VIDEO_WIDTH; x += 4) {
            uint32_t a = src[off[x]], b = src[off[x + 1]], c = src[off[x + 2]], d = src[off[x + 3]];
            dst[x / 4] = map[a] | map[b] << 8 | map[c] << 16 | (uint32_t)map[d] << 24;
        }
        memcpy(dst + VIDEO_WIDTH / 4, dst, VIDEO_WIDTH);
    }
    if (s_vline == FIRST_ACTIVE + ACTIVE_LINES - 1) emit_field();

    /* Every 13th sample of each line is plenty for the level histogram. */
    for (uint32_t i = 0; i < (LINE_Q16 >> 16); i += 13) ++s_histogram[(uint8_t)(s_hist[(s_line + i) % HIST] + 128)];
    if (s_hit) ++s_field_hits;
    s_missed = s_hit ? 0 : s_missed + 1;
    s_hit = false;
    /* Coast through a missing vertical sync at 262.5 lines per field: an even field counts from line 4
     * to the odd field's mid-line pulse 262.5 lines later, and an odd field from line 3. */
    if (++s_vline >= FIELD_LINES + !s_parity) {
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

            /* The count wraps at 262 and 263 lines alternately, so one line either way is expected. */
            if (abs(s_vline - VSYNC_LINE) > 1 && s_vline < FIELD_LINES - 1) ++s_vjumps;
            /* An odd field's first broad pulse starts mid-way through line 3; an even field's starts line 4,
             * which is the line the clock has just begun. */
            s_vline = s_parity ? VSYNC_LINE : VSYNC_LINE + 1;
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

/* A copy of the raw words the decoder processed, for checking offline with iq.py. Copying into PSRAM
 * slows the demodulator, so a tap can itself cause overruns. */
#define TAP_WORDS (1400 * 1000)
static uint16_t *s_tap;
static volatile uint32_t s_tap_len, s_tap_want;

static void process(const uint16_t *w, uint32_t words)
{
    if (s_tap_len < s_tap_want) {
        uint32_t n = words < s_tap_want - s_tap_len ? words : s_tap_want - s_tap_len;
        memcpy(s_tap + s_tap_len, w, n * 2);
        s_tap_len += n;
    }
    int32_t si = 0, sq = 0;
    uint32_t power = 0, clipped = 0;
    for (int k = 0; k < DC_WORDS && k < (int)words; ++k) {
        int32_t i = (int8_t)w[k], q = (int8_t)(w[k] >> 8);
        si += i;
        sq += q;
        power += i * i + q * q;
        clipped += i >= 125 || i <= -125 || q >= 125 || q <= -125;
    }
    s_dc_sum_i += si;
    s_dc_sum_q += sq;
    s_dc_power += power;
    s_dc_clipped += clipped;
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
static void control_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (s_running) control();
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
        ulTaskNotifyTake(pdTRUE, 1);
        uint32_t t0 = esp_cpu_get_cycle_count();
        if (s_running) {
            uint32_t end = dma_offset();
            int64_t now = esp_timer_get_time();
            if (now - s_read_us > s_max_gap_us) s_max_gap_us = now - s_read_us;
            /* Nearly a ring's worth of time without reading means the DMA has lapped us. */
            if (now - s_read_us > RING_US - 500) {
                ++s_overruns;
                s_read = end;
            }
            while (s_read != end) {
                uint32_t stop = end > s_read ? end : RING_BYTES;
                uint8_t *p = (uint8_t *)s_ring + s_read;
                esp_cache_msync(p, stop - s_read, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
                process((const uint16_t *)p, (stop - s_read) / 2);
                s_read = stop % RING_BYTES;
            }
            s_read_us = now;
        }
        account(&s_demod_load, t0);
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
    const parlio_receive_config_t rcfg = {.delimiter = s_delim, .flags.partial_rx_en = 1};
    if ((err = parlio_rx_unit_receive(s_rx, s_ring, RING_BYTES, &rcfg)) != ESP_OK) return err;
    if ((s_dma_ch = find_dma_channel()) < 0) return ESP_ERR_NOT_FOUND;
    s_read = 0;
    s_read_us = esp_timer_get_time();
    s_running = true;
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
        build_lut(s_luts[0], 0, 0);
        build_offsets();
        async_memcpy_config_t mcfg = ASYNC_MEMCPY_DEFAULT_CONFIG();
        if (esp_async_memcpy_install_gdma_axi(&mcfg, &s_mcp) != ESP_OK) {
            say("decode: no DMA channel for the frame\n");
            return;
        }
        xTaskCreatePinnedToCore(control_task, "decode_ctl", 3072, NULL, 1, NULL, 0);
        set_thresholds();
        s_frame = video_field_buffer();
        /* The demodulator has core 1 to itself; sync and rendering share core 0 with the DMA interrupt,
         * USB and the console. */
        xTaskCreatePinnedToCore(sync_task, "decode_sync", 4096, NULL, 9, &s_sync_task, 0);
        xTaskCreatePinnedToCore(decode_task, "decode", 4096, NULL, 10, &s_task, 1);
    }
    if (!iq_start(channel, EVERY)) return;
    set_gain(s_gain);
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
    say("%lu vertical corrections\n", (unsigned long)s_vjumps);

    say("gain %d (%s), I/Q RMS %d, %d.%d%% clipped\n", s_gain, s_agc ? "auto" : "fixed", s_rms,
        s_clip_permille / 10, s_clip_permille % 10);
    say("sync tip %d kHz, blanking %d kHz, I/Q DC %d.%02d %d.%02d\n", s_tip * KHZ_PER_UNIT, s_blank * KHZ_PER_UNIT,
        s_lut_dc_i / 4, abs(s_lut_dc_i % 4) * 25, s_lut_dc_q / 4, abs(s_lut_dc_q % 4) * 25);
    say("%lu waits on the frame DMA, %lu errors\n", (unsigned long)s_strip_waits, (unsigned long)s_strip_errors);
    say("longest gap between ring reads since last asked %lld us, of the %lld us the ring holds\n", s_max_gap_us,
        RING_US);
    s_max_gap_us = 0;
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

static void tap(void)
{
    if (!s_tap) s_tap = heap_caps_malloc(TAP_WORDS * 2, MALLOC_CAP_SPIRAM);
    if (!s_tap || !s_running) {
        say("decode: tap needs memory and a running decoder\n");
        return;
    }
    s_tap_len = 0;
    s_tap_want = TAP_WORDS;
    for (int k = 0; k < 100 && s_tap_len < TAP_WORDS; ++k) vTaskDelay(pdMS_TO_TICKS(10));
    s_tap_want = 0;

    say("-----BEGIN IQ %u-----\n", (unsigned)(s_tap_len * 2));
    host_write(s_tap, s_tap_len * 2);
    say("\n-----END IQ-----\n");
}

void decode_start(const char *channel)
{
    static char last[16] = "R3";
    if (channel) strlcpy(last, channel, sizeof last);
    decode_stop();
    start(last);
}

void decode_command(int argc, char **argv)
{
    const char *sub = argc > 1 ? argv[1] : "";
    if (strcmp(sub, "on") == 0) {
        decode_start(argc > 2 ? argv[2] : NULL);
    } else if (strcmp(sub, "off") == 0) {
        decode_stop();
    } else if (strcmp(sub, "gain") == 0 && argc > 2) {
        s_agc = strcmp(argv[2], "auto") == 0;
        if (!s_agc) set_gain(atoi(argv[2]));
    } else if (strcmp(sub, "rx") == 0) {
        /* The ring reader alone, on whatever the C5 is already sending (for example its link counter). */
        decode_stop();
        esp_err_t err = s_ring ? start_rx() : ESP_ERR_INVALID_STATE;
        say(err == ESP_OK ? "receiving\n" : "decode: %s\n", esp_err_to_name(err));
    } else if (strcmp(sub, "tap") == 0) {
        tap();
    } else if (strcmp(sub, "bench") == 0) {
        bench();
    } else if (strcmp(sub, "") == 0) {
        status();
    } else {
        say("decode [on [channel] | off | gain auto|N | rx | tap | bench]\n");
    }
}
