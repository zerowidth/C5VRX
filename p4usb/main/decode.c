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
/* Once the line's last pixel and the luma filter's lookahead are demodulated. */
#define RENDER_AT (ACTIVE_START + US(52.66) + 16)
/* A detected hsync this close to the predicted one steers the line clock. */
#define LOCK_WINDOW US(3)
#define LOST_AFTER_LINES 30
#define FIELD_LINES 262
#define FIRST_ACTIVE 20
#define ACTIVE_LINES (VIDEO_HEIGHT / 2)
/* Broad pulses begin half-way through line 3 or at the start of line 4, depending on the field. */
#define VSYNC_LINE 3
/* A vsync is believed only this close to where the line count expects one, until this many fields in a
 * row have passed without one there. */
#define VSYNC_WINDOW 6
#define VSYNC_REACQUIRE 3

/* Demodulated samples, 2.5 ms of them, which is how far drawing on the other core may fall behind. */
#define HIST 32768
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

/* The subcarrier is exactly 189/704 of the sample rate, so its phase repeats every 704 samples. The
 * mixing table's amplitude keeps a block's packed sine sum inside 16 bits. */
#define LO_PERIOD 704
#define LO_CYCLES 189
#define LO_AMP 31
/* Samples per mixed block, about 0.6 us, which sets the color resolution. */
#define MIX 8
/* The burst lies 5.3-7.8 us after the sync's leading edge. */
#define BURST_START 72
#define BURST_END 104
/* Filtered burst magnitude, in mixed units, that turns color on and back off. */
#define COLOR_ON 700
#define COLOR_OFF 400

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
/* The subcarrier at each sample of its 704-sample period (and a picture's width beyond, so a line never
 * wraps), packed as 65536 cos + sin so one multiply-add per sample mixes both. */
static int32_t s_lo[LO_PERIOD + VIDEO_WIDTH + MIX];
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
/* The same for the sum of four samples, which nulls the subcarrier's dots out of luma when there is color. */
static uint8_t s_map4[1024];
static uint8_t *s_frame;

/* Per-field counters for the status line. */
static uint32_t s_fields, s_field_hits, s_field_vsync;
static uint32_t s_last_hits, s_last_vsync_ok;
static uint32_t s_overruns;
/* Vertical syncs that moved the line count, beyond the first. */
static uint32_t s_vjumps;
/* Broad pulses that fell outside the vsync window, and fields in a row without a vsync in it. */
static uint32_t s_vsync_ignored;
static int s_vsync_missed = VSYNC_REACQUIRE;
typedef struct {
    uint32_t busy_cycles, permille;
    int64_t start;
} load_t;
static load_t s_demod_load, s_draw_load;

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

/* Yields every 1024 entries when asked, so a rebuild at high priority never holds sync up for long. */
static void build_lut(uint8_t *lut, float dc_i, float dc_q, bool yield)
{
    for (int k = 0; k < LUT_SIZE; ++k) {
        if (yield && k % 1024 == 0) vTaskDelay(1);
        int i = ((int8_t)(k << 2) >> 2) * 4 + 2, q = ((int8_t)((k >> 7) << 1) >> 1) * 2 + 1;
        float turns = atan2f(q - dc_q, i - dc_i) * (float)(0.5 / M_PI);
        lut[k] = (uint8_t)(int)lrintf(turns * 256);
    }
}

static void build_lo(void)
{
    for (int k = 0; k < LO_PERIOD + VIDEO_WIDTH + MIX; ++k) {
        float t = 2 * (float)M_PI * LO_CYCLES * (k % LO_PERIOD) / LO_PERIOD;
        s_lo[k] = (int32_t)lrintf(LO_AMP * cosf(t)) * 65536 + (int32_t)lrintf(LO_AMP * sinf(t));
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
    build_lut(spare, qi / 4.0f, qq / 4.0f, true);
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
    for (int v = -512; v < 512; ++v) {
        int pix = (v - 4 * s_blank) * 255 * 2 / (span * 20);
        s_map4[v + 512] = pix < 0 ? 0 : pix > 255 ? 255 : pix;
    }
}

/* The burst vector, filtered over lines, and the color killer's state. */
static float s_burst_re, s_burst_im;
static int s_burst;
static bool s_color;
static int s_saturation = 100;

static void flush_strip(void);
static void wait_strip(int k);

static uint32_t s_finish_max_us;

/* The drawing side's end of a field: hand the frame to the encoder. */
static void finish_field(void)
{
    int64_t t0 = esp_timer_get_time();
    flush_strip();
    wait_strip(0);
    wait_strip(1);
    video_field_done();
    s_frame = video_field_buffer();
    if (esp_timer_get_time() - t0 > s_finish_max_us) s_finish_max_us = esp_timer_get_time() - t0;
}

/* The sync side's end of a field. */
static void emit_field(void)
{
    if (!s_field_vsync && s_vsync_missed < VSYNC_REACQUIRE) ++s_vsync_missed;
    s_burst = (int)hypotf(s_burst_re, s_burst_im);
    s_color = s_burst > (s_color ? COLOR_OFF : COLOR_ON);
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
#define ROW_BYTES (VIDEO_WIDTH * 2)
static uint8_t s_strips[2][STRIP_ROWS * ROW_BYTES] __attribute__((aligned(64)));
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

static uint32_t s_prof_flush, s_prof_pix, s_prof_chroma, s_prof_lines;
static void flush_strip(void)
{
    if (!s_strip_rows) return;
    uint32_t t0 = esp_cpu_get_cycle_count();
    int rows = s_strip_rows;
    if (s_strip_row0 + rows > VIDEO_HEIGHT) rows = VIDEO_HEIGHT - s_strip_row0;
    s_strip_busy[s_strip] = true;
    if (esp_async_memcpy(s_mcp, s_frame + s_strip_row0 * ROW_BYTES, s_strips[s_strip], rows * ROW_BYTES, strip_done,
                         (void *)s_strip) != ESP_OK) {
        s_strip_busy[s_strip] = false;
        ++s_strip_errors;
    }
    s_strip ^= 1;
    s_strip_rows = 0;
    s_prof_flush += esp_cpu_get_cycle_count() - t0;
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
    uint8_t *dst = s_strips[s_strip] + s_strip_rows * ROW_BYTES;
    s_strip_rows += 2;
    return dst;
}

/* Each pixel is one sample: 720 at 13.33 MS/s span 54 us, centred on the 52.66 us picture, much as
 * 720 samples at 13.5 MHz frame it in Rec. 601. */
#define PICTURE_START (ACTIVE_START - (VIDEO_WIDTH - US(52.66)) / 2)
/* The luma filter reads one sample before and two after each pixel. */
#define LINE_SPAN (PICTURE_START + VIDEO_WIDTH + 3)
/* A line's samples in one piece, for each core, when the line straddles the end of the ring. */
static uint8_t s_wrapped[2][LINE_Q16 >> 16] __attribute__((aligned(4)));

static const int8_t *line_samples(uint32_t l, uint8_t *wrapped)
{
    uint32_t start = l % HIST;
    if (start + LINE_SPAN <= HIST) return s_hist + start;
    memcpy(wrapped, s_hist + start, HIST - start);
    memcpy(wrapped + HIST - start, s_hist, LINE_SPAN - (HIST - start));
    return (const int8_t *)wrapped;
}

/* The sum of s[k] times the subcarrier lo[k], packed as in s_lo. */
static inline int32_t mixed(const int8_t *s, const int32_t *lo)
{
    return s[0] * lo[0] + s[1] * lo[1] + s[2] * lo[2] + s[3] * lo[3] + s[4] * lo[4] + s[5] * lo[5] + s[6] * lo[6] +
           s[7] * lo[7];
}

static int32_t mixed_re(int32_t m)
{
    return (m - (int16_t)m) >> 16;
}

/* The mixing table follows the absolute sample count, so the burst's phase against it is fixed and
 * averaging over lines only removes noise. */
static void measure_burst(const int8_t *s, uint32_t l)
{
    int32_t m = 0;
    const int32_t *lo = s_lo + l % LO_PERIOD;
    for (int k = BURST_START; k + MIX <= BURST_END; k += MIX) m += mixed(s + k, lo + k);
    s_burst_re += (mixed_re(m) - s_burst_re) * 0.25f;
    s_burst_im += (-(int16_t)m - s_burst_im) * 0.25f;
}

/* U - jV = -j z conj(burst) / |burst|^2 * 20 IRE for a block's mixed sum z, since the burst sits at 180
 * degrees on the U axis with 20 IRE amplitude; then full-range Cb and Cr at 100 IRE to 255. The signs
 * were set against SMPTE bars. */
static void line_chroma(const int8_t *s, uint32_t l, uint32_t *uv)
{
    float m2 = s_burst_re * s_burst_re + s_burst_im * s_burst_im;
    /* The burst is summed over several blocks, and a chroma block's sum weighted 1 2 1 with its neighbours. */
    float k = 20.0f * (BURST_END - BURST_START) / MIX / 4 * s_saturation / 100 / m2 * 4096;
    float wr = s_burst_im * k, wi = s_burst_re * k;
    int32_t a1 = lrintf(2.925f * wr), a2 = lrintf(-2.925f * wi), a3 = lrintf(-2.074f * wi), a4 = lrintf(-2.074f * wr);
    const int32_t *lo = s_lo + l % LO_PERIOD;
    enum { N = VIDEO_WIDTH / MIX };
    int32_t zr[N + 2], zi[N + 2];
    for (int j = 0; j < N; ++j) {
        int32_t m = mixed(s + j * MIX, lo + j * MIX);
        zr[j + 1] = mixed_re(m);
        zi[j + 1] = -(int16_t)m;
    }
    zr[0] = zr[1], zi[0] = zi[1], zr[N + 1] = zr[N], zi[N + 1] = zi[N];
    /* Chroma sits where the FM noise is strongest, so trade some color resolution for a quieter picture. */
    for (int j = 0; j < N; ++j) {
        int32_t re = zr[j] + 2 * zr[j + 1] + zr[j + 2], im = zi[j] + 2 * zi[j + 1] + zi[j + 2];
        int32_t cb = 128 + ((re * a1 + im * a2) >> 12), cr = 128 + ((re * a3 + im * a4) >> 12);
        cb = cb < 0 ? 0 : cb > 255 ? 255 : cb;
        cr = cr < 0 ? 0 : cr > 255 ? 255 : cr;
        uv[j] = (uint32_t)cr | (uint32_t)cb << 16;
    }
}

/* A line for the drawing task: where it starts, where it goes, its chroma (none without color), and
 * whether it ends the field. */
typedef struct {
    uint32_t l;
    int16_t row;
    bool draw, last;
    const uint32_t *uv;
} job_t;

/* Lines queued from sync detection on core 1 to drawing on core 0, and the chroma core 1 worked out
 * for each: per block of MIX pixels, Cb and Cr placed where the encoder's words want them. */
/* The sample ring holds about 38 lines, so a longer queue would only hold stale ones. */
#define JOBS 40
static job_t s_jobs[JOBS];
static uint32_t s_job_uv[JOBS][VIDEO_WIDTH / MIX];
static const uint32_t *s_last_uv;
static volatile uint32_t s_job_head, s_job_tail;
static TaskHandle_t s_draw_task;
static uint32_t s_draw_dropped, s_draw_full, s_draw_max_lag, s_draw_max_gap_us, s_draw_max_run_us;

static void draw_line(const job_t *j)
{
    if (j->draw) {
        uint32_t *dst = (uint32_t *)strip_rows(j->row);
        const int8_t *s = line_samples(j->l, s_wrapped[0]) + PICTURE_START;
        /* The encoder reads its Y0 V Y1 U as big-endian halfwords, so in memory they are V Y0 U Y1. */
        uint32_t t0 = esp_cpu_get_cycle_count();
        ++s_prof_lines;
        if (j->uv) {
            const uint8_t *map4 = s_map4 + 512;
            const uint32_t *uv = j->uv;
            /* A running 4-sample sum, 8 pixels to one chroma word. */
            int32_t p0 = s[-1], p1 = s[0], p2 = s[1];
            int32_t sum = p0 + p1 + p2;
            for (int x = 0; x < VIDEO_WIDTH; x += MIX, s += MIX, dst += MIX / 2) {
                uint32_t c = *uv++;
                int32_t q0 = s[2], q1 = s[3], q2 = s[4], q3 = s[5], q4 = s[6], q5 = s[7], q6 = s[8], q7 = s[9];
                int32_t a0 = sum + q0, a1 = a0 - p0 + q1, a2 = a1 - p1 + q2, a3 = a2 - p2 + q3;
                int32_t a4 = a3 - q0 + q4, a5 = a4 - q1 + q5, a6 = a5 - q2 + q6, a7 = a6 - q3 + q7;
                dst[0] = (uint32_t)map4[a0] << 8 | (uint32_t)map4[a1] << 24 | c;
                dst[1] = (uint32_t)map4[a2] << 8 | (uint32_t)map4[a3] << 24 | c;
                dst[2] = (uint32_t)map4[a4] << 8 | (uint32_t)map4[a5] << 24 | c;
                dst[3] = (uint32_t)map4[a6] << 8 | (uint32_t)map4[a7] << 24 | c;
                p0 = q5;
                p1 = q6;
                p2 = q7;
                sum = a7 - q4;
            }
            dst -= VIDEO_WIDTH / 2;
        } else {
            const uint8_t *map = s_map;
            const uint8_t *u = (const uint8_t *)s;
            for (int x = 0; x < VIDEO_WIDTH; x += 2) dst[x / 2] = (uint32_t)map[u[x]] << 8 | (uint32_t)map[u[x + 1]] << 24 | 0x00800080u;
        }
        memcpy(dst + VIDEO_WIDTH / 2, dst, ROW_BYTES);
        s_prof_pix += esp_cpu_get_cycle_count() - t0;
    }
    if (j->last) finish_field();
}

static void submit(const job_t *j)
{
    uint32_t head = s_job_head;
    if (!s_draw_task) {
        draw_line(j);
        return;
    }
    if (head - s_job_tail >= JOBS) {
        ++s_draw_full;
        return;
    }
    s_jobs[head % JOBS] = *j;
    __atomic_store_n(&s_job_head, head + 1, __ATOMIC_RELEASE);
    if (head % 4 == 3 || j->last) xTaskNotifyGive(s_draw_task);
}

static void render_line(void)
{
    int row = (s_vline - FIRST_ACTIVE) * 2 + s_parity;
    bool last = s_vline == FIRST_ACTIVE + ACTIVE_LINES - 1;
    bool draw = !s_skip_render && s_vline >= FIRST_ACTIVE && s_vline < FIRST_ACTIVE + ACTIVE_LINES && row < VIDEO_HEIGHT;
    if (draw || last) {
        /* The line clock's nearest sample. */
        job_t j = {.l = s_line + (s_line_frac >> 15), .row = row, .draw = draw, .last = last};
        if (draw && s_job_head - s_job_tail < JOBS) {
            uint32_t t0 = esp_cpu_get_cycle_count();
            const int8_t *src = line_samples(j.l, s_wrapped[1]);
            if (s_hit) measure_burst(src, j.l);
            if (s_color) {
                /* Every other line's chroma serves two, which halves its cost. */
                if (s_vline % 2 == 0 || !s_last_uv) {
                    uint32_t *uv = s_job_uv[s_job_head % JOBS];
                    line_chroma(src + PICTURE_START, j.l + PICTURE_START, uv);
                    s_last_uv = uv;
                }
                j.uv = s_last_uv;
            }
            s_prof_chroma += esp_cpu_get_cycle_count() - t0;
        }
        submit(&j);
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
        bool expected = s_vline >= FIELD_LINES - VSYNC_WINDOW || s_vline <= VSYNC_LINE + VSYNC_WINDOW;
        bool first = start - s_last_broad > 4 * (LINE_Q16 >> 16);
        if (first && !expected && s_vsync_missed < VSYNC_REACQUIRE) {
            ++s_vsync_ignored;
        } else if (first) {
            s_vsync_missed = 0;
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
                /* Locked in the picture, nothing matters until the next hsync's window, so skip there. */
                if (s_hit && s_vline > VSYNC_LINE + VSYNC_WINDOW && s_vline < FIELD_LINES - VSYNC_WINDOW - 1) {
                    uint32_t skip = (s_line + (s_period >> 16) - LOCK_WINDOW - 8) & ~3u;
                    if ((int32_t)(skip - nb) > 4) nb = skip - 4;
                }
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
    if (!s_skip_detect) detect(n);
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

/* Lines are drawn here, on core 0, a few behind sync detection on core 1. */
static void draw_task(void *arg)
{
    int64_t last = esp_timer_get_time();
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        int64_t woke = esp_timer_get_time();
        uint32_t t0 = esp_cpu_get_cycle_count();
        uint32_t head = __atomic_load_n(&s_job_head, __ATOMIC_ACQUIRE);
        uint32_t pending = head - s_job_tail;
        for (; s_job_tail != head; ++s_job_tail) {
            job_t j = s_jobs[s_job_tail % JOBS];
            if (s_n - j.l > s_draw_max_lag) s_draw_max_lag = s_n - j.l;
            /* The demodulator has overwritten a line this far behind it. */
            if (j.draw && s_n - j.l > HIST - LINE_SPAN - 4096) {
                j.draw = false;
                ++s_draw_dropped;
            }
            draw_line(&j);
        }
        int64_t done = esp_timer_get_time();
        if (pending > 8 && woke - last > s_draw_max_gap_us) s_draw_max_gap_us = woke - last;
        if (done - woke > s_draw_max_run_us) s_draw_max_run_us = done - woke;
        last = done;
        account(&s_draw_load, t0);
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
        build_lut(s_luts[0], 0, 0, false);
        build_lo();
        async_memcpy_config_t mcfg = ASYNC_MEMCPY_DEFAULT_CONFIG();
        if (esp_async_memcpy_install_gdma_axi(&mcfg, &s_mcp) != ESP_OK) {
            say("decode: no DMA channel for the frame\n");
            return;
        }
        /* Above sync, so the gain comes down even when a clipped signal floods the sync detector. */
        xTaskCreatePinnedToCore(control_task, "decode_ctl", 3072, NULL, 6, NULL, 0);
        set_thresholds();
        s_frame = video_field_buffer();
        /* Demodulation and sync detection stream through every sample on core 1; drawing shares core 0 with
         * USB and the console, above them since level tasks would time-slice it 1 ms at a time. */
        xTaskCreatePinnedToCore(draw_task, "decode_draw", 4096, NULL, 7, &s_draw_task, 0);
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
    say("%lu vertical corrections, %lu broad pulses ignored outside the vsync window\n", (unsigned long)s_vjumps,
        (unsigned long)s_vsync_ignored);

    say("gain %d (%s), I/Q RMS %d, %d.%d%% clipped\n", s_gain, s_agc ? "auto" : "fixed", s_rms,
        s_clip_permille / 10, s_clip_permille % 10);
    say("sync tip %d kHz, blanking %d kHz, I/Q DC %d.%02d %d.%02d\n", s_tip * KHZ_PER_UNIT, s_blank * KHZ_PER_UNIT,
        s_lut_dc_i / 4, abs(s_lut_dc_i % 4) * 25, s_lut_dc_q / 4, abs(s_lut_dc_q % 4) * 25);
    say("color %s, burst %d (on above %d), saturation %d%%\n", s_color ? "on" : "off", s_burst, COLOR_ON, s_saturation);
    say("draw: %lu stale, %lu queue full, max lag %lu samples\n", (unsigned long)s_draw_dropped, (unsigned long)s_draw_full, (unsigned long)s_draw_max_lag);
    say("longest field hand-off %lu us, longest wait with 8+ lines queued %lu us, longest run %lu us\n",
        (unsigned long)s_finish_max_us, (unsigned long)s_draw_max_gap_us, (unsigned long)s_draw_max_run_us);
    s_draw_max_gap_us = s_draw_max_run_us = 0;
    s_draw_max_lag = s_finish_max_us = 0;
    say("%lu waits on the frame DMA, %lu errors\n", (unsigned long)s_strip_waits, (unsigned long)s_strip_errors);
    say("longest gap between ring reads since last asked %lld us, of the %lld us the ring holds\n", s_max_gap_us,
        RING_US);
    s_max_gap_us = 0;
    say("demod and sync on core 1 %lu.%lu%% with %lu overruns, drawing on core 0 %lu.%lu%% with %lu lines dropped\n",
        (unsigned long)(s_demod_load.permille / 10), (unsigned long)(s_demod_load.permille % 10),
        (unsigned long)s_overruns, (unsigned long)(s_draw_load.permille / 10),
        (unsigned long)(s_draw_load.permille % 10), (unsigned long)s_draw_dropped);
}

/* Runs the stopped decoder over whatever the ring holds, to price each stage per demodulated sample. */
static void bench(void)
{
    if (!s_ring || s_running) {
        say("decode: bench needs a stopped decoder that has run\n");
        return;
    }
    TaskHandle_t draw = s_draw_task;
    s_draw_task = NULL;
    const uint32_t small = 2048, small_rounds = 200;
    s_skip_detect = true;
    uint32_t c0 = esp_cpu_get_cycle_count();
    for (uint32_t r = 0; r < small_rounds; ++r) process(s_ring, small);
    uint32_t cs = esp_cpu_get_cycle_count() - c0;
    say("demod, 4 KB cached: %lu.%02lu cycles per sample\n", (unsigned long)(cs / (small * small_rounds)),
        (unsigned long)(cs * 100ull / (small * small_rounds) % 100));
    const uint32_t words = RING_BYTES / 2, rounds = 20;
    s_prof_flush = s_prof_pix = s_prof_chroma = s_prof_lines = 0;
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
    s_draw_task = draw;
    if (s_prof_lines)
        say("per rendered line: %lu cycles, %lu of them chroma; strip flushes %lu cycles per line\n",
            (unsigned long)(s_prof_pix / s_prof_lines), (unsigned long)(s_prof_chroma / s_prof_lines),
            (unsigned long)(s_prof_flush / s_prof_lines));
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
    } else if (strcmp(sub, "sat") == 0 && argc > 2) {
        s_saturation = atoi(argv[2]);
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
        say("decode [on [channel] | off | gain auto|N | sat PERCENT | rx | tap | bench]\n");
    }
}
