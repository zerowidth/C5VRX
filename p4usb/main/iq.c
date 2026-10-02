#include "iq.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/bitscrambler.h"
#include "driver/parlio_rx.h"
#include "esp_cache.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hal/bitscrambler_ll.h"
#include "hal/parlio_periph.h"
#include "hal/bitscrambler_peri_select.h"
#include "soc/gpio_pins.h"

#include "c5.h"
#include "console.h"
#include "decode.h"
#include "pins.h"

#define BUS_HZ 80000000
/* Two NTSC fields, so a whole field is in every capture. */
#define CAPTURE_SAMPLES (1400 * 1000)
#define CAPTURE_BYTES (CAPTURE_SAMPLES * 2)
/* The DMA keeps writing, circularly, until the task stops it; this absorbs that latency, which into
 * PSRAM has exceeded 64 KB. */
#define MARGIN_BYTES (64 * 1024)
#define PSRAM_MARGIN_BYTES (1024 * 1024)
/* The soft delimiter's limit; with partial receive it only sets the EOF interrupt period. */
#define EOF_BYTES 0xfc00
/* Enough to compare edges: about 100 lines. */
#define PROBE_SAMPLES (256 * 1024)
#define SKIP 64

static uint16_t *s_psram;
static uint16_t *s_sram;
static size_t s_sram_samples;
/* The buffer the last capture went to. */
static uint16_t *s_buf;
static size_t s_len;
static parlio_sample_edge_t s_edge = PARLIO_SAMPLE_EDGE_POS;
/* The C5 keeps one of every s_every samples of its 80 MS/s bus. */
static int s_every = 2;
static char s_tuned[8];
static unsigned s_mhz;
static SemaphoreHandle_t s_full;
static size_t s_want;
static size_t s_got;
/* A BitScrambler program to run between PARLIO RX and its DMA during a capture, for `iq bs`. */
static const void *s_program;
static bitscrambler_state_t s_bs_state;

BITSCRAMBLER_PROGRAM(bs_pass, "bs_pass");
BITSCRAMBLER_PROGRAM(bs_half, "bs_half");
BITSCRAMBLER_PROGRAM(bs_phase_check, "bs_phase_check");
BITSCRAMBLER_PROGRAM(bs_phase, "bs_phase");

/* The companded phase table bs_phase.bsasm looks up (its layout is there): 32 log-spaced magnitude codes a lane. */
#define BS_CODES 32
#define BS_KNEE 6.0f
#define BS_MAG_MAX 160.0f
/* In PSRAM: internal RAM is short by the time the decoder allocates its ring. */
static uint16_t *s_bs_lut;
static bool s_load_lut;
/* The lane tables for a second offset, written over the first while a capture runs, for `iq bs relut`. */
static uint16_t *s_bs_lut2;
static bool s_relut;
static uint32_t s_relut_cycles;

static float bs_code_scale(void)
{
    return BS_CODES / log2f(1 + BS_MAG_MAX / BS_KNEE);
}

/* A lane's 7 bits as wired, around its offset, to a magnitude code with the sign in bit 5. */
static uint8_t bs_lane_entry(int bits, float dc)
{
    float v = 2 * ((int8_t)(bits << 1) >> 1) + 1 - dc;
    int code = (int)(bs_code_scale() * log2f(1 + fabsf(v) / BS_KNEE));
    int raw = 2 * ((int8_t)(bits << 1) >> 1) + 1;
    return (code >= BS_CODES ? BS_CODES - 1 : code) | (v < 0 ? 0x20 : 0) | (raw >= 125 || raw <= -125 ? 0x40 : 0);
}

float iq_bs_code_centre(int code)
{
    float lo = BS_KNEE * (exp2f(code / bs_code_scale()) - 1), hi = BS_KNEE * (exp2f((code + 1) / bs_code_scale()) - 1);
    return (lo + hi) / 2;
}

void iq_bs_build_lut(uint16_t *lut, float dc_i, float dc_q)
{
    for (int k = 0; k < 1024; ++k) {
        int angle = (int)lrintf(atan2f(iq_bs_code_centre(k & 31), iq_bs_code_centre(k >> 5)) * (float)(64 / M_PI_2));
        lut[k] = (angle > 63 ? 63 : angle) << 8;
    }
    /* Q's table comes twice, for I positive and negative: bit 5 is set where the signs differ (the odd
     * quadrants) and bit 7 is Q's sign. */
    for (int k = 0; k < 128; ++k) {
        uint8_t q = bs_lane_entry(k, dc_q), neg = q & 0x20;
        lut[k] |= bs_lane_entry(k, dc_i);
        lut[128 + k] |= q | neg << 2;
        lut[384 + k] |= (q ^ 0x20) | neg << 2;
    }
}

static void build_bs_lut(float dc_i, float dc_q)
{
    iq_bs_build_lut(s_bs_lut, dc_i, dc_q);
}

static bool IRAM_ATTR on_partial(parlio_rx_unit_handle_t rx, const parlio_rx_event_data_t *e, void *arg)
{
    BaseType_t woken = pdFALSE;
    size_t before = s_got;
    s_got += e->recv_bytes;
    if (before < s_want && s_got >= s_want) xSemaphoreGiveFromISR(s_full, &woken);
    return woken == pdTRUE;
}

esp_err_t iq_new_rx(size_t max_recv_size, parlio_rx_unit_handle_t *ret)
{
    parlio_rx_unit_config_t cfg = {
        .trans_queue_depth = 1,
        .max_recv_size = max_recv_size,
        .data_width = 16,
        .clk_src = PARLIO_CLK_SRC_EXTERNAL,
        .ext_clk_freq_hz = BUS_HZ / s_every,
        .exp_clk_freq_hz = BUS_HZ / s_every,
        .clk_in_gpio_num = BUS_WIRES[0].p4,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .flags.free_clk = 1,
    };
    for (int k = 0; k < 16; ++k) cfg.data_gpio_nums[k] = -1;
    for (int k = 1; k <= 7; ++k) {
        cfg.data_gpio_nums[k] = BUS_WIRES[15 - k].p4;
        cfg.data_gpio_nums[8 + k] = BUS_WIRES[8 - k].p4;
    }
    esp_err_t err = parlio_new_rx_unit(&cfg, ret);
    if (err != ESP_OK) return err;
    for (int k = 0; k < 16; k += 8) {
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT,
                                       soc_parlio_signals[0].rx_units[0].data_sigs[k], false);
    }
    return ESP_OK;
}

/* Same line map as the link test: I in the low byte, Q in the high byte, lines 0 and 8 at 1. */
static esp_err_t capture(uint16_t *buf, size_t samples, parlio_sample_edge_t edge)
{
    s_buf = buf;
    const size_t margin = buf == s_psram ? PSRAM_MARGIN_BYTES : MARGIN_BYTES;
    parlio_rx_unit_handle_t rx = NULL;
    parlio_rx_delimiter_handle_t delim = NULL;
    esp_err_t err = iq_new_rx(samples * 2 + margin, &rx);
    if (err != ESP_OK) return err;
    const parlio_rx_soft_delimiter_config_t dcfg = {
        .sample_edge = edge,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = EOF_BYTES,
    };
    const parlio_rx_event_callbacks_t cbs = {.on_partial_receive = on_partial};
    s_want = samples * 2;
    s_got = 0;
    xSemaphoreTake(s_full, 0);
    err = parlio_new_rx_soft_delimiter(&dcfg, &delim);
    if (err == ESP_OK) err = parlio_rx_unit_register_event_callbacks(rx, &cbs, NULL);
    bitscrambler_handle_t bs = NULL;
    if (err == ESP_OK && s_program) {
        const bitscrambler_config_t bcfg = {.dir = BITSCRAMBLER_DIR_RX, .attach_to = SOC_BITSCRAMBLER_ATTACH_PARL_IO};
        err = bitscrambler_new(&bcfg, &bs);
        if (err == ESP_OK) err = bitscrambler_enable(bs);
        if (err == ESP_OK) err = bitscrambler_load_program(bs, s_program);
        if (err == ESP_OK && s_load_lut) err = bitscrambler_load_lut(bs, s_bs_lut, 1024 * sizeof *s_bs_lut);
        if (err == ESP_OK) err = bitscrambler_reset(bs);
        if (err == ESP_OK) err = bitscrambler_start(bs);
    }
    if (err == ESP_OK) err = parlio_rx_unit_enable(rx, true);
    if (err == ESP_OK) {
        const parlio_receive_config_t rcfg = {.delimiter = delim, .flags.partial_rx_en = 1};
        err = parlio_rx_unit_receive(rx, s_buf, samples * 2 + margin, &rcfg);
        if (err == ESP_OK) err = parlio_rx_soft_delimiter_start_stop(rx, delim, true);
        if (err == ESP_OK && s_relut) {
            vTaskDelay(pdMS_TO_TICKS(30));
            /* The LUT ignores writes while the program runs. The host sees it as 32-bit words whatever
             * width the program reads it at. */
            uint32_t t0 = esp_cpu_get_cycle_count();
            bitscrambler_reset(bs);
            bitscrambler_load_lut(bs, s_bs_lut2, 256 * sizeof *s_bs_lut2);
            bitscrambler_start(bs);
            s_relut_cycles = esp_cpu_get_cycle_count() - t0;
        }
        if (err == ESP_OK && xSemaphoreTake(s_full, pdMS_TO_TICKS(1000)) != pdTRUE) err = ESP_ERR_TIMEOUT;
        if (bs) s_bs_state = bitscrambler_ll_get_current_state(BITSCRAMBLER_LL_GET_HW(0), BITSCRAMBLER_DIR_RX);
        parlio_rx_soft_delimiter_start_stop(rx, delim, false);
        parlio_rx_unit_disable(rx);
    }
    if (bs) {
        bitscrambler_disable(bs);
        bitscrambler_free(bs);
    }
    if (delim) parlio_del_rx_delimiter(delim);
    parlio_del_rx_unit(rx);
    if (err == ESP_OK && s_got > samples * 2 + margin) {
        say("iq: DMA wrapped %u bytes past the end\n", (unsigned)(s_got - samples * 2 - margin));
        err = ESP_ERR_INVALID_SIZE;
    }
    if (err == ESP_OK) {
        esp_cache_msync(s_buf, samples * 2, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        s_len = samples;
    }
    return err;
}

/* An edge that lands on a bus transition mixes two samples' bits, which shows as large jumps. */
static uint32_t roughness(size_t samples)
{
    uint64_t sum = 0;
    for (size_t n = SKIP + 1; n < samples; ++n) {
        sum += abs((int8_t)s_buf[n] - (int8_t)s_buf[n - 1]);
        sum += abs((int8_t)(s_buf[n] >> 8) - (int8_t)(s_buf[n - 1] >> 8));
    }
    return (uint32_t)(sum * 100 / (samples - SKIP - 1));
}

static void stats(void)
{
    int64_t sum_i = 0, sum_q = 0;
    uint64_t power = 0;
    int min_i = 127, max_i = -128, min_q = 127, max_q = -128;
    uint32_t clipped = 0, bad_ones = 0;
    for (size_t n = SKIP; n < s_len; ++n) {
        int i = (int8_t)s_buf[n], q = (int8_t)(s_buf[n] >> 8);
        sum_i += i;
        sum_q += q;
        power += i * i + q * q;
        if (i < min_i) min_i = i;
        if (i > max_i) max_i = i;
        if (q < min_q) min_q = q;
        if (q > max_q) max_q = q;
        if (i <= -127 || i >= 127 || q <= -127 || q >= 127) ++clipped;
        if ((s_buf[n] & 0x0101) != 0x0101) ++bad_ones;
    }
    size_t n = s_len - SKIP;
    uint32_t rms = 0;
    for (uint64_t p = power / n; (uint64_t)(rms + 1) * (rms + 1) <= p; ++rms) {
    }
    say("%u samples: I %d..%d mean %d, Q %d..%d mean %d, rms %lu, clipped %lu, roughness %lu.%02lu\n",
        (unsigned)n, min_i, max_i, (int)(sum_i / (int64_t)n), min_q, max_q, (int)(sum_q / (int64_t)n),
        (unsigned long)rms, (unsigned long)clipped, (unsigned long)(roughness(s_len) / 100),
        (unsigned long)(roughness(s_len) % 100));
    if (bad_ones) say("constant-1 lines read low %lu times\n", (unsigned long)bad_ones);
}

/* The low byte of each word alone, as bs_half leaves it. */
static void byte_stats(size_t bytes)
{
    const int8_t *b = (const int8_t *)s_buf;
    int64_t sum = 0;
    uint64_t rough = 0;
    uint32_t bad_ones = 0;
    for (size_t n = SKIP; n < bytes; ++n) {
        sum += b[n];
        rough += abs(b[n] - b[n - 1]);
        if (!(b[n] & 1)) ++bad_ones;
    }
    size_t n = bytes - SKIP;
    say("%u bytes: mean %d, roughness %lu.%02lu, constant-1 line low %lu times\n", (unsigned)n, (int)(sum / (int64_t)n),
        (unsigned long)(rough / n), (unsigned long)(rough * 100 / n % 100), (unsigned long)bad_ones);
}

/* Every word of bs_phase_check carries the lanes its phase byte was looked up from. The BitScrambler drops
 * a few bytes at the capture's first EOF, so the check starts after it and finds the words' byte offset. */
static void phase_check(size_t bytes, float dc_i, float dc_q)
{
    const size_t skip = 2 * 2 * EOF_BYTES;
    if (bytes < skip + 4096) {
        say("too short to check: need more than %u samples\n", (unsigned)(skip / 2));
        return;
    }
    uint32_t best = UINT32_MAX, best_off = 0, best_old = 0, best_new = 0, best_switch = 0;
    uint32_t words = (bytes - skip) / 4 - 1;
    for (uint32_t off = 0; off < 4; ++off) {
        uint32_t wrong = 0, old = 0, new = 0, first_new = 0;
        for (uint32_t n = 0; n < words; ++n) {
            uint32_t w;
            memcpy(&w, (const uint8_t *)s_buf + skip + off + 4 * n, 4);
            int li = (w >> 8) & 0x7f, lq = 128 + ((w >> 15 & 1) | (w >> 26) << 1);
            uint8_t want[2];
            for (int t = 0; t < 2; ++t) {
                uint8_t i = t ? s_bs_lut2[li] : s_bs_lut[li], q = t ? s_bs_lut2[lq] : s_bs_lut[lq];
                want[t] = s_bs_lut[(i & 31) << 5 | (q & 31)] >> 8 | (i & 0x20) << 1 | (q & 0x20) << 2;
            }
            if ((uint8_t)w == want[0]) {
                old += want[0] != want[1];
            } else if (s_relut && (uint8_t)w == want[1]) {
                if (!new++) first_new = n;
            } else {
                ++wrong;
            }
        }
        if (wrong < best) {
            best = wrong;
            best_off = off;
            best_old = old;
            best_new = new;
            best_switch = first_new;
        }
    }
    say("%lu samples around offset %d, %d: %lu phase bytes differ from the CPU's lookup, at byte offset %lu\n",
        (unsigned long)words, (int)dc_i, (int)dc_q, (unsigned long)best, (unsigned long)best_off);
    if (s_relut) {
        say("rewritten in a %lu-cycle halt: %lu samples match only the first table, %lu only the second, from sample %lu\n",
            (unsigned long)s_relut_cycles, (unsigned long)best_old, (unsigned long)best_new, (unsigned long)best_switch);
    }
}

/* Where bs_phase's marked byte changes lane, which is where the DMA's buffer has a hole. */
static void phase_frames(size_t bytes)
{
    const uint8_t *b = (const uint8_t *)s_buf;
    int lane = -1, changes = 0;
    say("marker lane by byte offset:");
    for (size_t n = SKIP; n + 64 <= bytes; n += 2) {
        int odd = 0, even = 0;
        for (int k = 0; k < 64; k += 2) {
            even += b[n + k] >> 7;
            odd += b[n + k + 1] >> 7;
        }
        int now = odd == 32 && even < 32 ? 1 : even == 32 && odd < 32 ? 0 : -1;
        if (now >= 0 && now != lane) {
            if (changes++ < 12) say(" %u:%d", (unsigned)n, now);
            lane = now;
        }
    }
    say(", %d changes\n", changes);
}

static void bs_capture(const char *name, size_t samples, float dc_i, float dc_q)
{
    static const char *const STATES[] = {"idle", "run", "wait", "paused", "unknown"};
    bool half = strcmp(name, "half") == 0, relut = strcmp(name, "relut") == 0, check = relut || strcmp(name, "check") == 0;
    bool phase = strncmp(name, "phase", 5) == 0;
    if (phase) {
        relut = name[5] == 'h';
        s_program = bs_phase;
        s_load_lut = true;
        build_bs_lut(dc_i + 24, dc_q - 16);
        memcpy(s_bs_lut2, s_bs_lut, 256 * sizeof *s_bs_lut2);
        build_bs_lut(dc_i, dc_q);
        s_relut = relut;
        esp_err_t err = capture(s_psram, samples, s_edge);
        s_program = NULL;
        s_load_lut = s_relut = false;
        say("bitscrambler %s: capture %s, DMA wrote %u bytes\n", name, esp_err_to_name(err), (unsigned)s_got);
        if (err == ESP_OK) phase_frames(samples * 2);
        return;
    }
    if (!half && !check && strcmp(name, "pass") != 0) {
        say("iq bs pass|half|check|relut|phase [samples [dc_i dc_q]]\n");
        return;
    }
    s_program = half ? bs_half : check ? bs_phase_check : bs_pass;
    s_load_lut = check;
    if (relut) {
        build_bs_lut(dc_i + 24, dc_q - 16);
        memcpy(s_bs_lut2, s_bs_lut, 256 * sizeof *s_bs_lut2);
    }
    if (check) build_bs_lut(dc_i, dc_q);
    if (!relut) memcpy(s_bs_lut2, s_bs_lut, 256 * sizeof *s_bs_lut2);
    s_relut = relut;
    esp_err_t err = capture(s_psram, samples, s_edge);
    s_program = NULL;
    s_load_lut = false;
    say("bitscrambler %s: capture %s, DMA wrote %u bytes, state %s\n", name, esp_err_to_name(err), (unsigned)s_got,
        STATES[s_bs_state]);
    if (err != ESP_OK) return;
    if (check) phase_check(samples * 2, dc_i, dc_q);
    s_relut = false;
    if (check) return;
    if (half) byte_stats(samples * 2);
    else stats();
}

/* A PSRAM capture's first 64,512 bytes hold the capture before it. */
#define STALE 32256
/* Enough fresh samples to set the gain by. */
#define GAIN_SAMPLES (STALE + 65536)
#define PROBE_GAIN_MIN 22
#define PROBE_GAIN_MAX 70

static int s_probe_gain = 52;

typedef struct {
    /* Per mille of samples standing further outside their two neighbours than the lane's RMS, for I and Q. */
    uint32_t glitch[2];
    uint32_t rms, clip;
    /* Per mille of samples within half the mean power of it: most of an FM carrier's, 4 in 10 of noise's. */
    uint32_t ring;
    /* Per mille under 0.35 or over 2.1 times the mean power: off a carrier's ring, as garbage reads are. */
    uint32_t off_ring;
} quality_t;

/* A lane sampled while the C5's bus is changing reads garbage where it crosses zero, since every bit turns
 * over there: Q read -19, -113, 45 in one bad start. That happens in about three C5 starts in ten, on the Q
 * lanes, and adds some 16 dB of noise to the picture. Neither P4 edge avoids it, and restarting the C5's
 * clock does not move it; only a C5 reset does. */
static quality_t quality(size_t samples)
{
    quality_t q = {0};
    size_t n = samples - STALE - 2;
    uint64_t power = 0;
    int means[2];
    for (int lane = 0; lane < 2; ++lane) {
        int shift = lane * 8;
        int64_t sum = 0, sq = 0;
        for (size_t k = STALE; k < samples; ++k) {
            int v = (int8_t)(s_buf[k] >> shift);
            sum += v;
            sq += v * v;
            if (v >= 125 || v <= -125) ++q.clip;
        }
        int mean = means[lane] = sum / (int64_t)(samples - STALE);
        uint64_t var = sq / (samples - STALE) - (int64_t)mean * mean;
        power += var;
        int margin = 0;
        while ((uint64_t)(margin + 1) * (margin + 1) <= var) ++margin;
        uint32_t count = 0;
        for (size_t k = STALE + 1; k + 1 < samples; ++k) {
            int a = (int8_t)(s_buf[k - 1] >> shift), v = (int8_t)(s_buf[k] >> shift), b = (int8_t)(s_buf[k + 1] >> shift);
            int lo = a < b ? a : b, hi = a < b ? b : a;
            if (v < lo - margin || v > hi + margin) ++count;
        }
        q.glitch[lane] = (uint32_t)((uint64_t)count * 1000 / n);

    }
    while ((uint64_t)(q.rms + 1) * (q.rms + 1) <= power) ++q.rms;
    uint32_t near = 0, off = 0;
    for (size_t k = STALE; k < samples; ++k) {
        int i = (int8_t)s_buf[k] - means[0], v = (int8_t)(s_buf[k] >> 8) - means[1];
        uint32_t p = i * i + v * v;
        near += p > power / 2 && p < power * 3 / 2;
        off += p * 20 < power * 7 || p * 10 > power * 21;
    }
    q.ring = (uint32_t)((uint64_t)near * 1000 / (samples - STALE));
    q.off_ring = (uint32_t)((uint64_t)off * 1000 / (samples - STALE));
    q.clip = (uint32_t)((uint64_t)q.clip * 1000 / (samples - STALE));
    return q;
}

static bool set_probe_gain(int gain)
{
    char cmd[16], reply[48];
    s_probe_gain = gain;
    snprintf(cmd, sizeof cmd, "gain %d", gain);
    return c5_request(cmd, reply, sizeof reply, 500);
}

/* The C5 starts at a gain that clips a near transmitter flat, which hides everything a probe looks for. */
static bool settle_gain(void)
{
    for (int pass = 0; pass < 8; ++pass) {
        if (capture(s_psram, GAIN_SAMPLES, s_edge) != ESP_OK) return false;
        quality_t q = quality(GAIN_SAMPLES);
        int gain = s_probe_gain;
        if (q.clip > 20 || q.rms > 80) gain -= 6;
        else if (q.rms < 30) gain += 6;
        gain = gain < PROBE_GAIN_MIN ? PROBE_GAIN_MIN : gain > PROBE_GAIN_MAX ? PROBE_GAIN_MAX : gain;
        if (gain == s_probe_gain) break;
        if (!set_probe_gain(gain)) return false;
    }
    return true;
}

/* With a carrier, a lane read mid-change puts 3 or 4 samples in a hundred well off its ring and a clean one
 * almost none. The glitch count is a poorer test there: 1 to 8 clean and 6 to 26 not. Noise has no ring, and
 * glitches 6 to 9 times in a thousand read clean, as independent samples do, and 11 to 200 or hardly at all
 * otherwise, the last when the garbage is most of the lane's power. */
#define OFF_RING_MAX 10
#define NOISE_GLITCH_MIN 4
#define NOISE_GLITCH_MAX 9
#define RING_MIN 600

/* Whether the last probe had a carrier to judge by; noise is a poorer guide. */
static bool s_verified;

static bool mid_change(const quality_t *q)
{
    if (q->ring >= RING_MIN) return q->off_ring > OFF_RING_MAX;
    uint32_t worst = q->glitch[0] > q->glitch[1] ? q->glitch[0] : q->glitch[1];
    uint32_t least = q->glitch[0] < q->glitch[1] ? q->glitch[0] : q->glitch[1];
    return worst > NOISE_GLITCH_MAX || least < NOISE_GLITCH_MIN;
}

/* Captures on each edge and keeps the one read further from the change. Reports whether each edge is clean. */
static bool pick_edge(bool clean[2])
{
    const parlio_sample_edge_t edges[] = {PARLIO_SAMPLE_EDGE_POS, PARLIO_SAMPLE_EDGE_NEG};
    quality_t q[2];
    for (int e = 0; e < 2; ++e) {
        esp_err_t err = capture(s_psram, PROBE_SAMPLES, edges[e]);
        if (err != ESP_OK) {
            say("iq: capture failed, %s\n", esp_err_to_name(err));
            return false;
        }
        q[e] = quality(PROBE_SAMPLES);
        clean[e] = !mid_change(&q[e]);
    }
    uint32_t score[2];
    for (int e = 0; e < 2; ++e) score[e] = q[e].off_ring * 1000 + (q[e].glitch[0] > q[e].glitch[1] ? q[e].glitch[0] : q[e].glitch[1]);
    int e = clean[0] != clean[1] ? clean[1] : score[1] < score[0];
    s_edge = edges[e];
    s_verified = q[e].ring >= RING_MIN;
    say("per mille glitching on I and Q and off the ring: rise %lu %lu %lu, fall %lu %lu %lu; gain %d, RMS %lu%s: using %s%s\n",
        (unsigned long)q[0].glitch[0], (unsigned long)q[0].glitch[1], (unsigned long)q[0].off_ring,
        (unsigned long)q[1].glitch[0], (unsigned long)q[1].glitch[1], (unsigned long)q[1].off_ring, s_probe_gain,
        (unsigned long)q[e].rms, q[e].ring >= RING_MIN ? "" : ", no carrier", e ? "fall" : "rise",
        clean[e] ? "" : ", which reads a lane mid-change");
    return true;
}

/* Raw little-endian words between text markers, so a host script can cut them out of the console. */
static void dump(void)
{
    if (!s_len) {
        say("iq: nothing captured\n");
        return;
    }
    say("-----BEGIN IQ %u-----\n", (unsigned)(s_len * 2));
    host_write(s_buf, s_len * 2);
    say("\n-----END IQ-----\n");
}

static bool prepare(void)
{
    if (!s_psram) {
        s_psram = heap_caps_aligned_calloc(128, 1, CAPTURE_BYTES + PSRAM_MARGIN_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        s_full = xSemaphoreCreateBinary();
        s_bs_lut = heap_caps_malloc(1280 * sizeof *s_bs_lut, MALLOC_CAP_SPIRAM);
        s_bs_lut2 = s_bs_lut ? s_bs_lut + 1024 : NULL;
        if (!s_psram || !s_bs_lut) {
            say("iq: no memory\n");
            return false;
        }
    }
    /* Lines 0 and 8 have no pin, which the driver warns about on every capture. */
    esp_log_level_set("parlio", ESP_LOG_ERROR);
    return true;
}

static bool start_c5(const char *channel, int every)
{
    char cmd[24], reply[96];
    if (!c5_run_c5rx(2000)) {
        say("iq: c5rx did not start\n");
        return false;
    }
    snprintf(cmd, sizeof cmd, "tune %s", channel);
    char iq_on[16];
    snprintf(iq_on, sizeof iq_on, "iq on %d", every);
    s_every = every;
    const char *steps[] = {cmd, iq_on};
    for (int i = 0; i < 2; ++i) {
        if (!c5_request(steps[i], reply, sizeof reply, 5000)) {
            say("iq: %s: %s\n", steps[i], reply);
            return false;
        }
        say("%s\n", reply);
        /* A bare frequency tunes with no label, so fall back to what was asked for. */
        if (i == 0 && sscanf(reply, "ok tune %7s %u", s_tuned, &s_mhz) != 2) {
            s_mhz = (unsigned)atoi(reply + 8);
            strlcpy(s_tuned, channel, sizeof s_tuned);
        }
    }
    return true;
}

/* How the C5's clock edge falls against its bus is settled at its reset and wrong about three times in
 * ten. Running the clock a tick slow for a moment moves the edge some ticks later, so this slips it until
 * both of the P4's edges read clean, which puts the one in use well inside the steady part. */
#define SLIP_TRIES 16

bool iq_start(const char *channel, int every)
{
    if (!prepare()) return false;
    decode_stop();
    if (!start_c5(channel, every) || !set_probe_gain(s_probe_gain) || !settle_gain()) return false;
    bool clean[2], one_clean = false;
    for (int try = 0; try < SLIP_TRIES; ++try) {
        if (!pick_edge(clean)) return false;
        if (clean[0] && clean[1]) return true;
        /* Settle for one clean edge in the second half of the tries. */
        one_clean = clean[0] || clean[1];
        if (one_clean && try >= SLIP_TRIES / 2) return true;
        char reply[64];
        if (!c5_request("clk slip 1", reply, sizeof reply, 500)) {
            say("iq: the C5 can't slip its clock: %s\n", reply);
            return true;
        }
    }
    if (!one_clean) say("iq: no clean clock position found\n");
    return true;
}

int iq_gain(void)
{
    return s_probe_gain;
}

bool iq_verified(void)
{
    return s_verified;
}


const char *iq_channel(unsigned *mhz)
{
    *mhz = s_mhz;
    return s_tuned;
}

parlio_sample_edge_t iq_edge(void)
{
    return s_edge;
}

void iq_set_edge(parlio_sample_edge_t edge)
{
    s_edge = edge;
}

void iq_command(int argc, char **argv)
{
    if (!prepare()) return;
    const char *sub = argc > 1 ? argv[1] : "";
    /* The decoder owns the one PARLIO RX unit while it runs. */
    if (strcmp(sub, "dump") != 0) decode_stop();
    if (strcmp(sub, "start") == 0) {
        iq_start(argc > 2 ? argv[2] : "R3", argc > 3 ? atoi(argv[3]) : 2);
    } else if (strcmp(sub, "edge") == 0) {
        bool clean[2];
        pick_edge(clean);
    } else if (strcmp(sub, "dump") == 0) {
        dump();
    } else if (strcmp(sub, "bs") == 0) {
        size_t n = argc > 3 && atoi(argv[3]) > 0 && atoi(argv[3]) < CAPTURE_SAMPLES ? atoi(argv[3]) : CAPTURE_SAMPLES;
        bs_capture(argc > 2 ? argv[2] : "", n, argc > 4 ? atof(argv[4]) : 0, argc > 5 ? atof(argv[5]) : 0);
    } else if (strcmp(sub, "sram") == 0) {
        if (!s_sram) {
            size_t bytes = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) - 64 * 1024;
            bytes &= ~(size_t)127;
            s_sram = heap_caps_aligned_calloc(128, 1, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            if (!s_sram) {
                say("iq: no internal memory\n");
                return;
            }
            s_sram_samples = (bytes - MARGIN_BYTES) / 2;
        }
        esp_err_t err = capture(s_sram, s_sram_samples, s_edge);
        if (err == ESP_OK) stats();
        else say("iq: capture failed, %s\n", esp_err_to_name(err));
    } else if (strcmp(sub, "") == 0 || atoi(sub) > 0) {
        size_t n = atoi(sub) > 0 && atoi(sub) < CAPTURE_SAMPLES ? atoi(sub) : CAPTURE_SAMPLES;
        esp_err_t err = capture(s_psram, n, s_edge);
        if (err == ESP_OK) stats();
        else say("iq: capture failed, %s\n", esp_err_to_name(err));
    } else {
        say("iq [samples | sram | bs pass|half|check|relut|phase [samples [dc_i dc_q]] | start [channel [every]] | edge | dump]\n");
    }
}
