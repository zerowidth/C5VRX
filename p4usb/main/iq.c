#include "iq.h"

#include <stdlib.h>
#include <string.h>

#include "driver/parlio_rx.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hal/parlio_periph.h"
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
static SemaphoreHandle_t s_full;
static size_t s_want;
static size_t s_got;

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
    if (err == ESP_OK) err = parlio_rx_unit_enable(rx, true);
    if (err == ESP_OK) {
        const parlio_receive_config_t rcfg = {.delimiter = delim, .flags.partial_rx_en = 1};
        err = parlio_rx_unit_receive(rx, s_buf, samples * 2 + margin, &rcfg);
        if (err == ESP_OK) err = parlio_rx_soft_delimiter_start_stop(rx, delim, true);
        if (err == ESP_OK && xSemaphoreTake(s_full, pdMS_TO_TICKS(1000)) != pdTRUE) err = ESP_ERR_TIMEOUT;
        parlio_rx_soft_delimiter_start_stop(rx, delim, false);
        parlio_rx_unit_disable(rx);
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

static bool pick_edge(void)
{
    const parlio_sample_edge_t edges[] = {PARLIO_SAMPLE_EDGE_POS, PARLIO_SAMPLE_EDGE_NEG};
    uint32_t r[2];
    for (int e = 0; e < 2; ++e) {
        esp_err_t err = capture(s_psram, PROBE_SAMPLES, edges[e]);
        if (err != ESP_OK) {
            say("iq: capture failed, %s\n", esp_err_to_name(err));
            return false;
        }
        r[e] = roughness(PROBE_SAMPLES);
    }
    s_edge = r[1] < r[0] ? PARLIO_SAMPLE_EDGE_NEG : PARLIO_SAMPLE_EDGE_POS;
    say("roughness rise %lu.%02lu, fall %lu.%02lu: using %s\n", (unsigned long)(r[0] / 100),
        (unsigned long)(r[0] % 100), (unsigned long)(r[1] / 100), (unsigned long)(r[1] % 100),
        s_edge == PARLIO_SAMPLE_EDGE_POS ? "rise" : "fall");
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
        if (!s_psram) {
            say("iq: no memory\n");
            return false;
        }
    }
    /* Lines 0 and 8 have no pin, which the driver warns about on every capture. */
    esp_log_level_set("parlio", ESP_LOG_ERROR);
    return true;
}

bool iq_start(const char *channel, int every)
{
    if (!prepare()) return false;
    char cmd[24], reply[96];
    decode_stop();
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
    }
    return pick_edge();
}

parlio_sample_edge_t iq_edge(void)
{
    return s_edge;
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
        pick_edge();
    } else if (strcmp(sub, "dump") == 0) {
        dump();
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
        say("iq [samples | sram | start [channel [every]] | edge | dump]\n");
    }
}
