#include "link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/parlio_rx.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "hal/parlio_periph.h"
#include "soc/gpio_pins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "c5.h"
#include "console.h"
#include "pins.h"

#define SAMPLES 8192
#define BYTES (SAMPLES * 2)
/* The first words after the delimiter starts can predate a settled clock. */
#define SKIP 16

static const int RATES_MHZ[] = {10, 20, 40, 80};
static uint16_t *s_buf;

typedef struct {
    uint32_t errors;
    uint8_t bad;  /* counter bits that ever arrived wrong */
    uint8_t idle; /* other side's bits that were ever high */
    uint32_t low_ones; /* words where line 0 or 8 read low */
} result_t;

/* PARLIO line k carries I(k) for k = 1..7 and Q(k-8) for k = 9..15, so each word holds I in
 * the low byte and Q in the high byte, MSB-aligned. Lines 0 and 8 read constant 1, the midpoint
 * of the dropped LSB. */
static esp_err_t capture(uint32_t hz, parlio_sample_edge_t edge)
{
    parlio_rx_unit_config_t cfg = {
        .trans_queue_depth = 1,
        .max_recv_size = BYTES,
        .data_width = 16,
        .clk_src = PARLIO_CLK_SRC_EXTERNAL,
        .ext_clk_freq_hz = hz,
        .exp_clk_freq_hz = hz,
        .clk_in_gpio_num = BUS_WIRES[0].p4,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .flags.free_clk = 1,
    };
    for (int k = 0; k < 16; ++k) cfg.data_gpio_nums[k] = -1;
    for (int k = 1; k <= 7; ++k) {
        cfg.data_gpio_nums[k] = BUS_WIRES[15 - k].p4;    /* I1 is wire 14, I7 wire 8 */
        cfg.data_gpio_nums[8 + k] = BUS_WIRES[8 - k].p4; /* Q1 is wire 7, Q7 wire 1 */
    }

    parlio_rx_unit_handle_t rx = NULL;
    parlio_rx_delimiter_handle_t delim = NULL;
    esp_err_t err = parlio_new_rx_unit(&cfg, &rx);
    if (err != ESP_OK) return err;
    for (int k = 0; k < 16; k += 8) {
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT,
                                       soc_parlio_signals[0].rx_units[0].data_sigs[k], false);
    }
    const parlio_rx_soft_delimiter_config_t dcfg = {
        .sample_edge = edge,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = BYTES,
    };
    err = parlio_new_rx_soft_delimiter(&dcfg, &delim);
    if (err == ESP_OK) err = parlio_rx_unit_enable(rx, true);
    if (err == ESP_OK) {
        memset(s_buf, 0, BYTES);
        const parlio_receive_config_t rcfg = {.delimiter = delim};
        err = parlio_rx_unit_receive(rx, s_buf, BYTES, &rcfg);
        if (err == ESP_OK) err = parlio_rx_soft_delimiter_start_stop(rx, delim, true);
        if (err == ESP_OK) err = parlio_rx_unit_wait_all_done(rx, 500);
        parlio_rx_soft_delimiter_start_stop(rx, delim, false);
        parlio_rx_unit_disable(rx);
    }
    if (delim) parlio_del_rx_delimiter(delim);
    parlio_del_rx_unit(rx);
    return err;
}

static result_t check(char side)
{
    int shift = side == 'q' ? 9 : 1;
    int other = side == 'q' ? 1 : 9;
    result_t r = {0};
    uint8_t prev = (s_buf[SKIP] >> shift) & 0x7f;
    for (int n = SKIP + 1; n < SAMPLES; ++n) {
        uint8_t v = (s_buf[n] >> shift) & 0x7f;
        uint8_t want = (prev + 1) & 0x7f;
        if (v != want) {
            ++r.errors;
            r.bad |= v ^ want;
        }
        r.idle |= (s_buf[n] >> other) & 0x7f;
        if ((s_buf[n] & 0x0101) != 0x0101) ++r.low_ones;
        prev = v;
    }
    return r;
}

static void lanes(char *out, size_t len, char side, uint8_t bits)
{
    out[0] = '\0';
    for (int k = 0; k < 7; ++k) {
        if (bits & (1u << k)) {
            char name[5];
            snprintf(name, sizeof name, " %c%d", side == 'q' ? 'Q' : 'I', k + 1);
            strlcat(out, name, len);
        }
    }
}

static void test(char side, int mhz)
{
    char cmd[24], reply[96];
    snprintf(cmd, sizeof cmd, "link %c %d", side, mhz);
    if (!c5_request(cmd, reply, sizeof reply, 1000)) {
        say("%c %2d MHz: C5 said '%s'\n", side == 'q' ? 'Q' : 'I', mhz, reply);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    const parlio_sample_edge_t edges[] = {PARLIO_SAMPLE_EDGE_POS, PARLIO_SAMPLE_EDGE_NEG};
    for (int e = 0; e < 2; ++e) {
        say("%c %2d MHz %s: ", side == 'q' ? 'Q' : 'I', mhz, e == 0 ? "rise" : "fall");
        esp_err_t err = capture(mhz * 1000000u, edges[e]);
        if (err != ESP_OK) {
            say("capture failed, %s\n", esp_err_to_name(err));
            continue;
        }
        result_t r = check(side);
        char bad[40], idle[40];
        lanes(bad, sizeof bad, side, r.bad);
        lanes(idle, sizeof idle, side == 'q' ? 'i' : 'q', r.idle);
        say("%lu errors in %d samples", (unsigned long)r.errors, SAMPLES - SKIP - 1);
        if (r.bad) say(", wrong on%s", bad);
        if (r.idle) say(", idle lanes high:%s", idle);
        if (r.low_ones) say(", constant-1 lines low %lu times", (unsigned long)r.low_ones);
        say("\n");
    }
}

void link_run(int argc, char **argv)
{
    if (!s_buf) {
        s_buf = heap_caps_aligned_calloc(128, 1, BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_buf) {
            say("link: no memory\n");
            return;
        }
    }
    /* Lines 0 and 8 have no pin, which the driver warns about on every capture. */
    esp_log_level_set("parlio", ESP_LOG_ERROR);
    if (!c5_run_c5rx(2000)) {
        say("link: c5rx did not start\n");
        return;
    }
    int one = argc > 1 ? atoi(argv[1]) : 0;
    for (size_t i = 0; i < sizeof RATES_MHZ / sizeof RATES_MHZ[0]; ++i) {
        if (one && RATES_MHZ[i] != one) continue;
        test('q', RATES_MHZ[i]);
        test('i', RATES_MHZ[i]);
    }
    char reply[32];
    c5_request("link off", reply, sizeof reply, 1000);
}
