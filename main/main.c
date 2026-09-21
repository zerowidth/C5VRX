/**
 * main.c - RSSI meter for analog FPV video on the ESP32-C5.
 *
 * PARLIO RX streams MODEM_DIAG I/Q bytes into a cyclic DMA ring with no CPU
 * involvement. Once per millisecond a task averages signal power over the
 * ring and prints one line over USB. With an 8-bit single-component layout,
 * power is 2 * mean(Q^2) (or I^2): FM keeps the phasor rotating, so each
 * component carries half the power on average. See viewer/ for the host side and README.md
 * for the line protocol.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/parlio_rx.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/dma_types.h"
#include "hal/usb_serial_jtag_ll.h"
#include "soc/ahb_dma_struct.h"
#include "soc/parl_io_struct.h"

#include "rf.h"
#include "rhnode.h"
#include "rx5808.h"

#define IQ_RATE_HZ    40000000u
#define RING_BYTES    16384u
#define SAMPLE_STRIDE 4u
#define SAMPLES_PER_READING (RING_BYTES / SAMPLE_STRIDE)
#define DUMP_BYTES    1024u
#define DUMP_CHUNK    128u
#define PARLIO_PERI_ID 9
#define STATUS_PERIOD_MS 1000u
#define PEAK_WINDOW_MS 100u

static const char *TAG = "rssi";

static DMA_ATTR __attribute__((aligned(64))) uint8_t s_ring[RING_BYTES];

/* Indexed by one captured byte, in units of 8-bit LSB^2. Bits 0-15: power.
 * Bit 16: clipped. */
static uint32_t s_power_luts[RF_LAYOUT_COUNT][256];

static volatile uint32_t s_retune_us;
/* Diagnostics in the status line: the slowest measurement pass in the last
 * second, and every byte the command task has received. If the first nears
 * 1000 us the command task can starve; if the second never moves, nothing is
 * arriving over USB. */
static volatile uint32_t s_loop_max_us;
static volatile uint32_t s_rx_bytes;

/* Silent until the first text command. Opening the port resets the board, and
 * a RotorHazard server probes within microseconds of flushing, so any text
 * already streaming would bury its first replies. */
static volatile bool s_streaming;

static void build_power_lut(void)
{
    for (int b = 0; b < 256; ++b) {
        int q4 = (int8_t)((b & 0x0f) << 4) >> 4;
        int i4 = (int8_t)(b & 0xf0) >> 4;
        bool clip4 = i4 == -8 || i4 == 7 || q4 == -8 || q4 == 7;
        s_power_luts[RF_LAYOUT_IQ4][b] =
            (uint32_t)(i4 * i4 + q4 * q4) * 256u | (clip4 ? 1u << 16 : 0u);

        int v = (int8_t)b;
        bool clip8 = v == -128 || v == 127;
        s_power_luts[RF_LAYOUT_Q8][b] = (uint32_t)(2 * v * v) | (clip8 ? 1u << 16 : 0u);
        s_power_luts[RF_LAYOUT_I8][b] = s_power_luts[RF_LAYOUT_Q8][b];
    }
}

/* Cyclic receive with EOF generation off and GDMA interrupts disabled, so the
 * ring is refilled forever without the driver stalling it on each wrap. */
static esp_err_t start_capture(void)
{
    parlio_rx_unit_handle_t rx;
    const parlio_rx_unit_config_t cfg = {
        .trans_queue_depth = 1u,
        .max_recv_size = sizeof(s_ring),
        .dma_burst_size = 32u,
        .data_width = RF_IQ_LANES,
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .exp_clk_freq_hz = IQ_RATE_HZ,
        .clk_in_gpio_num = -1,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .flags = { .free_clk = true },
    };
    parlio_rx_unit_config_t cfg_pins = cfg;
    memcpy(cfg_pins.data_gpio_nums, rf_iq_pins, sizeof(rf_iq_pins));
    esp_err_t err = parlio_new_rx_unit(&cfg_pins, &rx);
    if (err != ESP_OK) return err;

    parlio_rx_delimiter_handle_t delim;
    const parlio_rx_soft_delimiter_config_t delim_cfg = {
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = sizeof(s_ring),
    };
    if ((err = parlio_new_rx_soft_delimiter(&delim_cfg, &delim)) != ESP_OK) return err;
    if ((err = parlio_rx_unit_enable(rx, false)) != ESP_OK) return err;
    if ((err = parlio_rx_soft_delimiter_start_stop(rx, delim, true)) != ESP_OK) return err;

    const parlio_receive_config_t rcv = {
        .delimiter = delim,
        .flags.partial_rx_en = true,
    };
    if ((err = parlio_rx_unit_receive(rx, s_ring, sizeof(s_ring), &rcv)) != ESP_OK) return err;

    for (int ch = 0; ch < 3; ++ch) AHB_DMA.in_intr[ch].ena.val = 0;
    PARL_IO.rx_genrl_cfg.rx_eof_gen_sel = 1;

    for (int ch = 0; ch < 3; ++ch) {
        if (AHB_DMA.channel[ch].in.in_peri_sel.peri_in_sel_chn != PARLIO_PERI_ID) continue;
        uint32_t first = AHB_DMA.channel[ch].in.in_dscr_bf0.val;
        dma_descriptor_t *d = (dma_descriptor_t *)(uintptr_t)first;
        for (int n = 0; d && n < 32; ++n) {
            d->dw0.suc_eof = 0;
            d = d->next;
            if ((uintptr_t)d == first) break;
        }
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

static void out(const char *buf, size_t len)
{
    /* A RotorHazard server owns the port once it starts talking to us. */
    if (s_streaming && !rhnode_active() && usb_serial_jtag_is_connected())
        usb_serial_jtag_write_bytes(buf, len, 0);
}

/* Raw bytes for checking the bit mapping on the host, as D lines of hex. */
static void dump_samples(void)
{
    static uint8_t snap[DUMP_BYTES];
    (void)esp_cache_msync(s_ring, sizeof(s_ring), ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    memcpy(snap, s_ring, sizeof(snap));

    char line[24 + DUMP_CHUNK * 2];
    for (unsigned c = 0; c < DUMP_BYTES / DUMP_CHUNK; ++c) {
        int n = snprintf(line, sizeof(line), "D %u %u %u ", (unsigned)rf_get_layout(), c,
                         DUMP_BYTES / DUMP_CHUNK);
        for (unsigned w = 0; w < DUMP_CHUNK; ++w)
            n += snprintf(line + n, sizeof(line) - n, "%02x", snap[c * DUMP_CHUNK + w]);
        line[n++] = '\n';
        usb_serial_jtag_write_bytes(line, n, pdMS_TO_TICKS(100));
    }
}

static void print_status(void)
{
    char line[96];
    int n = snprintf(line, sizeof(line), "I %u %u %u %u %u %lu %lu %lu\n",
                     rf_get_freq(), rf_get_gain(), rf_get_bw40() ? 1u : 0u,
                     rf_get_external_antenna() ? 1u : 0u, (unsigned)rf_get_layout(),
                     (unsigned long)s_retune_us, (unsigned long)s_loop_max_us,
                     (unsigned long)s_rx_bytes);
    out(line, n);
}

/* Fading makes single readings swing by 20 dB while the drone's distance
 * barely changes, so the peak of each window tracks the envelope better than
 * any average of it. */
static void measure_task(void *arg)
{
    char batch[512];
    size_t used = 0;
    uint32_t ticks = 0;
    uint32_t peak = 0, window_n = 0, rx_peak = 0;
    uint64_t window_sum = 0, rx_sum = 0;
    uint32_t loop_max = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&wake, 1);
        int64_t loop_start = esp_timer_get_time();

        (void)esp_cache_msync(s_ring, sizeof(s_ring), ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        const uint32_t *lut = s_power_luts[rf_get_layout()];
        uint32_t sum = 0, clip = 0;
        for (uint32_t i = 0; i < RING_BYTES; i += SAMPLE_STRIDE) {
            uint32_t v = lut[s_ring[i]];
            sum += v & 0xffffu;
            clip += v >> 16;
        }
        uint32_t pwr_x100 = (uint32_t)((uint64_t)sum * 100u / SAMPLES_PER_READING);

        int rx_mv = rx5808_read_mv();

        /* RotorHazard works in 0-255. Ours is 4 counts per dB; the module's
         * matches its node firmware, which treats 2.5 V as full scale. */
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        int db4x = (int)(40.0f * log10f(fmaxf((float)pwr_x100, 1.0f) / 100.0f));
        rhnode_feed(RHNODE_C5, (uint8_t)(db4x < 0 ? 0 : (db4x > 255 ? 255 : db4x)), now_ms);
        rhnode_feed(RHNODE_RX5808, (uint8_t)(rx_mv > 2500 ? 255 : rx_mv * 255 / 2500), now_ms);

        used += snprintf(batch + used, sizeof(batch) - used, "S %llu %lu %lu %d\n",
                         (unsigned long long)esp_timer_get_time(),
                         (unsigned long)pwr_x100, (unsigned long)clip, rx_mv);
        if (pwr_x100 > peak) peak = pwr_x100;
        if ((uint32_t)rx_mv > rx_peak) rx_peak = (uint32_t)rx_mv;
        window_sum += pwr_x100;
        rx_sum += (uint32_t)rx_mv;
        if (++window_n == PEAK_WINDOW_MS) {
            used += snprintf(batch + used, sizeof(batch) - used, "P %llu %u %lu %lu %lu %lu\n",
                             (unsigned long long)esp_timer_get_time(), (unsigned)PEAK_WINDOW_MS,
                             (unsigned long)peak, (unsigned long)(window_sum / window_n),
                             (unsigned long)rx_peak, (unsigned long)(rx_sum / window_n));
            peak = 0;
            rx_peak = 0;
            window_n = 0;
            window_sum = 0;
            rx_sum = 0;
        }

        if (used > sizeof(batch) - 128) {
            out(batch, used);
            used = 0;
        }
        uint32_t loop_us = (uint32_t)(esp_timer_get_time() - loop_start);
        if (loop_us > loop_max) loop_max = loop_us;
        if (++ticks % STATUS_PERIOD_MS == 0) {
            s_loop_max_us = loop_max;
            rhnode_set_loop_us(loop_max);
            loop_max = 0;
            print_status();
        }
    }
}

static void handle_command(char *line)
{
    char cmd = line[0];
    long arg = strtol(line + 1, NULL, 10);
    esp_err_t err = ESP_OK;
    if (!s_streaming) {
        s_streaming = true;
        esp_log_level_set("*", ESP_LOG_WARN);
    }

    switch (cmd) {
    case 'f': {
        int64_t t0 = esp_timer_get_time();
        err = rf_set_freq((uint16_t)arg);
        s_retune_us = (uint32_t)(esp_timer_get_time() - t0);
        if (err == ESP_OK) rx5808_set_freq((uint16_t)arg);
        break;
    }
    case 'g':
        if (arg < 0 || arg > 62) err = ESP_ERR_INVALID_ARG;
        else rf_set_gain((uint8_t)arg);
        break;
    case 'b':
        rf_set_bw40(arg != 0);
        break;
    case 'a':
        rf_set_external_antenna(arg != 0);
        break;
    case 'm':
        if (arg < 0 || arg >= RF_LAYOUT_COUNT) err = ESP_ERR_INVALID_ARG;
        else rf_set_layout((rf_layout_t)arg);
        break;
    case 'd':
        dump_samples();
        break;
    case 's':
        break;
    default:
        err = ESP_ERR_NOT_SUPPORTED;
    }

    if (err != ESP_OK) {
        char msg[80];
        int n = snprintf(msg, sizeof(msg), "E %s: %s\n", line, esp_err_to_name(err));
        out(msg, n);
    }
    print_status();
}

static void command_task(void *arg)
{
    char line[32];
    size_t len = 0;
    for (;;) {
        char c;
        if (usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY) != 1) continue;
        s_rx_bytes++;
        if (rhnode_active() || rhnode_is_command_byte((uint8_t)c)) {
            rhnode_rx_byte((uint8_t)c);
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (len == 0) continue;
            line[len] = '\0';
            handle_command(line);
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = c;
        }
    }
}

void app_main(void)
{
    usb_serial_jtag_driver_config_t usb_cfg = {
        .tx_buffer_size = 4096,
        .rx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    usb_serial_jtag_vfs_use_driver();
    /* Startup logging can still be arriving when a RotorHazard server,
     * which reset the board by opening the port, sends its first probe. */
    esp_log_level_set("*", ESP_LOG_NONE);
    /* The peripheral holds a partial packet until something flushes it, and
     * it survives the reset that opening the port causes. Send it now, while
     * the server is still waiting out its 2 s, rather than glued to the front
     * of our first reply. */
    usb_serial_jtag_ll_txfifo_flush();

    build_power_lut();
    ESP_ERROR_CHECK(rx5808_init());
    ESP_ERROR_CHECK(rf_start());
    ESP_ERROR_CHECK(start_capture());
    ESP_ERROR_CHECK(rf_set_freq(5658));
    rx5808_set_freq(5658);
    ESP_LOGW(TAG, "capturing");

    /* Commands outrank measurement: a RotorHazard server gives up after 250 ms,
     * and this task spends nearly all its time blocked on USB anyway. */
    xTaskCreate(measure_task, "measure", 4096, NULL, 5, NULL);
    xTaskCreate(command_task, "command", 4096, NULL, 6, NULL);
}
