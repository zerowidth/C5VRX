/**
 * main.c - RSSI meter for analog FPV video on the ESP32-C5.
 *
 * PARLIO RX streams MODEM_DIAG I/Q into a cyclic DMA ring with no CPU
 * involvement. Once per millisecond a task averages I^2 + Q^2 over the ring
 * and prints one line over USB. See viewer/ for the host side and README.md
 * for the line protocol.
 */

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
#include "soc/ahb_dma_struct.h"
#include "soc/parl_io_struct.h"

#include "rf.h"

#define IQ_RATE_HZ    40000000u
#define RING_BYTES    16384u
#define SAMPLE_STRIDE 4u
#define PARLIO_PERI_ID 9
#define STATUS_PERIOD_MS 1000u

static const char *TAG = "rssi";

static DMA_ATTR __attribute__((aligned(64))) uint8_t s_ring[RING_BYTES];

/* Low byte: I^2 + Q^2 of the signed 4-bit pair. High byte: 1 if clipped. */
static uint16_t s_power_lut[256];

static volatile uint32_t s_retune_us;

static void build_power_lut(void)
{
    for (int b = 0; b < 256; ++b) {
        int q = (int8_t)((b & 0x0f) << 4) >> 4;
        int i = (int8_t)(b & 0xf0) >> 4;
        bool clip = i == -8 || i == 7 || q == -8 || q == 7;
        s_power_lut[b] = (uint16_t)((i * i + q * q) | (clip ? 0x100 : 0));
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
        .data_width = 8u,
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .exp_clk_freq_hz = IQ_RATE_HZ,
        .clk_in_gpio_num = -1,
        .clk_out_gpio_num = -1,
        .valid_gpio_num = -1,
        .data_gpio_nums = {
            GPIO_NUM_1, GPIO_NUM_0, GPIO_NUM_25, GPIO_NUM_7,
            GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_3, GPIO_NUM_4,
        },
        .flags = { .free_clk = true },
    };
    esp_err_t err = parlio_new_rx_unit(&cfg, &rx);
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
        for (int n = 0; d && n < 16; ++n) {
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
    if (usb_serial_jtag_is_connected())
        usb_serial_jtag_write_bytes(buf, len, 0);
}

static void print_status(void)
{
    char line[64];
    int n = snprintf(line, sizeof(line), "I %u %u %u %u %lu\n",
                     rf_get_freq(), rf_get_gain(), rf_get_bw40() ? 1u : 0u,
                     rf_get_external_antenna() ? 1u : 0u, (unsigned long)s_retune_us);
    out(line, n);
}

static void measure_task(void *arg)
{
    char batch[512];
    size_t used = 0;
    uint32_t ticks = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&wake, 1);

        (void)esp_cache_msync(s_ring, sizeof(s_ring), ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        uint32_t sum = 0, clip = 0;
        for (uint32_t i = 0; i < RING_BYTES; i += SAMPLE_STRIDE) {
            uint16_t v = s_power_lut[s_ring[i]];
            sum += v & 0xffu;
            clip += v >> 8;
        }
        uint32_t pwr_x100 = (uint32_t)((uint64_t)sum * 100u / (RING_BYTES / SAMPLE_STRIDE));

        used += snprintf(batch + used, sizeof(batch) - used, "S %llu %lu %lu\n",
                         (unsigned long long)esp_timer_get_time(),
                         (unsigned long)pwr_x100, (unsigned long)clip);
        if (used > sizeof(batch) - 64) {
            out(batch, used);
            used = 0;
        }
        if (++ticks % STATUS_PERIOD_MS == 0) print_status();
    }
}

static void handle_command(char *line)
{
    char cmd = line[0];
    long arg = strtol(line + 1, NULL, 10);
    esp_err_t err = ESP_OK;

    switch (cmd) {
    case 'f': {
        int64_t t0 = esp_timer_get_time();
        err = rf_set_freq((uint16_t)arg);
        s_retune_us = (uint32_t)(esp_timer_get_time() - t0);
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
    case '?':
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

    build_power_lut();
    ESP_ERROR_CHECK(rf_start());
    ESP_ERROR_CHECK(start_capture());
    ESP_ERROR_CHECK(rf_set_freq(5658));
    ESP_LOGW(TAG, "capturing");

    xTaskCreate(measure_task, "measure", 4096, NULL, 5, NULL);
    xTaskCreate(command_task, "command", 4096, NULL, 4, NULL);
}
