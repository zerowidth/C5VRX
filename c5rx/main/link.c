#include "link.h"

#include "driver/parlio_tx.h"
#include "esp_heap_caps.h"

#include "lanes.h"

#define PATTERN_BYTES 512

static parlio_tx_unit_handle_t s_tx;
static uint8_t *s_pattern;

esp_err_t link_start(char side, uint32_t clock_hz)
{
    link_stop();
    if (side != 'q' && side != 'i') return ESP_ERR_INVALID_ARG;
    if (!s_pattern) {
        s_pattern = heap_caps_malloc(PATTERN_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_pattern) return ESP_ERR_NO_MEM;
        for (int k = 0; k < PATTERN_BYTES; ++k) s_pattern[k] = k & 0x7f;
    }

    /* Data line k carries counter bit k, on lane Q(k+1) or I(k+1). */
    int first = side == 'q' ? LANE_Q1 : LANE_I1;
    parlio_tx_unit_config_t cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = -1,
        .output_clk_freq_hz = clock_hz,
        .data_width = 8,
        .clk_out_gpio_num = LANES[LANE_CLK].gpio,
        .valid_gpio_num = -1,
        .trans_queue_depth = 1,
        .max_transfer_size = PATTERN_BYTES,
        .shift_edge = PARLIO_SHIFT_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    for (int k = 0; k < 7; ++k) cfg.data_gpio_nums[k] = LANES[first - k].gpio;
    cfg.data_gpio_nums[7] = -1;

    esp_err_t err = parlio_new_tx_unit(&cfg, &s_tx);
    if (err != ESP_OK) return err;
    err = parlio_tx_unit_enable(s_tx);
    if (err != ESP_OK) {
        link_stop();
        return err;
    }
    const parlio_transmit_config_t tx = {.flags.loop_transmission = 1};
    err = parlio_tx_unit_transmit(s_tx, s_pattern, PATTERN_BYTES * 8, &tx);
    if (err != ESP_OK) link_stop();
    return err;
}

void link_stop(void)
{
    if (!s_tx) return;
    parlio_tx_unit_disable(s_tx);
    parlio_del_tx_unit(s_tx);
    s_tx = NULL;
    lanes_drive_low();
}
