#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

/* rf_iq_pins[n] carries bit n of each captured byte. */
#define RF_IQ_LANES 8
extern const gpio_num_t rf_iq_pins[RF_IQ_LANES];

/* What each captured byte holds. PARLIO RX is limited to 8 lines, so full
 * resolution means giving up one of I or Q. */
typedef enum {
    RF_LAYOUT_IQ4 = 0, /* high nibble signed I[9:6], low nibble signed Q[9:6] */
    RF_LAYOUT_Q8 = 1,  /* signed Q[9:2] */
    RF_LAYOUT_I8 = 2,  /* signed I[9:2] */
    RF_LAYOUT_COUNT
} rf_layout_t;

void rf_set_layout(rf_layout_t layout);
rf_layout_t rf_get_layout(void);

/* Receive-only Wi-Fi PHY with MODEM_DIAG I/Q routed to GPIO. See rf.c. */
esp_err_t rf_start(void);

/* Tune to any frequency in RF_MIN_MHZ..RF_MAX_MHZ. Frequencies that are not
 * Wi-Fi channel centers go through the undocumented phy_set_freq(). */
esp_err_t rf_set_freq(uint16_t mhz);
uint16_t rf_get_freq(void);

/* Fixed receive gain index, 0..62. Higher is more sensitive. */
void rf_set_gain(uint8_t gain);
uint8_t rf_get_gain(void);

/* Waveshare ESP32-C5-Zero antenna switch: on-board chip antenna or U.FL. */
void rf_set_external_antenna(bool external);
bool rf_get_external_antenna(void);

/* Analog baseband filter: BW40 (+-20 MHz) or BW20 (+-10 MHz). */
void rf_set_bw40(bool bw40);
bool rf_get_bw40(void);
