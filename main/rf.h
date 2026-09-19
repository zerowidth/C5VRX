#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Receive-only Wi-Fi PHY with MODEM_DIAG I/Q routed to GPIO. See rf.c. */
esp_err_t rf_start(void);

/* Tune to any frequency in RF_MIN_MHZ..RF_MAX_MHZ. Frequencies that are not
 * Wi-Fi channel centers go through the undocumented phy_set_freq(). */
esp_err_t rf_set_freq(uint16_t mhz);
uint16_t rf_get_freq(void);

/* Fixed receive gain index, 0..62. Higher is more sensitive. */
void rf_set_gain(uint8_t gain);
uint8_t rf_get_gain(void);

/* Analog baseband filter: BW40 (+-20 MHz) or BW20 (+-10 MHz). */
void rf_set_bw40(bool bw40);
bool rf_get_bw40(void);
