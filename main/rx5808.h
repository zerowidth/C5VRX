#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* An RX5808 receiver module wired alongside the C5's own receiver, as a
 * known-good reference: it reports signal strength as an analog voltage.
 * Pins: RSSI on GP6 (ADC), DATA GP8, LE GP9, CLK GP10. */
esp_err_t rx5808_init(void);

/* Tune the module. Its synthesizer steps in 2 MHz, so the nearest step is
 * used; FPV channels are all even MHz. */
void rx5808_set_freq(uint16_t mhz);

uint16_t rx5808_get_freq(void);

/* Latest RSSI in millivolts. Reads about 0 when no module is connected. */
int rx5808_read_mv(void);
