#pragma once

#include <stdint.h>

#include "esp_err.h"

/* Drives a 7-bit counter on one side's lanes ('q' or 'i') with the sample clock on the
 * clock lane, advancing once per clock. */
esp_err_t link_start(char side, uint32_t clock_hz);
void link_stop(void);
