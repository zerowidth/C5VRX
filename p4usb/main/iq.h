#pragma once

#include <stdbool.h>

#include "driver/parlio_rx.h"

void iq_command(int argc, char **argv);
/* Resets the C5 into c5rx, tunes it, starts its I/Q export with one of every `every` samples of its
 * 80 MS/s bus, and picks the P4's sampling edge. */
bool iq_start(const char *channel, int every);
/* The C5's name and frequency for the channel last tuned. */
const char *iq_channel(unsigned *mhz);
parlio_sample_edge_t iq_edge(void);
/* A PARLIO RX unit on the C5's clock and 14 lanes: I in the low byte and Q in the high byte of each
 * 16-bit word, MSB-aligned, with lines 0 and 8 held at 1. */
esp_err_t iq_new_rx(size_t max_recv_size, parlio_rx_unit_handle_t *ret);
/* The 1024-entry table bs_phase.bsasm looks phases up in, around an I/Q offset. Only its first 512
 * entries depend on the offset. */
void iq_bs_build_lut(uint16_t *lut, float dc_i, float dc_q);
/* The magnitude, in the lanes' 2v+1 units, at the middle of one of the table's 32 magnitude codes. */
float iq_bs_code_centre(int code);
