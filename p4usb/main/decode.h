#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Realtime FM demodulation and NTSC decoding of the C5's I/Q into the camera's frames. */
void decode_command(int argc, char **argv);
/* Resets the C5, tunes it (NULL for the last channel, R3 at first) and decodes; the test pattern stays
 * if the C5 doesn't answer. False if the channel was refused and the last one resumed. */
bool decode_start(const char *channel);
bool decode_running(void);
/* The channel decoded, or last decoded, and its frequency (0 until the C5 first confirms it). */
const char *decode_channel(unsigned *mhz);
/* Whether the last field locked, and the channel's level in decibels above an arbitrary floor: the I/Q RMS
 * less the gain it was read at. */
bool decode_signal(int *level);
/* Tunes each channel in turn for about 12 ms, gives its level as decode_signal does, and returns to the
 * channel decoded. The picture is lost meanwhile. False if stop() cut it short or the decoder is stopped. */
bool decode_survey(const char *const *channels, int n, int8_t *levels, bool (*stop)(void));
void decode_stop(void);
