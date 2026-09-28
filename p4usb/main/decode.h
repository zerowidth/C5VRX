#pragma once

#include <stdbool.h>

/* Realtime FM demodulation and NTSC decoding of the C5's I/Q into the camera's frames. */
void decode_command(int argc, char **argv);
/* Resets the C5, tunes it (NULL for the last channel, R3 at first) and decodes; the test pattern stays
 * if the C5 doesn't answer. */
void decode_start(const char *channel);
bool decode_running(void);
void decode_stop(void);
