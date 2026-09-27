#pragma once

#include <stdbool.h>

/* Realtime FM demodulation and NTSC decoding of the C5's I/Q into the camera's frames. */
void decode_command(int argc, char **argv);
bool decode_running(void);
void decode_stop(void);
