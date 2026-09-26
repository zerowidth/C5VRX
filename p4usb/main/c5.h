#pragma once

#include <stdbool.h>

#include "driver/uart.h"

#define C5_UART UART_NUM_1

void c5_init(void);
void c5_hold(void);
void c5_run(void);
void c5_download(void);
void c5_command(int argc, char **argv);
