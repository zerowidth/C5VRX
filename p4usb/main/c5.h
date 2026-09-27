#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"

#define C5_UART UART_NUM_1

void c5_init(void);
void c5_hold(void);
void c5_run(void);
void c5_download(void);
void c5_command(int argc, char **argv);
/* TX to the C5's GPIO 12, attached only while the ROM or c5rx is known to own that pin. */
void c5_tx_attach(void);
void c5_tx_detach(void);
/* Resets the C5 and waits for c5rx to announce itself. */
bool c5_run_c5rx(uint32_t timeout_ms);
/* Sends one command line to c5rx and waits for its "ok ..." or "err ..." reply. */
bool c5_request(const char *cmd, char *reply, size_t reply_len, uint32_t timeout_ms);
