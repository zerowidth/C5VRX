#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"

void console_init(void);
void host_write(const void *data, size_t len);
void say(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void console_feed(uint8_t c);
size_t uart_read_some(uart_port_t port, uint8_t *buf, size_t len, uint32_t timeout_ms);
size_t host_read(uint8_t *buf, size_t len, uint32_t timeout_ms);
