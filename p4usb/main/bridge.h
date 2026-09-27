#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool bridge_active(void);
/* Returns true when the bridge took the bytes and the console must not see them. */
bool bridge_host_bytes(const uint8_t *buf, size_t n);
void bridge_c5_bytes(const uint8_t *buf, size_t n);
void bridge_poll(void);
void bridge_info(void);
