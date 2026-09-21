#pragma once
#include <stddef.h>
#include <stdint.h>
extern unsigned char tx_buf[512];
extern int tx_len;
static inline int usb_serial_jtag_write_bytes(const void *src, size_t size, uint32_t t) {
    for (size_t i = 0; i < size; i++) tx_buf[tx_len++] = ((const unsigned char *)src)[i];
    return (int)size;
}
