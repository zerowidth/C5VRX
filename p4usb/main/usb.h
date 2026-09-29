#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The high-speed port carries a CDC console next to the UVC camera. */
#define USB_EP_VIDEO_IN 0x83

void usb_init(void);
/* True while a program has the console open (DTR asserted). */
void usb_console_info(void);
bool usb_console_connected(void);
void usb_console_write(const void *data, size_t len);
size_t usb_console_read(uint8_t *buf, size_t len, uint32_t timeout_ms);
