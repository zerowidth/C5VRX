#pragma once

#include <stdint.h>

void uvc_init(void);
void uvc_counts(uint32_t *sent, uint32_t *skipped);
void uvc_info(void);
