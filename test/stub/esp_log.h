#pragma once
#define ESP_LOG_NONE 0
static inline void esp_log_level_set(const char *t, int l) { (void)t; (void)l; }
