#pragma once

#include <stdint.h>

/* Draws into an 8-bit grayscale frame of the given width. */
void overlay_text(uint8_t *px, int width, int x, int y, int scale, const char *s);
void overlay_box(uint8_t *px, int width, int x, int y, int w, int h, uint8_t level);
