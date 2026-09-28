#include "overlay.h"

#include <string.h>

#include "menu_font.h"

static void glyph(uint8_t *px, int width, int x, int y, int halves, char c)
{
    if (c < 32 || c > 126) c = '?';
    const uint8_t *rows = s_font8x8[c - 32];
    for (int r = 0; r < 8; ++r) {
        for (int b = 0; b < 8; ++b) {
            if (!(rows[r] & (0x80 >> b))) continue;
            int x0 = b * halves / 2, y0 = r * halves / 2;
            overlay_box(px, width, x + x0, y + y0, (b + 1) * halves / 2 - x0, (r + 1) * halves / 2 - y0, 0xff);
        }
    }
}

void overlay_text(uint8_t *px, int width, int x, int y, int halves, const char *s)
{
    for (; *s; ++s, x += 4 * halves) glyph(px, width, x, y, halves, *s);
}

void overlay_box(uint8_t *px, int width, int x, int y, int w, int h, uint8_t level)
{
    for (int r = 0; r < h; ++r) memset(px + (size_t)(y + r) * width + x, level, w);
}
