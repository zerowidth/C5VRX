"""Synthesize the DAC code stream for NTSC frames, for testing the decoder without hardware."""

import numpy as np

import ntsc

HALF = ntsc.LINE / 2
EQUALIZING = ntsc.us(2.3)
HSYNC = ntsc.us(4.7)
BROAD = HALF - HSYNC


def half_line_pulse(h):
    """Sync pulse width starting at half-line h (0..1049) of a 525-line frame, or 0 for none."""
    if h < 6 or 12 <= h < 18 or 525 <= h < 531 or 537 <= h < 543:
        return EQUALIZING
    if 6 <= h < 12 or 531 <= h < 537:
        return BROAD
    return HSYNC if h % 2 == 0 else 0


def frames(count, picture):
    """`picture(frame, field, row)` returns 0..1 levels for the 720 active pixels of a field row."""
    total = int(round(count * 1050 * HALF))
    codes = np.full(total, ntsc.BLANK_CODE, np.uint8)
    x = ntsc.us(ntsc.ACTIVE_START_US) + np.arange(ntsc.PIXELS) * ntsc.RATE / ntsc.PIXEL_RATE
    for frame in range(count):
        for h in range(1050):
            start = (frame * 1050 + h) * HALF
            width = half_line_pulse(h)
            if width:
                codes[int(round(start)): int(round(start + width))] = ntsc.SYNC_CODE
            if h % 2 or width != HSYNC:
                continue
            line = h // 2 + 1
            field, first = (0, 21) if line <= 263 else (1, 283)
            row = line - first
            if 0 <= row < ntsc.ACTIVE_LINES:
                level = picture(frame, field, row)
                pixels = ntsc.BLANK_CODE + level * (ntsc.WHITE_CODE - ntsc.BLANK_CODE)
                span = np.arange(int(start + x[0]), int(start + x[-1]))
                codes[span] = np.rint(np.interp(span - start, x, pixels)).astype(np.uint8)
    return codes
