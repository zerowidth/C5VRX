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


def frames(count, picture, color=None):
    """`picture(frame, field, row)` returns 0..1 levels for the 720 active pixels of a field row.

    `color`, if given, returns (U, V) in IRE for the same pixels, and every line gets a burst.
    """
    total = int(round(count * 1050 * HALF))
    level = np.full(total, float(ntsc.BLANK_CODE))
    per_ire = (ntsc.WHITE_CODE - ntsc.BLANK_CODE) / 100
    phase = 2 * np.pi * ntsc.FSC / ntsc.RATE
    x = ntsc.us(ntsc.ACTIVE_START_US) + np.arange(ntsc.PIXELS) * ntsc.RATE / ntsc.PIXEL_RATE
    for frame in range(count):
        for h in range(1050):
            start = (frame * 1050 + h) * HALF
            width = half_line_pulse(h)
            if width:
                level[int(round(start)): int(round(start + width))] = ntsc.SYNC_CODE
            if h % 2 or width != HSYNC:
                continue
            if color:
                burst = np.arange(int(start + ntsc.us(5.3)), int(start + ntsc.us(7.8)))
                level[burst] -= ntsc.BURST_IRE * per_ire * np.sin(phase * burst)
            line = h // 2 + 1
            field, first = (0, 21) if line <= 263 else (1, 284)
            row = line - first
            if 0 <= row < ntsc.ACTIVE_LINES:
                span = np.arange(int(start + x[0]), int(start + x[-1]))
                luma = picture(frame, field, row) * 100
                level[span] = ntsc.BLANK_CODE + np.interp(span - start, x, luma) * per_ire
                if color:
                    u, v = color(frame, field, row)
                    chroma = (np.interp(span - start, x, u) * np.sin(phase * span)
                              + np.interp(span - start, x, v) * np.cos(phase * span))
                    level[span] += chroma * per_ire
    return np.clip(np.rint(level), 0, 63).astype(np.uint8)
