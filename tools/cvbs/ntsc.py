"""Turn a 20 MS/s stream of C5VRX DAC codes into NTSC fields."""

from dataclasses import dataclass

import numpy as np

RATE = 20e6
LINE = RATE / (4.5e6 / 286)  # 1271.1 samples
SYNC_CODE = 0
BLANK_CODE = 20
WHITE_CODE = 62

ACTIVE_LINES = 240
# Lines 21 and 284 are each field's first full picture line, both 17 lines after the first broad pulse.
FIRST_SLOT = 17
PIXELS = 720
PIXEL_RATE = 13.5e6
ACTIVE_START_US = 8.8
BACK_PORCH_US = (5.6, 8.4)
BURST_US = (5.6, 7.6)
FSC = 315e6 / 88
BURST_IRE = 20


def us(value):
    return value * RATE / 1e6


def lowpass(signal, cutoff_hz, taps=31):
    n = np.arange(taps) - (taps - 1) / 2
    kernel = np.sinc(2 * cutoff_hz / RATE * n) * np.hamming(taps)
    return np.convolve(signal, kernel / kernel.sum(), mode="same")


@dataclass
class Pulses:
    start: np.ndarray
    width: np.ndarray


def sync_threshold(smooth):
    """Halfway between sync tip and blanking; a VTX frequency offset shifts both."""
    tip = np.percentile(smooth, 1)
    counts, edges = np.histogram(smooth, bins=np.arange(tip + 3, tip + 40, 0.5))
    blank = edges[np.argmax(counts)] + 0.25
    return (tip + blank) / 2


def find_pulses(codes):
    # 1.5 MHz removes the color burst, which otherwise dips below threshold right after sync.
    smooth = lowpass(codes.astype(np.float32), 1.5e6)
    threshold = sync_threshold(smooth)
    below = smooth < threshold
    change = np.diff(below.astype(np.int8))
    falls = np.flatnonzero(change == 1) + 1
    rises = np.flatnonzero(change == -1) + 1
    if len(falls) == 0:
        return Pulses(np.array([]), np.array([]))
    rises = rises[rises > falls[0]]
    falls = falls[: len(rises)]
    before, after = smooth[falls - 1], smooth[falls]
    start = falls - 1 + (before - threshold) / (before - after)
    return Pulses(start, (rises - falls).astype(float))


def classify(pulses):
    width = pulses.width
    equalizing = (width > us(1.5)) & (width < us(3.5))
    hsync = (width >= us(3.5)) & (width < us(7))
    broad = (width > us(20)) & (width < us(32))
    return pulses.start[equalizing], pulses.start[hsync], pulses.start[broad]


@dataclass
class Field:
    vsync: float
    parity: int
    edges: np.ndarray
    missing: int


def line_period(hsync):
    gaps = np.diff(hsync)
    gaps = gaps[(gaps > 0.9 * LINE) & (gaps < 1.1 * LINE)]
    return float(np.median(gaps)) if len(gaps) else LINE


def find_fields(codes):
    _, hsync, broad = classify(find_pulses(codes))
    period = line_period(hsync)

    groups = []
    for start in broad:
        if groups and start - groups[-1][-1] < 1.5 * period:
            groups[-1].append(start)
        else:
            groups.append([start])
    vsyncs = [group[0] for group in groups if len(group) >= 3]

    last_slot = FIRST_SLOT + ACTIVE_LINES
    fields = []
    for vsync in vsyncs:
        if vsync + (last_slot + 1) * period > len(codes):
            break
        after = hsync[(hsync > vsync + 8 * period) & (hsync < vsync + (last_slot + 1) * period)]
        if len(after) < 10:
            continue
        offsets = ((after - vsync) / period) % 1
        parity = int(np.median(np.abs(offsets - 0.5)) < 0.25)
        base = vsync + 0.5 * period * parity

        slots = FIRST_SLOT + np.arange(ACTIVE_LINES)
        slot_of = np.rint((after - base) / period)
        on_grid = np.abs(after - base - slot_of * period) < us(1)
        fit = np.polyfit(slot_of[on_grid], after[on_grid], 1)
        predicted = np.polyval(fit, slots)

        found = np.searchsorted(after, predicted)
        edges = predicted.copy()
        missing = 0
        for i, (guess, index) in enumerate(zip(predicted, found)):
            near = after[max(index - 1, 0): index + 1]
            near = near[np.abs(near - guess) < us(0.5)]
            if len(near):
                edges[i] = near[0]
            else:
                missing += 1
        fields.append(Field(vsync, parity, edges, missing))
    return fields


def chroma_baseband(signal, cutoff_hz=1.3e6):
    """Mix the subcarrier to DC with a free-running oscillator: z = (V - jU) / 2."""
    oscillator = np.exp(-2j * np.pi * FSC / RATE * np.arange(len(signal)))
    mixed = signal * oscillator
    z = lowpass(mixed.real, cutoff_hz, taps=63) + 1j * lowpass(mixed.imag, cutoff_hz, taps=63)
    return z, oscillator


def remove_chroma(signal, z, oscillator):
    return signal - 2 * np.real(z * np.conj(oscillator))


def _rows(signal, field):
    x = us(ACTIVE_START_US) + np.arange(PIXELS) * RATE / PIXEL_RATE
    return np.interp(field.edges[:, None] + x[None, :], np.arange(len(signal)), signal)


def _blank(signal, field):
    porch = np.concatenate([
        signal[int(edge + us(BACK_PORCH_US[0])): int(edge + us(BACK_PORCH_US[1]))]
        for edge in field.edges
    ])
    return float(np.median(porch))


def field_image(signal, field, white=WHITE_CODE):
    """Resample each line from its own sync edge, which corrects line-to-line timing jitter."""
    blank = _blank(signal, field)
    rows = _rows(signal, field)
    return np.clip((rows - blank) / (white - BLANK_CODE) * 255, 0, 255).astype(np.uint8), blank


def field_color(luma, z, field, white=WHITE_CODE, saturation=1.0):
    """Decode one field to RGB, using each line's color burst as the hue and saturation reference."""
    blank = _blank(luma, field)
    y = (_rows(luma, field) - blank) / (white - BLANK_CODE) * 100

    burst = np.array([
        z[int(edge + us(BURST_US[0])): int(edge + us(BURST_US[1]))].mean() for edge in field.edges
    ])
    # The subcarrier runs continuously, so neighboring lines' bursts agree in the oscillator's frame.
    burst = np.convolve(burst, np.ones(5) / 5, mode="same")
    power = np.maximum(np.abs(burst) ** 2, 1e-9)
    chroma = _rows(z.real, field) + 1j * _rows(z.imag, field)
    # Rotate the burst onto -U and scale it to 20 IRE: U + jV in IRE.
    uv = -BURST_IRE * saturation * chroma * (np.conj(burst) / power)[:, None]
    u, v = uv.real, uv.imag

    r = y + v / 0.877
    b = y + u / 0.492
    g = (y - 0.299 * r - 0.114 * b) / 0.587
    rgb = np.stack([r, g, b], axis=-1) * 2.55
    return np.clip(rgb, 0, 255).astype(np.uint8), blank


def bob(image, parity):
    """Line-double one field, placing the second field half a line lower."""
    rows = image.astype(np.float32)
    below = np.vstack([rows[1:], rows[-1:]])
    above = np.vstack([rows[:1], rows[:-1]])
    frame = np.empty((2 * len(rows),) + rows.shape[1:], np.float32)
    if parity == 0:
        frame[0::2], frame[1::2] = rows, (rows + below) / 2
    else:
        frame[0::2], frame[1::2] = (above + rows) / 2, rows
    return frame.astype(np.uint8)


def weave(first, second):
    frame = np.empty((2 * len(first),) + first.shape[1:], np.uint8)
    frame[0::2], frame[1::2] = first, second
    return frame
