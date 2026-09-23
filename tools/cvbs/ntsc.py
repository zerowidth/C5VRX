"""Turn a 20 MS/s stream of C5VRX DAC codes into NTSC fields."""

from dataclasses import dataclass

import numpy as np

RATE = 20e6
LINE = RATE / (4.5e6 / 286)  # 1271.1 samples
SYNC_CODE = 0
BLANK_CODE = 20
WHITE_CODE = 62
SYNC_THRESHOLD = 10

ACTIVE_LINES = 240
# Field-local line 21 is the first full picture line; slots count lines from the first broad pulse.
FIRST_SLOT = {0: 17, 1: 16}
PIXELS = 720
PIXEL_RATE = 13.5e6
ACTIVE_START_US = 8.8
BACK_PORCH_US = (5.6, 8.4)


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


def find_pulses(codes):
    smooth = np.convolve(codes.astype(np.float32), np.ones(5) / 5, mode="same")
    below = smooth < SYNC_THRESHOLD
    change = np.diff(below.astype(np.int8))
    falls = np.flatnonzero(change == 1) + 1
    rises = np.flatnonzero(change == -1) + 1
    if len(falls) == 0:
        return Pulses(np.array([]), np.array([]))
    rises = rises[rises > falls[0]]
    falls = falls[: len(rises)]
    # Interpolate the threshold crossing between the last sample above and the first below.
    before, after = smooth[falls - 1], smooth[falls]
    start = falls - 1 + (before - SYNC_THRESHOLD) / (before - after)
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

    last_slot = max(FIRST_SLOT.values()) + ACTIVE_LINES
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

        slots = FIRST_SLOT[parity] + np.arange(ACTIVE_LINES)
        slot_of = np.rint((after - base) / period)
        on_grid = np.abs(after - base - slot_of * period) < us(1)
        fit = np.polyfit(slot_of[on_grid], after[on_grid], 1)
        predicted = np.polyval(fit, slots)

        found = np.searchsorted(after, predicted)
        edges = predicted.copy()
        for i, (guess, index) in enumerate(zip(predicted, found)):
            near = after[max(index - 1, 0): index + 1]
            near = near[np.abs(near - guess) < us(0.5)]
            if len(near):
                edges[i] = near[0]
        fields.append(Field(vsync, parity, edges, int(np.sum(edges == predicted))))
    return fields


def field_image(signal, field, white=WHITE_CODE):
    """Resample each line from its own sync edge, which corrects line-to-line timing jitter."""
    porch = np.concatenate([
        signal[int(edge + us(BACK_PORCH_US[0])): int(edge + us(BACK_PORCH_US[1]))]
        for edge in field.edges
    ])
    blank = float(np.median(porch))
    x = us(ACTIVE_START_US) + np.arange(PIXELS) * RATE / PIXEL_RATE
    rows = np.interp(field.edges[:, None] + x[None, :], np.arange(len(signal)), signal)
    return np.clip((rows - blank) / (white - BLANK_CODE) * 255, 0, 255).astype(np.uint8), blank


def bob(image, parity):
    """Line-double one field, placing the second field half a line lower."""
    rows = image.astype(np.float32)
    below = np.vstack([rows[1:], rows[-1:]])
    above = np.vstack([rows[:1], rows[:-1]])
    frame = np.empty((2 * len(rows), rows.shape[1]), np.float32)
    if parity == 0:
        frame[0::2], frame[1::2] = rows, (rows + below) / 2
    else:
        frame[0::2], frame[1::2] = (above + rows) / 2, rows
    return frame.astype(np.uint8)


def weave(first, second):
    frame = np.empty((2 * len(first), first.shape[1]), np.uint8)
    frame[0::2], frame[1::2] = first, second
    return frame
