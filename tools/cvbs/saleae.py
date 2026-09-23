"""Recover the C5VRX DAC codes from a Saleae Logic 2 binary export of the DAC pins."""

import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# PARLIO TX clocks at 40 MHz and writes each 6-bit code twice.
SYMBOL_RATE_HZ = 20e6
DAC_BITS = 6

_HEADER = struct.Struct("<8siiIddQ")


@dataclass
class Channel:
    initial_state: int
    begin_time: float
    end_time: float
    transitions: np.ndarray


def read_channel(path):
    data = Path(path).read_bytes()
    ident, version, kind, initial, begin, end, count = _HEADER.unpack_from(data)
    if ident != b"<SALEAE>":
        raise ValueError(f"{path}: not a Saleae binary export")
    if version != 0 or kind != 0:
        raise ValueError(f"{path}: unsupported format version {version}, type {kind}")
    transitions = np.frombuffer(data, "<f8", count=count, offset=_HEADER.size)
    return Channel(initial, begin, end, transitions)


def write_channel(path, channel):
    header = _HEADER.pack(
        b"<SALEAE>", 0, 0, channel.initial_state,
        channel.begin_time, channel.end_time, len(channel.transitions),
    )
    Path(path).write_bytes(header + np.asarray(channel.transitions, "<f8").tobytes())


def read_bus(directory, bits=DAC_BITS):
    """Channel N of the analyzer must be DAC bit N (GPIO 6, 8, 9, 10, 12, 11)."""
    return [read_channel(Path(directory) / f"digital_{bit}.bin") for bit in range(bits)]


@dataclass
class Grid:
    first_center: float
    period: float
    count: int
    residual_ns: float
    offsets_ns: list = None

    @property
    def ppm(self):
        return (1 / self.period / SYMBOL_RATE_HZ - 1) * 1e6

    def centers(self):
        return self.first_center + np.arange(self.count) * self.period


def symbol_grid(bus, block=50e-6):
    """Fit the C5's symbol clock from the edge times, in the analyzer's timebase.

    Edges only occur on the 50 ns symbol grid, so the phase of each block's edges
    against the nominal period drifts linearly with the clock offset between the two boards.
    """
    edges = np.sort(np.concatenate([c.transitions for c in bus]))
    begin = max(c.begin_time for c in bus)
    end = min(c.end_time for c in bus)
    edges = edges[(edges >= begin) & (edges < end)]
    if len(edges) < 100:
        raise ValueError("too few transitions to recover the symbol clock")

    nominal = 1 / SYMBOL_RATE_HZ
    blocks = int((end - begin) // block)
    index = ((edges - begin) // block).astype(np.int64)
    keep = index < blocks
    index, edges = index[keep], edges[keep]
    phasor = np.exp(2j * np.pi * (edges - begin) / nominal)
    sums = (np.bincount(index, phasor.real, blocks) + 1j * np.bincount(index, phasor.imag, blocks))
    counts = np.bincount(index, minlength=blocks)
    valid = (counts >= 8) & (np.abs(sums) > 0.5 * np.maximum(counts, 1))
    if valid.sum() < 2:
        raise ValueError("edges do not line up on a 50 ns grid; check wiring and sample rate")

    t = begin + (np.flatnonzero(valid) + 0.5) * block
    phase = np.unwrap(np.angle(sums[valid]))
    slope, intercept = np.polyfit(t - begin, phase, 1)
    residual = phase - (slope * (t - begin) + intercept)

    frequency = SYMBOL_RATE_HZ - slope / (2 * np.pi)
    period = 1 / frequency
    # The fitted phase marks where edges land; sample half a symbol later.
    edge_offset = (intercept / (2 * np.pi) % 1) * nominal
    first_center = begin + edge_offset + period / 2
    if first_center - period >= begin:
        first_center -= period
    count = int((end - first_center) // period)
    return Grid(first_center, period, count, float(np.std(residual) / (2 * np.pi) * period * 1e9))


def edge_lag(channel, grid):
    """How far this channel's edges trail the symbol grid, as a sampling offset.

    Pins switch at different speeds (GPIO11 lags the others by about 12 ns
    on the Waveshare board), so one shared sampling point can land mid-edge.
    """
    edge = grid.first_center - grid.period / 2
    angle = np.angle(np.mean(np.exp(2j * np.pi * (channel.transitions - edge) / grid.period)))
    return angle / (2 * np.pi) * grid.period


def sample_bus(bus, times, offsets=None):
    offsets = offsets if offsets is not None else [0.0] * len(bus)
    codes = np.zeros(len(times), np.uint8)
    for bit, (channel, offset) in enumerate(zip(bus, offsets)):
        flips = np.searchsorted(channel.transitions, times + offset, side="right")
        codes |= (((channel.initial_state + flips) & 1) << bit).astype(np.uint8)
    return codes


def recover_codes(bus):
    grid = symbol_grid(bus)
    grid.offsets_ns = [edge_lag(channel, grid) * 1e9 for channel in bus]
    offsets = [ns * 1e-9 for ns in grid.offsets_ns]
    return sample_bus(bus, grid.centers(), offsets), grid
