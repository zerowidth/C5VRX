import numpy as np

import saleae


def simulate_capture(codes, ppm, analyzer_rate, start, tmp_path):
    """Write the export a Logic 2 capture of `codes` would produce, quantized to analyzer samples."""
    period = 1 / (saleae.SYMBOL_RATE_HZ * (1 + ppm * 1e-6))
    step = 1 / analyzer_rate
    end = start + len(codes) * period
    for bit in range(saleae.DAC_BITS):
        levels = (codes >> bit) & 1
        flips = np.flatnonzero(np.diff(levels)) + 1
        times = np.ceil((start + flips * period) / step) * step
        saleae.write_channel(
            tmp_path / f"digital_{bit}.bin",
            saleae.Channel(int(levels[0]), start, end, times),
        )


def contains(haystack, needle):
    windows = np.lib.stride_tricks.sliding_window_view(haystack, len(needle))
    return bool((windows == needle).all(axis=1).any())


def test_recovers_codes_across_clock_offsets(tmp_path):
    rng = np.random.default_rng(1)
    codes = rng.integers(0, 64, 200_000, dtype=np.uint8)
    for ppm, rate in [(0, 50e6), (-80, 50e6), (60, 100e6)]:
        simulate_capture(codes, ppm, rate, start=3.7e-9, tmp_path=tmp_path)
        recovered, grid = saleae.recover_codes(saleae.read_bus(tmp_path))
        assert abs(grid.ppm - ppm) < 1
        assert contains(recovered, codes[2:-2])
