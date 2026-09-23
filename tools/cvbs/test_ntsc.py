import numpy as np

import ntsc
import synth


def ramp(frame, field, row):
    return np.linspace(0, 1, ntsc.PIXELS) * (row % 2 == field)


def test_finds_alternating_fields_and_recovers_picture():
    codes = synth.frames(3, ramp)
    rng = np.random.default_rng(2)
    noisy = np.clip(codes + rng.integers(-2, 3, len(codes)), 0, 63).astype(np.uint8)

    fields = ntsc.find_fields(noisy)
    assert [f.parity for f in fields] == [0, 1, 0, 1, 0, 1]
    assert all(f.missing == 0 for f in fields)

    signal = ntsc.lowpass(noisy.astype(np.float32), 3e6)
    for field in fields:
        image, blank = ntsc.field_image(signal, field)
        assert abs(blank - ntsc.BLANK_CODE) < 1
        lit = image[field.parity::2, 100:620]
        dark = image[1 - field.parity::2, 100:620]
        expected = np.linspace(0, 255, ntsc.PIXELS)[100:620]
        assert np.abs(lit.mean(axis=0) - expected).max() < 8
        assert dark.mean() < 4


def test_tolerates_a_lost_sync_pulse():
    codes = synth.frames(2, ramp)
    fields = ntsc.find_fields(codes)
    edge = int(fields[0].edges[50])
    codes[edge: edge + int(ntsc.us(4.7))] = ntsc.BLANK_CODE

    damaged = ntsc.find_fields(codes)
    assert damaged[0].missing == 1
    assert np.abs(damaged[0].edges - fields[0].edges).max() < 1


def test_finds_syncs_when_a_frequency_offset_lowers_blanking():
    codes = synth.frames(2, ramp).astype(np.int16)
    # Measured on a live capture: sync near 1, blanking near 11.
    shifted = np.clip(np.rint(codes * 0.55) + 1, 0, 63).astype(np.uint8)
    fields = ntsc.find_fields(shifted)
    assert [f.parity for f in fields] == [0, 1, 0, 1]
    assert all(f.missing == 0 for f in fields)
