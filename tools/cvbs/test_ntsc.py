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


def test_decodes_hue_and_saturation_against_the_burst():
    orange = (-15.0, 25.0)  # U, V in IRE
    def flat(frame, field, row):
        return np.full(ntsc.PIXELS, 0.5)
    def color(frame, field, row):
        return np.full(ntsc.PIXELS, orange[0]), np.full(ntsc.PIXELS, orange[1])
    codes = synth.frames(2, flat, color)
    rng = np.random.default_rng(4)
    noisy = np.clip(codes + rng.integers(-1, 2, len(codes)), 0, 63).astype(np.float32)

    z = ntsc.chroma_baseband(noisy)
    wide = ntsc.lowpass(noisy, 4.2e6)
    for field in ntsc.find_fields(noisy.astype(np.uint8)):
        rgb, _ = ntsc.field_color(wide, z, field)
        r, g, b = rgb[20:220, 100:620].reshape(-1, 3).mean(axis=0)
        y = 50
        expected = (y + orange[1] / 0.877, None, y + orange[0] / 0.492)
        assert abs(r / 2.55 - expected[0]) < 5
        assert abs(b / 2.55 - expected[2]) < 5
        assert r > g > b


def test_comb_keeps_fine_luma_detail_out_of_the_color():
    # Vertical stripes near the subcarrier frequency, with no color at all.
    x = np.arange(ntsc.PIXELS) / ntsc.PIXEL_RATE
    stripes = 0.5 + 0.3 * np.sin(2 * np.pi * 3.4e6 * x)
    def picture(frame, field, row):
        return stripes
    def no_color(frame, field, row):
        return np.zeros(ntsc.PIXELS), np.zeros(ntsc.PIXELS)
    codes = synth.frames(1, picture, no_color).astype(np.float32)
    z = ntsc.chroma_baseband(codes)
    wide = ntsc.lowpass(codes, 4.2e6)
    field = ntsc.find_fields(codes.astype(np.uint8))[0]

    def false_color(comb):
        rgb, _ = ntsc.field_color(wide, z, field, comb=comb)
        rgb = rgb[20:220, 100:620].astype(float)
        return np.abs(rgb - rgb.mean(axis=-1, keepdims=True)).mean()

    assert false_color(comb=True) < 0.2 * false_color(comb=False)
