#!/usr/bin/env python3
"""Back-porch noise of candidate phase tables, in kHz, against an exact demodulator.

A constant-envelope carrier at the back porch's frequency, with the C5's I offset and noise set so the
exact demodulator matches the measured 165 kHz, quantized to the 7+7 lanes the P4 reads. Needs no packages.
"""
import math
import random

FS = 13.333e6
N = 120_000
DC_I = 20.0
NOISE = 0.055
PORCH_HZ = -1.0e6
TAU = 2 * math.pi


def signal(rms):
    out = []
    for n in range(N):
        ph = TAU * PORCH_HZ / FS * n + 0.3
        x = rms * (math.cos(ph) + NOISE * random.gauss(0, 1)) + DC_I
        y = rms * (math.sin(ph) + NOISE * random.gauss(0, 1))
        out.append((max(-64, min(63, math.floor(x / 2))), max(-64, min(63, math.floor(y / 2)))))
    return out


def khz(turns):
    d = []
    for a, b in zip(turns, turns[1:]):
        x = b - a
        d.append(x - round(x))
    mean = sum(d) / len(d)
    return math.sqrt(sum((x - mean) ** 2 for x in d) / len(d)) * FS / 1e3


def exact(i, q):
    return math.atan2(2 * q + 1, 2 * i + 1 - DC_I) / TAU


def trunc(v, bits):
    """The top bits of a 7-bit lane value, as the midpoint of what was dropped, in 2v+1 units."""
    s = 7 - bits
    return ((v >> s) << s) * 2 + (1 << s)


def direct(bi, bq):
    """One table indexed by the top bits of I and Q, 8-bit phase out."""
    return lambda i, q: round(math.atan2(trunc(q, bq), trunc(i, bi) - DC_I) / TAU * 256) / 256


def bipartite():
    """A coarse 5+5 table plus a correction indexed by the top 3 and low 2 bits of each."""
    coarse = direct(5, 5)
    num, den = {}, {}
    for i in range(-64, 64):
        for q in range(-64, 64):
            res = (exact(i, q) - coarse(i, q)) * 256
            res -= 256 * round(res / 256)
            # Fit the correction where the gain control keeps the amplitude.
            r = math.hypot(2 * i + 1 - DC_I, 2 * q + 1)
            w = math.exp(-0.5 * ((r - 55) / 25) ** 2)
            k = (i >> 4, q >> 4, i & 3, q & 3)
            num[k] = num.get(k, 0) + res * w
            den[k] = den.get(k, 0) + w
    fine = {k: round(num[k] / den[k]) for k in num}
    return lambda i, q: coarse(i, q) + fine[(i >> 4, q >> 4, i & 3, q & 3)] / 256


def companded(levels=32, knee=6, angle_levels=64):
    """Per-lane tables give a sign and a log-spaced magnitude code around the offset; a pair table maps the
    two codes to a first-quadrant angle, which the CPU unfolds with the signs."""
    scale = (levels - 1e-6) / math.log2(1 + (128 + abs(DC_I)) / knee)

    def code(m):
        return min(levels - 1, int(scale * math.log2(1 + m / knee)))

    cells = {}
    for dc in (DC_I, 0.0):
        for v in range(-64, 64):
            m = abs(2 * v + 1 - dc)
            cells.setdefault(code(m), []).append(m)
    centre = {c: sum(ms) / len(ms) for c, ms in cells.items()}

    def phase(i, q):
        a, b = 2 * i + 1 - DC_I, 2 * q + 1
        t = round(math.atan2(centre[code(abs(b))], centre[code(abs(a))]) / (TAU / 4) * angle_levels)
        t = min(t, angle_levels - 1) / angle_levels / 4
        if a < 0:
            t = 0.5 - t
        return t if b >= 0 else -t

    return phase


TABLES = [
    ("exact", exact),
    ("6+6", direct(6, 6)),
    ("6+7", direct(6, 7)),
    ("6+5", direct(6, 5)),
    ("bipartite", bipartite()),
    ("companded", companded()),
]

if __name__ == "__main__":
    random.seed(1)
    print(f"{'rms':>4} " + " ".join(f"{name:>9}" for name, _ in TABLES))
    for rms in (20, 30, 40, 60, 72, 90):
        s = signal(rms)
        print(f"{rms:4d} " + " ".join(f"{khz([f(i, q) for i, q in s]):9.0f}" for _, f in TABLES))
