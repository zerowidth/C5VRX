#!/usr/bin/env python3
"""Demodulates one raw I/Q capture with the CPU's phase table and with the BitScrambler's, to compare them
on the same samples.

    table_compare.py capture PORT out.bin    let the decoder set the gain, then capture 105 ms of raw I/Q
    table_compare.py compare in.bin [out.png]

compare prints each table's error against an exact demodulator, by signal amplitude, and with out.png
draws the exact, CPU and BitScrambler pictures side by side. `iq bs check` has shown the BitScrambler's
output matches this model of its table sample for sample. Needs pyserial to capture, nothing to compare.
"""
import math
import re
import struct
import sys
import time
import zlib

FS = 80e6 / 6
LINE = FS / 15734.264
TAU = 2 * math.pi
# A PSRAM capture's first 64,512 bytes hold the capture before it.
STALE = 32256

BS_CODES, BS_KNEE, BS_MAG_MAX = 32, 6.0, 160.0
BS_SCALE = BS_CODES / math.log2(1 + BS_MAG_MAX / BS_KNEE)


def capture(port, path):
    import serial

    s = serial.Serial()
    s.port, s.timeout, s.rts, s.dtr = port, 0.3, False, True
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()

    def cmd(c, wait):
        s.write((c + "\n").encode())
        end, buf = time.time() + wait, b""
        while time.time() < end:
            d = s.read(65536)
            if d:
                buf += d
                end = max(end, time.time() + 0.5)
        return buf

    status = cmd("decode", 1.5).decode(errors="replace")
    if "running" not in status:
        cmd("decode on", 8)
        status = cmd("decode", 1.5).decode(errors="replace")
    gain = re.search(r"gain (\d+)", status)
    print(cmd("iq 1400000", 1.5).decode(errors="replace").replace("\r", "").strip().split("\n")[1])
    b = cmd("iq dump", 4)
    m = re.search(rb"-----BEGIN IQ (\d+)-----\n", b)
    data = b[m.end() : m.end() + int(m.group(1))]
    open(path, "wb").write(data)
    print(f"{len(data) // 2} samples at gain {gain.group(1) if gain else '?'} to {path}")
    # The decoder was stopped for the capture.
    cmd("decode on", 6)


def load(path):
    d = open(path, "rb").read()
    w = struct.unpack(f"<{len(d) // 2}H", d)[STALE:]
    return [((x & 0xFF) ^ 0x80) - 0x80 for x in w], [(x >> 8 ^ 0x80) - 0x80 for x in w]


def cpu_phase(dc_i, dc_q):
    """decode.c's build_lut: the top 6 bits of I as the midpoint of the dropped one, and all 7 of Q."""
    dc_i, dc_q = round(dc_i * 4) / 4, round(dc_q * 4) / 4
    table = {}

    def phase(i, q):
        k = (i >> 2, q)
        if k not in table:
            table[k] = round(math.atan2(q - dc_q, (i >> 2) * 4 + 2 - dc_i) / TAU * 256) & 255
        return table[k]

    return phase


def bs_phase(dc_i, dc_q):
    """iq_bs_build_lut and bs_phase.bsasm: a log magnitude code a lane, and the pair's angle on from the
    start of its quadrant."""

    def centre(c):
        return BS_KNEE * ((2 ** (c / BS_SCALE) - 1) + (2 ** ((c + 1) / BS_SCALE) - 1)) / 2

    pair = [[min(63, round(math.atan2(centre(y), centre(x)) / (TAU / 4) * 64)) for y in range(32)] for x in range(32)]

    def code(v):
        return min(BS_CODES - 1, int(BS_SCALE * math.log2(1 + abs(v) / BS_KNEE)))

    lane_i = {v: (code(v - dc_i), v - dc_i < 0) for v in range(-128, 128)}
    lane_q = {v: (code(v - dc_q), v - dc_q < 0) for v in range(-128, 128)}

    def phase(i, q):
        (ci, ni), (cq, nq) = lane_i[i], lane_q[q]
        odd = ni != nq
        return nq << 7 | odd << 6 | (pair[cq][ci] if odd else pair[ci][cq])

    return phase


def demod(phases):
    """Differences in units of FS / 256."""
    return [((b - a + 128) & 255) - 128 for a, b in zip(phases, phases[1:])]


def write_png(path, rows, width):
    raw = b"".join(b"\0" + bytes(r) for r in rows)

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    head = struct.pack(">IIBBBBB", width, len(rows), 8, 0, 0, 0, 0)
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", head) + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def compare(path, png):
    i, q = load(path)
    n = len(i)
    dc_i, dc_q = sum(i) / n, sum(q) / n
    exact = [math.atan2(b - dc_q, a - dc_i) / TAU * 256 for a, b in zip(i, q)]
    d_exact = [((b - a + 128) % 256) - 128 for a, b in zip(exact, exact[1:])]
    radius = [math.hypot(a - dc_i, b - dc_q) for a, b in zip(i, q)]
    print(f"{n} samples, I/Q offset {dc_i:.2f} {dc_q:.2f}, RMS {math.sqrt(sum(r * r for r in radius) / n):.0f}")

    cases = [
        ("CPU", cpu_phase(dc_i, dc_q)),
        ("BS", bs_phase(dc_i, dc_q)),
        ("BS, offset 1 out", bs_phase(dc_i + 1, dc_q)),
        ("BS, offset 3 out", bs_phase(dc_i + 3, dc_q)),
    ]
    demods = [(name, demod([f(a, b) for a, b in zip(i, q)])) for name, f in cases]

    # Each table's error against the exact demodulator, over stretches of 64 samples sorted by amplitude.
    bins = [(0, 8), (8, 16), (16, 32), (32, 64), (64, 999)]
    print(f"{'amplitude':>10} {'share':>6} " + " ".join(f"{name:>17}" for name, _ in demods) + "   error in kHz rms")
    block = 64
    amp = [math.sqrt(sum(r * r for r in radius[k : k + block]) / block) for k in range(0, n - block, block)]
    for lo, hi in bins:
        ks = [k for k, a in enumerate(amp) if lo <= a < hi]
        if not ks:
            continue
        row = []
        for _, d in demods:
            sq = sum((d[j] - d_exact[j]) ** 2 for k in ks for j in range(k * block, k * block + block))
            row.append(math.sqrt(sq / (len(ks) * block)) * FS / 256e3)
        print(f"{lo:>4}-{hi:<5} {100 * len(ks) / len(amp):5.1f}% " + " ".join(f"{x:17.0f}" for x in row))

    if png:
        width = int(LINE)
        panels = [d_exact] + [d for _, d in demods[:3]]
        rows = []
        for line in range(int((n - 1) / LINE)):
            start = int(line * LINE)
            row = []
            for d in panels:
                row += [max(0, min(255, int((v + 52) * 2.4))) for v in d[start : start + width]] + [0] * 8
            rows.append(row)
        write_png(png, rows, len(panels) * (width + 8))
        print(f"{png}: exact, CPU, BS, and BS with its offset 1 out, {len(rows)} lines")


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "capture":
        capture(sys.argv[2], sys.argv[3])
    elif len(sys.argv) >= 3 and sys.argv[1] == "compare":
        compare(sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None)
    else:
        sys.exit(__doc__)
