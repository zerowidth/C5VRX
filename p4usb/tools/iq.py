"""Capture I/Q from the P4 console and look at it offline.

    iq.py capture out.bin [sram] capture two fields (or what fits in internal RAM) and save the raw words
    iq.py lanes in.bin         per-lane toggle rates and I/Q statistics
    iq.py field in.bin out.png FM-demodulate and draw the samples as NTSC lines
    iq.py gaps in.bin [every]  find hsync spacings that aren't whole lines, which mean lost samples
    iq.py burst in.bin [every] look for an NTSC or PAL colorburst on the back porch

Needs numpy, pillow and pyserial.
"""

import sys
import time

import numpy as np

RATE = 40e6
LINE = 63.5555e-6


def console():
    import serial
    import serial.tools.list_ports

    port = next(p.device for p in serial.tools.list_ports.comports() if (p.vid, p.pid) == (0x303A, 0x8000))
    s = serial.Serial(None, 115200, timeout=0.2)
    s.port = port
    # DTR on so the P4 writes to us; RTS off, since RTS with DTR low looks like an esptool reset.
    s.rts = False
    s.dtr = True
    s.open()
    return s


def run(s, cmd, until, timeout):
    s.reset_input_buffer()
    s.write((cmd + "\n").encode())
    out = b""
    end = time.time() + timeout
    while until not in out and time.time() < end:
        out += s.read(1 << 16)
    return out


def capture(path, cmd="iq"):
    s = console()
    text = run(s, cmd, b"\n> ", 5).decode(errors="replace")
    print(text.strip())
    out = run(s, "iq dump", b"-----END IQ-----", 30)
    head = out.index(b"-----BEGIN IQ ")
    nl = out.index(b"\n", head)
    n = int(out[head + 14 : nl].split(b"-")[0])
    data = out[nl + 1 : nl + 1 + n]
    if len(data) < n:
        # The P4 gives up on a stalled console write, for example while the camera streams.
        print(f"only {len(data)} of {n} bytes arrived")
        data = data[: len(data) // 2 * 2]
    open(path, "wb").write(data)
    print(f"saved {n // 2} samples to {path}")


def load(path):
    w = np.fromfile(path, dtype="<u2")[64:]
    i = (w & 0xFF).astype(np.uint8).view(np.int8).astype(np.float32)
    q = (w >> 8).astype(np.uint8).view(np.int8).astype(np.float32)
    return w, i, q


def lanes(path):
    w, i, q = load(path)
    for side, shift in (("I", 0), ("Q", 8)):
        rates = []
        for k in range(1, 8):
            b = (w >> (shift + k)) & 1
            rates.append(f"{side}{k} high {b.mean():.2f} toggles {np.mean(b[1:] != b[:-1]):.2f}")
        print("  ".join(rates))
    for name, v in (("I", i), ("Q", q)):
        print(f"{name}: mean {v.mean():.1f} std {v.std():.1f} min {v.min():.0f} max {v.max():.0f}")
    mag = np.hypot(i, q)
    print(f"|IQ| mean {mag.mean():.1f}, near zero (<8) {np.mean(mag < 8):.2f}")


def demod(i, q):
    z = (i - i.mean()) + 1j * (q - q.mean())
    return np.angle(z[1:] * np.conj(z[:-1])) * RATE / (2 * np.pi)


def field(path, png):
    from PIL import Image

    w, i, q = load(path)
    f = demod(i, q)
    print(f"instantaneous frequency: mean {f.mean() / 1e6:.2f} MHz, std {f.std() / 1e6:.2f} MHz")
    # Smooth to roughly video bandwidth, then fold at the nominal line period.
    k = np.ones(8) / 8
    v = np.convolve(f, k, mode="same")
    spl = LINE * RATE
    nlines = int(len(v) / spl) - 1
    width = 720
    rows = np.empty((nlines, width), np.float32)
    for n in range(nlines):
        x = n * spl + np.arange(width) * spl / width
        rows[n] = np.interp(x, np.arange(len(v)), v)
    lo, hi = np.percentile(rows, [1, 99])
    img = np.clip((rows - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
    Image.fromarray(img).save(png)
    print(f"{nlines} lines, levels {lo / 1e6:.2f}..{hi / 1e6:.2f} MHz, saved {png}")


def lut_demod(w, i, q, step=3):
    """The firmware's demodulator: I/Q index to 8-bit phase, DC removed, every `step`th word, so 13.33 MS/s."""
    k = np.arange(1 << 14)

    def s7(x):
        x = x & 0x7F
        return np.where(x >= 64, x - 128, x) * 2 + 1

    lut = (np.round(np.arctan2(s7(k >> 7) - q.mean(), s7(k) - i.mean()) / (2 * np.pi) * 256) % 256).astype(np.uint8)
    p = lut[((w >> 1) & 0x7F) | (((w >> 9) & 0x7F) << 7)][::step].astype(np.int32)
    return (((p[1:] - p[:-1] + 128) % 256) - 128) * (RATE / 3) / 256


def hsyncs(f, fs):
    """Leading and trailing edges of the hsync pulses in demodulated frequency f."""
    s = np.convolve(f, np.ones(6) / 6, "same")
    tip = np.percentile(s, 2)
    blank = np.median(s[(s > tip + 1e6) & (s < tip + 3e6)])
    low = s < (tip + blank) / 2
    starts = np.flatnonzero(low[1:] & ~low[:-1]) + 1
    ends = np.flatnonzero(~low[1:] & low[:-1]) + 1
    ends = ends[np.searchsorted(ends, starts[0]) :]
    n = min(len(starts), len(ends))
    width = (ends[:n] - starts[:n]) / fs
    ok = (width > 3.5e-6) & (width < 6e-6)
    return starts[:n][ok], ends[:n][ok], tip, blank


def gaps(path, every=2):
    """every: the C5 kept one of this many 80 MS/s bus samples (2 for 40 MS/s captures, 6 for the decoder)."""
    w, i, q = load(path)
    f = lut_demod(w, i, q, step=6 // every)
    fs = RATE / 3
    line = LINE * fs
    hs, _, tip, blank = hsyncs(f, fs)
    hs = hs.astype(float)
    r = np.diff(hs) / line
    off = np.abs(r - np.round(r))
    bad = np.flatnonzero((off > 0.03) & (np.round(r) >= 1) & (np.round(r) <= 3))
    print(f"sync tip {tip / 1e6:.2f} MHz, blanking {blank / 1e6:.2f} MHz, {len(hs)} hsyncs over {len(f) / line:.0f} lines")
    print(f"{len(bad)} spacings off the line grid")
    for b in bad[:20]:
        print(f"  byte {int(hs[b] * 6)}: {r[b]:.3f} lines")


def burst(path, every=2):
    """Compares the back porch's spectrum at the NTSC and PAL subcarriers with FM noise either side."""
    w, i, q = load(path)
    f = lut_demod(w, i, q, step=6 // every)
    fs = RATE / 3
    _, ends, _, _ = hsyncs(f, fs)
    # The burst starts about 0.6 us after the sync's trailing edge and lasts 2.5 us.
    a, b = int(0.2e-6 * fs), int(3.6e-6 * fs)
    ends = ends[ends + b < len(f)]
    porch = np.array([f[e + a : e + b] - f[e + a : e + b].mean() for e in ends])
    p = np.mean(np.abs(np.fft.rfft(porch, 512, axis=1)) ** 2, axis=0)
    freq = np.fft.rfftfreq(512, 1 / fs)
    print(f"{len(ends)} lines")
    for name, fsc in (("NTSC", 315e6 / 88), ("PAL", 4.43361875e6)):
        at = np.argmin(abs(freq - fsc))
        side = np.r_[p[at - 30 : at - 10], p[at + 10 : at + 30]]
        print(f"{name} {fsc / 1e6:.2f} MHz: {10 * np.log10(p[at] / np.median(side)):+.1f} dB over the noise beside it (a 40 IRE burst is some 20 dB)")


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "capture":
        capture(sys.argv[2], " ".join(["iq"] + sys.argv[3:]))
    elif cmd == "lanes":
        lanes(sys.argv[2])
    elif cmd == "gaps":
        gaps(sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 2)
    elif cmd == "burst":
        burst(sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 2)
    elif cmd == "field":
        field(sys.argv[2], sys.argv[3])
