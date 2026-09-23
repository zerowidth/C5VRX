"""Decode a capture of the C5VRX DAC bus into grayscale NTSC video.

Input is a Logic 2 binary export directory (digital_0.bin .. digital_5.bin, channel N on
DAC bit N, captured at 50 MS/s, the Logic 8's ceiling with six channels) or a raw file of 20 MS/s codes, one byte each, as written by --codes.
"""

import argparse
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image

import ntsc
import saleae


def load(source):
    source = Path(source)
    if source.is_dir():
        codes, grid = saleae.recover_codes(saleae.read_bus(source))
        print(f"symbols {grid.count}, C5 clock {grid.ppm:+.1f} ppm vs analyzer, "
              f"edge phase residual {grid.residual_ns:.2f} ns rms")
        return codes
    return np.fromfile(source, np.uint8)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source")
    parser.add_argument("--codes", help="write the recovered 20 MS/s codes here")
    parser.add_argument("--png", help="write one PNG per output frame into this directory")
    parser.add_argument("--mp4", help="write an H.264 video here (needs ffmpeg)")
    parser.add_argument("--weave", action="store_true", help="pair fields into frames instead of line-doubling each")
    parser.add_argument("--white", type=float, default=ntsc.WHITE_CODE, help="DAC code that maps to white")
    parser.add_argument("--luma-cutoff", type=float, default=3e6, help="low-pass before sampling, in Hz, to hide chroma")
    args = parser.parse_args()

    codes = load(args.source)
    if args.codes:
        codes.tofile(args.codes)
    print(f"{len(codes) / ntsc.RATE * 1e3:.1f} ms of video, code histogram peaks: "
          + ", ".join(str(c) for c in np.argsort(np.bincount(codes, minlength=64))[::-1][:5]))

    fields = ntsc.find_fields(codes)
    print(f"{len(fields)} fields")
    if not fields:
        return
    signal = ntsc.lowpass(codes.astype(np.float32), args.luma_cutoff)
    images = []
    for field in fields:
        image, blank = ntsc.field_image(signal, field, args.white)
        images.append((field, image))
        print(f"  field at {field.vsync / ntsc.RATE * 1e3:8.3f} ms, parity {field.parity}, "
              f"blank {blank:5.1f}, {field.missing} missing syncs")

    if args.weave:
        frames = [ntsc.weave(a, b) for (fa, a), (fb, b) in zip(images, images[1:])
                  if fa.parity == 0 and fb.parity == 1]
    else:
        frames = [ntsc.bob(image, field.parity) for field, image in images]

    if args.png:
        out = Path(args.png)
        out.mkdir(parents=True, exist_ok=True)
        for i, frame in enumerate(frames):
            Image.fromarray(frame).save(out / f"frame_{i:04d}.png")
    if args.mp4:
        rate = "30000/1001" if args.weave else "60000/1001"
        height, width = frames[0].shape
        subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "gray",
             "-s", f"{width}x{height}", "-r", rate, "-i", "-",
             "-vf", "setsar=10/11", "-c:v", "libx264", "-pix_fmt", "yuv420p", args.mp4],
            input=b"".join(frame.tobytes() for frame in frames), check=True,
        )


if __name__ == "__main__":
    main()
