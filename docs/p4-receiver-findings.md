# P4 receiver findings

What the C5 + P4 receiver has shown on the hardware so far: the C5-Zero exporting MODEM_DIAG I/Q to the P4-Pico, which demodulates FM, decodes NTSC and serves a UVC camera. The build itself is described in [P4 receiver plan](p4-receiver-plan.md). Numbers are from one transmitter on R3 (5732 MHz) on the bench unless noted.

## The C5's I/Q

- MODEM_DIAG carries Q on bits 0-9 and I on bits 10-19, each 10-bit two's complement, changing at about 80 MS/s. The C5 exports the top 7 of each (DIAG 9-3 and 19-13) on 14 lanes, and the P4 reads each as 2v+1 by tying PARLIO lines 0 and 8 to constant 1.
- A PARLIO TX looped clock divided from the modem's PLL keeps one of every N bus samples with no drift. N=6 (13.33 MS/s) is enough: the carrier plus deviation spans about −3 to +1.5 MHz, inside the ±6.67 MHz the phase difference follows without wrapping. Decimating without a filter only folds in noise, since each kept sample's phase is still exact.
- I carries a DC offset that grows with gain: about 20 of ±127 at gain 64 and about 32 at 68. Q's is near 0. Upper I lanes that look stuck are sign bits riding on that offset. The phase table has to be built around the measured offset, or a small signal's phase is badly distorted.
- The gain index is steep and nonlinear. On one bench setup, index 62 gave an I/Q RMS of 19, 72 gave 61 with 0.4% clipped, and 80 gave 114 with 69% clipped.
- The carrier sits within about 0.3 MHz of the tuned center. R3 is reached from Wi-Fi channel 144 with `phy_set_freq`.
- Nothing upstream filters the video: a sync edge rises from tip to blanking within one 75 ns sample, so 3.58 MHz chroma would pass.

## The demodulated signal

- One unit of 8-bit phase difference at 13.33 MS/s is about 52 kHz. Sync tip sits near −3.0 MHz and blanking near −0.85 MHz, so 40 IRE is about 2.1 MHz and 1 IRE about 52 kHz, one unit.
- On a good signal the back porch carries about 165 kHz RMS of noise (about 3 IRE) with an exact floating-point demodulator. It is FM noise, rising about 17 dB from 0.3 to 6.5 MHz, so a luma low-pass removes most of the visible grain.
- The phase table's rounding matters only at low amplitude. At RMS 19 the porch noise was 241 kHz with a 6+6-bit table against 185 kHz exact; at RMS 61 it was 172 against 164. Hence the gain control now aims for RMS 40-72, and a 7+7-bit table isn't worth its 32 KB.
- The signal can degrade within minutes for reasons outside the receiver: in one session porch noise rose to 700-870 kHz at every gain from 62 to 72 and lock fell to 40-210 of 262 lines.
- The current source (the flight controller's OSD on gray, no camera) has no colorburst: the back porch shows nothing above noise at 3.58 or 4.43 MHz, where a 40 IRE burst would stand about 20 dB clear. `iq.py burst` checks this.

## PARLIO RX and DMA on the P4

- The receive call puts its DMA descriptor list (a VLA, 16 bytes per 4 KB) on the caller's stack, so a 2.8 MB capture needs a 20 KB task stack.
- The soft delimiter's EOF length is limited to 0xFFFF bytes; long or continuous captures use partial receive.
- The driver counts one DMA node per interrupt and falls behind when two nodes finish before it runs. The decoder reads the write position from the AXI-GDMA instead: the channel's `in_dscr_bf0` points at the current descriptor, whose second word is its buffer address.
- A soft-delimiter EOF that lands mid-node ends that node early and leaves the rest of it stale, which showed up as staircase shifts in the picture. The ring (24 nodes of 4032 bytes) and the EOF period (8 nodes) are whole numbers of nodes.
- The best P4 sampling edge varies; a boot check that captures on each edge and keeps the one with smaller sample-to-sample jumps works.

## P4 CPU and memory

- The HP cores at 360 MHz give 27 cycles per sample at 13.33 MS/s. Loops here are instruction-bound: about one ALU operation per cycle, loads a few cycles each, and the 8 KB SPM (TCM) no faster than cached internal RAM.
- Index arithmetic dominates the demodulator. Looking up a 16 KB table with the raw word shifted by two, instead of a 4 KB table needing five operations to build its index, took the demodulator from 68% of core 1 to 53%.
- Writing PSRAM through the CPU cache costs about 5 cycles per byte and slows everything else on the memory path. Rendering lines into internal RAM and DMA'ing them to the PSRAM frame 16 rows at a time took core 0 from 74% to 64%, core 1 from 84% to 68% with no change to its code, and the JPEG encode from 4.6 to 2.5 ms. The AXI-GDMA async memcpy accepts a PSRAM destination at 16-byte alignment.
- Packing four pixels per word store to avoid byte-store aliasing gained little (9.4 to 7.5 cycles per sample).
- `decode bench` varies about 15% between runs, and its small cached-buffer case runs slower than the full ring for reasons not understood. The live load figures in `decode` are the better measure.
- BitScrambler can attach to PARLIO RX on the P4 and holds a 2048-entry, 32-bit table (11-bit address), so it could take the phase lookup off the CPU with a smaller index.

## Encoding and USB

- The hardware JPEG encoder takes 2.5-3.5 ms for a 720×480 grayscale frame. At quality 80 grainy fields overflowed the 128 KB slot; quality 70 gives about 60-70 KB.
- On macOS, open the P4 console (303a:8000) with RTS off and DTR on: RTS high with DTR low looks like esptool's reset, and the P4 only writes to a port with DTR set.
