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
- The OSD alone carries no colorburst: the back porch shows nothing above noise at 3.58 or 4.43 MHz, where a 40 IRE burst would stand about 20 dB clear. With a camera attached the burst is there. `iq.py burst` checks this.

## Field timing

- The line clock tracks hsyncs to about 0.12 samples, but drawing each line from its nearest whole sample added 0.29 samples rms of rounding that changes from line to line, so vertical edges looked wavy. Starting lines on the nearest half sample (averaging neighbouring luma sums) halves it.

- An odd field's first broad pulse starts half-way through line 3 and an even field's at the start of line 4, which the line clock has only just begun. Counting both as line 3 drew one field two rows too high, so static text alternated between positions 3 rows apart at 60 fps and every horizontal edge looked doubled. Counting the even field from line 4, and wrapping an even field after 263 lines and an odd one after 262, puts the fields 1 row apart and cut vertical corrections from about one per 10 fields to a handful per thousand.

- With a camera attached, stretches of the picture stay below the sync slicer for over 18 µs several times a field, which read as vsync and reset the line count before the field ended. Fields then stopped for seconds and the camera fell back to the test pattern. A vertical flywheel fixes it: a vsync counts only within 6 lines of where the count expects one, unless 3 fields in a row have passed without one there. In 1,545 fields it ignored 386 false broad pulses and made one correction.
- `decode tap` dumps can stall part way while the camera streams, and the console then stops answering until the P4 is reset.

## Color

- At 13.33 MS/s the NTSC subcarrier is exactly 189/704 of the sample rate, so a 704-entry table of the subcarrier repeats with no drift and the burst phase measured against the absolute sample count holds steady from line to line. Averaging it over lines (weight 1/4 per line) leaves about 4° of jitter. The transmitter's subcarrier was about 48 Hz off nominal, which the averaging follows.
- Packing cos and sin into one 32-bit table entry (65536·cos + sin, amplitude 31) gets both products of a sample from one multiply-add.
- Hue calibrated against SMPTE bars from a test card came within about 15° on every bar.
- FM noise rises with frequency, so chroma is the noisiest part of the picture. Smoothing 8-sample chroma blocks 1 2 1 with their neighbours quiets it visibly at full saturation.
- Sharp luma edges such as OSD text leak into chroma as color fringes, which the 1 2 1 smoothing spread about 24 pixels sideways. The subcarrier turns half a cycle each line, so summing two lines' demodulated chroma cancels the leak while the color adds; it cleared most of the fringing around the OSD text.
- The hardware JPEG encoder times out converting YUV 4:2:2 input to 4:2:0, so fields are encoded as 4:2:2. It reads each pixel pair from memory as V Y0 U Y1.

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
- Chroma and sync cost too much to share a core with drawing, and an overloaded core 0 took USB and the console down with it. Moving sync to core 1 and drawing to its own task on core 0 fixed that. The drawing task then fell up to 60 lines behind (dropping hundreds a second) while using only 58% of the core, because USB and UVC tasks at its priority time-sliced it in 1 ms slices. Raising it above them removed every drop.
- Unrolling the luma loop to 8 pixels from a running 4-sample sum took drawing from about 72-88% of core 0 to 58%.
- The two HP cores share one L1 data cache. Core 0 reading the previous field from PSRAM through the cache (1.4 KB a line) took core 1 from 82% to 98%, and moving the hot code to IRAM made no difference. Fetching the rows by DMA into internal RAM instead left core 1 unaffected.
- `esp_cache_msync` costs about 4K cycles a call around a much cheaper cache operation, since it takes a mutex and a critical section. Calling the cache HAL's invalidate directly is not safe: the cache sync unit is shared, and unlocked calls on one core collided with the other core's and corrupted the picture. The fix is fewer calls: one invalidate per eight fetched rows.
- `esp_async_memcpy` from a PSRAM source cost about 40K cycles a call, writing the source back through the cache and allocating descriptors every time. A dedicated AXI-GDMA channel pair with descriptors written through the uncached alias costs a few hundred. The driver also rejects PSRAM addresses and lengths that aren't whole 64-byte cache lines, and logs each rejection, which on a per-line path saturated core 0.
- PIE vector loads fault on the uncached internal RAM alias; scalar accesses there work.
- Chroma mixing with PIE (`esp.vmulas.s8.xacc`, reading the running accumulator after each 8-sample half) takes about 1.5K cycles a line against about 4K for the scalar multiply-adds. Blocks sit on the absolute 8-sample grid so the ring and the subcarrier table are both 16-byte aligned (704 is a multiple of 16), and each picture block is interpolated from the two it straddles. The scalar code around it runs near one instruction per cycle whether its data is cached or not, so the remaining cost is instruction count: computing and finishing chroma for every line cost a third of a core, and was split so core 1 mixes each line and core 0 finishes each pair.
- Detecting lines after each 2016-sample DMA node, rather than after everything that has arrived, got lines to core 0 sooner but made vertical corrections jump from about one per thousand fields to one per twelve, likely from broad pulses split across calls. It was reverted.
- Packing four pixels per word store to avoid byte-store aliasing gained little (9.4 to 7.5 cycles per sample).
- `decode bench` varies about 15% between runs, and its small cached-buffer case runs slower than the full ring for reasons not understood. The live load figures in `decode` are the better measure.
- BitScrambler can attach to PARLIO RX on the P4 and holds a 2048-entry, 32-bit table (11-bit address), so it could take the phase lookup off the CPU with a smaller index.

## Encoding and USB

- The hardware JPEG encoder takes 2.5-3.5 ms for a 720×480 grayscale frame. At quality 80 grainy fields overflowed the 128 KB slot; quality 70 gives about 60-70 KB.
- On macOS, open the P4 console (303a:8000) with RTS off and DTR on: RTS high with DTR low looks like esptool's reset, and the P4 only writes to a port with DTR set.
