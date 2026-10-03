# P4 receiver findings

What the C5 + P4 receiver has shown on the hardware so far: the C5-Zero exporting MODEM_DIAG I/Q to the P4-Pico, which demodulates FM, decodes NTSC and serves a UVC camera. The build itself is described in [P4 receiver plan](p4-receiver-plan.md). Numbers are from one transmitter on R3 (5732 MHz) on the bench unless noted. Sections follow the signal path, from the C5's I/Q through demodulation, timing and color to the P4's own limits and USB.

## The C5's I/Q

- MODEM_DIAG carries Q on bits 0-9 and I on bits 10-19, each 10-bit two's complement, changing at about 80 MS/s. The C5 exports the top 7 of each (DIAG 9-3 and 19-13) on 14 lanes, and the P4 reads each as 2v+1 by tying PARLIO lines 0 and 8 to constant 1.
- A PARLIO TX looped clock divided from the modem's PLL keeps one of every N bus samples with no drift. N=6 (13.33 MS/s) is enough: the carrier plus deviation spans about −3 to +1.5 MHz, inside the ±6.67 MHz the phase difference follows without wrapping. Decimating without a filter only folds in noise, since each kept sample's phase is still exact.
- I carries a DC offset that grows with gain: about 20 of ±127 at gain 64 and about 32 at 68. Q's is near 0. Upper I lanes that look stuck are sign bits riding on that offset. The phase table has to be built around the measured offset, or a small signal's phase is badly distorted.
- The gain index is steep and nonlinear. On one bench setup, index 62 gave an I/Q RMS of 19, 72 gave 61 with 0.4% clipped, and 80 gave 114 with 69% clipped.
- The carrier sits within about 0.3 MHz of the tuned center. R3 is reached from Wi-Fi channel 144 with `phy_set_freq`.
- Nothing upstream filters the video: a sync edge rises from tip to blanking within one 75 ns sample, so 3.58 MHz chroma would pass.

## C5 starts that read the lanes mid-change

About three C5 starts in ten come up with the P4 sampling the I/Q lanes while they change, which adds about 18 dB of noise. The C5's clock can be moved against its bus by running it a tick slow for a moment, and at each start the P4 now moves it until the lanes read steady: with a transmitter on, 38 starts in 38 ended clean. The clock has three positions against the bus, one of them bad, and one slip always leaves the bad one.

How it happens:

- The C5 routes the modem's diagnostic bus straight to 14 pads through the GPIO matrix. The bus takes a new value every 12.5 ns, timed by the modem's own clock.
- The sample clock on the 15th wire comes from somewhere else: the C5's PARLIO transmitter, dividing its 240 MHz PLL output by 18.
- Nothing registers the data onto that clock. The P4 latches all 14 lanes on the clock's edge, which has to land where they are steady. On a bad start it lands on the change, and some lanes hold the last sample's bit and some the next one's.
- Neighbouring bus samples mostly differ in their low bits, so a mixed read is nearly right. Where a lane crosses zero every upper bit turns over at once and a mixed read is garbage: Q read -19, -113, 45 in one capture.

What it does, on the same signal at the same gain:

- 3.7% of samples sit far off the carrier's ring, whose radius otherwise varies 4%. Both lanes glitch, Q more than I.
- Sync tip noise is about 1000 kHz against 100 to 160, and 220 of 262 lines lock against 253.
- Every measurement in this document from before this was found may have been made in either state. The weak-signal figures of about 1000 kHz and 220 lines were the bad one.

What is established:

- It is fixed at the C5's reset: 3 bad starts in 10, 3 in 10 and 3 in 12.
- Retuning 12 times, restarting the C5's clock 10 times, and running the clock at other divisors in between 10 more times all left it as it was.
- Neither P4 edge avoids it. At one sample in six the edges are 37.5 ns apart, a whole 3 bus periods, so they read the bus at the same point.

Moving the clock:

- `clk slip US` on the C5 raises the PARLIO clock's divisor from 18 to 19 for about that many microseconds and puts it back. Each clock period that passes meanwhile moves the edge one 240 MHz tick (4.17 ns) later, and the bus repeats every 3 ticks. The register writes alone take a few periods, so the move is some ticks, not a chosen number.
- It works: one slip after another, the two edges' glitch counts on noise went from 13 and 15 per thousand to 0 and 243, then 6 and 6, and so on.
- So restarting the clock, which earlier never moved it, must restart the divider the same way each time.
- On noise the two P4 edges did not always read alike after a slip (6 and 6 on one, 0 and 218 on the other), and more than three different readings turned up. Noise is a poor guide: the map below, with a carrier, shows neither.

The off-ring count after each of 40 slips of 1 us, with a carrier, both edges read twice:

- There are two states and nothing between. In 24 positions both edges read 0 to 18 per thousand, and in 16 the rising edge read 33 to 51 and the falling 24 to 42.
- The edges go bad together, as edges 3 bus periods apart should. Their sum separates the states better than either: 25 at most clean, 62 at least bad.
- A bad position never followed a bad one in 39 slips, and clean followed clean 9 times. That fits three positions a tick apart, one bad, with a slip of 1 us moving one or two ticks and never three. 16 bad in 40 fits one in three.
- So the bad window is narrower than 4.17 ns, and each clean position is one tick from it. Where the window sits inside its tick, and so the smaller margin, is not known.
- Twice in 130 reads one edge read bad and the other clean (35 and 9, 38 and 7). The edges are captured one after the other and the received strength was moving by 10 dB, so a fade is the likely cause.

What is not known:

- What sets the position at reset. The guess is how the modem's 80 MHz clock and the 240 MHz clock line up.
- How much margin the nearer clean position has: somewhere between 0 and 4.17 ns. If it is small, temperature or drift could carry a session over the edge, and nothing would notice.
- Whether the two clean positions differ. The counts do not tell them apart.
- Whether the standalone C5 receiver, which samples the same bus on a looped PARLIO clock, has the same lottery.

What `iq_start` does:

- It sets a gain that leaves the RMS between 30 and 80 with under 2% clipped, since a clipped signal hides what the test looks for.
- With a carrier it counts, on each P4 edge, the samples well off the carrier's ring (under 0.35 or over 2.1 times the mean power). The two edges read the bus at the same point, so it judges their sum: over 40 per thousand counts as mid-change, between the 25 and 62 of the map above.
- With no carrier (under 60% of samples near the mean power) there is no ring, so it counts the samples that stand further outside their two neighbours than the lane's RMS. Noise gives 6 to 9 per thousand read clean, as independent samples do, and 11 to 200, or hardly any when the garbage is most of the lane's power, otherwise.
- That second count is a poor test with a carrier: clean reads gave 1 to 8 and bad ones 6 to 26, and a start it passed at 6 and 9 decoded with 1025 kHz of noise. That is why a carrier is judged by its ring.
- While it reads mid-change it slips the clock and counts again, up to 16 times. On noise, where the edges are judged apart, it settles for one clean edge after 8.
- With a carrier and each edge judged alone at 10 per thousand, 14 starts took 1 to 4 probes each and all decoded with 151 to 249 kHz of sync tip noise. Six of the 14 came up mid-change.
- Judged on the sum, 24 starts all ended clean: 18 on the first probe, 5 after one slip and 1 after two, the second probe of that one being a 38 and 7 read. Sync tip noise after the last was 182 kHz.
- On noise the search often runs all 16 tries without both edges clean, about 3 s, and noise is the poorer guide. So a start with no carrier is marked, and the decoder starts once more when a transmitter first locks most of a field. The restart runs on the control task, whose 3 KB stack the probe's receive overflowed (about 6 KB of DMA descriptors), so the first lock after a boot on an empty channel panicked. With 12 KB it restarts about a second after the lock and is decoding again 2 s later.
- Before slipping was found, the P4 reset the C5 instead until it came up clean: with a carrier, 3 starts in 12 needed one more reset and all 12 ended clean.

Still to do:

1. Find how much margin the chosen position has, and whether it holds as the boards warm up.
2. Look at the clock against a Q lane on an oscilloscope, on good and bad starts, for the margin. The Saleae's 50 MS/s is too slow.

## The C5's gain table

- The IDF 6.1 PHY library (esp-phy-lib 5695f4f) keeps the gain table where main's pinned library (59c1234) does: stage spans at `phy_param+0x422` and table maxima at `+0x124..0x126`, confirmed in `phy_set_rx_gain_table`'s disassembly.
- Read back from c5rx after tuning, the spans are 15, 13, 5, 8, 6, 1, 4, 6 and the maxima 77, 77, 83. Main's defaults assume 4 for the sixth span. The table was the same on L1, R1, R3 and 5885 MHz and after a gain write.
- Decoded with main's `arc_gain_tuple_decode`, RF stages start at indices 15, 28, 33, 41, 47, 48, 52 and 58. Stage 8 runs 58-77 through BB banks 1, 3, 7 and 15 with six fine steps each, 5 down to 0.
- Indices above 77 are outside the table. With no VTX, 79 clipped 3% of samples, 80 clipped 10-47% and 81-89 clipped 95-99%, so the old gain limit of 80 was itself a broken state.
- On noise with no VTX, RMS rose from 1 below index 35 to about 23 at 77 in one run. In another, with something else on the air, it rose from 13 to 37 across RF 7's fine steps (52-57) and plateaued near 46 from 60 upward. Stage boundaries don't give an even step, so a fixed index step of 2 lands unevenly.

- With no VTX on R3, V3 held the survival gain of 58, but bursts of interference with 13-18% of a window's samples clipped and a median power of 1 made it drop the gain and return about 6 times a second: 32 overloads and 65 writes in about 10 s. Depending on origin occupancy and coherence, the same window reads as no carrier or as saturation.

- With a strong VTX on R3, line lock stayed at 253-254 of 262 at every fixed gain from 28 to 50, including 42-50 where 46-100% of samples clipped. FM survives hard limiting, so clipping alone is not a reason to cut gain. Lock fell only from 52 upward (205 at 52, 178 at 64).
- V3 on that signal: from a forced gain of 58 (all samples clipped) it stepped 58, 52, 48, 47, 41, 33, 34 and held within about 85 ms. The old control takes about 2 s to come down that far. Left alone for a minute it made two fine steps and held at 254/262. Over 30 s runs both modes averaged about 220 of 262 lines while the signal was weaker, but V3 made 211-262 writes per run against none for the old control.
- Those writes came from windows with a median power of 1 and all samples at the origin, alternating with interference bursts. V3 reads the first as no carrier and goes to 58, and reads the bursts as overloads and drops to 52, so it swings between the two. This matches the no-VTX behaviour, so the signal was probably missing or very weak during those runs.
- At a median power of 7 the coherence at 13.33 MS/s fell to about 45%, under V3's 55% carrier test, so V3 stopped raising the gain there (gain 33). At a median power of 9 it was 79-88% and V3 stepped up normally.

## The demodulated signal

- One unit of 8-bit phase difference at 13.33 MS/s is about 52 kHz. Averaged over locked lines, the sync tip sits near −2.2 MHz and the back porch near −1.0 MHz, so 40 IRE is about 1.2 MHz and 1 IRE about 30 kHz. The picture's white reaches about +2 MHz.
- Blanking taken as the median of the level histogram 1-3 MHz above the sync tip reads about 0.45 MHz (15 IRE) high, because dark picture content falls in that band too. With the sync depth scaled from it, white was mapped about 1.45 times too far out and the darkest sixth of the picture clipped to black, which looked dim beside an FPV monitor. Averaging the sync tip and back porch of every other locked line fixed both.
- On a good signal the back porch carries about 165 kHz RMS of noise (about 3 IRE) with an exact floating-point demodulator. It is FM noise, rising about 17 dB from 0.3 to 6.5 MHz, so a luma low-pass removes most of the visible grain.
- Luma is each pixel's sum of four samples smoothed 1 2 1 (seven taps, in `luma.S`). Against the plain sum, flat areas of a recording were 3.6 dB quieter from field to field (2.2 levels rms against 3.3, after JPEG), OSD text stayed crisp, and core 0 went from 39% to 43%. The response gives up 0.5 dB at 1 MHz and 2 dB at 2 MHz. `decode luma plain` returns to the plain sum for comparison, and `decode luma peak` peaks it -1 4 -1 instead, which brings back an OSD outline beside a stem (one or two samples wide, and averaged with the stem by the sum) for twice the noise of the smoothed sum. By eye the smoothed sum is still the better picture, so it stays the default.
- A line with an FM click is drawn in the color of the lines above. A click is a step of most of a cycle between two samples, and it rings in the chroma band as a dash of saturated color. In a raw capture, lines with a dash had a sample-to-sample step of 150 or more of the 256 in a cycle, and clean lines stayed under 80 (an OSD edge is about 72), so `jumps.S` finds each line's largest step and 110 marks a click. The line and the one it shares chroma with take the last pair's color, and the click stays as a black or white speck, so the noise still shows without the dash. At 320 kHz of sync-tip noise about one line in ten had a click. It costs 2.5% of core 1. `decode clicks gray` draws them without color instead, which showed as gray rows in a colorful scene, and `decode clicks off` draws them in their own color for comparison.
- With the plain sum, lines starting on a half sample (five taps, 1 2 2 2 1) carried 3.8 dB less noise than whole-sample ones, by calculation for noise rising 6 dB an octave. Smoothed, the two differ by 0.8 dB.
- The phase table's rounding matters only at low amplitude. At RMS 19 the porch noise was 241 kHz with a 6+6-bit table against 185 kHz exact; at RMS 61 it was 172 against 164. Hence the gain control now aims for RMS 40-72, and a 7+7-bit table isn't worth its 32 KB. The table now in use keeps 6 bits of I and all 7 of Q in 16 KB, since indexing by the raw word brought the seventh Q bit for free.
- The signal can degrade within minutes for reasons outside the receiver: in one session porch noise rose to 700-870 kHz at every gain from 62 to 72 and lock fell to 40-210 of 262 lines.
- The OSD alone carries no colorburst: the back porch shows nothing above noise at 3.58 or 4.43 MHz, where a 40 IRE burst would stand about 20 dB clear. With a camera attached the burst is there. `iq.py burst` checks this.

## Field timing

- The line clock tracks hsyncs to about 0.12 samples, but drawing each line from its nearest whole sample added 0.29 samples rms of rounding that changes from line to line, so vertical edges looked wavy. Starting lines on the nearest half sample (averaging neighbouring luma sums) halves it.
- A line is about 847.46 samples at 13.33 MS/s. A leading hsync edge interpolated between samples has about 0.1 samples of noise; taken to the nearest sample it has about 0.36. The line clock's 1/8 phase gain filters either down to about 0.12, so sub-sample edge detection would not improve it further.

- An odd field's first broad pulse starts half-way through line 3 and an even field's at the start of line 4, which the line clock has only just begun. Counting both as line 3 drew one field two rows too high, so static text alternated between positions 3 rows apart at 60 fps and every horizontal edge looked doubled. Counting the even field from line 4, and wrapping an even field after 263 lines and an odd one after 262, puts the fields 1 row apart and cut vertical corrections from about one per 10 fields to a handful per thousand.

- With a camera attached, stretches of the picture stay below the sync slicer for over 18 µs several times a field, which read as vsync and reset the line count before the field ended. Fields then stopped for seconds and the camera fell back to the test pattern. A vertical flywheel fixes it: a vsync counts only within 6 lines of where the count expects one, unless 3 fields in a row have passed without one there. In 1,545 fields it ignored 386 false broad pulses and made one correction.

## Color

- At 13.33 MS/s the NTSC subcarrier is exactly 189/704 of the sample rate, so a 704-entry table of the subcarrier repeats with no drift and the burst phase measured against the absolute sample count holds steady from line to line. Averaging it over lines (weight 1/4 per line) leaves about 4° of jitter. The transmitter's subcarrier was about 48 Hz off nominal, which the averaging follows.
- Packing cos and sin into one 32-bit table entry (65536·cos + sin, amplitude 31) gets both products of a sample from one multiply-add.
- Hue calibrated against SMPTE bars from a test card came within about 15° on every bar.
- FM noise rises with frequency, so chroma is the noisiest part of the picture. Smoothing 8-sample chroma blocks 1 2 1 with their neighbours quiets it visibly at full saturation.
- Sharp luma edges such as OSD text leak into chroma as color fringes, which the 1 2 1 smoothing spread about 24 pixels sideways. The subcarrier turns half a cycle each line, so summing two lines' demodulated chroma cancels the leak while the color adds; it cleared most of the fringing around the OSD text.
- Chroma lines up with luma to within about a pixel, and a bar edge's chroma transition spans about 12 pixels, so horizontal chroma resolution is not what makes colors look smeared.
- At JPEG quality 70 the encoder flattens chroma noise into 8-row blocks, which show as horizontal color bands in flat areas. Chroma noise that changes from line to line is what the blocks are made of, so averaging chroma over lines reduces them too.
- The hardware JPEG encoder times out converting YUV 4:2:2 input to 4:2:0, so fields are encoded as 4:2:2. It reads each pixel pair from memory as V Y0 U Y1.

## PARLIO RX and DMA on the P4

- The receive call puts its DMA descriptor list (a VLA, 16 bytes per 4 KB) on the caller's stack, so a 2.8 MB capture needs a 20 KB task stack.
- The soft delimiter's EOF length is limited to 0xFFFF bytes; long or continuous captures use partial receive.
- The driver counts one DMA node per interrupt and falls behind when two nodes finish before it runs. The decoder reads the write position from the AXI-GDMA instead: the channel's `in_dscr_bf0` points at the current descriptor, whose second word is its buffer address.
- `in_dscr_bf0` moves on to a node while the end of the one before is not yet in memory, so the decoder reads up to one node behind it. Reading right up to it, the BitScrambler mode's fast demodulator took the old contents of a node's last third in a small share of nodes, which drew as stretches of picture from 38 lines earlier. A 10 s recording had about 900 such rows in 600 frames, 700 of them in the top 8 rows, which are black, against 5 with the CPU's table. The CPU's table and a C demodulator read the same nodes clean, being slow enough for the data to arrive. A node behind, 16 s of top rows had none.
- A soft-delimiter EOF that lands mid-node ends that node early and leaves the rest of it stale, which showed up as staircase shifts in the picture. The ring (24 nodes of 4032 bytes) and the EOF period (8 nodes) are whole numbers of nodes.
- The driver's node interrupt finds a zero length on the first node of every receive and logs "finished buffer is NULL or length is 0" with an early log, which prints straight to the UART at 115200 baud from the interrupt. That takes 6.4 ms with core 0's interrupts off, so the tick stops, the decode task on core 1 is not woken, and it loses about a ring and a half of samples (100 lines). It happened at every decoder start and retune, with or without the BitScrambler. The decoder now turns the ROM's printing off from each start until the control task's next pass.
- The P4's two sampling edges read the C5's bus at the same point (see [C5 starts that read the lanes mid-change](#c5-starts-that-read-the-lanes-mid-change)), so the choice between them matters little. `iq_start` keeps the one that glitches less.

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
- Luma in PIE (`luma.S`) takes about 2.4K cycles a line against about 6.8K for the scalar loop, and took core 0 from 75.5% to 57.5%. Each pixel was a running sum of four samples looked up in a 1 KB level table. Now 16 pixels at a time are weighted sums of five byte-shifted copies of the samples in the vector accumulator (`esp.vmulas.s8.qacc`), shifted and clamped to bytes by `esp.srcmb.s8.qacc`, and zipped with the chroma bytes. The weights carry the gain: their sum is as near 504 as the shift allows, which sets the level to about 0.35%. Blanking comes off each sample in whole units, with the remainder as a constant product that starts each sum. The output matched a C model on every pixel of random lines.
- PIE's aligned loads ignore an address's low four bits rather than faulting. The second copy of luma's constants sat 4 bytes off a 16-byte boundary, so the first 4 pixels of every 16 took the vector before's values and drew about 100 levels dark, in stripes down the picture. The check against the C model had only run on the first copy.
- `esp.srcmb.s8.qacc` clamps only with its last operand at 1; at 0 it wraps, and so does the unsigned form on a negative sum. `esp.movx.w.sar.bytes` takes a0 to a5 or t3 to t6 but not t0 to t2.
- The async memcpy driver cost about 46K cycles a strip, 5.7K a drawn line: it frees and builds both descriptor lists on every call, and for a destination off a cache-line boundary it splits the copy through a stash buffer. A DMA channel pair of the decoder's own, with descriptors rewritten in place and strips cut at even rows so every copy is whole cache lines, costs 1.1K a line (most of it writing the strip back from the cache) and took core 0 from 57.5% to 39.0%.
- Per drawn line with color, core 0 now spends about 8.2K cycles: 2.2K finishing chroma, 2.4K on luma, and the rest copying the line's samples to an aligned buffer, deinterlacing and fetching the previous field. Before luma moved to PIE it spent about 10.7K drawing luma and deinterlacing and about 2.2K finishing chroma, and core 1 about 4.1K mixing and interpolating it. Doing all of chroma on core 0 took it to 99.7% and dropped thousands of lines a second; doing all of it on core 1 took that to 99.6%.
- Detecting lines after each 2016-sample DMA node, rather than after everything that has arrived, got lines to core 0 sooner but made vertical corrections jump from about one per thousand fields to one per twelve, likely from broad pulses split across calls. It was reverted.
- Packing four pixels per word store to avoid byte-store aliasing gained little (9.4 to 7.5 cycles per sample).
- `decode bench` varies about 15% between runs, and its small cached-buffer case runs slower than the full ring for reasons not understood. The live load figures in `decode` are the better measure.

## The P4's BitScrambler

The BitScrambler does the phase lookup in the receive path, with a smaller table than a CPU can use, and nothing more. The decoder runs that way: core 1's load fell from 83% to 41% at the same line lock (see [In the decoder](#in-the-decoder)). The 16 KB table the CPU looked up before, and the `decode bs on|off` switch between the two, have been removed, so the comparisons below against the CPU's table can no longer be run. The BitScrambler cannot hold that 16 KB table, and it cannot do the arithmetic that follows the lookup. The limits below are read from ESP-IDF 6.1 (the BitScrambler docs, `tools/bsasm.py` and `bsasm_targets/esp32p4.json`), and the noise figures are simulated.

Its limits:

- The LUT is 2048 bytes in total: 2048×8, 1024×16 or 512×32. The widest address is 11 bits. The decoder's table is 16 KB (14-bit address), so it is 8 times too big, and a 16-bit table would be 32 times too big.
- There is no data-dependent add. `ADD` takes an immediate only, and `ADDCTI`, which adds routed bits to a counter, exists on the C5 and S31 but not the P4. So the P4 cannot subtract one phase from the last, sum samples, or add a correction to a table value.
- A `set` only routes bits. There is no XOR, negate or data-dependent shift.
- A program is at most 8 instructions.
- ESP-IDF connects the BitScrambler to PARLIO TX and to loopback only. On PARLIO RX it is attached by hand: create it for the RX direction on `SOC_BITSCRAMBLER_ATTACH_PARL_IO`, then enable, load, reset and start it before enabling the PARLIO unit.
- The LUT's address is output bits 16 and up of the cycle before (16..25 at 16-bit width), as the C5 programs use it. The IDF docs' "most significant N bits" reads as bits 22..31, which looks up the wrong entries.

What that rules out:

- CORDIC. It needs about three data-dependent add or subtracts per iteration over 7 to 8 iterations, with conditional negation. It suits hardware with adders and no memory, and the BitScrambler is the opposite.
- The C5 Golden program's method of looking up the difference from the previous and current phase. Both phases have to fit one address, so each gets 5 bits, and a 5-bit phase step is 417 kHz at 13.33 MS/s, about 14 IRE.
- The C5 `fm_phase8_hr_live` program's 8-bit subtraction in a counter, which needs `ADDCTI`.
- Moving the lookup to the C5, which has `ADDCTI`: its PARLIO is 8 lanes wide, so it would see only 4 bits each of I and Q.
- Anything after the lookup: the phase difference, the 4-sample sums and sync slicing all need an adder.

What is left is the lookup alone, with a smaller table, leaving the CPU to subtract. The lookup is the one step the CPU cannot vectorize, so with phase arriving in the ring the rest could become PIE vector operations. Three tables fit:

- A single 2048×8 table indexed by 6 bits of I and 5 of Q, writing 8 bits per sample.
- A bipartite pair in 1024×16, packed as the high and low byte of each word the way the C5's `fm.bsasm` shares one LUT between two tables: a coarse phase from the top 5 bits of I and Q, and a correction from the top 3 and low 2 bits of each. The P4 cannot add them, so it would write both bytes and the CPU would add.
- A companded pair, also in 1024×16. A 128-entry table per lane turns each of I and Q into a sign and a 5-bit log-spaced magnitude around its offset, and a 1024-entry table turns the two magnitudes into a 6-bit first-quadrant angle. It takes three lookups per sample, and only the lane entries change with the I/Q offset. This is the one the decoder uses.

Simulated back-porch noise in kHz for each table, against an exact demodulator, from [phase_table_sim.py](../p4usb/tools/phase_table_sim.py). The model is a constant-envelope carrier with an I offset of 20 and noise set to the measured exact figure, and it reproduces the measured 6+6 results above (241 against 185 at RMS 19).

| I/Q RMS | Exact | 6+7 (in use, 16 KB) | 6+5 (2048×8) | Bipartite (1024×16) | Companded (1024×16) |
|---|---|---|---|---|---|
| 20 | 187 | 216 | 303 | 258 | 190 |
| 30 | 175 | 189 | 251 | 198 | 182 |
| 40 | 171 | 181 | 216 | 180 | 179 |
| 60 | 168 | 173 | 190 | 172 | 176 |
| 72 | 167 | 171 | 184 | 171 | 175 |
| 90 | 167 | 170 | 178 | 170 | 174 |

The bipartite and companded pairs both come within 3% of the table in use across the gain control's range (RMS 40-72), and the companded pair is quieter than it below RMS 40, because log spacing keeps the phase error from growing as the signal shrinks. The single 6+5 table is 0.6 to 1.5 dB noisier in that range.

### On the hardware

`iq bs` runs a BitScrambler program between PARLIO RX and its DMA during a capture. `bs_phase_check.bsasm` is the companded lookup writing each sample's phase byte beside the lanes it came from, so the CPU can repeat every lookup.

- The lookup is right. On live I/Q at 13.33 MS/s, all 585,487 consecutive samples of a capture matched the CPU's lookup, at each of three I/Q offsets. That is 40 million instructions a second. With the C5's counter at 40 MS/s (120 million a second), one sample in 585,487 differed.
- It is gapless across EOFs. A 2:1 program (`bs_half.bsasm`) carried the C5's counter with no break through 36 soft-delimiter EOFs at 40 MHz, and the program stays in its run state.
- The first EOF after a start ends its DMA node early, because the EOF passes straight through while a few bytes are still inside the BitScrambler: 9 bytes for a program writing 16 bits a sample, 19 for one writing 32. Nothing is lost, but the node is short, and 9 is odd, so from then on each word's second byte sits on the even addresses. Later EOFs land on node boundaries.
- The decoder reads each node's length from its DMA descriptor, and tells the two bytes apart by a bit the program always sets in one of them.
- The LUT takes writes only while the program is halted. Writes while it runs, or while it is paused, change nothing, and the driver's `bitscrambler_load_lut` on a running program corrupts a few hundred samples (it switches the LUT's width to write) and still changes nothing.
- A halt, rewrite and restart takes about 18,000 CPU cycles (50 µs), but in the running decoder the DMA then raced through nodes about five times faster than real time for several milliseconds, twice. Stopping the PARLIO unit, rewriting, and starting the receive again takes 133 µs and comes back clean.
- PIE takes its integer operands from a0..a5 only, and its 8-bit subtract saturates, so wrapped phase differences are taken in 16-bit lanes.
- A PSRAM `iq` capture's first 64,512 bytes hold the previous capture's data, with or without the BitScrambler. Checks start after them.

### In the decoder

The decoder runs `bs_phase.bsasm` on PARLIO RX.

- The program takes four cycles a sample (53 million instructions a second). Three are lookups: I's lane, Q's lane, then the pair. The fourth is a branch that swaps the two magnitude codes in the odd quadrants, so the table's angle is always measured on from the start of the quadrant and the output byte is the whole phase, 256 to a turn, with no arithmetic.
- It writes 16 bits a sample: the phase, and a byte for the gain control holding I's magnitude code and both clipped flags.
- The CPU's part (`demod.S`) is PIE: split the two bytes, subtract each phase from the one before, and sum each four differences for the sync detector. It matched a C version on every test at each start while both existed.
- On one steady signal, 40 s each way: 253.4 of 262 lines locked at 83.5% of core 1 with the CPU's table, and 252.5 at 40.7% with the BitScrambler, with no vertical corrections against one. Core 0 was unchanged at 76-77%, and sync tip, blanking and RMS read the same.
- `decode` reports the sync tip's noise (its scatter about each line's own mean). The simulation puts the two tables 2% apart there. An offset error adds more: d units off at an RMS of r adds a ripple of d/r times the carrier's frequency, which is largest at the sync tip.
- I/Q power comes from I's magnitude code alone (doubled), and the offset from the balance of the sign bits, since a carrier circling its centre spends half its time on each side. It settled within a unit of the CPU's mean-based estimate.
- A new offset means a new table, which stops the receive for 133 µs. The decoder waits for the field's last 2 lines or its first 4, reads what the DMA wrote meanwhile, then lets the line clock take the next hsync and moves the line count on by the lines that passed. It first waited for lines 8 to 14, but it only checks after working through a batch of nodes, so the gap ran into the top of the picture.
- Each rewrite at first cost about 100 lines, not 3: the next vsync fell outside its window and was ignored, and vertical sync was lost for a few fields. The cause was the PARLIO driver's warning at each start (see [PARLIO RX and DMA on the P4](#parlio-rx-and-dma-on-the-p4)), which is now muted.
- The table is rewritten when the offset, smoothed over about 4 s, has moved 1 unit (the CPU's followed to a quarter). One half second's reading wanders a unit or two on a weak signal, and unsmoothed it rewrote the table about once a second there. On a weak signal (220 of 262 lines) that is 6 to 10 rewrites in 40 s.
- On that weak signal, 40 s each way, twice: 217.3 and 217.6 lines locked with the BitScrambler against 216.6 and 214.4 with the CPU's table, the same number of fields, no overruns, and sync tip noise of 994 and 943 kHz against 982 and 1005. With the table 2 units off and unsmoothed, before these fixes, a strong signal's sync tip noise was 122 kHz against the CPU table's 106; that has not been measured again at 1 unit.
- At a start the DMA still names a node of the last receive until the first new one is done, so the decoder read a ring of old data as new. As BitScrambler bytes that was garbage: it sent the offset as far as -29 and stepped the gain down 2, and once left the decoder stopped. Each start now waits for the DMA to move, and clears the gain control's sums.
- The Q lane's sign can stay the same at every word the lane check looks at, which made both bytes look marked and dropped about one node a second until an ambiguous read fell back to the last known order.
- Internal RAM is short: the decoder's ring needs a 96,768-byte block, and 5 KB of new static tables left the largest at 94,208, so the decoder failed to start at boot. The BitScrambler's tables are in PSRAM.

- `table_compare.py` demodulates one raw capture with a model of each table, against an exact demodulator. On a capture through a fade, with a quarter of it clipped: at an amplitude above 64 the CPU table's error was 59 kHz and the BitScrambler's 84; from 32 to 64, 97 and 92; from 16 to 32, 159 and 102. With its offset 1 unit out the BitScrambler's was 94, 127 and 162, and 3 units out 113, 203 and 225. So the table itself is about even, and an offset that lags is what costs.
- The offset moves about half a unit a gain step, so a fade that steps the gain left the table several units out for seconds. The decoder now remembers the offset each gain settled at and rewrites the table straight to it when the gain steps, but only when the table is a unit or more from it.
- On a moving transmitter that scheme rewrote the table 24 times in 32 s. Smoothing over one pass after a gain step swung the offset 3 units either way (4.0 to 7.0 to 1.25 on I at one gain), and three of the rewrites left 23 to 40 lines undrawn. The reading is now always smoothed over eight passes. This has not been judged on a moving transmitter yet.
- The white and black lines seen across the picture with the BitScrambler were mostly not these rewrites but nodes read before the DMA had finished them (see PARLIO RX and DMA).
- The tables themselves are not the difference. At fixed low gains the BitScrambler's gave less sync tip noise than the CPU's (250 against 288 kHz at RMS 12, 403 against 533 at RMS 8, 532 against 686 at RMS 4), and a capture scaled down to RMS 6, 3 and 1.5 with noise added drew alike through both.

- With the CPU's table gone, the first table is built around the mean of the start's raw probe capture, which is taken at the gain the decoder starts with. Starting from zero instead, the sign balance closes an eighth of the error each half second and rewrites the table each time. Three starts from the probe's offset needed no rewrite in the 20 to 30 s after, at 40.8 to 41.0% of core 1.
- By eye on a moving transmitter the BitScrambler's picture was good enough to drop the CPU's table.
- R4 (5769 MHz, reached from Wi-Fi channel 153) decodes as R3 does: 253 to 254 of 262 lines, sync tip near -2.3 MHz and 80 to 90 kHz of sync tip noise at gain 37 to 40.
- An empty channel at gain 77 has an I offset of about 84 to 90, against about 1 at gain 37. Retuning from there to a strong transmitter left the table's centre outside the carrier's ring. Every I then has one sign, I's power reads as the offset (RMS 148 for a true 45), and the gain went to its floor of 30 while the offset walked back 6 units a pass: about 15 s without a picture.
- With every I (or Q) on one side, the table now moves in one pass: by the root of half the power on I, which is the offset when it dwarfs the radius, and by a radius on Q. Meanwhile the RMS is an upper bound, so it may raise the gain but only clipping lowers it. Inside the ring the offset is the sign balance times the radius, and the radius is the root of the power over 1 + 2s², s being the sine the balance gives. The same retune now locks in 2.3 to 3.0 s.
- The sums are cleared after each gain step, so every pass reads one gain. Before, the pass after a step was skipped, and a gain stepping every pass held the offset still.
- Applying every reading over 6 units without smoothing was tried and dropped: with the transmitter off it rewrote the table every pass.

Not done yet:

- Using the free time: about 60% of each core, with core 1 at 40.5% and core 0 at 39%.

What the offload was estimated to buy, before it was measured:

- The demodulator costs about 14 cycles per sample (53% of core 1, about 190M cycles a second). With the lookup in the BitScrambler and the rest in PIE it should cost 2 to 3.
- That frees about 150M cycles a second, 42% of a core, and takes core 1 from 83% to roughly 40%. The measured 40.8% bears this out. Spread over the 20.7M pixels output each second (720×480 at 59.94 fields), it is about 7 cycles a pixel, which is what drawing and the median deinterlace cost now (7.4).

Estimated cost of further picture processing, as a share of one core:

| Work | Pixels a second | Scalar | PIE |
|---|---|---|---|
| Luma 1 2 1 low-pass | 20.7M | 11% | 3-6% |
| Temporal noise reduction, blending with the previous frame | 20.7M | 17-23% | 6% |
| Motion-adaptive deinterlacing of the missing rows | 10.4M | 15-25% | under 10% |
| Motion-adaptive deinterlacing of every pixel | 20.7M | 30-45% | 10-15% |

Simple smoothing in PIE fits the headroom there is without the offload (about 22% of core 0 and 17% of core 1). Scalar per-pixel motion-adaptive deinterlacing does not, and does with it.

Motion-adaptive deinterlacing of the missing rows was tried and dropped. It compared each 8-pixel block's mean luma with the same field two back and, where nothing moved, took the previous field's pixels outright. Split-screen frames against the median showed no difference, since the median already weaves still detail that appears in both fields, and detail one row thin flickers when woven. It took core 0 from 43% to 61%, with the block means in PSRAM because internal RAM had no 43 KB free. Against a low-detail background moving fast it also left blocks dozens of pixels wide, since a block's mean barely changes there and stale rows were woven in.

Temporal noise reduction (`tnr.S`) moves each byte of a drawn row, luma and chroma, halfway towards the same field two back, which is what the frame being drawn over still holds, so the filter recurses on real rows only. It applies in full under 8 levels of difference and fades out by 16, per pixel. On a strong signal a split-screen recording measured 1.7 dB less field-to-field noise in luma (1.8 to 1.5 levels rms), 1.5 dB in U and 2.3 dB in V, well short of the 7.8 dB the filter gives white noise, so JPEG's own noise probably sets the floor there. Fast motion showed no trails. It costs 9% of core 0 (41% to 50%), most of it fetching twice the rows from PSRAM. `decode tnr 0` turns it off and a negative level filters the left half only.

The threshold now follows the noise: it is the sync tip's scatter per sample, in picture levels, held between 4 and 32. A fixed 8 suited the strong signal it was measured on (110 kHz rms of scatter is 8.2 levels) and would read a weaker signal's noise as motion. With the scatter near 200 kHz (15 levels), frames at quality 70 were about 64 KB with the filter off, 57 KB at the fixed 8 and 51 KB following the noise; frame size is a proxy for noise here. Trails on a weak signal have not been judged by eye. `decode tnr auto|LEVELS` chooses.

## Encoding and USB

- The hardware JPEG encoder takes 2.5-3.5 ms for a 720×480 grayscale frame. At quality 80 grainy fields overflowed the 128 KB slot; quality 70 gives about 60-70 KB.
- With the luma low-pass and temporal noise reduction, a strong signal's frames are about 55 KB at quality 70, 62 KB at 80 and 93 KB at 90, and overflow at 95, so the ceiling is now 90. Quality drops a step when a frame passes 7/8 of the slot, before one overflows and is lost, and climbs back while frames stay under half of it. `video quality N` sets the ceiling.
- With no signal, every field of noise overflowed the 128 KB slot at quality 70, so no frame reached the camera and it froze. At quality 20 noise fits in about 113 KB, so quality now drops 10 per overflow and climbs back after 30 frames under 60% of the slot. A third was too low: normal video at quality 40 is about 45 KB, so quality stuck there after a loss and the picture stayed blocky.
- On noise the burst's random vector adds up past the color threshold and the histogram's levels drift far from any real signal's, which drew saturated rainbow speckle. Color now needs locked lines, and the picture keeps the last locked levels, so a lost signal shows as gray snow. Noise locks no lines at all, while a transmitter being moved around often dips to 130-190 of 262 with a strong burst, so a 200-line gate dropped color needlessly. Color now comes on at 150 locked lines and stays on down to 60.
- With no signal the gain control climbed to its maximum (then 80, since capped at 77), and a returning signal then clips completely and doesn't lock until the gain is back near 50. Stepping down 8 at a time while more than 10% of samples clip cut that from about 8 s to 2 s.
- An esptool that gives up on the bridge leaves a queue of SYNCs in the console's input. Each one started a bridge session, waited 2 s for the C5 and started the decoder again, which kept the console from answering for minutes. The bridge now drops waiting input when a session ends. Esptool gives up when it starts while the main task is still in the decoder's start, which takes several seconds at boot.
- On macOS, open the P4 console (303a:8000) with RTS off and DTR on: RTS high with DTR low looks like esptool's reset, and the P4 only writes to a port with DTR set.
- TinyUSB's video class reopens its bulk endpoint on every SET_INTERFACE, including each time a host opens or closes the stream, and its DWC2 port has no endpoint close. Each reopen took another 128-word TX FIFO from the P4's 1024-word FIFO RAM (the video FIFO moved from 0x2EE to 0x26E by enumeration, then 0x1EE and 0x16E) until the allocation check refused more. Twice, after a host had opened and closed the stream, the USB console still received commands but its replies stuck in a full send buffer; the link to the leak is likely but unproven. Wrapping `dcd_edpt_open` to reactivate an already active IN endpoint in place, as TinyUSB does for isochronous ones, kept the FIFO fixed through 20 stream starts with the console polled twice a second, and no replies went missing.
- Live video reached the host at about 38 fps while the encoder ran at 59.9: the P4 skipped 5726 of 10500 frames. TinyUSB's video class sends each UVC payload as its own transfer, sized up to `CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE`, which was 512 bytes, so a 64 KB frame took about 128 round trips through the USB task and didn't fit in 16.7 ms. With 16 KB payloads (about 4 per frame) the P4 sent 4234 frames in 70 s with none skipped, and AVFoundation received 4231. The endpoint's wMaxPacketSize has to stay 512: the class's descriptor macro took the buffer size at first, and macOS then refused the whole configuration. The class sends no zero-length packet, so a frame whose last payload fills whole packets gets one padding byte after EOI.
