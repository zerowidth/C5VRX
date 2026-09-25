# P4 receiver plan

Plan for a self-contained receiver built from the Waveshare ESP32-C5-Zero and the Waveshare ESP32-P4-Pico. The C5 tunes and exports raw I/Q, and the P4 demodulates, decodes NTSC, and presents the video as a USB webcam (UVC) with a serial control port on the same cable.

This is a separate build from the standalone C5 receiver. The C5-only path, which recovers composite video onto the resistor DAC and never decodes pixels, stays as it is.

Nothing here is built yet. The open questions at the end need answers before some choices are final.

## System overview

```mermaid
flowchart LR
    subgraph stick["Receiver stick enclosure"]
        ant["5.8 GHz antenna"] --> c5["ESP32-C5-Zero<br/>tuner + I/Q export"]
        c5 -- "12 I/Q lanes + 40 MHz clock" --> p4["ESP32-P4-Pico<br/>demod, NTSC decode, JPEG"]
        p4 -- "UART: tuning, status" --> c5
        p4 -- "EN, BOOT" --> c5
        oled["SPI OLED"] --- p4
        buttons["Buttons"] --- p4
        sd["microSD"] --- p4
    end
    p4 -- "High-speed USB: UVC + serial" --> host["Computer"]
```

## Roles

- The C5 tunes the 5.8 GHz front end and drives its MODEM_DIAG I/Q bits onto pads, with a sample clock. It answers tuning and status commands on a UART. Its DAC and PARLIO loopback are not used in this build.
- The P4 samples the I/Q bus with PARLIO RX, demodulates FM, decodes NTSC fields, encodes JPEG, and serves UVC. It owns all settings (band, channel) and pushes them to the C5 at boot. It runs the OLED, buttons, and optional SD recording.
- The computer receives a webcam stream and a serial port. A browser page (Web Serial) controls the receiver and shows signal strength.

## Power

- The P4-Pico is powered through its high-speed USB connector (the bottom 4-pin MX1.25 JST), which feeds its VCC_5V rail directly. The top USB-C also powers it, through a power-path FET, so both can be plugged in at once.
- The C5 takes 5V from the P4's VSYS pin (VCC_5V) and GND. Do not use VBUS, which is the USB-C input before the power-path FET.
- Do not power the C5 from its own USB while it is also fed from VSYS, unless the C5-Zero schematic shows a diode on its VBUS.
- Budget: roughly 100-150 mA for the C5, 200-400 mA for the P4, and about 20 mA for the OLED. This fits the 500 mA of a USB 2.0 port.

## C5-Zero pin plan

The C5 can route any MODEM_DIAG lane to any pad, so lanes are assigned for easy soldering, and firmware maps them.

| Function | C5 pins | Notes |
|---|---|---|
| USB | 13, 14 | Kept for development flashing and console |
| UART to P4 | 11 (TX), 12 (RX) | UART0, so the ROM bootloader can be reached and the P4 can reflash the C5 |
| BOOT | 28 (back pad) | Driven by the P4, open-drain |
| Reset | EN, via the RESET button pad | Driven by the P4, open-drain; EN is probably not on the header |
| Sample clock out | One of 0-10, 23, 24, 25 | PARLIO TX clock output or equivalent |
| I/Q data (12 lanes) | The remaining 12 of 0-10, 23, 24, 25 | Q[9:4] and I[9:4] |
| Spare | One pad | Could make it 7+6 bits, but symmetric is simpler |
| Antenna select | 26 | Internal to the board |

GPIO 2, 7 and 25 are strapping pins. They are safe as data outputs because the P4 inputs are high-impedance while the C5 reads its straps at reset.

8+8 bits does not fit: it needs 16 lanes plus a clock, and only 13 pads remain after the UART and BOOT. Moving control to the C5's USB would free two more, still one short.

## P4-Pico pin plan

The C5 sits past the P4's bottom end (the JST end), so the fast signals use the pins nearest that end, and slow signals use the far end.

| Function | P4 pins | Notes |
|---|---|---|
| I/Q data (12) | Left: 2, 3, 4, 5, 6, 14. Right: 27, 32, 33, 46, 47, 48 | Bottom half of both headers, with GNDs interleaved |
| Sample clock | 26 | Next to a GND |
| C5 EN | 20 | Open-drain |
| C5 BOOT | 21 | Open-drain |
| UART TX to C5 RX (GPIO12) | 22 | |
| UART RX from C5 TX (GPIO11) | 23 | |
| OLED SPI: SCLK, MOSI, CS, DC, RST | 15, 16, 17, 18, 19 | |
| Buttons | 7, 8, 24, 25 | 7/8 are I2C to the codec with 2.2k pull-ups; 24/25 are the unused full-speed USB pair |
| Spare | 28-31, 49-52, 54 | |
| C5 power | VSYS, GND | |

Board facts from the [P4-Pico schematic](https://files.waveshare.com/wiki/ESP32-P4-Pico/ESP32-P4-Pico-datasheet.pdf):

- The top USB-C goes only to a CH343 USB-UART (P4 UART0 on GPIO 37/38, with auto-reset). It is for flashing and the console, not UVC.
- High-speed USB goes only to the bottom MX1.25 connector (V, D-, D+, G).
- The header pin labeled EN is the 3.3 V regulator enable, and RUN resets the P4. Neither is used here.
- The audio codec (I2S on GPIO 9-13), speaker amp (GPIO 53), SD card (GPIO 39-45) and flash are on pins not used by this plan.
- The back through-hole pads are GPIO 28-31, 34, 36 and 49-52. GPIO 34 and 36 are strapping pins; leave them alone.

## Physical build

### Stick layout

The enclosure is a long stick: antenna at the front, then the C5, then the P4, with the connectors at the rear.

- Keep the last 2-3 cm around the antenna free of wires, screws and metal.
- The P4's switching regulator sits at its USB-C end, which is the rear, away from the antenna.
- If the external dipole is used, mount an SMA bulkhead at the tip with a short U.FL pigtail.
- The OLED and buttons can go anywhere along the stick.
- Hold the boards with printed standoffs and clips or double-sided tape, and anchor the wire bundles so flexing doesn't land on solder joints.

### C5 to P4 wiring

- Use 30 AWG (Kynar wire-wrap wire works well) for signals, up to 5 cm. At that length the wires behave as plain wires, and length matching doesn't matter (2 cm is about 0.1 ns against a 25 ns sample period).
- Use 28 AWG, or two 30 AWG in parallel, for 5V and the main GND.
- Run 3 or 4 extra ground wires between the boards, each connected at both ends and spread through the data bundle, with one next to the clock. A ground connected at one end only carries no return current and does nothing useful.
- The C5-Zero has one GND pin. Tie several P4 grounds to it. If the C5's USB-C shell measures as grounded, also solder a ground wire there, to put grounds at both ends of the C5.
- Keep each bundle together with its grounds alongside. What matters is the loop area between each signal and its return.
- Firmware sets low drive strength on the C5 data and clock pins to slow the edges. If the link still shows errors, add 22-33 Ω series resistors at the C5 end.

### USB and panel connector

- Wire the P4's MX1.25 JST to a panel-mount USB-C receptacle in the rear of the enclosure.
- The receptacle needs 5.1 kΩ from each CC pin to GND, or a USB-C to USB-C cable will not supply power.
- Twist D+ and D- together and keep them under about 10 cm. This is the only 480 Mbit/s link in the build.
- The P4's own USB-C stays reachable at the rear for flashing, so the rear has two ports: video and power, and programming.

## Firmware

### C5

- Route Q[9:4] and I[9:4] from MODEM_DIAG to the 12 data pads, and output a 40 MHz sample clock.
- Set low drive strength on the data and clock pads.
- Add a text command set on the USB console first, then the same parser on the UART to the P4: band, channel, frequency, status.
- Status includes the existing `strength` score, wideband RSSI (`phy_get_rssi`), noise floor, and current gain, all already in `C5VRX_LAB_ROW`.
- Drop the bench-only commits (forced R3, U.FL antenna) once tuning is controllable.
- A test mode that drives a counter pattern on the data lanes, for checking the link.

### P4

- Sample the 12 lanes plus clock with PARLIO RX, choosing the clock edge that samples mid-bit.
- Demodulate FM from I/Q. With 6+6 bits, a lookup table is too large, so use a cross-product discriminator on the CPU (SIMD). That leaves roughly 20 cycles per sample per core at 400 MHz, which is tight.
- Decode NTSC fields: adaptive sync threshold, per-line resampling, burst-locked color, three-line comb.
- Encode JPEG with the hardware encoder and serve a composite USB device: UVC plus CDC-ACM serial (TinyUSB, Espressif's `usb_device_uvc`).
- Put per-frame metadata in a JPEG COM segment written after SOI: frame counter, field-sync timestamp (esp_timer µs), C5 signal level, gain changes tagged with IQ sample index, and sync and noise quality. The metadata survives only if the host keeps the compressed MJPEG (libuvc, or ffmpeg with `-c:v copy`); OS webcam APIs that hand over decoded frames drop it. UVC payload headers also carry a device-clock PTS per frame, as a cross-check.
- Offer 720×480 at 60 fps (one frame per field, deinterlaced) and at 30 fps (fields woven). 60 is the default for FPV. 720×480 at 60 fps is about 20 megapixels per second, within the JPEG encoder's limit.
- Drive the OLED (128×64 SSD1306 or SH1106: band, channel, frequency, strength bar) and the buttons (band and channel).
- Control the C5's EN and BOOT pins for reset and reflashing it over UART0 (esp-serial-flasher).
- Optional standalone recording of MJPEG to microSD (roughly 3-4 MB/s).

### Control protocol

Plain text lines over the USB serial port, so they can be typed by hand, parsed in a browser, and logged. Buttons and serial commands go through the same handler, so the OLED and the browser always agree.

```text
> band R
> ch 3            (or: freq 5732)
< ok band=R ch=3 freq=5732
> status
< status band=R ch=3 freq=5732 strength=87 rssi=-48 gain=34 sync=1 fields=59.9
< event status ...          (pushed a few times a second when subscribed)
```

### Browser page

A Web Serial page with band and channel buttons, the current frequency, and a live strength graph. It first talks to the C5's own USB console, then unchanged to the P4 once it relays the same protocol.

## Open questions

- Are the lower diag bits (Q[5:4] and I[5:4]) live and clocked with the upper bits in continuous receive? The 10-bit layout is proven only for the dump path. Test on the C5 alone by routing them onto the current loopback pads and comparing decoded video.
- Does the camera capture 60 images per second, or 30 split across fields? Capture a moving scene and compare the two fields of one frame.
- Can the P4 demodulate 6+6-bit I/Q at 40 MS/s with time left for decoding and encoding?
- What is the maximum PARLIO RX external clock on the P4, and which clock edge gives margin?
- Are C5 GPIO 11/12 the ROM bootloader's UART0 pins?
- Is the C5's EN reachable only at the RESET button pad? Is its USB-C shell grounded?

## Build order

1. C5 command set on its USB console, and the Web Serial page against the C5 directly.
2. Lower diag bit test on the C5, to settle 6+6 against 4+4.
3. When the P4 arrives: power, UART, EN and BOOT, OLED and buttons. Channel control and status work end to end, with no video.
4. C5 clock output and I/Q export. Capture on the P4, check the link with the counter pattern, and compare samples with a host-side decode offline.
5. Demodulation and NTSC decode on the P4, then JPEG and UVC with the serial port alongside.
6. Optional: microSD recording.
