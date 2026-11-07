# Baram Warp 4 on ESP32-C3 (USB display)

This adds a small bare-metal ESP32-C3 entry point that uses the existing `Warp4Engine`,
reads a two-axis analog joystick with a push button, and streams the rendered
135x240 UI to a PC over the ESP32-C3's native USB Serial/JTAG connection. The PC
viewer can also send a virtual joystick back over the same link. The original
ST7789 LCD output was removed because the panel was damaged; the PC viewer
replaces it.

## Required workspace

This repository is self-contained. The shared `baram-core`, `baram-bsd`, `baram-font`, `baram-graphics`, and `stb_truetype_rust` crates live under `crates/`, and the root `Cargo.toml` acts as the workspace manifest (including `[workspace.dependencies]`).

The firmware example is written for the `esp-hal` 1.2 API. Its ADC is enabled via `esp-hal/unstable`. The embedded ports in `crates/` were wired for ESP32-C3: `parallel.rs` falls back to a serial dispatcher, the VFS has no block storage (apps ship as embedded archive members), and `baram-bsd`'s config revision counter uses atomic load/store instead of CAS (riscv32imc has no atomic CAS).

## Wiring

All signals are 3.3 V logic. The LCD wiring is obsolete (panel damaged and code removed).

| ESP32-C3 GPIO | Signal |
|---|---|
| GPIO0 | Joystick VRx / X analog output |
| GPIO1 | Joystick VRy / Y analog output |
| GPIO20 | Joystick SW / push button, active low |
| 3V3 | Joystick VCC (subject to the module's specified voltage) |
| GND | All grounds |
| USB (GPIO18/19) | Native USB Serial/JTAG — frame stream + logs to the PC |

GPIO12–GPIO17 (module flash), GPIO18–GPIO19 (USB), GPIO11 (VDD_SPI) and strapping
pins GPIO2/8/9 are otherwise unassigned. GPIO20 is repurposed from its usual UART
function; do not use UART0 on GPIO20 while the joystick button is connected.

## Build and flash

```sh
./buildc3.sh          # release build for riscv32imc-unknown-none-elf
./buildc3.sh flash    # build + flash (uses --before usb-reset)
./buildc3.sh viewer   # open the PC viewer (bootstraps a venv with pyserial)
```

The script is equivalent to:

```sh
rustup target add riscv32imc-unknown-none-elf
cargo build --release -p baram-warp4 --example esp32c3 --target riscv32imc-unknown-none-elf --no-default-features --features esp32c3,ttf
espflash flash --before usb-reset target/riscv32imc-unknown-none-elf/release/examples/esp32c3
```

`--before usb-reset` matters: while the firmware is streaming frames the normal
DTR/RTS reset sequence cannot always enter the ROM download mode.

The example config allocates a 256 KiB heap. The UI font is `files/data/ui/HMOS_SANS_SUBSET.ttf`,
embedded in flash and copied into RAM at boot, then rasterized with 3-bit anti-aliasing.

## Rendering pipeline and RAM

The screen is cached as two planes, so cursor motion does not repaint the UI:

* **Shape plane**: 135x240 8-bit RGB332, 32,400 bytes. Everything except text.
* **Text plane**: 135x240 3-bit coverage, bit-packed, 12,150 bytes. Glyph color is not
  stored per pixel; each text draw call is recorded as a run (bounding box + RGB332
  color), at most 160 per frame.
* **Band scratch**: 8 rows of 32-bit pixels, 4,320 bytes, used only while rebuilding.
* **Font**: the TTF is copied to RAM, about 9.7 KB.

The planes are rebuilt only when the UI is dirty (`engine.is_dirty()`). When a frame
is transmitted, the shape, text and cursor are composed per pixel: text is blended
over the shape with its coverage, and the cursor is drawn on top. Nothing is
composed into a third buffer. A later opaque shape write removes the text beneath
it (z-order is kept); translucent writes (shadows, global alpha) leave text visible.

Approximate RAM: 32.4 KB shape + 12.2 KB text + 9.7 KB font + 4.3 KB band + glyph
caches. This is well under the 100 KB budget.

## Frame stream protocol

Frames are pushed over USB Serial/JTAG with a little-endian header:

```
magic "ZWF2" | width u16 | height u16 | flags u16 | seq u32 |
payload_len u32 | crc32 u32
```

* `flags` bit0: payload is RLE with entries `(run_count u8, sample u8)`, run_count
  1..255. This is the only compression on the wire, and it is lossless. The payload
  is always RLE.
* `flags` bit1: Chroma subcarrier modulation mode (Y luminance base with chroma C modulated on subcarrier).
* `flags` bit2: 8-bit color mode (RGB332).
* `crc32` is the standard CRC-32 of the payload bytes as transmitted.
* The frame is sent when the UI redraws or the cursor moves, throttled to at most one
  per `MIN_FRAME_INTERVAL_MS` (66 ms). The UI is only repainted when it is dirty.
* The RLE payload is produced twice per frame: once to get the length and CRC, once
  to send. No frame buffer is needed for either pass.

`tools/zhiwatch_viewer.py` implements the receiving side (pure Python, tkinter +
pyserial, no other dependencies). Press **C** or click the **CHROMA CARRIER** button
in the GUI to toggle the Y + chroma subcarrier mode dynamically.

## Viewer controls

The side panel shows **RAM** (heap) and **FLASH** usage plus device **uptime**,
updated from the telemetry packet described below. **J** or the **PHYSICAL
STICK** button disables/enables the board's real joystick axes and push button;
the virtual stick and Space keep working while it is disabled. **R** or the
**RESTART** button asks for confirmation and reboots the board. The viewer then
reconnects automatically when the USB device re-enumerates.

## Virtual joystick (host input)

The GUI shows a virtual joystick next to the streamed frame. Drag the pad with
the mouse (or a touchscreen that emits normal pointer events), or use the arrow
keys; **Space** presses the joystick button. **C** toggles Chroma Carrier mode,
**J** toggles the physical stick, **R** restarts the device, **P** pauses the
display and **Q**/Escape quits. The virtual stick overrides the physical stick
while it is deflected, and the virtual button and physical button are OR-ed
together (as long as the physical stick is enabled).

The viewer sends this packet over the same USB serial connection:

```
magic "ZWI1" | version u8 | flags u8 | x i8 | y i8 | buttons u8
```

`flags` bit1 enables/disables Chroma subcarrier mode on the device.

`x`/`y` are `-127..127` screen-space axes (right/down positive) and `buttons`
bit0 is the push button. The viewer sends a heartbeat every 100 ms while the
stick is deflected; firmware drops host input after 500 ms without a packet, so
a closed viewer cannot leave the pointer stuck. Input is only applied when the
host sends it, so using the viewer without the virtual stick behaves exactly as
before.

## Commands and telemetry

Viewer actions use a separate one-shot command packet:

```
magic "ZWC1" | version u8 | command u8 | arg u8
```

* `command` 1: reboot the device (`arg` ignored; firmware waits 20 ms for USB
  to drain, then calls `software_reset`).
* `command` 2: enable/disable the physical joystick (`arg` 0 = disabled,
  non-zero = enabled). While disabled, the firmware skips both ADC reads and the
  physical button; the virtual stick and button are unaffected.
* `command` 3: send telemetry immediately.

The device replies with:

```
magic "ZWT1" | version u8 | flags u8 | heap_used u32 | heap_total u32 |
flash_used u32 | flash_total u32 | uptime_ms u32
```

`flags` bit0 means the physical joystick is disabled. `heap_used`/`heap_total`
come from the `esp-alloc` heap. `flash_used` is the app image size computed from
the image header at offset `0x10000`; `flash_total` is the flash chip capacity.
Telemetry is sent immediately after a telemetry request or physical-stick change,
and every second while frames are streaming. `tools/zhiwatch_viewer.py --headless`
prints the same values.

## Joystick settings

The firmware measures each axis' resting ADC value at boot and uses that as its
center, because the ESP32-C3 ADC range and the joystick's mechanical center are
not guaranteed to be exactly mid-scale. Keep the stick centered while the board
powers up; the calibrated values are printed as `[input] joystick center x=...
y=...`. The center then follows slow drift while the stick is at rest, but is
left fixed while the stick is deflected.

The joystick deadzone, speed, and axis reversal are controlled by
`JOYSTICK_DEADZONE`, `JOYSTICK_FAST_ZONE`, `JOYSTICK_X_REVERSED`, and
`JOYSTICK_Y_REVERSED` near the top of `examples/esp32c3.rs`. Deadzone and fast
zone are offsets from the calibrated center, not from 2048.

## Status / verification

Verified on real hardware (ESP32-C3 rev v0.4, MAC 50:78:7d:47:56:c0):

* Firmware boots and reports `[ok] First frame streamed to USB`.
* Headless validation: ~115 frames in 12 s, 0 CRC errors, 0 bad headers,
  average payload 5.7 KiB.
* GUI viewer renders the live stream (~8 fps shown, 0 CRC errors).

`espflash flash` still requires `--ignore-app-descriptor`-free builds only
because `esp_bootloader_esp_idf::esp_app_desc!()` is included in the example;
flashing works with plain `espflash flash --before usb-reset`.
