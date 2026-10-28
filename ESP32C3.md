# Baram Warp 4 on ESP32-C3 (USB display)

This adds a small bare-metal ESP32-C3 entry point that uses the existing `Warp4Engine`,
reads a two-axis analog joystick with a push button, and streams the rendered
135x240 UI to a PC over the ESP32-C3's native USB Serial/JTAG connection. The
original ST7789 LCD output was removed because the panel was damaged; the PC
viewer replaces it.

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
cargo build --release -p baram-warp4 --example esp32c3 --target riscv32imc-unknown-none-elf --no-default-features --features esp32c3,bdf
espflash flash --before usb-reset target/riscv32imc-unknown-none-elf/release/examples/esp32c3
```

`--before usb-reset` matters: while the firmware is streaming frames the normal
DTR/RTS reset sequence cannot always enter the ROM download mode.

The example config selects BDF fonts instead of TTF and allocates a 256 KiB heap.
The logical framebuffer is 135x240 (129,600 bytes in the renderer's 32-bit pixel
format) and is streamed to the host; no panel driver or RGB565 framebuffer is kept.

## Frame stream protocol

Frames are pushed over USB Serial/JTAG with a little-endian header:

```
magic "ZWF2" | width u16 | height u16 | flags u16 | seq u32 |
payload_len u32 | crc32 u32
```

* `flags` bit0: payload is RLE with entries `(run_count u8, color_hi u8, color_lo u8)`.
  Otherwise the payload is raw `width*height` RGB565 samples (big-endian).
* `crc32` is the standard CRC-32 of the payload bytes as transmitted.
* RLE is chosen automatically per frame when it is smaller than raw; typical UI
  frames compress to about 5.7 KiB (11x), which keeps ~10 fps on the ~220 KiB/s
  link. Non-blocking writes with a 30 ms stall timeout keep the input loop alive
  when no viewer is attached.
* Frames are sent only when the UI redraws, throttled to at most one per
  `MIN_FRAME_INTERVAL_MS` (66 ms).

`tools/zhiwatch_viewer.py` implements the receiving side (pure Python, tkinter +
pyserial, no other dependencies). `--headless N` validates the stream without a
GUI; `--smoke N` renders a synthetic frame offline.

## Joystick settings

The joystick deadzone, speed, and axis reversal are controlled by
`JOYSTICK_DEADZONE`, `JOYSTICK_FAST_ZONE`, `JOYSTICK_X_REVERSED`, and
`JOYSTICK_Y_REVERSED` near the top of `examples/esp32c3.rs`.

## Status / verification

Verified on real hardware (ESP32-C3 rev v0.4, MAC 50:78:7d:47:56:c0):

* Firmware boots and reports `[ok] First frame streamed to USB`.
* Headless validation: ~115 frames in 12 s, 0 CRC errors, 0 bad headers,
  average payload 5.7 KiB.
* GUI viewer renders the live stream (~8 fps shown, 0 CRC errors).

`espflash flash` still requires `--ignore-app-descriptor`-free builds only
because `esp_bootloader_esp_idf::esp_app_desc!()` is included in the example;
flashing works with plain `espflash flash --before usb-reset`.
