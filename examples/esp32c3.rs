#![no_std]
#![no_main]

extern crate alloc;

use core::panic::PanicInfo;
use baram_core::frame::encode_frame_rle;
use baram_core::text_plane::{rgb332_from_color, TextPlane};
use baram_core::{Color, LayerSystem};
use baram_warp4::{set_ui_mode, set_ui_scale_percent, set_xiao_size, UiMode, Warp4Engine};
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::CpuClock,
    delay::Delay,
    gpio::{Input, InputConfig, Pull},
    time::Instant,
    usb::usb_serial_jtag::UsbSerialJtag,
    Blocking,
};

const LOGICAL_WIDTH: usize = 135;
const LOGICAL_HEIGHT: usize = 240;
// Only this many rows are rasterized at a time. The firmware streams a
// complete frame band by band, so no full-screen RGB buffer is ever resident.
// Eight rows cost 135*8*4 = 4.2 KiB instead of 126 KiB for a full 32-bit layer.
const BAND_HEIGHT: usize = 8;
// Fallback only; the real centers are measured at boot because the ADC's
// zero/scale and the joystick's mechanical center are not guaranteed to be
// exactly mid-scale on every board.
const JOYSTICK_CENTER: i32 = 2048;
const JOYSTICK_DEADZONE: i32 = 300;
const JOYSTICK_FAST_ZONE: i32 = 1200;
const JOYSTICK_CAL_SAMPLES: usize = 16;
const JOYSTICK_CAL_DELAY_MS: u32 = 8;
const JOYSTICK_RECENTER_BAND: i32 = 80;
const JOYSTICK_RECENTER_SHIFT: i32 = 4;
const LOOP_DELAY_MS: u32 = 20;
const FRAME_TIME_NS: u64 = LOOP_DELAY_MS as u64 * 1_000_000;
// USB full-speed CDC sustains well under 1.5 MB/s; with RLE payloads around
// 6 KiB this cap allows roughly 10-15 frames per second.
const MIN_FRAME_INTERVAL_MS: u64 = 66;

// Frame stream protocol (little-endian header, 8-bit samples):
// magic "ZWF2" | width u16 | height u16 | flags u16 | seq u32 | payload_len u32 | crc32 u32
// flags bit0: payload is RLE (the only compression used; lossless)
// flags bit1: payload is chroma carrier modulated (Y luminance base + C subcarrier)
// flags bit2: payload is 8-bit color (RGB332)
// The CRC covers the payload bytes as transmitted (the RLE stream).
const FRAME_MAGIC: [u8; 4] = *b"ZWF2";
const FRAME_HEADER_LEN: usize = 22;
const FRAME_FLAG_RLE: u16 = 1;
const FRAME_FLAG_CHROMA_CARRIER: u16 = 2;
const FRAME_FLAG_8BIT: u16 = 4;
// The UI font: the HMOS subset is copied to RAM at boot (see main).
const FONT_BYTES: &[u8] = include_bytes!("../files/data/ui/HMOS_SANS_SUBSET.ttf");
// The stick moves keyboard focus like arrow keys. Holding a direction repeats
// after the first delay, then at the repeat rate.
const NAV_FIRST_REPEAT_MS: u64 = 380;
const NAV_REPEAT_MS: u64 = 170;
// Coordinates that keep the frame cursor off-screen. The cursor is no longer
// used for pointing; focus is.
const NO_CURSOR: i32 = -1000;
const PERF_REPORT_MS: u64 = 2000;
// Give up on a frame if the host stops draining the USB FIFO for this long,
// so the input loop keeps running when no viewer is attached.
const USB_STALL_TIMEOUT_MS: u64 = 30;

// Host -> device virtual joystick protocol (see tools/zhiwatch_viewer.py):
// magic "ZWI1" | version u8 | flags u8 | x i8 | y i8 | buttons u8
// x/y are -127..127 and buttons bit0 is the push button. The viewer sends a
// heartbeat while the stick is deflected; drop the state if it stops.
const INPUT_MAGIC: [u8; 4] = *b"ZWI1";
const INPUT_VERSION: u8 = 1;
const INPUT_PACKET_LEN: usize = 9;
const INPUT_BUTTON: u8 = 1;
const INPUT_FLAG_CHROMA_CARRIER: u8 = 2;
const VIRTUAL_INPUT_TIMEOUT_MS: u64 = 500;
// Match the physical stick's deadzone/fast-zone ratios (about 25% and 66%).
const VIRTUAL_DEADZONE: i32 = 24;
const VIRTUAL_FAST_ZONE: i32 = 84;

// Host -> device one-shot commands (viewer buttons / keys):
// magic "ZWC1" | version u8 | command u8 | arg u8
const COMMAND_MAGIC: [u8; 4] = *b"ZWC1";
const COMMAND_VERSION: u8 = 1;
const COMMAND_PACKET_LEN: usize = 7;
const COMMAND_REBOOT: u8 = 1;
const COMMAND_SET_PHYSICAL_JOYSTICK: u8 = 2;
const COMMAND_REQUEST_TELEMETRY: u8 = 3;

// Device -> host telemetry, sent once a second while the stream is active:
// magic "ZWT1" | version u8 | flags u8 | heap_used u32 | heap_total u32 |
// flash_used u32 | flash_total u32 | uptime_ms u32
const TELEMETRY_MAGIC: [u8; 4] = *b"ZWT1";
const TELEMETRY_VERSION: u8 = 1;
const TELEMETRY_PACKET_LEN: usize = 26;
const TELEMETRY_FLAG_PHYSICAL_JOYSTICK_DISABLED: u8 = 1;
const TELEMETRY_INTERVAL_MS: u64 = 1000;

// The app is flashed by espflash at the start of the factory partition. The
// image header tells us how many bytes of that partition are actually used.
const APP_IMAGE_OFFSET: u32 = 0x10_000;
const APP_PARTITION_SIZE_FALLBACK: u32 = 0x30_0000;
const APP_IMAGE_MAGIC: u8 = 0xE9;

esp_bootloader_esp_idf::esp_app_desc!();

// Set either to true if the physical joystick axis is reversed on your module.
const JOYSTICK_X_REVERSED: bool = false;
const JOYSTICK_Y_REVERSED: bool = false;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    esp_println::println!("PANIC: {info}");
    loop {
        core::hint::spin_loop();
    }
}

#[esp_hal::main]
fn main() -> ! {
    esp_println::println!("[boot] Baram Warp 4 / ESP32-C3 starting");
    esp_println::println!(
        "[config] frame={}x{} (streamed over USB Serial/JTAG)",
        LOGICAL_WIDTH,
        LOGICAL_HEIGHT
    );
    esp_println::println!("[pins] Joystick X=GPIO0, Y=GPIO1, SW=GPIO20");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The band renderer only needs a few KiB of pixels; the heap size is kept
    // generous so documents can grow without a full-screen layer. Steady-state
    // usage after the band conversion is a small fraction of this region.
    esp_alloc::heap_allocator!(size: 256 * 1024);
    esp_println::println!("[ok] Heap allocator initialized (256 KiB)");

    // The UI face is copied into RAM (heap); the parser and rasterizer never
    // read flash after this point.
    baram_font::ttf_font::init_with_data(FONT_BYTES);
    esp_println::println!("[ok] Font HMOS_SANS_SUBSET loaded to RAM ({} bytes)", FONT_BYTES.len());

    // Measure the flashed app image once at boot. The viewer displays this as
    // flash usage; failures fall back to "unknown" instead of blocking boot.
    let mut flash = esp_bootloader_esp_idf::partitions::FlashStorage::new(peripherals.FLASH);
    let flash_total = flash.capacity() as u32;
    let flash_used = read_app_image_size(
        &mut flash,
        APP_IMAGE_OFFSET,
        APP_PARTITION_SIZE_FALLBACK,
    )
    .unwrap_or(0);
    esp_println::println!(
        "[ok] Flash usage {}/{} bytes",
        flash_used,
        flash_total
    );

    let delay = Delay::new();
    let mut usb = UsbSerialJtag::new(peripherals.USB_DEVICE);

    let mut adc_config = AdcConfig::new();
    let mut joystick_x_pin = adc_config.enable_pin(peripherals.GPIO0, Attenuation::_11dB);
    let mut joystick_y_pin = adc_config.enable_pin(peripherals.GPIO1, Attenuation::_11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);
    let joystick_button = Input::new(
        peripherals.GPIO20,
        InputConfig::default().with_pull(Pull::Up),
    );

    // Calibrate the resting position instead of assuming exactly mid-scale.
    // The ESP32-C3 ADC's usable range and the module's mechanical center can
    // both be offset, which otherwise makes a resting stick look permanently
    // deflected (the reported "always drifts right" symptom).
    let mut x_samples = [0_i32; JOYSTICK_CAL_SAMPLES];
    let mut y_samples = [0_i32; JOYSTICK_CAL_SAMPLES];
    for sample in 0..JOYSTICK_CAL_SAMPLES {
        x_samples[sample] = match nb::block!(adc.read_oneshot(&mut joystick_x_pin)) {
            Ok(value) => value as i32,
            Err(_) => JOYSTICK_CENTER,
        };
        y_samples[sample] = match nb::block!(adc.read_oneshot(&mut joystick_y_pin)) {
            Ok(value) => value as i32,
            Err(_) => JOYSTICK_CENTER,
        };
        delay.delay_millis(JOYSTICK_CAL_DELAY_MS);
    }
    let mut joystick_center_x = median(&mut x_samples).clamp(256, 3840);
    let mut joystick_center_y = median(&mut y_samples).clamp(256, 3840);
    esp_println::println!(
        "[input] joystick center x={} y={} (raw x {}..{}, y {}..{})",
        joystick_center_x,
        joystick_center_y,
        x_samples[0],
        x_samples[JOYSTICK_CAL_SAMPLES - 1],
        y_samples[0],
        y_samples[JOYSTICK_CAL_SAMPLES - 1]
    );
    esp_println::println!("[ok] Joystick ADC and push button initialized");

    baram_bsd::config::init_config_minimal();
    esp_println::println!("[ok] Config initialized (defaults only, no XML tree)");

    set_ui_mode(UiMode::Normal);
    set_ui_scale_percent(100);
    set_xiao_size(true);
    let mut engine = Warp4Engine::new_embedded(
        "esp32c3-demo",
        &[
            ("config.ini", include_str!("esp32c3/config.ini")),
            ("main.w4u", include_str!("esp32c3/main.w4u")),
            ("main.w4s", include_str!("esp32c3/main.w4s")),
        ],
    );
    engine.set_chrome_visible(false);
    engine.update(LOGICAL_WIDTH as i32, LOGICAL_HEIGHT as i32);

    // Shape plane: every pixel except text, 8-bit RGB332, cached until the UI
    // changes. Text lives in the band layer's text plane (3-bit coverage).
    let mut band = LayerSystem::new(LOGICAL_WIDTH, BAND_HEIGHT);
    band.enable_text_plane(LOGICAL_WIDTH, LOGICAL_HEIGHT);
    let mut shape = alloc::vec![0u8; LOGICAL_WIDTH * LOGICAL_HEIGHT];
    let cache_bytes = shape.len()
        + band.text_plane().map(TextPlane::ram_bytes).unwrap_or(0);
    esp_println::println!("[ok] Frame cache: shape {} B + text plane {} B", shape.len(), cache_bytes - shape.len());
    let cursor_x = NO_CURSOR;
    let cursor_y = NO_CURSOR;
    let mut chroma_carrier_mode = false;

    let mut seq = 0_u32;
    let mut last_frame_sent: Option<Instant> = None;
    let mut pending_send = true;
    let mut cache_valid = false;
    let mut stream_active = false;

    let mut now_ns = 0_u64;
    let boot_instant = Instant::now();
    let mut previous_button_down = false;
    let mut nav_held = (0_i32, 0_i32);
    let mut nav_next_ms: Option<u64> = None;
    // Rows of the screen that the last focus move changed, and whether the
    // whole frame must be rebuilt instead.
    let mut focus_damage: Option<(i32, i32)> = None;
    let mut full_redraw = false;
    let mut perf = Perf::default();
    let mut last_perf_ms = 0_u64;
    let mut x_adc_fault = false;
    let mut y_adc_fault = false;
    let mut input_parser = HostParser::new();
    let mut virtual_x = 0_i8;
    let mut virtual_y = 0_i8;
    let mut virtual_button = false;
    let mut virtual_last_rx: Option<Instant> = None;
    let mut physical_joystick_enabled = true;
    let mut last_telemetry: Option<Instant> = None;
    let mut telemetry_pending = false;

    loop {
        // Drain host input every iteration; the viewer drives the same cursor
        // as the physical stick and sends a heartbeat while it is held.
        while let Ok(byte) = usb.read_byte() {
            match input_parser.push(byte) {
                Some(HostPacket::Input { x, y, button, flags }) => {
                    virtual_x = x;
                    virtual_y = y;
                    virtual_button = button;
                    virtual_last_rx = Some(Instant::now());
                    let host_carrier = flags & INPUT_FLAG_CHROMA_CARRIER != 0;
                    if host_carrier != chroma_carrier_mode {
                        chroma_carrier_mode = host_carrier;
                        esp_println::println!(
                            "[video] Chroma subcarrier mode {}",
                            if chroma_carrier_mode { "ENABLED" } else { "DISABLED" }
                        );
                        // The next frame re-renders every band with the new
                        // sample conversion; there is no cached framebuffer.
                        pending_send = true;
                    }
                }
                Some(HostPacket::Command { command, arg }) => match command {
                    COMMAND_REBOOT => {
                        esp_println::println!("[host] Reboot requested by viewer");
                        // Give the USB FIFO a moment to drain before resetting.
                        delay.delay_millis(20);
                        esp_hal::system::software_reset();
                    }
                    COMMAND_SET_PHYSICAL_JOYSTICK => {
                        let enabled = arg != 0;
                        if enabled != physical_joystick_enabled {
                            physical_joystick_enabled = enabled;
                            esp_println::println!(
                                "[input] Physical joystick {}",
                                if enabled { "ENABLED" } else { "DISABLED" }
                            );
                            telemetry_pending = true;
                        }
                    }
                    COMMAND_REQUEST_TELEMETRY => telemetry_pending = true,
                    _ => {}
                },
                None => {}
            }
        }
        if let Some(last_rx) = virtual_last_rx {
            if last_rx.elapsed().as_millis() >= VIRTUAL_INPUT_TIMEOUT_MS {
                virtual_x = 0;
                virtual_y = 0;
                virtual_button = false;
                virtual_last_rx = None;
            }
        }

        let raw_x = if physical_joystick_enabled {
            match nb::block!(adc.read_oneshot(&mut joystick_x_pin)) {
                Ok(value) => {
                    if x_adc_fault {
                        esp_println::println!("[ok] Joystick X ADC recovered");
                        x_adc_fault = false;
                    }
                    value as i32
                }
                Err(_) => {
                    if !x_adc_fault {
                        esp_println::println!("[error] Joystick X ADC read failed; using center value");
                        x_adc_fault = true;
                    }
                    joystick_center_x
                }
            }
        } else {
            joystick_center_x
        };
        let raw_y = if physical_joystick_enabled {
            match nb::block!(adc.read_oneshot(&mut joystick_y_pin)) {
                Ok(value) => {
                    if y_adc_fault {
                        esp_println::println!("[ok] Joystick Y ADC recovered");
                        y_adc_fault = false;
                    }
                    value as i32
                }
                Err(_) => {
                    if !y_adc_fault {
                        esp_println::println!("[error] Joystick Y ADC read failed; using center value");
                        y_adc_fault = true;
                    }
                    joystick_center_y
                }
            }
        } else {
            joystick_center_y
        };

        // Follow slow electrical/mechanical drift while the stick is at rest.
        // Leave the center alone while it is deflected, otherwise holding a
        // direction would gradually cancel itself out.
        if physical_joystick_enabled && !x_adc_fault {
            joystick_center_x = recenter(joystick_center_x, raw_x);
        }
        if physical_joystick_enabled && !y_adc_fault {
            joystick_center_y = recenter(joystick_center_y, raw_y);
        }

        let mut dx = if physical_joystick_enabled {
            axis_step(raw_x, joystick_center_x)
        } else {
            0
        };
        let mut dy = if physical_joystick_enabled {
            axis_step(raw_y, joystick_center_y)
        } else {
            0
        };
        if JOYSTICK_X_REVERSED { dx = -dx; }
        if JOYSTICK_Y_REVERSED { dy = -dy; }
        // The host already defines screen-space directions, so the physical
        // axis-reversal switches do not apply to the virtual stick.
        let virtual_dx = virtual_axis_step(virtual_x);
        let virtual_dy = virtual_axis_step(virtual_y);
        if virtual_dx != 0 { dx = virtual_dx; }
        if virtual_dy != 0 { dy = virtual_dy; }

        engine.tick(now_ns);

        // The stick moves focus like arrow keys. The dominant axis picks the
        // direction; a new direction moves at once, a held one repeats.
        let (nav_x, nav_y) = dominant_direction(dx, dy);
        let now_ms = boot_instant.elapsed().as_millis() as u64;
        if (nav_x, nav_y) == (0, 0) {
            nav_held = (0, 0);
            nav_next_ms = None;
        } else if (nav_x, nav_y) != nav_held {
            nav_held = (nav_x, nav_y);
            nav_next_ms = Some(now_ms + NAV_FIRST_REPEAT_MS);
            move_focus(&mut engine, nav_x, nav_y, &mut focus_damage, &mut full_redraw);
        } else if nav_next_ms.is_some_and(|due| now_ms >= due) {
            nav_next_ms = Some(now_ms + NAV_REPEAT_MS);
            move_focus(&mut engine, nav_x, nav_y, &mut focus_damage, &mut full_redraw);
        }

        // Button press activates the focused control, like Space. Activation
        // can change any part of the UI, so the whole frame is rebuilt.
        // esp-hal's `Input::is_low()` is an inherent method returning `bool`.
        let button_down = (physical_joystick_enabled && joystick_button.is_low())
            || virtual_button;
        if button_down && !previous_button_down {
            if engine.activate_focused() {
                full_redraw = true;
                esp_println::println!("[input] focused control activated");
            }
        } else if !button_down && previous_button_down {
            engine.release();
        }
        previous_button_down = button_down;

        // The planes are rebuilt only when the UI changed. A focus move repaints
        // only the rows it touched (the old and new control), unless scrolling
        // is involved; then the whole frame is rebuilt.
        if engine.is_dirty() || !cache_valid || full_redraw {
            let started = Instant::now();
            match focus_damage.take() {
                Some((y0, y1)) if cache_valid && !full_redraw && !engine.is_animating() => {
                    rebuild_rows(&mut engine, &mut band, &mut shape, y0, y1);
                    perf.partial_rebuilds += 1;
                }
                _ => {
                    rebuild_cache(&mut engine, &mut band, &mut shape);
                    perf.full_rebuilds += 1;
                }
            }
            perf.rebuild_us += started.elapsed().as_micros() as u64;
            full_redraw = false;
            cache_valid = true;
            pending_send = true;
        }

        // Transmit at most once per MIN_FRAME_INTERVAL_MS.
        if pending_send {
            let started = Instant::now();
            let attempt = try_send(
                &mut usb,
                &shape,
                band.text_plane(),
                cursor_x,
                cursor_y,
                &mut seq,
                &mut last_frame_sent,
                chroma_carrier_mode,
            );
            if let Some(true) = attempt {
                perf.send_us += started.elapsed().as_micros() as u64;
                perf.frames += 1;
            }
            match attempt {
                Some(sent) => {
                    pending_send = !sent;
                    if sent != stream_active {
                        if sent {
                            esp_println::println!("[ok] USB stream active (frame {})", seq);
                        } else {
                            esp_println::println!("[wait] USB host stalled; holding frames");
                        }
                        stream_active = sent;
                    }
                }
                // Throttled: send when the next frame is due.
                None => {}
            }
        }

        let report_now = boot_instant.elapsed().as_millis() as u64;
        if report_now.saturating_sub(last_perf_ms) >= PERF_REPORT_MS {
            let elapsed_ms = (report_now - last_perf_ms).max(1);
            // Frames per second with one decimal place.
            let fps_x10 = perf.frames as u64 * 10_000 / elapsed_ms;
            if perf.frames > 0 || perf.full_rebuilds + perf.partial_rebuilds > 0 {
                esp_println::println!(
                    "[perf] fps {}.{} | rebuild full {} partial {} avg {} us | send avg {} us",
                    fps_x10 / 10,
                    fps_x10 % 10,
                    perf.full_rebuilds,
                    perf.partial_rebuilds,
                    perf.rebuild_us / (perf.full_rebuilds + perf.partial_rebuilds).max(1) as u64,
                    perf.send_us / perf.frames.max(1) as u64,
                );
            }
            perf = Perf::default();
            last_perf_ms = report_now;
        }

        // Report heap/flash usage once a second while frames are flowing, and
        // immediately after the viewer asks for it or toggles the physical
        // joystick. Telemetry is tiny and never blocks frame rendering.
        let telemetry_due = telemetry_pending
            || (stream_active
                && last_telemetry
                    .map(|sent_at| sent_at.elapsed().as_millis() >= TELEMETRY_INTERVAL_MS)
                    .unwrap_or(true));
        if telemetry_due {
            let heap = esp_alloc::HEAP.stats();
            let uptime_ms = boot_instant.elapsed().as_millis() as u32;
            send_telemetry(
                &mut usb,
                heap.current_usage as u32,
                heap.size as u32,
                flash_used,
                flash_total,
                uptime_ms,
                physical_joystick_enabled,
            );
            last_telemetry = Some(Instant::now());
            telemetry_pending = false;
        }

        delay.delay_millis(LOOP_DELAY_MS);
        now_ns = now_ns.saturating_add(FRAME_TIME_NS);
    }
}

/// Throttled frame attempt: `None` while the minimum frame interval has not
/// elapsed, `Some(sent)` once an attempt was actually made.
fn try_send(
    usb: &mut UsbSerialJtag<'_, Blocking>,
    shape: &[u8],
    text: Option<&TextPlane>,
    cursor_x: i32,
    cursor_y: i32,
    seq: &mut u32,
    last_frame_sent: &mut Option<Instant>,
    chroma_carrier: bool,
) -> Option<bool> {
    if let Some(sent_at) = *last_frame_sent {
        if sent_at.elapsed().as_millis() < MIN_FRAME_INTERVAL_MS {
            return None;
        }
    }
    *seq = seq.wrapping_add(1);
    let ok = send_frame(usb, shape, text, cursor_x, cursor_y, *seq, chroma_carrier);
    *last_frame_sent = Some(Instant::now());
    Some(ok)
}

fn axis_step(raw: i32, center: i32) -> i32 {
    let delta = raw - center;
    let magnitude = delta.abs();
    if magnitude < JOYSTICK_DEADZONE {
        0
    } else if magnitude >= JOYSTICK_FAST_ZONE {
        delta.signum() * 3
    } else {
        delta.signum()
    }
}

/// Median of the boot-time samples. The median rejects a single noisy sample
/// (or a brief touch during power-up) better than a plain average.
fn median(samples: &mut [i32]) -> i32 {
    if samples.is_empty() {
        return JOYSTICK_CENTER;
    }
    samples.sort_unstable();
    samples[samples.len() / 2]
}

/// Follow slow center drift only while the stick is within a narrow band
/// around the current center. Moving the stick cancels the update, so holding
/// a direction cannot slowly re-center the stick and reduce its travel.
fn recenter(center: i32, raw: i32) -> i32 {
    let delta = raw - center;
    if delta.abs() <= JOYSTICK_RECENTER_BAND {
        center + delta / JOYSTICK_RECENTER_SHIFT
    } else {
        center
    }
}

fn virtual_axis_step(raw: i8) -> i32 {
    let delta = raw as i32;
    let magnitude = delta.abs();
    if magnitude < VIRTUAL_DEADZONE {
        0
    } else if magnitude >= VIRTUAL_FAST_ZONE {
        delta.signum() * 3
    } else {
        delta.signum()
    }
}

/// A complete packet from the viewer. Input and command packets share the
/// "ZW" prefix but have different lengths.
enum HostPacket {
    Input { x: i8, y: i8, button: bool, flags: u8 },
    Command { command: u8, arg: u8 },
}

#[derive(Clone, Copy, PartialEq)]
enum PacketKind {
    Input,
    Command,
}

/// Incremental parser for both host input and command packets. Non-magic bytes
/// such as USB console input are discarded while the magic prefix is matched.
struct HostParser {
    buf: [u8; INPUT_PACKET_LEN],
    len: usize,
    kind: PacketKind,
}

impl HostParser {
    fn new() -> Self {
        Self {
            buf: [0; INPUT_PACKET_LEN],
            len: 0,
            kind: PacketKind::Input,
        }
    }

    fn target_len(&self) -> usize {
        match self.kind {
            PacketKind::Input => INPUT_PACKET_LEN,
            PacketKind::Command => COMMAND_PACKET_LEN,
        }
    }

    fn push(&mut self, byte: u8) -> Option<HostPacket> {
        // Match the shared "ZW?1" magic before reading the payload.
        if self.len < INPUT_MAGIC.len() {
            let matched = match self.len {
                0 => byte == INPUT_MAGIC[0],
                1 => byte == INPUT_MAGIC[1],
                2 => {
                    if byte == INPUT_MAGIC[2] {
                        self.kind = PacketKind::Input;
                        true
                    } else if byte == COMMAND_MAGIC[2] {
                        self.kind = PacketKind::Command;
                        true
                    } else {
                        false
                    }
                }
                _ => byte == INPUT_MAGIC[3] && byte == COMMAND_MAGIC[3],
            };
            if matched {
                self.buf[self.len] = byte;
                self.len += 1;
                return None;
            }
            self.kind = PacketKind::Input;
            if byte == b'Z' {
                self.buf[0] = byte;
                self.len = 1;
            } else {
                self.len = 0;
            }
            return None;
        }

        self.buf[self.len] = byte;
        self.len += 1;
        if self.len < self.target_len() {
            return None;
        }
        self.len = 0;
        let version = match self.kind {
            PacketKind::Input => INPUT_VERSION,
            PacketKind::Command => COMMAND_VERSION,
        };
        if self.buf[4] != version {
            return None;
        }
        match self.kind {
            PacketKind::Input => Some(HostPacket::Input {
                x: self.buf[6] as i8,
                y: self.buf[7] as i8,
                button: self.buf[8] & INPUT_BUTTON != 0,
                flags: self.buf[5],
            }),
            PacketKind::Command => Some(HostPacket::Command {
                command: self.buf[5],
                arg: self.buf[6],
            }),
        }
    }
}

/// Read the length of the ESP application image at `offset` by walking its
/// segment table. Returns `None` for a blank or malformed image.
fn read_app_image_size(
    flash: &mut esp_bootloader_esp_idf::partitions::FlashStorage<'_>,
    offset: u32,
    max_size: u32,
) -> Option<u32> {
    let mut header = [0_u8; 24];
    flash.read(offset, &mut header).ok()?;
    if header[0] != APP_IMAGE_MAGIC {
        return None;
    }
    let segments = header[1] as usize;
    if segments == 0 || segments > 16 {
        return None;
    }
    let mut len = header.len() as u32;
    for _ in 0..segments {
        let mut segment = [0_u8; 8];
        flash.read(offset.checked_add(len)?, &mut segment).ok()?;
        let data_len = u32::from_le_bytes([
            segment[4],
            segment[5],
            segment[6],
            segment[7],
        ]);
        len = len.checked_add(8)?.checked_add(data_len)?;
        if len > max_size {
            return None;
        }
    }
    // The image has a checksum padded to 16 bytes and usually an appended
    // SHA-256 digest; round up so the displayed size matches the flash image.
    Some((((len + 15) & !15).checked_add(32)?).min(max_size))
}

/// Send a fixed-size telemetry packet. Failures (no host draining USB) are
/// intentionally ignored; the next interval retries.
fn send_telemetry(
    usb: &mut UsbSerialJtag<'_, Blocking>,
    heap_used: u32,
    heap_total: u32,
    flash_used: u32,
    flash_total: u32,
    uptime_ms: u32,
    physical_joystick_enabled: bool,
) -> bool {
    let mut packet = [0_u8; TELEMETRY_PACKET_LEN];
    packet[0..4].copy_from_slice(&TELEMETRY_MAGIC);
    packet[4] = TELEMETRY_VERSION;
    packet[5] = if physical_joystick_enabled {
        0
    } else {
        TELEMETRY_FLAG_PHYSICAL_JOYSTICK_DISABLED
    };
    packet[6..10].copy_from_slice(&heap_used.to_le_bytes());
    packet[10..14].copy_from_slice(&heap_total.to_le_bytes());
    packet[14..18].copy_from_slice(&flash_used.to_le_bytes());
    packet[18..22].copy_from_slice(&flash_total.to_le_bytes());
    packet[22..26].copy_from_slice(&uptime_ms.to_le_bytes());
    let mut last_progress = Instant::now();
    if !write_all_nb(usb, &packet, &mut last_progress) {
        return false;
    }
    flush_nb(usb, &mut last_progress)
}

fn crc32_byte(crc: u32, byte: u8) -> u32 {
    let mut crc = crc ^ byte as u32;
    for _ in 0..8 {
        crc = if crc & 1 != 0 {
            (crc >> 1) ^ 0xEDB8_8320
        } else {
            crc >> 1
        };
    }
    crc
}

/// Write every byte with the non-blocking TX path. Returns false when the
/// host stops draining the FIFO for `USB_STALL_TIMEOUT_MS`.
fn write_all_nb(usb: &mut UsbSerialJtag<'_, Blocking>, data: &[u8], last_progress: &mut Instant) -> bool {
    for &byte in data {
        loop {
            match usb.write_byte_nb(byte) {
                Ok(()) => {
                    *last_progress = Instant::now();
                    break;
                }
                Err(nb::Error::Other(_)) => return false,
                Err(nb::Error::WouldBlock) => {
                    if last_progress.elapsed().as_millis() > USB_STALL_TIMEOUT_MS {
                        return false;
                    }
                }
            }
        }
    }
    true
}

fn flush_nb(usb: &mut UsbSerialJtag<'_, Blocking>, last_progress: &mut Instant) -> bool {
    loop {
        match usb.flush_tx_nb() {
            Ok(()) => return true,
            Err(nb::Error::Other(_)) => return false,
            Err(nb::Error::WouldBlock) => {
                if last_progress.elapsed().as_millis() > USB_STALL_TIMEOUT_MS {
                    return false;
                }
            }
        }
    }
}


/// Running counters for the once-per-report timing line.
#[derive(Default)]
struct Perf {
    full_rebuilds: u32,
    partial_rebuilds: u32,
    rebuild_us: u64,
    frames: u32,
    send_us: u64,
}

/// Turn the stick deflection into one of the four arrow directions. The
/// dominant axis wins; ties go to the horizontal axis.
fn dominant_direction(dx: i32, dy: i32) -> (i32, i32) {
    if dx == 0 && dy == 0 {
        (0, 0)
    } else if dx != 0 && dx.abs() >= dy.abs() {
        (dx.signum(), 0)
    } else {
        (0, dy.signum())
    }
}

/// Move focus one control in a direction. Records the screen rows that change
/// (the old and new control) so only those bands are repainted. A scroll
/// starts a full rebuild, because the whole content moves.
fn move_focus(
    engine: &mut Warp4Engine,
    dx: i32,
    dy: i32,
    damage: &mut Option<(i32, i32)>,
    full_redraw: &mut bool,
) {
    let before = engine.keyboard_focus_rect();
    let scroll_before = engine.scroll_position();
    if !engine.focus_spatial(dx, dy) {
        return;
    }
    if engine.scroll_position() != scroll_before {
        *full_redraw = true;
        return;
    }
    for (_, y, _, h) in [before, engine.keyboard_focus_rect()].into_iter().flatten() {
        let (y0, y1) = (y, y + h);
        *damage = Some(match *damage {
            Some((a, b)) => (a.min(y0), b.max(y1)),
            None => (y0, y1),
        });
    }
}

/// Rebuild the shape plane and text plane for every band (full frame).
fn rebuild_cache(engine: &mut Warp4Engine, band: &mut LayerSystem, shape: &mut [u8]) {
    if engine.is_dirty() {
        engine.update(LOGICAL_WIDTH as i32, LOGICAL_HEIGHT as i32);
    }
    let mut band_y = 0usize;
    while band_y < LOGICAL_HEIGHT {
        rebuild_band(engine, band, shape, band_y);
        band_y += BAND_HEIGHT;
    }
}

/// Rebuild only the bands that overlap screen rows `y0..y1`.
fn rebuild_rows(engine: &mut Warp4Engine, band: &mut LayerSystem, shape: &mut [u8], y0: i32, y1: i32) {
    if engine.is_dirty() {
        engine.update(LOGICAL_WIDTH as i32, LOGICAL_HEIGHT as i32);
    }
    let end = y1.clamp(0, LOGICAL_HEIGHT as i32) as usize;
    let mut band_y = (y0.clamp(0, LOGICAL_HEIGHT as i32) as usize / BAND_HEIGHT) * BAND_HEIGHT;
    while band_y < end {
        rebuild_band(engine, band, shape, band_y);
        band_y += BAND_HEIGHT;
    }
}

/// Paint one band into the text plane and the shape plane. The band pass fills
/// its own rows first, so text from the previous frame in those rows is
/// cleared by the same occlusion rule as any other shape write.
fn rebuild_band(engine: &mut Warp4Engine, band: &mut LayerSystem, shape: &mut [u8], band_y: usize) {
    let rows = BAND_HEIGHT.min(LOGICAL_HEIGHT - band_y);
    band.set_text_origin(band_y);
    engine.draw_band_to_layer(band, band_y as i32);
    let pixels = band.buf_ref();
    for row in 0..rows {
        let dst = &mut shape[(band_y + row) * LOGICAL_WIDTH..(band_y + row + 1) * LOGICAL_WIDTH];
        for (x, out) in dst.iter_mut().enumerate() {
            *out = rgb332_from_color(Color(pixels[row * LOGICAL_WIDTH + x]));
        }
    }
}

/// Transmit one frame. The RLE stream is produced twice: first to learn the
/// payload length and CRC (nothing is stored), then to send. Each pass composes
/// the frame pixel by pixel from the cached planes and the cursor.
fn send_frame(
    usb: &mut UsbSerialJtag<'_, Blocking>,
    shape: &[u8],
    text: Option<&TextPlane>,
    cursor_x: i32,
    cursor_y: i32,
    seq: u32,
    chroma_carrier: bool,
) -> bool {
    let cursor = (cursor_x, cursor_y);
    let mut payload_len = 0u32;
    let mut crc = 0xFFFF_FFFF_u32;
    encode_frame_rle(
        shape,
        text,
        LOGICAL_WIDTH,
        LOGICAL_HEIGHT,
        cursor,
        chroma_carrier,
        &mut |count, sample| {
            crc = crc32_byte(crc, count);
            crc = crc32_byte(crc, sample);
            payload_len += 2;
        },
    );
    let crc = !crc;

    let flags = FRAME_FLAG_RLE
        | FRAME_FLAG_8BIT
        | if chroma_carrier { FRAME_FLAG_CHROMA_CARRIER } else { 0 };
    let mut header = [0_u8; FRAME_HEADER_LEN];
    header[0..4].copy_from_slice(&FRAME_MAGIC);
    header[4..6].copy_from_slice(&(LOGICAL_WIDTH as u16).to_le_bytes());
    header[6..8].copy_from_slice(&(LOGICAL_HEIGHT as u16).to_le_bytes());
    header[8..10].copy_from_slice(&flags.to_le_bytes());
    header[10..14].copy_from_slice(&seq.to_le_bytes());
    header[14..18].copy_from_slice(&payload_len.to_le_bytes());
    header[18..22].copy_from_slice(&crc.to_le_bytes());

    let mut last_progress = Instant::now();
    if !write_all_nb(usb, &header, &mut last_progress) {
        return false;
    }

    let mut chunk = [0_u8; 256];
    let mut used = 0usize;
    let mut ok = true;
    encode_frame_rle(
        shape,
        text,
        LOGICAL_WIDTH,
        LOGICAL_HEIGHT,
        cursor,
        chroma_carrier,
        &mut |count, sample| {
            if !ok {
                return;
            }
            chunk[used] = count;
            chunk[used + 1] = sample;
            used += 2;
            if used == chunk.len() {
                ok = write_all_nb(usb, &chunk, &mut last_progress);
                used = 0;
            }
        },
    );
    if !ok {
        return false;
    }
    if used > 0 && !write_all_nb(usb, &chunk[..used], &mut last_progress) {
        return false;
    }
    flush_nb(usb, &mut last_progress)
}
