#![no_std]
#![no_main]

extern crate alloc;

use core::panic::PanicInfo;
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
// The cursor is a 5x5 plus sign. Its pixels are the only part of the layer
// that changes while the UI is idle, so keep a tiny backing store for them
// instead of repainting the whole view tree for every pointer step.
const CURSOR_RADIUS: i32 = 2;
const CURSOR_BOX: usize = (CURSOR_RADIUS * 2 + 1) as usize;
const CURSOR_BACKING_LEN: usize = CURSOR_BOX * CURSOR_BOX;
const LOOP_DELAY_MS: u32 = 20;
const FRAME_TIME_NS: u64 = LOOP_DELAY_MS as u64 * 1_000_000;
// USB full-speed CDC sustains well under 1.5 MB/s; with RLE payloads around
// 6 KiB this cap allows roughly 10-15 frames per second.
const MIN_FRAME_INTERVAL_MS: u64 = 66;

// Frame stream protocol (little-endian header, 8-bit payload):
// magic "ZWF2" | width u16 | height u16 | flags u16 | seq u32 | payload_len u32 | crc32 u32
// flags bit0: payload is RLE (entries: run_count u8, color u8)
// flags bit1: payload is chroma carrier modulated (Y luminance base + C subcarrier)
// flags bit2: payload is 8-bit color (RGB332)
const FRAME_MAGIC: [u8; 4] = *b"ZWF2";
const FRAME_HEADER_LEN: usize = 22;
const FRAME_FLAG_RLE: u16 = 1;
const FRAME_FLAG_CHROMA_CARRIER: u16 = 2;
const FRAME_FLAG_8BIT: u16 = 4;
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

    // The renderer stores 32-bit pixels. A 135x240 logical framebuffer uses
    // 129,600 bytes; the whole runtime plus renderer is kept in this bounded heap.
    esp_alloc::heap_allocator!(size: 256 * 1024);
    esp_println::println!("[ok] Heap allocator initialized (256 KiB)");

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

    baram_bsd::config::init_config();
    esp_println::println!("[ok] Config initialized");

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

    let mut layer = LayerSystem::new(LOGICAL_WIDTH, LOGICAL_HEIGHT);
    let mut fb = alloc::vec![0_u8; LOGICAL_WIDTH * LOGICAL_HEIGHT];
    let mut cursor_x = (LOGICAL_WIDTH / 2) as i32;
    let mut cursor_y = (LOGICAL_HEIGHT / 2) as i32;
    let mut cursor_overlay = CursorOverlay::new();
    let mut chroma_carrier_mode = false;
    let _ = engine.set_hover_changed(cursor_x, cursor_y);
    redraw_ui(&mut layer, &mut engine, &mut fb, &mut cursor_overlay, cursor_x, cursor_y, chroma_carrier_mode);

    let mut seq = 0_u32;
    let mut last_frame_sent: Option<Instant> = None;
    let mut pending_send = true;
    let mut cursor_dirty = false;
    let mut stream_active = false;
    if let Some(sent) = try_send(&mut usb, &fb, &mut seq, &mut last_frame_sent, chroma_carrier_mode) {
        pending_send = !sent;
        stream_active = sent;
        if sent {
            esp_println::println!("[ok] First frame streamed to USB (8-bit color)");
        } else {
            esp_println::println!("[wait] No USB host draining yet; frames resume when the viewer opens");
        }
    }

    let mut now_ns = 0_u64;
    let mut previous_button_down = false;
    let mut x_adc_fault = false;
    let mut y_adc_fault = false;
    let mut input_parser = InputParser::new();
    let mut virtual_x = 0_i8;
    let mut virtual_y = 0_i8;
    let mut virtual_button = false;
    let mut virtual_last_rx: Option<Instant> = None;

    loop {
        // Drain host input every iteration; the viewer drives the same cursor
        // as the physical stick and sends a heartbeat while it is held.
        while let Ok(byte) = usb.read_byte() {
            if let Some((x, y, button, flags)) = input_parser.push(byte) {
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
                    render_layer_to_fb(&layer, &mut fb, chroma_carrier_mode);
                    cursor_overlay.draw(&mut fb, cursor_x, cursor_y, chroma_carrier_mode);
                    pending_send = true;
                }
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

        let raw_x = match nb::block!(adc.read_oneshot(&mut joystick_x_pin)) {
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
        };
        let raw_y = match nb::block!(adc.read_oneshot(&mut joystick_y_pin)) {
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
        };

        // Follow slow electrical/mechanical drift while the stick is at rest.
        // Leave the center alone while it is deflected, otherwise holding a
        // direction would gradually cancel itself out.
        if !x_adc_fault {
            joystick_center_x = recenter(joystick_center_x, raw_x);
        }
        if !y_adc_fault {
            joystick_center_y = recenter(joystick_center_y, raw_y);
        }

        let mut dx = axis_step(raw_x, joystick_center_x);
        let mut dy = axis_step(raw_y, joystick_center_y);
        if JOYSTICK_X_REVERSED { dx = -dx; }
        if JOYSTICK_Y_REVERSED { dy = -dy; }
        // The host already defines screen-space directions, so the physical
        // axis-reversal switches do not apply to the virtual stick.
        let virtual_dx = virtual_axis_step(virtual_x);
        let virtual_dy = virtual_axis_step(virtual_y);
        if virtual_dx != 0 { dx = virtual_dx; }
        if virtual_dy != 0 { dy = virtual_dy; }

        engine.tick(now_ns);

        if dx != 0 || dy != 0 {
            let next_x = (cursor_x + dx).clamp(0, LOGICAL_WIDTH as i32 - 1);
            let next_y = (cursor_y + dy).clamp(0, LOGICAL_HEIGHT as i32 - 1);
            if next_x != cursor_x || next_y != cursor_y {
                cursor_x = next_x;
                cursor_y = next_y;
                let _ = engine.set_hover_changed(cursor_x, cursor_y);
                cursor_dirty = true;
            }
        }

        // esp-hal's `Input::is_low()` is an inherent method returning `bool`.
        let button_down = joystick_button.is_low() || virtual_button;
        if button_down && !previous_button_down {
            engine.click(cursor_x, cursor_y);
            esp_println::println!("[input] Joystick pressed at ({}, {})", cursor_x, cursor_y);
        } else if !button_down && previous_button_down {
            engine.release();
        }
        previous_button_down = button_down;

        // Do not repaint faster than the USB link can carry frames. A full
        // view repaint is only needed when the engine state changed; plain
        // cursor motion just replaces the 5x5 cursor patch.
        let frame_due = match last_frame_sent {
            Some(sent_at) => sent_at.elapsed().as_millis() >= MIN_FRAME_INTERVAL_MS,
            None => true,
        };
        if frame_due {
            if engine.is_dirty() {
                redraw_ui(&mut layer, &mut engine, &mut fb, &mut cursor_overlay, cursor_x, cursor_y, chroma_carrier_mode);
                cursor_dirty = false;
                pending_send = true;
            } else if cursor_dirty {
                cursor_overlay.clear(&mut fb);
                cursor_overlay.draw(&mut fb, cursor_x, cursor_y, chroma_carrier_mode);
                cursor_dirty = false;
                pending_send = true;
            }
        }

        if pending_send {
            match try_send(&mut usb, &fb, &mut seq, &mut last_frame_sent, chroma_carrier_mode) {
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
                // Throttled: keep the rendered frame and send it when due.
                None => {}
            }
        }

        delay.delay_millis(LOOP_DELAY_MS);
        now_ns = now_ns.saturating_add(FRAME_TIME_NS);
    }
}

/// Throttled frame attempt: `None` while the minimum frame interval has not
/// elapsed, `Some(sent)` once an attempt was actually made.
fn try_send(
    usb: &mut UsbSerialJtag<'_, Blocking>,
    fb: &[u8],
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
    let ok = send_frame(usb, fb, *seq, chroma_carrier);
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

/// Incremental parser for host input packets. Non-magic bytes such as USB
/// console input are discarded while the magic prefix is matched.
struct InputParser {
    buf: [u8; INPUT_PACKET_LEN],
    len: usize,
}

impl InputParser {
    fn new() -> Self {
        Self {
            buf: [0; INPUT_PACKET_LEN],
            len: 0,
        }
    }

    /// Returns `(x, y, button, flags)` once a complete packet has been received.
    fn push(&mut self, byte: u8) -> Option<(i8, i8, bool, u8)> {
        if self.len < INPUT_MAGIC.len() {
            if byte == INPUT_MAGIC[self.len] {
                self.buf[self.len] = byte;
                self.len += 1;
            } else if byte == INPUT_MAGIC[0] {
                self.buf[0] = byte;
                self.len = 1;
            } else {
                self.len = 0;
            }
            return None;
        }
        self.buf[self.len] = byte;
        self.len += 1;
        if self.len < INPUT_PACKET_LEN {
            return None;
        }
        self.len = 0;
        if self.buf[4] != INPUT_VERSION {
            return None;
        }
        Some((
            self.buf[6] as i8,
            self.buf[7] as i8,
            self.buf[8] & INPUT_BUTTON != 0,
            self.buf[5],
        ))
    }
}

type CursorRect = (usize, usize, usize, usize);

/// Cursor pixels saved from the 8-bit framebuffer before the overlay was drawn.
/// Restoring this 5x5 patch is enough to remove the cursor without re-rendering.
struct CursorOverlay {
    rect: Option<CursorRect>,
    backing: [u8; CURSOR_BACKING_LEN],
}

impl CursorOverlay {
    fn new() -> Self {
        Self {
            rect: None,
            backing: [0; CURSOR_BACKING_LEN],
        }
    }

    /// Restore the pixels hidden by the cursor currently in the framebuffer.
    fn clear(&mut self, fb: &mut [u8]) {
        let Some((x0, y0, x1, y1)) = self.rect.take() else {
            return;
        };
        let width = x1 - x0;
        let height = y1 - y0;
        for row in 0..height {
            let src = row * width;
            let dst = (y0 + row) * LOGICAL_WIDTH + x0;
            fb[dst..dst + width].copy_from_slice(&self.backing[src..src + width]);
        }
    }

    /// Save the background under `(x, y)` and draw the cursor on top.
    fn draw(&mut self, fb: &mut [u8], x: i32, y: i32, chroma_carrier: bool) {
        let rect = cursor_footprint(x, y);
        let (x0, y0, x1, y1) = rect;
        let width = x1 - x0;
        let height = y1 - y0;
        for row in 0..height {
            let src = (y0 + row) * LOGICAL_WIDTH + x0;
            let dst = row * width;
            self.backing[dst..dst + width].copy_from_slice(&fb[src..src + width]);
        }
        draw_cursor_8bit(fb, x, y, chroma_carrier);
        self.rect = Some(rect);
    }
}

fn cursor_footprint(x: i32, y: i32) -> CursorRect {
    let x = x.clamp(0, LOGICAL_WIDTH as i32 - 1);
    let y = y.clamp(0, LOGICAL_HEIGHT as i32 - 1);
    let x0 = x.saturating_sub(CURSOR_RADIUS) as usize;
    let y0 = y.saturating_sub(CURSOR_RADIUS) as usize;
    let x1 = (x0 + CURSOR_BOX).min(LOGICAL_WIDTH);
    let y1 = (y0 + CURSOR_BOX).min(LOGICAL_HEIGHT);
    (x0, y0, x1, y1)
}

#[inline(always)]
fn rgb_to_rgb332(r: u8, g: u8, b: u8) -> u8 {
    (r & 0xE0) | ((g & 0xE0) >> 3) | (b >> 6)
}

#[inline(always)]
fn rgb_to_chroma_carrier(r: u8, g: u8, b: u8, x: usize, y: usize) -> u8 {
    let r_i = r as i32;
    let g_i = g as i32;
    let b_i = b as i32;
    // Y: 0.299 R + 0.587 G + 0.114 B -> (77*R + 150*G + 29*B) >> 8
    let y_val = (77 * r_i + 150 * g_i + 29 * b_i) >> 8;
    // U: 0.492 (B - Y) -> ((b_i - y_val) * 126) >> 8
    // V: 0.877 (R - Y) -> ((r_i - y_val) * 224) >> 8
    let u_val = ((b_i - y_val) * 126) >> 8;
    let v_val = ((r_i - y_val) * 224) >> 8;

    let y_phase = (y & 1) * 2;
    let phase = (x + y_phase) & 3;
    let carrier = match phase {
        0 => u_val,
        1 => v_val,
        2 => -u_val,
        _ => -v_val,
    };
    (y_val + carrier).clamp(0, 255) as u8
}

fn render_layer_to_fb(layer: &LayerSystem, fb: &mut [u8], chroma_carrier: bool) {
    let pixels = layer.buf_ref();
    let total = LOGICAL_WIDTH * LOGICAL_HEIGHT;
    if chroma_carrier {
        for y in 0..LOGICAL_HEIGHT {
            let row_off = y * LOGICAL_WIDTH;
            for x in 0..LOGICAL_WIDTH {
                let col = Color(pixels[row_off + x]);
                fb[row_off + x] = rgb_to_chroma_carrier(col.r(), col.g(), col.b(), x, y);
            }
        }
    } else {
        for i in 0..total {
            let col = Color(pixels[i]);
            fb[i] = rgb_to_rgb332(col.r(), col.g(), col.b());
        }
    }
}

/// Repaint the full view tree and update the 8-bit framebuffer.
fn redraw_ui(
    layer: &mut LayerSystem,
    engine: &mut Warp4Engine,
    fb: &mut [u8],
    cursor: &mut CursorOverlay,
    x: i32,
    y: i32,
    chroma_carrier: bool,
) {
    cursor.clear(fb);
    engine.draw_to_layer(layer, 0, 0);
    render_layer_to_fb(layer, fb, chroma_carrier);
    cursor.draw(fb, x, y, chroma_carrier);
}

fn draw_cursor_8bit(fb: &mut [u8], x: i32, y: i32, chroma_carrier: bool) {
    let x = x.clamp(0, LOGICAL_WIDTH as i32 - 1) as usize;
    let y = y.clamp(0, LOGICAL_HEIGHT as i32 - 1) as usize;
    let (black, white) = if chroma_carrier {
        (0_u8, 255_u8)
    } else {
        (rgb_to_rgb332(0, 0, 0), rgb_to_rgb332(255, 255, 255))
    };
    let radius = CURSOR_RADIUS as usize;

    let x_start = x.saturating_sub(radius);
    let x_end = (x_start + CURSOR_BOX).min(LOGICAL_WIDTH);
    let row_off = y * LOGICAL_WIDTH;
    for px in x_start..x_end {
        fb[row_off + px] = black;
    }

    let y_start = y.saturating_sub(radius);
    let y_end = (y_start + CURSOR_BOX).min(LOGICAL_HEIGHT);
    for py in y_start..y_end {
        fb[py * LOGICAL_WIDTH + x] = black;
    }

    let cx_start = x.saturating_sub(1);
    let cx_end = (cx_start + 3).min(LOGICAL_WIDTH);
    for px in cx_start..cx_end {
        fb[row_off + px] = white;
    }

    let cy_start = y.saturating_sub(1);
    let cy_end = (cy_start + 3).min(LOGICAL_HEIGHT);
    for py in cy_start..cy_end {
        fb[py * LOGICAL_WIDTH + x] = white;
    }
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

/// Run-length encoder over 8-bit color / modulated samples.
/// Entries are `[run_count, sample]` with `run_count` in 1..=255.
struct RleEncoder8 {
    color: Option<u8>,
    run: u8,
    crc: u32,
    size: usize,
}

impl RleEncoder8 {
    fn new() -> Self {
        Self {
            color: None,
            run: 0,
            crc: 0xFFFF_FFFF,
            size: 0,
        }
    }

    fn push(&mut self, color: u8, emit: &mut impl FnMut(&[u8])) {
        if self.color == Some(color) {
            if self.run == 255 {
                self.emit_current(emit);
                self.run = 1;
            } else {
                self.run += 1;
            }
        } else {
            self.finish(emit);
            self.color = Some(color);
            self.run = 1;
        }
    }

    fn finish(&mut self, emit: &mut impl FnMut(&[u8])) {
        if self.run > 0 {
            self.emit_current(emit);
        }
        self.color = None;
        self.run = 0;
    }

    fn emit_current(&mut self, emit: &mut impl FnMut(&[u8])) {
        let color = self.color.unwrap_or(0);
        let bytes = [self.run, color];
        for &b in &bytes {
            self.crc = crc32_byte(self.crc, b);
        }
        self.size += 2;
        emit(&bytes);
        self.run = 0;
    }
}

fn send_frame(
    usb: &mut UsbSerialJtag<'_, Blocking>,
    fb: &[u8],
    seq: u32,
    chroma_carrier: bool,
) -> bool {
    let pixel_count = LOGICAL_WIDTH * LOGICAL_HEIGHT;
    if fb.len() < pixel_count {
        return false;
    }

    let raw_len = pixel_count;
    let mut rle = RleEncoder8::new();
    for &sample in &fb[..pixel_count] {
        rle.push(sample, &mut |_b: &[u8]| {});
    }
    rle.finish(&mut |_b: &[u8]| {});

    let use_rle = rle.size < raw_len;
    let flags_base = FRAME_FLAG_8BIT | if chroma_carrier { FRAME_FLAG_CHROMA_CARRIER } else { 0 };
    let (flags, payload_len, crc) = if use_rle {
        (flags_base | FRAME_FLAG_RLE, rle.size, !rle.crc)
    } else {
        let mut crc_raw = 0xFFFF_FFFF_u32;
        for &sample in &fb[..pixel_count] {
            crc_raw = crc32_byte(crc_raw, sample);
        }
        (flags_base, raw_len, !crc_raw)
    };

    let mut header = [0_u8; FRAME_HEADER_LEN];
    header[0..4].copy_from_slice(&FRAME_MAGIC);
    header[4..6].copy_from_slice(&(LOGICAL_WIDTH as u16).to_le_bytes());
    header[6..8].copy_from_slice(&(LOGICAL_HEIGHT as u16).to_le_bytes());
    header[8..10].copy_from_slice(&flags.to_le_bytes());
    header[10..14].copy_from_slice(&seq.to_le_bytes());
    header[14..18].copy_from_slice(&(payload_len as u32).to_le_bytes());
    header[18..22].copy_from_slice(&crc.to_le_bytes());

    let mut last_progress = Instant::now();
    if !write_all_nb(usb, &header, &mut last_progress) {
        return false;
    }

    if use_rle {
        let mut buf = [0_u8; 512];
        let mut pos = 0_usize;
        let mut ok = true;
        {
            let mut emit = |bytes: &[u8]| {
                if !ok {
                    return;
                }
                if pos + bytes.len() > buf.len() {
                    if !write_all_nb(usb, &buf[..pos], &mut last_progress) {
                        ok = false;
                        return;
                    }
                    pos = 0;
                }
                buf[pos..pos + bytes.len()].copy_from_slice(bytes);
                pos += bytes.len();
            };
            let mut rle = RleEncoder8::new();
            for &sample in &fb[..pixel_count] {
                rle.push(sample, &mut emit);
            }
            rle.finish(&mut emit);
        }
        if !ok {
            return false;
        }
        if pos > 0 && !write_all_nb(usb, &buf[..pos], &mut last_progress) {
            return false;
        }
    } else {
        for y in 0..LOGICAL_HEIGHT {
            let row_start = y * LOGICAL_WIDTH;
            let line = &fb[row_start..row_start + LOGICAL_WIDTH];
            if !write_all_nb(usb, line, &mut last_progress) {
                return false;
            }
        }
    }

    flush_nb(usb, &mut last_progress)
}
