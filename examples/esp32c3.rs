#![no_std]
#![no_main]

extern crate alloc;

use core::panic::PanicInfo;
use baram_core::{Color, LayerSystem};
use baram_warp4::{set_ui_mode, set_ui_scale_percent, UiMode, Warp4Engine};
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
const JOYSTICK_CENTER: i32 = 2048;
const JOYSTICK_DEADZONE: i32 = 520;
const JOYSTICK_FAST_ZONE: i32 = 1350;
const LOOP_DELAY_MS: u32 = 20;
const FRAME_TIME_NS: u64 = LOOP_DELAY_MS as u64 * 1_000_000;
// USB full-speed CDC sustains well under 1.5 MB/s; with RLE payloads around
// 6 KiB this cap allows roughly 10-15 frames per second.
const MIN_FRAME_INTERVAL_MS: u64 = 66;

// Frame stream protocol (little-endian header, RGB565 big-endian payload):
// magic "ZWF2" | width u16 | height u16 | flags u16 | seq u32 | payload_len u32 | crc32 u32
// flags bit0: payload is RLE (entries: run_count u8, color_hi u8, color_lo u8)
const FRAME_MAGIC: [u8; 4] = *b"ZWF2";
const FRAME_HEADER_LEN: usize = 22;
const FRAME_FLAG_RLE: u16 = 1;
// Give up on a frame if the host stops draining the USB FIFO for this long,
// so the input loop keeps running when no viewer is attached.
const USB_STALL_TIMEOUT_MS: u64 = 30;

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

    let mut delay = Delay::new();
    let mut usb = UsbSerialJtag::new(peripherals.USB_DEVICE);

    let mut adc_config = AdcConfig::new();
    let mut joystick_x_pin = adc_config.enable_pin(peripherals.GPIO0, Attenuation::_11dB);
    let mut joystick_y_pin = adc_config.enable_pin(peripherals.GPIO1, Attenuation::_11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);
    let joystick_button = Input::new(
        peripherals.GPIO20,
        InputConfig::default().with_pull(Pull::Up),
    );
    esp_println::println!("[ok] Joystick ADC and push button initialized");

    baram_bsd::config::init_config();
    esp_println::println!("[ok] Config initialized");

    set_ui_mode(UiMode::Xiao);
    set_ui_scale_percent(100);
    let mut engine = Warp4Engine::new_embedded(
        "esp32c3-demo",
        &[
            ("config.ini", include_str!("esp32c3/config.ini")),
            ("main.w4u", include_str!("esp32c3/main.w4u")),
        ],
    );
    engine.set_chrome_visible(false);
    engine.update(LOGICAL_WIDTH as i32, LOGICAL_HEIGHT as i32);

    let mut layer = LayerSystem::new(LOGICAL_WIDTH, LOGICAL_HEIGHT);
    let mut cursor_x = (LOGICAL_WIDTH / 2) as i32;
    let mut cursor_y = (LOGICAL_HEIGHT / 2) as i32;
    let _ = engine.set_hover_changed(cursor_x, cursor_y);
    engine.draw_to_layer(&mut layer, 0, 0);
    draw_cursor(&mut layer, cursor_x, cursor_y);

    let mut seq = 0_u32;
    let mut last_frame_sent: Option<Instant> = None;
    let mut pending_send = true;
    let mut stream_active = false;
    if let Some(sent) = try_send(&mut usb, &layer, &mut seq, &mut last_frame_sent) {
        pending_send = !sent;
        stream_active = sent;
        if sent {
            esp_println::println!("[ok] First frame streamed to USB");
        } else {
            esp_println::println!("[wait] No USB host draining yet; frames resume when the viewer opens");
        }
    }

    let mut now_ns = 0_u64;
    let mut previous_button_down = false;
    let mut x_adc_fault = false;
    let mut y_adc_fault = false;

    loop {
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
                JOYSTICK_CENTER
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
                JOYSTICK_CENTER
            }
        };

        let mut dx = axis_step(raw_x);
        let mut dy = axis_step(raw_y);
        if JOYSTICK_X_REVERSED { dx = -dx; }
        if JOYSTICK_Y_REVERSED { dy = -dy; }

        let mut redraw = engine.tick(now_ns);
        if dx != 0 || dy != 0 {
            cursor_x = (cursor_x + dx).clamp(0, LOGICAL_WIDTH as i32 - 1);
            cursor_y = (cursor_y + dy).clamp(0, LOGICAL_HEIGHT as i32 - 1);
            let _ = engine.set_hover_changed(cursor_x, cursor_y);
            redraw = true;
        }

        // esp-hal's `Input::is_low()` is an inherent method returning `bool`.
        let button_down = joystick_button.is_low();
        if button_down && !previous_button_down {
            engine.click(cursor_x, cursor_y);
            esp_println::println!("[input] Joystick pressed at ({}, {})", cursor_x, cursor_y);
            redraw = true;
        } else if !button_down && previous_button_down {
            engine.release();
            redraw = true;
        }
        previous_button_down = button_down;

        if redraw {
            engine.draw_to_layer(&mut layer, 0, 0);
            draw_cursor(&mut layer, cursor_x, cursor_y);
            pending_send = true;
        }

        if pending_send {
            match try_send(&mut usb, &layer, &mut seq, &mut last_frame_sent) {
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
    layer: &LayerSystem,
    seq: &mut u32,
    last_frame_sent: &mut Option<Instant>,
) -> Option<bool> {
    if let Some(sent_at) = *last_frame_sent {
        if sent_at.elapsed().as_millis() < MIN_FRAME_INTERVAL_MS {
            return None;
        }
    }
    *seq = seq.wrapping_add(1);
    let ok = send_frame(usb, layer, *seq);
    *last_frame_sent = Some(Instant::now());
    Some(ok)
}

fn axis_step(raw: i32) -> i32 {
    let delta = raw - JOYSTICK_CENTER;
    let magnitude = delta.abs();
    if magnitude < JOYSTICK_DEADZONE {
        0
    } else if magnitude >= JOYSTICK_FAST_ZONE {
        delta.signum() * 3
    } else {
        delta.signum()
    }
}

fn draw_cursor(layer: &mut LayerSystem, x: i32, y: i32) {
    let x = x.clamp(0, LOGICAL_WIDTH as i32 - 1) as usize;
    let y = y.clamp(0, LOGICAL_HEIGHT as i32 - 1) as usize;
    let black = Color::rgb(0, 0, 0);
    let white = Color::rgb(255, 255, 255);
    layer.fill_rect(x.saturating_sub(2), y, 5, 1, black);
    layer.fill_rect(x, y.saturating_sub(2), 1, 5, black);
    layer.fill_rect(x.saturating_sub(1), y, 3, 1, white);
    layer.fill_rect(x, y.saturating_sub(1), 1, 3, white);
}

fn rgb565_bytes(color: Color) -> [u8; 2] {
    let rgb565 = ((color.r() as u16 & 0xF8) << 8)
        | ((color.g() as u16 & 0xFC) << 3)
        | (color.b() as u16 >> 3);
    [(rgb565 >> 8) as u8, rgb565 as u8]
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

/// Run-length encoder over RGB565 colors. Entries are `[run_count, hi, lo]`
/// with `run_count` in 1..=255. The same encoder drives both the sizing pass
/// (CRC/length only) and the streaming pass, so both produce identical bytes.
struct RleEncoder {
    color: Option<u16>,
    run: u8,
    crc: u32,
    size: usize,
}

impl RleEncoder {
    fn new() -> Self {
        Self {
            color: None,
            run: 0,
            crc: 0xFFFF_FFFF,
            size: 0,
        }
    }

    fn push(&mut self, color: u16, emit: &mut impl FnMut(&[u8])) {
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
        let bytes = [self.run, (color >> 8) as u8, color as u8];
        for &b in &bytes {
            self.crc = crc32_byte(self.crc, b);
        }
        self.size += 3;
        emit(&bytes);
        self.run = 0;
    }
}

fn send_frame(usb: &mut UsbSerialJtag<'_, Blocking>, layer: &LayerSystem, seq: u32) -> bool {
    if layer.width() != LOGICAL_WIDTH || layer.height() != LOGICAL_HEIGHT {
        return false;
    }
    let pixels = layer.buf_ref();
    let pixel_count = LOGICAL_WIDTH * LOGICAL_HEIGHT;
    if pixels.len() < pixel_count {
        return false;
    }

    // Pass 1: RLE size and CRC. The raw encoding's length is known up front,
    // so the raw CRC is only computed on the (rare) fallback path.
    let raw_len = pixel_count * 2;
    let mut rle = RleEncoder::new();
    for &pixel in &pixels[..pixel_count] {
        rle.push(u16::from_be_bytes(rgb565_bytes(Color(pixel))), &mut |_b: &[u8]| {});
    }
    rle.finish(&mut |_b: &[u8]| {});

    let use_rle = rle.size < raw_len;
    let (flags, payload_len, crc) = if use_rle {
        (FRAME_FLAG_RLE, rle.size, !rle.crc)
    } else {
        let mut crc_raw = 0xFFFF_FFFF_u32;
        for &pixel in &pixels[..pixel_count] {
            let bytes = rgb565_bytes(Color(pixel));
            crc_raw = crc32_byte(crc_raw, bytes[0]);
            crc_raw = crc32_byte(crc_raw, bytes[1]);
        }
        (0, raw_len, !crc_raw)
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
        let mut buf = [0_u8; 768];
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
            let mut rle = RleEncoder::new();
            for &pixel in &pixels[..pixel_count] {
                rle.push(u16::from_be_bytes(rgb565_bytes(Color(pixel))), &mut emit);
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
        let mut line = [0_u8; LOGICAL_WIDTH * 2];
        for y in 0..LOGICAL_HEIGHT {
            let row_start = y * LOGICAL_WIDTH;
            let mut out = 0;
            for x in 0..LOGICAL_WIDTH {
                let bytes = rgb565_bytes(Color(pixels[row_start + x]));
                line[out] = bytes[0];
                line[out + 1] = bytes[1];
                out += 2;
            }
            if !write_all_nb(usb, &line, &mut last_progress) {
                return false;
            }
        }
    }

    flush_nb(usb, &mut last_progress)
}
