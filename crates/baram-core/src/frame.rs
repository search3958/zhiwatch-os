//! Transmit-time composition for the embedded frame stream.
//!
//! The cached frame is two planes:
//!   * `shape`: 8-bit RGB332 pixels of everything except text, width*height bytes.
//!   * `text`:  the 3-bit coverage plane and its per-run colors (`TextPlane`).
//!
//! Nothing is composed into a third buffer. When a frame is sent, each pixel is
//! composed on the fly: shape pixel, then text blended with its coverage, then
//! the cursor. This is the only composition step per frame, and it happens only
//! when a frame is actually transmitted. A UI redraw (which rebuilds the two
//! planes) is only needed when the UI changes.
//!
//! The only compression on the wire is the 8-bit run-length code below, which
//! is lossless. Entries are `(run_count u8, sample u8)`, run_count is 1..=255.

use crate::text_plane::{rgb332_blend, rgb332_expand, TextPlane};

/// 3x3-ish plus-shaped cursor: black arms of length 5, white inner plus.
pub const CURSOR_RADIUS: i32 = 2;
const RGB332_BLACK: u8 = 0x00;
const RGB332_WHITE: u8 = 0xFF;

/// Cursor color at a pixel, or `None` when the pixel is not part of the cursor.
#[inline]
pub fn cursor_pixel(x: i32, y: i32, cx: i32, cy: i32) -> Option<u8> {
    let dx = x - cx;
    let dy = y - cy;
    let on_h = dy == 0 && dx.abs() <= CURSOR_RADIUS;
    let on_v = dx == 0 && dy.abs() <= CURSOR_RADIUS;
    if !(on_h || on_v) {
        return None;
    }
    let white = (dy == 0 && dx.abs() <= 1) || (dx == 0 && dy.abs() <= 1);
    Some(if white { RGB332_WHITE } else { RGB332_BLACK })
}

/// Sample for the chroma-subcarrier display mode. Input is 8-bit RGB332 and
/// the output is one byte per pixel, as in the original firmware.
#[inline]
pub fn chroma_carrier_sample(rgb332: u8, x: usize, y: usize) -> u8 {
    let (r, g, b) = rgb332_expand(rgb332);
    let r_i = r as i32;
    let g_i = g as i32;
    let b_i = b as i32;
    // Y: 0.299 R + 0.587 G + 0.114 B -> (77*R + 150*G + 29*B) >> 8
    let y_val = (77 * r_i + 150 * g_i + 29 * b_i) >> 8;
    // U: 0.492 (B - Y); V: 0.877 (R - Y)
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

/// Composed RGB332 pixel at `(x, y)` before cursor and display-mode mapping.
#[inline]
pub fn composed_pixel(shape: &[u8], text: Option<&TextPlane>, width: usize, x: usize, y: usize) -> u8 {
    let base = shape[y * width + x];
    let Some(plane) = text else {
        return base;
    };
    let coverage = plane.coverage(x, y);
    if coverage == 0 {
        return base;
    }
    let fg = plane.color_at(x, y).unwrap_or(RGB332_BLACK);
    rgb332_blend(base, fg, coverage)
}

/// Final sample for the wire: composed pixel, cursor overlay, then the display
/// mode mapping (plain RGB332 or chroma subcarrier).
#[inline]
pub fn frame_sample(
    shape: &[u8],
    text: Option<&TextPlane>,
    width: usize,
    x: usize,
    y: usize,
    cursor: (i32, i32),
    chroma_carrier: bool,
) -> u8 {
    let rgb332 = cursor_pixel(x as i32, y as i32, cursor.0, cursor.1)
        .unwrap_or_else(|| composed_pixel(shape, text, width, x, y));
    if chroma_carrier {
        chroma_carrier_sample(rgb332, x, y)
    } else {
        rgb332
    }
}

/// Streaming 8-bit run-length encoder. `push` emits a finished entry through
/// the callback whenever the sample changes or the run reaches 255.
#[derive(Default)]
pub struct Rle8 {
    sample: u8,
    count: u8,
}

impl Rle8 {
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    pub fn push(&mut self, sample: u8, emit: &mut impl FnMut(u8, u8)) {
        if self.count != 0 && self.sample == sample && self.count < 255 {
            self.count += 1;
            return;
        }
        if self.count != 0 {
            emit(self.count, self.sample);
        }
        self.sample = sample;
        self.count = 1;
    }

    /// Emit the final pending run, if any.
    pub fn finish(&mut self, emit: &mut impl FnMut(u8, u8)) {
        if self.count != 0 {
            emit(self.count, self.sample);
            self.count = 0;
        }
    }
}

/// Encode a whole frame. `emit(run_count, sample)` is called for each entry in
/// stream order. The device calls this twice per frame: once to learn the
/// payload length and CRC, once to transmit.
pub fn encode_frame_rle(
    shape: &[u8],
    text: Option<&TextPlane>,
    width: usize,
    height: usize,
    cursor: (i32, i32),
    chroma_carrier: bool,
    emit: &mut impl FnMut(u8, u8),
) {
    let mut rle = Rle8::new();
    for y in 0..height {
        for x in 0..width {
            let sample = frame_sample(shape, text, width, x, y, cursor, chroma_carrier);
            rle.push(sample, emit);
        }
    }
    rle.finish(emit);
}

/// Encode only a rectangular region while sampling from the original full-frame
/// planes. Coordinates in the packet remain absolute; samples are emitted in
/// row-major order within the rectangle.
pub fn encode_rect_rle(
    shape: &[u8],
    text: Option<&TextPlane>,
    width: usize,
    height: usize,
    rect: (usize, usize, usize, usize),
    cursor: (i32, i32),
    emit: &mut impl FnMut(u8, u8),
) {
    let (x0, y0, rect_w, rect_h) = rect;
    if width == 0
        || height == 0
        || x0 >= width
        || y0 >= height
        || rect_w == 0
        || rect_h == 0
        || shape.len() < width.saturating_mul(height)
    {
        return;
    }
    let x1 = x0.saturating_add(rect_w).min(width);
    let y1 = y0.saturating_add(rect_h).min(height);
    let mut rle = Rle8::new();
    for y in y0..y1 {
        for x in x0..x1 {
            rle.push(frame_sample(shape, text, width, x, y, cursor, false), emit);
        }
    }
    rle.finish(emit);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::text_plane::rgb332_from_rgb;
    use alloc::vec;
    use alloc::vec::Vec;

    fn decode(entries: &[(u8, u8)]) -> Vec<u8> {
        let mut out = Vec::new();
        for &(count, sample) in entries {
            assert!(count >= 1, "zero-length run");
            out.extend(core::iter::repeat(sample).take(count as usize));
        }
        out
    }

    #[test]
    fn rle_round_trips_arbitrary_data() {
        let mut data = Vec::new();
        let mut seed = 12345u32;
        for _ in 0..5000 {
            seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
            let v = ((seed >> 16) % 4) as u8 * 60;
            let run = 1 + ((seed >> 8) % 600) as usize;
            data.extend(core::iter::repeat(v).take(run));
        }
        let mut entries = Vec::new();
        let mut rle = Rle8::new();
        for &s in &data {
            rle.push(s, &mut |c, v| entries.push((c, v)));
        }
        rle.finish(&mut |c, v| entries.push((c, v)));
        assert_eq!(decode(&entries), data);
    }

    #[test]
    fn rle_splits_runs_at_255() {
        let mut entries = Vec::new();
        let mut rle = Rle8::new();
        for _ in 0..600 {
            rle.push(7, &mut |c, v| entries.push((c, v)));
        }
        rle.finish(&mut |c, v| entries.push((c, v)));
        assert_eq!(entries, vec![(255, 7), (255, 7), (90, 7)]);
    }

    #[test]
    fn rect_encoder_matches_frame_subregion() {
        let w = 11;
        let h = 7;
        let mut shape = vec![0u8; w * h];
        for (i, v) in shape.iter_mut().enumerate() {
            *v = (i * 29 % 256) as u8;
        }
        let rect = (3, 2, 5, 4);
        let mut entries = Vec::new();
        encode_rect_rle(&shape, None, w, h, rect, (-1000, -1000), &mut |c, v| {
            entries.push((c, v));
        });
        let decoded = decode(&entries);
        let mut expected = Vec::new();
        for y in rect.1..rect.1 + rect.3 {
            expected.extend_from_slice(&shape[y * w + rect.0..y * w + rect.0 + rect.2]);
        }
        assert_eq!(decoded, expected);
    }

    #[test]
    fn cursor_is_a_plus_with_white_center() {
        assert_eq!(cursor_pixel(10, 10, 10, 10), Some(RGB332_WHITE));
        assert_eq!(cursor_pixel(11, 10, 10, 10), Some(RGB332_WHITE));
        assert_eq!(cursor_pixel(12, 10, 10, 10), Some(RGB332_BLACK));
        assert_eq!(cursor_pixel(10, 8, 10, 10), Some(RGB332_BLACK));
        assert_eq!(cursor_pixel(13, 10, 10, 10), None);
        assert_eq!(cursor_pixel(11, 11, 10, 10), None);
    }

    #[test]
    fn composition_applies_text_over_shape_and_cursor_on_top() {
        let w = 4;
        let shape = vec![rgb332_from_rgb(255, 255, 255); w * 4];
        let mut text = TextPlane::new(w, 4);
        text.begin_band(0);
        text.begin_run(0, 0, 4, 4, rgb332_from_rgb(0, 0, 0));
        text.put_coverage(0, 0, 7);
        let px = composed_pixel(&shape, Some(&text), w, 0, 0);
        assert_eq!(px, rgb332_from_rgb(0, 0, 0));
        // Uncovered pixel keeps the shape color.
        assert_eq!(composed_pixel(&shape, Some(&text), w, 1, 0), shape[1]);
        // Cursor wins over text and shape.
        assert_eq!(frame_sample(&shape, Some(&text), w, 0, 0, (0, 0), false), RGB332_WHITE);
    }

    #[test]
    fn encode_matches_per_pixel_sampling() {
        let w = 9;
        let h = 5;
        let mut shape = vec![0u8; w * h];
        for (i, v) in shape.iter_mut().enumerate() {
            *v = (i * 37 % 256) as u8;
        }
        let mut entries = Vec::new();
        encode_frame_rle(&shape, None, w, h, (3, 2), false, &mut |c, v| entries.push((c, v)));
        let decoded = decode(&entries);
        let mut expected = Vec::new();
        for y in 0..h {
            for x in 0..w {
                expected.push(frame_sample(&shape, None, w, x, y, (3, 2), false));
            }
        }
        assert_eq!(decoded, expected);
    }
}
