//! Text plane for the embedded (ESP32-C3) renderer.
//!
//! Text is kept out of the shape layer. Every glyph writes only a 3-bit
//! anti-aliasing coverage value into a full-frame plane (3 bits per pixel,
//! bit-packed). The color of a piece of text is not stored per pixel; it is
//! kept once per draw call as a `TextRun` (bounding box + RGB332 color), so
//! the coverage plane stays small and the color information stays separate.
//!
//! Z-order is preserved by occlusion: any later opaque shape write clears the
//! coverage under its rectangle (see `LayerSystem::occlude_text`), so text that
//! was painted before a shape can no longer show through it.
//!
//! The layer draws band by band. Coordinates passed to the `LayerSystem`
//! methods are band-local; `TextPlane` stores absolute rows.

use crate::color::Color;
use crate::layer::LayerSystem;
use alloc::vec;
use alloc::vec::Vec;

/// Maximum number of text draw calls tracked per frame. Calls beyond this
/// still draw their coverage, but their pixels fall back to black.
pub const TEXT_RUN_CAPACITY: usize = 256;

#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct TextRun {
    pub x0: i16,
    pub y0: i16,
    pub x1: i16,
    pub y1: i16,
    /// RGB332 color of this draw call.
    pub color: u8,
}

pub struct TextPlane {
    width: usize,
    height: usize,
    /// Absolute row of band-local row 0 for the band currently being drawn.
    origin_y: usize,
    /// 3 bits per pixel, little-endian bit order, plus one padding byte.
    cov: Vec<u8>,
    runs: Vec<TextRun>,
    /// Runs registered by the band pass in progress (or, after a complete
    /// frame, the number of runs in that frame).
    run_count: usize,
}

/// Round an 8-bit glyph alpha to the 3-bit coverage used by the plane.
#[inline]
pub fn quantize_coverage(alpha: u8) -> u8 {
    ((alpha as u16 * 7 + 127) / 255) as u8
}

impl TextPlane {
    pub fn new(width: usize, height: usize) -> Self {
        let bytes = (width * height * 3 + 7) / 8 + 1;
        Self {
            width,
            height,
            origin_y: 0,
            cov: vec![0u8; bytes],
            runs: vec![TextRun::default(); TEXT_RUN_CAPACITY],
            run_count: 0,
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    /// Bytes held by the packed coverage plane and the run table.
    pub fn ram_bytes(&self) -> usize {
        self.cov.len() + self.runs.len() * core::mem::size_of::<TextRun>()
    }

    /// 3-bit coverage at an absolute pixel.
    #[inline]
    pub fn coverage(&self, x: usize, y: usize) -> u8 {
        self.get3(y * self.width + x)
    }

    #[inline]
    fn get3(&self, idx: usize) -> u8 {
        let bit = idx * 3;
        let byte = bit >> 3;
        let shift = bit & 7;
        let word = self.cov[byte] as u16 | ((self.cov[byte + 1] as u16) << 8);
        ((word >> shift) & 7) as u8
    }

    #[inline]
    fn set3(&mut self, idx: usize, value: u8) {
        let bit = idx * 3;
        let byte = bit >> 3;
        let shift = bit & 7;
        let word = self.cov[byte] as u16 | ((self.cov[byte + 1] as u16) << 8);
        let mask = 7u16 << shift;
        let word = (word & !mask) | (((value & 7) as u16) << shift);
        self.cov[byte] = word as u8;
        self.cov[byte + 1] = (word >> 8) as u8;
    }

    /// Start a new isolated band pass at absolute row `origin_y`.
    /// Kept for callers that intentionally rebuild the run table from scratch.
    pub fn begin_band(&mut self, origin_y: usize) {
        self.origin_y = origin_y;
        self.run_count = 0;
    }

    /// Begin repainting one band-local region. Old color runs are clipped to
    /// the unchanged area outside it, and pixel coverage inside it is cleared.
    /// This preserves outside colors without retaining stale metadata.
    pub fn begin_region(
        &mut self,
        origin_y: usize,
        x0: usize,
        y0: usize,
        x1: usize,
        y1: usize,
    ) {
        self.origin_y = origin_y;
        let x0 = x0.min(self.width) as i32;
        let x1 = x1.min(self.width) as i32;
        let local_y0 = y0.min(self.height.saturating_sub(origin_y));
        let local_y1 = y1.min(self.height.saturating_sub(origin_y));
        let abs_y0 = origin_y.saturating_add(local_y0).min(self.height) as i32;
        let abs_y1 = origin_y.saturating_add(local_y1).min(self.height) as i32;
        if x0 >= x1 || abs_y0 >= abs_y1 {
            return;
        }

        // Remove only the part of each old color run covered by this repaint.
        // Runs extending outside the clip are split into up to four pieces;
        // this preserves their old color outside while preventing stale runs
        // from accumulating after repeated small updates. The fixed run buffer
        // is reused in place, so this path performs no allocation.
        let mut count = self.run_count.min(self.runs.len());
        let mut read = 0usize;
        while read < count {
            let run = self.runs[read];
            let rx0 = run.x0 as i32;
            let ry0 = run.y0 as i32;
            let rx1 = run.x1 as i32;
            let ry1 = run.y1 as i32;
            let ix0 = rx0.max(x0);
            let iy0 = ry0.max(abs_y0);
            let ix1 = rx1.min(x1);
            let iy1 = ry1.min(abs_y1);
            if ix0 >= ix1 || iy0 >= iy1 {
                read += 1;
                continue;
            }

            let mut pieces = [TextRun::default(); 4];
            let mut piece_count = 0usize;
            if ry0 < iy0 {
                pieces[piece_count] = TextRun {
                    x0: run.x0,
                    y0: run.y0,
                    x1: run.x1,
                    y1: iy0 as i16,
                    color: run.color,
                };
                piece_count += 1;
            }
            if iy1 < ry1 {
                pieces[piece_count] = TextRun {
                    x0: run.x0,
                    y0: iy1 as i16,
                    x1: run.x1,
                    y1: run.y1,
                    color: run.color,
                };
                piece_count += 1;
            }
            if rx0 < ix0 {
                pieces[piece_count] = TextRun {
                    x0: run.x0,
                    y0: iy0 as i16,
                    x1: ix0 as i16,
                    y1: iy1 as i16,
                    color: run.color,
                };
                piece_count += 1;
            }
            if ix1 < rx1 {
                pieces[piece_count] = TextRun {
                    x0: ix1 as i16,
                    y0: iy0 as i16,
                    x1: run.x1,
                    y1: iy1 as i16,
                    color: run.color,
                };
                piece_count += 1;
            }

            // If the bounded run table is full, keeping the original run is
            // still safe: coverage inside this clip is cleared below, and any
            // newly drawn text run is appended later and takes precedence.
            let new_count = count - 1 + piece_count;
            if new_count <= self.runs.len() {
                self.runs.copy_within(read + 1..count, read + piece_count);
                for (offset, piece) in pieces.iter().take(piece_count).copied().enumerate() {
                    self.runs[read + offset] = piece;
                }
                count = new_count;
                read += piece_count;
            } else {
                read += 1;
            }
        }
        self.run_count = count;

        // Coverage is pixel data, so clear it independently from color runs.
        // The subsequent clipped render repopulates only the changed pixels.
        self.clear_local_rect(x0 as usize, local_y0, x1 as usize, local_y1);
    }

    /// Register one text draw call. Coordinates are band-local.
    pub fn begin_run(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, color: u8) {
        let abs_y0 = y0 + self.origin_y as i32;
        let abs_y1 = y1 + self.origin_y as i32;
        if self.run_count < self.runs.len() {
            self.runs[self.run_count] = TextRun {
                x0: x0.clamp(-32768, 32767) as i16,
                y0: abs_y0.clamp(-32768, 32767) as i16,
                x1: x1.clamp(-32768, 32767) as i16,
                y1: abs_y1.clamp(-32768, 32767) as i16,
                color,
            };
        }
        self.run_count += 1;
    }

    /// Color of the most recently registered run that covers this pixel.
    pub fn color_at(&self, x: usize, y: usize) -> Option<u8> {
        let count = self.run_count.min(self.runs.len());
        let (px, py) = (x as i32, y as i32);
        self.runs[..count]
            .iter()
            .rev()
            .find(|run| {
                px >= run.x0 as i32 && px < run.x1 as i32 && py >= run.y0 as i32 && py < run.y1 as i32
            })
            .map(|run| run.color)
    }

    /// Store coverage for a band-local pixel with max-combine, so overlapping
    /// glyph edges keep the stronger coverage. Out-of-frame pixels are ignored.
    #[inline]
    pub fn put_coverage(&mut self, x: usize, local_y: usize, coverage: u8) {
        let y = local_y + self.origin_y;
        if x >= self.width || y >= self.height || coverage == 0 {
            return;
        }
        let idx = y * self.width + x;
        if self.get3(idx) < coverage {
            self.set3(idx, coverage);
        }
    }

    /// Clear coverage in a band-local rectangle (clipped to the frame).
    pub fn clear_local_rect(&mut self, x0: usize, y0: usize, x1: usize, y1: usize) {
        let x1 = x1.min(self.width);
        let y0 = y0 + self.origin_y;
        let y1 = (y1 + self.origin_y).min(self.height);
        for y in y0..y1 {
            for x in x0.min(x1)..x1 {
                let idx = y * self.width + x;
                if self.get3(idx) != 0 {
                    self.set3(idx, 0);
                }
            }
        }
    }

    /// Clear the whole plane and forget the runs of the previous frame.
    pub fn clear_all(&mut self) {
        self.cov.fill(0);
        self.run_count = 0;
    }
}

/// RGB332 (8-bit color) pack used by the device: 3 bits red, 3 bits green,
/// 2 bits blue. Each channel is rounded to the nearest level. Truncation was
/// used before, but it pulls dark grays toward olive (0x3A3A3C became
/// (36,36,0)) and makes some UI colors vanish into their background.
#[inline]
pub fn rgb332_from_rgb(r: u8, g: u8, b: u8) -> u8 {
    let r3 = (r as u16 * 7 + 127) / 255;
    let g3 = (g as u16 * 7 + 127) / 255;
    let b2 = (b as u16 * 3 + 127) / 255;
    ((r3 as u8) << 5) | ((g3 as u8) << 2) | (b2 as u8)
}

#[inline]
pub fn rgb332_from_color(c: Color) -> u8 {
    rgb332_from_rgb(c.r(), c.g(), c.b())
}

/// Expand RGB332 to 8-bit channels. These are the same expansions the PC
/// viewer applies, so what the viewer shows is bit-for-bit the device output.
#[inline]
pub fn rgb332_expand(v: u8) -> (u8, u8, u8) {
    let r3 = (v >> 5) as u16;
    let g3 = ((v >> 2) & 7) as u16;
    let b2 = (v & 3) as u16;
    ((r3 * 255 / 7) as u8, (g3 * 255 / 7) as u8, (b2 * 255 / 3) as u8)
}

/// Blend an RGB332 foreground over an RGB332 background with a 3-bit
/// coverage (0 = background only, 7 = foreground only). Arithmetic is done on
/// the expanded 8-bit channels and the result is re-packed to RGB332.
#[inline]
pub fn rgb332_blend(bg: u8, fg: u8, coverage: u8) -> u8 {
    if coverage == 0 {
        return bg;
    }
    if coverage >= 7 {
        return fg;
    }
    let a = coverage as u16;
    let inv = 7 - a;
    let (br, bg_, bb) = rgb332_expand(bg);
    let (fr, fg_, fb) = rgb332_expand(fg);
    let mix = |f: u8, b: u8| -> u16 { (f as u16 * a + b as u16 * inv + 3) / 7 };
    let r = mix(fr, br);
    let g = mix(fg_, bg_);
    let b = mix(fb, bb);
    // 8-bit channels back to the RGB332 grid (nearest level).
    let r3 = (r * 7 + 127) / 255;
    let g3 = (g * 7 + 127) / 255;
    let b2 = (b * 3 + 127) / 255;
    ((r3 as u8) << 5) | ((g3 as u8) << 2) | (b2 as u8)
}

impl LayerSystem {
    /// Allocate the text plane for a full frame. The layer keeps drawing its
    /// band into the 32-bit buffer, but text goes to the plane instead.
    pub fn enable_text_plane(&mut self, frame_w: usize, frame_h: usize) {
        self.text = Some(TextPlane::new(frame_w, frame_h));
    }

    pub fn text_active(&self) -> bool {
        self.text.is_some()
    }

    pub fn text_plane(&self) -> Option<&TextPlane> {
        self.text.as_ref()
    }

    pub fn text_plane_mut(&mut self) -> Option<&mut TextPlane> {
        self.text.as_mut()
    }

    /// Select the absolute frame row where this band starts. Call before each
    /// full-width band pass; run metadata outside the band is preserved.
    pub fn set_text_origin(&mut self, frame_y: usize) {
        let width = self.width;
        let height = self.height;
        if let Some(plane) = self.text.as_mut() {
            plane.begin_region(frame_y, 0, 0, width, height);
        }
    }

    /// Select an absolute frame row and the exact band-local clip that will be
    /// repainted. Color metadata outside this region remains untouched.
    pub fn set_text_region(
        &mut self,
        frame_y: usize,
        x0: usize,
        y0: usize,
        x1: usize,
        y1: usize,
    ) {
        if let Some(plane) = self.text.as_mut() {
            plane.begin_region(frame_y, x0, y0, x1, y1);
        }
    }

    /// Band-local coverage write, clipped to the current clip rectangle.
    /// Returns false when there is no text plane.
    pub fn text_put_coverage(&mut self, x: i32, y: i32, alpha: u8) -> bool {
        if self.text.is_none() {
            return false;
        }
        let (cx0, cy0, cx1, cy1) = self.clip_bounds();
        if x < cx0 as i32 || y < cy0 as i32 || x >= cx1 as i32 || y >= cy1 as i32 {
            return true;
        }
        let coverage = quantize_coverage(alpha);
        if let Some(plane) = self.text.as_mut() {
            plane.put_coverage(x as usize, y as usize, coverage);
        }
        true
    }

    /// Register a text draw call (band-local bounds) with its color.
    pub fn text_begin_run(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, color: Color) {
        let (clip_x0, clip_y0, clip_x1, clip_y1) = self.clip_bounds();
        let x0 = x0.max(clip_x0 as i32);
        let y0 = y0.max(clip_y0 as i32);
        let x1 = x1.min(clip_x1 as i32);
        let y1 = y1.min(clip_y1 as i32);
        if x0 >= x1 || y0 >= y1 {
            return;
        }
        if let Some(plane) = self.text.as_mut() {
            plane.begin_run(x0, y0, x1, y1, rgb332_from_color(color));
        }
    }

    /// Remove text under a shape that is written directly into the buffer
    /// (icons, blends). Use this from code that bypasses the fill primitives.
    /// Coordinates are band-local and may be negative or past the edge.
    pub fn occlude_text_signed(&mut self, x: i32, y: i32, w: usize, h: usize) {
        if self.text.is_none() || w == 0 || h == 0 {
            return;
        }
        let x0 = x.max(0) as usize;
        let y0 = y.max(0) as usize;
        let x1 = (x as i64 + w as i64).clamp(0, self.width as i64) as usize;
        let y1 = (y as i64 + h as i64).clamp(0, self.height as i64) as usize;
        self.occlude_text(x0, y0, x1, y1);
    }

    /// Remove text under an opaque shape write (band-local rectangle).
    #[inline]
    pub(crate) fn occlude_text(&mut self, x0: usize, y0: usize, x1: usize, y1: usize) {
        if let Some(plane) = self.text.as_mut() {
            let x1 = x1.min(self.width);
            let y1 = y1.min(self.height);
            if x0 < x1 && y0 < y1 {
                plane.clear_local_rect(x0, y0, x1, y1);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packed_coverage_round_trips_every_pixel() {
        let mut plane = TextPlane::new(135, 240);
        plane.begin_band(0);
        // Write a pattern that touches every bit alignment.
        for i in 0..(135 * 240) {
            let x = i % 135;
            let y = i / 135;
            plane.set3(i, ((i * 5 + 3) % 8) as u8);
            assert_eq!(plane.coverage(x, y), ((i * 5 + 3) % 8) as u8);
        }
        for i in 0..(135 * 240) {
            assert_eq!(plane.get3(i), ((i * 5 + 3) % 8) as u8, "pixel {i}");
        }
    }

    #[test]
    fn coverage_is_three_bits_and_monotonic() {
        assert_eq!(quantize_coverage(0), 0);
        assert_eq!(quantize_coverage(255), 7);
        let mut last = 0;
        for a in 0..=255u8 {
            let q = quantize_coverage(a);
            assert!(q <= 7);
            assert!(q >= last);
            last = q;
        }
    }

    #[test]
    fn occlusion_clears_only_the_band_rows_it_touches() {
        let mut plane = TextPlane::new(10, 16);
        plane.begin_band(8);
        plane.put_coverage(2, 1, 7); // absolute row 9
        plane.put_coverage(2, 0, 5); // absolute row 8
        plane.clear_local_rect(2, 1, 3, 2);
        assert_eq!(plane.coverage(2, 9), 0);
        assert_eq!(plane.coverage(2, 8), 5);
    }

    #[test]
    fn color_lookup_prefers_latest_run() {
        let mut plane = TextPlane::new(20, 20);
        plane.begin_band(0);
        plane.begin_run(0, 0, 10, 10, 0xE0);
        plane.begin_run(5, 5, 15, 15, 0x1C);
        assert_eq!(plane.color_at(2, 2), Some(0xE0));
        assert_eq!(plane.color_at(7, 7), Some(0x1C));
        assert_eq!(plane.color_at(18, 18), None);
    }

    #[test]
    fn partial_region_keeps_unmodified_text_colors() {
        let mut plane = TextPlane::new(20, 20);
        plane.begin_band(0);
        plane.begin_run(0, 1, 8, 6, 0xE0);
        plane.begin_region(8, 0, 0, 20, 8);
        plane.begin_run(10, 1, 18, 6, 0x1C);
        assert_eq!(plane.color_at(2, 2), Some(0xE0));
        assert_eq!(plane.color_at(12, 10), Some(0x1C));

        // Repainting a small part removes fully-contained runs only.
        plane.begin_region(0, 0, 0, 9, 8);
        assert_eq!(plane.color_at(2, 2), None);
        assert_eq!(plane.color_at(12, 10), Some(0x1C));
        plane.begin_run(0, 1, 8, 6, 0x20);
        assert_eq!(plane.color_at(2, 2), Some(0x20));
    }

    #[test]
    fn partial_repaint_preserves_color_outside_clip() {
        let mut plane = TextPlane::new(20, 8);
        plane.begin_band(0);
        plane.begin_run(0, 1, 20, 6, 0xE0);
        plane.put_coverage(5, 2, 7);
        plane.put_coverage(15, 2, 7);
        plane.begin_region(0, 0, 0, 10, 8);
        assert_eq!(plane.coverage(5, 2), 0);
        assert_eq!(plane.coverage(15, 2), 7);
        assert_eq!(plane.color_at(5, 2), None);
        assert_eq!(plane.color_at(15, 2), Some(0xE0));
        plane.begin_run(0, 1, 10, 6, 0x1C);
        assert_eq!(plane.color_at(5, 2), Some(0x1C));
        assert_eq!(plane.color_at(15, 2), Some(0xE0));
    }

    #[test]
    fn layer_text_run_is_clipped_to_active_region() {
        use crate::layer::LayerSystem;
        let mut layer = LayerSystem::new(20, 8);
        layer.enable_text_plane(20, 8);
        layer.push_clip(0, 0, 10, 8);
        layer.text_begin_run(0, 1, 20, 6, Color::rgb(255, 0, 0));
        layer.pop_clip();
        let plane = layer.text_plane().unwrap();
        assert!(plane.color_at(5, 2).is_some());
        assert_eq!(plane.color_at(15, 2), None);
    }

    #[test]
    fn rgb332_expansion_round_trips() {
        for v in 0..=255u8 {
            let (r, g, b) = rgb332_expand(v);
            assert_eq!(rgb332_from_rgb(r, g, b), v, "value {v:#x}");
        }
    }

    #[test]
    fn blend_endpoints_and_midpoint() {
        let bg = rgb332_from_rgb(255, 255, 255);
        let fg = rgb332_from_rgb(0, 0, 0);
        assert_eq!(rgb332_blend(bg, fg, 0), bg);
        assert_eq!(rgb332_blend(bg, fg, 7), fg);
        let mid = rgb332_blend(bg, fg, 3);
        let (r, _, _) = rgb332_expand(mid);
        assert!(r > 80 && r < 180, "mid red {r}");
    }

    #[test]
    fn later_opaque_shape_hides_earlier_text_but_translucent_does_not() {
        use crate::layer::LayerSystem;
        let mut layer = LayerSystem::new(20, 8);
        layer.enable_text_plane(20, 16);
        layer.set_text_origin(8);
        // Text first (band row 0..8 of the second band).
        layer.text_begin_run(0, 0, 20, 8, Color::rgb(0, 0, 0));
        assert!(layer.text_put_coverage(3, 2, 255));
        assert!(layer.text_put_coverage(15, 2, 255));
        assert_eq!(layer.text_plane().unwrap().coverage(3, 10), 7);
        // Opaque shape over x 0..10 hides text there only.
        layer.fill_rect(0, 0, 10, 8, Color::rgb(255, 0, 0));
        assert_eq!(layer.text_plane().unwrap().coverage(3, 10), 0);
        assert_eq!(layer.text_plane().unwrap().coverage(15, 10), 7);
        // A translucent write keeps the text visible.
        layer.text_put_coverage(15, 3, 255);
        let mut src = LayerSystem::new(4, 4);
        src.clear(Color::rgb(0, 0, 0));
        layer.composit_shadow_alpha(&src, 12, 0, 0, 0, 4, 4);
        assert_eq!(layer.text_plane().unwrap().coverage(15, 11), 7);
    }

    #[test]
    fn text_outside_clip_is_not_stored() {
        use crate::layer::LayerSystem;
        let mut layer = LayerSystem::new(20, 8);
        layer.enable_text_plane(20, 8);
        layer.set_text_origin(0);
        layer.push_clip(0, 0, 5, 8);
        layer.text_put_coverage(7, 1, 255);
        layer.text_put_coverage(2, 1, 255);
        layer.pop_clip();
        assert_eq!(layer.text_plane().unwrap().coverage(7, 1), 0);
        assert_eq!(layer.text_plane().unwrap().coverage(2, 1), 7);
    }
}
