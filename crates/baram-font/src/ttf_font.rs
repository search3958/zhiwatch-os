extern crate alloc;

use alloc::vec;
use alloc::vec::Vec;
use stb_truetype_rust::*;

const FONT_SIZE: f32 = 14.0;

/// Upper bound on cached glyphs. The UI uses only a few pixel sizes, so the
/// cache normally holds one entry per distinct character and size. When the
/// bound is reached the cache is dropped and refilled on demand; this is
/// cheaper on the device than tracking recency for every lookup.
const MAX_CACHED_GLYPHS: usize = 192;

static mut FONT_DATA: Option<Vec<u8>> = None;
static mut FONT_INFO: Option<stbtt_fontinfo> = None;
static mut ASCENT: i32 = 0;
/// One entry per (character, pixel size). Coverage is stored as 3-bit values,
/// packed three bits per pixel, which is enough for the anti-aliasing levels
/// the text plane keeps.
static mut CACHE: Vec<GlyphEntry> = Vec::new();
/// Unpacked alpha for the glyph being painted. Reused across calls so painting
/// does not allocate per glyph.
static mut SCRATCH: Vec<u8> = Vec::new();

struct GlyphEntry {
    ch: char,
    size_bits: u32,
    w: i32,
    h: i32,
    advance: i32,
    y_off: i32,
    packed: Vec<u8>,
}

/// Initialize the primary UI face from font bytes.
///
/// The bytes are copied into RAM (heap), so the parser and glyph rasterizer
/// never touch flash. Passing the bytes in keeps this crate free of asset
/// paths: the application decides which font file is shipped.
pub fn init_with_data(data: &[u8]) {
    unsafe {
        FONT_DATA = Some(Vec::from(data));
        let font_vec = FONT_DATA.as_ref().unwrap();
        let mut info = stbtt_fontinfo::default();
        if stbtt_InitFont(&mut info, font_vec.as_ptr(), 0) == 0 {
            FONT_DATA = None;
            return;
        }
        let scale = stbtt_ScaleForPixelHeight(&mut info, FONT_SIZE);
        ASCENT = scaled_ascent(&mut info, scale);
        FONT_INFO = Some(info);
        CACHE = Vec::new();
        SCRATCH = Vec::new();
    }
}

/// Default pixel size used by `ascent`, `glyph`, `advance` and `with_glyph`.
pub fn default_size() -> f32 {
    FONT_SIZE
}

/// Distance from ascender to descender at `pixel_size`, in pixels.
pub fn line_height_at_size(pixel_size: f32) -> i32 {
    unsafe {
        let Some(info) = FONT_INFO.as_mut() else {
            return 0;
        };
        let scale = stbtt_ScaleForPixelHeight(info, pixel_size);
        let mut ascent = 0;
        let mut descent = 0;
        let mut line_gap = 0;
        stbtt_GetFontVMetrics(info, &mut ascent, &mut descent, &mut line_gap);
        ((ascent - descent) as f32 * scale + 0.5) as i32
    }
}

unsafe fn scaled_ascent(info: &mut stbtt_fontinfo, scale: f32) -> i32 {
    let mut ascent = 0;
    let mut descent = 0;
    let mut line_gap = 0;
    stbtt_GetFontVMetrics(info, &mut ascent, &mut descent, &mut line_gap);
    (ascent as f32 * scale + 0.5) as i32
}

unsafe fn primary_ascent_at_size(pixel_size: f32) -> i32 {
    let Some(info) = FONT_INFO.as_mut() else {
        return 0;
    };
    let scale = stbtt_ScaleForPixelHeight(info, pixel_size);
    scaled_ascent(info, scale)
}

pub fn is_available() -> bool {
    unsafe { FONT_INFO.is_some() }
}

pub fn ascent() -> i32 {
    unsafe { ASCENT }
}

pub fn ascent_at_size(pixel_size: f32) -> i32 {
    unsafe {
        if FONT_INFO.is_none() {
            return 0;
        }
        primary_ascent_at_size(pixel_size)
    }
}

pub struct GlyphBitmap {
    pub data: Vec<u8>,
    pub w: i32,
    pub h: i32,
    pub advance: i32,
    pub y_off: i32,
}

#[inline]
fn coverage_at(packed: &[u8], index: usize) -> u8 {
    let bit = index * 3;
    let byte = bit >> 3;
    let shift = bit & 7;
    let word = packed[byte] as u16 | ((packed[byte + 1] as u16) << 8);
    ((word >> shift) & 7) as u8
}

/// Pack 8-bit alpha into 3-bit coverage, three bits per pixel.
fn pack_coverage(alpha: &[u8]) -> Vec<u8> {
    let mut out = vec![0u8; (alpha.len() * 3 + 7) / 8 + 1];
    for (index, &a) in alpha.iter().enumerate() {
        let coverage = (a as u16 * 7 + 127) / 255;
        let bit = index * 3;
        let byte = bit >> 3;
        let shift = bit & 7;
        let word = out[byte] as u16 | ((out[byte + 1] as u16) << 8);
        let word = word | (coverage << shift);
        out[byte] = word as u8;
        out[byte + 1] = (word >> 8) as u8;
    }
    out
}

/// Expand packed coverage to 8-bit alpha. Re-quantizing the result gives back
/// the same 3-bit coverage, so callers see the stored glyph exactly.
fn unpack_alpha(packed: &[u8], count: usize, out: &mut Vec<u8>) {
    out.clear();
    for index in 0..count {
        let coverage = coverage_at(packed, index) as u16;
        out.push((coverage * 255 / 7) as u8);
    }
}

/// Resolve a character to a glyph index in the primary face.
unsafe fn glyph_index(ch: char) -> Option<(*mut stbtt_fontinfo, i32)> {
    let info = FONT_INFO.as_mut()?;
    let glyph_id = stbtt_FindGlyphIndex(info, ch as i32);
    if glyph_id != 0 || ch == '\0' {
        Some((info as *mut _, glyph_id))
    } else {
        None
    }
}

/// Find the cache entry for (ch, size), rasterizing it on first use. Missing
/// glyphs are cached too, so the font is not searched again for them.
unsafe fn cache_index(ch: char, pixel_size: f32) -> usize {
    let size_bits = pixel_size.to_bits();
    if let Some(index) = CACHE
        .iter()
        .position(|entry| entry.ch == ch && entry.size_bits == size_bits)
    {
        return index;
    }

    let mut entry = GlyphEntry {
        ch,
        size_bits,
        w: 0,
        h: 0,
        advance: 0,
        y_off: 0,
        packed: Vec::new(),
    };
    if let Some((info_ptr, glyph_id)) = glyph_index(ch) {
        let info = &mut *info_ptr;
        let scale = stbtt_ScaleForPixelHeight(info, pixel_size);
        let mut advance = 0;
        let mut lsb = 0;
        stbtt_GetGlyphHMetrics(info, glyph_id, &mut advance, &mut lsb);
        entry.advance = (advance as f32 * scale + 0.5) as i32;

        let mut x0 = 0;
        let mut y0 = 0;
        let mut x1 = 0;
        let mut y1 = 0;
        stbtt_GetGlyphBitmapBox(info, glyph_id, scale, scale, &mut x0, &mut y0, &mut x1, &mut y1);
        let w = x1 - x0;
        let h = y1 - y0;
        if w > 0 && h > 0 {
            // The raster temporary is freed as soon as it is packed.
            let mut alpha = vec![0u8; (w * h) as usize];
            stbtt_MakeGlyphBitmap(info, alpha.as_mut_ptr(), w, h, w, scale, scale, glyph_id);
            entry.packed = pack_coverage(&alpha);
            entry.w = w;
            entry.h = h;
            entry.y_off = y0;
        }
    }

    if CACHE.len() >= MAX_CACHED_GLYPHS {
        CACHE.clear();
    }
    CACHE.push(entry);
    CACHE.len() - 1
}

/// Borrow a glyph at `pixel_size` for immediate painting. The alpha slice is
/// valid only inside `paint`, and it is unpacked from the 3-bit cache into a
/// reused buffer. `paint` must not call back into this module.
pub fn with_glyph_at_size<R>(
    ch: char,
    pixel_size: f32,
    paint: impl FnOnce(&[u8], i32, i32, i32, i32) -> R,
) -> R {
    unsafe {
        if FONT_INFO.is_none() {
            return paint(&[], 0, 0, 0, 0);
        }
        let index = cache_index(ch, pixel_size);
        let entry = &CACHE[index];
        let (w, h, advance, y_off) = (entry.w, entry.h, entry.advance, entry.y_off);
        let count = (w.max(0) * h.max(0)) as usize;
        let scratch = &mut *core::ptr::addr_of_mut!(SCRATCH);
        unpack_alpha(&entry.packed, count, scratch);
        paint(scratch.as_slice(), w, h, advance, y_off)
    }
}

/// Owned copy of a glyph at `pixel_size`, for callers that need the bitmap
/// outside a paint call.
pub fn glyph_at_size(ch: char, pixel_size: f32) -> GlyphBitmap {
    with_glyph_at_size(ch, pixel_size, |data, w, h, advance, y_off| GlyphBitmap {
        data: data.to_vec(),
        w,
        h,
        advance,
        y_off,
    })
}

/// Owned copy of a glyph at the default size.
pub fn glyph(ch: char) -> GlyphBitmap {
    glyph_at_size(ch, FONT_SIZE)
}

/// Advance width at the default size. Layout uses this on every character, so
/// it reads the cache without copying any bitmap.
pub fn advance(ch: char) -> i32 {
    unsafe {
        if FONT_INFO.is_none() {
            return 8;
        }
        let index = cache_index(ch, FONT_SIZE);
        CACHE[index].advance
    }
}

/// Borrow a cached glyph at the default size for immediate painting.
pub fn with_glyph<R>(ch: char, paint: impl FnOnce(&[u8], i32, i32, i32, i32) -> R) -> R {
    with_glyph_at_size(ch, FONT_SIZE, paint)
}
