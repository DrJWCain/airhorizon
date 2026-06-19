//! Bitmap font atlas + text layout for map labels, via fontdue.
//!
//! Rasterises printable ASCII once into a single R8 coverage atlas (shelf
//! packed), records each glyph's atlas UVs and metrics, and lays a string out
//! into screen-space textured quads. Labels are drawn screen-aligned at a fixed
//! pixel size, so they don't scale with map zoom.

use std::collections::HashMap;

use bytemuck::{Pod, Zeroable};
use fontdue::{Font, FontSettings};

/// Screen-space textured vertex for label glyphs (and solid marker triangles,
/// which sample a reserved fully-opaque texel).
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct TextVertex {
    pub pos: [f32; 2], // screen pixels (origin top-left)
    pub uv: [f32; 2],
    pub color: [f32; 3],
}

#[derive(Clone, Copy)]
struct Glyph {
    uv0: [f32; 2],
    uv1: [f32; 2],
    w: f32,
    h: f32,
    xmin: f32,
    ymin: f32,
    advance: f32,
}

pub struct FontAtlas {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>, // R8 coverage, row-major width*height
    pub px: f32,
    glyphs: HashMap<char, Glyph>,
}

impl FontAtlas {
    /// Build an atlas from TTF/OTF bytes at `px` raster size.
    pub fn build(font_bytes: &[u8], px: f32) -> Self {
        let font = Font::from_bytes(font_bytes, FontSettings::default()).expect("load font");
        // 512 wide keeps R8 rows a multiple of 256 (no write_texture padding).
        let atlas_w = 512u32;
        let pad = 1u32;
        let rasters: Vec<(char, (fontdue::Metrics, Vec<u8>))> =
            (32u8..=126).map(|c| (c as char, font.rasterize(c as char, px))).collect();

        let mut glyphs: HashMap<char, Glyph> = HashMap::new();
        let mut place: Vec<(usize, u32, u32)> = Vec::new(); // (raster idx, x, y)
        let (mut x, mut y, mut row_h) = (pad, pad, 0u32);
        for (i, (ch, (m, _))) in rasters.iter().enumerate() {
            let (w, h) = (m.width as u32, m.height as u32);
            if w == 0 || h == 0 {
                // Whitespace etc.: advance only, no bitmap.
                glyphs.insert(
                    *ch,
                    Glyph {
                        uv0: [0.0; 2],
                        uv1: [0.0; 2],
                        w: 0.0,
                        h: 0.0,
                        xmin: m.xmin as f32,
                        ymin: m.ymin as f32,
                        advance: m.advance_width,
                    },
                );
                continue;
            }
            if x + w + pad > atlas_w {
                x = pad;
                y += row_h + pad;
                row_h = 0;
            }
            place.push((i, x, y));
            row_h = row_h.max(h);
            x += w + pad;
        }
        let atlas_h = y + row_h + pad;
        let mut pixels = vec![0u8; (atlas_w * atlas_h) as usize];
        for (i, gx, gy) in place {
            let ch = rasters[i].0;
            let (m, bmp) = &rasters[i].1;
            let (w, h) = (m.width, m.height);
            for row in 0..h {
                for col in 0..w {
                    pixels[((gy + row as u32) * atlas_w + (gx + col as u32)) as usize] =
                        bmp[row * w + col];
                }
            }
            glyphs.insert(
                ch,
                Glyph {
                    uv0: [gx as f32 / atlas_w as f32, gy as f32 / atlas_h as f32],
                    uv1: [
                        (gx + w as u32) as f32 / atlas_w as f32,
                        (gy + h as u32) as f32 / atlas_h as f32,
                    ],
                    w: w as f32,
                    h: h as f32,
                    xmin: m.xmin as f32,
                    ymin: m.ymin as f32,
                    advance: m.advance_width,
                },
            );
        }
        // Reserve texel (0,0) as a fully-opaque pixel so marker triangles can
        // sample it through the same pipeline. Glyph packing starts at (1,1),
        // and row/col 0 are padding, so this texel is never touched by glyphs.
        pixels[0] = 255;

        FontAtlas { width: atlas_w, height: atlas_h, pixels, px, glyphs }
    }

    /// UV of the reserved fully-opaque texel — used to draw solid shapes
    /// (peak markers) with the text pipeline.
    pub fn solid_uv(&self) -> [f32; 2] {
        [0.5 / self.width as f32, 0.5 / self.height as f32]
    }

    /// Total advance width of `s` in pixels.
    pub fn measure(&self, s: &str) -> f32 {
        s.chars()
            .map(|c| self.glyphs.get(&c).map(|g| g.advance).unwrap_or(self.px * 0.5))
            .sum()
    }

    /// Lay `s` out with its left edge at `pen` and text baseline at `baseline`
    /// (screen pixels) in `color`, appending two triangles per visible glyph.
    pub fn layout(&self, s: &str, mut pen: f32, baseline: f32, color: [f32; 3], out: &mut Vec<TextVertex>) {
        for c in s.chars() {
            let Some(g) = self.glyphs.get(&c) else {
                pen += self.px * 0.5;
                continue;
            };
            if g.w > 0.0 && g.h > 0.0 {
                let left = pen + g.xmin;
                let top = baseline - g.ymin - g.h; // ymin is bottom edge above baseline (y up)
                let (right, bottom) = (left + g.w, top + g.h);
                let (u0, v0, u1, v1) = (g.uv0[0], g.uv0[1], g.uv1[0], g.uv1[1]);
                out.push(TextVertex { pos: [left, top], uv: [u0, v0], color });
                out.push(TextVertex { pos: [left, bottom], uv: [u0, v1], color });
                out.push(TextVertex { pos: [right, top], uv: [u1, v0], color });
                out.push(TextVertex { pos: [right, top], uv: [u1, v0], color });
                out.push(TextVertex { pos: [left, bottom], uv: [u0, v1], color });
                out.push(TextVertex { pos: [right, bottom], uv: [u1, v1], color });
            }
            pen += g.advance;
        }
    }

    /// Append a small upward-pointing solid triangle centred at (cx, cy), used
    /// as a peak marker. Samples the reserved opaque texel.
    pub fn marker(&self, cx: f32, cy: f32, size: f32, color: [f32; 3], out: &mut Vec<TextVertex>) {
        let uv = self.solid_uv();
        out.push(TextVertex { pos: [cx, cy - size], uv, color });
        out.push(TextVertex { pos: [cx - size * 0.85, cy + size * 0.7], uv, color });
        out.push(TextVertex { pos: [cx + size * 0.85, cy + size * 0.7], uv, color });
    }
}
