//! Dirty-rectangle extraction for a small RGB565 target.

use crate::protocol::Rect;

pub const SCREEN_W: usize = 320;
pub const SCREEN_H: usize = 240;
pub const TILE: usize = 16;

/// One changed rectangle and its tightly packed RGB565 pixels.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DirtyRect {
    pub rect: Rect,
    pub pixels: Vec<u8>,
}

/// Finds changed tiles and merges adjacent changed tiles across scanlines.
///
/// The output is bounded to the number of distinct horizontal runs. A full
/// refresh is one packet, while normal desktop changes stay compact. The
/// caller can send each result immediately without allocating a full-frame
/// wire packet.
pub fn dirty_rects(previous: &[u16], current: &[u16]) -> Vec<DirtyRect> {
    assert_eq!(previous.len(), SCREEN_W * SCREEN_H);
    assert_eq!(current.len(), SCREEN_W * SCREEN_H);

    let tiles_x = SCREEN_W / TILE;
    let tiles_y = SCREEN_H / TILE;
    let mut changed = vec![false; tiles_x * tiles_y];

    for ty in 0..tiles_y {
        for tx in 0..tiles_x {
            let mut different = false;
            'pixels: for y in ty * TILE..(ty + 1) * TILE {
                let start = y * SCREEN_W + tx * TILE;
                if previous[start..start + TILE] != current[start..start + TILE] {
                    different = true;
                    break 'pixels;
                }
            }
            changed[ty * tiles_x + tx] = different;
        }
    }

    // Keep runs alive while the next tile row has the same x/width span. The
    // firmware accepts rectangles taller than one tile and splits them into
    // LCD DMA stripes, so this removes a large amount of packet overhead.
    let mut active: Vec<(usize, usize, usize, usize)> = Vec::new();
    let mut result_runs: Vec<(usize, usize, usize, usize)> = Vec::new();
    for ty in 0..tiles_y {
        let mut tx = 0;
        let mut runs = Vec::new();
        while tx < tiles_x {
            if !changed[ty * tiles_x + tx] {
                tx += 1;
                continue;
            }
            let start_tx = tx;
            while tx + 1 < tiles_x && changed[ty * tiles_x + tx + 1] {
                tx += 1;
            }
            let width = (tx - start_tx + 1) * TILE;
            let x = start_tx * TILE;
            runs.push((x, width));
            tx += 1;
        }

        let mut next = Vec::with_capacity(runs.len());
        for mut run in active.drain(..) {
            if let Some(index) = runs.iter().position(|candidate| *candidate == (run.0, run.1)) {
                runs.swap_remove(index);
                run.3 += TILE;
                next.push(run);
            } else {
                result_runs.push(run);
            }
        }
        for (x, width) in runs {
            next.push((x, width, ty * TILE, TILE));
        }
        active = next;
    }

    result_runs.extend(active);
    let mut result = Vec::with_capacity(result_runs.len());
    for (x, width, y, height) in result_runs {
        let mut pixels = Vec::with_capacity(width * height * 2);
        for row in y..y + height {
            for &pixel in &current[row * SCREEN_W + x..row * SCREEN_W + x + width] {
                pixels.extend_from_slice(&pixel.to_be_bytes());
            }
        }
        result.push(DirtyRect {
            rect: Rect {
                x: x as u16,
                y: y as u16,
                w: width as u16,
                h: height as u16,
            },
            pixels,
        });
    }
    result
}

/// Converts compositor XRGB8888 memory (`B,G,R,X` on little-endian hosts) to
/// the ST7789's big-endian RGB565 wire order.
pub fn xrgb8888_to_rgb565(src: &[u8], src_stride: usize, src_w: usize, src_h: usize) -> Vec<u16> {
    let mut out = vec![0u16; SCREEN_W * SCREEN_H];
    for y in 0..SCREEN_H {
        let sy = y * src_h / SCREEN_H;
        for x in 0..SCREEN_W {
            let sx = x * src_w / SCREEN_W;
            let p = sy * src_stride + sx * 4;
            let b = src[p] as u16;
            let g = src[p + 1] as u16;
            let r = src[p + 2] as u16;
            out[y * SCREEN_W + x] = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unchanged_frame_has_no_rectangles() {
        let frame = vec![0x1234; SCREEN_W * SCREEN_H];
        assert!(dirty_rects(&frame, &frame).is_empty());
    }

    #[test]
    fn one_tile_is_sent_as_one_rect() {
        let previous = vec![0u16; SCREEN_W * SCREEN_H];
        let mut current = previous.clone();
        current[3 * SCREEN_W + 5] = 0xffff;
        let rects = dirty_rects(&previous, &current);
        assert_eq!(rects.len(), 1);
        assert_eq!(
            rects[0].rect,
            Rect {
                x: 0,
                y: 0,
                w: 16,
                h: 16
            }
        );
        assert_eq!(rects[0].pixels.len(), 16 * 16 * 2);
    }

    #[test]
    fn downscale_converts_rgb_channels() {
        let src = vec![0u8, 0, 255, 0];
        let out = xrgb8888_to_rgb565(&src, 4, 1, 1);
        assert_eq!(out[0], 0xf800);
    }
}
