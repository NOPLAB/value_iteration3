//! P5 (binary) PGM + ROS `map_server` YAML loader and occupancy → penalty
//! conversion, used by the `bench_map` binary.
//!
//! vi_rs has no map loader of its own (the C host has `map_pgm.c`; the
//! `vi_node` ROS bridge takes a pre-parsed `OccupancyGridView`). `bench_map`
//! needs to read a real `.pgm`/`.yaml` pair directly, so the parsing lives
//! here where it can be unit-tested without ROS or the FPGA toolchain.
//!
//! ## Obstacle convention
//!
//! This follows the standard ROS `map_server` reading, NOT the C
//! `host/src/penalty.c` reading. `penalty.c` treats high pixel intensity as
//! obstacle because it is tuned for *cost* maps (`cost.pgm`); applying it to a
//! SLAM occupancy map (white = free, gray 205 = unknown, black = occupied)
//! would mark ~99% of cells as obstacles. Instead we compute
//! `occ = (255 - pixel) / 255` (with `negate` honoured) and classify against
//! `occupied_thresh` / `free_thresh`.

use ndarray::Array2;
use std::path::Path;

use vi_core::{Penalty, PENALTY_OBSTACLE};

/// Per-cell occupancy class derived from a pixel value and the YAML thresholds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Occupancy {
    /// `occ >= occupied_thresh` — impassable.
    Obstacle,
    /// `occ < free_thresh` — navigable.
    Free,
    /// Between the two thresholds — unscanned / ambiguous.
    Unknown,
}

/// Parsed YAML metadata (the subset `map_server` defines that we use).
#[derive(Debug, Clone, PartialEq)]
pub struct MapMeta {
    pub image: String,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub occupied_thresh: f64,
    pub free_thresh: f64,
    pub negate: bool,
}

/// A loaded map: YAML metadata plus the raw P5 pixel grid (`pixels[iy*w + ix]`,
/// row 0 = top of the image, as stored on disk).
#[derive(Debug, Clone)]
pub struct PgmMap {
    pub width: usize,
    pub height: usize,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub occupied_thresh: f64,
    pub free_thresh: f64,
    pub negate: bool,
    pub pixels: Vec<u8>,
}

/// Parse the `key: value` subset of a `map_server` YAML document.
///
/// Recognises `image`, `resolution`, `origin` (`[x, y, yaw]`),
/// `occupied_thresh`, `free_thresh`, `negate`. Missing thresholds default to
/// the `map_server` defaults (0.65 / 0.196); missing `negate` defaults to 0.
/// `image`, `resolution`, and `origin` are required.
pub fn parse_yaml(text: &str) -> Result<MapMeta, String> {
    fn find(text: &str, key: &str) -> Option<String> {
        for line in text.lines() {
            let line = line.trim_start();
            if let Some(rest) = line.strip_prefix(key) {
                let rest = rest.trim_start();
                if let Some(val) = rest.strip_prefix(':') {
                    // Strip trailing comment, then whitespace.
                    let val = val.split('#').next().unwrap_or("").trim();
                    return Some(val.to_string());
                }
            }
        }
        None
    }

    let image = find(text, "image").ok_or("missing 'image'")?;
    let resolution = find(text, "resolution")
        .ok_or("missing 'resolution'")?
        .parse::<f64>()
        .map_err(|e| format!("bad resolution: {e}"))?;

    let origin_raw = find(text, "origin").ok_or("missing 'origin'")?;
    let nums: Vec<f64> = origin_raw
        .trim_matches(|c| c == '[' || c == ']')
        .split(',')
        .filter_map(|s| s.trim().parse::<f64>().ok())
        .collect();
    if nums.len() < 2 {
        return Err(format!("bad origin: {origin_raw}"));
    }
    let origin_x = nums[0];
    let origin_y = nums[1];

    let occupied_thresh = find(text, "occupied_thresh")
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.65);
    let free_thresh = find(text, "free_thresh")
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.196);
    let negate = find(text, "negate")
        .and_then(|s| s.parse::<i32>().ok())
        .map(|n| n != 0)
        .unwrap_or(false);

    Ok(MapMeta {
        image,
        resolution,
        origin_x,
        origin_y,
        occupied_thresh,
        free_thresh,
        negate,
    })
}

/// Parse a binary (P5) PGM. Returns `(width, height, pixels)` with
/// `pixels.len() == width * height`. Handles `#` comment lines in the header.
/// Only `maxval <= 255` (one byte per pixel) is supported.
pub fn parse_pgm_p5(bytes: &[u8]) -> Result<(usize, usize, Vec<u8>), String> {
    let mut pos = 0usize;

    // Magic.
    let magic = next_token(bytes, &mut pos).ok_or("empty PGM")?;
    if magic != b"P5" {
        return Err("not a binary (P5) PGM".to_string());
    }

    let w = parse_token_usize(bytes, &mut pos, "width")?;
    let h = parse_token_usize(bytes, &mut pos, "height")?;
    let maxval = parse_token_usize(bytes, &mut pos, "maxval")?;
    if maxval > 255 {
        return Err(format!("maxval {maxval} > 255 unsupported"));
    }

    // Exactly one whitespace byte separates the header from the raster.
    if pos < bytes.len() && bytes[pos].is_ascii_whitespace() {
        pos += 1;
    }

    let n = w.checked_mul(h).ok_or("width*height overflow")?;
    if bytes.len() < pos + n {
        return Err(format!(
            "PGM raster too short: need {n} bytes, have {}",
            bytes.len() - pos
        ));
    }
    let pixels = bytes[pos..pos + n].to_vec();
    Ok((w, h, pixels))
}

/// Read the next whitespace-delimited token, skipping `#` comment lines.
fn next_token<'a>(bytes: &'a [u8], pos: &mut usize) -> Option<&'a [u8]> {
    loop {
        while *pos < bytes.len() && bytes[*pos].is_ascii_whitespace() {
            *pos += 1;
        }
        if *pos < bytes.len() && bytes[*pos] == b'#' {
            while *pos < bytes.len() && bytes[*pos] != b'\n' {
                *pos += 1;
            }
            continue;
        }
        break;
    }
    if *pos >= bytes.len() {
        return None;
    }
    let start = *pos;
    while *pos < bytes.len() && !bytes[*pos].is_ascii_whitespace() {
        *pos += 1;
    }
    Some(&bytes[start..*pos])
}

fn parse_token_usize(bytes: &[u8], pos: &mut usize, what: &str) -> Result<usize, String> {
    let tok = next_token(bytes, pos).ok_or_else(|| format!("missing {what}"))?;
    std::str::from_utf8(tok)
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .ok_or_else(|| format!("bad {what}"))
}

/// Load a `map_server` YAML and the PGM it references. The image path is
/// resolved relative to the YAML file's directory unless it is absolute.
pub fn load(yaml_path: &Path) -> Result<PgmMap, String> {
    let yaml_text = std::fs::read_to_string(yaml_path)
        .map_err(|e| format!("read {}: {e}", yaml_path.display()))?;
    let meta = parse_yaml(&yaml_text)?;

    let img_path = Path::new(&meta.image);
    let resolved = if img_path.is_absolute() {
        img_path.to_path_buf()
    } else {
        yaml_path
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .join(img_path)
    };

    let raw = std::fs::read(&resolved).map_err(|e| format!("read {}: {e}", resolved.display()))?;
    let (width, height, pixels) = parse_pgm_p5(&raw)?;

    Ok(PgmMap {
        width,
        height,
        resolution: meta.resolution,
        origin_x: meta.origin_x,
        origin_y: meta.origin_y,
        occupied_thresh: meta.occupied_thresh,
        free_thresh: meta.free_thresh,
        negate: meta.negate,
        pixels,
    })
}

/// Classify one pixel using the `map_server` rule.
///
/// `occ = (255 - p) / 255` for `negate == false` (dark = occupied), or
/// `occ = p / 255` for `negate == true`. `occ >= occupied_thresh` is an
/// obstacle, `occ < free_thresh` is free, anything between is unknown.
pub fn classify(pixel: u8, negate: bool, occupied_thresh: f64, free_thresh: f64) -> Occupancy {
    let p = pixel as f64;
    let occ = if negate { p / 255.0 } else { (255.0 - p) / 255.0 };
    if occ >= occupied_thresh {
        Occupancy::Obstacle
    } else if occ < free_thresh {
        Occupancy::Free
    } else {
        Occupancy::Unknown
    }
}

/// Build a `[height, width]` penalty field from a loaded map.
///
/// - The image is flipped vertically so that grid row `iy = 0` corresponds to
///   world `y = origin_y` (ROS stores occupancy bottom-up; PGM rows run
///   top-down). This keeps grid indices consistent with [`vi_core::make_goal_mask`].
/// - Obstacles (and `Unknown` when `unknown_as_obstacle`) become
///   [`PENALTY_OBSTACLE`]. Everything else starts at 0.
/// - When `safety_radius_cells > 0`, free cells within that chessboard radius
///   of an obstacle are raised to `safety_penalty` (kept below the obstacle
///   sentinel).
pub fn build_penalty(
    map: &PgmMap,
    unknown_as_obstacle: bool,
    safety_radius_cells: usize,
    safety_penalty: u16,
) -> Array2<Penalty> {
    let w = map.width;
    let h = map.height;
    let mut pen = Array2::<Penalty>::zeros((h, w));

    for iy in 0..h {
        let src_row = h - 1 - iy; // vertical flip
        for ix in 0..w {
            let pixel = map.pixels[src_row * w + ix];
            let occ = classify(pixel, map.negate, map.occupied_thresh, map.free_thresh);
            let is_obstacle = matches!(occ, Occupancy::Obstacle)
                || (matches!(occ, Occupancy::Unknown) && unknown_as_obstacle);
            if is_obstacle {
                pen[[iy, ix]] = PENALTY_OBSTACLE;
            }
        }
    }

    if safety_radius_cells > 0 && safety_penalty > 0 {
        dilate(&mut pen, safety_radius_cells, safety_penalty);
    }

    pen
}

/// Raise every free cell within `radius` chessboard cells of an obstacle to
/// `value` (leaving existing obstacles and higher penalties untouched).
fn dilate(pen: &mut Array2<Penalty>, radius: usize, value: u16) {
    let (h, w) = pen.dim();
    let r = radius as isize;
    let snapshot = pen.clone();
    for iy in 0..h {
        for ix in 0..w {
            if snapshot[[iy, ix]] == PENALTY_OBSTACLE {
                continue;
            }
            let mut near = false;
            'scan: for dy in -r..=r {
                for dx in -r..=r {
                    let ny = iy as isize + dy;
                    let nx = ix as isize + dx;
                    if ny < 0 || nx < 0 || ny >= h as isize || nx >= w as isize {
                        continue;
                    }
                    if snapshot[[ny as usize, nx as usize]] == PENALTY_OBSTACLE {
                        near = true;
                        break 'scan;
                    }
                }
            }
            if near && pen[[iy, ix]] < value {
                pen[[iy, ix]] = value;
            }
        }
    }
}

/// Conservatively downsample a penalty field by an integer `scale`. Each
/// output cell takes the **maximum** penalty of its `scale × scale` source
/// block, so obstacles ([`PENALTY_OBSTACLE`] = `0xFFFF`) always dominate and
/// are never erased by neighbouring free space. Output dims are
/// `ceil(dim / scale)`. `scale == 1` returns a clone.
pub fn downsample_penalty(pen: &Array2<Penalty>, scale: usize) -> Array2<Penalty> {
    assert!(scale >= 1, "scale must be >= 1");
    if scale == 1 {
        return pen.clone();
    }
    let (h, w) = pen.dim();
    let nh = h.div_ceil(scale);
    let nw = w.div_ceil(scale);
    let mut out = Array2::<Penalty>::zeros((nh, nw));
    for oy in 0..nh {
        for ox in 0..nw {
            let mut m: Penalty = 0;
            for dy in 0..scale {
                let sy = oy * scale + dy;
                if sy >= h {
                    break;
                }
                for dx in 0..scale {
                    let sx = ox * scale + dx;
                    if sx >= w {
                        break;
                    }
                    let v = pen[[sy, sx]];
                    if v > m {
                        m = v;
                    }
                }
            }
            out[[oy, ox]] = m;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- YAML parsing ---

    #[test]
    fn parse_yaml_tsudanuma_fields() {
        let text = "image: map_tsudanuma.pgm\n\
                    resolution: 0.050000\n\
                    origin: [-100.000000, -100.000000, 0.000000]\n\
                    negate: 0\n\
                    occupied_thresh: 0.65\n\
                    free_thresh: 0.196\n";
        let m = parse_yaml(text).unwrap();
        assert_eq!(m.image, "map_tsudanuma.pgm");
        assert_eq!(m.resolution, 0.05);
        assert_eq!(m.origin_x, -100.0);
        assert_eq!(m.origin_y, -100.0);
        assert_eq!(m.occupied_thresh, 0.65);
        assert_eq!(m.free_thresh, 0.196);
        assert!(!m.negate);
    }

    #[test]
    fn parse_yaml_defaults_thresholds_when_absent() {
        let text = "image: m.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n";
        let m = parse_yaml(text).unwrap();
        assert_eq!(m.occupied_thresh, 0.65);
        assert_eq!(m.free_thresh, 0.196);
        assert!(!m.negate);
    }

    #[test]
    fn parse_yaml_missing_image_errors() {
        let text = "resolution: 0.1\norigin: [0, 0, 0]\n";
        assert!(parse_yaml(text).is_err());
    }

    // --- P5 PGM parsing ---

    fn make_p5(w: usize, h: usize, pixels: &[u8]) -> Vec<u8> {
        let mut v = format!("P5\n{w} {h}\n255\n").into_bytes();
        v.extend_from_slice(pixels);
        v
    }

    #[test]
    fn parse_pgm_p5_roundtrips_dims_and_pixels() {
        let pixels = vec![0u8, 50, 205, 255, 100, 150];
        let raw = make_p5(3, 2, &pixels);
        let (w, h, px) = parse_pgm_p5(&raw).unwrap();
        assert_eq!((w, h), (3, 2));
        assert_eq!(px, pixels);
    }

    #[test]
    fn parse_pgm_p5_skips_comment_line() {
        let mut raw = b"P5\n# GIMP comment\n2 2\n255\n".to_vec();
        raw.extend_from_slice(&[1, 2, 3, 4]);
        let (w, h, px) = parse_pgm_p5(&raw).unwrap();
        assert_eq!((w, h), (2, 2));
        assert_eq!(px, vec![1, 2, 3, 4]);
    }

    #[test]
    fn parse_pgm_p5_rejects_p2_ascii() {
        let raw = b"P2\n2 2\n255\n1 2 3 4\n".to_vec();
        assert!(parse_pgm_p5(&raw).is_err());
    }

    #[test]
    fn parse_pgm_p5_rejects_short_raster() {
        let raw = make_p5(4, 4, &[0, 0, 0]); // only 3 of 16 bytes
        assert!(parse_pgm_p5(&raw).is_err());
    }

    // --- Classification (standard ROS map_server convention) ---

    #[test]
    fn classify_black_is_obstacle() {
        // p=0 -> occ=1.0 >= 0.65
        assert_eq!(classify(0, false, 0.65, 0.196), Occupancy::Obstacle);
    }

    #[test]
    fn classify_white_is_free() {
        // p=255 -> occ=0.0 < 0.196
        assert_eq!(classify(255, false, 0.65, 0.196), Occupancy::Free);
    }

    #[test]
    fn classify_gray_205_is_unknown() {
        // p=205 -> occ=(50)/255=0.196078, not < free_thresh, not >= occupied_thresh
        assert_eq!(classify(205, false, 0.65, 0.196), Occupancy::Unknown);
    }

    #[test]
    fn classify_negate_inverts() {
        // negate: occ = p/255. p=255 -> occ=1.0 -> obstacle.
        assert_eq!(classify(255, true, 0.65, 0.196), Occupancy::Obstacle);
        assert_eq!(classify(0, true, 0.65, 0.196), Occupancy::Free);
    }

    // --- Penalty build ---

    fn map_from_pixels(w: usize, h: usize, pixels: Vec<u8>) -> PgmMap {
        PgmMap {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            occupied_thresh: 0.65,
            free_thresh: 0.196,
            negate: false,
            pixels,
        }
    }

    #[test]
    fn build_penalty_marks_black_as_obstacle() {
        // 2x1: left black (obstacle), right white (free).
        let map = map_from_pixels(2, 1, vec![0, 255]);
        let pen = build_penalty(&map, true, 0, 0);
        assert_eq!(pen[[0, 0]], PENALTY_OBSTACLE);
        assert_eq!(pen[[0, 1]], 0);
    }

    #[test]
    fn build_penalty_unknown_as_obstacle_toggle() {
        let map = map_from_pixels(1, 1, vec![205]); // unknown
        assert_eq!(build_penalty(&map, true, 0, 0)[[0, 0]], PENALTY_OBSTACLE);
        assert_eq!(build_penalty(&map, false, 0, 0)[[0, 0]], 0);
    }

    #[test]
    fn build_penalty_flips_vertically() {
        // 1x2 image: row0 (top) = black, row1 (bottom) = white.
        // After flip, grid row 0 (y=origin_y, bottom) = white = free,
        // grid row 1 (top) = black = obstacle.
        let map = map_from_pixels(1, 2, vec![0, 255]);
        let pen = build_penalty(&map, true, 0, 0);
        assert_eq!(pen[[0, 0]], 0, "bottom grid row is the white image row");
        assert_eq!(pen[[1, 0]], PENALTY_OBSTACLE, "top grid row is the black image row");
    }

    #[test]
    fn build_penalty_dilation_raises_neighbours() {
        // 3x3 all free except center black. radius 1 -> 8 neighbours get safety_penalty.
        let map = map_from_pixels(3, 3, vec![255, 255, 255, 255, 0, 255, 255, 255, 255]);
        let pen = build_penalty(&map, true, 1, 500);
        assert_eq!(pen[[1, 1]], PENALTY_OBSTACLE);
        for (iy, ix) in [(0, 0), (0, 1), (0, 2), (1, 0), (1, 2), (2, 0), (2, 1), (2, 2)] {
            assert_eq!(pen[[iy, ix]], 500, "dilated cell ({iy},{ix})");
        }
    }

    // --- Downsample ---

    #[test]
    fn downsample_scale1_is_clone() {
        let map = map_from_pixels(2, 1, vec![0, 255]);
        let pen = build_penalty(&map, true, 0, 0);
        let ds = downsample_penalty(&pen, 1);
        assert_eq!(ds, pen);
    }

    #[test]
    fn downsample_obstacle_dominates_block() {
        // 2x2 penalty, one obstacle in the block -> single output cell = obstacle.
        let mut pen = Array2::<Penalty>::zeros((2, 2));
        pen[[1, 1]] = PENALTY_OBSTACLE;
        let ds = downsample_penalty(&pen, 2);
        assert_eq!(ds.dim(), (1, 1));
        assert_eq!(ds[[0, 0]], PENALTY_OBSTACLE);
    }

    #[test]
    fn downsample_ceil_dims_for_non_multiple() {
        let pen = Array2::<Penalty>::zeros((3, 5));
        let ds = downsample_penalty(&pen, 2);
        assert_eq!(ds.dim(), (2, 3)); // ceil(3/2)=2, ceil(5/2)=3
    }
}
