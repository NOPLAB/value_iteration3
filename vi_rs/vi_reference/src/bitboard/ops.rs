//! Bitwise operations on [`Bitboard2D`] and [`Bitboard3D`].
//!
//! Algorithms mirror `vi_matlab/src/shared/bitboard/bb_dilate2d.m`,
//! `bb_dilate3d.m`, `bb_shift_row.m`, `bb_popcount.m`, `bb_row_mask.m`.

use super::{row_mask, Bitboard2D, Bitboard3D};

// ---------------------------------------------------------------------------
// Slice-level helpers (shared by 2D and 3D)
// ---------------------------------------------------------------------------

#[inline]
pub(crate) fn popcount_slice(data: &[u64]) -> u64 {
    data.iter().map(|w| w.count_ones() as u64).sum()
}

#[inline]
pub(crate) fn and_slice(a: &mut [u64], b: &[u64]) {
    for (x, y) in a.iter_mut().zip(b.iter()) {
        *x &= y;
    }
}

#[inline]
pub(crate) fn or_slice(a: &mut [u64], b: &[u64]) {
    for (x, y) in a.iter_mut().zip(b.iter()) {
        *x |= y;
    }
}

/// Row layout for [`dilate2d_into`] (constant across all rows of a map):
/// rows per map, words per row, and the trailing-bit row mask.
#[derive(Clone, Copy)]
pub(crate) struct RowLayout<'a> {
    pub map_y: usize,
    pub wpr: usize,
    pub rmask: &'a [u64],
}

/// Row-sized scratch buffers reused across [`dilate2d_into`] calls to avoid
/// per-call allocation (one `dilate3d` shares a single `Scratch` over all layers).
#[derive(Default)]
pub(crate) struct Scratch {
    pub pos: Vec<u64>,
    pub neg: Vec<u64>,
}

// ---------------------------------------------------------------------------
// shift_row
// ---------------------------------------------------------------------------

/// Horizontal big-integer shift of one row by `sx` bit positions.
///
/// Writes the shifted, masked result into `out`.
/// - Positive `sx`: shifts bits toward higher x indices (left in memory-word
///   sense, i.e. higher bit positions).
/// - Negative `sx`: shifts toward lower x indices.
/// - Constraint: `-64 < sx < 64`.
/// - `out.len() == row.len() == row_mask.len()` (debug-asserted).
///
/// Mirrors `bb_shift_row.m`.
pub(crate) fn shift_row(out: &mut [u64], row: &[u64], sx: i32, rmask: &[u64]) {
    let nw = row.len();
    debug_assert_eq!(out.len(), nw);
    debug_assert_eq!(rmask.len(), nw);

    if sx == 0 {
        for i in 0..nw {
            out[i] = row[i] & rmask[i];
        }
        return;
    }

    if sx > 0 {
        let s = sx as u32;
        for i in 0..nw {
            // Shift current word left by s
            out[i] = row[i].wrapping_shl(s);
            // Carry bits from word i-1: right-shift by (64-s) to bring the
            // top `s` bits of row[i-1] into the bottom of word i.
            if i > 0 {
                // sx - 64 is negative in MATLAB → right shift by (64 - sx)
                out[i] |= row[i - 1].wrapping_shr(64 - s);
            }
        }
    } else {
        let n = (-sx) as u32; // n = |sx|, 1..63
        for i in 0..nw {
            // Shift current word right by n
            out[i] = row[i].wrapping_shr(n);
            // Carry from word i+1: left-shift by (64-n) to bring the bottom
            // n bits of row[i+1] into the top of word i.
            if i + 1 < nw {
                out[i] |= row[i + 1].wrapping_shl(64 - n);
            }
        }
    }

    // Mask to valid bits
    for i in 0..nw {
        out[i] &= rmask[i];
    }
}

// ---------------------------------------------------------------------------
// dilate2d_into  (hot inner helper)
// ---------------------------------------------------------------------------

/// L-infinity box dilation written into an existing slice `dst`.
///
/// `dst`, `src`, and `layout.rmask` all have length `layout.map_y * layout.wpr`.
/// `scratch` holds caller-provided row-sized buffers (length `wpr`) reused
/// across calls to avoid per-call allocation.
///
/// Algorithm (mirrors `bb_dilate2d.m`):
/// 1. Y-pass: copy `src` into `dst`, then OR shifted rows from `src`.
/// 2. X-pass (if `dx > 0`): for each row, save the Y-dilated row into
///    `scratch.pos`, then for each `sx_abs` OR ±shifted versions into `dst`.
/// 3. Final mask applied inside the X-pass (and in the dx==0 fast-path).
///
/// Panics if `dx >= 64`.
pub(crate) fn dilate2d_into(
    dst: &mut [u64],
    src: &[u64],
    layout: &RowLayout,
    dx: u32,
    dy: u32,
    scratch: &mut Scratch,
) {
    let RowLayout { map_y, wpr, rmask } = *layout;
    assert!(dx < 64, "bb_dilate2d: dx >= 64 ({dx}) not supported");

    // --- Y-pass: dst ← y_dilated ---
    // Copy src into dst, then OR in shifted rows (reading always from src).
    dst.copy_from_slice(src);

    for sy in 1..=(dy as usize) {
        for r in 0..map_y {
            let r_start = r * wpr;
            if r + sy < map_y {
                let dst_start = (r + sy) * wpr;
                for w in 0..wpr {
                    dst[dst_start + w] |= src[r_start + w];
                }
            }
            if r >= sy {
                let dst_start = (r - sy) * wpr;
                for w in 0..wpr {
                    dst[dst_start + w] |= src[r_start + w];
                }
            }
        }
    }

    // --- Early return if dx == 0 ---
    if dx == 0 {
        // Just mask the Y-dilated result
        for iy in 0..map_y {
            for w in 0..wpr {
                dst[iy * wpr + w] &= rmask[w];
            }
        }
        return;
    }

    // --- X-pass ---
    // For each row: save its Y-dilated value into scratch.pos, then for each
    // sx_abs OR ±shifted versions into dst.
    // scratch.neg is reused for each individual shifted row.
    scratch.pos.resize(wpr, 0u64);
    scratch.neg.resize(wpr, 0u64);

    for iy in 0..map_y {
        let row_start = iy * wpr;
        // Copy the Y-dilated row into scratch.pos (so reads are stable while
        // we accumulate the OR into dst[row_start..]).
        scratch.pos.copy_from_slice(&dst[row_start..row_start + wpr]);

        for sx_abs in 1..=(dx as i32) {
            shift_row(&mut scratch.neg, &scratch.pos, sx_abs, rmask);
            for w in 0..wpr {
                dst[row_start + w] |= scratch.neg[w];
            }
            shift_row(&mut scratch.neg, &scratch.pos, -sx_abs, rmask);
            for w in 0..wpr {
                dst[row_start + w] |= scratch.neg[w];
            }
        }

        // Final mask (shift_row already masks, but the original Y-dilated
        // row stored in scratch_pos may have unmasked bits from the Y-pass).
        for w in 0..wpr {
            dst[row_start + w] &= rmask[w];
        }
    }
}

// ---------------------------------------------------------------------------
// dilate2d  (public entry point)
// ---------------------------------------------------------------------------

/// L-infinity box dilation of a 2-D bitboard by `(dx, dy)`.
///
/// Panics if `dx >= 64`.
pub(crate) fn dilate2d(bb: &Bitboard2D, dx: u32, dy: u32) -> Bitboard2D {
    let map_x = bb.map_x();
    let map_y = bb.map_y() as usize;
    let wpr = bb.words_per_row() as usize;
    let rmask = row_mask(map_x);
    let src = bb.data();

    // Allocate output + two scratch row-buffers (the only heap allocs here).
    let mut out_data = vec![0u64; map_y * wpr];
    let mut scratch = Scratch::default();

    let layout = RowLayout { map_y, wpr, rmask: &rmask };
    dilate2d_into(&mut out_data, src, &layout, dx, dy, &mut scratch);

    Bitboard2D {
        data: out_data,
        map_x,
        map_y: bb.map_y(),
        words_per_row: bb.words_per_row(),
    }
}

// ---------------------------------------------------------------------------
// dilate3d
// ---------------------------------------------------------------------------

/// 3-D box dilation: XY by `(dx, dy)`, periodic-theta by `dt`.
///
/// Mirrors `bb_dilate3d.m`:
/// 1. Apply XY dilation to every theta layer → `temp`.
/// 2. For each `st in 1..=dt`, OR `temp[it]` with
///    `temp[(it + st) % n_theta]` and `temp[(it - st + n_theta) % n_theta]`.
///    Source is always `temp` (post-XY, pre-theta-dilation buffer).
///
/// # WHY two buffers in the theta-pass
/// Each output theta layer reads up to 2*dt source layers via the wrap.
/// Modifying output layer `it` in-place would corrupt the contributions still
/// owed to subsequent layers (it+1 .. it+dt). Using `temp` as the read-only
/// source and `out` as the write target decouples reads from writes.
///
/// Panics if `dx >= 64`.
pub(crate) fn dilate3d(bb: &Bitboard3D, dx: u32, dy: u32, dt: u32) -> Bitboard3D {
    let map_x = bb.map_x();
    let map_y = bb.map_y() as usize;
    let n_theta = bb.n_theta() as usize;
    let wpr = bb.words_per_row() as usize;
    let layer_stride = bb.layer_stride();
    let rmask = row_mask(map_x);

    // Allocate: output buffer + temp (post-XY) + shared scratch row-buffers.
    // All shared across all theta layers — no per-layer allocation.
    let mut out = vec![0u64; bb.data().len()];
    let mut temp = vec![0u64; bb.data().len()];
    let mut scratch = Scratch::default();

    // Step 1: XY dilate each layer into `temp`, reusing scratch buffers.
    let layout = RowLayout { map_y, wpr, rmask: &rmask };
    for it in 0..n_theta {
        let layer_start = it * layer_stride;
        let src_layer = &bb.data()[layer_start..layer_start + layer_stride];
        dilate2d_into(
            &mut temp[layer_start..layer_start + layer_stride],
            src_layer,
            &layout,
            dx,
            dy,
            &mut scratch,
        );
    }

    if dt == 0 {
        return Bitboard3D {
            data: temp,
            map_x,
            map_y: bb.map_y(),
            n_theta: bb.n_theta(),
            words_per_row: bb.words_per_row(),
        };
    }

    // Step 2: theta dilation.
    // `out` starts as a copy of `temp` (each layer already contains its own
    // XY-dilated bits); the theta-pass only ORs in neighbours.
    out.copy_from_slice(&temp);

    for st in 1..=(dt as usize) {
        for it in 0..n_theta {
            let idx_plus = (it + st) % n_theta;
            let idx_minus = (it + n_theta - st) % n_theta;

            let dst_start = it * layer_stride;
            let src_plus_start = idx_plus * layer_stride;
            let src_minus_start = idx_minus * layer_stride;

            for w in 0..layer_stride {
                out[dst_start + w] |=
                    temp[src_plus_start + w] | temp[src_minus_start + w];
            }
        }
    }

    Bitboard3D {
        data: out,
        map_x,
        map_y: bb.map_y(),
        n_theta: bb.n_theta(),
        words_per_row: bb.words_per_row(),
    }
}


// ---------------------------------------------------------------------------
// Tests for ops
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bitboard::{Bitboard2D, Bitboard3D};

    fn make_bb2(map_x: u32, map_y: u32, cells: &[(u32, u32)]) -> Bitboard2D {
        let mut bb = Bitboard2D::new(map_x, map_y);
        for &(x, y) in cells {
            bb.set(x, y);
        }
        bb
    }

    fn make_bb3(map_x: u32, map_y: u32, n_theta: u32, cells: &[(u32, u32, u32)]) -> Bitboard3D {
        let mut bb = Bitboard3D::new(map_x, map_y, n_theta);
        for &(x, y, t) in cells {
            bb.set(x, y, t);
        }
        bb
    }

    // -----------------------------------------------------------------------
    // e. dilate2d identity at (dx=0, dy=0)
    // -----------------------------------------------------------------------
    #[test]
    fn unit_dilate2d_identity_at_zero() {
        let bb = make_bb2(10, 10, &[(2, 2), (7, 5), (0, 9)]);
        let d = bb.dilate(0, 0);
        assert_eq!(d.popcount(), bb.popcount());
        assert_eq!(d, bb);
    }

    // -----------------------------------------------------------------------
    // f. dilate2d single cell 5x5, bit at (2,2), dilate(1,1) → 3x3 box
    // -----------------------------------------------------------------------
    #[test]
    fn unit_dilate2d_single_cell_expands_correctly() {
        let bb = make_bb2(5, 5, &[(2, 2)]);
        let d = bb.dilate(1, 1);
        // 3x3 box: x in [1,3], y in [1,3]
        assert_eq!(d.popcount(), 9);
        for y in 1..=3u32 {
            for x in 1..=3u32 {
                assert!(d.test(x, y), "Expected ({x},{y}) set");
            }
        }
        // corners outside box should be clear
        assert!(!d.test(0, 0));
        assert!(!d.test(4, 4));
    }

    // -----------------------------------------------------------------------
    // g. dilate2d clamps to map_x
    // -----------------------------------------------------------------------
    #[test]
    fn unit_dilate2d_clamps_to_map_x() {
        // map_x=5, bit at (3,0), dilate(2,0)
        // bits that would be set: 1,2,3,4,5 → masked to 1,2,3,4 (ix<5)
        let bb = make_bb2(5, 5, &[(3, 0)]);
        let d = bb.dilate(2, 0);
        assert_eq!(d.popcount(), 4, "Expected 4 bits (ix=1..4)");
        assert!(d.test(1, 0));
        assert!(d.test(2, 0));
        assert!(d.test(3, 0));
        assert!(d.test(4, 0));
        // ix=0 not set (3-2=1, not 0), ix≥5 masked
        assert!(!d.test(0, 0));
    }

    // -----------------------------------------------------------------------
    // h. dilate3d theta wraps
    // -----------------------------------------------------------------------
    #[test]
    fn unit_dilate3d_theta_wraps() {
        // 3x3 board, N_THETA=4, bit at (1,1,0)
        // dilate(0,0,1): layers 0, 1, and 3 (wrap). Layer 2 unchanged.
        let bb = make_bb3(3, 3, 4, &[(1, 1, 0)]);
        let d = bb.dilate(0, 0, 1);

        // Layer 0 has the original bit
        assert!(d.test(1, 1, 0));
        // Layer 1 = (0+1)%4 = 1 gets the bit
        assert!(d.test(1, 1, 1));
        // Layer 3 = (0-1+4)%4 = 3 gets the bit
        assert!(d.test(1, 1, 3));
        // Layer 2 should NOT have the bit
        assert!(!d.test(1, 1, 2));

        // Total: layers 0, 1, 3 each have 1 bit
        assert_eq!(d.popcount(), 3);
    }

    // -----------------------------------------------------------------------
    // k. and_inplace
    // -----------------------------------------------------------------------
    #[test]
    fn unit_and_inplace() {
        let a = make_bb2(10, 5, &[(0, 0), (3, 2), (7, 4)]);
        let b = make_bb2(10, 5, &[(3, 2), (7, 4), (9, 1)]);
        let mut res = a.clone();
        res.and_inplace(&b);
        assert_eq!(res.popcount(), 2);
        assert!(res.test(3, 2));
        assert!(res.test(7, 4));
        assert!(!res.test(0, 0));
        assert!(!res.test(9, 1));
    }

    // -----------------------------------------------------------------------
    // l. or_inplace
    // -----------------------------------------------------------------------
    #[test]
    fn unit_or_inplace() {
        let a = make_bb2(10, 5, &[(0, 0), (3, 2)]);
        let b = make_bb2(10, 5, &[(3, 2), (7, 4)]);
        let mut res = a.clone();
        res.or_inplace(&b);
        assert_eq!(res.popcount(), 3);
        assert!(res.test(0, 0));
        assert!(res.test(3, 2));
        assert!(res.test(7, 4));
    }

    // -----------------------------------------------------------------------
    // shift_row: sanity check positive shift
    // -----------------------------------------------------------------------
    #[test]
    fn unit_shift_row_positive() {
        // Single word, row = [1] (bit 0 set), shift by 3 → bit 3 set
        let row = vec![1u64];
        let rmask = vec![u64::MAX];
        let mut out = vec![0u64; 1];
        shift_row(&mut out, &row, 3, &rmask);
        assert_eq!(out[0], 1u64 << 3);
    }

    #[test]
    fn unit_shift_row_negative() {
        // Single word, row = [8] (bit 3 set), shift by -3 → bit 0 set
        let row = vec![8u64];
        let rmask = vec![u64::MAX];
        let mut out = vec![0u64; 1];
        shift_row(&mut out, &row, -3, &rmask);
        assert_eq!(out[0], 1u64);
    }

    #[test]
    fn unit_shift_row_carry_across_words() {
        // Two words. Word 0 = u64::MAX, word 1 = 0. Shift +1.
        // After shift: word 0 = u64::MAX << 1, word 1 = MAX >> 63 = 1.
        let row = vec![u64::MAX, 0u64];
        let rmask = vec![u64::MAX, u64::MAX];
        let mut out = vec![0u64; 2];
        shift_row(&mut out, &row, 1, &rmask);
        assert_eq!(out[0], u64::MAX << 1);
        assert_eq!(out[1], 1u64);
    }

    #[test]
    fn unit_shift_row_carry_across_words_neg() {
        // Word 0 = 0, word 1 = 1 (bit 64 of the big integer). Shift -1.
        // Out[0] gets carry: word[1] << (64-1) = 1 << 63
        // Out[1] = 1 >> 1 = 0
        let row = vec![0u64, 1u64];
        let rmask = vec![u64::MAX, u64::MAX];
        let mut out = vec![0u64; 2];
        shift_row(&mut out, &row, -1, &rmask);
        assert_eq!(out[0], 1u64 << 63);
        assert_eq!(out[1], 0u64);
    }
}
