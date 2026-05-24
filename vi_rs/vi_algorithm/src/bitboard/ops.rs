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

// ---------------------------------------------------------------------------
// shift_row
// ---------------------------------------------------------------------------

/// Horizontal big-integer shift of one row by `sx` bit positions.
///
/// - Positive `sx`: shifts bits toward higher x indices (left in memory-word
///   sense, i.e. higher bit positions).
/// - Negative `sx`: shifts toward lower x indices.
/// - Constraint: `-64 < sx < 64`.
///
/// Mirrors `bb_shift_row.m`.
pub(crate) fn shift_row(row: &[u64], sx: i32, rmask: &[u64]) -> Vec<u64> {
    let nw = row.len();
    let mut out = vec![0u64; nw];

    if sx == 0 {
        for i in 0..nw {
            out[i] = row[i] & rmask[i];
        }
        return out;
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
    out
}

// ---------------------------------------------------------------------------
// dilate2d
// ---------------------------------------------------------------------------

/// L-infinity box dilation of a 2-D bitboard by `(dx, dy)`.
///
/// Mirrors `bb_dilate2d.m`:
/// 1. Y-pass: for each `sy in 1..=dy`, OR rows with their ±sy shifted
///    versions (reading from the **original** `bb`, not the accumulator).
/// 2. X-pass: for each `sx in 1..=dx`, OR each row of the output with its
///    ±sx horizontally-shifted version (shifting the Y-dilated row).
/// 3. Final mask with row_mask.
///
/// Panics if `dx >= 64`.
pub(crate) fn dilate2d(bb: &Bitboard2D, dx: u32, dy: u32) -> Bitboard2D {
    assert!(dx < 64, "bb_dilate2d: dx >= 64 ({dx}) not supported");

    let map_x = bb.map_x();
    let map_y = bb.map_y() as usize;
    let wpr = bb.words_per_row() as usize;
    let rmask = row_mask(map_x);
    let src = bb.data();

    // --- Y-pass ---
    // y_dilated starts as a copy of bb; accumulate by reading from src.
    let mut y_dilated = src.to_vec();

    for sy in 1..=(dy as usize) {
        // For row r: y_dilated[r + sy] |= src[r]  (if r + sy < map_y)
        //            y_dilated[r - sy] |= src[r]  (if r >= sy)
        for r in 0..map_y {
            let r_start = r * wpr;
            if r + sy < map_y {
                let dst_start = (r + sy) * wpr;
                for w in 0..wpr {
                    y_dilated[dst_start + w] |= src[r_start + w];
                }
            }
            if r >= sy {
                let dst_start = (r - sy) * wpr;
                for w in 0..wpr {
                    y_dilated[dst_start + w] |= src[r_start + w];
                }
            }
        }
    }

    // out starts as y_dilated (dx == 0 → returns y_dilated directly)
    let mut out_data = y_dilated.clone();

    // --- X-pass ---
    if dx > 0 {
        for sx_abs in 1..=(dx as i32) {
            for iy in 0..map_y {
                let row_slice = &y_dilated[iy * wpr..(iy + 1) * wpr];
                let row_pos = shift_row(row_slice, sx_abs, &rmask);
                let row_neg = shift_row(row_slice, -sx_abs, &rmask);
                for w in 0..wpr {
                    out_data[iy * wpr + w] |= row_pos[w] | row_neg[w];
                }
            }
        }
    }

    // Final mask (handles overflow from the X-pass)
    for iy in 0..map_y {
        for w in 0..wpr {
            out_data[iy * wpr + w] &= rmask[w];
        }
    }

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
/// 1. Apply `dilate2d` to every theta layer → `temp`.
/// 2. For each `st in 1..=dt`, OR `temp[it]` with
///    `temp[(it + st) % n_theta]` and `temp[(it - st + n_theta) % n_theta]`.
///    Source is always `temp` (post-XY, pre-theta-dilation copy).
///
/// Panics if `dx >= 64`.
pub(crate) fn dilate3d(bb: &Bitboard3D, dx: u32, dy: u32, dt: u32) -> Bitboard3D {
    let map_x = bb.map_x();
    let map_y = bb.map_y();
    let n_theta = bb.n_theta() as usize;
    let layer_stride = bb.layer_stride();

    // Step 1: XY dilate each layer into `temp`
    let mut temp = vec![0u64; bb.data().len()];
    for it in 0..n_theta {
        // Build a temporary Bitboard2D for this layer
        let src_start = it * layer_stride;
        let src_layer = &bb.data()[src_start..src_start + layer_stride];
        let bb2 = Bitboard2D {
            data: src_layer.to_vec(),
            map_x,
            map_y,
            words_per_row: bb.words_per_row(),
        };
        let dilated = dilate2d(&bb2, dx, dy);
        temp[src_start..src_start + layer_stride].copy_from_slice(dilated.data());
    }

    if dt == 0 {
        return Bitboard3D {
            data: temp,
            map_x,
            map_y,
            n_theta: bb.n_theta(),
            words_per_row: bb.words_per_row(),
        };
    }

    // Step 2: theta dilation — source is `temp` (unchanged during this pass)
    let temp_snap = temp.clone();
    let mut out = temp; // accumulate into this

    for st in 1..=(dt as usize) {
        for it in 0..n_theta {
            let idx_plus = (it + st) % n_theta;
            let idx_minus = (it + n_theta - st) % n_theta;

            let dst_start = it * layer_stride;
            let src_plus_start = idx_plus * layer_stride;
            let src_minus_start = idx_minus * layer_stride;

            for w in 0..layer_stride {
                out[dst_start + w] |=
                    temp_snap[src_plus_start + w] | temp_snap[src_minus_start + w];
            }
        }
    }

    Bitboard3D {
        data: out,
        map_x,
        map_y,
        n_theta: bb.n_theta(),
        words_per_row: bb.words_per_row(),
    }
}

// ---------------------------------------------------------------------------
// complement
// ---------------------------------------------------------------------------

pub(crate) fn complement2d(bb: &Bitboard2D) -> Bitboard2D {
    let map_x = bb.map_x();
    let map_y = bb.map_y() as usize;
    let wpr = bb.words_per_row() as usize;
    let rmask = row_mask(map_x);
    let mut data = bb.data().to_vec();
    for iy in 0..map_y {
        for w in 0..wpr {
            data[iy * wpr + w] = (!data[iy * wpr + w]) & rmask[w];
        }
    }
    Bitboard2D {
        data,
        map_x,
        map_y: bb.map_y(),
        words_per_row: bb.words_per_row(),
    }
}

pub(crate) fn complement3d(bb: &Bitboard3D) -> Bitboard3D {
    let map_x = bb.map_x();
    let wpr = bb.words_per_row() as usize;
    let rmask = row_mask(map_x);
    let total_rows = (bb.n_theta() * bb.map_y()) as usize;
    let mut data = bb.data().to_vec();
    for row in 0..total_rows {
        for w in 0..wpr {
            data[row * wpr + w] = (!data[row * wpr + w]) & rmask[w];
        }
    }
    Bitboard3D {
        data,
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
    // m. complement masks oob bits
    // -----------------------------------------------------------------------
    #[test]
    fn unit_complement_masks_oob_bits() {
        let map_x = 5u32;
        let map_y = 3u32;
        let bb = Bitboard2D::new(map_x, map_y);
        let c = bb.complement();
        // Only map_x * map_y bits should be set, not 64 * map_y
        assert_eq!(c.popcount(), (map_x * map_y) as u64);
    }

    #[test]
    fn unit_complement_3d_masks_oob_bits() {
        let map_x = 5u32;
        let map_y = 3u32;
        let n_theta = 4u32;
        let bb = Bitboard3D::new(map_x, map_y, n_theta);
        let c = bb.complement();
        assert_eq!(c.popcount(), (map_x * map_y * n_theta) as u64);
    }

    // -----------------------------------------------------------------------
    // shift_row: sanity check positive shift
    // -----------------------------------------------------------------------
    #[test]
    fn unit_shift_row_positive() {
        // Single word, row = [1] (bit 0 set), shift by 3 → bit 3 set
        let row = vec![1u64];
        let rmask = vec![u64::MAX];
        let out = shift_row(&row, 3, &rmask);
        assert_eq!(out[0], 1u64 << 3);
    }

    #[test]
    fn unit_shift_row_negative() {
        // Single word, row = [8] (bit 3 set), shift by -3 → bit 0 set
        let row = vec![8u64];
        let rmask = vec![u64::MAX];
        let out = shift_row(&row, -3, &rmask);
        assert_eq!(out[0], 1u64);
    }

    #[test]
    fn unit_shift_row_carry_across_words() {
        // Two words. Word 0 = u64::MAX, word 1 = 0. Shift +1.
        // After shift: word 0 = u64::MAX << 1, word 1 = MAX >> 63 = 1.
        let row = vec![u64::MAX, 0u64];
        let rmask = vec![u64::MAX, u64::MAX];
        let out = shift_row(&row, 1, &rmask);
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
        let out = shift_row(&row, -1, &rmask);
        assert_eq!(out[0], 1u64 << 63);
        assert_eq!(out[1], 0u64);
    }
}
