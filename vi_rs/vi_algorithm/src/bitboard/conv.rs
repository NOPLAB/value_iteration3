//! Conversions between bitboards and [`ndarray`] boolean arrays.
//!
//! Mirrors `bb_from_logical2d.m`, `bb_from_logical3d.m`,
//! `bb_to_logical2d.m`, `bb_to_logical3d.m`.
//!
//! Array shape conventions (matching MATLAB row-major layout):
//! - 2D: `[map_y, map_x]`
//! - 3D: `[map_y, map_x, n_theta]`

use ndarray::{Array2, Array3, ArrayView2, ArrayView3};

use super::{Bitboard2D, Bitboard3D};

// ---------------------------------------------------------------------------
// 2D
// ---------------------------------------------------------------------------

/// Convert an `ndarray::ArrayView2<bool>` (shape `[map_y, map_x]`) into a
/// [`Bitboard2D`].
pub fn from_logical2d(mask: ArrayView2<bool>) -> Bitboard2D {
    let map_y = mask.nrows() as u32;
    let map_x = mask.ncols() as u32;
    let mut bb = Bitboard2D::new(map_x, map_y);
    for iy in 0..map_y {
        for ix in 0..map_x {
            if mask[[iy as usize, ix as usize]] {
                bb.set(ix, iy);
            }
        }
    }
    bb
}

/// Convert a [`Bitboard2D`] to `Array2<bool>` (shape `[map_y, map_x]`).
pub fn to_logical2d(bb: &Bitboard2D) -> Array2<bool> {
    let map_y = bb.map_y() as usize;
    let map_x = bb.map_x() as usize;
    let wpr = bb.words_per_row() as usize;
    let mut out = Array2::from_elem((map_y, map_x), false);
    for iy in 0..map_y {
        for wi in 0..wpr {
            let mut w = bb.data()[iy * wpr + wi];
            let base = wi * 64;
            while w != 0 {
                let b = w.trailing_zeros() as usize;
                let ix = base + b;
                if ix < map_x {
                    out[[iy, ix]] = true;
                }
                w &= w - 1;
            }
        }
    }
    out
}

// ---------------------------------------------------------------------------
// 3D
// ---------------------------------------------------------------------------

/// Convert an `ndarray::ArrayView3<bool>` (shape `[map_y, map_x, n_theta]`)
/// into a [`Bitboard3D`].
pub fn from_logical3d(mask: ArrayView3<bool>) -> Bitboard3D {
    let map_y = mask.dim().0 as u32;
    let map_x = mask.dim().1 as u32;
    let n_theta = mask.dim().2 as u32;
    let mut bb = Bitboard3D::new(map_x, map_y, n_theta);
    for it in 0..n_theta {
        for iy in 0..map_y {
            for ix in 0..map_x {
                if mask[[iy as usize, ix as usize, it as usize]] {
                    bb.set(ix, iy, it);
                }
            }
        }
    }
    bb
}

/// Convert a [`Bitboard3D`] to `Array3<bool>` (shape `[map_y, map_x, n_theta]`).
pub fn to_logical3d(bb: &Bitboard3D) -> Array3<bool> {
    let map_y = bb.map_y() as usize;
    let map_x = bb.map_x() as usize;
    let n_theta = bb.n_theta() as usize;
    let wpr = bb.words_per_row() as usize;
    let mut out = Array3::from_elem((map_y, map_x, n_theta), false);
    for it in 0..n_theta {
        for iy in 0..map_y {
            for wi in 0..wpr {
                let idx = it * map_y * wpr + iy * wpr + wi;
                let mut w = bb.data()[idx];
                let base = wi * 64;
                while w != 0 {
                    let b = w.trailing_zeros() as usize;
                    let ix = base + b;
                    if ix < map_x {
                        out[[iy, ix, it]] = true;
                    }
                    w &= w - 1;
                }
            }
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use ndarray::array;

    // -----------------------------------------------------------------------
    // n. from_logical / to_logical roundtrip 2D
    // -----------------------------------------------------------------------
    #[test]
    fn unit_from_logical_to_logical_roundtrip_2d() {
        // 3x4 array (map_y=3, map_x=4)
        let mask = array![
            [true,  false, true,  false],
            [false, true,  false, true ],
            [true,  true,  false, false],
        ];
        let bb = Bitboard2D::from_logical(mask.view());
        let back = bb.to_logical();
        assert_eq!(back, mask);
    }

    // -----------------------------------------------------------------------
    // o. from_logical / to_logical roundtrip 3D
    // -----------------------------------------------------------------------
    #[test]
    fn unit_from_logical_to_logical_roundtrip_3d() {
        // shape [map_y=2, map_x=3, n_theta=2]
        let mut mask = Array3::from_elem((2, 3, 2), false);
        mask[[0, 0, 0]] = true;
        mask[[0, 2, 1]] = true;
        mask[[1, 1, 0]] = true;
        let bb = Bitboard3D::from_logical(mask.view());
        let back = bb.to_logical();
        assert_eq!(back, mask);
    }

    // -----------------------------------------------------------------------
    // all-false mask
    // -----------------------------------------------------------------------
    #[test]
    fn unit_from_logical_all_false_2d() {
        let mask = Array2::from_elem((5, 8), false);
        let bb = Bitboard2D::from_logical(mask.view());
        assert_eq!(bb.popcount(), 0);
    }

    // -----------------------------------------------------------------------
    // all-true mask
    // -----------------------------------------------------------------------
    #[test]
    fn unit_from_logical_all_true_2d() {
        let mask = Array2::from_elem((3, 7), true);
        let bb = Bitboard2D::from_logical(mask.view());
        assert_eq!(bb.popcount(), 21);
        let back = bb.to_logical();
        assert_eq!(back, mask);
    }
}
