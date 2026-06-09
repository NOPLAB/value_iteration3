//! Bitboard data structures for 2-D and 3-D binary grids.
//!
//! Bits are packed 64-per-word, LSB = lowest x index.  Row layout mirrors the
//! MATLAB reference (`vi_matlab/src/shared/bitboard/`).
//!
//! # Index conventions (0-indexed, matching the Rust API)
//! - `ix ∈ [0, map_x)`, `iy ∈ [0, map_y)`, `it ∈ [0, n_theta)`.
//! - Word index  `wi = ix / 64`.
//! - Bit position `bi = ix % 64`.
//! - 2D word flat index: `iy * words_per_row + wi`.
//! - 3D word flat index: `it * map_y * words_per_row + iy * words_per_row + wi`.

pub(crate) mod conv;
pub(crate) mod enumerate;
pub(crate) mod ops;

pub use enumerate::{Bitboard2DIter, Bitboard3DIter};

use ndarray::{Array2, Array3, ArrayView2, ArrayView3};

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Number of 64-bit words needed to hold `map_x` bits.
#[inline]
pub(crate) fn words_per_row(map_x: u32) -> u32 {
    map_x.div_ceil(64)
}

/// Per-word validity mask for a row of `map_x` bits.
///
/// Words 0..words_per_row-2: all-ones (`u64::MAX`).
/// Last word: `(1 << (map_x % 64)) - 1`, or all-ones if `map_x` is a
/// multiple of 64.
pub(crate) fn row_mask(map_x: u32) -> Vec<u64> {
    let nw = words_per_row(map_x) as usize;
    let mut mask = vec![u64::MAX; nw];
    let rem = map_x % 64;
    if rem != 0 {
        mask[nw - 1] = (1u64 << rem) - 1;
    }
    mask
}

// ---------------------------------------------------------------------------
// Bitboard2D
// ---------------------------------------------------------------------------

/// Flat-packed 2-D bitboard (row-major, 64 bits per word).
///
/// Storage: `data[iy * words_per_row + wi]`.
#[derive(Clone, Debug, PartialEq)]
pub struct Bitboard2D {
    data: Vec<u64>,
    map_x: u32,
    map_y: u32,
    words_per_row: u32,
}

impl Bitboard2D {
    /// Allocate an all-zero bitboard of size `map_x × map_y`.
    pub fn new(map_x: u32, map_y: u32) -> Self {
        let wpr = words_per_row(map_x);
        Self {
            data: vec![0u64; (map_y * wpr) as usize],
            map_x,
            map_y,
            words_per_row: wpr,
        }
    }

    pub fn map_x(&self) -> u32 { self.map_x }
    pub fn map_y(&self) -> u32 { self.map_y }
    pub fn words_per_row(&self) -> u32 { self.words_per_row }

    /// Set the bit at `(ix, iy)` (0-indexed).
    pub fn set(&mut self, ix: u32, iy: u32) {
        debug_assert!(ix < self.map_x && iy < self.map_y);
        let wi = ix / 64;
        let bi = ix % 64;
        let idx = (iy * self.words_per_row + wi) as usize;
        self.data[idx] |= 1u64 << bi;
    }

    /// Return `true` if the bit at `(ix, iy)` is set.
    pub fn test(&self, ix: u32, iy: u32) -> bool {
        debug_assert!(ix < self.map_x && iy < self.map_y);
        let wi = ix / 64;
        let bi = ix % 64;
        let idx = (iy * self.words_per_row + wi) as usize;
        self.data[idx] & (1u64 << bi) != 0
    }

    /// Total number of set bits.
    pub fn popcount(&self) -> u64 {
        ops::popcount_slice(&self.data)
    }

    /// L-infinity box dilation by `(dx, dy)`.  Panics if `dx >= 64`.
    pub fn dilate(&self, dx: u32, dy: u32) -> Self {
        ops::dilate2d(self, dx, dy)
    }

    /// Bitwise AND (in-place): `self &= other`.
    pub fn and_inplace(&mut self, other: &Self) {
        debug_assert_eq!(self.map_x, other.map_x);
        debug_assert_eq!(self.map_y, other.map_y);
        ops::and_slice(&mut self.data, &other.data);
    }

    /// Bitwise OR (in-place): `self |= other`.
    pub fn or_inplace(&mut self, other: &Self) {
        debug_assert_eq!(self.map_x, other.map_x);
        debug_assert_eq!(self.map_y, other.map_y);
        ops::or_slice(&mut self.data, &other.data);
    }

    /// Bitwise complement, masked to valid bits (out-of-range bits stay zero).
    pub fn complement(&self) -> Self {
        ops::complement2d(self)
    }

    /// Lazy iterator over `(ix, iy)` of every set bit.
    pub fn enumerate(&self) -> Bitboard2DIter<'_> {
        Bitboard2DIter::new(self)
    }

    /// Convert from an `Array2<bool>` with shape `[map_y, map_x]`.
    pub fn from_logical(mask: ArrayView2<bool>) -> Self {
        conv::from_logical2d(mask)
    }

    /// Convert to `Array2<bool>` with shape `[map_y, map_x]`.
    pub fn to_logical(&self) -> Array2<bool> {
        conv::to_logical2d(self)
    }

    // --- internal accessors used by sibling modules ---

    pub(crate) fn data(&self) -> &[u64] { &self.data }
}

// ---------------------------------------------------------------------------
// Bitboard3D
// ---------------------------------------------------------------------------

/// Flat-packed 3-D bitboard (theta-major → row-major → word).
///
/// Storage: `data[it * map_y * words_per_row + iy * words_per_row + wi]`.
#[derive(Clone, Debug, PartialEq)]
pub struct Bitboard3D {
    data: Vec<u64>,
    map_x: u32,
    map_y: u32,
    n_theta: u32,
    words_per_row: u32,
}

impl Bitboard3D {
    /// Allocate an all-zero bitboard of size `map_x × map_y × n_theta`.
    pub fn new(map_x: u32, map_y: u32, n_theta: u32) -> Self {
        let wpr = words_per_row(map_x);
        Self {
            data: vec![0u64; (n_theta * map_y * wpr) as usize],
            map_x,
            map_y,
            n_theta,
            words_per_row: wpr,
        }
    }

    pub fn map_x(&self) -> u32 { self.map_x }
    pub fn map_y(&self) -> u32 { self.map_y }
    pub fn n_theta(&self) -> u32 { self.n_theta }
    pub fn words_per_row(&self) -> u32 { self.words_per_row }

    /// Set the bit at `(ix, iy, it)` (0-indexed).
    pub fn set(&mut self, ix: u32, iy: u32, it: u32) {
        debug_assert!(ix < self.map_x && iy < self.map_y && it < self.n_theta);
        let wi = ix / 64;
        let bi = ix % 64;
        let idx = (it * self.map_y * self.words_per_row
            + iy * self.words_per_row
            + wi) as usize;
        self.data[idx] |= 1u64 << bi;
    }

    /// Return `true` if the bit at `(ix, iy, it)` is set.
    pub fn test(&self, ix: u32, iy: u32, it: u32) -> bool {
        debug_assert!(ix < self.map_x && iy < self.map_y && it < self.n_theta);
        let wi = ix / 64;
        let bi = ix % 64;
        let idx = (it * self.map_y * self.words_per_row
            + iy * self.words_per_row
            + wi) as usize;
        self.data[idx] & (1u64 << bi) != 0
    }

    /// Total number of set bits.
    pub fn popcount(&self) -> u64 {
        ops::popcount_slice(&self.data)
    }

    /// 3-D box dilation: XY by `(dx, dy)`, theta (periodic) by `dt`.
    /// Panics if `dx >= 64`.
    pub fn dilate(&self, dx: u32, dy: u32, dt: u32) -> Self {
        ops::dilate3d(self, dx, dy, dt)
    }

    /// Bitwise AND (in-place): `self &= other`.
    pub fn and_inplace(&mut self, other: &Self) {
        debug_assert_eq!(self.map_x, other.map_x);
        debug_assert_eq!(self.map_y, other.map_y);
        debug_assert_eq!(self.n_theta, other.n_theta);
        ops::and_slice(&mut self.data, &other.data);
    }

    /// Bitwise OR (in-place): `self |= other`.
    pub fn or_inplace(&mut self, other: &Self) {
        debug_assert_eq!(self.map_x, other.map_x);
        debug_assert_eq!(self.map_y, other.map_y);
        debug_assert_eq!(self.n_theta, other.n_theta);
        ops::or_slice(&mut self.data, &other.data);
    }

    /// Bitwise complement, masked to valid bits.
    pub fn complement(&self) -> Self {
        ops::complement3d(self)
    }

    /// Lazy iterator over `(ix, iy, it)` of every set bit.
    pub fn enumerate(&self) -> Bitboard3DIter<'_> {
        Bitboard3DIter::new(self)
    }

    /// Convert from an `Array3<bool>` with shape `[map_y, map_x, n_theta]`.
    pub fn from_logical(mask: ArrayView3<bool>) -> Self {
        conv::from_logical3d(mask)
    }

    /// Convert to `Array3<bool>` with shape `[map_y, map_x, n_theta]`.
    pub fn to_logical(&self) -> Array3<bool> {
        conv::to_logical3d(self)
    }

    // --- internal accessors ---

    pub(crate) fn data(&self) -> &[u64] { &self.data }
    /// Stride in words between consecutive theta layers.
    pub(crate) fn layer_stride(&self) -> usize {
        (self.map_y * self.words_per_row) as usize
    }
}

// ---------------------------------------------------------------------------
// Unit tests (structs + set/test + popcount)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // a. set/test roundtrip 2D
    // -----------------------------------------------------------------------
    #[test]
    fn unit_set_test_roundtrip_2d() {
        let mut bb = Bitboard2D::new(10, 10);
        bb.set(3, 5);
        assert!(bb.test(3, 5));
        assert!(!bb.test(3, 6));
        assert!(!bb.test(4, 5));
        assert!(!bb.test(0, 0));
    }

    // -----------------------------------------------------------------------
    // b. set/test roundtrip 3D
    // -----------------------------------------------------------------------
    #[test]
    fn unit_set_test_roundtrip_3d() {
        let mut bb = Bitboard3D::new(10, 10, 4);
        bb.set(3, 5, 2);
        assert!(bb.test(3, 5, 2));
        assert!(!bb.test(3, 5, 1));
        assert!(!bb.test(3, 5, 3));
        assert!(!bb.test(4, 5, 2));
        assert!(!bb.test(3, 6, 2));
    }

    // -----------------------------------------------------------------------
    // c. popcount zero
    // -----------------------------------------------------------------------
    #[test]
    fn unit_popcount_zero() {
        let bb2 = Bitboard2D::new(10, 10);
        assert_eq!(bb2.popcount(), 0);
        let bb3 = Bitboard3D::new(10, 10, 4);
        assert_eq!(bb3.popcount(), 0);
    }

    // -----------------------------------------------------------------------
    // d. popcount after set (3 distinct cells)
    // -----------------------------------------------------------------------
    #[test]
    fn unit_popcount_after_set() {
        let mut bb2 = Bitboard2D::new(20, 10);
        bb2.set(0, 0);
        bb2.set(7, 3);
        bb2.set(15, 9);
        assert_eq!(bb2.popcount(), 3);

        let mut bb3 = Bitboard3D::new(20, 10, 8);
        bb3.set(0, 0, 0);
        bb3.set(7, 3, 2);
        bb3.set(15, 9, 7);
        assert_eq!(bb3.popcount(), 3);
    }

    // -----------------------------------------------------------------------
    // boundary: bit at ix=63 (last bit of word 0 on a 64+-wide board)
    // -----------------------------------------------------------------------
    #[test]
    fn unit_set_test_at_word_boundary() {
        let mut bb = Bitboard2D::new(128, 4);
        bb.set(63, 0);
        bb.set(64, 0);
        assert!(bb.test(63, 0));
        assert!(bb.test(64, 0));
        assert!(!bb.test(62, 0));
        assert!(!bb.test(65, 0));
        assert_eq!(bb.popcount(), 2);
    }

    // -----------------------------------------------------------------------
    // row_mask helper
    // -----------------------------------------------------------------------
    #[test]
    fn unit_row_mask_partial_word() {
        // map_x=5 → 1 word, mask = 0x1F
        let m = row_mask(5);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0], 0x1F);
    }

    #[test]
    fn unit_row_mask_full_word() {
        // map_x=64 → 1 word, mask = u64::MAX
        let m = row_mask(64);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0], u64::MAX);
    }

    #[test]
    fn unit_row_mask_two_words_partial() {
        // map_x=70 → 2 words, first MAX, second has 6 bits
        let m = row_mask(70);
        assert_eq!(m.len(), 2);
        assert_eq!(m[0], u64::MAX);
        assert_eq!(m[1], (1u64 << 6) - 1);
    }
}

// ---------------------------------------------------------------------------
// Proptest invariants
// ---------------------------------------------------------------------------

#[cfg(test)]
mod prop_tests {
    use super::*;
    use ndarray::Array3;
    use proptest::prelude::*;

    // -----------------------------------------------------------------------
    // p. prop_set_test_roundtrip
    // -----------------------------------------------------------------------
    proptest! {
        #[test]
        fn prop_set_test_roundtrip(
            map_x in 1u32..=20,
            map_y in 1u32..=20,
            ix in 0u32..20,
            iy in 0u32..20,
        ) {
            let ix = ix % map_x;
            let iy = iy % map_y;
            let mut bb = Bitboard2D::new(map_x, map_y);
            bb.set(ix, iy);
            prop_assert!(bb.test(ix, iy));
        }
    }

    // -----------------------------------------------------------------------
    // q. prop_popcount_monotone_under_or
    // -----------------------------------------------------------------------
    proptest! {
        #[test]
        fn prop_popcount_monotone_under_or(
            map_x in 1u32..=20,
            map_y in 1u32..=20,
            bits_a in proptest::collection::vec(proptest::bool::ANY, 1..=400usize),
            bits_b in proptest::collection::vec(proptest::bool::ANY, 1..=400usize),
        ) {
            let mut a = Bitboard2D::new(map_x, map_y);
            let mut b = Bitboard2D::new(map_x, map_y);
            let total = (map_x * map_y) as usize;
            for (i, v) in bits_a.iter().enumerate().take(total) {
                if *v { a.set((i as u32) % map_x, (i as u32) / map_x % map_y); }
            }
            for (i, v) in bits_b.iter().enumerate().take(total) {
                if *v { b.set((i as u32) % map_x, (i as u32) / map_x % map_y); }
            }
            let mut combined = a.clone();
            combined.or_inplace(&b);
            prop_assert!(combined.popcount() >= a.popcount());
            prop_assert!(combined.popcount() >= b.popcount());
        }
    }

    // -----------------------------------------------------------------------
    // r. prop_popcount_zero_under_and_disjoint
    // -----------------------------------------------------------------------
    proptest! {
        #[test]
        fn prop_popcount_zero_under_and_disjoint(
            map_x in 1u32..=20,
            map_y in 1u32..=20,
            // Use alternating bits to ensure disjoint sets
            pattern in 0u64..u64::MAX,
        ) {
            let total = (map_x * map_y) as usize;
            let mut a = Bitboard2D::new(map_x, map_y);
            let mut b = Bitboard2D::new(map_x, map_y);
            for i in 0..total {
                let ix = (i as u32) % map_x;
                let iy = (i as u32) / map_x % map_y;
                if (pattern >> (i % 64)) & 1 == 1 {
                    a.set(ix, iy);
                } else {
                    b.set(ix, iy);
                }
            }
            // a and b are disjoint by construction
            let mut intersection = a.clone();
            intersection.and_inplace(&b);
            prop_assert_eq!(intersection.popcount(), 0);
        }
    }

    // -----------------------------------------------------------------------
    // s. prop_dilate_monotone
    // -----------------------------------------------------------------------
    proptest! {
        #[test]
        fn prop_dilate_monotone(
            map_x in 1u32..=20,
            map_y in 1u32..=20,
            d in 0u32..4,
            bits in proptest::collection::vec(proptest::bool::ANY, 1..=400usize),
        ) {
            let total = (map_x * map_y) as usize;
            let mut bb = Bitboard2D::new(map_x, map_y);
            for (i, v) in bits.iter().enumerate().take(total) {
                if *v { bb.set((i as u32) % map_x, (i as u32) / map_x % map_y); }
            }
            let dilated = bb.dilate(d, d);
            prop_assert!(dilated.popcount() >= bb.popcount());
        }
    }

    // -----------------------------------------------------------------------
    // t. prop_enumerate_count_equals_popcount
    // -----------------------------------------------------------------------
    proptest! {
        #[test]
        fn prop_enumerate_count_equals_popcount(
            map_x in 1u32..=20,
            map_y in 1u32..=20,
            bits in proptest::collection::vec(proptest::bool::ANY, 1..=400usize),
        ) {
            let total = (map_x * map_y) as usize;
            let mut bb = Bitboard2D::new(map_x, map_y);
            for (i, v) in bits.iter().enumerate().take(total) {
                if *v { bb.set((i as u32) % map_x, (i as u32) / map_x % map_y); }
            }
            let count = bb.enumerate().count() as u64;
            prop_assert_eq!(count, bb.popcount());
        }
    }

    // -----------------------------------------------------------------------
    // u. prop_logical_roundtrip_2d
    // -----------------------------------------------------------------------
    proptest! {
        #[test]
        fn prop_logical_roundtrip_2d(
            map_x in 1u32..=20,
            map_y in 1u32..=20,
            bits in proptest::collection::vec(proptest::bool::ANY, 1..=400usize),
        ) {
            let total = (map_x * map_y) as usize;
            let mut bb = Bitboard2D::new(map_x, map_y);
            for (i, v) in bits.iter().enumerate().take(total) {
                if *v { bb.set((i as u32) % map_x, (i as u32) / map_x % map_y); }
            }
            let logical = bb.to_logical();
            let bb2 = Bitboard2D::from_logical(logical.view());
            prop_assert_eq!(bb, bb2);
        }
    }

    // -----------------------------------------------------------------------
    // v. prop_logical_roundtrip_3d  (Minor #8)
    // -----------------------------------------------------------------------
    proptest! {
        #[test]
        fn prop_logical_roundtrip_3d(
            map_x in 1u32..=10,
            map_y in 1u32..=10,
            n_theta in 1u32..=10,
        ) {
            // Deterministic pattern: set cell if (ix + iy + it) % 3 == 0
            let mask = Array3::from_shape_fn(
                (map_y as usize, map_x as usize, n_theta as usize),
                |(iy, ix, it)| (ix + iy + it) % 3 == 0,
            );
            let bb = Bitboard3D::from_logical(mask.view());
            let back = bb.to_logical();
            prop_assert_eq!(mask, back);
        }
    }

    // -----------------------------------------------------------------------
    // w. prop_dilate2d_crosses_word_boundary  (Minor #9)
    // -----------------------------------------------------------------------
    proptest! {
        #[test]
        fn prop_dilate2d_crosses_word_boundary(
            map_x in 60u32..=70,
            map_y in 1u32..=5,
            dx in 0u32..=4,
            dy in 0u32..=2,
        ) {
            let mut bb = Bitboard2D::new(map_x, map_y);
            // Set bits straddling the word boundary (ix=62, 63, 64) where applicable
            for iy in 0..map_y {
                for &ix in &[62u32, 63, 64] {
                    if ix < map_x {
                        bb.set(ix, iy);
                    }
                }
            }
            let d = bb.dilate(dx, dy);
            // Invariant: popcount monotone (dilation can only add bits)
            prop_assert!(d.popcount() >= bb.popcount());
            // Invariant: every set bit in `bb` is still set in `d`
            for (ix, iy) in bb.enumerate() {
                prop_assert!(d.test(ix, iy), "bit ({ix},{iy}) was set but missing after dilate");
            }
        }
    }
}
