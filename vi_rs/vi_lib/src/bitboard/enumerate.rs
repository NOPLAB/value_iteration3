//! Lazy iterators over set bits in [`Bitboard2D`] and [`Bitboard3D`].
//!
//! Mirrors `bb_enumerate2d.m` and `bb_enumerate3d.m`:
//! - Outer loop: `it` (3D only) → `iy` → word index `wi`.
//! - Within each word: Brian Kernighan bit loop (`w &= w - 1`), using
//!   `u64::trailing_zeros()` (equivalent to `bb_ctz_word`).
//! - Bits with `ix >= map_x` are silently skipped (should not occur in a
//!   correctly maintained bitboard, but we guard anyway).

use std::iter::FusedIterator;

use super::{Bitboard2D, Bitboard3D};

// ---------------------------------------------------------------------------
// Bitboard2DIter
// ---------------------------------------------------------------------------

/// Lazy iterator over `(ix, iy)` for every set bit in a [`Bitboard2D`].
pub struct Bitboard2DIter<'a> {
    bb: &'a Bitboard2D,
    iy: u32,
    wi: u32,
    /// Remaining bits in the current word (bits are cleared as we emit them).
    word: u64,
}

impl<'a> Bitboard2DIter<'a> {
    pub(crate) fn new(bb: &'a Bitboard2D) -> Self {
        // Load the first word (if any).
        let word = if bb.map_y() > 0 && bb.words_per_row() > 0 {
            bb.data()[0]
        } else {
            0
        };
        Self { bb, iy: 0, wi: 0, word }
    }

    /// Advance `(iy, wi, word)` to the next word, skipping zero words.
    fn advance(&mut self) {
        let wpr = self.bb.words_per_row();
        let map_y = self.bb.map_y();
        loop {
            self.wi += 1;
            if self.wi >= wpr {
                self.wi = 0;
                self.iy += 1;
                if self.iy >= map_y {
                    self.word = 0;
                    return;
                }
            }
            let idx = (self.iy * wpr + self.wi) as usize;
            self.word = self.bb.data()[idx];
            if self.word != 0 {
                return;
            }
        }
    }
}

impl<'a> Iterator for Bitboard2DIter<'a> {
    type Item = (u32, u32);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            // Skip empty words
            while self.word == 0 {
                if self.iy >= self.bb.map_y() {
                    return None;
                }
                self.advance();
                if self.iy >= self.bb.map_y() {
                    return None;
                }
            }

            // Extract the lowest set bit
            let bit = self.word.trailing_zeros();
            let ix = self.wi * 64 + bit;
            // Clear that bit (Brian Kernighan)
            self.word &= self.word - 1;

            // Guard: skip out-of-range bits (shouldn't happen with correct row_mask).
            // Instead of recursing, continue the loop to retry within this word.
            if ix < self.bb.map_x() {
                return Some((ix, self.iy));
            }
            // ix out of range — strip bit and retry in the same word
        }
    }
}

impl FusedIterator for Bitboard2DIter<'_> {}

// ---------------------------------------------------------------------------
// Bitboard3DIter
// ---------------------------------------------------------------------------

/// Lazy iterator over `(ix, iy, it)` for every set bit in a [`Bitboard3D`].
pub struct Bitboard3DIter<'a> {
    bb: &'a Bitboard3D,
    it: u32,
    iy: u32,
    wi: u32,
    word: u64,
}

impl<'a> Bitboard3DIter<'a> {
    pub(crate) fn new(bb: &'a Bitboard3D) -> Self {
        let word = if bb.n_theta() > 0 && bb.map_y() > 0 && bb.words_per_row() > 0 {
            bb.data()[0]
        } else {
            0
        };
        Self { bb, it: 0, iy: 0, wi: 0, word }
    }

    fn advance(&mut self) {
        let wpr = self.bb.words_per_row();
        let map_y = self.bb.map_y();
        let n_theta = self.bb.n_theta();
        loop {
            self.wi += 1;
            if self.wi >= wpr {
                self.wi = 0;
                self.iy += 1;
                if self.iy >= map_y {
                    self.iy = 0;
                    self.it += 1;
                    if self.it >= n_theta {
                        self.word = 0;
                        return;
                    }
                }
            }
            let idx = (self.it * map_y * wpr + self.iy * wpr + self.wi) as usize;
            self.word = self.bb.data()[idx];
            if self.word != 0 {
                return;
            }
        }
    }
}

impl<'a> Iterator for Bitboard3DIter<'a> {
    type Item = (u32, u32, u32);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            while self.word == 0 {
                if self.it >= self.bb.n_theta() {
                    return None;
                }
                self.advance();
                if self.it >= self.bb.n_theta() {
                    return None;
                }
            }

            let bit = self.word.trailing_zeros();
            let ix = self.wi * 64 + bit;
            self.word &= self.word - 1;

            // Guard: skip out-of-range bits. Continue loop instead of recursing.
            if ix < self.bb.map_x() {
                return Some((ix, self.iy, self.it));
            }
            // ix out of range — strip bit and retry in the same word
        }
    }
}

impl FusedIterator for Bitboard3DIter<'_> {}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    // -----------------------------------------------------------------------
    // i. enumerate2d yields set cells
    // -----------------------------------------------------------------------
    #[test]
    fn unit_enumerate2d_yields_set_cells() {
        let mut bb = Bitboard2D::new(10, 5);
        let cells = [(0u32, 0u32), (3, 1), (7, 2)];
        for &(x, y) in &cells {
            bb.set(x, y);
        }
        let got: HashSet<(u32, u32)> = bb.enumerate().collect();
        let expected: HashSet<(u32, u32)> = cells.iter().copied().collect();
        assert_eq!(got, expected);
    }

    // -----------------------------------------------------------------------
    // j. enumerate3d yields set cells
    // -----------------------------------------------------------------------
    #[test]
    fn unit_enumerate3d_yields_set_cells() {
        let mut bb = Bitboard3D::new(10, 5, 4);
        let cells = [(0u32, 0u32, 0u32), (3, 1, 2), (7, 4, 3)];
        for &(x, y, t) in &cells {
            bb.set(x, y, t);
        }
        let got: HashSet<(u32, u32, u32)> = bb.enumerate().collect();
        let expected: HashSet<(u32, u32, u32)> = cells.iter().copied().collect();
        assert_eq!(got, expected);
    }

    // -----------------------------------------------------------------------
    // empty bitboard yields nothing
    // -----------------------------------------------------------------------
    #[test]
    fn unit_enumerate2d_empty() {
        let bb = Bitboard2D::new(10, 10);
        assert_eq!(bb.enumerate().count(), 0);
    }

    #[test]
    fn unit_enumerate3d_empty() {
        let bb = Bitboard3D::new(10, 10, 4);
        assert_eq!(bb.enumerate().count(), 0);
    }

    // -----------------------------------------------------------------------
    // cross-word enumeration (map_x=130, spans 3 words)
    // -----------------------------------------------------------------------
    #[test]
    fn unit_enumerate2d_cross_word() {
        let mut bb = Bitboard2D::new(130, 3);
        bb.set(0, 0);
        bb.set(63, 0);
        bb.set(64, 1);
        bb.set(128, 2);
        let got: HashSet<(u32, u32)> = bb.enumerate().collect();
        assert!(got.contains(&(0, 0)));
        assert!(got.contains(&(63, 0)));
        assert!(got.contains(&(64, 1)));
        assert!(got.contains(&(128, 2)));
        assert_eq!(got.len(), 4);
    }
}
