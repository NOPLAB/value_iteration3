//! Pack/unpack the value-iteration transition table. Mirrors
//! `vi_matlab/workflows/validation/tests/gen_transitions.m::pack_model` and
//! `vi_matlab/src/common/unpack_transitions.m`.

use crate::params::{MAX_OUTCOMES, N_ACTIONS, N_THETA, TRANS_TABLE_SIZE, TRANS_WORD_STRIDE};
use crate::types::Offset;

pub struct PackedTransitions(pub Vec<u32>);

// NOTE: #[derive(Default)] is not used here because Rust does not implement Default for
// arrays longer than 32 elements on stable toolchains older than 1.87. The manual impl below
// is equivalent and kept for portability.
pub struct TransitionModel {
    pub n_outcomes: [[u8; N_THETA]; N_ACTIONS],
    pub dix: [[[Offset; MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
    pub diy: [[[Offset; MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
    pub dit: [[[Offset; MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
    pub prob: [[[u32; MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
}

impl Default for TransitionModel {
    fn default() -> Self {
        TransitionModel {
            n_outcomes: [[0u8; N_THETA]; N_ACTIONS],
            dix: [[[0i8; MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
            diy: [[[0i8; MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
            dit: [[[0i8; MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
            prob: [[[0u32; MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
        }
    }
}

#[inline]
fn pack_delta(dix: Offset, diy: Offset, dit: Offset) -> u32 {
    (dix as u8 as u32) | ((diy as u8 as u32) << 8) | ((dit as u8 as u32) << 16)
}

#[inline]
fn unpack_delta(word: u32) -> (Offset, Offset, Offset) {
    let dix = (word & 0xFF) as u8 as i8;
    let diy = ((word >> 8) & 0xFF) as u8 as i8;
    let dit = ((word >> 16) & 0xFF) as u8 as i8;
    (dix, diy, dit)
}

impl PackedTransitions {
    /// Unpack a flat DMA buffer into a `TransitionModel`. Mirrors `unpack_transitions.m`.
    pub fn unpack(&self) -> TransitionModel {
        let mut m = TransitionModel::default();
        for a in 0..N_ACTIONS {
            for it in 0..N_THETA {
                let base = (a * N_THETA + it) * TRANS_WORD_STRIDE;
                let n_out = self.0[base] as usize;
                m.n_outcomes[a][it] = n_out as u8;
                for k in 0..n_out {
                    let (dix, diy, dit) = unpack_delta(self.0[base + 2 * k + 1]);
                    m.dix[a][it][k] = dix;
                    m.diy[a][it][k] = diy;
                    m.dit[a][it][k] = dit;
                    m.prob[a][it][k] = self.0[base + 2 * k + 2];
                }
            }
        }
        m
    }
}

impl TransitionModel {
    /// Pack outcomes into a flat DMA buffer. Mirrors `gen_transitions.m::pack_model`.
    pub fn pack(&self) -> PackedTransitions {
        let mut v = vec![0u32; TRANS_TABLE_SIZE];
        for a in 0..N_ACTIONS {
            for it in 0..N_THETA {
                let base = (a * N_THETA + it) * TRANS_WORD_STRIDE;
                let n_out = self.n_outcomes[a][it] as usize;
                v[base] = n_out as u32;
                for k in 0..n_out {
                    v[base + 2 * k + 1] = pack_delta(self.dix[a][it][k], self.diy[a][it][k], self.dit[a][it][k]);
                    v[base + 2 * k + 2] = self.prob[a][it][k];
                }
            }
        }
        PackedTransitions(v)
    }

    /// Return the maximum absolute displacement `(dx, dy, dtheta)` across all actions and outcomes.
    pub fn max_displacement(&self) -> (u8, u8, u8) {
        let mut mx: u8 = 0;
        let mut my: u8 = 0;
        let mut mt: u8 = 0;
        for a in 0..N_ACTIONS {
            for it in 0..N_THETA {
                let n_out = self.n_outcomes[a][it] as usize;
                for k in 0..n_out {
                    // WHY: unsigned_abs avoids i8::MIN panic in debug builds
                    mx = mx.max(self.dix[a][it][k].unsigned_abs());
                    my = my.max(self.diy[a][it][k].unsigned_abs());
                    mt = mt.max(self.dit[a][it][k].unsigned_abs());
                }
            }
        }
        (mx, my, mt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::PROB_BASE;

    #[test]
    fn roundtrip_pack_unpack() {
        let mut m = TransitionModel::default();
        m.n_outcomes[0][0] = 2;
        m.dix[0][0][0] = 3;
        m.diy[0][0][0] = -1;
        m.dit[0][0][0] = 0;
        m.prob[0][0][0] = PROB_BASE;
        m.dix[0][0][1] = -2;
        m.diy[0][0][1] = 4;
        m.dit[0][0][1] = 1;
        m.prob[0][0][1] = PROB_BASE / 2;

        m.n_outcomes[1][5] = 1;
        m.dix[1][5][0] = 0;
        m.diy[1][5][0] = 0;
        m.dit[1][5][0] = -3;
        m.prob[1][5][0] = PROB_BASE;

        let packed = m.pack();
        let u = packed.unpack();

        assert_eq!(u.n_outcomes[0][0], 2);
        assert_eq!(u.dix[0][0][0], 3);
        assert_eq!(u.diy[0][0][0], -1);
        assert_eq!(u.dit[0][0][0], 0);
        assert_eq!(u.prob[0][0][0], PROB_BASE);
        assert_eq!(u.dix[0][0][1], -2);
        assert_eq!(u.diy[0][0][1], 4);
        assert_eq!(u.dit[0][0][1], 1);
        assert_eq!(u.prob[0][0][1], PROB_BASE / 2);

        assert_eq!(u.n_outcomes[1][5], 1);
        assert_eq!(u.dit[1][5][0], -3);
        assert_eq!(u.prob[1][5][0], PROB_BASE);
    }

    #[test]
    fn pack_size_is_constant() {
        let packed = TransitionModel::default().pack();
        assert_eq!(packed.0.len(), TRANS_TABLE_SIZE);
    }

    #[test]
    fn negative_offset_roundtrip() {
        let mut m = TransitionModel::default();
        m.n_outcomes[0][0] = 1;
        m.dix[0][0][0] = -3;
        m.diy[0][0][0] = -127;
        m.dit[0][0][0] = -1;
        m.prob[0][0][0] = PROB_BASE;

        let u = m.pack().unpack();
        assert_eq!(u.dix[0][0][0], -3);
        assert_eq!(u.diy[0][0][0], -127);
        assert_eq!(u.dit[0][0][0], -1);
    }

    #[test]
    fn roundtrip_with_max_outcomes() {
        let mut m = TransitionModel::default();
        // Fill (a=0, it=0) with all MAX_OUTCOMES slots using distinct values.
        m.n_outcomes[0][0] = MAX_OUTCOMES as u8;
        for k in 0..MAX_OUTCOMES {
            m.dix[0][0][k] = (k as i8) - 5;
            m.diy[0][0][k] = (k as i8) * 2 - 9;
            m.dit[0][0][k] = (k as i8) - 4;
            m.prob[0][0][k] = PROB_BASE - k as u32 * 1000;
        }
        // Give every other (a, it) exactly 1 outcome so the table is non-trivial.
        for a in 0..N_ACTIONS {
            for it in 0..N_THETA {
                if a == 0 && it == 0 {
                    continue;
                }
                m.n_outcomes[a][it] = 1;
                m.dix[a][it][0] = 0;
                m.diy[a][it][0] = 0;
                m.dit[a][it][0] = 0;
                m.prob[a][it][0] = PROB_BASE;
            }
        }

        let packed = m.pack();
        let u = packed.unpack();

        assert_eq!(u.n_outcomes[0][0], MAX_OUTCOMES as u8);
        for k in 0..MAX_OUTCOMES {
            assert_eq!(u.dix[0][0][k], (k as i8) - 5, "dix mismatch at k={k}");
            assert_eq!(u.diy[0][0][k], (k as i8) * 2 - 9, "diy mismatch at k={k}");
            assert_eq!(u.dit[0][0][k], (k as i8) - 4, "dit mismatch at k={k}");
            assert_eq!(u.prob[0][0][k], PROB_BASE - k as u32 * 1000, "prob mismatch at k={k}");
        }
    }

    #[test]
    fn max_displacement_finds_max() {
        let mut m = TransitionModel::default();
        m.n_outcomes[0][0] = 3;
        m.dix[0][0][0] = -5;
        m.dix[0][0][1] = 2;
        m.dix[0][0][2] = 0;
        for k in 0..3 {
            m.prob[0][0][k] = PROB_BASE;
        }

        let (mx, _, _) = m.max_displacement();
        assert_eq!(mx, 5);
    }
}
