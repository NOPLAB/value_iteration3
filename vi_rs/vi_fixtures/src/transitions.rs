//! Transition table generation. Mirrors `vi_matlab/workflows/validation/tests/gen_transitions.m`.

use std::collections::HashMap;
use std::sync::Mutex;
use once_cell::sync::Lazy;
use vi_core::{PackedTransitions, TransitionModel,
              N_ACTIONS, N_THETA, PROB_BASE,
              ACTION_FW, ACTION_ROT,
              RESOLUTION_XY_BIT, RESOLUTION_T_BIT,
              params::MAX_OUTCOMES};

#[derive(Clone, Copy, Debug)]
pub enum TransitionMode {
    Trivial,
    Full { xy_resolution: f64 },
    PaperMonteCarlo { xy_resolution: f64 },
}

static CACHE: Lazy<Mutex<HashMap<(u8, u64), PackedTransitions>>> =
    Lazy::new(|| Mutex::new(HashMap::new()));

fn cache_key(mode: &TransitionMode) -> (u8, u64) {
    match mode {
        TransitionMode::Trivial => (0, 0),
        TransitionMode::Full { xy_resolution } => (1, xy_resolution.to_bits()),
        TransitionMode::PaperMonteCarlo { xy_resolution } => (2, xy_resolution.to_bits()),
    }
}

pub fn generate_transitions(mode: TransitionMode) -> PackedTransitions {
    let key = cache_key(&mode);
    let mut cache = CACHE.lock().unwrap();
    if let Some(cached) = cache.get(&key) {
        return PackedTransitions(cached.0.clone());
    }
    let result = match mode {
        TransitionMode::Trivial => build_trivial(),
        TransitionMode::Full { xy_resolution } => build_full(xy_resolution),
        TransitionMode::PaperMonteCarlo { xy_resolution } => build_paper_mc(xy_resolution),
    };
    cache.insert(key, PackedTransitions(result.0.clone()));
    result
}

fn build_trivial() -> PackedTransitions {
    let mut m = TransitionModel::default();
    for it in 0..N_THETA {
        m.n_outcomes[0][it] = 1;
        m.dix[0][it][0] = 1;
        m.prob[0][it][0] = PROB_BASE;

        m.n_outcomes[1][it] = 1;
        m.dix[1][it][0] = -1;
        m.prob[1][it][0] = PROB_BASE;

        for a in 2..N_ACTIONS {
            m.n_outcomes[a][it] = 1;
            m.prob[a][it][0] = PROB_BASE;
        }
    }
    m.pack()
}

fn build_full(xy_resolution: f64) -> PackedTransitions {
    let mut m = TransitionModel::default();
    let t_resolution = 360.0 / N_THETA as f64;

    for a in 0..N_ACTIONS {
        for it in 0..N_THETA {
            let theta_deg = it as f64 * t_resolution + 0.5 * t_resolution;
            let theta_rad = theta_deg.to_radians();
            let dx = ACTION_FW[a] * theta_rad.cos();
            let dy = ACTION_FW[a] * theta_rad.sin();
            let dix = (dx / xy_resolution).floor() as i8;
            let diy = (dy / xy_resolution).floor() as i8;

            let mut new_theta = theta_deg + ACTION_ROT[a];
            while new_theta < 0.0 { new_theta += 360.0; }
            while new_theta >= 360.0 { new_theta -= 360.0; }
            let new_it = (new_theta / t_resolution).floor() as i32;
            let mut dit = new_it - it as i32;
            if dit > N_THETA as i32 / 2 { dit -= N_THETA as i32; }
            if dit < -(N_THETA as i32 / 2) { dit += N_THETA as i32; }

            m.n_outcomes[a][it] = 1;
            m.dix[a][it][0] = dix;
            m.diy[a][it][0] = diy;
            m.dit[a][it][0] = dit as i8;
            m.prob[a][it][0] = PROB_BASE;
        }
    }
    m.pack()
}

fn build_paper_mc(xy_resolution: f64) -> PackedTransitions {
    let t_resolution = 360.0 / N_THETA as f64;
    let xy_sample_num = 1u32 << RESOLUTION_XY_BIT; // 64
    let t_sample_num = 1u32 << RESOLUTION_T_BIT;   // 64
    let xy_step = xy_resolution / xy_sample_num as f64;
    let t_step = t_resolution / t_sample_num as f64;

    let ox_vals: Vec<f64> = (0..xy_sample_num).map(|i| 0.5 * xy_step + i as f64 * xy_step).collect();
    let oy_vals: Vec<f64> = (0..xy_sample_num).map(|i| 0.5 * xy_step + i as f64 * xy_step).collect();
    let ot_vals: Vec<f64> = (0..t_sample_num).map(|i| 0.5 * t_step + i as f64 * t_step).collect();

    let mut m = TransitionModel::default();

    for a in 0..N_ACTIONS {
        for it in 0..N_THETA {
            let theta_origin = it as f64 * t_resolution;
            let mut counts: std::collections::BTreeMap<(i32, i32, i32), u32> =
                std::collections::BTreeMap::new();

            for oy in &oy_vals {
                for ox in &ox_vals {
                    for ot in &ot_vals {
                        let ang = (ot + theta_origin).to_radians();
                        let dx = ox + ACTION_FW[a] * ang.cos();
                        let dy = oy + ACTION_FW[a] * ang.sin();
                        let dt = ((ot + theta_origin + ACTION_ROT[a]) % 360.0 + 360.0) % 360.0;

                        // MATLAB convention: floor(abs/res), negate-and-subtract-1 for negative
                        let dix = if dx >= 0.0 {
                            (dx / xy_resolution).floor() as i32
                        } else {
                            -((dx.abs() / xy_resolution).floor() as i32) - 1
                        };
                        let diy = if dy >= 0.0 {
                            (dy / xy_resolution).floor() as i32
                        } else {
                            -((dy.abs() / xy_resolution).floor() as i32) - 1
                        };

                        let dit_abs = (dt / t_resolution).floor() as i32;
                        let mut dit = dit_abs - it as i32;
                        if dit > N_THETA as i32 / 2 { dit -= N_THETA as i32; }
                        if dit < -(N_THETA as i32 / 2) { dit += N_THETA as i32; }

                        *counts.entry((dix, diy, dit)).or_insert(0) += 1;
                    }
                }
            }

            let n_out = counts.len();
            assert!(n_out <= MAX_OUTCOMES,
                "MAX_OUTCOMES too small: got {n_out} for a={a} it={it}");

            m.n_outcomes[a][it] = n_out as u8;
            for (k, (&(dix, diy, dit), &count)) in counts.iter().enumerate() {
                m.dix[a][it][k] = dix as i8;
                m.diy[a][it][k] = diy as i8;
                m.dit[a][it][k] = dit as i8;
                m.prob[a][it][k] = count;
            }
        }
    }
    m.pack()
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Trivial mode tests ---

    #[test]
    fn trivial_action0_moves_plus_x() {
        let packed = generate_transitions(TransitionMode::Trivial);
        let m = packed.unpack();
        for it in 0..N_THETA {
            assert_eq!(m.n_outcomes[0][it], 1);
            assert_eq!(m.dix[0][it][0], 1);
            assert_eq!(m.diy[0][it][0], 0);
            assert_eq!(m.dit[0][it][0], 0);
            assert_eq!(m.prob[0][it][0], PROB_BASE);
        }
    }

    #[test]
    fn trivial_action1_moves_minus_x() {
        let packed = generate_transitions(TransitionMode::Trivial);
        let m = packed.unpack();
        for it in 0..N_THETA {
            assert_eq!(m.n_outcomes[1][it], 1);
            assert_eq!(m.dix[1][it][0], -1);
            assert_eq!(m.diy[1][it][0], 0);
            assert_eq!(m.dit[1][it][0], 0);
            assert_eq!(m.prob[1][it][0], PROB_BASE);
        }
    }

    #[test]
    fn trivial_actions_2_through_5_are_noop() {
        let packed = generate_transitions(TransitionMode::Trivial);
        let m = packed.unpack();
        for a in 2..N_ACTIONS {
            for it in 0..N_THETA {
                assert_eq!(m.n_outcomes[a][it], 1);
                assert_eq!(m.dix[a][it][0], 0);
                assert_eq!(m.diy[a][it][0], 0);
                assert_eq!(m.dit[a][it][0], 0);
                assert_eq!(m.prob[a][it][0], PROB_BASE);
            }
        }
    }

    #[test]
    fn trivial_roundtrip_preserves_table() {
        let packed = generate_transitions(TransitionMode::Trivial);
        let m = packed.unpack();
        let repacked = m.pack();
        assert_eq!(packed.0, repacked.0);
    }

    // --- Full mode tests ---

    #[test]
    fn full_all_deterministic() {
        let packed = generate_transitions(TransitionMode::Full { xy_resolution: 0.05 });
        let m = packed.unpack();
        for a in 0..N_ACTIONS {
            for it in 0..N_THETA {
                assert_eq!(m.n_outcomes[a][it], 1,
                    "full mode is deterministic: a={a} it={it}");
                assert_eq!(m.prob[a][it][0], PROB_BASE);
            }
        }
    }

    #[test]
    fn full_action0_theta0_forward_positive_x() {
        let packed = generate_transitions(TransitionMode::Full { xy_resolution: 0.05 });
        let m = packed.unpack();
        assert_eq!(m.dix[0][0][0], 5, "forward action at theta=0 moves +5 in x");
        assert_eq!(m.diy[0][0][0], 0);
        assert_eq!(m.dit[0][0][0], 0);
    }

    #[test]
    fn full_action0_theta15_forward_positive_y() {
        let packed = generate_transitions(TransitionMode::Full { xy_resolution: 0.05 });
        let m = packed.unpack();
        assert_eq!(m.dix[0][15][0], -1);
        assert_eq!(m.diy[0][15][0], 5);
        assert_eq!(m.dit[0][15][0], 0);
    }

    #[test]
    fn full_action2_rotates_minus20() {
        let packed = generate_transitions(TransitionMode::Full { xy_resolution: 0.05 });
        let m = packed.unpack();
        for it in 0..N_THETA {
            assert_eq!(m.dix[2][it][0], 0, "pure rotation, no x displacement");
            assert_eq!(m.diy[2][it][0], 0, "pure rotation, no y displacement");
        }
        assert_eq!(m.dit[2][5][0], -3);
    }

    #[test]
    fn full_dit_wraps_around_at_boundaries() {
        let packed = generate_transitions(TransitionMode::Full { xy_resolution: 0.05 });
        let m = packed.unpack();
        assert_eq!(m.dit[2][0][0], -3);
    }

    // --- PaperMonteCarlo mode tests ---

    #[test]
    fn paper_mc_prob_sum_equals_prob_base() {
        let packed = generate_transitions(TransitionMode::PaperMonteCarlo { xy_resolution: 0.05 });
        let m = packed.unpack();
        for a in 0..N_ACTIONS {
            for it in 0..N_THETA {
                let n = m.n_outcomes[a][it] as usize;
                let sum: u32 = (0..n).map(|k| m.prob[a][it][k]).sum();
                assert_eq!(sum, PROB_BASE,
                    "prob sum must equal PROB_BASE for a={a} it={it}, got {sum}");
            }
        }
    }

    #[test]
    fn paper_mc_has_multiple_outcomes() {
        let packed = generate_transitions(TransitionMode::PaperMonteCarlo { xy_resolution: 0.05 });
        let m = packed.unpack();
        let multi = (0..N_ACTIONS).flat_map(|a| (0..N_THETA).map(move |it| (a, it)))
            .any(|(a, it)| m.n_outcomes[a][it] > 1);
        assert!(multi, "paper_mc should produce multi-outcome transitions");
    }

    #[test]
    fn paper_mc_n_outcomes_within_max() {
        let packed = generate_transitions(TransitionMode::PaperMonteCarlo { xy_resolution: 0.05 });
        let m = packed.unpack();
        for a in 0..N_ACTIONS {
            for it in 0..N_THETA {
                assert!(m.n_outcomes[a][it] as usize <= vi_core::params::MAX_OUTCOMES,
                    "n_outcomes out of range for a={a} it={it}");
            }
        }
    }

    #[test]
    fn paper_mc_outcomes_sorted_by_delta() {
        let packed = generate_transitions(TransitionMode::PaperMonteCarlo { xy_resolution: 0.05 });
        let m = packed.unpack();
        for a in 0..N_ACTIONS {
            for it in 0..N_THETA {
                let n = m.n_outcomes[a][it] as usize;
                for k in 1..n {
                    let prev = (m.dix[a][it][k-1], m.diy[a][it][k-1], m.dit[a][it][k-1]);
                    let curr = (m.dix[a][it][k], m.diy[a][it][k], m.dit[a][it][k]);
                    assert!(prev <= curr,
                        "outcomes must be sorted by (dix,diy,dit) for a={a} it={it}");
                }
            }
        }
    }

    #[test]
    fn paper_mc_roundtrip_preserves_table() {
        let packed = generate_transitions(TransitionMode::PaperMonteCarlo { xy_resolution: 0.05 });
        let m = packed.unpack();
        let repacked = m.pack();
        assert_eq!(packed.0, repacked.0);
    }

    #[test]
    fn paper_mc_cache_returns_same_result() {
        let p1 = generate_transitions(TransitionMode::PaperMonteCarlo { xy_resolution: 0.05 });
        let p2 = generate_transitions(TransitionMode::PaperMonteCarlo { xy_resolution: 0.05 });
        assert_eq!(p1.0, p2.0);
    }
}
