//! Single-cell Bellman backup shared by all Solver variants.
//!
//! Mirrors `vi_matlab/src/cpu/frontier/vi_frontier_bellman.m`.
//! See `docs/superpowers/specs/2026-05-22-vi-rs-algorithm-port-design.md` §4.5.

use ndarray::{Array2, Array3};
use vi_core::{cost_of, Penalty, Value};
use vi_core::params::{MAX_VALUE, N_ACTIONS, N_THETA, PROB_BASE};
use vi_core::TransitionModel;

/// Single-cell Bellman backup. Returns the best-action expected cost for `(ix, iy, it)`.
/// Mirrors `vi_matlab/src/cpu/frontier/vi_frontier_bellman.m`.
///
/// Bit-exact with MATLAB `vi_full_reference` and the C reference in
/// `host/src/vi_reference_c.c`.
#[allow(clippy::too_many_arguments)]
pub fn bellman_backup(
    value: &Array3<Value>,
    penalty: &Array2<Penalty>,
    trans: &TransitionModel,
    ix: u32,
    iy: u32,
    it: u32,
    map_x: u32,
    map_y: u32,
) -> Value {
    let it_us = it as usize;
    let mut v_new = MAX_VALUE;
    for a in 0..N_ACTIONS {
        let n_out = trans.n_outcomes[a][it_us] as usize;
        // WHY: n_out == 0 means this (action, theta) pair has no defined transition.
        // MATLAB's empty loop produces c=0 (which would make the action appear optimal);
        // Rust treats it as undefined and skips. The right signal for undefined is
        // MAX_VALUE — staying with the initial v_new = MAX_VALUE accomplishes that.
        if n_out == 0 {
            continue;
        }
        // max accum: MAX_VALUE * PROB_BASE * MAX_OUTCOMES = 65535 * 262144 * 10 ≈ 2^37, << u64::MAX
        let mut accum: u64 = 0;
        let mut valid = true;
        for k in 0..n_out {
            let dix = trans.dix[a][it_us][k] as i32;
            let diy = trans.diy[a][it_us][k] as i32;
            let dit = trans.dit[a][it_us][k] as i32;
            let nx = ix as i32 + dix;
            let ny = iy as i32 + diy;
            let mut nt = it as i32 + dit;
            if nt < 0 {
                nt += N_THETA as i32;
            } else if nt >= N_THETA as i32 {
                nt -= N_THETA as i32;
            }
            if nx < 0 || nx >= map_x as i32 || ny < 0 || ny >= map_y as i32 {
                valid = false;
                break;
            }
            let step_cost = cost_of(
                value[[ny as usize, nx as usize, nt as usize]],
                penalty[[ny as usize, nx as usize]],
            );
            if step_cost == MAX_VALUE {
                valid = false;
                break;
            }
            accum += step_cost as u64 * trans.prob[a][it_us][k] as u64;
        }
        let c: Value = if !valid {
            MAX_VALUE
        } else {
            let div = accum / PROB_BASE as u64;
            if div >= MAX_VALUE as u64 { MAX_VALUE - 1 } else { div as Value }
        };
        if c < v_new {
            v_new = c;
        }
    }
    v_new
}

#[cfg(test)]
mod tests {
    use super::*;
    use vi_core::params::{MAX_VALUE, N_THETA, PROB_BASE, PENALTY_OBSTACLE};
    use vi_core::{Penalty, TransitionModel, Value};

    fn make_single_outcome_trans(a: usize, it: usize, dix: i8, diy: i8, dit: i8) -> TransitionModel {
        let mut tm = TransitionModel::default();
        tm.n_outcomes[a][it] = 1;
        tm.dix[a][it][0] = dix;
        tm.diy[a][it][0] = diy;
        tm.dit[a][it][0] = dit;
        tm.prob[a][it][0] = PROB_BASE;
        tm
    }

    #[test]
    fn single_outcome_deterministic() {
        // 3x3 map, action 0 at theta 0 moves dix=+1. Other actions have n_out=0.
        // value[[0,1,0]] = 100. From (ix=0, iy=0, it=0) → neighbor at (1,0,0).
        // cost_of(100, 0) = 101. accum = 101 * PROB_BASE. result = 101.
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::zeros((3, 3, N_THETA));
        value[[0, 1, 0]] = 100;
        value[[0, 2, 0]] = 200;
        let penalty = Array2::<Penalty>::zeros((3, 3));
        let trans = make_single_outcome_trans(0, 0, 1, 0, 0);

        let result = bellman_backup(&value, &penalty, &trans, 0, 0, 0, map_x, map_y);
        assert_eq!(result, 101);
    }

    #[test]
    fn obstacle_neighbor_returns_max_value() {
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let value = Array3::<Value>::zeros((3, 3, N_THETA));
        let mut penalty = Array2::<Penalty>::zeros((3, 3));
        // Neighbor at (1,0) is an obstacle.
        penalty[[0, 1]] = PENALTY_OBSTACLE;
        let trans = make_single_outcome_trans(0, 0, 1, 0, 0);

        let result = bellman_backup(&value, &penalty, &trans, 0, 0, 0, map_x, map_y);
        assert_eq!(result, MAX_VALUE);
    }

    #[test]
    fn out_of_bounds_neighbor_returns_max_value() {
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let value = Array3::<Value>::zeros((3, 3, N_THETA));
        let penalty = Array2::<Penalty>::zeros((3, 3));
        // dix=-1 from (0,0,0) goes out of bounds.
        let trans = make_single_outcome_trans(0, 0, -1, 0, 0);

        let result = bellman_backup(&value, &penalty, &trans, 0, 0, 0, map_x, map_y);
        assert_eq!(result, MAX_VALUE);
    }

    #[test]
    fn theta_wrap_negative() {
        // dit=-1 from (1,1,0) wraps to N_THETA-1.
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::zeros((3, 3, N_THETA));
        value[[1, 1, N_THETA - 1]] = 42;
        let penalty = Array2::<Penalty>::zeros((3, 3));
        let trans = make_single_outcome_trans(0, 0, 0, 0, -1);

        // From (ix=1, iy=1, it=0): neighbor at (1,1, N_THETA-1).
        // cost_of(42, 0) = 43. result = 43.
        let result = bellman_backup(&value, &penalty, &trans, 1, 1, 0, map_x, map_y);
        assert_eq!(result, 43);
    }

    #[test]
    fn theta_wrap_positive() {
        // dit=+1 from (1,1,N_THETA-1) wraps to 0.
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::zeros((3, 3, N_THETA));
        value[[1, 1, 0]] = 5;
        let penalty = Array2::<Penalty>::zeros((3, 3));
        let trans = make_single_outcome_trans(0, N_THETA - 1, 0, 0, 1);

        // From (ix=1, iy=1, it=N_THETA-1): neighbor at (1,1,0).
        // cost_of(5, 0) = 6. result = 6.
        let result = bellman_backup(&value, &penalty, &trans, 1, 1, (N_THETA - 1) as u32, map_x, map_y);
        assert_eq!(result, 6);
    }

    #[test]
    fn multiple_outcomes_probability_weighted() {
        // Two outcomes from (1,1,0) on a 3x3 free map.
        // outcome 0: dix=1, diy=0, dit=0, prob=PROB_BASE/2 → neighbor (2,1,0), value=200, cost=201
        // outcome 1: dix=-1, diy=0, dit=0, prob=PROB_BASE/2 → neighbor (0,1,0), value=100, cost=101
        // accum = 201*(PROB_BASE/2) + 101*(PROB_BASE/2) = 302*(PROB_BASE/2)
        // result = 302*(PROB_BASE/2) / PROB_BASE = 302/2 = 151
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::zeros((3, 3, N_THETA));
        value[[1, 2, 0]] = 200; // iy=1, ix=2
        value[[1, 0, 0]] = 100; // iy=1, ix=0
        let penalty = Array2::<Penalty>::zeros((3, 3));

        let mut tm = TransitionModel::default();
        tm.n_outcomes[0][0] = 2;
        tm.dix[0][0][0] = 1;
        tm.diy[0][0][0] = 0;
        tm.dit[0][0][0] = 0;
        tm.prob[0][0][0] = PROB_BASE / 2;
        tm.dix[0][0][1] = -1;
        tm.diy[0][0][1] = 0;
        tm.dit[0][0][1] = 0;
        tm.prob[0][0][1] = PROB_BASE / 2;

        let result = bellman_backup(&value, &penalty, &tm, 1, 1, 0, map_x, map_y);
        assert_eq!(result, 151);
    }

    #[test]
    fn picks_minimum_across_actions() {
        // Action 0 at it=0: moves dix=+1, value=199 → cost_of(199,0)=200
        // Action 1 at it=0: moves dix=-1 from (1,1,0), value=99 → cost_of(99,0)=100
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::zeros((3, 3, N_THETA));
        value[[1, 2, 0]] = 199; // iy=1, ix=2
        value[[1, 0, 0]] = 99;  // iy=1, ix=0
        let penalty = Array2::<Penalty>::zeros((3, 3));

        let mut tm = TransitionModel::default();
        // Action 0: dix=+1
        tm.n_outcomes[0][0] = 1;
        tm.dix[0][0][0] = 1;
        tm.diy[0][0][0] = 0;
        tm.dit[0][0][0] = 0;
        tm.prob[0][0][0] = PROB_BASE;
        // Action 1: dix=-1
        tm.n_outcomes[1][0] = 1;
        tm.dix[1][0][0] = -1;
        tm.diy[1][0][0] = 0;
        tm.dit[1][0][0] = 0;
        tm.prob[1][0][0] = PROB_BASE;

        let result = bellman_backup(&value, &penalty, &tm, 1, 1, 0, map_x, map_y);
        assert_eq!(result, 100);
    }

    #[test]
    fn all_actions_with_zero_outcomes_returns_max_value() {
        // WHY: locks in the Rust-side decision to treat n_out==0 as "undefined
        // action" (returning MAX_VALUE), diverging from MATLAB which returns 0.
        let map_x = 3u32;
        let map_y = 3u32;
        let value = Array3::<Value>::zeros((map_y as usize, map_x as usize, N_THETA));
        let penalty = Array2::<Penalty>::zeros((map_y as usize, map_x as usize));
        let trans = TransitionModel::default();  // all n_outcomes = 0
        let v = bellman_backup(&value, &penalty, &trans, 1, 1, 0, map_x, map_y);
        assert_eq!(v, MAX_VALUE);
    }

    #[test]
    fn multi_outcome_break_on_mid_loop_oob() {
        // Action with 2 outcomes. Outcome 0 is valid, outcome 1 is OOB.
        // The whole action is invalidated → returns MAX_VALUE for that action.
        // All other actions have n_out=0, so v_new stays at MAX_VALUE.
        //
        // From (ix=0, iy=0, it=0):
        //   outcome 0: dix=1, diy=0, dit=0 → (1, 0, 0), valid
        //   outcome 1: dix=-1, diy=0, dit=0 → (-1, 0, 0), OOB
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::zeros((3, 3, N_THETA));
        value[[0, 1, 0]] = 10;
        let penalty = Array2::<Penalty>::zeros((3, 3));

        let mut tm = TransitionModel::default();
        tm.n_outcomes[0][0] = 2;
        tm.dix[0][0][0] = 1;
        tm.diy[0][0][0] = 0;
        tm.dit[0][0][0] = 0;
        tm.prob[0][0][0] = PROB_BASE / 2;
        tm.dix[0][0][1] = -1;
        tm.diy[0][0][1] = 0;
        tm.dit[0][0][1] = 0;
        tm.prob[0][0][1] = PROB_BASE / 2;

        let result = bellman_backup(&value, &penalty, &tm, 0, 0, 0, map_x, map_y);
        assert_eq!(result, MAX_VALUE);
    }

    #[test]
    fn combined_spatial_and_theta_wrap() {
        // From (ix=1, iy=1, it=0): single outcome dix=1, diy=0, dit=-1.
        // Theta wraps: 0 + (-1) = -1 → N_THETA-1.
        // Neighbor accessed: value[[1, 2, N_THETA-1]].
        // cost_of(7, 0) = 7 + 0 + 1 = 8. Expected = 8.
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::zeros((3, 3, N_THETA));
        value[[1, 2, N_THETA - 1]] = 7;
        let penalty = Array2::<Penalty>::zeros((3, 3));
        let trans = make_single_outcome_trans(0, 0, 1, 0, -1);

        let result = bellman_backup(&value, &penalty, &trans, 1, 1, 0, map_x, map_y);
        assert_eq!(result, 8);
    }

    #[test]
    fn three_outcomes_with_rounding() {
        // Action with 3 outcomes, each prob = PROB_BASE / 3 = 87381 (loses 1 per outcome).
        // All three neighbors have value=100, penalty=0.
        // cost_of(100, 0) = 101 each.
        // accum = 3 * (101 * 87381) = 3 * 8_825_481 = 26_476_443
        // result = 26_476_443 / 262_144 = 100
        // (since 101 * 262_144 = 26_476_544 > 26_476_443, integer div gives 100)
        // This documents that PROB_BASE not being divisible by 3 rounds DOWN.
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::zeros((3, 3, N_THETA));
        value[[1, 2, 0]] = 100; // dix=+1 → (2, 1, 0)
        value[[1, 0, 0]] = 100; // dix=-1 → (0, 1, 0)
        value[[2, 1, 0]] = 100; // diy=+1 → (1, 2, 0)
        let penalty = Array2::<Penalty>::zeros((3, 3));

        let mut tm = TransitionModel::default();
        tm.n_outcomes[0][0] = 3;
        tm.dix[0][0][0] = 1;
        tm.diy[0][0][0] = 0;
        tm.dit[0][0][0] = 0;
        tm.prob[0][0][0] = PROB_BASE / 3;
        tm.dix[0][0][1] = -1;
        tm.diy[0][0][1] = 0;
        tm.dit[0][0][1] = 0;
        tm.prob[0][0][1] = PROB_BASE / 3;
        tm.dix[0][0][2] = 0;
        tm.diy[0][0][2] = 1;
        tm.dit[0][0][2] = 0;
        tm.prob[0][0][2] = PROB_BASE / 3;

        let result = bellman_backup(&value, &penalty, &tm, 1, 1, 0, map_x, map_y);
        assert_eq!(result, 100);
    }
}
