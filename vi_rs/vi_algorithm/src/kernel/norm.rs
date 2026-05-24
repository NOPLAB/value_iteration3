//! Probability-sum-normalized Bellman backup for top-k pruned models.
//!
//! Used by [`crate::frontier::Frontier3DTopK`]. Returns `MAX_VALUE` if no
//! valid action exists (all outcomes out-of-bounds or obstacle-blocked).
//!
//! Mirrors `vi_frontier_bellman_norm` inside
//! `vi_matlab/src/cpu/frontier/vi_frontier_3d_topk.m`.
//! See spec §4.2, §4.8.

use ndarray::{Array2, Array3};
use vi_core::cost_of;
use vi_core::params::{MAX_VALUE, N_ACTIONS, N_THETA};
use vi_core::{Penalty, TransitionModel, Value};

/// Single-cell Bellman backup normalised by the sum of retained probabilities.
///
/// Unlike [`crate::kernel::bellman_backup`] (which divides by `PROB_BASE`),
/// this version divides by `prob_sum` — the sum of probabilities of the
/// top-k outcomes actually used. This corrects the expectation when some
/// outcomes have been pruned away.
///
/// Returns `MAX_VALUE` when no action produces a finite-cost result.
///
/// # Note
///
/// NOTE: structurally near-duplicate of bellman_backup in kernel/bellman.rs.
/// Differs only in divisor (prob_sum vs PROB_BASE) and prob_sum==0 guard.
/// Deduplication via a shared inner-loop helper deferred to a future cleanup.
#[allow(clippy::too_many_arguments)]
pub fn bellman_backup_norm(
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
        if n_out == 0 {
            continue;
        }

        // max accum: MAX_VALUE * PROB_BASE * MAX_OUTCOMES = 65535 * 262144 * 10 ≈ 2^37, << u64::MAX
        // max prob_sum: PROB_BASE * MAX_OUTCOMES = 2^18 * 10 ≈ 2^22, << u64::MAX
        let mut accum: u64 = 0;
        let mut prob_sum: u64 = 0;
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

            let prob = trans.prob[a][it_us][k] as u64;
            accum += step_cost as u64 * prob;
            prob_sum += prob;
        }

        let c: Value = if !valid || prob_sum == 0 {
            MAX_VALUE
        } else {
            // Mirror MATLAB: floor(accum / prob_sum), cap at MV-1.
            let div = accum / prob_sum;
            if div >= MAX_VALUE as u64 {
                MAX_VALUE - 1
            } else {
                div as Value
            }
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
    use ndarray::{Array2, Array3};
    use vi_core::params::{N_THETA, PENALTY_OBSTACLE, PROB_BASE};
    use vi_core::{Penalty, TransitionModel, Value};

    fn single_outcome_trans(a: usize, it: usize, dix: i8, diy: i8, dit: i8) -> TransitionModel {
        let mut tm = TransitionModel::default();
        tm.n_outcomes[a][it] = 1;
        tm.dix[a][it][0] = dix;
        tm.diy[a][it][0] = diy;
        tm.dit[a][it][0] = dit;
        tm.prob[a][it][0] = PROB_BASE;
        tm
    }

    #[test]
    fn single_outcome_identical_to_bellman_backup() {
        // With one outcome and prob = PROB_BASE, norm and standard should agree.
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::from_elem((3, 3, N_THETA), MAX_VALUE);
        value[[0, 1, 0]] = 100;
        let penalty = Array2::<Penalty>::zeros((3, 3));
        let trans = single_outcome_trans(0, 0, 1, 0, 0);

        let result = bellman_backup_norm(&value, &penalty, &trans, 0, 0, 0, map_x, map_y);
        // cost_of(100, 0) = 101; prob_sum = PROB_BASE; 101 * PROB_BASE / PROB_BASE = 101
        assert_eq!(result, 101);
    }

    #[test]
    fn two_outcomes_normalises_by_sum() {
        // Two outcomes with prob = PROB_BASE/2 each.
        // The standard backup divides by PROB_BASE; norm divides by PROB_BASE.
        // When probs sum to PROB_BASE the result is identical.
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::from_elem((3, 3, N_THETA), MAX_VALUE);
        value[[1, 2, 0]] = 200; // iy=1, ix=2
        value[[1, 0, 0]] = 100; // iy=1, ix=0
        let penalty = Array2::<Penalty>::zeros((3, 3));

        let mut tm = TransitionModel::default();
        tm.n_outcomes[0][0] = 2;
        tm.dix[0][0][0] = 1;
        tm.prob[0][0][0] = PROB_BASE / 2;
        tm.dix[0][0][1] = -1;
        tm.prob[0][0][1] = PROB_BASE / 2;

        let result = bellman_backup_norm(&value, &penalty, &tm, 1, 1, 0, map_x, map_y);
        // cost_of(200,0)=201, cost_of(100,0)=101
        // accum = 201*(PB/2) + 101*(PB/2) = 302*(PB/2)
        // prob_sum = PB; result = 302*(PB/2) / PB = 151
        assert_eq!(result, 151);
    }

    #[test]
    fn two_pruned_outcomes_normalises_by_partial_sum() {
        // Pruned model: only 1 of 2 outcomes retained, prob = PROB_BASE/2.
        // prob_sum = PROB_BASE/2; result = cost * (PB/2) / (PB/2) = cost.
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::from_elem((3, 3, N_THETA), MAX_VALUE);
        value[[1, 2, 0]] = 200; // iy=1, ix=2, cost_of=201
        let penalty = Array2::<Penalty>::zeros((3, 3));

        let mut tm = TransitionModel::default();
        tm.n_outcomes[0][0] = 1;
        tm.dix[0][0][0] = 1; // → (2,1,0)
        tm.prob[0][0][0] = PROB_BASE / 2;

        let result = bellman_backup_norm(&value, &penalty, &tm, 1, 1, 0, map_x, map_y);
        // accum = 201 * (PB/2); prob_sum = PB/2; div = 201
        assert_eq!(result, 201);
    }

    #[test]
    fn obstacle_neighbor_returns_max_value() {
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let value = Array3::<Value>::zeros((3, 3, N_THETA));
        let mut penalty = Array2::<Penalty>::zeros((3, 3));
        penalty[[0, 1]] = PENALTY_OBSTACLE;
        let trans = single_outcome_trans(0, 0, 1, 0, 0);

        let result = bellman_backup_norm(&value, &penalty, &trans, 0, 0, 0, map_x, map_y);
        assert_eq!(result, MAX_VALUE);
    }

    #[test]
    fn out_of_bounds_returns_max_value() {
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let value = Array3::<Value>::zeros((3, 3, N_THETA));
        let penalty = Array2::<Penalty>::zeros((3, 3));
        let trans = single_outcome_trans(0, 0, -1, 0, 0);

        let result = bellman_backup_norm(&value, &penalty, &trans, 0, 0, 0, map_x, map_y);
        assert_eq!(result, MAX_VALUE);
    }

    #[test]
    fn zero_outcomes_returns_max_value() {
        // n_out == 0 for all actions → no valid action → MAX_VALUE.
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let value = Array3::<Value>::zeros((3, 3, N_THETA));
        let penalty = Array2::<Penalty>::zeros((3, 3));
        let trans = TransitionModel::default(); // all n_outcomes = 0

        let result = bellman_backup_norm(&value, &penalty, &trans, 1, 1, 0, map_x, map_y);
        assert_eq!(result, MAX_VALUE);
    }

    #[test]
    fn prob_sum_zero_returns_max_value() {
        // prob = 0 → prob_sum = 0 → c = MAX_VALUE.
        let map_x: u32 = 3;
        let map_y: u32 = 3;
        let mut value = Array3::<Value>::zeros((3, 3, N_THETA));
        value[[0, 1, 0]] = 50;
        let penalty = Array2::<Penalty>::zeros((3, 3));

        let mut tm = TransitionModel::default();
        tm.n_outcomes[0][0] = 1;
        tm.dix[0][0][0] = 1;
        tm.prob[0][0][0] = 0; // zero probability

        let result = bellman_backup_norm(&value, &penalty, &tm, 0, 0, 0, map_x, map_y);
        assert_eq!(result, MAX_VALUE);
    }
}
