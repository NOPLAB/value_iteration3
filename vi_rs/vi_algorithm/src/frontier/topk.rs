//! Approximate 3D frontier VI using top-k outcome pruning.
//!
//! Keeps the highest-probability transitions per (action, theta) and
//! normalizes the Bellman expectation by the retained probability mass.
//! Mirrors `vi_matlab/src/cpu/frontier/vi_frontier_3d_topk.m`.
//! See spec §4.2, §4.8.

use vi_core::params::{MAX_OUTCOMES, N_ACTIONS, N_THETA};
use vi_core::TransitionModel;

use crate::bitboard::Bitboard3D;
use crate::context::{Budget, SolveStats, Solver, VIContext};
use crate::kernel::{bellman_backup, bellman_backup_norm};

use super::{
    build_passable_bb_2d, build_passable_bb_3d, build_value_seed_3d, max_iters, pin_goals,
};

/// Frontier-VI solver that prunes the transition model to the top-k
/// highest-probability outcomes per (action, theta), then runs the standard
/// frontier loop using probability-sum-normalized Bellman backups.
///
/// When `v_new == MAX_VALUE` after the norm backup (no finite action via the
/// pruned model), the solver falls back to a full-model standard backup for
/// that cell — mirroring the MATLAB fallback.
///
/// * `k >= MAX_OUTCOMES` → no pruning → behaviour equivalent to [`Frontier3D`]
///   (when every (action,theta) has exactly one outcome, as in the deterministic
///   4-dir test fixture, the result is bit-exact).
pub struct Frontier3DTopK {
    /// Number of highest-probability outcomes per (action, theta) to retain.
    /// `k >= MAX_OUTCOMES` is a no-op (no pruning). Clamped to `[1, MAX_OUTCOMES]`.
    pub k: u32,
}

impl Solver for Frontier3DTopK {
    fn name(&self) -> &'static str {
        "frontier_3d_topk"
    }

    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        let max_iter = max_iters(budget);
        let map_x = ctx.dims.map_x;
        let map_y = ctx.dims.map_y;

        // Build pruned model (full model is retained for fallback).
        let full_trans = ctx.transitions.clone();
        let pruned = prune_topk(&full_trans, self.k);

        let (mx, my, mt) = pruned.max_displacement();
        let mx = mx as u32;
        let my = my as u32;
        let mt = mt as u32;

        pin_goals(&mut ctx.value, &ctx.goal_mask);

        let passable_2d = build_passable_bb_2d(&ctx.penalty);
        let passable_bb = build_passable_bb_3d(&passable_2d, N_THETA as u32);

        let goal_bb = Bitboard3D::from_logical(ctx.goal_mask.view());
        let not_goal_bb = goal_bb.complement();

        let mut frontier = build_value_seed_3d(&ctx.value);

        let mut updates: u64 = 0;
        let mut iters: u32 = 0;

        while frontier.popcount() > 0 && iters < max_iter {
            iters += 1;

            let mut candidates = frontier.dilate(mx, my, mt);
            candidates.and_inplace(&passable_bb);
            candidates.and_inplace(&not_goal_bb);

            let mut new_frontier = Bitboard3D::new(map_x, map_y, N_THETA as u32);

            for (ix, iy, it) in candidates.enumerate() {
                let ix_us = ix as usize;
                let iy_us = iy as usize;
                let it_us = it as usize;
                let old_val = ctx.value[[iy_us, ix_us, it_us]];

                // Pruned-model normalised backup.
                let mut new_val = bellman_backup_norm(
                    &ctx.value,
                    &ctx.penalty,
                    &pruned,
                    ix,
                    iy,
                    it,
                    map_x,
                    map_y,
                );

                // WHY: defensive fallback mirroring MATLAB vi_frontier_3d_topk.m. The pruned
                // model's outcomes are a strict subset of the full model's, so if the pruned
                // backup returns MAX_VALUE (all actions invalid in pruned), the full backup
                // will also return MAX_VALUE (same OOB outcomes are present). In practice
                // this path is dead code — kept for MATLAB parity and as a guard against
                // future pruning-semantics changes.
                if new_val == vi_core::MAX_VALUE {
                    new_val = bellman_backup(
                        &ctx.value,
                        &ctx.penalty,
                        &full_trans,
                        ix,
                        iy,
                        it,
                        map_x,
                        map_y,
                    );
                }

                if new_val < old_val {
                    ctx.value[[iy_us, ix_us, it_us]] = new_val;
                    updates += 1;
                    new_frontier.set(ix, iy, it);
                }
            }

            frontier = new_frontier;
        }

        // Defensive goal re-pin (mirrors MATLAB's end-of-function pin).
        pin_goals(&mut ctx.value, &ctx.goal_mask);

        let converged = frontier.popcount() == 0;

        SolveStats {
            iters_or_sweeps: iters,
            updates,
            final_delta: 0,
            converged,
            extra: None,
        }
    }
}

/// Prune a [`TransitionModel`] to retain at most `k` highest-probability
/// outcomes per (action, theta).
///
/// The selection is a stable greedy pick (maximum first, ties broken by
/// original index order — matching MATLAB's `>` comparison which keeps the
/// first-encountered winner). Outcomes below the top-k are zeroed out.
///
/// Mirrors `vi_frontier_prune_topk` in
/// `vi_matlab/src/cpu/frontier/vi_frontier_3d_topk.m`.
fn prune_topk(model: &TransitionModel, k: u32) -> TransitionModel {
    let keep = (k as usize).clamp(1, MAX_OUTCOMES);
    let mut pruned = TransitionModel::default();

    for a in 0..N_ACTIONS {
        for it in 0..N_THETA {
            let n = model.n_outcomes[a][it] as usize;
            if n <= keep {
                // Copy as-is — no pruning needed.
                pruned.n_outcomes[a][it] = n as u8;
                for k_idx in 0..n {
                    pruned.dix[a][it][k_idx] = model.dix[a][it][k_idx];
                    pruned.diy[a][it][k_idx] = model.diy[a][it][k_idx];
                    pruned.dit[a][it][k_idx] = model.dit[a][it][k_idx];
                    pruned.prob[a][it][k_idx] = model.prob[a][it][k_idx];
                }
                continue;
            }

            // Greedy selection: pick the `keep` highest-prob outcomes.
            // Ties: first-encountered wins (strict `>` preserves original order).
            // Mirrors MATLAB's used-mask algorithm exactly.
            let mut used = [false; MAX_OUTCOMES];
            for dst in 0..keep {
                let mut best_k = 0usize;
                let mut best_prob: i64 = -1;
                for (k_idx, &already_used) in used.iter().enumerate().take(n) {
                    let p = model.prob[a][it][k_idx] as i64;
                    if !already_used && p > best_prob {
                        best_prob = p;
                        best_k = k_idx;
                    }
                }
                used[best_k] = true;
                pruned.dix[a][it][dst] = model.dix[a][it][best_k];
                pruned.diy[a][it][dst] = model.diy[a][it][best_k];
                pruned.dit[a][it][dst] = model.dit[a][it][best_k];
                pruned.prob[a][it][dst] = model.prob[a][it][best_k];
            }
            pruned.n_outcomes[a][it] = keep as u8;
        }
    }

    pruned
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::Budget;
    use crate::frontier::Frontier3D;
    use crate::reference::Reference;
    use vi_core::params::{MAX_VALUE, PROB_BASE};
    use super::super::test_helpers::empty_5x5_ctx;

    #[test]
    fn topk_full_matches_frontier_3d() {
        // k >= MAX_OUTCOMES → no pruning occurs.
        // The deterministic 4-dir test fixture has n_out=1 for every (a,it),
        // so pruning to k=10 is a no-op → results must be bit-exact with Frontier3D.
        let mut ctx_topk = empty_5x5_ctx();
        let mut ctx_f3d = ctx_topk.clone_value();

        Frontier3DTopK { k: 10 }.run(&mut ctx_topk, Budget::Iterations(200));
        Frontier3D.run(&mut ctx_f3d, Budget::Iterations(200));

        assert_eq!(
            ctx_topk.value, ctx_f3d.value,
            "k=10 (no-op pruning) must be bit-exact with Frontier3D"
        );
    }

    #[test]
    fn topk_converges() {
        let mut ctx = empty_5x5_ctx();
        let stats = Frontier3DTopK { k: 2 }.run(&mut ctx, Budget::Iterations(200));
        assert!(stats.converged, "TopK k=2 must converge within 200 iters");
    }

    #[test]
    fn topk_close_to_reference() {
        // The deterministic 4-dir fixture has n_out=1 → pruning to k=3 is a no-op.
        // Both Reference (layer 0) and TopK (layer 0) should agree exactly.
        let mut ctx_ref = empty_5x5_ctx();
        let mut ctx_topk = empty_5x5_ctx();

        Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(100));
        Frontier3DTopK { k: 3 }.run(&mut ctx_topk, Budget::Iterations(200));

        // Compare layer 0 only (dit=0, so other layers are unreachable).
        let count = 5 * 5;
        let mean_abs_diff: f64 = (0..5usize)
            .flat_map(|iy| (0..5usize).map(move |ix| (iy, ix)))
            .map(|(iy, ix)| {
                let r = ctx_ref.value[[iy, ix, 0]];
                let t = ctx_topk.value[[iy, ix, 0]];
                (r as i32 - t as i32).abs() as f64
            })
            .sum::<f64>()
            / count as f64;

        assert!(
            mean_abs_diff < 1.0,
            "layer-0 mean abs diff vs Reference = {:.2} (k=3)",
            mean_abs_diff
        );
    }

    #[test]
    fn prune_topk_no_op_when_n_leq_k() {
        // When n_out <= k, prune_topk must copy unchanged.
        use vi_core::TransitionModel;
        let mut model = TransitionModel::default();
        model.n_outcomes[0][0] = 2;
        model.dix[0][0][0] = 1;
        model.diy[0][0][0] = 0;
        model.dit[0][0][0] = 0;
        model.prob[0][0][0] = PROB_BASE;
        model.dix[0][0][1] = -1;
        model.prob[0][0][1] = PROB_BASE / 2;

        let pruned = prune_topk(&model, 3);
        assert_eq!(pruned.n_outcomes[0][0], 2);
        assert_eq!(pruned.prob[0][0][0], PROB_BASE);
        assert_eq!(pruned.prob[0][0][1], PROB_BASE / 2);
    }

    #[test]
    fn prune_topk_keeps_highest_prob() {
        // Three outcomes with probs PB, PB/2, PB*3/4.
        // Top-1 should be the PB outcome (index 0).
        use vi_core::TransitionModel;
        let mut model = TransitionModel::default();
        model.n_outcomes[0][0] = 3;
        model.dix[0][0][0] = 1;
        model.prob[0][0][0] = PROB_BASE;        // highest
        model.dix[0][0][1] = 2;
        model.prob[0][0][1] = PROB_BASE / 2;
        model.dix[0][0][2] = 3;
        model.prob[0][0][2] = (PROB_BASE / 4) * 3;

        let pruned = prune_topk(&model, 1);
        assert_eq!(pruned.n_outcomes[0][0], 1);
        assert_eq!(pruned.dix[0][0][0], 1); // index 0 was highest
        assert_eq!(pruned.prob[0][0][0], PROB_BASE);
    }

    #[test]
    fn topk_stats_fields_sane() {
        let mut ctx = empty_5x5_ctx();
        let stats = Frontier3DTopK { k: 5 }.run(&mut ctx, Budget::Iterations(200));
        assert_eq!(stats.final_delta, 0);
        assert!(stats.extra.is_none());
    }

    #[test]
    fn goal_cell_pinned_after_topk_run() {
        let mut ctx = empty_5x5_ctx();
        Frontier3DTopK { k: 2 }.run(&mut ctx, Budget::Iterations(200));
        // Goal at (iy=2, ix=2, it=0).
        assert_eq!(ctx.value[[2, 2, 0]], 0, "goal cell must be pinned to 0");
    }

    #[test]
    fn topk_unused_check() {
        // Quick smoke: with the 5x5 fixture, layer 0 values should be finite
        // and less than MAX_VALUE for reachable cells.
        let mut ctx = empty_5x5_ctx();
        Frontier3DTopK { k: 3 }.run(&mut ctx, Budget::Iterations(200));
        // Cell (0,0,0) is reachable from goal (2,2,0) with actions +x/-x+y/-y.
        assert!(
            ctx.value[[0, 0, 0]] < MAX_VALUE,
            "corner cell (0,0,0) should be reachable"
        );
    }
}
