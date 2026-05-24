//! Approximate 3D frontier VI with residual thresholding.
//!
//! Only value drops larger than `tau` are applied and re-enqueued. When
//! `tau == 0` the solver is identical to [`Frontier3D`]. Mirrors
//! `vi_matlab/src/cpu/frontier/vi_frontier_3d_tau.m`.
//! See spec §4.2, §4.8.

use vi_core::{Value, N_THETA};

use crate::bitboard::Bitboard3D;
use crate::context::{Budget, SolveStats, Solver, VIContext};
use crate::kernel::bellman_backup;

use super::{
    Frontier3D, build_passable_bb_2d, build_passable_bb_3d, build_value_seed_3d, max_iters,
    pin_goals,
};

/// Frontier-VI with residual-threshold gating.
///
/// Only updates where `old_val - new_val > tau` are applied and propagated.
/// Convergence is declared when the frontier is empty (no cell in the latest
/// iteration produced a qualifying update).
///
/// * `tau = 0` → delegates to [`Frontier3D`] (bit-exact equivalent).
/// * `tau > 0` → approximate: some updates are suppressed, so the final
///   value table may differ from the exact solution.
pub struct Frontier3DTau {
    /// Residual threshold for re-enqueuing. Only updates with
    /// `old_val - new_val > tau` mark the new frontier. `tau == 0` is
    /// equivalent to (and delegates to) [`Frontier3D`].
    pub tau: Value,
}

impl Solver for Frontier3DTau {
    fn name(&self) -> &'static str {
        "frontier_3d_tau"
    }

    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        // MATLAB: if tau <= 0, delegate to vi_frontier_3d.
        if self.tau == 0 {
            return Frontier3D.run(ctx, budget);
        }

        let max_iter = max_iters(budget);
        let map_x = ctx.dims.map_x;
        let map_y = ctx.dims.map_y;

        let (mx, my, mt) = ctx.transitions.max_displacement();
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
                let new_val = bellman_backup(
                    &ctx.value,
                    &ctx.penalty,
                    &ctx.transitions,
                    ix,
                    iy,
                    it,
                    map_x,
                    map_y,
                );
                // MATLAB: if old_val - v_new > tau
                // Use saturating_sub to guard against underflow when new_val > old_val.
                if old_val.saturating_sub(new_val) > self.tau {
                    ctx.value[[iy_us, ix_us, it_us]] = new_val;
                    updates += 1;
                    new_frontier.set(ix, iy, it);
                }
            }

            frontier = new_frontier;
        }

        // MATLAB re-pins goals at the very end (defensive against tau suppressing a
        // goal update — harmless when tau=0 but the MATLAB code always does it).
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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::Budget;
    use crate::reference::Reference;
    use vi_core::MAX_VALUE;
    use super::super::test_helpers::empty_5x5_ctx;

    #[test]
    fn tau_zero_matches_frontier_3d() {
        // tau=0 must produce bit-exact result with Frontier3D.
        let mut ctx_tau = empty_5x5_ctx();
        let mut ctx_f3d = ctx_tau.clone_value();

        Frontier3DTau { tau: 0 }.run(&mut ctx_tau, Budget::Iterations(200));
        Frontier3D.run(&mut ctx_f3d, Budget::Iterations(200));

        assert_eq!(
            ctx_tau.value, ctx_f3d.value,
            "tau=0 must be bit-exact with Frontier3D"
        );
    }

    #[test]
    fn tau_nonzero_terminates() {
        // tau=5 must converge (frontier empties) within budget.
        let mut ctx = empty_5x5_ctx();
        let stats = Frontier3DTau { tau: 5 }.run(&mut ctx, Budget::Iterations(200));
        assert!(stats.converged, "tau=5 should converge within 200 iters");
    }

    #[test]
    fn tau_nonzero_close_to_reference() {
        // Mean absolute difference vs Reference on layer-0 should be small.
        // We compare only layer 0 (the one layer that matters for the
        // deterministic 4-dir transitions with dit=0).
        let mut ctx_ref = empty_5x5_ctx();
        let mut ctx_tau = empty_5x5_ctx();

        Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(100));
        Frontier3DTau { tau: 5 }.run(&mut ctx_tau, Budget::Iterations(200));

        let count = 5 * 5; // 5x5 cells, layer 0
        let mean_abs_diff: f64 = (0..5usize)
            .flat_map(|iy| (0..5usize).map(move |ix| (iy, ix)))
            .map(|(iy, ix)| {
                let r = ctx_ref.value[[iy, ix, 0]];
                let t = ctx_tau.value[[iy, ix, 0]];
                // Skip cells that are MAX_VALUE in both (unreachable).
                if r == MAX_VALUE && t == MAX_VALUE {
                    return 0.0;
                }
                (r as i32 - t as i32).abs() as f64
            })
            .sum::<f64>()
            / count as f64;

        // WHY: tau=5 suppresses small updates; some cells settle at values 5-9 below
        // the strict-convergence floor. Bound of 10.0 catches gross regressions while
        // allowing the expected tau-induced drift.
        assert!(mean_abs_diff < 10.0, "mean abs diff vs reference = {}", mean_abs_diff);
    }

    #[test]
    fn goal_cell_pinned_after_tau_run() {
        let mut ctx = empty_5x5_ctx();
        Frontier3DTau { tau: 10 }.run(&mut ctx, Budget::Iterations(200));
        // Goal at (iy=2, ix=2, it=0).
        assert_eq!(ctx.value[[2, 2, 0]], 0, "goal cell must be pinned to 0 after tau run");
    }

    #[test]
    fn stats_fields_sane() {
        let mut ctx = empty_5x5_ctx();
        let stats = Frontier3DTau { tau: 3 }.run(&mut ctx, Budget::Iterations(200));
        assert_eq!(stats.final_delta, 0);
        assert!(stats.extra.is_none());
    }
}
