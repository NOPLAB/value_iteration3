//! Frontier-VI with a 2D (spatial) bitboard.
//!
//! Mirrors `vi_matlab/src/cpu/frontier/vi_frontier_2d.m`.
//! Bit-exact with Reference: converged value table matches byte-for-byte.

use vi_core::{N_THETA};

use crate::bitboard::Bitboard2D;
use crate::context::{Budget, SolveStats, Solver, VIContext};
use crate::kernel::bellman_backup;

use super::{build_passable_bb_2d, build_value_seed_2d, max_iters, pin_goals};

pub struct Frontier2D;

impl Solver for Frontier2D {
    fn name(&self) -> &'static str {
        "frontier_2d"
    }

    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        let max_iter = max_iters(budget);
        let map_x = ctx.dims.map_x;
        let map_y = ctx.dims.map_y;

        let (mx, my, _mt) = ctx.transitions.max_displacement();
        let mx = mx as u32;
        let my = my as u32;

        // Pin goal cells to 0 BEFORE building the frontier seed.
        pin_goals(&mut ctx.value, &ctx.goal_mask);

        // Build 2D passable bitboard.
        let passable_bb = build_passable_bb_2d(&ctx.penalty);

        // Initial frontier: spatial cells where any theta has value < MAX_VALUE.
        // Mirrors MATLAB: `any(value_table < MV, 3)`.
        let mut frontier = build_value_seed_2d(&ctx.value);

        let mut updates: u64 = 0;
        let mut iters: u32 = 0;

        while frontier.popcount() > 0 && iters < max_iter {
            iters += 1;

            // Expand frontier → candidate spatial cells.
            let mut candidates = frontier.dilate(mx, my);
            candidates.and_inplace(&passable_bb);

            let mut new_frontier = Bitboard2D::new(map_x, map_y);

            for (ix, iy) in candidates.enumerate() {
                let mut changed = false;
                for it in 0..N_THETA {
                    if ctx.goal_mask[[iy as usize, ix as usize, it]] {
                        continue;
                    }
                    let old = ctx.value[[iy as usize, ix as usize, it]];
                    let new_val = bellman_backup(
                        &ctx.value,
                        &ctx.penalty,
                        &ctx.transitions,
                        ix,
                        iy,
                        it as u32,
                        map_x,
                        map_y,
                    );
                    if new_val < old {
                        ctx.value[[iy as usize, ix as usize, it]] = new_val;
                        updates += 1;
                        changed = true;
                    }
                }
                if changed {
                    new_frontier.set(ix, iy);
                }
            }

            frontier = new_frontier;
        }

        let converged = frontier.popcount() == 0;

        SolveStats {
            iters_or_sweeps: iters,
            updates,
            // WHY: Frontier solvers do not compute a residual per iteration;
            // convergence is signalled by "frontier empty". final_delta=0 per spec §4.8.
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
    use super::super::test_helpers::{
        empty_3x3_ctx, empty_5x5_ctx, obstacle_3x3_ctx, sentinel_3x3_ctx,
    };

    // -----------------------------------------------------------------------
    // Parity tests vs Reference (bit-exact convergence)
    // -----------------------------------------------------------------------

    #[test]
    fn parity_f2d_empty_3x3() {
        let mut ctx_ref = empty_3x3_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        assert!(ref_stats.converged, "Reference must converge for parity test to be valid");

        let stats = Frontier2D.run(&mut ctx_frontier, Budget::Iterations(200));
        assert!(stats.converged, "Frontier2D must converge: iters={}", stats.iters_or_sweeps);

        assert_eq!(
            ctx_ref.value, ctx_frontier.value,
            "bit-exact parity required: Frontier2D value table must match Reference"
        );
    }

    #[test]
    fn parity_f2d_obstacle_3x3() {
        let mut ctx_ref = obstacle_3x3_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        assert!(ref_stats.converged, "Reference must converge");

        let stats = Frontier2D.run(&mut ctx_frontier, Budget::Iterations(200));
        assert!(stats.converged, "Frontier2D must converge");

        assert_eq!(ctx_ref.value, ctx_frontier.value, "bit-exact parity required (obstacle map)");
    }

    #[test]
    fn parity_f2d_sentinel_3x3() {
        let mut ctx_ref = sentinel_3x3_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        assert!(ref_stats.converged, "Reference must converge");

        let stats = Frontier2D.run(&mut ctx_frontier, Budget::Iterations(200));
        assert!(stats.converged, "Frontier2D must converge");

        assert_eq!(ctx_ref.value, ctx_frontier.value, "bit-exact parity required (sentinel map)");
    }

    #[test]
    fn parity_f2d_larger_5x5() {
        let mut ctx_ref = empty_5x5_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(100));
        assert!(ref_stats.converged, "Reference must converge for parity test to be valid");

        let stats = Frontier2D.run(&mut ctx_frontier, Budget::Iterations(300));
        assert!(stats.converged, "Frontier2D must converge: iters={}", stats.iters_or_sweeps);

        assert_eq!(ctx_ref.value, ctx_frontier.value, "bit-exact parity required (5x5 map)");
    }

    // -----------------------------------------------------------------------
    // Convergence and budget tests
    // -----------------------------------------------------------------------

    #[test]
    fn convergence_within_budget_f2d() {
        let mut ctx = empty_3x3_ctx();
        let stats = Frontier2D.run(&mut ctx, Budget::Iterations(100));
        assert!(stats.converged);
        assert!(stats.iters_or_sweeps < 100);
        assert!(stats.updates > 0);
    }

    #[test]
    fn budget_exhaustion_f2d() {
        // 5x5 with tiny budget — should NOT converge.
        let mut ctx = empty_5x5_ctx();
        let stats = Frontier2D.run(&mut ctx, Budget::Iterations(1));
        assert!(!stats.converged);
    }

    #[test]
    fn goal_cell_pinned_to_zero_f2d() {
        let mut ctx = empty_3x3_ctx();
        Frontier2D.run(&mut ctx, Budget::Iterations(100));
        assert_eq!(ctx.value[[1, 1, 0]], 0, "goal cell must be pinned to 0");
    }

    #[test]
    fn stats_fields_sane_f2d() {
        let mut ctx = empty_3x3_ctx();
        let stats = Frontier2D.run(&mut ctx, Budget::Iterations(100));
        assert_eq!(stats.final_delta, 0, "Frontier2D always returns final_delta=0");
        assert!(stats.extra.is_none());
    }
}
