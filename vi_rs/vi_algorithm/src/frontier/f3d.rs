//! Frontier-VI with a 3D (x, y, theta) bitboard.
//!
//! Mirrors `vi_matlab/src/cpu/frontier/vi_frontier_3d.m`.
//! Bit-exact with Reference: converged value table matches byte-for-byte.
//! See spec §4.2, §4.8.

use vi_core::N_THETA;

use crate::bitboard::Bitboard3D;
use crate::context::{Budget, SolveStats, Solver, VIContext};
use crate::kernel::bellman_backup;

use super::{build_passable_bb_3d, build_passable_bb_2d, build_value_seed_3d, max_iters, pin_goals};

/// Frontier-VI solver using a single 3D bitboard (x, y, theta).
/// Higher per-iter cost than [`Frontier2D`] but tracks theta independently per cell.
pub struct Frontier3D;

impl Solver for Frontier3D {
    fn name(&self) -> &'static str {
        "frontier_3d"
    }

    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        let max_iter = max_iters(budget);
        let map_x = ctx.dims.map_x;
        let map_y = ctx.dims.map_y;

        let (mx, my, mt) = ctx.transitions.max_displacement();
        let mx = mx as u32;
        let my = my as u32;
        let mt = mt as u32;

        // Pin goal cells to 0 BEFORE building the frontier seed so that
        // goal cells (value drops to 0 < MAX_VALUE) are included.
        pin_goals(&mut ctx.value, &ctx.goal_mask);

        // Build passable bitboard (3D): each theta layer = passable_2d.
        let passable_2d = build_passable_bb_2d(&ctx.penalty);
        let passable_bb = build_passable_bb_3d(&passable_2d, N_THETA as u32);

        // Build goal bitboard and its complement.
        let goal_bb = Bitboard3D::from_logical(ctx.goal_mask.view());
        let not_goal_bb = goal_bb.complement();

        // Initial frontier: cells with value < MAX_VALUE.
        let mut frontier = build_value_seed_3d(&ctx.value);

        let mut updates: u64 = 0;
        let mut iters: u32 = 0;

        while frontier.popcount() > 0 && iters < max_iter {
            iters += 1;

            // Expand frontier → candidate set.
            let mut candidates = frontier.dilate(mx, my, mt);
            candidates.and_inplace(&passable_bb);
            candidates.and_inplace(&not_goal_bb);

            let mut new_frontier = Bitboard3D::new(map_x, map_y, N_THETA as u32);

            for (ix, iy, it) in candidates.enumerate() {
                let ix_us = ix as usize;
                let iy_us = iy as usize;
                let old = ctx.value[[iy_us, ix_us, it as usize]];
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
                if new_val < old {
                    ctx.value[[iy_us, ix_us, it as usize]] = new_val;
                    updates += 1;
                    new_frontier.set(ix, iy, it);
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
    fn parity_empty_3x3() {
        let mut ctx_ref = empty_3x3_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        assert!(ref_stats.converged, "Reference must converge for parity test to be valid");

        let stats = Frontier3D.run(&mut ctx_frontier, Budget::Iterations(200));
        assert!(stats.converged, "Frontier3D must converge: iters={}", stats.iters_or_sweeps);

        assert_eq!(
            ctx_ref.value, ctx_frontier.value,
            "bit-exact parity required: Frontier3D value table must match Reference"
        );
    }

    #[test]
    fn parity_obstacle_3x3() {
        let mut ctx_ref = obstacle_3x3_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        assert!(ref_stats.converged, "Reference must converge");

        let stats = Frontier3D.run(&mut ctx_frontier, Budget::Iterations(200));
        assert!(stats.converged, "Frontier3D must converge");

        assert_eq!(ctx_ref.value, ctx_frontier.value, "bit-exact parity required (obstacle map)");
    }

    #[test]
    fn parity_sentinel_3x3() {
        let mut ctx_ref = sentinel_3x3_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        assert!(ref_stats.converged, "Reference must converge");

        let stats = Frontier3D.run(&mut ctx_frontier, Budget::Iterations(200));
        assert!(stats.converged, "Frontier3D must converge");

        assert_eq!(ctx_ref.value, ctx_frontier.value, "bit-exact parity required (sentinel map)");
    }

    #[test]
    fn parity_larger_5x5() {
        let mut ctx_ref = empty_5x5_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(100));
        assert!(ref_stats.converged, "Reference must converge for parity test to be valid");

        let stats = Frontier3D.run(&mut ctx_frontier, Budget::Iterations(300));
        assert!(stats.converged, "Frontier3D must converge: iters={}", stats.iters_or_sweeps);

        assert_eq!(ctx_ref.value, ctx_frontier.value, "bit-exact parity required (5x5 map)");
    }

    // -----------------------------------------------------------------------
    // Convergence and budget tests
    // -----------------------------------------------------------------------

    #[test]
    fn convergence_within_budget() {
        let mut ctx = empty_3x3_ctx();
        let stats = Frontier3D.run(&mut ctx, Budget::Iterations(100));
        assert!(stats.converged);
        assert!(stats.iters_or_sweeps < 100);
        assert!(stats.updates > 0);
    }

    #[test]
    fn budget_exhaustion() {
        // 5x5 with tiny budget — should NOT converge.
        let mut ctx = empty_5x5_ctx();
        let stats = Frontier3D.run(&mut ctx, Budget::Iterations(1));
        assert!(!stats.converged);
        assert_eq!(stats.iters_or_sweeps, 1, "budget of 1 must produce exactly 1 iteration");
    }

    #[test]
    fn goal_cell_pinned_to_zero() {
        let mut ctx = empty_3x3_ctx();
        Frontier3D.run(&mut ctx, Budget::Iterations(100));
        assert_eq!(ctx.value[[1, 1, 0]], 0, "goal cell must be pinned to 0");
    }

    #[test]
    fn stats_fields_sane() {
        let mut ctx = empty_3x3_ctx();
        let stats = Frontier3D.run(&mut ctx, Budget::Iterations(100));
        assert_eq!(stats.final_delta, 0, "Frontier3D always returns final_delta=0");
        assert!(stats.extra.is_none());
    }
}
