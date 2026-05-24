//! Frontier-VI with 60 stacked 2D bitboards (one per theta layer).
//!
//! Mirrors `vi_matlab/src/cpu/frontier/vi_frontier_stack.m`.
//! Bit-exact with Reference: converged value table matches byte-for-byte.

use vi_core::{MAX_VALUE, N_THETA};

use crate::bitboard::Bitboard2D;
use crate::context::{Budget, SolveStats, Solver, VIContext};
use crate::kernel::bellman_backup;

use super::{build_passable_bb_2d, max_iters, pin_goals};

pub struct FrontierStack;

impl Solver for FrontierStack {
    fn name(&self) -> &'static str {
        "frontier_stack"
    }

    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        let max_iter = max_iters(budget);
        let map_x = ctx.dims.map_x;
        let map_y = ctx.dims.map_y;

        let (mx, my, mt) = ctx.transitions.max_displacement();
        let mx = mx as u32;
        let my = my as u32;
        let mt = mt as usize; // used in theta-wrap arithmetic

        // Pin goal cells to 0 BEFORE building the frontier seeds.
        pin_goals(&mut ctx.value, &ctx.goal_mask);

        // Single shared 2D passable bitboard.
        let passable_bb = build_passable_bb_2d(&ctx.penalty);

        // Per-layer goal bitboards and initial frontiers.
        let goal_layers: Vec<Bitboard2D> = (0..N_THETA)
            .map(|it| {
                Bitboard2D::from_logical(
                    ctx.goal_mask.slice(ndarray::s![.., .., it]),
                )
            })
            .collect();

        let mut frontier: Vec<Bitboard2D> = (0..N_THETA)
            .map(|it| {
                // Seed: spatial cells where value[.., .., it] < MAX_VALUE.
                let mut bb = Bitboard2D::new(map_x, map_y);
                for iy in 0..map_y as usize {
                    for ix in 0..map_x as usize {
                        if ctx.value[[iy, ix, it]] < MAX_VALUE {
                            bb.set(ix as u32, iy as u32);
                        }
                    }
                }
                bb
            })
            .collect();

        let mut updates: u64 = 0;
        let mut iters: u32 = 0;

        // Precompute per-layer goal complements once (reused every iteration).
        // WHY: complement() allocates; caching avoids N_THETA allocs per iter.
        let goal_complements: Vec<Bitboard2D> =
            goal_layers.iter().map(|g| g.complement()).collect();

        // Stack popcount: total set bits across all theta layers.
        let stack_popcount = |layers: &Vec<Bitboard2D>| -> u64 {
            layers.iter().map(|bb| bb.popcount()).sum()
        };

        while stack_popcount(&frontier) > 0 && iters < max_iter {
            iters += 1;

            // Step 1: per-layer 2D dilation (reads from `frontier`, writes to `dilated_self`).
            let dilated_self: Vec<Bitboard2D> =
                frontier.iter().map(|bb| bb.dilate(mx, my)).collect();

            // Step 2: candidates per layer = XY-dilated | neighboring theta layers.
            let candidates: Vec<Bitboard2D> = (0..N_THETA)
                .map(|it| {
                    let mut cand = dilated_self[it].clone();
                    // OR in theta-neighboring layers (mt steps in each direction).
                    for st in 1..=mt {
                        let it_minus = (it + N_THETA - st) % N_THETA;
                        let it_plus = (it + st) % N_THETA;
                        cand.or_inplace(&dilated_self[it_minus]);
                        cand.or_inplace(&dilated_self[it_plus]);
                    }
                    // Mask: must be passable and not a goal cell.
                    cand.and_inplace(&passable_bb);
                    cand.and_inplace(&goal_complements[it]);
                    cand
                })
                .collect();

            // Step 3: Bellman backup for each candidate cell in each layer.
            let mut new_frontier: Vec<Bitboard2D> =
                (0..N_THETA).map(|_| Bitboard2D::new(map_x, map_y)).collect();

            for it in 0..N_THETA {
                for (ix, iy) in candidates[it].enumerate() {
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
                        new_frontier[it].set(ix, iy);
                    }
                }
            }

            frontier = new_frontier;
        }

        let converged = stack_popcount(&frontier) == 0;

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
    fn parity_stack_empty_3x3() {
        let mut ctx_ref = empty_3x3_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        assert!(ref_stats.converged, "Reference must converge for parity test to be valid");

        let stats = FrontierStack.run(&mut ctx_frontier, Budget::Iterations(200));
        assert!(stats.converged, "FrontierStack must converge: iters={}", stats.iters_or_sweeps);

        assert_eq!(
            ctx_ref.value, ctx_frontier.value,
            "bit-exact parity required: FrontierStack value table must match Reference"
        );
    }

    #[test]
    fn parity_stack_obstacle_3x3() {
        let mut ctx_ref = obstacle_3x3_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        assert!(ref_stats.converged, "Reference must converge");

        let stats = FrontierStack.run(&mut ctx_frontier, Budget::Iterations(200));
        assert!(stats.converged, "FrontierStack must converge");

        assert_eq!(ctx_ref.value, ctx_frontier.value, "bit-exact parity required (obstacle map)");
    }

    #[test]
    fn parity_stack_sentinel_3x3() {
        let mut ctx_ref = sentinel_3x3_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        assert!(ref_stats.converged, "Reference must converge");

        let stats = FrontierStack.run(&mut ctx_frontier, Budget::Iterations(200));
        assert!(stats.converged, "FrontierStack must converge");

        assert_eq!(ctx_ref.value, ctx_frontier.value, "bit-exact parity required (sentinel map)");
    }

    #[test]
    fn parity_stack_larger_5x5() {
        let mut ctx_ref = empty_5x5_ctx();
        let mut ctx_frontier = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(100));
        assert!(ref_stats.converged, "Reference must converge for parity test to be valid");

        let stats = FrontierStack.run(&mut ctx_frontier, Budget::Iterations(300));
        assert!(stats.converged, "FrontierStack must converge: iters={}", stats.iters_or_sweeps);

        assert_eq!(ctx_ref.value, ctx_frontier.value, "bit-exact parity required (5x5 map)");
    }

    // -----------------------------------------------------------------------
    // Convergence and budget tests
    // -----------------------------------------------------------------------

    #[test]
    fn convergence_within_budget_stack() {
        let mut ctx = empty_3x3_ctx();
        let stats = FrontierStack.run(&mut ctx, Budget::Iterations(100));
        assert!(stats.converged);
        assert!(stats.iters_or_sweeps < 100);
        assert!(stats.updates > 0);
    }

    #[test]
    fn budget_exhaustion_stack() {
        // 5x5 with tiny budget — should NOT converge.
        let mut ctx = empty_5x5_ctx();
        let stats = FrontierStack.run(&mut ctx, Budget::Iterations(1));
        assert!(!stats.converged);
    }

    #[test]
    fn goal_cell_pinned_to_zero_stack() {
        let mut ctx = empty_3x3_ctx();
        FrontierStack.run(&mut ctx, Budget::Iterations(100));
        assert_eq!(ctx.value[[1, 1, 0]], 0, "goal cell must be pinned to 0");
    }

    #[test]
    fn stats_fields_sane_stack() {
        let mut ctx = empty_3x3_ctx();
        let stats = FrontierStack.run(&mut ctx, Budget::Iterations(100));
        assert_eq!(stats.final_delta, 0, "FrontierStack always returns final_delta=0");
        assert!(stats.extra.is_none());
    }
}
