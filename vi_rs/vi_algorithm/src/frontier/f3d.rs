//! Frontier-VI with a 3D (x, y, theta) bitboard.
//!
//! Mirrors `vi_matlab/src/cpu/frontier/vi_frontier_3d.m`.
//! Bit-exact with Reference: converged value table matches byte-for-byte
//! (serial path only; see [`Frontier3D::run_parallel`] for the Jacobi variant).
//! See spec §4.2, §4.7, §4.8.

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

        #[cfg(not(feature = "parallel"))]
        let (iters, updates, converged) = self.run_serial_inner(ctx, max_iter);

        #[cfg(feature = "parallel")]
        let (iters, updates, converged) = self.run_parallel(ctx, max_iter);

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

impl Frontier3D {
    /// Public serial entry-point. Mirrors [`Solver::run`] but is guaranteed to
    /// run the Gauss-Seidel serial frontier iteration regardless of whether
    /// the crate was compiled with `--features parallel`. Used by
    /// `bench_summary --parallel` to compare serial vs parallel timings in the
    /// same process.
    pub fn run_serial(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        let max_iter = max_iters(budget);
        let (iters, updates, converged) = self.run_serial_inner(ctx, max_iter);
        SolveStats {
            iters_or_sweeps: iters,
            updates,
            final_delta: 0,
            converged,
            extra: None,
        }
    }

    /// Serial Gauss-Seidel frontier iteration. Bit-exact with MATLAB
    /// `vi_frontier_3d.m`: per iteration, the candidate set is enumerated and
    /// each Bellman backup is written in-place into `ctx.value`, so backups
    /// later in the same iteration may read newly-updated neighbours.
    ///
    /// Returns `(iters_run, updates, converged)`. Internal helper — the public
    /// [`Self::run_serial`] wraps this with the `SolveStats` packaging.
    pub(crate) fn run_serial_inner(
        &self,
        ctx: &mut VIContext,
        max_iter: u32,
    ) -> (u32, u64, bool) {
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
        (iters, updates, converged)
    }

    /// Jacobi-style frontier iteration parallelised with rayon. Spec §4.7:
    /// the per-cell Bellman backups in one iteration all read from the
    /// pre-iteration snapshot (`ctx.value` is treated as immutable during the
    /// parallel pass), then writes and the new frontier are reduced
    /// sequentially. The fixed point is identical to the serial path; only
    /// `iters_or_sweeps` and `updates` may differ. Bit-exact MATLAB parity is
    /// preserved by the serial path only.
    ///
    /// Returns `(iters_run, updates, converged)`.
    #[cfg(feature = "parallel")]
    pub(crate) fn run_parallel(
        &self,
        ctx: &mut VIContext,
        max_iter: u32,
    ) -> (u32, u64, bool) {
        use rayon::prelude::*;

        let map_x = ctx.dims.map_x;
        let map_y = ctx.dims.map_y;

        let (mx, my, mt) = ctx.transitions.max_displacement();
        let mx = mx as u32;
        let my = my as u32;
        let mt = mt as u32;

        // Pin goal cells to 0 BEFORE building the frontier seed so that
        // goal cells (value drops to 0 < MAX_VALUE) are included, and so the
        // Jacobi snapshot already contains 0 at goal cells on the first
        // iteration (matching the serial path's pre-loop pin).
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

            // Expand frontier → candidate set.
            let mut candidates = frontier.dilate(mx, my, mt);
            candidates.and_inplace(&passable_bb);
            candidates.and_inplace(&not_goal_bb);

            // Collect candidate cells; the Bellman backups below all read
            // from `ctx.value` as an immutable snapshot (Jacobi within an
            // iteration), so we can fan them out across rayon workers and
            // apply writes sequentially after the parallel pass.
            let mut cells: Vec<(u32, u32, u32)> = Vec::with_capacity(candidates.popcount() as usize);
            cells.extend(candidates.enumerate());

            let prev = &ctx.value;
            let penalty = &ctx.penalty;
            let transitions = &ctx.transitions;

            // Per-cell result: (ix, iy, it, new_val), kept only when the new
            // value strictly improves the existing one.
            let results: Vec<(u32, u32, u32, vi_core::Value)> = cells
                .par_iter()
                .filter_map(|&(ix, iy, it)| {
                    let old = prev[[iy as usize, ix as usize, it as usize]];
                    let new_val = bellman_backup(
                        prev, penalty, transitions,
                        ix, iy, it, map_x, map_y,
                    );
                    if new_val < old {
                        Some((ix, iy, it, new_val))
                    } else {
                        None
                    }
                })
                .collect();

            // Sequential apply: write back to ctx.value, build new frontier,
            // and fold the updates count.
            let mut new_frontier = Bitboard3D::new(map_x, map_y, N_THETA as u32);
            for &(ix, iy, it, new_val) in &results {
                ctx.value[[iy as usize, ix as usize, it as usize]] = new_val;
                new_frontier.set(ix, iy, it);
            }
            updates += results.len() as u64;
            frontier = new_frontier;
        }

        let converged = frontier.popcount() == 0;
        (iters, updates, converged)
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

#[cfg(all(test, feature = "parallel"))]
mod parallel_tests {
    use super::*;
    use crate::context::Budget;
    use crate::reference::Reference;
    use super::super::test_helpers::{
        empty_3x3_ctx, empty_5x5_ctx, obstacle_3x3_ctx, sentinel_3x3_ctx,
    };

    /// Parallel (Jacobi) and serial (Gauss-Seidel) Frontier3D iterations
    /// converge to the same fixed point as Reference, even though they may
    /// take a different number of iterations to get there. Spec §4.7.
    #[test]
    fn parallel_frontier3d_parity_empty_5x5() {
        let mut ctx_ref = empty_5x5_ctx();
        let mut ctx_par = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(100));
        assert!(ref_stats.converged, "Reference must converge for parity test to be valid");

        let stats = Frontier3D.run(&mut ctx_par, Budget::Iterations(300));
        assert!(stats.converged, "parallel Frontier3D must converge");

        assert_eq!(
            ctx_ref.value, ctx_par.value,
            "parallel Frontier3D must converge to the same value table as Reference (5x5)"
        );
    }

    #[test]
    fn parallel_frontier3d_parity_obstacle_3x3() {
        let mut ctx_ref = obstacle_3x3_ctx();
        let mut ctx_par = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        assert!(ref_stats.converged, "Reference must converge");

        let stats = Frontier3D.run(&mut ctx_par, Budget::Iterations(200));
        assert!(stats.converged, "parallel Frontier3D must converge");

        assert_eq!(
            ctx_ref.value, ctx_par.value,
            "parallel Frontier3D must match Reference on obstacle_3x3"
        );
    }

    #[test]
    fn parallel_frontier3d_parity_sentinel_3x3() {
        let mut ctx_ref = sentinel_3x3_ctx();
        let mut ctx_par = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        assert!(ref_stats.converged, "Reference must converge");

        let stats = Frontier3D.run(&mut ctx_par, Budget::Iterations(200));
        assert!(stats.converged, "parallel Frontier3D must converge");

        assert_eq!(
            ctx_ref.value, ctx_par.value,
            "parallel Frontier3D must match Reference on sentinel_3x3"
        );
    }

    /// Serial and parallel Frontier3D from identical contexts must produce
    /// identical converged value tables (the fixed-point cross-check).
    #[test]
    fn parallel_matches_serial_frontier3d_empty_5x5() {
        let mut ctx_serial = empty_5x5_ctx();
        let mut ctx_par = ctx_serial.clone_value();

        Frontier3D.run_serial_inner(&mut ctx_serial, 300);
        Frontier3D.run_parallel(&mut ctx_par, 300);

        assert_eq!(
            ctx_serial.value, ctx_par.value,
            "serial and parallel Frontier3D must converge to the same value table"
        );
    }

    #[test]
    fn parallel_frontier3d_pins_goal_3x3() {
        let mut ctx = empty_3x3_ctx();
        Frontier3D.run(&mut ctx, Budget::Iterations(100));
        assert_eq!(ctx.value[[1, 1, 0]], 0, "goal cell must be pinned to 0 in parallel path");
    }
}
