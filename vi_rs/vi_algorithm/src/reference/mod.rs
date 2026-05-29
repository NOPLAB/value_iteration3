//! Paper-aligned brute-force value iteration reference.
//!
//! Mirrors `vi_matlab/src/cpu/reference/vi_full_reference.m`.
//! Bit-exact with the C reference in `host/src/vi_reference_c.c`.
//! See `docs/superpowers/specs/2026-05-22-vi-rs-algorithm-port-design.md` §4.2, §4.8.

use vi_core::{MAX_VALUE, N_THETA, PENALTY_OBSTACLE, Value};
use crate::context::{Budget, SolveExtra, SolveStats, Solver, VIContext};
use crate::kernel::bellman_backup;

pub mod action_table;

/// Brute-force value-iteration reference solver. Bit-exact with MATLAB `vi_full_reference.m`.
///
/// `threshold`: convergence residual cap. The solver terminates early
/// (sets `converged = true`) when the per-sweep maximum `|new - old|` falls at
/// or below this value. Use `threshold: 0` for strict MATLAB-matching convergence.
pub struct Reference {
    pub threshold: Value,
}

/// `Budget::Sweeps(n)` is the canonical variant for Reference. `Budget::Iterations(n)`
/// is also accepted: the number is used as max_sweeps regardless of variant.
impl Solver for Reference {
    fn name(&self) -> &'static str {
        "reference"
    }

    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        // Algorithm per vi_full_reference.m:
        //
        // for sweep in 1..=max_sweeps:
        //   max_delta = 0
        //   for iy in 0..map_y:
        //     for ix in 0..map_x:
        //       if penalty[iy, ix] == PENALTY_OBSTACLE: continue
        //       for it in 0..N_THETA:
        //         if goal_mask[iy, ix, it]:
        //           value[iy, ix, it] = 0
        //           continue
        //         old = value[iy, ix, it]
        //         new = bellman_backup(...)
        //         value[iy, ix, it] = new
        //         max_delta = max(max_delta, |new - old|)
        //   sweeps = sweep
        //   final_delta = max_delta
        //   if max_delta <= threshold: break
        //
        // After loop: value[goal_mask] = 0 (MATLAB does this at the end)
        // Then compute action_table and return as SolveExtra::ActionTable.

        let max_sweeps = match budget {
            Budget::Sweeps(n) => n,
            Budget::Iterations(n) => n,
        };

        #[cfg(not(feature = "parallel"))]
        let (sweeps, final_delta, converged) = self.run_serial_inner(ctx, max_sweeps);

        #[cfg(feature = "parallel")]
        let (sweeps, final_delta, converged) = self.run_parallel(ctx, max_sweeps);

        finalize_reference_stats(ctx, sweeps, final_delta, converged)
    }
}

/// Apply the post-sweep goal pin and action-table computation that both
/// `Solver::run` and the public `run_serial` need. Pulled out so the parallel
/// dispatch and the explicit serial entry-point share identical post-loop
/// behaviour.
fn finalize_reference_stats(
    ctx: &mut VIContext,
    sweeps: u32,
    final_delta: Value,
    converged: bool,
) -> SolveStats {
    let map_x = ctx.dims.map_x;
    let map_y = ctx.dims.map_y;
    let map_x_us = map_x as usize;
    let map_y_us = map_y as usize;

    // Pin goal cells to 0 again after the sweep loop. This matches
    // `value_table(goal_mask) = 0` at the end of vi_full_reference.m.
    // Two reasons this is not dead code:
    //   1. When max_sweeps == 0 the sweep loop never runs, so this is the only pin.
    //   2. Exact MATLAB parity: MATLAB pins after the loop regardless of convergence.
    // Note: the parallel path also pre-pins inside run_parallel for Jacobi
    // correctness, so this pass is redundant there but kept for parity (1).
    for iy in 0..map_y_us {
        for ix in 0..map_x_us {
            for it in 0..N_THETA {
                if ctx.goal_mask[[iy, ix, it]] {
                    ctx.value[[iy, ix, it]] = 0;
                }
            }
        }
    }

    // WHY: updates not tracked in vi_full_reference.m; left 0 per spec §4.8.
    let updates: u64 = 0;

    let at = action_table::compute_action_table_reference(
        &ctx.value, &ctx.penalty, &ctx.goal_mask, &ctx.transitions,
        map_x, map_y,
    );

    SolveStats {
        iters_or_sweeps: sweeps,
        updates,
        final_delta,
        converged,
        extra: Some(SolveExtra::ActionTable(at)),
    }
}

impl Reference {
    /// Public serial entry-point. Mirrors [`Solver::run`] but is guaranteed to
    /// run the Gauss-Seidel serial sweep regardless of whether the crate was
    /// compiled with `--features parallel`. Used by `bench_summary --parallel`
    /// to compare serial vs parallel timings in the same process.
    ///
    /// `Budget::Sweeps(n)` and `Budget::Iterations(n)` are treated identically
    /// — `n` becomes `max_sweeps`.
    pub fn run_serial(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        let max_sweeps = match budget {
            Budget::Sweeps(n) => n,
            Budget::Iterations(n) => n,
        };
        let (sweeps, final_delta, converged) = self.run_serial_inner(ctx, max_sweeps);
        finalize_reference_stats(ctx, sweeps, final_delta, converged)
    }

    /// Serial Gauss-Seidel in-place sweep. Bit-exact with MATLAB
    /// `vi_full_reference.m`. The iteration order `iy → ix → it` is
    /// load-bearing — do not reorder or substitute `ndarray::Zip::indexed`.
    ///
    /// Returns `(sweeps_run, final_delta, converged)`. Internal helper —
    /// the public [`Self::run_serial`] wraps this with the post-loop goal pin
    /// and action-table computation.
    pub(crate) fn run_serial_inner(
        &self,
        ctx: &mut VIContext,
        max_sweeps: u32,
    ) -> (u32, Value, bool) {
        let map_x = ctx.dims.map_x;
        let map_y = ctx.dims.map_y;
        let map_x_us = map_x as usize;
        let map_y_us = map_y as usize;

        let mut sweeps = 0u32;
        let mut final_delta: Value = MAX_VALUE;
        let mut converged = false;

        for sweep in 1..=max_sweeps {
            let mut max_delta: Value = 0;
            for iy in 0..map_y_us {
                for ix in 0..map_x_us {
                    if ctx.penalty[[iy, ix]] == PENALTY_OBSTACLE {
                        continue;
                    }
                    for it in 0..N_THETA {
                        if ctx.goal_mask[[iy, ix, it]] {
                            ctx.value[[iy, ix, it]] = 0;
                            continue;
                        }
                        let old = ctx.value[[iy, ix, it]];
                        let new = bellman_backup(
                            &ctx.value, &ctx.penalty, &ctx.transitions,
                            ix as u32, iy as u32, it as u32, map_x, map_y,
                        );
                        ctx.value[[iy, ix, it]] = new;
                        let d = new.abs_diff(old);
                        if d > max_delta {
                            max_delta = d;
                        }
                    }
                }
            }
            sweeps = sweep;
            final_delta = max_delta;
            if max_delta <= self.threshold {
                converged = true;
                break;
            }
        }

        (sweeps, final_delta, converged)
    }

    /// Jacobi-style double-buffered sweep, parallelised across rows with rayon.
    ///
    /// Spec §4.7: under the `parallel` feature the reference solver switches
    /// from Gauss-Seidel (in-place) to Jacobi (double-buffered) updates so the
    /// per-row work can run independently. The fixed point is identical to the
    /// serial sweep; only the sweep count and `SolveStats.iters_or_sweeps` /
    /// `updates` may differ. Bit-exact MATLAB parity is preserved by the
    /// serial path only.
    ///
    /// Returns `(sweeps_run, final_delta, converged)`.
    #[cfg(feature = "parallel")]
    pub(crate) fn run_parallel(
        &self,
        ctx: &mut VIContext,
        max_sweeps: u32,
    ) -> (u32, Value, bool) {
        use ndarray::Array3;
        use rayon::prelude::*;

        let map_x = ctx.dims.map_x;
        let map_y = ctx.dims.map_y;
        let map_x_us = map_x as usize;
        let map_y_us = map_y as usize;

        let mut sweeps = 0u32;
        let mut final_delta: Value = MAX_VALUE;
        let mut converged = false;

        // Pre-pin goal cells in the initial snapshot. This matches the serial
        // Gauss-Seidel sweep where the goal cell is forced to 0 the moment its
        // index is visited (so subsequent neighbour computations within the
        // same sweep already see 0). In the Jacobi formulation we have to pin
        // before the snapshot is read; otherwise sweep 1 would see goals at
        // their initial MAX_VALUE and the residual would falsely test as 0
        // (every non-goal cell would bellman-back to MAX_VALUE, equal to its
        // old MAX_VALUE).
        for iy in 0..map_y_us {
            for ix in 0..map_x_us {
                for it in 0..N_THETA {
                    if ctx.goal_mask[[iy, ix, it]] {
                        ctx.value[[iy, ix, it]] = 0;
                    }
                }
            }
        }

        // Workspace for the next sweep's values. Allocated once and reused.
        let mut next: Array3<Value> = Array3::<Value>::zeros(ctx.value.raw_dim());

        for sweep in 1..=max_sweeps {
            // Per-row results: (row_max_delta, Vec<Value> of length map_x_us * N_THETA).
            // We compute everything from the immutable snapshot `ctx.value`, then write
            // sequentially into `next` after the parallel pass. This avoids any
            // mutable aliasing of `next` from inside the rayon closures and keeps the
            // implementation portable across ndarray feature flags.
            let prev = &ctx.value;
            let penalty = &ctx.penalty;
            let goal_mask = &ctx.goal_mask;
            let transitions = &ctx.transitions;

            let row_results: Vec<(Value, Vec<Value>)> = (0..map_y_us)
                .into_par_iter()
                .map(|iy| {
                    let mut row_vals: Vec<Value> = Vec::with_capacity(map_x_us * N_THETA);
                    let mut row_max: Value = 0;
                    for ix in 0..map_x_us {
                        let is_obstacle = penalty[[iy, ix]] == PENALTY_OBSTACLE;
                        for it in 0..N_THETA {
                            let old = prev[[iy, ix, it]];
                            let new = if goal_mask[[iy, ix, it]] {
                                // Goal cells: hold at 0; serial path also short-circuits
                                // without contributing to max_delta.
                                0
                            } else if is_obstacle {
                                // Carry the obstacle cell's existing value through;
                                // the serial path also leaves obstacle cells untouched
                                // and does not contribute to max_delta.
                                old
                            } else {
                                let v = bellman_backup(
                                    prev, penalty, transitions,
                                    ix as u32, iy as u32, it as u32, map_x, map_y,
                                );
                                let d = v.abs_diff(old);
                                if d > row_max {
                                    row_max = d;
                                }
                                v
                            };
                            row_vals.push(new);
                        }
                    }
                    (row_max, row_vals)
                })
                .collect();

            // Reduce row maxes and write computed values into `next` sequentially.
            let mut max_delta: Value = 0;
            for (iy, (row_max, row_vals)) in row_results.into_iter().enumerate() {
                if row_max > max_delta {
                    max_delta = row_max;
                }
                let mut idx = 0;
                for ix in 0..map_x_us {
                    for it in 0..N_THETA {
                        next[[iy, ix, it]] = row_vals[idx];
                        idx += 1;
                    }
                }
            }

            sweeps = sweep;
            final_delta = max_delta;

            // Swap so the next sweep reads the freshly computed `next` as its snapshot.
            std::mem::swap(&mut ctx.value, &mut next);

            if max_delta <= self.threshold {
                converged = true;
                break;
            }
        }

        (sweeps, final_delta, converged)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use ndarray::{Array2, Array3};
    use vi_core::{Penalty, TransitionModel, MAX_VALUE, PENALTY_OBSTACLE, PROB_BASE};
    use crate::context::MapDims;

    pub fn empty_3x3_ctx() -> VIContext {
        // 3x3 map, all-free penalty, one goal at (iy=1, ix=1, theta=0)
        // value initialized to MAX_VALUE
        let map_x = 3u32;
        let map_y = 3u32;
        let value = Array3::<Value>::from_elem(
            (map_y as usize, map_x as usize, N_THETA),
            MAX_VALUE,
        );
        let penalty = Array2::<Penalty>::zeros((map_y as usize, map_x as usize));
        let mut goal_mask = Array3::<bool>::from_elem(
            (map_y as usize, map_x as usize, N_THETA),
            false,
        );
        goal_mask[[1, 1, 0]] = true;

        // 4 directional actions (deterministic), actions 4-5 have n_out=0 (no-op/undefined)
        let mut trans = TransitionModel::default();
        for it in 0..N_THETA {
            // action 0: dix=+1 (right)
            trans.n_outcomes[0][it] = 1;
            trans.dix[0][it][0] = 1;
            trans.prob[0][it][0] = PROB_BASE;
            // action 1: dix=-1 (left)
            trans.n_outcomes[1][it] = 1;
            trans.dix[1][it][0] = -1;
            trans.prob[1][it][0] = PROB_BASE;
            // action 2: diy=+1 (down)
            trans.n_outcomes[2][it] = 1;
            trans.diy[2][it][0] = 1;
            trans.prob[2][it][0] = PROB_BASE;
            // action 3: diy=-1 (up)
            trans.n_outcomes[3][it] = 1;
            trans.diy[3][it][0] = -1;
            trans.prob[3][it][0] = PROB_BASE;
            // actions 4-5: n_out=0 (no-op/undefined)
        }

        VIContext {
            dims: MapDims { map_x, map_y },
            value,
            penalty,
            goal_mask,
            transitions: trans,
        }
    }

    #[test]
    fn goal_pinned_to_zero() {
        let mut ctx = empty_3x3_ctx();
        let solver = Reference { threshold: 0 };
        solver.run(&mut ctx, Budget::Sweeps(20));
        assert_eq!(ctx.value[[1, 1, 0]], 0);
    }

    #[test]
    fn converges_on_empty_3x3() {
        let mut ctx = empty_3x3_ctx();
        let solver = Reference { threshold: 0 };
        let stats = solver.run(&mut ctx, Budget::Sweeps(20));

        assert!(stats.converged, "should converge");
        assert!(stats.iters_or_sweeps < 20, "should converge before budget");
        // Corner (iy=0, ix=0, it=0) should be reachable: 2 steps from (1,1)
        assert!(
            ctx.value[[0, 0, 0]] < MAX_VALUE,
            "corner cell should be reachable, got {}",
            ctx.value[[0, 0, 0]]
        );
        // On the 3x3 with deterministic unit-cost actions and goal at (1,1,0):
        // the goal-adjacent cell (1,0,0) should be exactly 1 step from goal (cost_of(0,0)=1).
        assert_eq!(ctx.value[[1, 0, 0]], 1, "left-of-goal cell should be exactly 1 step away");
        // Corner (0,0,0) is reachable in 2 steps via right-then-down → expected cost 2.
        assert_eq!(ctx.value[[0, 0, 0]], 2, "corner should be exactly 2 steps from goal");
    }

    #[test]
    fn obstacle_cell_skipped() {
        let mut ctx = empty_3x3_ctx();
        ctx.penalty[[0, 0]] = PENALTY_OBSTACLE;
        let solver = Reference { threshold: 0 };
        solver.run(&mut ctx, Budget::Sweeps(20));
        // Obstacle cell value is untouched; it was initialized to MAX_VALUE and skipped
        assert_eq!(ctx.value[[0, 0, 0]], MAX_VALUE);
    }

    #[test]
    fn action_table_produced() {
        let mut ctx = empty_3x3_ctx();
        let solver = Reference { threshold: 0 };
        let stats = solver.run(&mut ctx, Budget::Sweeps(20));

        let at = match stats.extra {
            Some(SolveExtra::ActionTable(ref t)) => t,
            _ => panic!("expected Some(SolveExtra::ActionTable)"),
        };
        let map_x = ctx.dims.map_x as usize;
        let map_y = ctx.dims.map_y as usize;
        assert_eq!(at.shape(), &[map_y, map_x, N_THETA]);
        // Goal and obstacle cells default to action 0
        assert_eq!(at[[1, 1, 0]], 0, "goal cell action should be 0");
    }

    #[test]
    fn idempotent_after_convergence() {
        let mut ctx = empty_3x3_ctx();
        let solver = Reference { threshold: 0 };
        solver.run(&mut ctx, Budget::Sweeps(50));

        // Save the converged value table
        let converged_value = ctx.value.clone();

        // Run again from the converged state
        solver.run(&mut ctx, Budget::Sweeps(50));

        assert_eq!(ctx.value, converged_value, "second run should not change already-converged values");
    }

    #[test]
    fn threshold_above_zero_terminates_earlier_than_strict() {
        // Run identical contexts with threshold=0 (strict) and threshold=5 (relaxed).
        // The relaxed run must terminate in <= sweeps as the strict run (typically fewer).
        let mut ctx_strict = empty_3x3_ctx();
        let mut ctx_relaxed = empty_3x3_ctx();
        let strict = Reference { threshold: 0 };
        let relaxed = Reference { threshold: 5 };
        let s_strict = strict.run(&mut ctx_strict, Budget::Sweeps(50));
        let s_relaxed = relaxed.run(&mut ctx_relaxed, Budget::Sweeps(50));
        // Both should converge in well under 50 sweeps on the trivial 3x3.
        assert!(s_strict.converged);
        assert!(s_relaxed.converged);
        assert!(s_relaxed.iters_or_sweeps <= s_strict.iters_or_sweeps,
            "relaxed (threshold=5) took {} sweeps; strict (threshold=0) took {}",
            s_relaxed.iters_or_sweeps, s_strict.iters_or_sweeps);
    }

    #[test]
    fn zero_sweeps_pins_goal() {
        // The post-loop goal pin is the only thing that runs when max_sweeps == 0.
        let mut ctx = empty_3x3_ctx();
        let solver = Reference { threshold: 0 };
        let stats = solver.run(&mut ctx, Budget::Sweeps(0));
        assert_eq!(ctx.value[[1, 1, 0]], 0, "goal must be pinned even with 0 sweeps");
        assert_eq!(stats.iters_or_sweeps, 0);
        assert!(!stats.converged);
    }
}

#[cfg(all(test, feature = "parallel"))]
mod parallel_tests {
    use super::*;
    use super::tests::empty_3x3_ctx;
    use crate::frontier::test_helpers::empty_5x5_ctx;
    use vi_core::PENALTY_OBSTACLE;

    /// Jacobi (parallel) and Gauss-Seidel (serial) sweeps converge to the same
    /// fixed point even though they may take a different number of sweeps to
    /// get there. Spec §4.7.
    #[test]
    fn parallel_converges_to_same_fixed_point_as_serial_3x3() {
        let solver = Reference { threshold: 0 };
        let mut ctx_serial = empty_3x3_ctx();
        let mut ctx_par = ctx_serial.clone_value();

        solver.run_serial_inner(&mut ctx_serial, 200);
        solver.run_parallel(&mut ctx_par, 200);

        assert_eq!(
            ctx_serial.value, ctx_par.value,
            "parallel and serial sweeps must converge to the same value table"
        );
    }

    #[test]
    fn parallel_converges_to_same_fixed_point_as_serial_5x5() {
        let solver = Reference { threshold: 0 };
        let mut ctx_serial = empty_5x5_ctx();
        let mut ctx_par = ctx_serial.clone_value();

        solver.run_serial_inner(&mut ctx_serial, 200);
        solver.run_parallel(&mut ctx_par, 200);

        assert_eq!(
            ctx_serial.value, ctx_par.value,
            "parallel and serial sweeps must converge to the same value table on 5x5"
        );
    }

    /// Parallel sweep must respect obstacle cells (their value is untouched)
    /// just like the serial sweep does.
    #[test]
    fn parallel_obstacle_cells_preserved() {
        let solver = Reference { threshold: 0 };
        let mut ctx = empty_3x3_ctx();
        ctx.penalty[[0, 0]] = PENALTY_OBSTACLE;
        let initial = ctx.value[[0, 0, 0]];

        solver.run_parallel(&mut ctx, 50);

        assert_eq!(
            ctx.value[[0, 0, 0]], initial,
            "obstacle cell must carry its initial value through the parallel sweep"
        );
    }
}
