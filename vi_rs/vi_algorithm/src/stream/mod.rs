//! FPGA-streaming-mimic value iteration.
//!
//! Mirrors `vi_matlab/src/fpga/stream/vi_sweep_stream_algo.m`.
//! Each sweep runs CU0 then CU1; converges to the same fixed point as
//! Reference (not required to be bit-exact — strip/CU scan order affects
//! sweep count, not final value table).
//! See spec §4.

use vi_core::{MAX_VALUE, N_THETA, PENALTY_OBSTACLE, Value};

use crate::context::{Budget, SolveStats, Solver, VIContext};
use crate::kernel::bellman_backup;

/// Width (in cells) of a single streaming strip. Mirrors `vi_params.STRIP_W_MAX`
/// in MATLAB / `STRIP_W_MAX` in `fpga/hls/stream/src/vi_stream_types.h`.
///
/// For maps narrower than `STRIP_W_MAX`, `num_strips == 1` and the CU0/CU1
/// distinction collapses to "ascending vs descending Y/X scan over the whole
/// grid" — which is enough to exercise the scan-order semantics on tiny
/// fixtures.
const STRIP_W_MAX: u32 = 145;

/// FPGA-streaming-mimic value-iteration solver.
///
/// Mirrors the Vitis HLS streaming kernel as driven from Linux user-space:
/// one sweep = CU0 left-to-right strip walk + CU1 right-to-left strip walk
/// over the entire grid. CU0 walks rows top-to-bottom within each strip; CU1
/// walks rows bottom-to-top. Within a row each CU also reverses column order.
///
/// The HLS DDR line-buffer is an implementation detail of the streaming
/// kernel — in software we read directly from `ctx.value` and write back
/// in place, so values propagate within a sweep in Gauss-Seidel order along
/// each CU's scan path. Goal cells are pinned to 0 after both CUs finish.
///
/// Not required to be bit-exact with Reference per spec §4.8 — the
/// (CU, strip, Y, X) scan order changes the sweep count but converges to the
/// same fixed point.
pub struct StreamMimic {
    /// Convergence residual cap. Use 0 for strict equality with Reference fixed point.
    pub threshold: Value,
}

/// `Budget::Sweeps(n)` is the canonical variant for StreamMimic. `Budget::Iterations(n)`
/// is also accepted: the number is used as max_sweeps regardless of variant.
impl Solver for StreamMimic {
    fn name(&self) -> &'static str {
        "stream_mimic"
    }

    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        let max_sweeps = match budget {
            Budget::Sweeps(n) => n,
            Budget::Iterations(n) => n,
        };

        let map_x = ctx.dims.map_x;
        let map_y = ctx.dims.map_y;
        let map_x_us = map_x as usize;
        let map_y_us = map_y as usize;

        let mut sweeps: u32 = 0;
        let mut final_delta: Value = MAX_VALUE;
        let mut converged = false;

        for sweep in 1..=max_sweeps {
            let mut max_delta: Value = 0;
            for cu_id in 0..2u32 {
                let d = run_cu(ctx, cu_id, map_x, map_y);
                if d > max_delta {
                    max_delta = d;
                }
            }

            // Goal-pin after both CUs (matches `value_table(goal_mask) = 0`
            // at the bottom of `vi_sweep_stream_algo.m`).
            for iy in 0..map_y_us {
                for ix in 0..map_x_us {
                    for it in 0..N_THETA {
                        if ctx.goal_mask[[iy, ix, it]] {
                            ctx.value[[iy, ix, it]] = 0;
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

        SolveStats {
            iters_or_sweeps: sweeps,
            // updates not tracked — matches the Reference solver convention.
            updates: 0,
            final_delta,
            converged,
            extra: None,
        }
    }
}

/// Run one CU pass: iterate over this CU's half of the strips and update
/// each cell in place. Returns the per-CU max residual.
fn run_cu(ctx: &mut VIContext, cu_id: u32, map_x: u32, map_y: u32) -> Value {
    let num_strips = map_x.div_ceil(STRIP_W_MAX);
    let half_strips = num_strips.div_ceil(2);
    let mut strip_max_delta: Value = 0;
    for si in 0..half_strips {
        let sx = if cu_id == 0 { si } else { num_strips - 1 - si };
        let strip_x0 = sx * STRIP_W_MAX;
        let strip_w = STRIP_W_MAX.min(map_x - strip_x0);
        let d = run_strip(ctx, cu_id, map_x, map_y, strip_x0, strip_w);
        if d > strip_max_delta {
            strip_max_delta = d;
        }
    }
    strip_max_delta
}

/// Run one strip pass. CU0 scans top-to-bottom × left-to-right; CU1 scans
/// bottom-to-top × right-to-left. In-place Gauss-Seidel updates against
/// `ctx.value` so downstream cells within the same strip see freshly
/// computed neighbours. Threshold is unused here — convergence is checked at
/// the sweep level after both CUs complete.
fn run_strip(
    ctx: &mut VIContext,
    cu_id: u32,
    map_x: u32,
    map_y: u32,
    strip_x0: u32,
    strip_w: u32,
) -> Value {
    let mut strip_max_delta: Value = 0;
    for iy_raw in 0..map_y {
        let iy = if cu_id == 0 { iy_raw } else { map_y - 1 - iy_raw };
        for ix_raw in 0..strip_w {
            let ix_local = if cu_id == 0 { ix_raw } else { strip_w - 1 - ix_raw };
            let ix = strip_x0 + ix_local;
            let iy_us = iy as usize;
            let ix_us = ix as usize;
            if ctx.penalty[[iy_us, ix_us]] == PENALTY_OBSTACLE {
                continue;
            }
            for it in 0..N_THETA {
                if ctx.goal_mask[[iy_us, ix_us, it]] {
                    ctx.value[[iy_us, ix_us, it]] = 0;
                    continue;
                }
                let old = ctx.value[[iy_us, ix_us, it]];
                let new = bellman_backup(
                    &ctx.value,
                    &ctx.penalty,
                    &ctx.transitions,
                    ix,
                    iy,
                    it as u32,
                    map_x,
                    map_y,
                );
                ctx.value[[iy_us, ix_us, it]] = new;
                let d = new.abs_diff(old);
                if d > strip_max_delta {
                    strip_max_delta = d;
                }
            }
        }
    }
    strip_max_delta
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frontier::test_helpers::{
        empty_3x3_ctx, empty_5x5_ctx, obstacle_3x3_ctx, sentinel_3x3_ctx,
    };
    use crate::reference::Reference;

    fn parity_run(mut ctx_ref: VIContext) {
        let mut ctx_stream = ctx_ref.clone_value();
        Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(100));
        let stats = StreamMimic { threshold: 0 }
            .run(&mut ctx_stream, Budget::Sweeps(200));
        assert!(
            stats.converged,
            "StreamMimic must converge: sweeps={}",
            stats.iters_or_sweeps
        );
        assert_eq!(
            ctx_ref.value, ctx_stream.value,
            "StreamMimic must reach same fixed point as Reference"
        );
    }

    #[test]
    fn parity_empty_3x3() {
        parity_run(empty_3x3_ctx());
    }

    #[test]
    fn parity_empty_5x5() {
        parity_run(empty_5x5_ctx());
    }

    #[test]
    fn parity_obstacle_3x3() {
        parity_run(obstacle_3x3_ctx());
    }

    #[test]
    fn parity_sentinel_3x3() {
        parity_run(sentinel_3x3_ctx());
    }

    #[test]
    fn budget_exhaustion_not_converged() {
        // A 5x5 grid does not converge in a single sweep — verify that a
        // tight budget causes the solver to report `converged = false`.
        let mut ctx = empty_5x5_ctx();
        let stats = StreamMimic { threshold: 0 }
            .run(&mut ctx, Budget::Sweeps(1));
        assert!(
            !stats.converged,
            "1-sweep budget should not converge on 5x5; got converged=true"
        );
        assert_eq!(stats.iters_or_sweeps, 1);
    }
}
