//! Coarse-theta approximate solve plus exact refine pass.
//!
//! Solves only every `coarse_step` theta layer using snapped transition theta,
//! upsamples that value field to all layers, then runs exact frontier refine.
//! Mirrors `vi_matlab/src/cpu/frontier/vi_frontier_3d_coarse_theta.m`.
//! See spec §4.2, §4.8.

use ndarray::{Array2, Array3};
use vi_core::params::{MAX_VALUE, N_ACTIONS, N_THETA, PROB_BASE};
use vi_core::{Penalty, TransitionModel, Value};

use crate::bitboard::Bitboard3D;
use crate::context::{Budget, SolveStats, Solver, VIContext};

use super::{
    Frontier3D, build_passable_bb_2d, build_passable_bb_3d, max_iters, pin_goals,
};

/// Frontier-VI using a two-phase coarse-then-refine strategy.
///
/// **Phase 1 (coarse pass)**: solves only every `coarse_step`-th theta
/// layer (0, step, 2·step, …), snapping neighbor theta indices to the
/// nearest coarse layer. This reduces the theta search space by `coarse_step×`.
///
/// **Upsample**: each non-coarse theta layer is filled by copying from
/// its nearest coarse layer.
///
/// **Phase 2 (refine pass)**: runs normal [`Frontier3D`] for up to
/// `refine_iters` iterations over the full (now-initialized) value field.
///
/// * `coarse_step <= 1` → delegates to [`Frontier3D`] directly.
pub struct Frontier3DCoarseTheta {
    /// Coarsening factor for the theta axis. Only every `coarse_step`-th theta
    /// layer is solved exactly during the coarse pass; non-coarse layers are
    /// filled via nearest-coarse-layer upsampling before refine. `coarse_step <= 1`
    /// delegates directly to [`Frontier3D`].
    pub coarse_step: u32,
    /// Number of iterations reserved for the refine pass (exact [`Frontier3D`]
    /// over all theta layers). Coarse pass gets `max_iters - refine_iters`.
    pub refine_iters: u32,
}

impl Solver for Frontier3DCoarseTheta {
    fn name(&self) -> &'static str {
        "frontier_3d_coarse_theta"
    }

    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        let step = self.coarse_step as usize;
        let max_iter = max_iters(budget);

        // MATLAB: if step <= 1, delegate to vi_frontier_3d.
        if step <= 1 {
            return Frontier3D.run(ctx, budget);
        }

        // Budget split: coarse gets `max_iter - refine_cap` iterations;
        // refine gets `refine_cap` (capped to [0, max_iter]).
        let refine_cap = (self.refine_iters).min(max_iter);
        let coarse_cap = max_iter - refine_cap;

        // ---------------------------------------------------------------------------
        // Coarse pass
        // ---------------------------------------------------------------------------
        let (coarse_iters, coarse_updates) =
            run_coarse_pass(ctx, step, coarse_cap);

        // ---------------------------------------------------------------------------
        // Upsample: copy each coarse layer to its non-coarse neighbours.
        // ---------------------------------------------------------------------------
        upsample_coarse_theta(&mut ctx.value, step);

        // Re-pin original goal cells (MATLAB does this after upsample, before refine).
        pin_goals(&mut ctx.value, &ctx.goal_mask);

        // ---------------------------------------------------------------------------
        // Refine pass (normal Frontier3D)
        // ---------------------------------------------------------------------------
        let (refine_iters_done, refine_updates, refine_converged) = if refine_cap > 0 {
            let stats = Frontier3D.run(ctx, Budget::Iterations(refine_cap));
            (stats.iters_or_sweeps, stats.updates, stats.converged)
        } else {
            // WHY: coarse convergence over the reduced (every step-th theta) problem does
            // not imply convergence over the full theta-axis problem. When refine_cap == 0
            // we have no measurement of full-space residual, so `false` is the correct
            // conservative signal (a caller checking `stats.converged` should consider this
            // run incomplete).
            (0, 0, false)
        };

        SolveStats {
            iters_or_sweeps: coarse_iters + refine_iters_done,
            updates: coarse_updates + refine_updates,
            final_delta: 0,
            converged: refine_converged,
            extra: None,
        }
    }
}

// ---------------------------------------------------------------------------
// Coarse pass implementation
// ---------------------------------------------------------------------------

/// Run the coarse-theta frontier pass.
///
/// Modifies `ctx.value` in place (resets it to MAX_VALUE, then pins coarse
/// goal cells to 0, then runs the frontier loop restricted to coarse layers).
///
/// Returns `(iters, total_updates)`.
fn run_coarse_pass(
    ctx: &mut VIContext,
    step: usize,
    coarse_cap: u32,
) -> (u32, u64) {
    let map_x = ctx.dims.map_x;
    let map_y = ctx.dims.map_y;

    let (mx, my, mt) = ctx.transitions.max_displacement();
    let mx = mx as u32;
    let my = my as u32;
    let mt = mt as u32;

    // Reset value table (coarse pass starts from scratch).
    ctx.value.fill(MAX_VALUE);

    // Build coarse-goal mask: for each source theta layer, map it to its
    // nearest coarse theta, then OR in the goal for that coarse layer.
    // MATLAB: for it=1:NT, cit=nearest..., coarse_goal(:,:,cit) |= goal_mask(:,:,it)
    let my_sz = ctx.goal_mask.shape()[0];
    let mx_sz = ctx.goal_mask.shape()[1];
    let mut coarse_goal = Array3::<bool>::from_elem((my_sz, mx_sz, N_THETA), false);
    for iy in 0..my_sz {
        for ix in 0..mx_sz {
            for it in 0..N_THETA {
                if ctx.goal_mask[[iy, ix, it]] {
                    let cit = nearest_coarse_theta(it, step);
                    coarse_goal[[iy, ix, cit]] = true;
                }
            }
        }
    }

    // Pin coarse goal cells to 0.
    for ((iy, ix, it), &is_cgoal) in coarse_goal.indexed_iter() {
        if is_cgoal {
            ctx.value[[iy, ix, it]] = 0;
        }
    }

    // Build passable bitboard.
    let passable_2d = build_passable_bb_2d(&ctx.penalty);
    let passable_bb = build_passable_bb_3d(&passable_2d, N_THETA as u32);

    // Build coarse-layer bitboard: full spatial plane × coarse theta layers only.
    // Each coarse theta layer: all cells true; non-coarse layers: all false.
    let mut coarse_layer_mask = Array3::<bool>::from_elem((my_sz, mx_sz, N_THETA), false);
    let mut it = 0usize;
    while it < N_THETA {
        for iy in 0..my_sz {
            for ix in 0..mx_sz {
                coarse_layer_mask[[iy, ix, it]] = true;
            }
        }
        it += step;
    }
    let coarse_layer_bb = Bitboard3D::from_logical(coarse_layer_mask.view());

    let coarse_goal_bb = Bitboard3D::from_logical(coarse_goal.view());
    let not_goal_bb = coarse_goal_bb.complement();

    // Frontier seed = coarse goal cells (where value just became 0).
    let mut frontier = coarse_goal_bb.clone();

    let mut total_updates: u64 = 0;
    let mut iters: u32 = 0;

    while frontier.popcount() > 0 && iters < coarse_cap {
        iters += 1;

        let mut candidates = frontier.dilate(mx, my, mt);
        candidates.and_inplace(&passable_bb);
        candidates.and_inplace(&coarse_layer_bb);
        candidates.and_inplace(&not_goal_bb);

        let mut new_frontier = Bitboard3D::new(map_x, map_y, N_THETA as u32);

        for (ix, iy, it) in candidates.enumerate() {
            let ix_us = ix as usize;
            let iy_us = iy as usize;
            let it_us = it as usize;
            let old_val = ctx.value[[iy_us, ix_us, it_us]];
            let new_val = bellman_backup_coarse_theta(
                &ctx.value,
                &ctx.penalty,
                &ctx.transitions,
                ix,
                iy,
                it,
                map_x,
                map_y,
                step,
            );
            if new_val < old_val {
                ctx.value[[iy_us, ix_us, it_us]] = new_val;
                total_updates += 1;
                new_frontier.set(ix, iy, it);
            }
        }

        frontier = new_frontier;
    }

    (iters, total_updates)
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Snap theta index `it` (0-based) to the nearest coarse theta layer.
///
/// Mirrors MATLAB's `nearest_coarse_theta` (1-based):
/// `q = floor((it-1 + floor(step/2)) / step)`, `cit = q*step + 1`.
/// In Rust 0-based: `cit = q * step` with modular wrap when `cit >= N_THETA`.
fn nearest_coarse_theta(it: usize, step: usize) -> usize {
    let q = (it + step / 2) / step;
    let mut cit = q * step;
    // Wrap: if cit >= NT, subtract NT (matches MATLAB's while cit > NT: cit -= NT).
    while cit >= N_THETA {
        cit -= N_THETA;
    }
    cit
}

/// Copy coarse layer values to their non-coarse neighbours in place.
///
/// Mirrors MATLAB's `upsample_coarse_theta`.
fn upsample_coarse_theta(value: &mut Array3<Value>, step: usize) {
    let (my, mx, _) = (value.shape()[0], value.shape()[1], value.shape()[2]);
    for it in 0..N_THETA {
        let cit = nearest_coarse_theta(it, step);
        if cit != it {
            for iy in 0..my {
                for ix in 0..mx {
                    value[[iy, ix, it]] = value[[iy, ix, cit]];
                }
            }
        }
    }
}

/// Single-cell Bellman backup with theta snapping to the nearest coarse layer.
///
/// Identical to `bellman_backup` except that after computing `nt` for each
/// outcome, it is snapped via `nearest_coarse_theta(nt, step)` before
/// indexing the value table. This ensures that only coarse-layer values are
/// read during the coarse pass.
///
/// Mirrors `vi_frontier_bellman_coarse_theta` in the MATLAB source.
///
/// NOTE: structurally near-duplicate of bellman_backup in kernel/bellman.rs.
/// Differs only in divisor (prob_sum vs PROB_BASE) and prob_sum==0 guard.
/// Deduplication via a shared inner-loop helper deferred to a future cleanup.
#[allow(clippy::too_many_arguments)]
fn bellman_backup_coarse_theta(
    value: &Array3<Value>,
    penalty: &Array2<Penalty>,
    trans: &TransitionModel,
    ix: u32,
    iy: u32,
    it: u32,
    map_x: u32,
    map_y: u32,
    step: usize,
) -> Value {
    let it_us = it as usize;
    let mut v_new = MAX_VALUE;

    for a in 0..N_ACTIONS {
        let n_out = trans.n_outcomes[a][it_us] as usize;
        if n_out == 0 {
            continue;
        }

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
            // Snap to nearest coarse layer.
            let nt_coarse = nearest_coarse_theta(nt as usize, step);

            if nx < 0 || nx >= map_x as i32 || ny < 0 || ny >= map_y as i32 {
                valid = false;
                break;
            }

            let neighbor_val = value[[ny as usize, nx as usize, nt_coarse]];
            let pen = penalty[[ny as usize, nx as usize]];
            let step_cost = vi_core::cost_of(neighbor_val, pen);
            if step_cost == MAX_VALUE {
                valid = false;
                break;
            }

            accum += step_cost as u64 * trans.prob[a][it_us][k] as u64;
        }

        let c: Value = if valid {
            let div = accum / PROB_BASE as u64;
            if div >= MAX_VALUE as u64 { MAX_VALUE - 1 } else { div as Value }
        } else {
            MAX_VALUE
        };

        if c < v_new {
            v_new = c;
        }
    }

    v_new
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
    use vi_core::params::MAX_VALUE;
    use super::super::test_helpers::empty_5x5_ctx;

    #[test]
    fn coarse_step_one_matches_frontier_3d() {
        // step <= 1 → delegate to Frontier3D → bit-exact.
        let mut ctx_coarse = empty_5x5_ctx();
        let mut ctx_f3d = ctx_coarse.clone_value();

        Frontier3DCoarseTheta { coarse_step: 1, refine_iters: 0 }
            .run(&mut ctx_coarse, Budget::Iterations(200));
        Frontier3D.run(&mut ctx_f3d, Budget::Iterations(200));

        assert_eq!(
            ctx_coarse.value, ctx_f3d.value,
            "coarse_step=1 must delegate to Frontier3D (bit-exact)"
        );
    }

    #[test]
    fn coarse_pass_terminates() {
        let mut ctx = empty_5x5_ctx();
        let stats = Frontier3DCoarseTheta { coarse_step: 5, refine_iters: 50 }
            .run(&mut ctx, Budget::Iterations(200));
        assert!(
            stats.iters_or_sweeps > 0,
            "should have run at least one iteration"
        );
    }

    #[test]
    fn coarse_close_to_reference_after_refine() {
        // With deterministic 4-dir transitions (dit=0), layer 0 is the only layer
        // where Reference produces finite values. Compare only layer 0.
        let mut ctx_ref = empty_5x5_ctx();
        let mut ctx_coarse = empty_5x5_ctx();

        Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(100));
        Frontier3DCoarseTheta { coarse_step: 5, refine_iters: 100 }
            .run(&mut ctx_coarse, Budget::Iterations(300));

        // Compare layer 0 only.
        let count = 5 * 5;
        let mean_abs_diff: f64 = (0..5usize)
            .flat_map(|iy| (0..5usize).map(move |ix| (iy, ix)))
            .map(|(iy, ix)| {
                let r = ctx_ref.value[[iy, ix, 0]];
                let c = ctx_coarse.value[[iy, ix, 0]];
                (r as i32 - c as i32).abs() as f64
            })
            .sum::<f64>()
            / count as f64;

        assert!(
            mean_abs_diff < 10.0,
            "layer-0 mean abs diff vs Reference = {:.2} (step=5, refine=100)",
            mean_abs_diff
        );
    }

    #[test]
    fn nearest_coarse_theta_basic() {
        // step=5: coarse layers at 0, 5, 10, ...
        // it=0 → 0, it=2 → 0, it=3 → 5, it=5 → 5, it=57 → 55, it=58 → 60%60=0
        assert_eq!(nearest_coarse_theta(0, 5), 0);
        assert_eq!(nearest_coarse_theta(2, 5), 0);
        assert_eq!(nearest_coarse_theta(3, 5), 5);
        assert_eq!(nearest_coarse_theta(5, 5), 5);
        assert_eq!(nearest_coarse_theta(55, 5), 55);
        assert_eq!(nearest_coarse_theta(57, 5), 55);
        // it=58: (58+2)/5=12, cit=60 → 60-60=0 ✓
        assert_eq!(nearest_coarse_theta(58, 5), 0);
    }

    #[test]
    fn upsample_fills_non_coarse_layers() {
        // 2×2 map, step=5.
        // Set coarse layer 0 to a known value, all others MAX_VALUE.
        // After upsample, layers 1–4 should equal layer 0.
        use ndarray::Array3;
        let mut value = Array3::<Value>::from_elem((2, 2, N_THETA), MAX_VALUE);
        // Fill coarse layer 0 with 42.
        for iy in 0..2 {
            for ix in 0..2 {
                value[[iy, ix, 0]] = 42;
            }
        }
        upsample_coarse_theta(&mut value, 5);
        // Layers 1–2 snap to nearest coarse 0; layer 3 snaps to 5 (still MV).
        for iy in 0..2usize {
            for ix in 0..2usize {
                assert_eq!(value[[iy, ix, 1]], 42, "layer 1 should copy from coarse layer 0");
                assert_eq!(value[[iy, ix, 2]], 42, "layer 2 should copy from coarse layer 0");
                assert_eq!(value[[iy, ix, 0]], 42, "coarse layer 0 unchanged");
            }
        }
    }

    #[test]
    fn goal_cell_pinned_after_coarse_run() {
        let mut ctx = empty_5x5_ctx();
        Frontier3DCoarseTheta { coarse_step: 5, refine_iters: 50 }
            .run(&mut ctx, Budget::Iterations(200));
        // Goal at (iy=2, ix=2, it=0).
        assert_eq!(ctx.value[[2, 2, 0]], 0, "goal cell must be pinned to 0");
    }

    #[test]
    fn stats_fields_sane() {
        let mut ctx = empty_5x5_ctx();
        let stats = Frontier3DCoarseTheta { coarse_step: 3, refine_iters: 50 }
            .run(&mut ctx, Budget::Iterations(200));
        assert_eq!(stats.final_delta, 0);
        assert!(stats.extra.is_none());
    }

}
