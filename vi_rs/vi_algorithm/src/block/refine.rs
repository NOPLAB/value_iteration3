//! Block-frontier value iteration with fine updates per block.
//!
//! The scheduler is coarse: it tracks changed spatial blocks. The backup is
//! fine: every active block updates all theta states with the reference
//! Bellman operator. With `threshold == 0` this converges to the same fixed
//! point as Reference, but usually touches fewer blocks on sparse maps.
//!
//! Mirrors `vi_matlab/src/cpu/block/vi_block_refine.m`.
//! See spec §4.2.

use ndarray::{Array2, Array3};
use vi_core::{Value, PENALTY_OBSTACLE, N_THETA, MAX_VALUE};

use crate::context::{Budget, SolveStats, Solver, VIContext};
use crate::kernel::bellman_backup;

/// Block-frontier value iteration with fine updates per block.
pub struct BlockRefine {
    /// Block width in cells. Default in MATLAB is 8.
    pub block_w: u32,
    /// Block height in cells. Defaults to `block_w` when 0.
    pub block_h: u32,
    /// Number of inner sweeps per active block. MATLAB default: 2.
    pub local_sweeps: u32,
    /// Residual threshold for marking a block "still changing". 0 = bit-exact with Reference.
    pub threshold: Value,
}

impl Solver for BlockRefine {
    fn name(&self) -> &'static str {
        "block_refine"
    }

    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        let max_iters = match budget {
            Budget::Sweeps(n) => n,
            Budget::Iterations(n) => n,
        };

        let map_x = ctx.dims.map_x;
        let map_y = ctx.dims.map_y;
        let map_x_us = map_x as usize;
        let map_y_us = map_y as usize;

        let bw = self.block_w.max(1);
        let bh = if self.block_h == 0 { bw } else { self.block_h }.max(1);
        let local_passes = self.local_sweeps.max(1);

        let n_bx = map_x.div_ceil(bw) as usize;
        let n_by = map_y.div_ceil(bh) as usize;

        let (mx, my, _) = ctx.transitions.max_displacement();
        let rx = (mx as u32).div_ceil(bw) as usize;
        let ry = (my as u32).div_ceil(bh) as usize;

        // Pin goal cells before building the frontier seed.
        for iy in 0..map_y_us {
            for ix in 0..map_x_us {
                for it in 0..N_THETA {
                    if ctx.goal_mask[[iy, ix, it]] {
                        ctx.value[[iy, ix, it]] = 0;
                    }
                }
            }
        }

        let passable_blocks = blocks_from_cell_mask(
            &build_passable_mask(&ctx.penalty),
            n_bx as u32, n_by as u32, bw, bh, map_x, map_y,
        );

        // Initial frontier: any block containing a non-max-value cell or a goal cell.
        let value_nonmax = build_value_nonmax_mask(&ctx.value);
        let goal_any = build_goal_any_mask(&ctx.goal_mask);
        let frontier_seed = array2_or(&value_nonmax, &goal_any);
        let mut frontier_blocks = blocks_from_cell_mask(
            &frontier_seed,
            n_bx as u32, n_by as u32, bw, bh, map_x, map_y,
        );

        let mut total_updates: u64 = 0;
        let mut iters: u32 = 0;
        let mut final_delta: Value = MAX_VALUE;
        let mut converged = false;

        while any_true(&frontier_blocks) && iters < max_iters {
            iters += 1;
            let mut active_blocks = dilate_blocks(&frontier_blocks, n_bx, n_by, rx, ry);
            // Restrict to passable blocks.
            for by in 0..n_by {
                for bx in 0..n_bx {
                    if !passable_blocks[[by, bx]] {
                        active_blocks[[by, bx]] = false;
                    }
                }
            }

            let mut next_frontier = Array2::<bool>::from_elem((n_by, n_bx), false);
            let mut max_delta: Value = 0;

            for by in 0..n_by {
                let y0 = by * bh as usize;
                let y1 = ((by + 1) * bh as usize).min(map_y_us) - 1;
                for bx in 0..n_bx {
                    if !active_blocks[[by, bx]] {
                        continue;
                    }
                    let x0 = bx * bw as usize;
                    let x1 = ((bx + 1) * bw as usize).min(map_x_us) - 1;

                    let (block_updates, block_delta) = update_block(
                        &mut ctx.value,
                        &ctx.penalty,
                        &ctx.goal_mask,
                        &ctx.transitions,
                        x0, x1, y0, y1,
                        map_x, map_y,
                        local_passes,
                    );
                    total_updates += block_updates;
                    if block_delta > max_delta {
                        max_delta = block_delta;
                    }
                    if block_delta > self.threshold {
                        next_frontier[[by, bx]] = true;
                    }
                }
            }

            final_delta = max_delta;
            frontier_blocks = next_frontier;
            if max_delta <= self.threshold {
                converged = true;
                break;
            }
        }

        // Final goal pin, matching MATLAB post-loop `value_table(goal_mask) = 0`.
        for iy in 0..map_y_us {
            for ix in 0..map_x_us {
                for it in 0..N_THETA {
                    if ctx.goal_mask[[iy, ix, it]] {
                        ctx.value[[iy, ix, it]] = 0;
                    }
                }
            }
        }

        SolveStats {
            iters_or_sweeps: iters,
            updates: total_updates,
            final_delta,
            converged,
            extra: None,
        }
    }
}

// ---------------------------------------------------------------------------
// Inner sweep — mirrors MATLAB `update_block`.
//
// Only applies updates where new < old (monotone descent). Returns (updates, max_delta).
// ---------------------------------------------------------------------------
#[allow(clippy::too_many_arguments)]
fn update_block(
    value: &mut Array3<Value>,
    penalty: &Array2<vi_core::Penalty>,
    goal_mask: &Array3<bool>,
    trans: &vi_core::TransitionModel,
    x0: usize, x1: usize,
    y0: usize, y1: usize,
    map_x: u32,
    map_y: u32,
    local_passes: u32,
) -> (u64, Value) {
    let mut updates: u64 = 0;
    let mut max_delta: Value = 0;

    for _pass in 0..local_passes {
        let mut pass_delta: Value = 0;
        for iy in y0..=y1 {
            for ix in x0..=x1 {
                if penalty[[iy, ix]] == PENALTY_OBSTACLE {
                    continue;
                }
                for it in 0..N_THETA {
                    if goal_mask[[iy, ix, it]] {
                        value[[iy, ix, it]] = 0;
                        continue;
                    }
                    let old = value[[iy, ix, it]];
                    let new_val = bellman_backup(
                        value, penalty, trans,
                        ix as u32, iy as u32, it as u32,
                        map_x, map_y,
                    );
                    if new_val < old {
                        value[[iy, ix, it]] = new_val;
                        updates += 1;
                        let d = old - new_val;
                        if d > pass_delta {
                            pass_delta = d;
                        }
                    }
                }
            }
        }
        if pass_delta > max_delta {
            max_delta = pass_delta;
        }
        if pass_delta == 0 {
            break;
        }
    }

    (updates, max_delta)
}

// ---------------------------------------------------------------------------
// Block-grid helpers
// ---------------------------------------------------------------------------

/// Build a `[n_by, n_bx]` boolean array: each block is true if any cell in
/// the corresponding region of `mask` is true.
fn blocks_from_cell_mask(
    mask: &Array2<bool>,
    n_bx: u32, n_by: u32,
    bw: u32, bh: u32,
    map_x: u32, map_y: u32,
) -> Array2<bool> {
    let n_by = n_by as usize;
    let n_bx = n_bx as usize;
    let bw = bw as usize;
    let bh = bh as usize;
    let map_x_us = map_x as usize;
    let map_y_us = map_y as usize;

    let mut out = Array2::<bool>::from_elem((n_by, n_bx), false);
    for by in 0..n_by {
        let y0 = by * bh;
        let y1 = ((by + 1) * bh).min(map_y_us);
        for bx in 0..n_bx {
            let x0 = bx * bw;
            let x1 = ((bx + 1) * bw).min(map_x_us);
            'cell: for iy in y0..y1 {
                for ix in x0..x1 {
                    if mask[[iy, ix]] {
                        out[[by, bx]] = true;
                        break 'cell;
                    }
                }
            }
        }
    }
    out
}

/// Dilate the block frontier: for each true block, OR a [-rx..rx, -ry..ry] box into output.
fn dilate_blocks(
    blocks: &Array2<bool>,
    n_bx: usize, n_by: usize,
    rx: usize, ry: usize,
) -> Array2<bool> {
    let mut out = Array2::<bool>::from_elem((n_by, n_bx), false);
    for by in 0..n_by {
        for bx in 0..n_bx {
            if !blocks[[by, bx]] {
                continue;
            }
            let x0 = bx.saturating_sub(rx);
            let x1 = (bx + rx).min(n_bx - 1);
            let y0 = by.saturating_sub(ry);
            let y1 = (by + ry).min(n_by - 1);
            for dy in y0..=y1 {
                for dx in x0..=x1 {
                    out[[dy, dx]] = true;
                }
            }
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Cell-level mask builders
// ---------------------------------------------------------------------------

fn build_passable_mask(penalty: &Array2<vi_core::Penalty>) -> Array2<bool> {
    penalty.mapv(|p| p != PENALTY_OBSTACLE)
}

fn build_value_nonmax_mask(value: &Array3<Value>) -> Array2<bool> {
    let (my, mx, _) = (value.shape()[0], value.shape()[1], value.shape()[2]);
    let mut out = Array2::<bool>::from_elem((my, mx), false);
    for ((iy, ix, _it), &v) in value.indexed_iter() {
        if v < MAX_VALUE {
            out[[iy, ix]] = true;
        }
    }
    out
}

fn build_goal_any_mask(goal_mask: &Array3<bool>) -> Array2<bool> {
    let (my, mx, _) = (goal_mask.shape()[0], goal_mask.shape()[1], goal_mask.shape()[2]);
    let mut out = Array2::<bool>::from_elem((my, mx), false);
    for ((iy, ix, _it), &g) in goal_mask.indexed_iter() {
        if g {
            out[[iy, ix]] = true;
        }
    }
    out
}

fn array2_or(a: &Array2<bool>, b: &Array2<bool>) -> Array2<bool> {
    let shape = a.raw_dim();
    let mut out = Array2::<bool>::from_elem(shape, false);
    for (i, (&av, &bv)) in a.iter().zip(b.iter()).enumerate() {
        out.as_slice_mut().unwrap()[i] = av || bv;
    }
    out
}

fn any_true(arr: &Array2<bool>) -> bool {
    arr.iter().any(|&v| v)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::Budget;
    use crate::reference::Reference;
    use crate::frontier::test_helpers::{empty_3x3_ctx, empty_5x5_ctx, obstacle_3x3_ctx};

    #[test]
    fn parity_block_refine_empty_3x3() {
        let mut ctx_ref = empty_3x3_ctx();
        let mut ctx_block = ctx_ref.clone_value();

        Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        BlockRefine { block_w: 2, block_h: 2, local_sweeps: 2, threshold: 0 }
            .run(&mut ctx_block, Budget::Sweeps(50));

        assert_eq!(ctx_ref.value, ctx_block.value, "BlockRefine(threshold=0) must be bit-exact with Reference on empty 3x3");
    }

    #[test]
    fn parity_block_refine_empty_5x5() {
        let mut ctx_ref = empty_5x5_ctx();
        let mut ctx_block = ctx_ref.clone_value();

        Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(100));
        BlockRefine { block_w: 2, block_h: 2, local_sweeps: 2, threshold: 0 }
            .run(&mut ctx_block, Budget::Sweeps(100));

        assert_eq!(ctx_ref.value, ctx_block.value, "BlockRefine(threshold=0) must be bit-exact with Reference on empty 5x5");
    }

    #[test]
    fn parity_block_refine_obstacle_3x3() {
        let mut ctx_ref = obstacle_3x3_ctx();
        let mut ctx_block = ctx_ref.clone_value();

        Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(50));
        BlockRefine { block_w: 2, block_h: 2, local_sweeps: 2, threshold: 0 }
            .run(&mut ctx_block, Budget::Sweeps(50));

        assert_eq!(ctx_ref.value, ctx_block.value, "BlockRefine(threshold=0) must be bit-exact with Reference on obstacle 3x3");
    }

    #[test]
    fn block_refine_terminates() {
        let mut ctx = empty_3x3_ctx();
        let stats = BlockRefine { block_w: 2, block_h: 2, local_sweeps: 2, threshold: 0 }
            .run(&mut ctx, Budget::Sweeps(100));
        assert!(stats.converged);
        assert!(stats.iters_or_sweeps < 100);
    }

    #[test]
    fn block_refine_threshold_above_zero_terminates_earlier() {
        let mut ctx_strict = empty_5x5_ctx();
        let mut ctx_relaxed = ctx_strict.clone_value();

        let s_strict = BlockRefine { block_w: 2, block_h: 2, local_sweeps: 2, threshold: 0 }
            .run(&mut ctx_strict, Budget::Sweeps(200));
        let s_relaxed = BlockRefine { block_w: 2, block_h: 2, local_sweeps: 2, threshold: 5 }
            .run(&mut ctx_relaxed, Budget::Sweeps(200));

        assert!(s_strict.converged);
        assert!(s_relaxed.converged);
        assert!(
            s_relaxed.iters_or_sweeps <= s_strict.iters_or_sweeps,
            "threshold=5 took {} iters; threshold=0 took {}",
            s_relaxed.iters_or_sweeps, s_strict.iters_or_sweeps
        );
    }
}
