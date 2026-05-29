//! Coarse-to-fine VI over a 2×2 spatial pyramid.
//!
//! Starts from the coarsest 2×2-reduced map, sweeps only active regions,
//! then descends into the children of changed coarse blocks.
//!
//! Mirrors `vi_matlab/src/cpu/block/vi_pyramid_sweep.m`.
//! See spec §4.2.

use ndarray::{Array2, Array3};
use vi_core::{Value, Penalty, TransitionModel, PENALTY_OBSTACLE, N_THETA, MAX_VALUE};
use vi_core::params::N_ACTIONS;

use crate::context::{Budget, PyramidLevelStat, SolveExtra, SolveStats, Solver, VIContext};
use crate::kernel::bellman_backup;

const MAX_LEVELS: usize = 16;

/// Coarse-to-fine VI over a 2×2 spatial pyramid.
pub struct PyramidSweep {
    /// Convergence residual cap. 0 = strict convergence within each level's sweep budget.
    pub threshold: Value,
    /// Stop coarsening when level dimensions are ≤ min_size on both axes.
    pub min_size: u32,
    /// Sweep budget per coarse level (level > 1).
    pub coarse_sweeps: u32,
    /// Sweep budget for the finest level (level 1).
    pub refine_sweeps: u32,
    /// Residual threshold for triggering descent: a cell marked as changed only if
    /// its max per-theta delta > descend_tau.
    pub descend_tau: Value,
}

impl Solver for PyramidSweep {
    fn name(&self) -> &'static str {
        "pyramid_sweep"
    }

    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats {
        let max_sweeps = match budget {
            Budget::Sweeps(n) => n,
            Budget::Iterations(n) => n,
        };

        let map_x = ctx.dims.map_x;
        let map_y = ctx.dims.map_y;

        // Build pyramid levels: level 0 = finest (original), higher = coarser.
        // We store them in Vec with index 0 = level 1 (finest), index k = level k+1.
        let mut level_mx = [0u32; MAX_LEVELS];
        let mut level_my = [0u32; MAX_LEVELS];
        let mut values: Vec<Array3<Value>> = Vec::with_capacity(MAX_LEVELS);
        let mut penalties: Vec<Array2<Penalty>> = Vec::with_capacity(MAX_LEVELS);
        let mut goals: Vec<Array3<bool>> = Vec::with_capacity(MAX_LEVELS);
        let mut active_masks: Vec<Option<Array2<bool>>> = Vec::with_capacity(MAX_LEVELS);

        // Level 0 in Vec = level 1 in MATLAB (finest).
        level_mx[0] = map_x;
        level_my[0] = map_y;
        values.push(ctx.value.clone());
        penalties.push(ctx.penalty.clone());
        goals.push(ctx.goal_mask.clone());
        active_masks.push(None);

        let mut n_levels: usize = 1;

        // Build coarser levels (matching MATLAB while loop).
        while n_levels < MAX_LEVELS {
            let prev = n_levels - 1;
            let mx = level_mx[prev];
            let my = level_my[prev];
            // Stop if both dimensions are at or below min_size, OR already 1x1.
            if (mx <= self.min_size && my <= self.min_size) || (mx <= 1 && my <= 1) {
                break;
            }
            let (cv, cp, cg) = coarsen_level(&values[prev], &penalties[prev], &goals[prev], mx, my);
            let cmx = mx.div_ceil(2);
            let cmy = my.div_ceil(2);
            level_mx[n_levels] = cmx;
            level_my[n_levels] = cmy;
            values.push(cv);
            penalties.push(cp);
            goals.push(cg);
            active_masks.push(None);
            n_levels += 1;
        }

        // Seed coarsest level with its goal spatial mask.
        {
            let top = n_levels - 1;
            active_masks[top] = Some(goal_any_2d(&goals[top]));
        }

        let mut sweeps_total: u32 = 0;
        let mut visited_total: u64 = 0;
        let mut final_delta: Value = MAX_VALUE;
        let mut per_level: Vec<PyramidLevelStat> = Vec::with_capacity(n_levels);
        // Pre-fill per_level with placeholder entries in level order.
        for li in 0..n_levels {
            per_level.push(PyramidLevelStat {
                level: (li + 1) as u32,
                map_x: level_mx[li],
                map_y: level_my[li],
                scale: 1u32 << li,
                sweeps: 0,
                changed_states: 0,
                visited_states: 0,
                final_delta: 0,
            });
        }

        let mut remaining = max_sweeps;

        // Descend from coarsest (n_levels-1) to finest (0).
        for li in (0..n_levels).rev() {
            // Prolongate value from the coarser level for *intermediate* levels only.
            //
            // WHY skip the finest level (li == 0): the probabilistic Bellman backup
            // averages multi-outcome neighbor costs and floors the result
            // (`accum / PROB_BASE`), so its fixed point is NOT unique — value
            // iteration from an under-estimate converges to a fixed point up to 1
            // below the one reached from above. Reference (and every other exact
            // solver) starts from the original pessimistic seed and converges from
            // above to the true V*. Prolongation hands the finest level an
            // *optimistic* (under-estimate) seed, which would land it on the lower
            // fixed point and break bit-exactness by ±1. Leaving `values[0]` at the
            // original seed makes the finest level converge from above, exactly like
            // Reference. The coarse descent still does its job: it builds the active
            // mask (which cells to sweep), which is all the finest level needs.
            if li > 0 && li < n_levels - 1 {
                let coarser = li + 1;
                let fine_val = prolongate_level(
                    &values[coarser],
                    &penalties[li],
                    &goals[li],
                    level_mx[li],
                    level_my[li],
                );
                values[li] = fine_val;
            }

            // Update active mask with goal cells.
            let goal_spatial = goal_any_2d(&goals[li]);
            let am = match active_masks[li].take() {
                None => goal_spatial,
                Some(prev) => array2_or(&prev, &goal_spatial),
            };
            active_masks[li] = Some(am);

            if !any_true(active_masks[li].as_ref().unwrap()) {
                break;
            }

            // Budget cap for this level.
            let cap = if li == 0 {
                remaining.min(self.refine_sweeps)
            } else {
                remaining.min(self.coarse_sweeps.max(1))
            };

            if cap == 0 {
                break;
            }

            // Scale transition model for this level (coarser = divide displacements).
            let scale = 1u32 << li;
            let trans_model = scale_transition_model(&ctx.transitions, scale);

            let (mx_disp, my_disp, _) = trans_model.max_displacement();
            let candidate_mask = dilate_spatial_mask(
                active_masks[li].as_ref().unwrap(),
                mx_disp as u32,
                my_disp as u32,
                level_mx[li],
                level_my[li],
            );

            let (done, changed, level_delta, descend_mask) = run_masked_sweeps(
                &mut values[li],
                &penalties[li],
                &goals[li],
                &trans_model,
                level_mx[li],
                level_my[li],
                self.threshold,
                cap,
                &candidate_mask,
                self.descend_tau,
            );

            sweeps_total += done;
            remaining = remaining.saturating_sub(done);
            let visited = candidate_mask.iter().filter(|&&v| v).count() as u64
                * N_THETA as u64
                * done as u64;
            visited_total += visited;
            final_delta = level_delta;

            per_level[li] = PyramidLevelStat {
                level: (li + 1) as u32,
                map_x: level_mx[li],
                map_y: level_my[li],
                scale,
                sweeps: done,
                changed_states: changed,
                visited_states: visited,
                final_delta: level_delta,
            };

            // Prepare child active mask for next (finer) level.
            if li > 0 {
                let goal_sp = goal_any_2d(&goals[li]);
                let combined = array2_or(&descend_mask, &goal_sp);
                let child_mask = prolongate_active_mask(
                    &combined,
                    level_mx[li - 1],
                    level_my[li - 1],
                );
                active_masks[li - 1] = Some(match active_masks[li - 1].take() {
                    None => child_mask,
                    Some(prev) => array2_or(&prev, &child_mask),
                });
            }

            if remaining == 0 {
                break;
            }
        }

        // Write back finest level value to context.
        ctx.value = values.swap_remove(0);

        // Pin goal cells.
        let map_x_us = map_x as usize;
        let map_y_us = map_y as usize;
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
            iters_or_sweeps: sweeps_total,
            updates: visited_total,
            final_delta,
            converged: final_delta <= self.threshold,
            extra: Some(SolveExtra::PyramidPerLevel(per_level)),
        }
    }
}

// ---------------------------------------------------------------------------
// Pyramid helpers
// ---------------------------------------------------------------------------

/// Coarsen a level by 2×2 max-pooling of values, min of free penalties, OR of goals.
///
/// For value: take min across all children (including obstacle-value-MV cells is harmless
/// since MV is the maximum — it never beats a real value). For penalty: min over free cells
/// only (if all children are obstacles the coarse cell stays OBSTACLE). For goal: OR.
fn coarsen_level(
    value: &Array3<Value>,
    penalty: &Array2<Penalty>,
    goal: &Array3<bool>,
    map_x: u32,
    map_y: u32,
) -> (Array3<Value>, Array2<Penalty>, Array3<bool>) {
    let cx = map_x.div_ceil(2) as usize;
    let cy = map_y.div_ceil(2) as usize;
    let map_y_us = map_y as usize;
    let map_x_us = map_x as usize;

    let mut coarse_value = Array3::<Value>::from_elem((cy, cx, N_THETA), MAX_VALUE);
    let mut coarse_penalty = Array2::<Penalty>::from_elem((cy, cx), PENALTY_OBSTACLE);
    let mut coarse_goal = Array3::<bool>::from_elem((cy, cx, N_THETA), false);

    for ciy in 0..cy {
        let y0 = ciy * 2;
        let y1 = (y0 + 2).min(map_y_us);
        for cix in 0..cx {
            let x0 = cix * 2;
            let x1 = (x0 + 2).min(map_x_us);

            // Penalty: min over free (non-obstacle) children.
            let mut best_pen: Option<Penalty> = None;
            for iy in y0..y1 {
                for ix in x0..x1 {
                    let p = penalty[[iy, ix]];
                    if p != PENALTY_OBSTACLE {
                        best_pen = Some(match best_pen {
                            None => p,
                            Some(prev) => prev.min(p),
                        });
                    }
                }
            }
            if let Some(p) = best_pen {
                coarse_penalty[[ciy, cix]] = p;
            }

            for it in 0..N_THETA {
                // Goal: OR over children.
                let mut any_goal = false;
                for iy in y0..y1 {
                    for ix in x0..x1 {
                        if goal[[iy, ix, it]] {
                            any_goal = true;
                            break;
                        }
                    }
                    if any_goal {
                        break;
                    }
                }
                coarse_goal[[ciy, cix, it]] = any_goal;

                if any_goal {
                    coarse_value[[ciy, cix, it]] = 0;
                } else if coarse_penalty[[ciy, cix]] != PENALTY_OBSTACLE {
                    // Value: min over all children (MV for obstacle children, real value otherwise).
                    let mut best_val = MAX_VALUE;
                    for iy in y0..y1 {
                        for ix in x0..x1 {
                            let v = value[[iy, ix, it]];
                            if v < best_val {
                                best_val = v;
                            }
                        }
                    }
                    coarse_value[[ciy, cix, it]] = best_val;
                }
            }
        }
    }

    (coarse_value, coarse_penalty, coarse_goal)
}

/// Prolongate coarse values to fine grid: each fine cell gets the value of its parent
/// coarse cell. Obstacle fine cells keep MAX_VALUE. Goal fine cells are pinned to 0.
fn prolongate_level(
    coarse_value: &Array3<Value>,
    fine_penalty: &Array2<Penalty>,
    fine_goal: &Array3<bool>,
    map_x: u32,
    map_y: u32,
) -> Array3<Value> {
    let map_x_us = map_x as usize;
    let map_y_us = map_y as usize;
    let mut fine_value = Array3::<Value>::from_elem((map_y_us, map_x_us, N_THETA), MAX_VALUE);

    for iy in 0..map_y_us {
        let cy = iy / 2;
        for ix in 0..map_x_us {
            if fine_penalty[[iy, ix]] == PENALTY_OBSTACLE {
                continue;
            }
            let cx = ix / 2;
            for it in 0..N_THETA {
                fine_value[[iy, ix, it]] = coarse_value[[cy, cx, it]];
            }
        }
    }
    // Pin goals.
    for ((iy, ix, it), &g) in fine_goal.indexed_iter() {
        if g {
            fine_value[[iy, ix, it]] = 0;
        }
    }

    fine_value
}

/// Prolongate an active spatial mask from parent (coarser) to child (finer) grid.
/// Each true parent cell activates up to a 2×2 block of child cells.
fn prolongate_active_mask(
    parent_mask: &Array2<bool>,
    child_x: u32,
    child_y: u32,
) -> Array2<bool> {
    let child_x_us = child_x as usize;
    let child_y_us = child_y as usize;
    let parent_y = parent_mask.shape()[0];
    let parent_x = parent_mask.shape()[1];

    let mut child_mask = Array2::<bool>::from_elem((child_y_us, child_x_us), false);

    for py in 0..parent_y {
        let y0 = py * 2;
        let y1 = (y0 + 2).min(child_y_us);
        for px in 0..parent_x {
            if !parent_mask[[py, px]] {
                continue;
            }
            let x0 = px * 2;
            let x1 = (x0 + 2).min(child_x_us);
            for iy in y0..y1 {
                for ix in x0..x1 {
                    child_mask[[iy, ix]] = true;
                }
            }
        }
    }

    child_mask
}

/// Dilate a spatial mask by (dx, dy) cells in each axis.
fn dilate_spatial_mask(
    mask: &Array2<bool>,
    dx: u32,
    dy: u32,
    map_x: u32,
    map_y: u32,
) -> Array2<bool> {
    let map_x_us = map_x as usize;
    let map_y_us = map_y as usize;
    let dx = dx as usize;
    let dy = dy as usize;
    let mut out = Array2::<bool>::from_elem((map_y_us, map_x_us), false);

    for ((iy, ix), &v) in mask.indexed_iter() {
        if !v {
            continue;
        }
        let x0 = ix.saturating_sub(dx);
        let x1 = (ix + dx).min(map_x_us - 1);
        let y0 = iy.saturating_sub(dy);
        let y1 = (iy + dy).min(map_y_us - 1);
        for sy in y0..=y1 {
            for sx in x0..=x1 {
                out[[sy, sx]] = true;
            }
        }
    }

    out
}

/// Scale a transition model for a coarser level: divide each displacement by `scale`,
/// preserving sign and rounding up the magnitude (0 stays 0).
///
/// WHY: at level k we treat 1 coarse cell = 2^(k-1) fine cells. The displacement
/// in coarse-cell units is ceil(|d_fine| / scale) * sign(d_fine).
fn scale_transition_model(base: &TransitionModel, scale: u32) -> TransitionModel {
    if scale <= 1 {
        return base.clone();
    }
    let mut m = base.clone();
    for a in 0..N_ACTIONS {
        for it in 0..N_THETA {
            let n_out = m.n_outcomes[a][it] as usize;
            for k in 0..n_out {
                m.dix[a][it][k] = coarse_delta(m.dix[a][it][k], scale);
                m.diy[a][it][k] = coarse_delta(m.diy[a][it][k], scale);
                // dit is not scaled — theta resolution is unchanged across pyramid levels.
            }
        }
    }
    m
}

/// Scale a single signed displacement offset by `scale`.
fn coarse_delta(d: i8, scale: u32) -> i8 {
    if d == 0 {
        return 0;
    }
    let abs_d = d.unsigned_abs() as u32;
    let scaled = abs_d.div_ceil(scale).min(i8::MAX as u32) as i8;
    if d < 0 { -scaled } else { scaled }
}

/// Run up to `max_sweeps` masked sweeps. Updates are applied unconditionally
/// (no `if new < old` guard) — this matches MATLAB `run_masked_sweeps`.
///
/// Returns `(done_sweeps, changed_states, final_delta, descend_mask)` where
/// `descend_mask` marks cells that changed by more than `descend_tau`.
#[allow(clippy::too_many_arguments)]
fn run_masked_sweeps(
    value: &mut Array3<Value>,
    penalty: &Array2<Penalty>,
    goal: &Array3<bool>,
    trans: &TransitionModel,
    map_x: u32,
    map_y: u32,
    threshold: Value,
    max_sweeps: u32,
    candidate_mask: &Array2<bool>,
    descend_tau: Value,
) -> (u32, u64, Value, Array2<bool>) {
    let map_x_us = map_x as usize;
    let map_y_us = map_y as usize;

    let mut done: u32 = 0;
    let mut changed_states: u64 = 0;
    let mut final_delta: Value = 0;
    let mut changed_mask = Array2::<bool>::from_elem((map_y_us, map_x_us), false);

    for _sweep in 0..max_sweeps {
        let mut max_delta: Value = 0;
        let mut changed_this: u64 = 0;

        for iy in 0..map_y_us {
            for ix in 0..map_x_us {
                if !candidate_mask[[iy, ix]] || penalty[[iy, ix]] == PENALTY_OBSTACLE {
                    continue;
                }

                let mut cell_changed = false;
                let mut cell_max_delta: Value = 0;

                for it in 0..N_THETA {
                    if goal[[iy, ix, it]] {
                        value[[iy, ix, it]] = 0;
                        continue;
                    }
                    let old_val = value[[iy, ix, it]];
                    let new_val = bellman_backup(
                        value, penalty, trans,
                        ix as u32, iy as u32, it as u32,
                        map_x, map_y,
                    );
                    // Unconditional update — matches MATLAB run_masked_sweeps.
                    value[[iy, ix, it]] = new_val;

                    let d = new_val.abs_diff(old_val);
                    if d > 0 {
                        cell_changed = true;
                        changed_this += 1;
                        if d > cell_max_delta {
                            cell_max_delta = d;
                        }
                        if d > max_delta {
                            max_delta = d;
                        }
                    }
                }

                if cell_changed && cell_max_delta > descend_tau {
                    changed_mask[[iy, ix]] = true;
                }
            }
        }

        done += 1;
        changed_states += changed_this;
        final_delta = max_delta;

        if max_delta <= threshold {
            break;
        }
    }

    (done, changed_states, final_delta, changed_mask)
}

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

fn goal_any_2d(goal: &Array3<bool>) -> Array2<bool> {
    let (my, mx, _) = (goal.shape()[0], goal.shape()[1], goal.shape()[2]);
    let mut out = Array2::<bool>::from_elem((my, mx), false);
    for ((iy, ix, _it), &g) in goal.indexed_iter() {
        if g {
            out[[iy, ix]] = true;
        }
    }
    out
}

fn any_true(arr: &Array2<bool>) -> bool {
    arr.iter().any(|&v| v)
}

fn array2_or(a: &Array2<bool>, b: &Array2<bool>) -> Array2<bool> {
    let mut out = a.clone();
    for (ov, &bv) in out.iter_mut().zip(b.iter()) {
        *ov = *ov || bv;
    }
    out
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::{Budget, SolveExtra};
    use crate::frontier::test_helpers::{empty_5x5_ctx, empty_3x3_ctx};
    use crate::reference::Reference;

    #[test]
    fn parity_pyramid_empty_5x5() {
        // PyramidSweep with threshold=0, min_size=4 builds a 2-level pyramid on the 5x5 map.
        // The coarse level (3x3) covers the entire map via dilation, so prolongation seeds
        // all fine cells. This should converge to the same value as Reference.
        let mut ctx_ref = empty_5x5_ctx();
        let mut ctx_pyr = ctx_ref.clone_value();

        let ref_stats = Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(100));
        assert!(ref_stats.converged, "Reference must converge for parity test to be valid");

        PyramidSweep {
            threshold: 0,
            min_size: 4,
            coarse_sweeps: 10,
            refine_sweeps: 50,
            descend_tau: 0,
        }.run(&mut ctx_pyr, Budget::Sweeps(100));

        // Spec §7.2: exact variants must match Reference on every cell. With
        // descend_tau=0 the finest level sweeps every reachable cell from the
        // original (pessimistic) seed, converging from above exactly like
        // Reference. Unreachable cells stay MAX_VALUE in both.
        assert_eq!(
            ctx_pyr.value, ctx_ref.value,
            "PyramidSweep must be bit-exact with Reference on empty 5x5"
        );
    }

    #[test]
    fn pyramid_min_size_larger_than_map_skips_coarsening() {
        // When min_size > map dimensions no pyramid is built. Only level 1 runs,
        // with its active mask seeded from goal cells only. The candidate_mask is
        // the dilation of goal spatial around (2,2) with dx=dy=1 — covering the 3×3
        // neighborhood. On 3×3 map this covers everything; on 5×5 corners are outside.
        // We verify convergence and that goal cell is pinned.
        let mut ctx_ref = empty_3x3_ctx();
        let mut ctx_pyr = ctx_ref.clone_value();

        Reference { threshold: 0 }.run(&mut ctx_ref, Budget::Sweeps(100));

        PyramidSweep {
            threshold: 0,
            min_size: 100,
            coarse_sweeps: 10,
            refine_sweeps: 100,
            descend_tau: 0,
        }.run(&mut ctx_pyr, Budget::Sweeps(100));

        // Goal must be pinned.
        assert_eq!(ctx_pyr.value[[1, 1, 0]], 0, "goal cell must be pinned to 0");

        // On 3x3 with 4-dir actions, dilation by 1 from goal at (1,1) covers all 9 cells.
        // So this should match Reference bit-exactly.
        assert_eq!(ctx_ref.value, ctx_pyr.value,
            "on 3x3 map, single-level pyramid covers all cells via dilation");
    }

    #[test]
    fn pyramid_per_level_stats_provided() {
        let mut ctx = empty_5x5_ctx();
        let stats = PyramidSweep {
            threshold: 0,
            min_size: 2,
            coarse_sweeps: 10,
            refine_sweeps: 30,
            descend_tau: 0,
        }.run(&mut ctx, Budget::Sweeps(50));

        match stats.extra {
            Some(SolveExtra::PyramidPerLevel(levels)) => {
                assert!(!levels.is_empty(), "per-level stats must be non-empty");
            }
            _ => panic!("expected PyramidPerLevel stats"),
        }
    }

    #[test]
    fn pyramid_terminates_within_budget() {
        let mut ctx = empty_5x5_ctx();
        let stats = PyramidSweep {
            threshold: 0,
            min_size: 4,
            coarse_sweeps: 5,
            refine_sweeps: 20,
            descend_tau: 0,
        }.run(&mut ctx, Budget::Sweeps(50));
        assert!(stats.iters_or_sweeps <= 50);
    }

    #[test]
    fn pyramid_goal_pinned_after_run() {
        let mut ctx = empty_5x5_ctx();
        PyramidSweep {
            threshold: 0,
            min_size: 4,
            coarse_sweeps: 10,
            refine_sweeps: 50,
            descend_tau: 0,
        }.run(&mut ctx, Budget::Sweeps(100));
        // Goal at (iy=2, ix=2, it=0) for empty_5x5_ctx.
        assert_eq!(ctx.value[[2, 2, 0]], 0, "goal cell must be pinned to 0 after pyramid run");
    }

    #[test]
    fn coarsen_level_smoke() {
        // 4x4 free map, all values=10, one goal at (0,0,0).
        let value = Array3::<Value>::from_elem((4, 4, N_THETA), 10);
        let penalty = Array2::<Penalty>::zeros((4, 4));
        let mut goal = Array3::<bool>::from_elem((4, 4, N_THETA), false);
        goal[[0, 0, 0]] = true;

        let (cv, cp, cg) = coarsen_level(&value, &penalty, &goal, 4, 4);

        // Coarse size: ceil(4/2)=2 x ceil(4/2)=2
        assert_eq!(cv.shape(), &[2, 2, N_THETA]);
        assert_eq!(cp.shape(), &[2, 2]);
        // Coarse (0,0) has goal in theta=0, so value=0.
        assert_eq!(cv[[0, 0, 0]], 0);
        assert!(cg[[0, 0, 0]]);
        // Other coarse cells: value = min(10,10,10,10) = 10.
        assert_eq!(cv[[0, 1, 0]], 10);
        // Penalty: all free.
        assert_eq!(cp[[0, 0]], 0);
    }

    #[test]
    fn prolongate_level_smoke() {
        // 2x2 coarse with uniform value=5, fine=4x4.
        let coarse_val = Array3::<Value>::from_elem((2, 2, N_THETA), 5);
        let fine_penalty = Array2::<Penalty>::zeros((4, 4));
        let fine_goal = Array3::<bool>::from_elem((4, 4, N_THETA), false);

        let fv = prolongate_level(&coarse_val, &fine_penalty, &fine_goal, 4, 4);
        // Every fine cell maps to its 2x2 parent coarse cell.
        for iy in 0..4usize {
            for ix in 0..4usize {
                assert_eq!(fv[[iy, ix, 0]], 5, "fine ({iy},{ix}) should be 5");
            }
        }
    }

    #[test]
    fn scale_transition_model_smoke() {
        use vi_core::{TransitionModel, PROB_BASE};
        let mut base = TransitionModel::default();
        base.n_outcomes[0][0] = 1;
        base.dix[0][0][0] = 3;
        base.diy[0][0][0] = -2;
        base.prob[0][0][0] = PROB_BASE;

        let scaled = scale_transition_model(&base, 2);
        // ceil(3/2) = 2, ceil(2/2) = 1 → dix=2, diy=-1
        assert_eq!(scaled.dix[0][0][0], 2);
        assert_eq!(scaled.diy[0][0][0], -1);

        // scale=1: no change
        let unscaled = scale_transition_model(&base, 1);
        assert_eq!(unscaled.dix[0][0][0], 3);

        // Zero displacement stays zero.
        let mut base2 = TransitionModel::default();
        base2.n_outcomes[0][0] = 1;
        base2.dix[0][0][0] = 0;
        base2.diy[0][0][0] = 0;
        base2.prob[0][0][0] = PROB_BASE;
        let scaled2 = scale_transition_model(&base2, 4);
        assert_eq!(scaled2.dix[0][0][0], 0);
        assert_eq!(scaled2.diy[0][0][0], 0);
    }
}
