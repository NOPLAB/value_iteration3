//! Per-cell optimal-action lookup. Mirrors the inner argmin of
//! `reference::action_table::compute_action_table_reference` but for a
//! single (ix, iy, it). Used by the vi_ros2 cmd_vel timer.

use ndarray::{Array2, Array3};
use vi_core::{
    cost_of, ActionIdx, Penalty, TransitionModel, Value,
    MAX_VALUE, N_ACTIONS, N_THETA, PENALTY_OBSTACLE, PROB_BASE,
};

use crate::context::VIContext;

/// Returns the action id that minimises expected cost at cell (ix, iy, it).
/// Returns 0 if the cell is an obstacle, on the goal mask, or fully blocked
/// (every action leads to MAX_VALUE).
pub fn optimal_action_at(ctx: &VIContext, ix: i32, iy: i32, it: usize) -> ActionIdx {
    let map_x = ctx.dims.map_x;
    let map_y = ctx.dims.map_y;
    if ix < 0 || iy < 0 || ix >= map_x as i32 || iy >= map_y as i32 {
        return 0;
    }
    if it >= vi_core::N_THETA {
        return 0;
    }
    let ix_u = ix as u32;
    let iy_u = iy as u32;
    if ctx.penalty[[iy as usize, ix as usize]] == PENALTY_OBSTACLE {
        return 0;
    }
    if ctx.goal_mask[[iy as usize, ix as usize, it]] {
        return 0;
    }
    let mut best_cost: Value = MAX_VALUE;
    let mut best_act: ActionIdx = 0;
    for a in 0..N_ACTIONS {
        let c = action_cost_single(
            &ctx.value, &ctx.penalty, &ctx.transitions,
            ix_u, iy_u, it as u32, map_x, map_y, a,
        );
        if c < best_cost {
            best_cost = c;
            best_act = a as ActionIdx;
        }
    }
    best_act
}

// NOTE: near-duplicate of action_table::action_cost and bellman_backup's
// inner loop; deduplication deferred (YAGNI). Update all three in lockstep.
#[allow(clippy::too_many_arguments)]
fn action_cost_single(
    value: &Array3<Value>,
    penalty: &Array2<Penalty>,
    trans: &TransitionModel,
    ix: u32, iy: u32, it: u32,
    map_x: u32, map_y: u32,
    a: usize,
) -> Value {
    let it_us = it as usize;
    let n_out = trans.n_outcomes[a][it_us] as usize;
    if n_out == 0 { return MAX_VALUE; }
    let mut accum: u64 = 0;
    for k in 0..n_out {
        let dix = trans.dix[a][it_us][k] as i32;
        let diy = trans.diy[a][it_us][k] as i32;
        let dit = trans.dit[a][it_us][k] as i32;
        let nx = ix as i32 + dix;
        let ny = iy as i32 + diy;
        let mut nt = it as i32 + dit;
        if nt < 0 { nt += N_THETA as i32; }
        else if nt >= N_THETA as i32 { nt -= N_THETA as i32; }
        if nx < 0 || nx >= map_x as i32 || ny < 0 || ny >= map_y as i32 {
            return MAX_VALUE;
        }
        let step = cost_of(
            value[[ny as usize, nx as usize, nt as usize]],
            penalty[[ny as usize, nx as usize]],
        );
        if step == MAX_VALUE { return MAX_VALUE; }
        accum += step as u64 * trans.prob[a][it_us][k] as u64;
    }
    let c = accum / PROB_BASE as u64;
    if c >= MAX_VALUE as u64 { MAX_VALUE - 1 } else { c as Value }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::{MapDims, VIContext};
    use vi_core::params::N_THETA;
    use vi_fixtures::{generate_map, generate_transitions, MapType, TransitionMode};

    fn small_ctx() -> VIContext {
        let m = generate_map(8, 8, MapType::Empty);
        let trans = generate_transitions(TransitionMode::Full { xy_resolution: 0.05 });
        VIContext {
            dims: MapDims { map_x: 8, map_y: 8 },
            value: m.value,
            penalty: m.penalty,
            goal_mask: m.goal_mask,
            transitions: trans.unpack(),
        }
    }

    #[test]
    fn obstacle_cell_returns_zero() {
        let mut ctx = small_ctx();
        ctx.penalty[[2, 3]] = vi_core::PENALTY_OBSTACLE;
        assert_eq!(optimal_action_at(&ctx, 3, 2, 0), 0);
    }

    #[test]
    fn out_of_bounds_returns_zero() {
        let ctx = small_ctx();
        assert_eq!(optimal_action_at(&ctx, -1, 0, 0), 0);
        assert_eq!(optimal_action_at(&ctx, 0, -1, 0), 0);
        assert_eq!(optimal_action_at(&ctx, 100, 0, 0), 0);
        assert_eq!(optimal_action_at(&ctx, 0, 0, N_THETA), 0);
    }

    #[test]
    fn goal_cell_returns_zero() {
        let ctx = small_ctx();
        let gx = ctx.dims.map_x as i32 / 2;
        let gy = ctx.dims.map_y as i32 / 2;
        let mut ctx_g = ctx;
        ctx_g.goal_mask[[gy as usize, gx as usize, 0]] = true;
        assert_eq!(optimal_action_at(&ctx_g, gx, gy, 0), 0);
    }

    #[test]
    fn agrees_with_reference_action_table() {
        use crate::reference::Reference;
        use crate::context::{Budget, Solver, SolveExtra};
        let mut ctx = small_ctx();
        let gx = ctx.dims.map_x as usize / 2;
        let gy = ctx.dims.map_y as usize / 2;
        for it in 0..N_THETA {
            ctx.goal_mask[[gy, gx, it]] = true;
            ctx.value[[gy, gx, it]] = 0;
        }
        let stats = Reference { threshold: 0 }.run(&mut ctx, Budget::Sweeps(50));
        let at = match stats.extra {
            Some(SolveExtra::ActionTable(t)) => t,
            _ => panic!("expected ActionTable"),
        };
        for iy in 0..ctx.dims.map_y as usize {
            for ix in 0..ctx.dims.map_x as usize {
                for it in 0..N_THETA {
                    let expected = at[[iy, ix, it]];
                    let actual = optimal_action_at(&ctx, ix as i32, iy as i32, it);
                    assert_eq!(actual, expected,
                        "divergence at iy={iy}, ix={ix}, it={it}");
                }
            }
        }
    }

    #[test]
    fn fully_blocked_cell_returns_zero() {
        // Free cell at (1, 1) surrounded by obstacles on every chessboard
        // neighbour. Every action's neighbour transition is OOB-or-obstacle,
        // so optimal_action_at should fall back to action 0.
        let mut ctx = small_ctx();
        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                if dx == 0 && dy == 0 { continue; }
                let nx = (1 + dx) as usize;
                let ny = (1 + dy) as usize;
                if nx < 8 && ny < 8 {
                    ctx.penalty[[ny, nx]] = vi_core::PENALTY_OBSTACLE;
                }
            }
        }
        assert_eq!(optimal_action_at(&ctx, 1, 1, 0), 0);
    }
}
