//! Compute argmin-action table from a (converged) value table.
//!
//! Mirrors `vi_matlab/src/cpu/reference/compute_action_table_reference.m`.

use ndarray::{Array2, Array3};
use vi_core::{
    cost_of, ActionIdx, Penalty, TransitionModel, Value,
    MAX_VALUE, N_ACTIONS, N_THETA, PENALTY_OBSTACLE, PROB_BASE,
};

pub fn compute_action_table_reference(
    value: &Array3<Value>,
    penalty: &Array2<Penalty>,
    goal_mask: &Array3<bool>,
    trans: &TransitionModel,
    map_x: u32,
    map_y: u32,
) -> Array3<ActionIdx> {
    let mut action_table = Array3::<ActionIdx>::zeros(
        (map_y as usize, map_x as usize, N_THETA),
    );

    for iy in 0..map_y as usize {
        for ix in 0..map_x as usize {
            for it in 0..N_THETA {
                if goal_mask[[iy, ix, it]] || penalty[[iy, ix]] == PENALTY_OBSTACLE {
                    action_table[[iy, ix, it]] = 0;
                    continue;
                }
                let mut best_cost: Value = MAX_VALUE;
                let mut best_act: ActionIdx = 0;
                for a in 0..N_ACTIONS {
                    let c = action_cost(
                        value, penalty, trans,
                        ix as u32, iy as u32, it as u32,
                        map_x, map_y, a,
                    );
                    if c < best_cost {
                        best_cost = c;
                        best_act = a as ActionIdx;
                    }
                }
                action_table[[iy, ix, it]] = best_act;
            }
        }
    }
    action_table
}

#[allow(clippy::too_many_arguments)]
fn action_cost(
    value: &Array3<Value>,
    penalty: &Array2<Penalty>,
    trans: &TransitionModel,
    ix: u32,
    iy: u32,
    it: u32,
    map_x: u32,
    map_y: u32,
    a: usize,
) -> Value {
    let it_us = it as usize;
    let n_out = trans.n_outcomes[a][it_us] as usize;
    // WHY: n_out == 0 means this (action, theta) pair has no defined transition;
    // treat as undefined (MAX_VALUE) consistent with bellman_backup's convention.
    if n_out == 0 {
        return MAX_VALUE;
    }
    let mut accum: u64 = 0;
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
        if nx < 0 || nx >= map_x as i32 || ny < 0 || ny >= map_y as i32 {
            return MAX_VALUE;
        }
        let step_cost = cost_of(
            value[[ny as usize, nx as usize, nt as usize]],
            penalty[[ny as usize, nx as usize]],
        );
        if step_cost == MAX_VALUE {
            return MAX_VALUE;
        }
        accum += step_cost as u64 * trans.prob[a][it_us][k] as u64;
    }
    let c = accum / PROB_BASE as u64;
    if c >= MAX_VALUE as u64 { MAX_VALUE - 1 } else { c as Value }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vi_core::PENALTY_OBSTACLE;
    use crate::reference::tests::empty_3x3_ctx;
    use crate::context::{Budget, SolveExtra, Solver};
    use crate::reference::Reference;

    #[test]
    fn obstacle_cell_action_is_zero() {
        let mut ctx = empty_3x3_ctx();
        ctx.penalty[[0, 0]] = PENALTY_OBSTACLE;
        let at = compute_action_table_reference(
            &ctx.value, &ctx.penalty, &ctx.goal_mask, &ctx.transitions,
            ctx.dims.map_x, ctx.dims.map_y,
        );
        assert_eq!(at[[0, 0, 0]], 0);
    }

    #[test]
    fn goal_cell_action_is_zero() {
        let ctx = empty_3x3_ctx();
        // goal_mask[[1, 1, 0]] == true in empty_3x3_ctx
        let at = compute_action_table_reference(
            &ctx.value, &ctx.penalty, &ctx.goal_mask, &ctx.transitions,
            ctx.dims.map_x, ctx.dims.map_y,
        );
        assert_eq!(at[[1, 1, 0]], 0);
    }

    #[test]
    fn picks_action_with_minimum_cost() {
        // Run Reference to convergence, then verify the action table points
        // the cell at (iy=1, ix=0, it=0) — left of goal at (iy=1, ix=1) —
        // toward action 0 (dix=+1, moves right toward goal).
        let mut ctx = empty_3x3_ctx();
        let solver = Reference { threshold: 0 };
        let stats = solver.run(&mut ctx, Budget::Sweeps(50));
        assert!(stats.converged);

        let at = match stats.extra.unwrap() {
            SolveExtra::ActionTable(t) => t,
            _ => panic!("expected ActionTable"),
        };

        // Cell (iy=1, ix=0, it=0): left of goal (iy=1, ix=1). action 0 = dix=+1 (right).
        assert_eq!(at[[1, 0, 0]], 0, "left-of-goal cell should pick action 0 (dix=+1)");
    }
}
