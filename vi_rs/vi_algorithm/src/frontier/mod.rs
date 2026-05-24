//! Frontier-tracking VI variants. All variants in this module are bit-exact
//! with the Reference solver on convergence (residual = 0).
//!
//! See `docs/superpowers/specs/2026-05-22-vi-rs-algorithm-port-design.md` §4.2, §4.8.

pub mod f2d;
pub mod f3d;
pub mod stack;

pub use f2d::Frontier2D;
pub use f3d::Frontier3D;
pub use stack::FrontierStack;

use ndarray::{Array2, Array3};
use vi_core::{MAX_VALUE, Penalty, Value, PENALTY_OBSTACLE};

use crate::bitboard::{Bitboard2D, Bitboard3D};

/// Build a 2D bitboard from `penalty != PENALTY_OBSTACLE`.
pub(crate) fn build_passable_bb_2d(penalty: &Array2<Penalty>) -> Bitboard2D {
    let map_y = penalty.shape()[0];
    let map_x = penalty.shape()[1];
    let mut bb = Bitboard2D::new(map_x as u32, map_y as u32);
    for iy in 0..map_y {
        for ix in 0..map_x {
            if penalty[[iy, ix]] != PENALTY_OBSTACLE {
                bb.set(ix as u32, iy as u32);
            }
        }
    }
    bb
}

/// Build a 3D bitboard with the same 2D layer (passable_2d) repeated across all theta.
pub(crate) fn build_passable_bb_3d(passable_2d: &Bitboard2D, n_theta: u32) -> Bitboard3D {
    let map_x = passable_2d.map_x();
    let map_y = passable_2d.map_y();
    let mut bb = Bitboard3D::new(map_x, map_y, n_theta);
    for (ix, iy) in passable_2d.enumerate() {
        for it in 0..n_theta {
            bb.set(ix, iy, it);
        }
    }
    bb
}

/// Pin goal cells in `value` to 0.
pub(crate) fn pin_goals(value: &mut Array3<Value>, goal_mask: &Array3<bool>) {
    for ((iy, ix, it), &is_goal) in goal_mask.indexed_iter() {
        if is_goal {
            value[[iy, ix, it]] = 0;
        }
    }
}

/// Build a 3D bitboard from `value < MAX_VALUE` (initial frontier seed).
pub(crate) fn build_value_seed_3d(value: &Array3<Value>) -> Bitboard3D {
    let (my, mx, nt) = (value.shape()[0], value.shape()[1], value.shape()[2]);
    let mut bb = Bitboard3D::new(mx as u32, my as u32, nt as u32);
    for ((iy, ix, it), &v) in value.indexed_iter() {
        if v < MAX_VALUE {
            bb.set(ix as u32, iy as u32, it as u32);
        }
    }
    bb
}

/// Build a 2D bitboard from `any(value[.., .., it] < MAX_VALUE over it)`.
/// Mirrors MATLAB `any(value_table < MV, 3)`.
pub(crate) fn build_value_seed_2d(value: &Array3<Value>) -> Bitboard2D {
    let (my, mx, _nt) = (value.shape()[0], value.shape()[1], value.shape()[2]);
    let mut bb = Bitboard2D::new(mx as u32, my as u32);
    for ((iy, ix, _it), &v) in value.indexed_iter() {
        if v < MAX_VALUE {
            bb.set(ix as u32, iy as u32);
        }
    }
    bb
}

/// Extract the maximum-iteration count from a `Budget`.
/// Frontier solvers treat `Sweeps(n)` and `Iterations(n)` identically.
pub(crate) fn max_iters(budget: crate::context::Budget) -> u32 {
    match budget {
        crate::context::Budget::Sweeps(n) => n,
        crate::context::Budget::Iterations(n) => n,
    }
}

// ---------------------------------------------------------------------------
// Shared test helpers (available to all three solver test modules)
// ---------------------------------------------------------------------------

#[cfg(test)]
pub(crate) mod test_helpers {
    use ndarray::{Array2, Array3};
    use vi_core::{
        MAX_VALUE, N_THETA, PENALTY_OBSTACLE, PROB_BASE, Penalty, TransitionModel, Value,
    };
    use crate::context::{MapDims, VIContext};

    /// A simple deterministic 4-direction transition model.
    /// Actions: 0=+x, 1=-x, 2=+y, 3=-y (each with prob=PROB_BASE, dit=0).
    /// Actions 4 and 5 have n_out=0 (undefined / no-op).
    pub(crate) fn deterministic_4dir_trans() -> TransitionModel {
        let mut trans = TransitionModel::default();
        // (dx, dy) for each action
        let dirs: [(i8, i8); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];
        for it in 0..N_THETA {
            for (a, &(dx, dy)) in dirs.iter().enumerate() {
                trans.n_outcomes[a][it] = 1;
                trans.dix[a][it][0] = dx;
                trans.diy[a][it][0] = dy;
                trans.dit[a][it][0] = 0;
                trans.prob[a][it][0] = PROB_BASE;
            }
            // actions 4 and 5: n_out stays 0
        }
        trans
    }

    /// Build a free (all-passable) context with one goal at `(goal_x, goal_y, 0)`.
    pub(crate) fn empty_ctx(map_x: u32, map_y: u32, goal_x: u32, goal_y: u32) -> VIContext {
        let value =
            Array3::<Value>::from_elem((map_y as usize, map_x as usize, N_THETA), MAX_VALUE);
        let penalty = Array2::<Penalty>::zeros((map_y as usize, map_x as usize));
        let mut goal_mask =
            Array3::<bool>::from_elem((map_y as usize, map_x as usize, N_THETA), false);
        goal_mask[[goal_y as usize, goal_x as usize, 0]] = true;
        VIContext {
            dims: MapDims { map_x, map_y },
            value,
            penalty,
            goal_mask,
            transitions: deterministic_4dir_trans(),
        }
    }

    pub(crate) fn empty_3x3_ctx() -> VIContext {
        empty_ctx(3, 3, 1, 1)
    }

    pub(crate) fn empty_5x5_ctx() -> VIContext {
        empty_ctx(5, 5, 2, 2)
    }

    /// 3x3 with obstacle at (iy=0, ix=0), goal at (iy=1, ix=1, it=0).
    pub(crate) fn obstacle_3x3_ctx() -> VIContext {
        let mut ctx = empty_3x3_ctx();
        ctx.penalty[[0, 0]] = PENALTY_OBSTACLE;
        ctx
    }

    /// 3x3 with three obstacle cells adjacent to the goal, leaving only one
    /// passable exit. Goal at (iy=1, ix=1, it=0).
    pub(crate) fn sentinel_3x3_ctx() -> VIContext {
        let mut ctx = empty_3x3_ctx();
        // Block three sides of the goal: above, below, left.
        ctx.penalty[[0, 1]] = PENALTY_OBSTACLE; // iy=0, ix=1 (above goal)
        ctx.penalty[[2, 1]] = PENALTY_OBSTACLE; // iy=2, ix=1 (below goal)
        ctx.penalty[[1, 0]] = PENALTY_OBSTACLE; // iy=1, ix=0 (left of goal)
        ctx
    }
}
