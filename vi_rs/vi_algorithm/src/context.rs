use ndarray::{Array2, Array3};
use vi_core::{ActionIdx, Penalty, TransitionModel, Value};

#[derive(Clone, Copy, Debug)]
pub struct MapDims {
    pub map_x: u32,
    pub map_y: u32,
}

pub struct VIContext {
    pub dims: MapDims,
    pub value: Array3<Value>,
    pub penalty: Array2<Penalty>,
    pub goal_mask: Array3<bool>,
    pub transitions: TransitionModel,
}

impl VIContext {
    /// Deep-clones the full context, producing a completely independent
    /// working state. Used by benchmarks to run multiple solvers from the
    /// same initial state without aliasing `value`, `penalty`, `goal_mask`,
    /// or `transitions`.
    pub fn clone_value(&self) -> Self {
        VIContext {
            dims: self.dims,
            value: self.value.clone(),
            penalty: self.penalty.clone(),
            goal_mask: self.goal_mask.clone(),
            transitions: self.transitions.clone(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Budget {
    /// Reference / block / pyramid sweep count.
    Sweeps(u32),
    /// Frontier iteration count.
    Iterations(u32),
}

#[derive(Debug)]
pub struct SolveStats {
    pub iters_or_sweeps: u32,
    pub updates: u64,
    pub final_delta: Value,
    pub converged: bool,
    pub extra: Option<SolveExtra>,
}

#[derive(Debug)]
pub enum SolveExtra {
    PyramidPerLevel(Vec<PyramidLevelStat>),
    ActionTable(Array3<ActionIdx>),
}

#[derive(Clone, Copy, Debug)]
pub struct PyramidLevelStat {
    pub level: u32,
    pub map_x: u32,
    pub map_y: u32,
    pub scale: u32,
    pub sweeps: u32,
    pub changed_states: u64,
    pub visited_states: u64,
    pub final_delta: Value,
}

/// Common interface for value-iteration solver variants.
///
/// All solvers operate on a shared [`VIContext`], updating `value` in place
/// while treating `penalty`, `goal_mask`, `transitions`, and `dims` as
/// read-only inputs.
///
/// Implementations should run until convergence (residual <= solver's
/// configured threshold) OR until the [`Budget`] is exhausted, whichever
/// comes first. The returned [`SolveStats`] records which condition fired
/// via `converged`.
pub trait Solver: Send + Sync {
    /// Short static identifier used to label benchmark rows
    /// (e.g. `"reference"`, `"frontier_3d"`, `"pyramid_sweep"`).
    fn name(&self) -> &'static str;

    /// Runs the solver to convergence or until `budget` is exhausted.
    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats;
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndarray::{Array2, Array3};
    use vi_core::params::N_THETA;

    fn make_test_ctx(map_x: u32, map_y: u32) -> VIContext {
        let mx = map_x as usize;
        let my = map_y as usize;
        VIContext {
            dims: MapDims { map_x, map_y },
            value: Array3::zeros((my, mx, N_THETA)),
            penalty: Array2::zeros((my, mx)),
            goal_mask: Array3::from_elem((my, mx, N_THETA), false),
            transitions: TransitionModel::default(),
        }
    }

    #[test]
    fn clone_value_independence() {
        let ctx = make_test_ctx(3, 3);
        let mut cloned = ctx.clone_value();

        cloned.value[[0, 0, 0]] = 42;
        assert_eq!(ctx.value[[0, 0, 0]], 0, "original must not be affected by mutation of clone value");
        assert_eq!(cloned.value[[0, 0, 0]], 42);

        cloned.penalty[[0, 0]] = 99;
        assert_eq!(ctx.penalty[[0, 0]], 0, "original must not be affected by mutation of clone penalty");
        assert_eq!(cloned.penalty[[0, 0]], 99);

        cloned.goal_mask[[0, 0, 0]] = true;
        assert!(!ctx.goal_mask[[0, 0, 0]], "original must not be affected by mutation of clone goal_mask");
        assert!(cloned.goal_mask[[0, 0, 0]]);

        cloned.transitions.n_outcomes[0][0] = 7;
        assert_eq!(ctx.transitions.n_outcomes[0][0], 0, "original must not be affected by mutation of clone transitions");
        assert_eq!(cloned.transitions.n_outcomes[0][0], 7);
    }
}
