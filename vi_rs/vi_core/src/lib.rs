//! Core value-iteration primitives: types, algorithm constants, and the `cost_of` function.

pub mod params;
pub mod types;
pub mod cost;
pub mod transitions;
pub mod goal;

pub use types::{Value, Penalty, Offset, ThetaIdx, ActionIdx};
pub use cost::cost_of;
pub use transitions::{PackedTransitions, TransitionModel};
pub use goal::{GoalSpec, make_goal_mask};
pub use params::{MAX_VALUE, N_THETA, N_ACTIONS, PROB_BASE,
                 PENALTY_OBSTACLE, PENALTY_GOAL, STEP_COST};
