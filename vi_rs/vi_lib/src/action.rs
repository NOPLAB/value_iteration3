//! 本家 `Action` 忠実移植。

use std::fmt;
use std::time::Duration;

use crate::params::PROB_BASE;
use crate::state_transition::StateTransition;

/// Invalid action cost in seconds.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ActionCostError;

impl fmt::Display for ActionCostError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "action cost must be finite, positive, at most 1000000 seconds, \
             and quantize to at least one PROB_BASE unit"
        )
    }
}

impl std::error::Error for ActionCostError {}

/// 行動 1 つ。`state_transitions[theta]` が θ ごとの遷移先リスト。
#[derive(Clone, Debug)]
pub struct Action {
    pub name: String,
    pub delta_fw: f64,  // _delta_fw [m]
    pub delta_rot: f64, // _delta_rot [deg]
    pub id: i32,        // id_
    /// Traversal cost in fixed-point seconds (`PROB_BASE` units per second).
    pub cost: u64,
    pub state_transitions: Vec<Vec<StateTransition>>,
}

impl Action {
    /// 本家 `Action(std::string name, double fw, double rot, int id)`。
    pub fn new(name: impl Into<String>, fw: f64, rot: f64, id: i32) -> Self {
        Self {
            name: name.into(),
            delta_fw: fw,
            delta_rot: rot,
            id,
            cost: PROB_BASE,
            state_transitions: Vec::new(),
        }
    }

    /// Set a cost in seconds, rounded down to `1 / PROB_BASE` seconds.
    pub fn with_cost_seconds(mut self, seconds: f64) -> Result<Self, ActionCostError> {
        if !seconds.is_finite() || seconds <= 0.0 || seconds > 1_000_000.0 {
            return Err(ActionCostError);
        }
        let duration = Duration::try_from_secs_f64(seconds).map_err(|_| ActionCostError)?;
        let units = duration
            .as_nanos()
            .checked_mul(u128::from(PROB_BASE))
            .ok_or(ActionCostError)?
            / 1_000_000_000;
        let mut cost = u64::try_from(units).map_err(|_| ActionCostError)?;
        // Duration rounds to nanoseconds. Correct across a fixed-point boundary
        // using its exactly representable binary fraction, so the result floors
        // the input seconds even when that boundary falls between nanoseconds.
        let base = f64::from(u32::try_from(PROB_BASE).map_err(|_| ActionCostError)?);
        let whole = u32::try_from(cost / PROB_BASE).map_err(|_| ActionCostError)?;
        let fraction = u32::try_from(cost % PROB_BASE).map_err(|_| ActionCostError)?;
        let quantized_seconds = f64::from(whole) + f64::from(fraction) / base;
        if quantized_seconds > seconds {
            cost -= 1;
        } else if quantized_seconds + 1.0 / base <= seconds {
            cost += 1;
        }
        if cost == 0 {
            return Err(ActionCostError);
        }
        self.cost = cost;
        Ok(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_sets_fields() {
        let a = Action::new("forward", 0.3, 0.0, 0);
        assert_eq!(a.name, "forward");
        assert_eq!(a.delta_fw, 0.3);
        assert_eq!(a.delta_rot, 0.0);
        assert_eq!(a.id, 0);
        assert_eq!(a.cost, PROB_BASE);
        assert!(a.state_transitions.is_empty());
    }

    #[test]
    fn cost_seconds_quantizes_down() {
        for (seconds, expected) in [
            (1.0, PROB_BASE),
            (0.5, PROB_BASE / 2),
            (1.25, PROB_BASE + PROB_BASE / 4),
            (0.1, PROB_BASE / 10),
            (1_000_000.0, 1_000_000 * PROB_BASE),
        ] {
            let action = Action::new("forward", 0.3, 0.0, 0)
                .with_cost_seconds(seconds)
                .unwrap();
            assert_eq!(action.cost, expected, "{seconds}");
            assert_eq!(action.delta_fw, 0.3);
        }
    }

    #[test]
    fn cost_seconds_preserves_fixed_point_boundaries() {
        let base = f64::from(u32::try_from(PROB_BASE).unwrap());
        for units in [1_u32, 2, 3, 17, 262_143, 262_144] {
            let seconds = f64::from(units) / base;
            let action = Action::new("forward", 0.3, 0.0, 0);
            assert_eq!(
                action.clone().with_cost_seconds(seconds).unwrap().cost,
                u64::from(units)
            );
            let below = f64::from_bits(seconds.to_bits() - 1);
            if units == 1 {
                assert_eq!(
                    action.with_cost_seconds(below).unwrap_err(),
                    ActionCostError
                );
            } else {
                assert_eq!(
                    action.with_cost_seconds(below).unwrap().cost,
                    u64::from(units - 1)
                );
            }
        }
    }

    #[test]
    fn cost_seconds_rejects_invalid_values() {
        for seconds in [
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            -1.0,
            -0.0,
            0.0,
            f64::MIN_POSITIVE,
            0.000_001,
            1_000_000.1,
            f64::MAX,
        ] {
            assert_eq!(
                Action::new("forward", 0.3, 0.0, 0)
                    .with_cost_seconds(seconds)
                    .unwrap_err(),
                ActionCostError,
                "{seconds}"
            );
        }
    }
}
