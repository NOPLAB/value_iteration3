//! 本家 `Action` 忠実移植。

use crate::state_transition::StateTransition;

/// 行動 1 つ。`state_transitions[theta]` が θ ごとの遷移先リスト。
#[derive(Clone, Debug)]
pub struct Action {
    pub name: String,
    pub delta_fw: f64,  // _delta_fw [m]
    pub delta_rot: f64, // _delta_rot [deg]
    pub id: i32,        // id_
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
            state_transitions: Vec::new(),
        }
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
        assert!(a.state_transitions.is_empty());
    }
}
