//! 本家 `StateTransition` 忠実移植。

/// 1 つの遷移先。`dix`/`diy` は変位 (delta)、`dit` は **絶対 θ インデックス**。
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StateTransition {
    pub dix: i32,
    pub diy: i32,
    pub dit: i32,
    pub prob: i32,
}

impl StateTransition {
    pub fn new(dix: i32, diy: i32, dit: i32, prob: i32) -> Self {
        Self { dix, diy, dit, prob }
    }

    /// 本家 `StateTransition::to_string`。
    #[allow(clippy::inherent_to_string)]
    pub fn to_string(&self) -> String {
        format!(
            "dix:{} diy:{} dit:{} prob:{}",
            self.dix, self.diy, self.dit, self.prob
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn to_string_matches_original_format() {
        let st = StateTransition::new(1, -2, 3, 4);
        assert_eq!(st.to_string(), "dix:1 diy:-2 dit:3 prob:4");
    }
}
