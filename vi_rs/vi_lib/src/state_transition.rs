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
}
