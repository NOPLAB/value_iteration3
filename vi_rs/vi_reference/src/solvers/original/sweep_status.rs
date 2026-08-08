//! 本家 `SweepWorkerStatus` 忠実移植。
//! 本家コンストラクタは `_finished=false; _sweep_step=0; _delta=max_cost_`。

use crate::params::MAX_COST;

#[derive(Clone, Debug, PartialEq)]
pub struct SweepWorkerStatus {
    pub finished: bool,
    pub sweep_step: i32,
    pub delta: f64,
}

impl Default for SweepWorkerStatus {
    fn default() -> Self {
        Self {
            finished: false,
            sweep_step: 0,
            delta: MAX_COST as f64,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_matches_original() {
        let s = SweepWorkerStatus::default();
        assert!(!s.finished);
        assert_eq!(s.sweep_step, 0);
        assert_eq!(s.delta, MAX_COST as f64);
    }
}
