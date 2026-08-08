//! 本家 ROS1 `value_iteration` の解き方そのもの — 全域 Gauss–Seidel 全走査。
//! `U64Solver::Reference` と `U64Solver::StreamMimic` (行優先 sweep = 本家順そのもの)
//! がここへ委譲し、他の全ソルバの bit-exact 検証のオラクル (conformance テストの
//! 比較基準) はこの全走査の固定点。
//!
//! - [`sweep_status`] — 本家 `SweepWorkerStatus` (スイープワーカーの進捗報告)。
//! - [`worker`] — 本家 `valueIterationWorker` の単スレッド/マルチスレッド経路
//!   (`ValueIterator` のメソッドとして定義。マルチスレッドは本家のデータ競合を忠実再現)。
//! - [`solve`] — 全走査を strict 固定点まで回す [`SolveObserver`] 対応ドライバ。
//!
//! [`SolveObserver`]: crate::solvers::observe::SolveObserver

pub mod solve;
pub mod sweep_status;
pub mod worker;

pub use solve::original_solve_observed;
pub use sweep_status::SweepWorkerStatus;

#[cfg(test)]
mod tests;
