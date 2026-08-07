//! StreamMimic の u64 版。`vi_algorithm/src/stream/` は HLS ストリーミングカーネル
//! （行ごとの line buffer・strip 処理）を模倣するが、その収束値は行ストリーミング順の全
//! Gauss-Seidel sweep であり Reference と bit-exact。u64 では 16bit ハードウェアのストリーミング
//! 詳細は値に影響しないため、本家全走査（`solvers::original` の行優先 sweep =
//! `sweep_orders[0]` = y,x,t 順 = 行ストリーミング順）にそのまま委譲する。

use crate::solvers::observe::{NullObserver, SolveObserver, SolveOutcome};
use crate::solvers::original::original_solve_observed;
use crate::value_iterator::ValueIterator;

/// セット済み `ValueIterator` を StreamMimic（= 行優先の本家全走査）で収束まで解く。
pub fn stream_mimic_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    original_solve_observed(vi, max_iter, obs)
}

/// 従来 API (observer なし)。
pub fn stream_mimic_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    stream_mimic_solve_observed(vi, max_iter, &mut NullObserver).tuple()
}
