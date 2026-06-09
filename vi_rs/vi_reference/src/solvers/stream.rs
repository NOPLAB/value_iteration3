//! StreamMimic の u64 版。`vi_algorithm/src/stream/` は HLS ストリーミングカーネル
//! （行ごとの line buffer・strip 処理）を模倣するが、その収束値は行ストリーミング順の全
//! Gauss-Seidel sweep であり Reference と bit-exact。u64 では 16bit ハードウェアのストリーミング
//! 詳細は値に影響しないため、Reference の行優先 sweep（`value_iteration_worker` の
//! `sweep_orders[0]` = y,x,t 順 = 行ストリーミング順）にそのまま委譲する。

use crate::solvers::reference_solve;
use crate::value_iterator::ValueIterator;

/// セット済み `ValueIterator` を StreamMimic（= 行優先 Reference）で収束まで解く。
pub fn stream_mimic_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    reference_solve(vi, max_iter)
}

#[cfg(test)]
mod tests {
    use super::stream_mimic_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn parity_standard_maps_stream_mimic() {
        parity_standard_maps(|vi| stream_mimic_solve(vi, 2000));
    }
}
