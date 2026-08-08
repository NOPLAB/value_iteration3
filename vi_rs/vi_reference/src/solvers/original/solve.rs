//! 全域全走査 ([`super::worker`] の `value_iteration_worker`) を strict 固定点
//! (到達可能セルが不変) まで回す [`SolveObserver`] 対応ドライバ。
//! 旧 `solvers::reference_solve` の置き場。

use crate::solvers::observe::{
    BoundaryPacer, InPlaceProbe, SolveFlow, SolveObserver, SolveOutcome,
};
use crate::solvers::REACH_THRESH;
use crate::value_iterator::ValueIterator;

/// 本家全走査を strict 固定点 (到達可能セルが不変) まで回す。
/// 境界 (= 1 スイープ) ごとに `obs.boundary` を呼ぶ (in-place なので観測は無料)。
pub fn original_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    let mut pacer = BoundaryPacer::new(obs);
    let mut prev: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
    let mut iters = 0u32;
    let converged = loop {
        vi.value_iteration_worker(1, 0);
        iters += 1;
        let mut changed = false;
        for (i, s) in vi.states.iter().enumerate() {
            if s.total_cost < REACH_THRESH && s.total_cost != prev[i] {
                changed = true;
            }
            prev[i] = s.total_cost;
        }
        if !changed {
            break true;
        }
        if iters >= max_iter {
            break false;
        }
        if pacer.due(iters as u64) {
            let mut probe = InPlaceProbe { vi, iters, updates: 0 };
            match obs.boundary(&mut probe) {
                SolveFlow::Continue => {}
                SolveFlow::Stop => return SolveOutcome::stopped(iters, 0),
                SolveFlow::Cancel => return SolveOutcome::cancelled(iters, 0),
            }
        }
    };
    SolveOutcome::running(iters, 0, converged)
}

/// 従来 API (observer なし)。`(iters, updates, converged)`。
pub fn original_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    original_solve_observed(vi, max_iter, &mut crate::solvers::observe::NullObserver).tuple()
}
