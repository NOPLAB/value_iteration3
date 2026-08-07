//! Frontier3D の u64 版。`vi_algorithm/src/frontier/f3d.rs` の `run_serial_inner` を
//! 本家 u64 モデル（`value_iteration_raw`）へ移植。コスト数式は不変なので、到達可能セルの
//! 収束値・方策は Reference (全走査) = 本家と bit-exact。

use crate::solvers::observe::{NullObserver, SolveObserver, SolveOutcome};
use crate::solvers::frontier3d_driver;
use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// セット済み `ValueIterator` を Frontier3D で収束まで解く。
///
/// 反復骨格は [`frontier3d_driver`] が担う。候補セルを `value_iteration_raw` で更新し、
/// `total_cost` が**厳密減少**したセルを次フロンティアに入れる。`final_state`/非 `free` セルは
/// `value_iteration_raw` が更新せず据置くので、候補に混ざっても安全に無視される。
pub fn frontier3d_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    frontier3d_driver(vi, max_iter, obs, |vi, ix, iy, it| {
        let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
        let idx = vi.to_index(ix as i32, iy as i32, it as i32) as usize;
        let before = vi.states[idx].total_cost;
        value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
        vi.states[idx].total_cost < before
    })
}

/// 従来 API (observer なし)。`(iters, updates, converged)`。
pub fn frontier3d_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    frontier3d_solve_observed(vi, max_iter, &mut NullObserver).tuple()
}
