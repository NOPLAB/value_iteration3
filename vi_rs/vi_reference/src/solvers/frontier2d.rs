//! Frontier2D の u64 版。`vi_algorithm/src/frontier/f2d.rs` を本家 u64 モデルへ移植。
//! 空間 2D フロンティア: 活性 (ix,iy) が現れたら全 θ 層を再評価する。dilation は空間のみで
//! 安い代わりに per-cell 仕事量が N_THETA 倍。収束値・方策は Reference = 本家と bit-exact。

use crate::solvers::observe::{
    InPlaceProbe, NullObserver, SolveFlow, SolveObserver, SolveOutcome,
};
use crate::solvers::{displacement, frontier2d_driver, seed_frontier_2d, Frontier2DSweep};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// in-place モデル: 候補セルの全 θ 層を `value_iteration_raw` で直接更新する。
struct Sweep<'a> {
    vi: &'a mut ValueIterator,
}

impl Frontier2DSweep for Sweep<'_> {
    fn cell(&mut self, ix: u32, iy: u32) -> u64 {
        let vi = &mut *self.vi;
        let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
        let mut updates = 0u64;
        for it in 0..nt {
            let idx = vi.to_index(ix as i32, iy as i32, it) as usize;
            let before = vi.states[idx].total_cost;
            value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
            if vi.states[idx].total_cost < before {
                updates += 1;
            }
        }
        updates
    }

    fn boundary(&mut self, obs: &mut dyn SolveObserver, iters: u32, updates: u64) -> SolveFlow {
        let mut probe = InPlaceProbe { vi: self.vi, iters, updates };
        obs.boundary(&mut probe)
    }
}

/// セット済み `ValueIterator` を Frontier2D で収束まで解く。
///
/// 反復骨格は [`frontier2d_driver`] が担う。候補 (ix,iy) の全 θ 層を `value_iteration_raw` で
/// 更新し、減少した θ 層数を返す（1 以上なら次フロンティアへ）。
pub fn frontier2d_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    let (nx, ny) = (vi.cell_num_x, vi.cell_num_y);
    let (mx, my, _mt) = displacement(vi);
    let seed = seed_frontier_2d(vi);
    let mut model = Sweep { vi };
    frontier2d_driver(nx, ny, seed, mx as u32, my as u32, max_iter, obs, &mut model)
}

/// 従来 API (observer なし)。`(iters, updates, converged)`。
pub fn frontier2d_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    frontier2d_solve_observed(vi, max_iter, &mut NullObserver).tuple()
}
