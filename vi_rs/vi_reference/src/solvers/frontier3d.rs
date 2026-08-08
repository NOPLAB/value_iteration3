//! Frontier3D の u64 版。`vi_algorithm/src/frontier/f3d.rs` の `run_serial_inner` を
//! 本家 u64 モデル（`value_iteration_raw`）へ移植。コスト数式は不変なので、到達可能セルの
//! 収束値・方策は Reference (全走査) = 本家と bit-exact。

use crate::solvers::observe::{
    BoundaryPacer, InPlaceProbe, SolveFlow, SolveObserver, SolveOutcome,
};
use crate::solvers::{displacement, seed_frontier, Bitboard3D};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// セット済み `ValueIterator` を Frontier3D で収束まで解く。
///
/// 「seed → (3D 膨張 → 候補セルを `value_iteration_raw` で更新 → `total_cost` が
/// **厳密減少**したセルを次フロンティアへ) を収束まで」。`final_state`/非 `free` セルは
/// `value_iteration_raw` が更新せず据置くので、候補に混ざっても安全に無視される。
/// ラウンド境界ごとに `obs.boundary` を呼ぶ (in-place なので観測は無料)。
pub fn frontier3d_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let (mx, my, mt) = displacement(vi);
    let (dx, dy, dt) = (mx as u32, my as u32, mt as u32);
    let mut pacer = BoundaryPacer::new(obs);
    let mut frontier = seed_frontier(vi);
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let candidates = frontier.dilate(dx, dy, dt);
        let mut new_frontier = Bitboard3D::new(nx as u32, ny as u32, nt as u32);
        for (ix, iy, it) in candidates.enumerate() {
            let idx = vi.to_index(ix as i32, iy as i32, it as i32) as usize;
            let before = vi.states[idx].total_cost;
            value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
            if vi.states[idx].total_cost < before {
                updates += 1;
                new_frontier.set(ix, iy, it);
            }
        }
        frontier = new_frontier;
        if frontier.popcount() > 0 && pacer.due(iters as u64) {
            let mut probe = InPlaceProbe { vi, iters, updates };
            match obs.boundary(&mut probe) {
                SolveFlow::Continue => {}
                SolveFlow::Stop => return SolveOutcome::stopped(iters, updates),
                SolveFlow::Cancel => return SolveOutcome::cancelled(iters, updates),
            }
        }
    }
    SolveOutcome::running(iters, updates, frontier.popcount() == 0)
}
