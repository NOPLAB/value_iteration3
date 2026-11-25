//! Frontier3D の u64 版。`vi_algorithm/src/frontier/f3d.rs` の `run_serial_inner` を
//! 本家 u64 モデル（`value_iteration_raw`）へ移植。コスト数式は不変なので、到達可能セルの
//! 収束値・方策は Reference (全走査) = 本家と bit-exact。

use crate::solvers::observe::{
    BoundaryPacer, InPlaceProbe, SolveFlow, SolveObserver, SolveOutcome,
};
use crate::solvers::{displacement, seed_frontier, Frontier2D};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// セット済み `ValueIterator` を Frontier3D で収束まで解く。
///
/// 「seed → (3D 膨張 → 候補セルを `value_iteration_raw` で更新 → `total_cost` が
/// **変化**したセルを次フロンティアへ) を収束まで」。`final_state`/非 `free` セルは
/// `value_iteration_raw` が更新せず据置くので、候補に混ざっても安全に無視される。
/// ラウンド境界ごとに `obs.boundary` を呼ぶ (in-place なので観測は無料)。
pub fn frontier3d_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let nt_us = nt as usize;
    let (mx, my, mt) = displacement(vi);
    let (dx, dy, dt) = (mx as u32, my as u32, mt as usize);
    let mut pacer = BoundaryPacer::new(obs);
    let mut dilator = Frontier2D::new(nx as u32, ny as u32);
    let mut frontier: Vec<Vec<(u32, u32)>> = vec![Vec::new(); nt_us];
    for (ix, iy, it) in seed_frontier(vi) {
        frontier[it as usize].push((ix, iy));
    }
    let mut merged: Vec<(u32, u32)> = Vec::new();
    let mut candidates: Vec<(u32, u32)> = Vec::new();
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    while frontier.iter().any(|l| !l.is_empty()) && iters < max_iter {
        iters += 1;
        let mut new_frontier: Vec<Vec<(u32, u32)>> = vec![Vec::new(); nt_us];
        for it in 0..nt_us {
            // 3D box 膨張の θ=it スライス: ±dt 循環近傍層のセルを集めて 2D 膨張する。
            merged.clear();
            merged.extend_from_slice(&frontier[it]);
            for st in 1..=dt {
                let it_minus = (it + nt_us - st) % nt_us;
                let it_plus = (it + st) % nt_us;
                merged.extend_from_slice(&frontier[it_minus]);
                if it_plus != it_minus {
                    merged.extend_from_slice(&frontier[it_plus]);
                }
            }
            if merged.is_empty() {
                continue;
            }
            dilator.dilate_into(&merged, dx, dy, &mut candidates);
            for &(ix, iy) in &candidates {
                let idx = vi.to_index(ix as i32, iy as i32, it as i32) as usize;
                // 値が「動いた」ら伝播する。白紙からの solve では値は単調に下がるので
                // `< before` と同じ挙動 (Reference と bit-exact) だが、収束後の掃き直しでは
                // local_penalty が値を**上げる**ので、下降しか追わないと伝播が黙って
                // 止まる (`SolverCaps::resweep`)。`value_iteration_raw` の戻り値は |Δ|。
                if value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt) > 0 {
                    updates += 1;
                    new_frontier[it].push((ix, iy));
                }
            }
        }
        frontier = new_frontier;
        let any = frontier.iter().any(|l| !l.is_empty());
        if any && pacer.due(iters as u64) {
            let mut probe = InPlaceProbe { vi, iters, updates };
            match obs.boundary(&mut probe) {
                SolveFlow::Continue => {}
                SolveFlow::Stop => return SolveOutcome::stopped(iters, updates),
                SolveFlow::Cancel => return SolveOutcome::cancelled(iters, updates),
            }
        }
    }
    SolveOutcome::running(iters, updates, frontier.iter().all(|l| l.is_empty()))
}
