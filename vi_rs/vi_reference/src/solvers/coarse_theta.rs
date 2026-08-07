//! Frontier3DCoarseTheta の u64 版。`vi_algorithm/src/frontier/coarse_theta.rs` を本家 u64
//! モデルへ移植。粗い θ（`step` ごと）だけを先に伝播させてから全 θ を refine する coarse-to-fine。
//!
//! `step <= 1` は Frontier3D と等価（bit-exact）。`step > 1` は coarse pass（θ%step==0 のセルのみ
//! 更新）で値を上から下げ、その後 Frontier3D で全 θ を収束まで refine する。coarse pass は妥当な
//! Bellman 更新の部分集合なので値は固定点以上に留まり、refine が真の固定点へ収束する → 本家と
//! bit-exact（u16 版の refine 上限による近似と異なり、ここでは完全収束させる）。

use crate::solvers::observe::{NullObserver, SolveObserver, SolveOutcome};
use crate::solvers::{frontier3d::frontier3d_solve_observed, frontier3d_driver};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

const COARSE_BUDGET: u32 = 64; // coarse pass の反復上限

/// セット済み `ValueIterator` を Frontier3DCoarseTheta で収束まで解く。
pub fn frontier3d_coarse_theta_solve_observed(
    vi: &mut ValueIterator,
    step: u32,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    if step <= 1 {
        return frontier3d_solve_observed(vi, max_iter, obs);
    }
    let step_i = step as i32;

    // ── coarse pass: θ%step==0 のセルのみ更新（値を上から下げる事前伝播） ──
    let coarse = frontier3d_driver(vi, COARSE_BUDGET, obs, |vi, ix, iy, it| {
        if (it as i32) % step_i != 0 {
            return false; // 粗い θ のみ更新
        }
        let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
        let idx = vi.to_index(ix as i32, iy as i32, it as i32) as usize;
        let before = vi.states[idx].total_cost;
        value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
        vi.states[idx].total_cost < before
    });
    if coarse.stopped || coarse.cancelled {
        return coarse;
    }

    // ── refine: 全 θ を Frontier3D で収束まで（上からの収束 → 本家と bit-exact） ──
    // observer から見た iters/updates が coarse pass から単調に続くようオフセットを重ねる。
    let mut off = crate::solvers::observe::OffsetObserver {
        inner: obs,
        iters_offset: coarse.iters,
        updates_offset: coarse.updates,
    };
    let refine =
        frontier3d_solve_observed(vi, max_iter.saturating_sub(coarse.iters), &mut off);
    SolveOutcome {
        iters: coarse.iters + refine.iters,
        updates: coarse.updates + refine.updates,
        ..refine
    }
}

/// 従来 API (observer なし)。`(iters, updates, converged)`。
pub fn frontier3d_coarse_theta_solve(
    vi: &mut ValueIterator,
    step: u32,
    max_iter: u32,
) -> (u32, u64, bool) {
    frontier3d_coarse_theta_solve_observed(vi, step, max_iter, &mut NullObserver).tuple()
}
