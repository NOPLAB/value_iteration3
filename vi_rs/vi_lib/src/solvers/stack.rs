//! FrontierStack の u64 版。`vi_algorithm/src/frontier/stack.rs` を本家 u64 モデルへ移植。
//! θ 層ごとに 2D フロンティアを持ち、拡張は「層ごと 2D dilate + θ 方向 ±mt の OR マージ」。
//! 収束値・方策は Reference = 本家と bit-exact。
//!
//! 注: u16 版は passable / goal 補集合マスクで候補を絞るが、本 u64 版は
//! `value_iteration_raw` が free でない/final セルを更新せず据置くため、マスクを省いても
//! bit-exact（評価が無駄に走るだけ）。簡潔さを優先しマスクは省略する。

use crate::params::MAX_COST;
use crate::solvers::observe::{
    BoundaryPacer, InPlaceProbe, SolveFlow, SolveObserver, SolveOutcome,
};
use crate::solvers::{displacement, Frontier2D};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// セット済み `ValueIterator` を FrontierStack で収束まで解く。
pub fn frontier_stack_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    let mut pacer = BoundaryPacer::new(obs);
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let nt_us = nt as usize;
    let (mx, my, mt) = displacement(vi);
    let (dx, dy) = (mx as u32, my as u32);
    let mt = mt as usize;
    let mut dilator = Frontier2D::new(nx as u32, ny as u32);

    // 種: θ 層ごとに total_cost<MAX_COST の (ix,iy)。
    let mut frontier: Vec<Vec<(u32, u32)>> = vec![Vec::new(); nt_us];
    for s in &vi.states {
        if s.total_cost < MAX_COST {
            frontier[s.it as usize].push((s.ix as u32, s.iy as u32));
        }
    }

    let mut merged: Vec<(u32, u32)> = Vec::new();
    let mut candidates: Vec<(u32, u32)> = Vec::new();
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    while frontier.iter().any(|l| !l.is_empty()) && iters < max_iter {
        iters += 1;

        // 各 θ 層について自層 + θ 近傍 (±mt, wrap) 層の活性セルを集めて 2D 膨張し更新する。
        let mut new_frontier: Vec<Vec<(u32, u32)>> = vec![Vec::new(); nt_us];
        for it in 0..nt_us {
            merged.clear();
            merged.extend_from_slice(&frontier[it]);
            for st in 1..=mt {
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
                // 値が「動いた」ら伝播する (向きの話は `SolverCaps::resweep` の doc)。
                if value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt) > 0 {
                    updates += 1;
                    new_frontier[it].push((ix, iy));
                }
            }
        }
        frontier = new_frontier;
        if frontier.iter().any(|l| !l.is_empty()) && pacer.due(iters as u64) {
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

