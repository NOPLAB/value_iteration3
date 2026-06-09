//! FrontierStack の u64 版。`vi_algorithm/src/frontier/stack.rs` を本家 u64 モデルへ移植。
//! θ 層ごとに 2D フロンティアを持ち、拡張は「層ごと 2D dilate + θ 方向 ±mt の OR マージ」。
//! 収束値・方策は Reference = 本家と bit-exact。
//!
//! 注: u16 版は passable / goal 補集合マスクで候補を絞るが、本 u64 版は
//! `value_iteration_raw` が free でない/final セルを更新せず据置くため、マスクを省いても
//! bit-exact（評価が無駄に走るだけ）。簡潔さを優先しマスクは省略する。

use crate::params::MAX_COST;
use crate::solvers::{displacement, Bitboard2D};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// セット済み `ValueIterator` を FrontierStack で収束まで解く。`(iters, updates, converged)` を返す。
pub fn frontier_stack_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let nt_us = nt as usize;
    let (mx, my, mt) = displacement(vi);
    let (dx, dy) = (mx as u32, my as u32);
    let mt = mt as usize;

    // 種: θ 層ごとに total_cost<MAX_COST の (ix,iy)。
    let mut frontier: Vec<Bitboard2D> =
        (0..nt_us).map(|_| Bitboard2D::new(nx as u32, ny as u32)).collect();
    for s in &vi.states {
        if s.total_cost < MAX_COST {
            frontier[s.it as usize].set(s.ix as u32, s.iy as u32);
        }
    }

    let stack_popcount = |layers: &[Bitboard2D]| -> u64 { layers.iter().map(|b| b.popcount()).sum() };

    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    while stack_popcount(&frontier) > 0 && iters < max_iter {
        iters += 1;

        // Step 1: 層ごと 2D dilate。
        let dilated: Vec<Bitboard2D> = frontier.iter().map(|b| b.dilate(dx, dy)).collect();

        // Step 2: 候補 = 自層 dilate | θ 近傍 (±mt, wrap) 層の dilate。
        let candidates: Vec<Bitboard2D> = (0..nt_us)
            .map(|it| {
                let mut cand = dilated[it].clone();
                for st in 1..=mt {
                    let it_minus = (it + nt_us - st) % nt_us;
                    let it_plus = (it + st) % nt_us;
                    cand.or_inplace(&dilated[it_minus]);
                    cand.or_inplace(&dilated[it_plus]);
                }
                cand
            })
            .collect();

        // Step 3: 各層の候補を value_iteration_raw で更新。
        let mut new_frontier: Vec<Bitboard2D> =
            (0..nt_us).map(|_| Bitboard2D::new(nx as u32, ny as u32)).collect();
        for it in 0..nt_us {
            for (ix, iy) in candidates[it].enumerate() {
                let idx = vi.to_index(ix as i32, iy as i32, it as i32) as usize;
                let before = vi.states[idx].total_cost;
                value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
                if vi.states[idx].total_cost < before {
                    updates += 1;
                    new_frontier[it].set(ix, iy);
                }
            }
        }
        frontier = new_frontier;
    }
    (iters, updates, stack_popcount(&frontier) == 0)
}

#[cfg(test)]
mod tests {
    use super::frontier_stack_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn parity_standard_maps_frontier_stack() {
        parity_standard_maps(|vi| frontier_stack_solve(vi, 2000));
    }
}
