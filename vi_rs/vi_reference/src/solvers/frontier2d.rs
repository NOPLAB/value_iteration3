//! Frontier2D の u64 版。`vi_algorithm/src/frontier/f2d.rs` を本家 u64 モデルへ移植。
//! 空間 2D フロンティア: 活性 (ix,iy) が現れたら全 θ 層を再評価する。dilation は空間のみで
//! 安い代わりに per-cell 仕事量が N_THETA 倍。収束値・方策は Reference = 本家と bit-exact。

use crate::solvers::{displacement, seed_frontier_2d, Bitboard2D};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// セット済み `ValueIterator` を Frontier2D で収束まで解く。`(iters, updates, converged)` を返す。
pub fn frontier2d_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let (mx, my, _mt) = displacement(vi);
    let (dx, dy) = (mx as u32, my as u32);
    let mut frontier = seed_frontier_2d(vi);
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let candidates = frontier.dilate(dx, dy);
        let mut new_frontier = Bitboard2D::new(nx as u32, ny as u32);
        for (ix, iy) in candidates.enumerate() {
            let mut changed = false;
            for it in 0..nt {
                let idx = vi.to_index(ix as i32, iy as i32, it) as usize;
                let before = vi.states[idx].total_cost;
                value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
                if vi.states[idx].total_cost < before {
                    updates += 1;
                    changed = true;
                }
            }
            if changed {
                new_frontier.set(ix, iy);
            }
        }
        frontier = new_frontier;
    }
    (iters, updates, frontier.popcount() == 0)
}

#[cfg(test)]
mod tests {
    use super::frontier2d_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn parity_standard_maps_frontier2d() {
        parity_standard_maps(|vi| frontier2d_solve(vi, 2000));
    }
}
