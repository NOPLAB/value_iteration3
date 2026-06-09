//! BlockRefine の u64 版。`vi_algorithm/src/block/refine.rs` を本家 u64 モデルへ移植。
//! スケジューラは粗い（変化したブロックを追跡）が、バックアップは細かい（活性ブロックは
//! 全 θ を `value_iteration_raw` で更新）。threshold=0 で Reference = 本家と bit-exact。

use crate::params::MAX_COST;
use crate::value_iterator::{value_iteration_raw, ValueIterator};

const BLOCK: i32 = 8; // ブロック幅・高さ (refine.rs 既定)
const LOCAL_SWEEPS: u32 = 2; // ブロック内 inner sweep 回数 (refine.rs 既定)

/// 活性ブロックの全セル×θ を `local_passes` 回更新する。`(updates, changed)` を返す。
/// `changed` は到達可能セルが1つでも減少したか（ブロックを次反復も活性にする信号）。
#[allow(clippy::too_many_arguments)]
fn update_block(
    vi: &mut ValueIterator,
    x0: i32, x1: i32, y0: i32, y1: i32,
    nx: i32, ny: i32, nt: i32,
    local_passes: u32,
) -> (u64, bool) {
    let mut updates = 0u64;
    let mut changed = false;
    for _ in 0..local_passes {
        for iy in y0..=y1 {
            for ix in x0..=x1 {
                for it in 0..nt {
                    let idx = vi.to_index(ix, iy, it) as usize;
                    let before = vi.states[idx].total_cost;
                    value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
                    if vi.states[idx].total_cost < before {
                        updates += 1;
                        changed = true;
                    }
                }
            }
        }
    }
    (updates, changed)
}

/// セット済み `ValueIterator` を BlockRefine で収束まで解く。`(iters, updates, converged)` を返す。
pub fn block_refine_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    // i32::div_ceil は unstable なので手動 (nx,ny,mx,my は非負)。
    let ceil_div = |a: i32, b: i32| (a + b - 1) / b;
    let n_bx = ceil_div(nx, BLOCK);
    let n_by = ceil_div(ny, BLOCK);
    let nb = (n_bx * n_by) as usize;
    let bidx = |bx: i32, by: i32| -> usize { (by * n_bx + bx) as usize };

    // ブロック膨張半径 (mx/my をブロック単位に換算)。
    let (mx, my, _mt) = super::displacement(vi);
    let rx = ceil_div(mx, BLOCK);
    let ry = ceil_div(my, BLOCK);

    // passable ブロック: free セルを含むブロック。
    let mut passable = vec![false; nb];
    // 種ブロック: goal (total_cost<MAX_COST) を含むブロック。
    let mut frontier = vec![false; nb];
    for s in &vi.states {
        let b = bidx(s.ix / BLOCK, s.iy / BLOCK);
        if s.free {
            passable[b] = true;
        }
        if s.total_cost < MAX_COST {
            frontier[b] = true;
        }
    }

    let any = |m: &[bool]| m.iter().any(|&b| b);

    let mut updates = 0u64;
    let mut iters = 0u32;
    let converged = loop {
        if !any(&frontier) {
            break true;
        }
        if iters >= max_iter {
            break false;
        }
        iters += 1;

        // 活性ブロック = frontier をブロック単位 ±(rx,ry) 膨張し passable に制限。
        let mut active = vec![false; nb];
        for by in 0..n_by {
            for bx in 0..n_bx {
                if !frontier[bidx(bx, by)] {
                    continue;
                }
                for dby in -ry..=ry {
                    let jy = by + dby;
                    if jy < 0 || jy >= n_by {
                        continue;
                    }
                    for dbx in -rx..=rx {
                        let jx = bx + dbx;
                        if jx < 0 || jx >= n_bx {
                            continue;
                        }
                        let j = bidx(jx, jy);
                        if passable[j] {
                            active[j] = true;
                        }
                    }
                }
            }
        }

        let mut next = vec![false; nb];
        let mut any_changed = false;
        for by in 0..n_by {
            let y0 = by * BLOCK;
            let y1 = ((by + 1) * BLOCK).min(ny) - 1;
            for bx in 0..n_bx {
                if !active[bidx(bx, by)] {
                    continue;
                }
                let x0 = bx * BLOCK;
                let x1 = ((bx + 1) * BLOCK).min(nx) - 1;
                let (u, changed) = update_block(vi, x0, x1, y0, y1, nx, ny, nt, LOCAL_SWEEPS);
                updates += u;
                if changed {
                    next[bidx(bx, by)] = true;
                    any_changed = true;
                }
            }
        }
        frontier = next;
        if !any_changed {
            break true;
        }
    };
    (iters, updates, converged)
}

#[cfg(test)]
mod tests {
    use super::block_refine_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn parity_standard_maps_block_refine() {
        parity_standard_maps(|vi| block_refine_solve(vi, 2000));
    }
}
