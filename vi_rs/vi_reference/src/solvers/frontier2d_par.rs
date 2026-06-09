//! B1: frontier2d_pad の決定的マルチスレッド版 (Jacobi)。
//!
//! 固定点は一意・更新順序非依存なので、ラウンド内を Jacobi 化して並列化しても到達可能セルの
//! 収束値・方策は本家と bit-exact。**決定性**を保つため:
//!  - compute フェーズ: 各スレッドは共有 `hot` を**読み取り専用**で参照し、自分の担当セルの
//!    新値を計算して返す (ラウンド内は誰も hot を書かない = スナップショット読み = スケジュール非依存)。
//!  - apply フェーズ: join 後に直列で hot へ書き戻し、new_frontier を構築。
//! スレッド数や分割の仕方に依らず同一の固定点へ収束する (安全な Rust、unsafe 不使用)。
//!
//! `optimal_action` は収束後の最終 argmin パスで確定する (到達可能セルは固定点値からの argmin が
//! 本家の最終 sweep と一致)。

use crate::params::MAX_COST;
use crate::value_iterator::ValueIterator;

use super::frontier2d_pad::{action_cost_pad, Padded};
use super::{seed_frontier_2d, Bitboard2D};

fn n_threads() -> usize {
    std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1)
}

/// セット済み `ValueIterator` を決定的並列 Jacobi frontier2d で解く。`(iters, updates, converged)`。
pub fn frontier2d_par_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let mut m = Padded::build(vi);
    let (nx, ny, nt) = (m.nx, m.ny, m.nt);
    let nthreads = n_threads();

    let (dx, dy) = (m.mx as u32, m.my as u32);
    let mut frontier = seed_frontier_2d(vi);
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;

    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let candidates: Vec<(u32, u32)> = frontier.dilate(dx, dy).enumerate().collect();
        let chunk = candidates.len().div_ceil(nthreads).max(1);

        // ── compute (並列・hot 読み取り専用): 変化したセルの (pad_idx, 新値, ix, iy) を収集。──
        let m_ref = &m;
        let results: Vec<Vec<(usize, u64, u32, u32)>> = std::thread::scope(|scope| {
            let handles: Vec<_> = candidates
                .chunks(chunk)
                .map(|part| {
                    scope.spawn(move || {
                        let mut ups: Vec<(usize, u64, u32, u32)> = Vec::new();
                        for &(ixu, iyu) in part {
                            let (ix, iy) = (ixu as i32, iyu as i32);
                            let pad_col = m_ref.pad_col(ix, iy);
                            for it in 0..nt {
                                let pad_idx = (pad_col + it as i64) as usize;
                                if !m_ref.free[pad_idx] || m_ref.finals[pad_idx] {
                                    continue;
                                }
                                let before = m_ref.hot[pad_idx][0];
                                let mut min_cost = MAX_COST;
                                for per_theta in m_ref.precomp.iter() {
                                    let c = action_cost_pad(
                                        &m_ref.hot,
                                        &m_ref.free,
                                        &per_theta[it as usize],
                                        pad_col,
                                    );
                                    if c < min_cost {
                                        min_cost = c;
                                    }
                                }
                                if min_cost < before {
                                    ups.push((pad_idx, min_cost, ixu, iyu));
                                }
                            }
                        }
                        ups
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().unwrap()).collect()
        });

        // ── apply (直列): hot 書き戻し + new_frontier 構築。──
        let mut new_frontier = Bitboard2D::new(nx as u32, ny as u32);
        for ups in &results {
            for &(pad_idx, min_cost, ixu, iyu) in ups {
                m.hot[pad_idx][0] = min_cost;
                updates += 1;
                new_frontier.set(ixu, iyu);
            }
        }
        frontier = new_frontier;
    }

    // ── 最終 argmin パス (並列): 収束値から optimal_action を確定。──
    let opt = final_policy(&m, nthreads);
    m.write_back(vi, Some(&opt));
    (iters, updates, frontier.popcount() == 0)
}

/// 収束した `hot` から全 free・非 final セルの optimal_action を計算 (並列・読み取り専用)。
/// 返り値はオリジナル座標 index の `Vec<Option<usize>>`。
fn final_policy(m: &Padded, nthreads: usize) -> Vec<Option<usize>> {
    let (nx, ny, nt) = (m.nx, m.ny, m.nt);
    let n = (nx * ny * nt) as usize;
    let rows: Vec<i32> = (0..ny).collect();
    let chunk = rows.len().div_ceil(nthreads).max(1);

    let parts: Vec<Vec<(usize, Option<usize>)>> = std::thread::scope(|scope| {
        let handles: Vec<_> = rows
            .chunks(chunk)
            .map(|band| {
                scope.spawn(move || {
                    let mut out: Vec<(usize, Option<usize>)> = Vec::new();
                    for &iy in band {
                        for ix in 0..nx {
                            let pad_col = m.pad_col(ix, iy);
                            let orig_col = (ix * nt + iy * (nt * nx)) as usize;
                            for it in 0..nt {
                                let pad_idx = (pad_col + it as i64) as usize;
                                if !m.free[pad_idx] || m.finals[pad_idx] {
                                    continue;
                                }
                                let mut min_cost = MAX_COST;
                                let mut min_action: Option<usize> = None;
                                for (ai, per_theta) in m.precomp.iter().enumerate() {
                                    let c = action_cost_pad(
                                        &m.hot,
                                        &m.free,
                                        &per_theta[it as usize],
                                        pad_col,
                                    );
                                    if c < min_cost {
                                        min_cost = c;
                                        min_action = Some(ai);
                                    }
                                }
                                out.push((orig_col + it as usize, min_action));
                            }
                        }
                    }
                    out
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });

    let mut opt = vec![None; n];
    for part in parts {
        for (orig, action) in part {
            opt[orig] = action;
        }
    }
    opt
}

#[cfg(test)]
mod tests {
    use super::frontier2d_par_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn parity_standard_maps_frontier2d_par() {
        parity_standard_maps(|vi| frontier2d_par_solve(vi, 2000));
    }
}
