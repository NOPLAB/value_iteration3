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
use crate::solvers::observe::{
    BoundaryPacer, MaterializeProbe, SolveFlow, SolveObserver, SolveOutcome,
};
use crate::value_iterator::ValueIterator;

use super::frontier2d_pad::{action_cost_pad, HotCells, Padded};
use super::{seed_frontier_2d, Bitboard2D};

pub(crate) fn n_threads() -> usize {
    // ベンチ用スレッド数オーバーライド (Fig.21/22 のスレッド掃引)。既定は論理コア数。
    // 決定的 Jacobi なのでスレッド数を変えても収束値・更新回数は不変 (wall-clock のみ変化)。
    if let Ok(v) = std::env::var("VI_THREADS") {
        if let Ok(n) = v.parse::<usize>() {
            if n >= 1 {
                return n;
            }
        }
    }
    std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1)
}

/// 1 ラウンド分の並列 compute: `candidates` の各セル全 θ を再評価し、減少したセルの
/// `(pad_idx, 新値, ix, iy)` を返す (hot 読み取り専用・決定的)。
/// `action_cost_pad` の未確定隣接クランプにより hot には実数値しか入らないので、
/// 単調降下 (`<`) だけで偽低値ラッチなしに真の固定点へ到達する。
fn compute_round(
    m: &Padded,
    candidates: &[(u32, u32)],
    nthreads: usize,
) -> Vec<Vec<(usize, u64, u32, u32)>> {
    let nt = m.nt;
    let chunk = candidates.len().div_ceil(nthreads).max(1);
    std::thread::scope(|scope| {
        let handles: Vec<_> = candidates
            .chunks(chunk)
            .map(|part| {
                scope.spawn(move || {
                    let mut ups: Vec<(usize, u64, u32, u32)> = Vec::new();
                    for &(ixu, iyu) in part {
                        let (ix, iy) = (ixu as i32, iyu as i32);
                        let pad_col = m.pad_col(ix, iy);
                        for it in 0..nt {
                            let pad_idx = (pad_col + it as i64) as usize;
                            if !m.free[pad_idx] || m.finals[pad_idx] {
                                continue;
                            }
                            let before = m.hot[pad_idx][0];
                            let mut min_cost = MAX_COST;
                            for per_theta in m.precomp.iter() {
                                let c = action_cost_pad(
                                    m.hot.as_slice(),
                                    &m.free,
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
    })
}

/// セット済み `ValueIterator` を決定的並列 Jacobi frontier2d で解く。
pub fn frontier2d_par_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    let mut m = Padded::build(vi);
    let (nx, ny) = (m.nx, m.ny);
    let nthreads = n_threads();

    let (dx, dy) = (m.mx as u32, m.my as u32);
    let mut pacer = BoundaryPacer::new(obs);
    let mut frontier = seed_frontier_2d(vi);
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    let mut flow = SolveFlow::Continue;

    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let candidates: Vec<(u32, u32)> = frontier.dilate(dx, dy).enumerate().collect();
        let results = compute_round(&m, &candidates, nthreads);

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

        if frontier.popcount() > 0 && pacer.due(iters as u64) {
            // ラウンド間なので m は静止している。具現化 (write_back + argmin) は
            // 観測されたときだけ走る。
            let mut mat = |vi: &mut ValueIterator| {
                let opt = final_policy(&m, nthreads);
                m.write_back(vi, Some(&opt));
            };
            let mut probe = MaterializeProbe::new(vi, &mut mat, iters, updates);
            flow = obs.boundary(&mut probe);
            match flow {
                SolveFlow::Continue => {}
                SolveFlow::Stop | SolveFlow::Cancel => break,
            }
        }
    }
    let converged = flow == SolveFlow::Continue && frontier.popcount() == 0;

    if flow == SolveFlow::Cancel {
        // 中断: 場は保証しない契約なので書き戻しをスキップして即戻る。
        return SolveOutcome::cancelled(iters, updates);
    }
    // ── 最終 argmin パス (並列): 現在値から optimal_action を確定。──
    let opt = final_policy(&m, nthreads);
    m.write_back(vi, Some(&opt));
    if flow == SolveFlow::Stop {
        return SolveOutcome::stopped(iters, updates);
    }
    SolveOutcome::running(iters, updates, converged)
}

/// 収束した `hot` から全 free・非 final セルの optimal_action を計算 (並列・読み取り専用)。
/// 返り値はオリジナル座標 index の `Vec<Option<usize>>`。
/// `frontier2d_par_unsafe` も収束後にこの最終 argmin パスを共有する。
/// 並列骨格は [`super::final_policy_parallel`] が担い、ここでは Padded 固有の
/// 評価ガードとコスト関数 (`action_cost_pad`) だけを与える。
pub(crate) fn final_policy(m: &Padded, nthreads: usize) -> Vec<Option<usize>> {
    final_policy_hot(m, m.hot.as_slice(), nthreads)
}

/// `final_policy` の hot 分離版。`frontier2d_par_unsafe` が hot を atomic ビューとして
/// 取り出したまま境界プローブを具現化するときに使う。
pub(crate) fn final_policy_hot<H: HotCells + ?Sized + Sync>(
    m: &Padded,
    hot: &H,
    nthreads: usize,
) -> Vec<Option<usize>> {
    super::final_policy_parallel(
        m.nx,
        m.ny,
        m.nt,
        m.mx,
        m.my,
        m.row_stride,
        &m.precomp,
        nthreads,
        |pad_idx| !m.free[pad_idx] || m.finals[pad_idx],
        |buckets, pad_col| action_cost_pad(hot, &m.free, buckets, pad_col),
    )
}
