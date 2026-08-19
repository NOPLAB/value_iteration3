//! B2: `frontier2d_par` の **非同期 (Gauss-Seidel) unsafe 版**。スレッド間同期を最小化する。
//!
//! 安全版 (`frontier2d_par`) は決定的 Jacobi:
//!  - compute フェーズは共有 `hot` を**読み取り専用**で参照（ラウンド内スナップショット）、
//!  - join 後に**直列**で書き戻し、
//!  - ラウンドごとに `thread::scope` でスレッドを再 spawn。
//!
//! この unsafe 版は「スレッドをまたぐ厳密解の不整合を無視して inter-thread を可能な限り高速化」
//! するため、上記 3 点をすべて崩す:
//!  1. **永続スレッド + 再利用バリア** — ラウンドごとの spawn/join を廃し `std::sync::Barrier` で同期。
//!  2. **in-place 非同期書き込み (Gauss-Seidel)** — compute 中に各スレッドが共有 `hot` へ直接書き込む。
//!     別スレッドの compute はその途中結果（または前ラウンド値）を混在して読む = **厳密解の不整合**。
//!     hot へのアクセスは `[[AtomicU64; 2]]` ビュー経由の Relaxed load/store — x86-64/aarch64 では
//!     素の load/store と同一命令 (ゼロコスト) でありながら、共有参照下の非 atomic レースという
//!     UB を避け、トーン読みも言語レベルで排除する (読めるのは old/new いずれかの完全な値のみ)。
//!  3. **直列 apply の廃止** — 値の確定は compute 内で完結。リーダースレッドは疎な changed 座標から
//!     次フロンティアを再構築するだけ（O(変化セル数)）。
//!  4. **work stealing** — 候補リストは BLOCK 件単位の fetch_add claim で動的分配（障害物近傍の
//!     軽いセルによる負荷不均衡を吸収）。さらに走査方向をラウンド毎に反転（対称 Gauss-Seidel 風）
//!     して逆向きの値伝播も同一ラウンド内で連鎖させる（house でラウンド 122→67、更新 4.5e7→1.5e7）。
//!
//! **なぜ結果は壊れないか**: 各ブロックの claim は一意なので、各セルへの**書き手は常に 1 スレッド**
//! （write-write 競合なし、neighbor の read-write 競合のみ）。VI の Bellman 作用素は単調・固定点一意で、
//! 値は単調減少し真の cost-to-go を下界に持つため、非同期更新でも一意固定点へ収束する
//! (Bertsekas–Tsitsiklis 非同期 VI)。終了は「1 ラウンド丸ごと無変化」で判定するので、停止時には
//! 全到達可能セルが現在の neighbor 値と整合した固定点にある → reference と bit-exact。
//! つまり「不整合」は中間状態・更新回数・収束パスのみで、**最終収束値は安全版と一致**する。
//!
//! `optimal_action` は収束後の最終 argmin パス（`frontier2d_par::final_policy`、並列・読み取り専用）で確定。

use std::sync::atomic::{AtomicU64, Ordering};

use crate::params::MAX_COST;
use crate::solvers::observe::{
    MaterializeProbe, SolveFlow, SolveObserver, SolveOutcome,
};
use crate::value_iterator::ValueIterator;

use super::frontier2d_pad::{action_cost_pad, Padded};
use super::frontier2d_par::{final_policy, final_policy_hot, n_threads};
use super::{async_gs_engine, seed_frontier_2d};

// AtomicU64 は u64 と同一のメモリ表現を持つ (std ドキュメント保証) — `Vec<[u64; 2]>` を
// `&[[AtomicU64; 2]]` として再解釈する前提をコンパイル時に固定する。
const _: () = assert!(
    std::mem::size_of::<[AtomicU64; 2]>() == std::mem::size_of::<[u64; 2]>()
        && std::mem::align_of::<[AtomicU64; 2]>() == std::mem::align_of::<[u64; 2]>()
);

/// セット済み `ValueIterator` を非同期 (Gauss-Seidel) unsafe 並列 frontier2d で解く。
/// `(iters, updates, converged)`。到達可能セルの収束値・方策は安全版と bit-exact。
///
/// 並列骨格 (永続スレッド + バリア×2 + work-stealing + リーダーの次フロンティア再構築) は
/// [`super::async_gs_engine`] が担い、ここでは pad モデル固有の per-cell 評価だけを与える。
pub fn frontier2d_par_unsafe_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    let mut m = Padded::build(vi);
    let (nx, ny, nt) = (m.nx, m.ny, m.nt);
    let (dx, dy) = (m.mx as u32, m.my as u32);
    let nthreads = n_threads();

    // hot を Padded から取り出し、atomic ビューで共有する（&m の不変借用と両立させるため分離）。
    // SAFETY: [AtomicU64; 2] は [u64; 2] とサイズ/アライン一致 (冒頭の const assert)。
    // engine 終了まで hot 本体には触れず、全アクセスはこのビュー経由の Relaxed load/store。
    let mut hot: Vec<[u64; 2]> = std::mem::take(&mut m.hot);
    let n_pad = hot.len();
    let hot_atomic: &[[AtomicU64; 2]] =
        unsafe { std::slice::from_raw_parts(hot.as_mut_ptr().cast::<[AtomicU64; 2]>(), n_pad) };

    let cand_list: Vec<(u32, u32)> = seed_frontier_2d(vi).dilate(dx, dy).enumerate().collect();
    let interval = obs.interval();
    let m_ref = &m;

    // per-cell 評価 (pad モデル): 全 θ を Bellman 更新し、(減少した θ があるか, 減少 θ 層数) を返す。
    // 自セルは単一書き手なので before は最新値 (Relaxed で十分)。
    let eval = |ix: i32, iy: i32| -> (bool, u64) {
        let pad_col = m_ref.pad_col(ix, iy);
        let mut cell_changed = false;
        let mut ups = 0u64;
        for it in 0..nt {
            let pad_idx = (pad_col + it as i64) as usize;
            if !m_ref.free[pad_idx] || m_ref.finals[pad_idx] {
                continue;
            }
            let before = hot_atomic[pad_idx][0].load(Ordering::Relaxed);
            let mut min_cost = MAX_COST;
            for per_theta in m_ref.precomp.iter() {
                let c = action_cost_pad(hot_atomic, &m_ref.free, &per_theta[it as usize], pad_col);
                if c < min_cost {
                    min_cost = c;
                }
            }
            // 値が「動いた」ら伝播 (向きの話は `SolverCaps::resweep` の doc)。
            if min_cost != before {
                hot_atomic[pad_idx][0].store(min_cost, Ordering::Relaxed);
                ups += 1;
                cell_changed = true;
            }
        }
        (cell_changed, ups)
    };

    let out = {
        // 境界はエンジンのリーダー相 (全ワーカーがバリアで静止) から呼ばれるので、
        // hot_atomic の Relaxed 読みで一貫したスナップショットが取れる。
        let mut boundary = |iters: u32, updates: u64| -> SolveFlow {
            let mut mat = |vi: &mut ValueIterator| {
                let opt = final_policy_hot(m_ref, hot_atomic, nthreads);
                m_ref.write_back_hot(vi, hot_atomic, Some(&opt));
            };
            let mut probe = MaterializeProbe::new(vi, &mut mat, iters, updates);
            obs.boundary(&mut probe)
        };
        async_gs_engine(
            nx, ny, dx, dy, nthreads, max_iter, cand_list, eval, interval, &mut boundary,
        )
    };

    // hot を Padded へ戻す。中断 (cancel) 時は場を保証しない契約なので書き戻さない。
    m.hot = hot;
    if out.cancelled {
        return out;
    }
    // 現在値から optimal_action を確定して書き戻す (Stop 時も観測可能な場を残す)。
    let opt = final_policy(&m, nthreads);
    m.write_back(vi, Some(&opt));

    out
}

