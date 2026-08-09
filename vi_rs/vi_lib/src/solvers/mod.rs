//! u64 コストモデル上で動く高速 VI ソルバ群。各ソルバは本家の per-cell 更新
//! `value_iteration_raw` を活性集合に対して呼ぶ。コスト数式は不変なので、到達可能
//! セルの収束値は Reference (全走査) = 本家と bit-exact。
//! 設計: `docs/superpowers/specs/2026-06-09-vi-u64-fast-solvers-design.md`

use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::Barrier;

use crate::params::MAX_COST;
use crate::value_iterator::ValueIterator;

// フロンティアには実績ある word 並列 Bitboard を再利用する（u16 frontier の高速化の源）。
// Bitboard は値の型に非依存なので u64 モデルでもそのまま使える。dilate は theta periodic。
pub(crate) use crate::bitboard::{Bitboard2D, Bitboard3D};

pub mod block;
#[cfg(test)]
mod conformance;
pub mod frontier2d;
pub mod frontier2d_pad;
pub mod frontier2d_par;
pub mod frontier2d_fused;
pub mod frontier2d_sparse;
pub mod frontier2d_sparse_compact;
pub mod frontier2d_par_unsafe;
pub mod frontier3d;
pub mod observe;
pub mod original;
pub mod pyramid;
pub mod stack;
pub mod stream;
pub mod priority;

pub use observe::{NullObserver, SolveFlow, SolveObserver, SolveOutcome, SolveProbe};

/// dilation 変位 `(mx, my, mt)` を `actions` の全遷移から算出する。`dit` は絶対 θ なので、
/// 各 (action, source theta `t`) について循環距離 `min(|dit-t|, nt-|dit-t|)` を取り `mt` とする。
/// これは「あるセルが変化したとき再評価が必要な前駆セル集合」の正しい上位集合を与える。
pub(crate) fn displacement(vi: &ValueIterator) -> (i32, i32, i32) {
    let nt = vi.cell_num_t;
    let (mut mx, mut my, mut mt) = (0i32, 0i32, 0i32);
    for a in &vi.actions {
        for (t, trans) in a.state_transitions.iter().enumerate() {
            for st in trans {
                mx = mx.max(st.dix.abs());
                my = my.max(st.diy.abs());
                let raw = (st.dit - t as i32).rem_euclid(nt);
                let circ = raw.min(nt - raw);
                mt = mt.max(circ);
            }
        }
    }
    (mx.max(1), my.max(1), mt)
}

/// 初期フロンティア種: `total_cost < MAX_COST` のセル（`set_goal` 後の `final_state` セル）。
pub(crate) fn seed_frontier(vi: &ValueIterator) -> Bitboard3D {
    let mut bb = Bitboard3D::new(vi.cell_num_x as u32, vi.cell_num_y as u32, vi.cell_num_t as u32);
    for s in &vi.states {
        if s.total_cost < MAX_COST {
            bb.set(s.ix as u32, s.iy as u32, s.it as u32);
        }
    }
    bb
}

/// 初期フロンティア種 (2D): いずれかの θ で `total_cost < MAX_COST` の (ix,iy)。
pub(crate) fn seed_frontier_2d(vi: &ValueIterator) -> Bitboard2D {
    let mut bb = Bitboard2D::new(vi.cell_num_x as u32, vi.cell_num_y as u32);
    for s in &vi.states {
        if s.total_cost < MAX_COST {
            bb.set(s.ix as u32, s.iy as u32);
        }
    }
    bb
}

/// 収束後の最終 argmin パス（並列・読み取り専用）の共通実装。frontier2d_par /
/// frontier2d_fused（および各々を呼ぶ par_unsafe / sparse）が共有する「全 (ix,iy,it) を
/// 行バンド並列に走査し、free・非 final セルの optimal_action を argmin で確定する」骨格を
/// まとめる。差分は評価対象判定 `skip(pad_idx)` とアクションコスト `cost(buckets, pad_col)` の2点。
///
/// `pad_col = (ix+mx)·nt + (iy+my)·row_stride`（`Padded`/`Geom` の `pad_col` と同一式）。
/// `precomp[ai][it]` は `(action, source θ)` ごとの隣接 `(相対オフセット, prob)`。
/// 返り値はオリジナル座標 index の `Vec<Option<usize>>`。
#[allow(clippy::too_many_arguments)]
pub(crate) fn final_policy_parallel<S, C>(
    nx: i32,
    ny: i32,
    nt: i32,
    mx: i32,
    my: i32,
    row_stride: i64,
    precomp: &[Vec<Vec<(i64, u64)>>],
    nthreads: usize,
    skip: S,
    cost: C,
) -> Vec<Option<usize>>
where
    S: Fn(usize) -> bool + Sync,
    C: Fn(&[(i64, u64)], i64) -> u64 + Sync,
{
    let n = (nx * ny * nt) as usize;
    let rows: Vec<i32> = (0..ny).collect();
    let chunk = rows.len().div_ceil(nthreads).max(1);
    let skip = &skip;
    let cost = &cost;

    let parts: Vec<Vec<(usize, Option<usize>)>> = std::thread::scope(|scope| {
        let handles: Vec<_> = rows
            .chunks(chunk)
            .map(|band| {
                scope.spawn(move || {
                    let mut out: Vec<(usize, Option<usize>)> = Vec::new();
                    for &iy in band {
                        for ix in 0..nx {
                            let pad_col =
                                (ix + mx) as i64 * nt as i64 + (iy + my) as i64 * row_stride;
                            let orig_col = (ix * nt + iy * (nt * nx)) as usize;
                            for it in 0..nt {
                                let pad_idx = (pad_col + it as i64) as usize;
                                if skip(pad_idx) {
                                    continue;
                                }
                                let mut min_cost = MAX_COST;
                                let mut min_action: Option<usize> = None;
                                for (ai, per_theta) in precomp.iter().enumerate() {
                                    let c = cost(&per_theta[it as usize], pad_col);
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

/// 非同期 (Gauss-Seidel) 並列フロンティアエンジンがスレッド間で共有する生ポインタ束。
/// 永続ワーカーが Copy で持つ。全アクセスはバリアで相分離し「単一書き手 + バリア後読み」を守る。
/// - `cand`: 今ラウンドの候補セル。リーダーが B1〜B2 間で差し替え、ワーカーは compute 相でのみ読む。
/// - `changed`: 長さ `nthreads` の配列の先頭。compute 相でワーカー `w` が `changed[w]` のみ書く。
#[derive(Clone, Copy)]
struct GsShared {
    cand: *mut Vec<(u32, u32)>,
    changed: *mut Vec<(u32, u32)>,
}
// SAFETY: 上記のとおり全アクセスはバリアで相分離され、単一書き手 + バリア後読みの規律を守る。
unsafe impl Send for GsShared {}
unsafe impl Sync for GsShared {}

/// 非同期 (Gauss-Seidel) unsafe 並列フロンティアの共通エンジン。`frontier2d_par_unsafe`
/// (pad モデル) と `frontier2d_fused` (cp 融合モデル) が共有する並列骨格 ——
/// 永続スレッド + 再利用バリア×2/ラウンド、work-stealing (BLOCK 件の `fetch_add` claim)、
/// in-place 非同期書き込み (各セルの書き手は claim により常に 1 スレッド)、リーダー (w==0) による
/// changed→次フロンティア再構築・走査方向のラウンド毎反転・終了判定 —— を 1 箇所にまとめる。
/// 2 モデルの差は per-cell 評価 `eval(ix, iy) -> (changed, updates)` だけで、データビュー
/// (`[[AtomicU64; 2]]` / `[AtomicU64]`) と数式は `eval` クロージャに閉じ込める。
///
/// `eval` は候補セル `(ix,iy)` の全 θ 層を Bellman 更新し、`(値を下げた θ があるか, 減少 θ 層数)`
/// を返す（共有 atomic ビューへの書き込みは `eval` 内で行う）。`Fn + Sync` なので全ワーカーで共有
/// できる。`(iters, updates, converged)` を返す。
///
/// θ マスク疎評価版 (`frontier2d_sparse`) は changed が θマスクを運び、リーダーがマスク配列を
/// 管理するため、このエンジンは共有せず独自実装を持つ。
#[allow(clippy::too_many_arguments)]
pub(crate) fn async_gs_engine<F>(
    nx: i32,
    ny: i32,
    dx: u32,
    dy: u32,
    nthreads: usize,
    max_iter: u32,
    mut cand_list: Vec<(u32, u32)>,
    eval: F,
    obs_interval: u32,
    boundary: &mut dyn FnMut(u32, u64) -> SolveFlow,
) -> SolveOutcome
where
    F: Fn(i32, i32) -> (bool, u64) + Sync,
{
    let mut changed_lists: Vec<Vec<(u32, u32)>> = vec![Vec::new(); nthreads];
    let shared = GsShared {
        cand: &mut cand_list as *mut Vec<(u32, u32)>,
        changed: changed_lists.as_mut_ptr(),
    };

    let barrier = Barrier::new(nthreads);
    let done = AtomicBool::new(false);
    // work-stealing カーソル: 候補リストを BLOCK 件単位で fetch_add により動的分配する。
    let cursor = AtomicUsize::new(0);
    // 境界プローブが読む「これまでの更新総数」。各スレッドが自ラウンド分を B1 前に足す。
    let updates_total = AtomicU64::new(0);
    let eval = &eval;

    /// compute 相 1 ラウンド分 (work stealing で候補を claim して評価)。ラウンドの
    /// 自スレッド更新数を返す。
    /// SAFETY: バリアで相分離された compute 相 (B2 後〜B1 前) からのみ呼ぶこと —
    /// cand はリーダーの差し替え (B1〜B2 間) と競合せず、changed[w] は w だけが触る。
    unsafe fn compute_phase<F: Fn(i32, i32) -> (bool, u64)>(
        shared: GsShared,
        cursor: &AtomicUsize,
        eval: &F,
        w: usize,
    ) -> u64 {
        let cand = unsafe { &*shared.cand };
        let n = cand.len();
        let my_changed = unsafe { &mut *shared.changed.add(w) };
        my_changed.clear();
        let mut ups_round = 0u64;
        // work stealing: BLOCK 件の連続ブロックを fetch_add で claim する。
        // 各ブロックの claim は一意なので「セルの書き手は 1 スレッド」が保たれる。
        const BLOCK: usize = 16;
        loop {
            let s = cursor.fetch_add(BLOCK, Ordering::Relaxed);
            if s >= n {
                break;
            }
            let e = (s + BLOCK).min(n);
            for &(ixu, iyu) in &cand[s..e] {
                let (cell_changed, ups) = eval(ixu as i32, iyu as i32);
                ups_round += ups;
                if cell_changed {
                    my_changed.push((ixu, iyu));
                }
            }
        }
        ups_round
    }

    std::thread::scope(|scope| {
        // ワーカー (w=1..nthreads)。compute とバリアだけを回す。
        for w in 1..nthreads {
            let barrier = &barrier;
            let done = &done;
            let cursor = &cursor;
            let updates_total = &updates_total;
            scope.spawn(move || {
                // `GsShared` 全体を再束縛してクロージャに「構造体まるごと」をキャプチャさせる
                // (Rust 2021 のフィールド分割キャプチャだと生ポインタ単体が捕まり Send にならない)。
                #[allow(clippy::redundant_locals)]
                let shared = shared;
                loop {
                    // SAFETY: compute 相 (B2 後〜B1 前)。
                    let ups = unsafe { compute_phase(shared, cursor, eval, w) };
                    if ups > 0 {
                        updates_total.fetch_add(ups, Ordering::Relaxed);
                    }
                    barrier.wait(); // B1: 全 hot/changed 書き込みが可視。
                    barrier.wait(); // B2: リーダーの cand 差し替え / done が可視。
                    if done.load(Ordering::Relaxed) {
                        break;
                    }
                }
            });
        }

        // リーダー (w=0) は**呼び出しスレッドでインライン実行**する。boundary フックは
        // ここから呼ばれるので、observer は Send である必要がない。
        let interval = obs_interval.max(1) as u64;
        let mut next_boundary: u64 = interval;
        let mut iter_count: u32 = 0;
        let mut outcome: Option<SolveOutcome> = None;
        loop {
            // SAFETY: compute 相 (B2 後〜B1 前)。
            let ups = unsafe { compute_phase(shared, &cursor, eval, 0) };
            if ups > 0 {
                updates_total.fetch_add(ups, Ordering::Relaxed);
            }
            barrier.wait(); // B1: 全 hot/changed 書き込みが可視。

            // ── リーダー直列: changed → 次フロンティア再構築 / 境界 / 終了判定 ──
            // ここでは全ワーカーが B2 で待機している (compute 書き込みは静止) ので、
            // boundary の具現化 (write_back / argmin) は共有配列を安全に読める。
            iter_count += 1;
            let mut any = false;
            let mut nf = Bitboard2D::new(nx as u32, ny as u32);
            for i in 0..nthreads {
                // SAFETY: B1 後、各 changed[i] への書きは完了し可視。
                let cl = unsafe { &*shared.changed.add(i) };
                if !cl.is_empty() {
                    any = true;
                }
                for &(ixu, iyu) in cl {
                    nf.set(ixu, iyu);
                }
            }
            let updates_now = updates_total.load(Ordering::Relaxed);
            let mut flow = SolveFlow::Continue;
            if any && iter_count < max_iter && iter_count as u64 >= next_boundary {
                next_boundary = (iter_count as u64).saturating_add(interval);
                flow = boundary(iter_count, updates_now);
            }
            if any && iter_count < max_iter && flow == SolveFlow::Continue {
                let mut next: Vec<(u32, u32)> = nf.dilate(dx, dy).enumerate().collect();
                // 対称 Gauss-Seidel 風: 走査方向をラウンドごとに反転。
                if iter_count % 2 == 1 {
                    next.reverse();
                }
                // SAFETY: 他ワーカーは B1〜B2 間 cand を読まない。
                unsafe {
                    *shared.cand = next;
                }
                cursor.store(0, Ordering::Relaxed);
            } else {
                outcome = Some(match flow {
                    SolveFlow::Stop => SolveOutcome::stopped(iter_count, updates_now),
                    SolveFlow::Cancel => SolveOutcome::cancelled(iter_count, updates_now),
                    SolveFlow::Continue => {
                        SolveOutcome::running(iter_count, updates_now, !any)
                    }
                });
                done.store(true, Ordering::Relaxed);
            }

            barrier.wait(); // B2: リーダーの cand 差し替え / done が可視。
            if done.load(Ordering::Relaxed) {
                break;
            }
        }
        outcome.expect("engine loop sets outcome before exiting")
    })
}

/// 到達可能とみなす total_cost 上限（compare.py の value>=1e6 境界と整合）。
pub const REACH_THRESH: u64 = 1_000_000u64 * crate::params::PROB_BASE;

/// u64 高速ソルバの種別。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum U64Solver {
    Reference,
    Frontier3D,
    Frontier2D,
    Frontier2DPar,
    Frontier2DParUnsafe,
    Frontier2DFused,
    Frontier2DSparse,
    /// アウトオブコア版。`band`=0 で auto（結合深さ安全側）、>0 で値バンド幅を明示（メモリ予算つまみ）。
    Frontier2DSparseCompact { band: u64 },
    FrontierStack,
    BlockRefine,
    PyramidSweep,
    StreamMimic,
    PriorityLabelSetting,
    PriorityLabelCorrecting,
}

impl U64Solver {
    pub fn from_name(s: &str) -> Option<Self> {
        Some(match s {
            "reference" => U64Solver::Reference,
            "frontier3d" => U64Solver::Frontier3D,
            "frontier2d" => U64Solver::Frontier2D,
            "frontier2d_par" => U64Solver::Frontier2DPar,
            "frontier2d_par_unsafe" => U64Solver::Frontier2DParUnsafe,
            "frontier2d_fused" => U64Solver::Frontier2DFused,
            "frontier2d_sparse" => U64Solver::Frontier2DSparse,
            "frontier2d_sparse_compact" => U64Solver::Frontier2DSparseCompact { band: 0 },
            "frontier_stack" => U64Solver::FrontierStack,
            "block_refine" => U64Solver::BlockRefine,
            "pyramid_sweep" => U64Solver::PyramidSweep,
            "stream_mimic" => U64Solver::StreamMimic,
            "prio_ls" => U64Solver::PriorityLabelSetting,
            "prio_lc" => U64Solver::PriorityLabelCorrecting,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct U64SolveStats {
    pub iters: u32,
    pub updates: u64,
    pub converged: bool,
}

/// 途中で打ち切った場 (Stop / max_iter 到達) の性質。early_start の健全性議論と
/// conformance テストの期待切り替えに使う。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Partiality {
    /// 途中の場は全セル V ≥ v* の単調上界 (MAX_COST から妥当な Bellman 更新のみで降下)。
    /// その場の上の貪欲降下は自分の値を厳密に下げながら進むので循環しない
    /// (最短とは限らない)。
    UpperBound,
    /// 確定済みセルは v* に bit-exact、未確定セルは `(MAX_COST, 方策なし)`
    /// (compact: finalize は値の昇順に進むため)。
    ExactPrefix,
}

/// ソルバの能力宣言。プランナ側のハードコード
/// (`matches!(solver, Frontier2DSparseCompact {..})` など) をここへ集約する。
#[derive(Clone, Copy, Debug)]
pub struct SolverCaps {
    /// 収束値・方策が本家 (Reference 固定点) と bit-exact か。false は近似ソルバ
    /// (実パラメタ付き tau/topk、settle-once の prio_ls)。
    pub exact: bool,
    /// 途中で打ち切った場の性質。
    pub partial: Partiality,
    /// `states` を確保しないアウトオブコア経路か (出力は `CompactSink`)。
    pub out_of_core: bool,
    /// マルチスレッド (`VI_THREADS`) を使うか。
    pub parallel: bool,
}

impl U64Solver {
    /// このソルバ (パラメタ込み) の能力。
    pub fn caps(&self) -> SolverCaps {
        use U64Solver::*;
        // settle-once (Dijkstra 流) は確率遷移では早すぎる確定があり得る → 近似。
        let exact = !matches!(*self, PriorityLabelSetting);
        let partial = match *self {
            Frontier2DSparseCompact { .. } => Partiality::ExactPrefix,
            _ => Partiality::UpperBound,
        };
        SolverCaps {
            exact,
            partial,
            out_of_core: matches!(self, Frontier2DSparseCompact { .. }),
            parallel: matches!(
                self,
                Frontier2DPar
                    | Frontier2DParUnsafe
                    | Frontier2DFused
                    | Frontier2DSparse
                    | Frontier2DSparseCompact { .. }
            ),
        }
    }
}

/// セット済み `ValueIterator` を指定ソルバで収束まで解く (従来 API)。
/// `solve_observed` の [`NullObserver`] ラッパ。
pub fn solve(vi: &mut ValueIterator, solver: U64Solver, max_iter: u32) -> U64SolveStats {
    let o = solve_observed(vi, solver, max_iter, &mut NullObserver);
    U64SolveStats { iters: o.iters, updates: o.updates, converged: o.converged }
}

/// セット済み `ValueIterator` を指定ソルバで解き、進行を `obs` に観測させる。
/// 各ソルバは自然な境界 (ラウンド / スイープ / ブロックパス / バンド finalize /
/// 一定 pop 数) ごとに [`SolveObserver::boundary`] を呼ぶ。詳細は [`observe`]。
pub fn solve_observed(
    vi: &mut ValueIterator,
    solver: U64Solver,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    match solver {
        U64Solver::Reference => original::original_solve_observed(vi, max_iter, obs),
        U64Solver::Frontier3D => frontier3d::frontier3d_solve_observed(vi, max_iter, obs),
        U64Solver::Frontier2D => frontier2d::frontier2d_solve_observed(vi, max_iter, obs),
        U64Solver::Frontier2DPar => {
            frontier2d_par::frontier2d_par_solve_observed(vi, max_iter, obs)
        }
        U64Solver::Frontier2DFused => {
            frontier2d_fused::frontier2d_fused_solve_observed(vi, max_iter, obs)
        }
        U64Solver::Frontier2DSparse => {
            frontier2d_sparse::frontier2d_sparse_solve_observed(vi, max_iter, obs)
        }
        U64Solver::Frontier2DSparseCompact { band } => {
            let s = frontier2d_sparse_compact::solve_compact_observed(
                vi,
                max_iter,
                if band == 0 { None } else { Some(band) },
                obs,
            );
            SolveOutcome {
                iters: s.iters,
                updates: s.updates,
                converged: s.converged,
                stopped: s.stopped,
                cancelled: s.cancelled,
            }
        }
        U64Solver::Frontier2DParUnsafe => {
            frontier2d_par_unsafe::frontier2d_par_unsafe_solve_observed(vi, max_iter, obs)
        }
        U64Solver::FrontierStack => stack::frontier_stack_solve_observed(vi, max_iter, obs),
        U64Solver::BlockRefine => block::block_refine_solve_observed(vi, max_iter, obs),
        U64Solver::PyramidSweep => pyramid::pyramid_sweep_solve_observed(vi, max_iter, obs),
        U64Solver::StreamMimic => stream::stream_mimic_solve_observed(vi, max_iter, obs),
        U64Solver::PriorityLabelSetting => {
            priority::priority_solve_observed(vi, max_iter, true, obs)
        }
        U64Solver::PriorityLabelCorrecting => {
            priority::priority_solve_observed(vi, max_iter, false, obs)
        }
    }
}

/// フロンティア/ブロック系ソルバの parity テスト共有ヘルパ。
#[cfg(test)]
pub(crate) mod test_support {
    use crate::action::Action;
    use crate::msg::OccupancyGrid;
    use crate::value_iterator::ValueIterator;

    pub(crate) use super::REACH_THRESH as REACH;

    pub(crate) fn actions() -> Vec<Action> {
        vec![
            Action::new("forward", 0.3, 0.0, 0),
            Action::new("back", -0.2, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2),
            Action::new("rightfw", 0.2, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4),
            Action::new("leftfw", 0.2, 20.0, 5),
        ]
    }

    pub(crate) fn make_vi(w: i32, h: i32, occ: Vec<i8>) -> ValueIterator {
        let mut vi = ValueIterator::new(actions(), 1);
        let map = OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: occ,
        };
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.3, 15);
        vi.set_goal(0.10, 0.10, 0);
        vi
    }

    /// Reference 全走査を strict 固定点（到達可能セルが変化しなくなる）まで回す。
    pub(crate) fn run_reference_to_fixed_point(vi: &mut ValueIterator) {
        let mut prev: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
        for _ in 0..2000 {
            vi.value_iteration_worker(1, 0);
            let mut changed = false;
            for (i, s) in vi.states.iter().enumerate() {
                if s.total_cost < REACH && s.total_cost != prev[i] {
                    changed = true;
                }
                prev[i] = s.total_cost;
            }
            if !changed {
                break;
            }
        }
    }

    /// `solve_fn` で解いた結果が Reference 固定点と到達可能セルで bit 一致することを assert。
    pub(crate) fn assert_parity<F>(w: i32, h: i32, occ: Vec<i8>, solve_fn: F)
    where
        F: Fn(&mut ValueIterator) -> (u32, u64, bool),
    {
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);
        let (_i, _u, converged) = solve_fn(&mut b);
        assert!(converged, "solver must converge");
        let mut n_reach = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH {
                n_reach += 1;
                assert_eq!(
                    a.states[i].total_cost, b.states[i].total_cost,
                    "total_cost mismatch @ state {i} (ix={},iy={},it={})",
                    a.states[i].ix, a.states[i].iy, a.states[i].it
                );
                assert_eq!(
                    a.states[i].optimal_action, b.states[i].optimal_action,
                    "policy mismatch @ state {i} (ix={},iy={},it={})",
                    a.states[i].ix, a.states[i].iy, a.states[i].it
                );
            }
        }
        assert!(n_reach > 0, "到達可能セルが存在するはず");
    }

    /// 標準の 3 マップ (empty / obstacle: x=5 の縦壁に隙間 / sentinel: goal を 3 方向で囲む)。
    /// `(name, w, h, occupancy)`。conformance / priority / parity テストの共通データ。
    pub(crate) fn standard_maps() -> Vec<(&'static str, i32, i32, Vec<i8>)> {
        let empty = vec![0i8; 64];
        let mut obstacle = vec![0i8; 64];
        for iy in 0..8 {
            obstacle[(iy * 8 + 5) as usize] = 100;
        }
        obstacle[5] = 0;
        let mut sentinel = vec![0i8; 64];
        sentinel[8 + 2] = 100;
        sentinel[3 * 8 + 2] = 100;
        sentinel[2 * 8 + 1] = 100;
        vec![
            ("empty", 8, 8, empty),
            ("obstacle", 8, 8, obstacle),
            ("sentinel", 8, 8, sentinel),
        ]
    }

    /// 標準の3マップ (empty / obstacle / sentinel) で parity を検証する共通テスト本体。
    pub(crate) fn parity_standard_maps<F>(solve_fn: F)
    where
        F: Fn(&mut ValueIterator) -> (u32, u64, bool) + Copy,
    {
        for (_name, w, h, occ) in standard_maps() {
            assert_parity(w, h, occ, solve_fn);
        }
    }
}

#[cfg(test)]
mod helper_tests {
    use super::*;
    use crate::value_iterator::ValueIterator;

    fn small_vi() -> ValueIterator {
        test_support::make_vi(5, 5, vec![0i8; 25])
    }

    #[test]
    fn displacement_is_bounded_and_positive() {
        let vi = small_vi();
        let (mx, my, mt) = displacement(&vi);
        assert!(mx >= 1 && my >= 1);
        assert!(mt >= 0 && mt < vi.cell_num_t);
    }

    #[test]
    fn seed_contains_goal_cells() {
        let vi = small_vi();
        let seed = seed_frontier(&vi);
        let n_final = vi.states.iter().filter(|s| s.total_cost < crate::params::MAX_COST).count();
        assert!(n_final > 0, "goal セルが存在するはず");
        assert_eq!(seed.popcount(), n_final as u64);
    }

    #[test]
    fn solve_reference_and_frontier3d_agree() {
        let mut a = small_vi();
        let mut b = small_vi();
        solve(&mut a, U64Solver::Reference, 2000);
        solve(&mut b, U64Solver::Frontier3D, 2000);
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH_THRESH {
                assert_eq!(a.states[i].total_cost, b.states[i].total_cost, "@ {i}");
                assert_eq!(a.states[i].optimal_action, b.states[i].optimal_action, "@ {i}");
            }
        }
    }

    #[test]
    fn solver_from_str() {
        assert!(matches!(U64Solver::from_name("frontier3d"), Some(U64Solver::Frontier3D)));
        assert!(matches!(U64Solver::from_name("reference"), Some(U64Solver::Reference)));
        assert!(U64Solver::from_name("nope").is_none());
    }
}
