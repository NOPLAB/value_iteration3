//! B3: `frontier2d_par_unsafe` (非同期 G-S) に **penalty 融合レイアウト (cp)** を重ねた版。
//!
//! Step0 計測のとおり frontier 系は per-bucket メモリ帯域律速。本家の Q 計算は毎バケット
//! `(total_cost + penalty + local_penalty) * prob` を読むので、セルごとに不変な
//! `pen = penalty +ʷ local_penalty` を値へ**事前合成**した単一 u64 配列
//! `cp[i] = total_cost +ʷ pen` を持てば、バケットあたりのロードが
//! `hot 16B + free 1B = 17B` → **`cp 8B` (53% 減)** になり、加算 1 回と free 分岐も消える。
//!
//! - **番兵**: 非 free セルと未確定 (total_cost==MAX_COST) セルは `cp = u64::MAX`。
//!   実セルの cp は `< MAX_COST + 2.7e10 ≪ u64::MAX` なので衝突しない。
//!   `action_cost_pad` の「!free → MAX_COST」と「未確定クランプ → MAX_COST」は
//!   同条件・同位置・同返り値の **1 分岐 `cp == u64::MAX`** に統合される。
//! - **bit-exact**: Q = Σ (cost +ʷ pen) ·ʷ prob ≫ 18 は加算の結合順まで本家と同一
//!   (cp は加算を先に固めただけ)。自セルの現在値は `before = cp −ʷ pen` で正確に復元
//!   (mod 2^64 の加減算は可逆)。よって更新判定・収束値・方策とも
//!   `frontier2d_par`/`frontier2d_par_unsafe` と bit 単位で一致する。
//! - **固定費の削減** (フェーズ計測で build/policy/writeback が時間の ~1/3 だった):
//!   `Padded` の中間 `hot` 配列 (16B×n_pad) を作らず、states から cp/pen/eval_ok を
//!   **直接・行バンド並列**で構築する (`Fused::build_direct` + 軽量 `Geom`)。
//!   書き戻しも行バンド並列 (`write_back_fused`)。
//! - 並列構造 (永続スレッド + バリア×2/round + work stealing + 走査方向ラウンド毎反転 +
//!   Relaxed atomic ビュー) は `frontier2d_par_unsafe` と同一。正しさの議論もそのまま:
//!   書き手は claim ブロックにより常に 1 スレッド、値は単調減少・真固定点が下界、
//!   終了は「1 ラウンド丸ごと無変化」(そのラウンドは並行書き込みゼロ = Bellman 整合)。

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::Barrier;

use crate::params::{MAX_COST, PROB_BASE_BIT};
use crate::value_iterator::ValueIterator;

use super::frontier2d_pad::build_precomp;
use super::frontier2d_par::n_threads;
use super::{displacement, seed_frontier_2d, Bitboard2D};

/// 未確定/非 free の番兵。実 cp (< MAX_COST + pen 上限) と衝突しない。
pub(crate) const UNREACHED: u64 = u64::MAX;

// AtomicU64 は u64 と同一のメモリ表現 (frontier2d_par_unsafe と同じ前提)。
const _: () = assert!(
    std::mem::size_of::<AtomicU64>() == std::mem::size_of::<u64>()
        && std::mem::align_of::<AtomicU64>() == std::mem::align_of::<u64>()
);

/// `cp` 配列の読み出し抽象 (`HotCells` の cp 版)。
pub(crate) trait CpCells {
    fn get(&self, i: usize) -> u64;
}
impl CpCells for [u64] {
    #[inline(always)]
    fn get(&self, i: usize) -> u64 {
        self[i]
    }
}
impl CpCells for [AtomicU64] {
    #[inline(always)]
    fn get(&self, i: usize) -> u64 {
        self[i].load(Ordering::Relaxed)
    }
}

/// 本家 `actionCost` の cp 融合版。返り値・短絡位置は `action_cost_pad` と一致。
#[inline]
pub(crate) fn action_cost_fused<C: CpCells + ?Sized>(
    cp: &C,
    buckets: &[(i64, u64)],
    col_base: i64,
) -> u64 {
    let mut cost: u64 = 0;
    for &(off, prob) in buckets {
        let v = cp.get((col_base + off) as usize); // パディングにより常に有効域
        if v == UNREACHED {
            return MAX_COST; // !free / 未確定の統合クランプ
        }
        cost = cost.wrapping_add(v.wrapping_mul(prob));
    }
    cost >> PROB_BASE_BIT
}

/// パディング座標系の幾何 + 遷移オフセットだけの軽量モデル (`Padded` から hot/free/finals を
/// 除いたもの)。fused/sparse 系は states→cp 直接構築なので中間配列を持たない。
pub(crate) struct Geom {
    pub(crate) precomp: Vec<Vec<Vec<(i64, u64)>>>,
    pub(crate) nx: i32,
    pub(crate) ny: i32,
    pub(crate) nt: i32,
    pub(crate) mx: i32,
    pub(crate) my: i32,
    pub(crate) row_stride: i64,
}

impl Geom {
    pub(crate) fn build(vi: &ValueIterator) -> Self {
        let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
        let (mx, my, _mt) = displacement(vi);
        let nx_pad = nx + 2 * mx;
        let row_stride = (nt * nx_pad) as i64;
        let precomp = build_precomp(vi, nt, row_stride);
        Self { precomp, nx, ny, nt, mx, my, row_stride }
    }

    /// セル (ix,iy) の θ=0 列ベース (パディング座標フラット)。
    #[inline]
    pub(crate) fn pad_col(&self, ix: i32, iy: i32) -> i64 {
        (ix + self.mx) as i64 * self.nt as i64 + (iy + self.my) as i64 * self.row_stride
    }

    pub(crate) fn n_pad(&self) -> usize {
        (self.row_stride * (self.ny + 2 * self.my) as i64) as usize
    }
}

/// penalty 融合モデル。
pub(crate) struct Fused {
    /// `total_cost +ʷ pen`、未確定/非 free は `UNREACHED`。
    pub(crate) cp: Vec<u64>,
    /// `penalty +ʷ local_penalty` (自セルの before 復元・書き戻し用、読み取り専用)。
    pub(crate) pen: Vec<u64>,
    /// free かつ非 final = Bellman 更新対象 (外側ループの 1 ロードに統合)。
    pub(crate) eval_ok: Vec<bool>,
}

impl Fused {
    /// states から cp/pen/eval_ok を直接構築 (行バンド並列、中間 hot なし)。
    /// states は iy-major 連続なので、y 行バンド ↔ pad 行バンドが両方連続スライスになる。
    pub(crate) fn build_direct(vi: &ValueIterator, g: &Geom) -> Self {
        let (nx, ny, nt) = (g.nx, g.ny, g.nt);
        let n_pad = g.n_pad();
        let mut cp = vec![UNREACHED; n_pad];
        let mut pen = vec![0u64; n_pad];
        let mut eval_ok = vec![false; n_pad];

        let row_pad = g.row_stride as usize; // pad 1 行分
        let row_st = (nx * nt) as usize; // states 1 行分
        let nthreads = n_threads();
        let band = (ny as usize).div_ceil(nthreads).max(1);

        std::thread::scope(|scope| {
            // 上端パディング my 行はデフォルト値のまま飛ばす。
            let mut cp_s = &mut cp[row_pad * g.my as usize..];
            let mut pen_s = &mut pen[row_pad * g.my as usize..];
            let mut ev_s = &mut eval_ok[row_pad * g.my as usize..];
            let mut st_s: &[crate::state::State] = &vi.states;
            let mut y0 = 0usize;
            while y0 < ny as usize {
                let rows = band.min(ny as usize - y0);
                let (cp_b, cp_r) = std::mem::take(&mut cp_s).split_at_mut(rows * row_pad);
                cp_s = cp_r;
                let (pen_b, pen_r) = std::mem::take(&mut pen_s).split_at_mut(rows * row_pad);
                pen_s = pen_r;
                let (ev_b, ev_r) = std::mem::take(&mut ev_s).split_at_mut(rows * row_pad);
                ev_s = ev_r;
                let (st_b, st_r) = st_s.split_at(rows * row_st);
                st_s = st_r;
                let mxnt = (g.mx * nt) as usize;
                scope.spawn(move || {
                    for y in 0..rows {
                        let pad_row = y * row_pad + mxnt; // 左 x パディングを跳ねる
                        let st_row = y * row_st;
                        for k in 0..row_st {
                            // states は (it + ix·nt) 連続 = pad 側も同順で連続。
                            let s = &st_b[st_row + k];
                            let i = pad_row + k;
                            let p = s.penalty.wrapping_add(s.local_penalty);
                            pen_b[i] = p;
                            if s.free && s.total_cost != MAX_COST {
                                cp_b[i] = s.total_cost.wrapping_add(p);
                            }
                            ev_b[i] = s.free && !s.final_state;
                        }
                    }
                });
                y0 += rows;
            }
        });

        Self { cp, pen, eval_ok }
    }
}

/// 収束した `cp` を states へ書き戻す (行バンド並列)。`opt` は optimal_action。
pub(crate) fn write_back_fused(
    vi: &mut ValueIterator,
    g: &Geom,
    f: &Fused,
    opt: &[Option<usize>],
) {
    let (nx, nt) = (g.nx, g.nt);
    let row_st = (nx * nt) as usize;
    let nthreads = n_threads();
    let band = ((vi.cell_num_y as usize).div_ceil(nthreads).max(1)) * row_st;

    std::thread::scope(|scope| {
        for (ci, chunk) in vi.states.chunks_mut(band).enumerate() {
            let base = ci * band;
            scope.spawn(move || {
                for (k, s) in chunk.iter_mut().enumerate() {
                    let orig = base + k;
                    let pad_idx =
                        (s.it as i64 + g.pad_col(s.ix, s.iy)) as usize;
                    s.total_cost = if f.cp[pad_idx] == UNREACHED {
                        MAX_COST
                    } else {
                        f.cp[pad_idx].wrapping_sub(f.pen[pad_idx])
                    };
                    s.optimal_action = opt[orig];
                }
            });
        }
    });
}

/// 収束した `cp` から全 free・非 final セルの optimal_action を計算 (並列・読み取り専用)。
/// `skip_unreached` (収束時のみ true にする): 固定点で `cp==UNREACHED` のセルは全アクション
/// Q=MAX_COST → argmin=None なので評価を省略できる (非収束時は途中波面で差が出るため不可)。
pub(crate) fn final_policy_fused(
    g: &Geom,
    f: &Fused,
    nthreads: usize,
    skip_unreached: bool,
) -> Vec<Option<usize>> {
    let (nx, ny, nt) = (g.nx, g.ny, g.nt);
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
                            let pad_col = g.pad_col(ix, iy);
                            let orig_col = (ix * nt + iy * (nt * nx)) as usize;
                            for it in 0..nt {
                                let pad_idx = (pad_col + it as i64) as usize;
                                if !f.eval_ok[pad_idx]
                                    || (skip_unreached && f.cp[pad_idx] == UNREACHED)
                                {
                                    continue;
                                }
                                let mut min_cost = MAX_COST;
                                let mut min_action: Option<usize> = None;
                                for (ai, per_theta) in g.precomp.iter().enumerate() {
                                    let c = action_cost_fused(
                                        f.cp.as_slice(),
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

/// `frontier2d_par_unsafe` と同じ共有ポインタ束 (cand/changed、バリアで相分離)。
#[derive(Clone, Copy)]
struct Shared {
    cand: *mut Vec<(u32, u32)>,
    changed: *mut Vec<(u32, u32)>,
}
// SAFETY: 全アクセスはバリアで相分離され、「単一書き手 + バリア後読み」の規律を守る。
unsafe impl Send for Shared {}
unsafe impl Sync for Shared {}

/// セット済み `ValueIterator` を penalty 融合 + 非同期 G-S 並列 frontier2d で解く。
/// `(iters, updates, converged)`。到達可能セルの収束値・方策は本家と bit-exact。
pub fn frontier2d_fused_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let g = Geom::build(vi);
    let (nx, ny, nt) = (g.nx, g.ny, g.nt);
    let (dx, dy) = (g.mx as u32, g.my as u32);
    let nthreads = n_threads();

    let mut f = Fused::build_direct(vi, &g);
    let n_pad = f.cp.len();
    // SAFETY: AtomicU64 は u64 と同一表現 (冒頭の const assert)。scope 終了まで f.cp 本体には
    // 触れず、全アクセスはこのビュー経由の Relaxed load/store。
    let cp_atomic: &[AtomicU64] =
        unsafe { std::slice::from_raw_parts(f.cp.as_mut_ptr().cast::<AtomicU64>(), n_pad) };

    let mut cand_list: Vec<(u32, u32)> =
        seed_frontier_2d(vi).dilate(dx, dy).enumerate().collect();
    let mut changed_lists: Vec<Vec<(u32, u32)>> = vec![Vec::new(); nthreads];

    let shared = Shared {
        cand: &mut cand_list as *mut Vec<(u32, u32)>,
        changed: changed_lists.as_mut_ptr(),
    };

    let barrier = Barrier::new(nthreads);
    let done = AtomicBool::new(false);
    let iters_out = AtomicU32::new(0);
    let converged_out = AtomicBool::new(false);
    let cursor = AtomicUsize::new(0);
    let g_ref = &g;
    let pen_ref: &[u64] = &f.pen;
    let eval_ref: &[bool] = &f.eval_ok;

    let total_updates: u64 = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..nthreads)
            .map(|w| {
                let barrier = &barrier;
                let done = &done;
                let iters_out = &iters_out;
                let converged_out = &converged_out;
                let cursor = &cursor;
                scope.spawn(move || -> u64 {
                    #[allow(clippy::redundant_locals)]
                    let shared = shared;
                    let mut my_updates: u64 = 0;
                    let mut iter_count: u32 = 0;
                    loop {
                        // ── compute (並列・in-place 非同期書き込み) ──
                        let cand = unsafe { &*shared.cand };
                        let n = cand.len();
                        // SAFETY: ワーカー w は changed[w] だけを触る（他スレッドと排他）。
                        let my_changed = unsafe { &mut *shared.changed.add(w) };
                        my_changed.clear();

                        const BLOCK: usize = 16;
                        loop {
                            let s = cursor.fetch_add(BLOCK, Ordering::Relaxed);
                            if s >= n {
                                break;
                            }
                            let e = (s + BLOCK).min(n);
                            for &(ixu, iyu) in &cand[s..e] {
                                let (ix, iy) = (ixu as i32, iyu as i32);
                                let pad_col = g_ref.pad_col(ix, iy);
                                let mut cell_changed = false;
                                for it in 0..nt {
                                    let pad_idx = (pad_col + it as i64) as usize;
                                    if !eval_ref[pad_idx] {
                                        continue;
                                    }
                                    // 自セルは単一書き手なので最新値。before = cp − pen で復元。
                                    let cp_self = cp_atomic[pad_idx].load(Ordering::Relaxed);
                                    let pen_self = pen_ref[pad_idx];
                                    let before = if cp_self == UNREACHED {
                                        MAX_COST
                                    } else {
                                        cp_self.wrapping_sub(pen_self)
                                    };
                                    let mut min_cost = MAX_COST;
                                    for per_theta in g_ref.precomp.iter() {
                                        let c = action_cost_fused(
                                            cp_atomic,
                                            &per_theta[it as usize],
                                            pad_col,
                                        );
                                        if c < min_cost {
                                            min_cost = c;
                                        }
                                    }
                                    if min_cost < before {
                                        // claim したブロック内のセル = 単一書き手。
                                        cp_atomic[pad_idx].store(
                                            min_cost.wrapping_add(pen_self),
                                            Ordering::Relaxed,
                                        );
                                        my_updates += 1;
                                        cell_changed = true;
                                    }
                                }
                                if cell_changed {
                                    my_changed.push((ixu, iyu));
                                }
                            }
                        }

                        barrier.wait(); // B1: 全 cp/changed 書き込みが可視。

                        // ── リーダー直列: changed → 次フロンティア再構築 / 終了判定 ──
                        if w == 0 {
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
                            if any && iter_count < max_iter {
                                let mut next: Vec<(u32, u32)> =
                                    nf.dilate(dx, dy).enumerate().collect();
                                // 対称 G-S 風: 走査方向をラウンドごとに反転。
                                if iter_count % 2 == 1 {
                                    next.reverse();
                                }
                                // SAFETY: 他ワーカーは B1〜B2 間 cand を読まない。
                                unsafe {
                                    *shared.cand = next;
                                }
                                cursor.store(0, Ordering::Relaxed);
                            } else {
                                iters_out.store(iter_count, Ordering::Relaxed);
                                converged_out.store(!any, Ordering::Relaxed);
                                done.store(true, Ordering::Relaxed);
                            }
                        }

                        barrier.wait(); // B2: リーダーの cand 差し替え / done が可視。
                        if done.load(Ordering::Relaxed) {
                            break;
                        }
                    }
                    my_updates
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).sum()
    });

    let iters = iters_out.load(Ordering::Relaxed);
    let converged = converged_out.load(Ordering::Relaxed);

    // 方策は cp から最終 argmin、書き戻しは行バンド並列。
    let opt = final_policy_fused(&g, &f, nthreads, converged);
    write_back_fused(vi, &g, &f, &opt);

    (iters, total_updates, converged)
}

#[cfg(test)]
mod tests {
    use super::frontier2d_fused_solve;
    use crate::solvers::test_support::{assert_parity, parity_standard_maps};

    #[test]
    fn parity_standard_maps_frontier2d_fused() {
        parity_standard_maps(|vi| frontier2d_fused_solve(vi, 2000));
    }

    /// 複数行バンドにまたがる候補で cross-thread 非同期パス + cp 復元を刺激する。
    #[test]
    fn parity_larger_empty_frontier2d_fused() {
        assert_parity(32, 24, vec![0i8; 32 * 24], |vi| frontier2d_fused_solve(vi, 2000));
    }
}
