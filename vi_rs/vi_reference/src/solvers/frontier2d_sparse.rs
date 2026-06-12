//! B4: `frontier2d_fused` (非同期 G-S + penalty 融合) に **θマスク疎評価** を重ねた版。
//!
//! 計測 (house): fused までは候補セルの **全 60 θ** を再評価し、評価 52.8M 回 vs 実更新 16.9M 回
//! (比 3.1) — 約 2/3 の Bellman 評価 (6 アクション×バケット読み) が無駄だった。
//!
//! 疎評価: (c,θ) の Q の入力は (c+Δxy, θ′)、ただし円環距離 circ(θ′−θ) ≤ mt
//! (`displacement` の第 3 成分; 回転アクションは dix=diy=0 なので自セルも窓に含まれる)。
//! → 前ラウンドに変化した (n,θn) があるとき再評価が必要なのは
//!    `θ ∈ rot_dilate(θn, mt)` のみ。セルごとの **変化 θ マスク (u64, nt≤64)** を
//! パディング付き 2D 配列に保持し、候補セルは compute 時に窓 (±mx,±my) のマスクを
//! OR で gather → 1 回 rot_dilate → 立っている θ だけ評価する。
//! マスク配列は ~1MB 級で L2 常駐のため gather は per-bucket ロードよりずっと安い。
//!
//! マスク配列の整合はリーダー直列相で管理 (前ラウンド分をゼロ化 → 今ラウンド分を store、
//! いずれも O(変化セル数))。書き手はセル単位 claim により常に 1 スレッドなので store で足りる。
//!
//! **bit-exact 性**: 評価集合は真の依存集合の保守的上位集合 (2D 窓 × θ 円環膨張)。
//! 入力が変化した (c,θ) は必ず次ラウンドの候補に入るので、「1 ラウンド丸ごと無変化」での
//! 終了時には全 (cell,θ) が現在値と Bellman 整合 = 一意固定点 (`frontier2d_par` らと同値)。
//! 初回ラウンドのみマスク未育成のため全 θ を評価する (上位集合なので無害)。

use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::Barrier;
use std::time::{Duration, Instant};

use crate::params::{MAX_COST, PROB_BASE};
use crate::value_iterator::ValueIterator;

use super::frontier2d_fused::{
    action_cost_fused, final_policy_fused, write_back_fused, Fused, Geom, UNREACHED,
};
use super::frontier2d_par::n_threads;
use super::{displacement, seed_frontier_2d, Bitboard2D};

/// θ マスクを円環 (周期 `nt`) で ±k 膨張する。
#[inline]
fn rot_dilate(m: u64, k: i32, nt: i32) -> u64 {
    let full: u64 = if nt >= 64 { u64::MAX } else { (1u64 << nt) - 1 };
    let nt = nt as u32;
    let mut acc = m;
    let mut l = m;
    let mut r = m;
    for _ in 0..k {
        l = ((l << 1) | (l >> (nt - 1))) & full;
        r = ((r >> 1) | (r << (nt - 1))) & full;
        acc |= l | r;
    }
    acc
}

/// 可視化用スナップショット (`VI_SNAP_DIR` 設定時のみ)。リーダー直列相で
/// min-θ 値場 (f32 秒, 未確定=+inf) を `snap_NNNN.bin` にダンプし、`times.csv` に
/// `idx,t_sec,round` を追記する。タイムスタンプはダンプ自身の所要時間を除いた純ソルバ時間。
struct Snapshotter {
    dir: String,
    every: u32,
    idx: u32,
    t0: Instant,
    overhead: Duration,
    csv: std::fs::File,
}

impl Snapshotter {
    fn from_env(t0: Instant) -> Option<Self> {
        let dir = std::env::var("VI_SNAP_DIR").ok()?;
        let every: u32 = std::env::var("VI_SNAP_EVERY")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(1);
        std::fs::create_dir_all(&dir).ok()?;
        let mut csv = std::fs::File::create(format!("{dir}/times.csv")).ok()?;
        writeln!(csv, "idx,t_sec,round").ok()?;
        Some(Self { dir, every, idx: 0, t0, overhead: Duration::ZERO, csv })
    }

    fn dump(&mut self, g: &Geom, cp: &[AtomicU64], pen: &[u64], round: u32) {
        let t_sec = (self.t0.elapsed() - self.overhead).as_secs_f64();
        let started = Instant::now();
        let (nx, ny, nt) = (g.nx, g.ny, g.nt);
        let mut buf = Vec::<u8>::with_capacity((nx * ny) as usize * 4);
        for iy in 0..ny {
            for ix in 0..nx {
                let col = g.pad_col(ix, iy);
                let mut best = UNREACHED;
                for it in 0..nt {
                    let v = cp[(col + it as i64) as usize].load(Ordering::Relaxed);
                    if v != UNREACHED {
                        let val = v.wrapping_sub(pen[(col + it as i64) as usize]);
                        if val < best {
                            best = val;
                        }
                    }
                }
                let f = if best == UNREACHED {
                    f32::INFINITY
                } else {
                    (best as f64 / PROB_BASE as f64) as f32
                };
                buf.extend_from_slice(&f.to_le_bytes());
            }
        }
        let path = format!("{}/snap_{:05}.bin", self.dir, self.idx);
        if std::fs::write(&path, &buf).is_ok() {
            let _ = writeln!(self.csv, "{},{:.6},{}", self.idx, t_sec, round);
            self.idx += 1;
        }
        self.overhead += started.elapsed();
    }
}

/// 共有ポインタ束 (`frontier2d_par_unsafe` と同じ規律: バリアで相分離)。
#[derive(Clone, Copy)]
struct Shared {
    cand: *mut Vec<(u32, u32)>,
    changed: *mut Vec<(u32, u32, u64)>,
}
// SAFETY: 全アクセスはバリアで相分離され、「単一書き手 + バリア後読み」の規律を守る。
unsafe impl Send for Shared {}
unsafe impl Sync for Shared {}

/// セット済み `ValueIterator` を θ疎評価 + penalty 融合 + 非同期 G-S 並列で解く。
/// `(iters, updates, converged)`。到達可能セルの収束値・方策は本家と bit-exact。
pub fn frontier2d_sparse_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let t_solve0 = Instant::now();
    let g = Geom::build(vi);
    let (nx, ny, nt) = (g.nx, g.ny, g.nt);
    assert!(nt <= 64, "θマスクは u64 前提 (nt={nt})");
    let (dx, dy) = (g.mx as u32, g.my as u32);
    let (mx, my, mt) = displacement(vi);
    let full_mask: u64 = if nt >= 64 { u64::MAX } else { (1u64 << nt) - 1 };
    let nthreads = n_threads();

    let mut f = Fused::build_direct(vi, &g);
    let n_pad = f.cp.len();
    // SAFETY: AtomicU64 は u64 と同一表現。scope 終了まで f.cp 本体には触れない。
    let cp_atomic: &[AtomicU64] =
        unsafe { std::slice::from_raw_parts(f.cp.as_mut_ptr().cast::<AtomicU64>(), n_pad) };
    let pen_ref: &[u64] = &f.pen;
    let eval_ref: &[bool] = &f.eval_ok;

    // 変化 θ マスクの 2D 配列 (gather 窓がはみ出さないよう (mx,my) パディング、ゼロ=変化なし)。
    let mw = (nx + 2 * mx) as usize;
    let mask_arr: Vec<AtomicU64> = (0..mw * (ny + 2 * my) as usize)
        .map(|_| AtomicU64::new(0))
        .collect();
    let midx = |ix: i32, iy: i32| -> usize { (ix + mx) as usize + (iy + my) as usize * mw };

    let mut cand_list: Vec<(u32, u32)> =
        seed_frontier_2d(vi).dilate(dx, dy).enumerate().collect();
    let mut changed_lists: Vec<Vec<(u32, u32, u64)>> = vec![Vec::new(); nthreads];

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
    let mask_ref: &[AtomicU64] = &mask_arr;

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
                    // リーダー専用: 前ラウンドに mask_arr へ書いたセル (ゼロ化用)。
                    let mut prev_cells: Vec<(u32, u32)> = Vec::new();
                    // リーダー専用: 可視化スナップショット (env 未設定なら None)。
                    let mut snap = if w == 0 { Snapshotter::from_env(t_solve0) } else { None };
                    loop {
                        // ── compute (並列・in-place 非同期書き込み) ──
                        let cand = unsafe { &*shared.cand };
                        let n = cand.len();
                        // SAFETY: ワーカー w は changed[w] だけを触る（他スレッドと排他）。
                        let my_changed = unsafe { &mut *shared.changed.add(w) };
                        my_changed.clear();
                        let first = iter_count == 0;

                        const BLOCK: usize = 16;
                        loop {
                            let s = cursor.fetch_add(BLOCK, Ordering::Relaxed);
                            if s >= n {
                                break;
                            }
                            let e = (s + BLOCK).min(n);
                            for j in s..e {
                                let (ixu, iyu) = cand[j];
                                let (ix, iy) = (ixu as i32, iyu as i32);

                                // 次セルのアクション先カラムを先読み (ヒントのみ、意味論不変)。
                                #[cfg(target_arch = "x86_64")]
                                if j + 1 < e {
                                    let (nix, niy) = cand[j + 1];
                                    let ncol = g_ref.pad_col(nix as i32, niy as i32);
                                    let base = cp_atomic.as_ptr();
                                    for per_theta in g_ref.precomp.iter() {
                                        for itq in [0usize, 16, 32, 48] {
                                            if let Some(&(off, _)) =
                                                per_theta[itq.min(nt as usize - 1)].first()
                                            {
                                                // SAFETY: prefetch はメモリを読まないヒント命令。
                                                unsafe {
                                                    std::arch::x86_64::_mm_prefetch::<
                                                        { std::arch::x86_64::_MM_HINT_T1 },
                                                    >(
                                                        base.add((ncol + off) as usize)
                                                            as *const i8,
                                                    );
                                                }
                                            }
                                        }
                                    }
                                }

                                // ── θ マスク gather: 窓内の前ラウンド変化マスクを OR ──
                                let eval_mask = if first {
                                    full_mask // 初回はマスク未育成 → 全 θ (保守的)
                                } else {
                                    let mut acc = 0u64;
                                    for dy2 in -my..=my {
                                        let row = midx(ix - mx, iy + dy2);
                                        for k in 0..(2 * mx + 1) as usize {
                                            acc |= mask_ref[row + k].load(Ordering::Relaxed);
                                        }
                                    }
                                    if acc == 0 {
                                        continue; // 依存入力に変化なし
                                    }
                                    rot_dilate(acc, mt, nt)
                                };

                                let pad_col = g_ref.pad_col(ix, iy);
                                let mut cmask: u64 = 0;
                                let mut bits = eval_mask;
                                while bits != 0 {
                                    let it = bits.trailing_zeros() as i32;
                                    bits &= bits - 1;
                                    let pad_idx = (pad_col + it as i64) as usize;
                                    if !eval_ref[pad_idx] {
                                        continue;
                                    }
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
                                        cmask |= 1u64 << it;
                                    }
                                }
                                if cmask != 0 {
                                    my_changed.push((ixu, iyu, cmask));
                                }
                            }
                        }

                        barrier.wait(); // B1: 全 cp/changed 書き込みが可視。

                        // ── リーダー直列: マスク配列更新 + 次フロンティア / 終了判定 ──
                        if w == 0 {
                            iter_count += 1;
                            // 前ラウンド分のマスクをゼロ化 (今ラウンドの compute はもう読み終えた)。
                            for &(x, y) in &prev_cells {
                                mask_ref[midx(x as i32, y as i32)].store(0, Ordering::Relaxed);
                            }
                            prev_cells.clear();

                            let mut any = false;
                            let mut nf = Bitboard2D::new(nx as u32, ny as u32);
                            for i in 0..nthreads {
                                // SAFETY: B1 後、各 changed[i] への書きは完了し可視。
                                let cl = unsafe { &*shared.changed.add(i) };
                                if !cl.is_empty() {
                                    any = true;
                                }
                                for &(x, y, cm) in cl {
                                    // セルの書き手は 1 スレッド = リストにも一意に現れる → store で足りる。
                                    mask_ref[midx(x as i32, y as i32)].store(cm, Ordering::Relaxed);
                                    nf.set(x, y);
                                    prev_cells.push((x, y));
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
                            if let Some(s) = snap.as_mut() {
                                let now_done = done.load(Ordering::Relaxed);
                                if now_done || iter_count % s.every == 0 {
                                    s.dump(g_ref, cp_atomic, pen_ref, iter_count);
                                }
                            }
                        } else {
                            iter_count += 1;
                        }

                        barrier.wait(); // B2: リーダーの cand/mask 差し替え / done が可視。
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

    // 方策は cp から最終 argmin (fused と共有)、書き戻しは行バンド並列。
    let opt = final_policy_fused(&g, &f, nthreads, converged);
    write_back_fused(vi, &g, &f, &opt);

    (iters, total_updates, converged)
}

#[cfg(test)]
mod tests {
    use super::frontier2d_sparse_solve;
    use crate::solvers::test_support::{assert_parity, parity_standard_maps};

    #[test]
    fn parity_standard_maps_frontier2d_sparse() {
        parity_standard_maps(|vi| frontier2d_sparse_solve(vi, 2000));
    }

    #[test]
    fn parity_larger_empty_frontier2d_sparse() {
        assert_parity(32, 24, vec![0i8; 32 * 24], |vi| frontier2d_sparse_solve(vi, 2000));
    }
}
