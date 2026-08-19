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
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::Barrier;
use std::time::{Duration, Instant};

use crate::params::{MAX_COST, PROB_BASE};
use crate::value_iterator::ValueIterator;

use super::frontier2d_fused::{
    action_cost_fused, final_policy_fused, final_policy_fused_on, write_back_fused,
    write_back_fused_on, Fused, Geom, UNREACHED,
};
use crate::solvers::observe::{
    MaterializeProbe, SolveFlow, SolveObserver, SolveOutcome,
};
use super::frontier2d_par::n_threads;
use super::{displacement, seed_frontier_2d, Bitboard2D};

/// θ マスクを円環 (周期 `nt`) で ±k 膨張する。回転アクションは dix=diy=0 で自セルも
/// 窓に含むため、変化 θ から `circ(θ′−θ) ≤ mt` の θ を再評価対象に広げる
/// (`frontier2d_sparse_compact` と共有)。
#[inline]
pub(crate) fn rot_dilate(m: u64, k: i32, nt: i32) -> u64 {
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
        let nxu = nx as usize;
        // min-θ 値場 (f32 秒) を行バンド並列で構築。dump はリーダー直列相で呼ばれ他ワーカーは
        // バリア待機中なので、ここで一時スレッドを起こしアイドルコアを使う (627M セル走査が
        // 単一コアだと scale3 で 1 回 ~4 s かかり wall-clock を支配するため)。
        let mut field = vec![0f32; nxu * ny as usize];
        let nthr = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1)
            .clamp(1, ny.max(1) as usize);
        let rows_per = (ny as usize).div_ceil(nthr);
        std::thread::scope(|s| {
            for (band, chunk) in field.chunks_mut(rows_per * nxu).enumerate() {
                let iy0 = (band * rows_per) as i32;
                s.spawn(move || {
                    let rows = (chunk.len() / nxu) as i32;
                    for r in 0..rows {
                        let iy = iy0 + r;
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
                            chunk[r as usize * nxu + ix as usize] = if best == UNREACHED {
                                f32::INFINITY
                            } else {
                                (best as f64 / PROB_BASE as f64) as f32
                            };
                        }
                    }
                });
            }
        });
        let mut buf = Vec::<u8>::with_capacity(nxu * ny as usize * 4);
        for f in &field {
            buf.extend_from_slice(&f.to_le_bytes());
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
/// 到達可能セルの収束値・方策は本家と bit-exact。
///
/// リーダー (w=0) は**呼び出しスレッドでインライン実行**し、ラウンド境界ごとに
/// `obs.boundary` を呼ぶ (Snapshotter がやっていたリーダー相での介入の一般化)。
/// 境界の具現化は全ワーカーがバリアで静止した相なので cp の Relaxed 読みで一貫する。
pub fn frontier2d_sparse_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    let g = Geom::build(vi);
    let (nx, ny, nt) = (g.nx, g.ny, g.nt);
    assert!(nt <= 64, "θマスクは u64 前提 (nt={nt})");
    let (dx, dy) = (g.mx as u32, g.my as u32);
    let (mx, my, mt) = displacement(vi);
    let full_mask: u64 = if nt >= 64 { u64::MAX } else { (1u64 << nt) - 1 };
    let nthreads = n_threads();

    let mut f = Fused::build_direct(vi, &g);

    // 低メモリ動画モード (`VI_SNAP_DROP_STATES=1`)。`Fused` は cp/pen/eval_ok を独立所有し、
    // 以降スイープ本体も Snapshotter も `vi.states` を一切参照しない (snapshot は cp/pen から
    // 直接ダンプ)。巨大マップ (例 tsukuba 0.15m = 4417×2367×60 ≈ 627M states ≈ 35 GB) では
    // states を抱えたままだと cp/pen と合わせ RAM を超えるため、初期フロンティアの
    // 種付け (seed_frontier_2d) 直後に解放する (フラグだけここで読む)。
    // 解放すると末尾の write_back / policy も境界プローブの具現化も不能になるので、
    // どちらもスキップする (動画は snapshot 出力で完結し、収束値は cp/pen 側に保持される)。
    let drop_states = std::env::var("VI_SNAP_DROP_STATES")
        .map(|v| v == "1")
        .unwrap_or(false);

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

    // 低メモリ動画モードの states 解放はここ (種付け後)。seed_frontier_2d は vi.states を
    // 走査して初期フロンティア (goal セル) を作るので、その前に解放すると種が空になり
    // updates=0 の縮退収束になる。種は cand_list に確保済みで、以降スイープ本体も
    // Snapshotter も states を参照しない (snapshot は cp/pen から直接ダンプ)。
    if drop_states {
        vi.states = Vec::new();
        vi.states.shrink_to_fit();
    }

    let mut changed_lists: Vec<Vec<(u32, u32, u64)>> = vec![Vec::new(); nthreads];

    let shared = Shared {
        cand: &mut cand_list as *mut Vec<(u32, u32)>,
        changed: changed_lists.as_mut_ptr(),
    };

    let barrier = Barrier::new(nthreads);
    let done = AtomicBool::new(false);
    let cursor = AtomicUsize::new(0);
    // 境界プローブが読む「これまでの更新総数」。各スレッドが自ラウンド分を B1 前に足す。
    let updates_total = AtomicU64::new(0);
    let g_ref = &g;
    let mask_ref: &[AtomicU64] = &mask_arr;
    let interval = obs.interval().max(1) as u64;

    // compute 相 1 ラウンド分 (θマスク gather + 疎評価)。changed[w] に (x, y, θマスク) を積み、
    // 自ラウンド更新数を返す。全キャプチャは共有参照なので全スレッドで共有できる。
    // SAFETY: バリアで相分離された compute 相からのみ呼ぶこと。
    let compute_phase = |w: usize, first: bool| -> u64 {
        // `Shared` 全体を再束縛してクロージャに「構造体まるごと」をキャプチャさせる
        // (Rust 2021 のフィールド分割キャプチャだと生ポインタ単体が捕まり Sync にならない)。
        #[allow(clippy::redundant_locals)]
        let shared = shared;
        // SAFETY (cand): リーダーの差し替えは B1〜B2 間のみ、ここは B2 後の compute 相。
        let cand = unsafe { &*shared.cand };
        let n = cand.len();
        // SAFETY: ワーカー w は changed[w] だけを触る（他スレッドと排他）。
        let my_changed = unsafe { &mut *shared.changed.add(w) };
        my_changed.clear();
        let mut my_updates = 0u64;

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
                                        base.add((ncol + off) as usize) as *const i8,
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
                    // 値が「動いた」ら伝播する。白紙からの solve では値は単調に
                    // 下がるので `< before` と同じ挙動 (Reference と bit-exact) だが、
                    // 収束後の掃き直しでは local_penalty が値を**上げる**ので、
                    // 下降しか追わないと伝播が黙って止まる (`SolverCaps::resweep`)。
                    if min_cost != before {
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
        my_updates
    };
    let compute_phase = &compute_phase;

    // スナップショット計時はここから (Geom/Fused::build_direct・seed_frontier_2d を除外した
    // 純スイープ時間)。巨大マップでは build_direct が支配的になり得るが、本家 ROS1 の
    // snapshotWorker もセットアップを除外して計測する (README「vi_rs と同条件」) ので合わせる。
    let t_solve0 = Instant::now();
    let out: SolveOutcome = std::thread::scope(|scope| {
        // ワーカー (w=1..nthreads)。compute とバリアだけを回す。
        for w in 1..nthreads {
            let barrier = &barrier;
            let done = &done;
            let updates_total = &updates_total;
            scope.spawn(move || {
                let mut round = 0u32;
                loop {
                    let ups = compute_phase(w, round == 0);
                    if ups > 0 {
                        updates_total.fetch_add(ups, Ordering::Relaxed);
                    }
                    round += 1;
                    barrier.wait(); // B1: 全 cp/changed 書き込みが可視。
                    barrier.wait(); // B2: リーダーの cand/mask 差し替え / done が可視。
                    if done.load(Ordering::Relaxed) {
                        break;
                    }
                }
            });
        }

        // リーダー (w=0)。呼び出しスレッドでインライン実行。
        let mut iter_count: u32 = 0;
        let mut next_boundary: u64 = interval;
        // 前ラウンドに mask_arr へ書いたセル (ゼロ化用)。
        let mut prev_cells: Vec<(u32, u32)> = Vec::new();
        // 可視化スナップショット (env 未設定なら None)。
        let mut snap = Snapshotter::from_env(t_solve0);
        let mut outcome: Option<SolveOutcome> = None;
        loop {
            let ups = compute_phase(0, iter_count == 0);
            if ups > 0 {
                updates_total.fetch_add(ups, Ordering::Relaxed);
            }
            barrier.wait(); // B1: 全 cp/changed 書き込みが可視。

            // ── リーダー直列: マスク配列更新 + 次フロンティア / 境界 / 終了判定 ──
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
            let updates_now = updates_total.load(Ordering::Relaxed);
            let mut flow = SolveFlow::Continue;
            // 全ワーカーは B2 で待機中 (cp への書き込みは静止) — 具現化はここで安全。
            // drop_states 時は書き戻す先が無いので境界は呼ばない。
            if any
                && iter_count < max_iter
                && !drop_states
                && iter_count as u64 >= next_boundary
            {
                next_boundary = (iter_count as u64).saturating_add(interval);
                let mut mat = |vi: &mut ValueIterator| {
                    let opt =
                        final_policy_fused_on(g_ref, cp_atomic, eval_ref, nthreads, false);
                    write_back_fused_on(vi, g_ref, cp_atomic, pen_ref, &opt);
                };
                let mut probe =
                    MaterializeProbe::new(vi, &mut mat, iter_count, updates_now);
                flow = obs.boundary(&mut probe);
            }
            if any && iter_count < max_iter && flow == SolveFlow::Continue {
                let mut next: Vec<(u32, u32)> = nf.dilate(dx, dy).enumerate().collect();
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
                outcome = Some(match flow {
                    SolveFlow::Stop => SolveOutcome::stopped(iter_count, updates_now),
                    SolveFlow::Cancel => SolveOutcome::cancelled(iter_count, updates_now),
                    SolveFlow::Continue => {
                        SolveOutcome::running(iter_count, updates_now, !any)
                    }
                });
                done.store(true, Ordering::Relaxed);
            }
            if let Some(s) = snap.as_mut() {
                let now_done = done.load(Ordering::Relaxed);
                if now_done || iter_count % s.every == 0 {
                    s.dump(g_ref, cp_atomic, pen_ref, iter_count);
                }
            }

            barrier.wait(); // B2: リーダーの cand/mask 差し替え / done が可視。
            if done.load(Ordering::Relaxed) {
                break;
            }
        }
        outcome.expect("engine loop sets outcome before exiting")
    });

    if out.cancelled {
        // 中断: 場は保証しない契約なので書き戻しをスキップして即戻る。
        return out;
    }
    // 方策は cp から最終 argmin (fused と共有)、書き戻しは行バンド並列。
    // 低メモリモードでは states を解放済みなので、書き戻し / policy 算出をスキップする
    // (どちらも states 規模の確保を伴うため、メモリ削減の意味も兼ねる)。
    if !drop_states {
        let opt = final_policy_fused(&g, &f, nthreads, out.converged);
        write_back_fused(vi, &g, &f, &opt);
    }

    out
}
