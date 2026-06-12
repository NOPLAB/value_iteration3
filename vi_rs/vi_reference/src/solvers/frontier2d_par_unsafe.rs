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

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::Barrier;

use crate::params::MAX_COST;
use crate::value_iterator::ValueIterator;

use super::frontier2d_pad::{action_cost_pad, Padded};
use super::frontier2d_par::{final_policy, n_threads};
use super::{seed_frontier_2d, Bitboard2D};

// AtomicU64 は u64 と同一のメモリ表現を持つ (std ドキュメント保証) — `Vec<[u64; 2]>` を
// `&[[AtomicU64; 2]]` として再解釈する前提をコンパイル時に固定する。
const _: () = assert!(
    std::mem::size_of::<[AtomicU64; 2]>() == std::mem::size_of::<[u64; 2]>()
        && std::mem::align_of::<[AtomicU64; 2]>() == std::mem::align_of::<[u64; 2]>()
);

/// スレッド間で共有する生ポインタ束。永続ワーカーが Copy で持つ。
///
/// - `cand`: 今ラウンドの候補セルリスト。リーダーが B1〜B2 間で差し替え、ワーカーは
///   compute 相でのみ読む — バリアが happens-before を与えるのでデータレースではない。
/// - `changed`: 長さ `nthreads` の配列の先頭。compute 相ではワーカー `w` が `changed[w]` のみ
///   に書き（排他）、B1 後のリーダー直列相でのみ全要素を読む。
///
/// (`hot` はここに含めない — `[[AtomicU64; 2]]` の共有スライスとして普通に渡る。)
#[derive(Clone, Copy)]
struct Shared {
    cand: *mut Vec<(u32, u32)>,
    changed: *mut Vec<(u32, u32)>,
}
// SAFETY: 上記のとおり全アクセスはバリアで相分離され、「単一書き手 + バリア後読み」の規律を守る。
unsafe impl Send for Shared {}
unsafe impl Sync for Shared {}

/// セット済み `ValueIterator` を非同期 (Gauss-Seidel) unsafe 並列 frontier2d で解く。
/// `(iters, updates, converged)`。到達可能セルの収束値・方策は安全版と bit-exact。
pub fn frontier2d_par_unsafe_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let mut m = Padded::build(vi);
    let (nx, ny, nt) = (m.nx, m.ny, m.nt);
    let (dx, dy) = (m.mx as u32, m.my as u32);
    let nthreads = n_threads();

    // hot を Padded から取り出し、atomic ビューで共有する（&m の不変借用と両立させるため分離）。
    // SAFETY: [AtomicU64; 2] は [u64; 2] とサイズ/アライン一致 (冒頭の const assert)。
    // scope 終了まで hot 本体には触れず、全アクセスはこのビュー経由の Relaxed load/store。
    let mut hot: Vec<[u64; 2]> = std::mem::take(&mut m.hot);
    let n_pad = hot.len();
    let hot_atomic: &[[AtomicU64; 2]] =
        unsafe { std::slice::from_raw_parts(hot.as_mut_ptr().cast::<[AtomicU64; 2]>(), n_pad) };

    // 初期候補 = dilate(seed)。以降はリーダーが changed から再構築する。
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
    // work-stealing カーソル: 候補リストを BLOCK 件単位で fetch_add により動的分配する。
    // 静的チャンクだと障害物近傍 (action_cost_pad が即 return) ばかりのチャンクが早く終わり
    // 負荷不均衡になる。各ブロックの claim は一意なので「セルの書き手は 1 スレッド」は保たれる。
    let cursor = AtomicUsize::new(0);
    let m_ref = &m;

    // 全ワーカーの累積更新数を合算して updates とする（per-(cell,theta) の減少回数）。
    let total_updates: u64 = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..nthreads)
            .map(|w| {
                let barrier = &barrier;
                let done = &done;
                let iters_out = &iters_out;
                let converged_out = &converged_out;
                let cursor = &cursor;
                scope.spawn(move || -> u64 {
                    // `Shared` 全体を再束縛してクロージャに「構造体まるごと」をキャプチャさせる
                    // (Rust 2021 のフィールド分割キャプチャだと生ポインタ単体が捕まり Send にならない)。
                    // clippy::redundant_locals は分割キャプチャ回避という意味を解さないので抑制する。
                    #[allow(clippy::redundant_locals)]
                    let shared = shared;
                    let mut my_updates: u64 = 0;
                    let mut iter_count: u32 = 0;
                    loop {
                        // ── compute (並列・in-place 非同期書き込み) ──
                        // SAFETY (cand): リーダーの差し替えは B1〜B2 間のみ、ここは B2 後の
                        // compute 相 — バリアの happens-before で可視・無競合。
                        let cand = unsafe { &*shared.cand };
                        let n = cand.len();
                        // SAFETY: ワーカー w は changed[w] だけを触る（他スレッドと排他）。
                        let my_changed = unsafe { &mut *shared.changed.add(w) };
                        my_changed.clear();

                        // work stealing: BLOCK 件の連続ブロックを fetch_add で claim する
                        // (連続なので行方向の cache 局所性は静的チャンクと同等)。
                        const BLOCK: usize = 16;
                        loop {
                            let s = cursor.fetch_add(BLOCK, Ordering::Relaxed);
                            if s >= n {
                                break;
                            }
                            let e = (s + BLOCK).min(n);
                            for &(ixu, iyu) in &cand[s..e] {
                                let (ix, iy) = (ixu as i32, iyu as i32);
                                let pad_col = m_ref.pad_col(ix, iy);
                                let mut cell_changed = false;
                                for it in 0..nt {
                                    let pad_idx = (pad_col + it as i64) as usize;
                                    if !m_ref.free[pad_idx] || m_ref.finals[pad_idx] {
                                        continue;
                                    }
                                    // 自セルは単一書き手なので before は最新値 (Relaxed で十分)。
                                    let before = hot_atomic[pad_idx][0].load(Ordering::Relaxed);
                                    let mut min_cost = MAX_COST;
                                    for per_theta in m_ref.precomp.iter() {
                                        let c = action_cost_pad(
                                            hot_atomic,
                                            &m_ref.free,
                                            &per_theta[it as usize],
                                            pad_col,
                                        );
                                        if c < min_cost {
                                            min_cost = c;
                                        }
                                    }
                                    if min_cost < before {
                                        // claim したブロック内のセル = 単一書き手。
                                        hot_atomic[pad_idx][0].store(min_cost, Ordering::Relaxed);
                                        my_updates += 1;
                                        cell_changed = true;
                                    }
                                }
                                if cell_changed {
                                    my_changed.push((ixu, iyu));
                                }
                            }
                        }

                        barrier.wait(); // B1: 全 hot/changed 書き込みが可視。

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
                                // 対称 Gauss-Seidel 風: 走査方向をラウンドごとに反転すると、
                                // 行順走査と逆向きの値伝播も同一ラウンド内で連鎖する。
                                if iter_count % 2 == 1 {
                                    next.reverse();
                                }
                                // SAFETY: 他ワーカーは B1〜B2 間 cand を読まない。
                                unsafe {
                                    *shared.cand = next;
                                }
                                // 次ラウンドの work-stealing カーソルを巻き戻す (B2 で可視化)。
                                cursor.store(0, Ordering::Relaxed);
                                // done は false のまま（次ラウンドへ）。
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

    // hot を Padded へ戻し、収束値から optimal_action を確定して書き戻す。
    m.hot = hot;
    let opt = final_policy(&m, nthreads);
    m.write_back(vi, Some(&opt));

    let iters = iters_out.load(Ordering::Relaxed);
    let converged = converged_out.load(Ordering::Relaxed);
    (iters, total_updates, converged)
}

#[cfg(test)]
mod tests {
    use super::frontier2d_par_unsafe_solve;
    use crate::solvers::test_support::{assert_parity, parity_standard_maps};

    #[test]
    fn parity_standard_maps_frontier2d_par_unsafe() {
        parity_standard_maps(|vi| frontier2d_par_unsafe_solve(vi, 2000));
    }

    /// より大きい空マップ: 複数行バンドにまたがる候補で cross-thread 非同期パスを刺激する。
    /// 非同期更新でも一意固定点へ収束するので reference と bit-exact のはず。
    #[test]
    fn parity_larger_empty_frontier2d_par_unsafe() {
        assert_parity(32, 24, vec![0i8; 32 * 24], |vi| {
            frontier2d_par_unsafe_solve(vi, 2000)
        });
    }

    /// 安全 Jacobi 版 (`frontier2d_par`) との wall-clock 比較（手動計測用、CI 非実行）。
    /// `VI_THREADS` でスレッド数を掃引可能。release 推奨:
    /// `cargo test -p vi_reference --release bench_unsafe_vs_par -- --ignored --nocapture`
    #[test]
    #[ignore = "wall-clock benchmark; run manually in release"]
    fn bench_unsafe_vs_par() {
        use crate::solvers::frontier2d_par::frontier2d_par_solve;
        use crate::solvers::test_support::make_vi;
        use std::time::Instant;

        let (w, h) = (400, 400);
        let occ = vec![0i8; (w * h) as usize];

        let mut a = make_vi(w, h, occ.clone());
        let t = Instant::now();
        let (pi, pu, pc) = frontier2d_par_solve(&mut a, 100_000);
        let par_ms = t.elapsed().as_secs_f64() * 1e3;

        let mut b = make_vi(w, h, occ);
        let t = Instant::now();
        let (ui, uu, uc) = frontier2d_par_unsafe_solve(&mut b, 100_000);
        let uns_ms = t.elapsed().as_secs_f64() * 1e3;

        // 到達可能セルの収束値が一致することも併せて確認。
        let mut mism = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < crate::solvers::REACH_THRESH
                && a.states[i].total_cost != b.states[i].total_cost
            {
                mism += 1;
            }
        }

        let threads = std::env::var("VI_THREADS").unwrap_or_else(|_| "auto".into());
        println!("\n=== {w}x{h} empty, threads={threads} ===");
        println!("  par   (safe Jacobi): iters={pi:6} updates={pu:10} {par_ms:8.1} ms conv={pc}");
        println!("  unsafe (async G-S) : iters={ui:6} updates={uu:10} {uns_ms:8.1} ms conv={uc}");
        println!("  speedup = {:.2}x   value-mismatch(reachable) = {mism}", par_ms / uns_ms);
        assert_eq!(mism, 0, "収束値は安全版と一致するはず");
    }
}
