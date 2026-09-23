//! 順序つきアクティブセット掃引 (ordered sweep) — **上界場を掃引順序として使う** exactify。
//!
//! frontier2d 型の掃き直しは 1 ラウンドに遷移 reach ぶんしか補正が進まない
//! (光円錐: tsudanuma s3 で 765 ラウンド、総仕事 7.8 n 更新)。ラウンドあたりの
//! 固定費 (バリア・全域 dilate・列挙) が支配的で、s3 の 16 s は仕事量換算 ~1 s。
//! ここでは状態 (x,y,θ) を**現在値の昇順** (後継 → 先行) に in-place 更新するので
//! 1 パスで経路全体に補正が走り、パス数は光円錐ではなく**閉路の幾何級数**で決まる。
//!
//! # 実測で分かった構造
//!
//! - 順序の種は上界場 (Schur V̂ / 近傍ゴールの場 + 定数 / 前ゴールの場) で十分。
//!   オラクル場で「最適行動の後継に自分より高い値がある状態」は ~1% (サブセル補間の
//!   横漏れ質量 m、平均 7%)。その 1% が閉路を作り、1 パスで誤差が m 倍にしか減らない
//!   ので、bit-exact までのパス数は log(初期誤差)/log(1/m) ≈ 30 (house)。
//! - 全パス方式だと上流全域が毎パス動く (house: 7.3M → 7.2M → 6.8M → … → 0、
//!   計 6.2 n)。仕事量は frontier2d と同じ水準なので、**動いた後継を持つ状態だけ**を
//!   次パスの対象にすれば、仕事 ≈ 6 n のままラウンド数が 765 → 30 になる。
//! - 値バケットつき label-correcting (Dial 型) は閉路の 1 反復ごとに上流を逐次
//!   再伝播して状態あたり 2,000 回以上処理になる (試作して破棄)。バッチ化された
//!   パスが閉路の補正を上流で合流させるのが効いている。
//!
//! 収束値は訪問順序によらない: 作用素は単調で、V ≥ ν (MAX_COST 起点の固定点) から
//! 出発する限り何順でも ν に着地する (`schur.rs` 冒頭の上界性の議論と同じ)。
//! 値は上下どちらにも動かす (上界破れのある場でも非同期 VI として収束する)。
//! 並列は `frontier2d_par_unsafe` と同じ非同期 GS (状態ごと単一書き手、隣接は
//! Relaxed atomic 読み)、停止は「ダーティ集合が空」。
//!
//! 実装: パス k の対象集合を値のバケット (幅 [`BUCKET_SHIFT`]) に counting sort し、
//! バケット昇順に、バケット内は並列 (work-stealing) に処理する。値が動いた状態は
//! 逆遷移で前駆をダーティにし (dedupe フラグ)、それがパス k+1 の対象集合。

use std::sync::atomic::{AtomicU64, AtomicU8, AtomicUsize, Ordering};
use std::sync::{Barrier, Mutex};

use crate::params::MAX_COST;
use crate::value_iterator::ValueIterator;

use super::frontier2d_pad::{action_cost_pad, Padded};
use super::frontier2d_par::{final_policy, n_threads};

/// バケット幅 = 2^BUCKET_SHIFT [値単位]。2^16 = 0.25 s。1 ステップのコスト
/// (free セルで 1.0 s) より狭ければ後継は必ず前のバケットに入る。
const BUCKET_SHIFT: u32 = 16;
/// バケット数の上限 (超える値は最終バケットへ)。2^14 × 0.25 s = 4096 s。
const MAX_BUCKETS: usize = 1 << 14;
/// work-stealing の claim 単位 [状態]。
const BLOCK: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OrderedOutcome {
    pub passes: u32,
    /// 処理した状態数の合計 (frontier2d の updates と同じ「仕事量」の尺度)。
    pub updates: u64,
    pub converged: bool,
}

/// `vi.states` の現在値 (上界場) を出発点に、値昇順のアクティブセット非同期 GS で
/// 固定点まで掃く。収束後は `optimal_action` も確定する。
pub fn ordered_sweep(vi: &mut ValueIterator, max_passes: u32) -> OrderedOutcome {
    let mut m = Padded::build(vi);
    let nt = m.nt as usize;
    let nthreads = n_threads();

    let mut hot: Vec<[u64; 2]> = std::mem::take(&mut m.hot);
    let n_pad = hot.len();
    // SAFETY: [AtomicU64; 2] は [u64; 2] と同サイズ・同アライン (par_unsafe と同じ前提)。
    let hot_atomic: &[[AtomicU64; 2]] =
        unsafe { std::slice::from_raw_parts(hot.as_mut_ptr().cast::<[AtomicU64; 2]>(), n_pad) };
    let m_ref = &m;

    // 逆遷移 (pad オフセット): 着地 θ ごとに、前駆 = p + off。
    let rev = super::priority::build_rev_theta(vi);
    let rev_off: Vec<Vec<i64>> = (0..nt)
        .map(|it| {
            rev[it]
                .iter()
                .map(|&(dix, diy, t_src)| {
                    (t_src as i64 - it as i64) - dix as i64 * nt as i64 - diy as i64 * m_ref.row_stride
                })
                .collect()
        })
        .collect();
    let rev_off = &rev_off;

    // ダーティフラグ (dedupe)。パス 0 の対象 = free 非 final 全部。
    let dirty: Vec<AtomicU8> = (0..n_pad).map(|_| AtomicU8::new(0)).collect();
    let dirty = &dirty;
    let mut items: Vec<u32> = (0..n_pad as u32)
        .filter(|&p| m_ref.free[p as usize] && !m_ref.finals[p as usize])
        .collect();
    for &p in &items {
        dirty[p as usize].store(1, Ordering::Relaxed);
    }

    let mut order: Vec<u32> = Vec::new();
    let mut keys: Vec<u16> = Vec::new();
    let mut counts: Vec<usize> = vec![0; MAX_BUCKETS + 1];
    let mut starts: Vec<usize> = vec![0; MAX_BUCKETS + 2];

    let mut passes = 0u32;
    let mut total_processed = 0u64;
    while passes < max_passes && !items.is_empty() {
        passes += 1;
        let n = items.len();
        // ── バケット化 (状態キー = 現在値)。
        order.resize(n, 0);
        keys.resize(n, 0);
        for (k, &p) in items.iter().enumerate() {
            let v = hot_atomic[p as usize][0].load(Ordering::Relaxed);
            keys[k] = if v >= MAX_COST {
                MAX_BUCKETS as u16
            } else {
                ((v >> BUCKET_SHIFT) as usize).min(MAX_BUCKETS) as u16
            };
        }
        counts.iter_mut().for_each(|c| *c = 0);
        for &b in &keys {
            counts[b as usize] += 1;
        }
        starts[0] = 0;
        for b in 0..=MAX_BUCKETS {
            starts[b + 1] = starts[b] + counts[b];
        }
        {
            let mut fill = starts.clone();
            for (k, &p) in items.iter().enumerate() {
                let b = keys[k] as usize;
                order[fill[b]] = p;
                fill[b] += 1;
            }
        }

        // ── バケット昇順・バケット内並列の非同期 GS。空バケットは全スレッドが
        //    同じ starts を見て同期なしに飛ばす。claim はバケットごとに独立なので
        //    バケット境界のバリアは 1 回で済む。動いた状態の前駆は next へ。
        let nonempty: Vec<usize> = (0..=MAX_BUCKETS).filter(|&b| starts[b + 1] > starts[b]).collect();
        let claims: Vec<AtomicUsize> = nonempty.iter().map(|_| AtomicUsize::new(0)).collect();
        let order_ref = &order;
        let starts_ref = &starts;
        let nonempty_ref = &nonempty;
        let claims_ref = &claims;
        let barrier = Barrier::new(nthreads);
        let processed = AtomicU64::new(0);
        let next_lists: Mutex<Vec<Vec<u32>>> = Mutex::new(Vec::with_capacity(nthreads));
        std::thread::scope(|scope| {
            for _ in 0..nthreads {
                let next_lists = &next_lists;
                let processed = &processed;
                let barrier = &barrier;
                scope.spawn(move || {
                    let mut local_next: Vec<u32> = Vec::new();
                    let mut local_processed = 0u64;
                    for (bi, &b) in nonempty_ref.iter().enumerate() {
                        let (s, e) = (starts_ref[b], starts_ref[b + 1]);
                        loop {
                            let i0 = s + claims_ref[bi].fetch_add(BLOCK, Ordering::Relaxed);
                            if i0 >= e {
                                break;
                            }
                            for &p in &order_ref[i0..(i0 + BLOCK).min(e)] {
                                let p = p as usize;
                                // 自分のフラグを先に下ろす: 処理中に後継が動けば再び立つ。
                                dirty[p].store(0, Ordering::Relaxed);
                                local_processed += 1;
                                if update_state(m_ref, hot_atomic, p) {
                                    let it = p % nt;
                                    for &off in &rev_off[it] {
                                        let q = p as i64 + off;
                                        if q < 0 || q >= n_pad as i64 {
                                            continue;
                                        }
                                        let q = q as usize;
                                        if !m_ref.free[q] || m_ref.finals[q] {
                                            continue;
                                        }
                                        if dirty[q].swap(1, Ordering::Relaxed) == 0 {
                                            local_next.push(q as u32);
                                        }
                                    }
                                }
                            }
                        }
                        barrier.wait();
                    }
                    processed.fetch_add(local_processed, Ordering::Relaxed);
                    next_lists.lock().unwrap().push(local_next);
                });
            }
        });
        total_processed += processed.load(Ordering::Relaxed);
        let lists = next_lists.into_inner().unwrap();
        let n_next: usize = lists.iter().map(|l| l.len()).sum();
        items.clear();
        items.reserve(n_next);
        for l in lists {
            items.extend_from_slice(&l);
        }
        if std::env::var_os("VI_ORDERED_TRACE").is_some() {
            eprintln!("ordered pass {passes}: processed {n}, next {n_next}");
        }
    }
    let converged = items.is_empty();

    m.hot = hot;
    let opt = final_policy(&m, nthreads);
    m.write_back(vi, Some(&opt));
    OrderedOutcome { passes, updates: total_processed, converged }
}

/// 状態 (pad 添字 p) を Bellman 更新する。動いたら true。
#[inline]
fn update_state(m: &Padded, hot: &[[AtomicU64; 2]], p: usize) -> bool {
    let nt = m.nt as usize;
    let it = p % nt;
    let base = (p - it) as i64;
    let before = hot[p][0].load(Ordering::Relaxed);
    let mut min_cost = MAX_COST;
    for per_theta in m.precomp.iter() {
        let c = action_cost_pad(hot, &m.free, &per_theta[it], base);
        if c < min_cost {
            min_cost = c;
        }
    }
    if min_cost != before {
        hot[p][0].store(min_cost, Ordering::Relaxed);
        true
    } else {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solvers::test_support::make_vi;
    use crate::solvers::{solve, U64Solver};

    /// 上界 (オラクル + 一様定数) から出発した ordered sweep がオラクルと bit-exact
    /// (値も方策も) に着地する。
    #[test]
    fn ordered_sweep_from_upper_bound_lands_on_oracle() {
        let (w, h) = (40, 30);
        let mut occ = vec![0i8; (w * h) as usize];
        for y in 5..25 {
            occ[(y * w + 20) as usize] = 100; // 壁 1 本 (上下に隙間)
        }
        let mut vi = make_vi(w, h, occ);
        let st = solve(&mut vi, U64Solver::Frontier2D, 100_000);
        assert!(st.converged);
        let oracle: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
        let opt: Vec<Option<usize>> = vi.states.iter().map(|s| s.optimal_action).collect();
        for s in vi.states.iter_mut() {
            if !s.final_state && s.total_cost < MAX_COST {
                s.total_cost += 12_345_678; // 上界に持ち上げる
            }
        }
        let out = ordered_sweep(&mut vi, 10_000);
        assert!(out.converged, "{out:?}");
        let mm = vi.states.iter().zip(&oracle).filter(|(s, &o)| s.total_cost != o).count();
        assert_eq!(mm, 0, "{out:?}");
        let mp = vi
            .states
            .iter()
            .zip(&opt)
            .filter(|(s, &o)| s.total_cost < MAX_COST && s.optimal_action != o)
            .count();
        assert_eq!(mp, 0);
        eprintln!("{out:?} (reachable {})", oracle.iter().filter(|&&v| v < MAX_COST).count());
    }
}
