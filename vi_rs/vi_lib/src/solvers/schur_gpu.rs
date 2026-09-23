//! [`super::schur`] の前計算 (タイル×ポータルのミニ VI) を NVIDIA GPU に
//! 載せるビルダー。ポータル配置・成果物スキーマは CPU の [`schur::build`] と
//! 完全共有 ([`schur::place_portals`]) で、違うのはタイルソルブの実装だけ。
//!
//! # CPU との等価性 (と、意図的な差分)
//!
//! カーネル (`schur_gpu.cu`) は `value_iteration_raw` / `action_cost_raw` の
//! u64 直訳 — 同じ 2^18 固定小数点・同じ打ち切り・同じ wrapping なので、
//! 上界性の議論 (schur.rs 冒頭) がそのまま通る。意図的に違うのは 2 点:
//!
//! 1. **掃引順**: CPU の方向交互 Gauss-Seidel ではなく全セル並列の in-place
//!    パス (chaotic relaxation)。波の進みは 1 パスあたり遷移 reach 程度に
//!    落ちるので、パス上限は CPU の 64 より深い [`GPU_MAX_PASSES`]。
//! 2. **停止則**: CPU の watch (ポータル値) 安定ではなく「clamp 後の全域
//!    **減少** ≤ TOL が [`STABLE`] パス連続」。並列パスは波の到達前に watch が
//!    無変化に見える (CPU の 94% 未到達バグと同型) ため全域判定にし、
//!    未到達ポケット境界セルのクリープ (単調微増、REACH 未満に留まる) が
//!    判定を永遠に阻むので上昇は数えない (カーネル側コメント参照)。
//!
//! どちらの停止則でも途中値は下降途中の上界なので W の**契約**は同一 —
//! ただし止まる場所が違うため **CPU 成果物とはバイト一致しない** (in-place
//! 並列の混在読みで実行ごとの再現性もない)。exactify が最大不動点に着地する
//! ことは変わらず、それが `tests::gpu_artifact_is_sound` のゲート。
//!
//! パスループはホスト側で回す (1 launch = 1 パス): Windows の WDDM TDR
//! (約 2 秒でカーネル強制リセット) を構造的に踏まないため。

use std::error::Error;

use cudarc::driver::{CudaContext, LaunchConfig, PushKernelArg};
use cudarc::nvrtc::compile_ptx;

use crate::params::{MAX_COST, PROB_BASE};
use crate::solvers::schur::{place_portals, SchurArtifact, SchurConfig, TileEdges};
use crate::solvers::REACH_THRESH;
use crate::value_iterator::{to_index_raw, ValueIterator};

const KERNEL_SRC: &str = include_str!("schur_gpu.cu");
const BLOCK: u32 = 256;
/// セル並列パスの波は 1 パス ≈ 遷移 reach セルしか進まない (CPU の in-place
/// 全面ラスタは 1 パスでタイル横断) ので、CPU の TILE_MAX_ITER=64 の 4 倍。
/// house 実測では CPU も大半のソルブが上限まで走る (watch のクリープで安定
/// 判定が成立しない) — 実効的な品質の決定者はこの上限で、bench の V̂ 品質
/// (gap/violations/rollout/mismatch) が CPU と同一なことを確認済み。
const GPU_MAX_PASSES: u32 = 256;
/// watch 判定 (vi_gather + 転送) の間隔 [パス]。判定は粗くなるが、per-pass の
/// 固定費 (カーネル 1 本 + dtoh 2 本) が 1/JUDGE_EVERY になる。
const JUDGE_EVERY: u32 = 4;
/// CPU `solve_gs` と同じ実質変化の許容量。
const TOL: u64 = 2;
/// 停止に要求する連続安定パス数 (CPU の STABLE と同値)。
const STABLE: u32 = 2;
/// 1 チャンクの問題数上限。値バッファ = 問題数 × 領域状態数 × 8 B
/// (t32/h8/60θ で ~1.1 MB/問題) と、1 launch の実行時間 (TDR) の両方を縛る。
const PROBS_PER_CHUNK: usize = 1024;

/// [`schur::build`] の GPU 版。成果物のスキーマ・ポータル配置は CPU と同一。
/// CUDA デバイス/ドライバーが無ければ Err (フォールバックは呼び出し側で)。
pub fn build_gpu(src: &ValueIterator, cfg: &SchurConfig) -> Result<SchurArtifact, Box<dyn Error>> {
    let (mut art, tile_portals) = place_portals(src, cfg);
    let side = art.side();
    let nt = art.nt;
    let dom = (side * side * nt) as usize;
    let na = src.actions.len();

    // ── 遷移表の平坦化: trans_off[a*nt+it]..[+1] が trans_dat の
    //    (dix,diy,dit,prob) 4 つ組の範囲。
    let mut t_off: Vec<i32> = Vec::with_capacity(na * nt as usize + 1);
    let mut t_dat: Vec<i32> = Vec::new();
    t_off.push(0);
    for a in &src.actions {
        assert_eq!(a.state_transitions.len(), nt as usize, "state_transitions 未構築");
        for it in 0..nt as usize {
            for tr in &a.state_transitions[it] {
                t_dat.extend_from_slice(&[tr.dix, tr.diy, tr.dit, tr.prob]);
            }
            t_off.push((t_dat.len() / 4) as i32);
        }
    }

    // ── VRAM 予算: 値バッファ (n_prob × dom × 8 B) が支配項。チャンクは問題数
    //    ではなくバイト数で絞り、1 タイルの問題数が予算を超える場合は**タイル内で
    //    問題を分割**する (t256 で 1 タイル ~数百問題 × dom 33 MB = 13 GB 超 →
    //    WDDM が host へページングし ~30 倍遅くなる事故の根治)。watch は常に
    //    タイルの全パッチなので、分割しても W の列 (問題 j) は完全に埋まる。
    let dom_bytes = dom * 8;
    let probs_cap = PROBS_PER_CHUNK.min(((2usize << 30) / dom_bytes.max(1)).max(1));

    // ── ジョブ = ポータルを持つタイルの問題スライス [j0, j0+jn)。ポータルごとの
    //    パッチ (タイル内ローカル状態 index の列、順序 = tile_portals[t] =
    //    TileEdges.portals の順)。問題 = パッチを釘付けする 1 ポータル、
    //    watch = タイルの全パッチ状態 (CPU `tile_edges` と同じ)。
    struct Job {
        t: usize,
        patches: Vec<Vec<i32>>,
        j0: usize,
        jn: usize,
    }
    let mut jobs: Vec<Job> = Vec::new();
    for t in 0..art.tiles.len() {
        let plist = &tile_portals[t];
        if plist.is_empty() {
            continue;
        }
        let (tx, ty) = (t as i32 % art.tnx, t as i32 / art.tnx);
        let (x0, y0) = art.domain_origin(tx, ty);
        let patches: Vec<Vec<i32>> = plist
            .iter()
            .map(|&pid| art.patch_local_idx(src, x0, y0, side, art.portals[pid as usize]))
            .collect();
        let n = patches.len();
        let mut j0 = 0usize;
        while j0 < n {
            let jn = probs_cap.min(n - j0);
            jobs.push(Job { t, patches: patches.clone(), j0, jn });
            j0 += jn;
        }
    }
    // W を事前初期化 (分割ジョブが列単位で書き込む)。
    for t in 0..art.tiles.len() {
        let plist = &tile_portals[t];
        if !plist.is_empty() {
            let n = plist.len();
            art.tiles[t] = TileEdges { portals: plist.clone(), w: vec![MAX_COST; n * n] };
        }
    }

    let ctx = CudaContext::new(0)?;
    // 既定のスピン待ち同期は GPU 待ちホストスレッドが 1 コアを 100% 専有する。
    // パスループは同期待ちが大半なのでブロッキング同期で寝かせる (性能影響なし)。
    ctx.set_blocking_synchronize()?;
    let stream = ctx.default_stream();
    let module = ctx.load_module(compile_ptx(KERNEL_SRC)?)?;
    let k_init = module.load_function("vi_init")?;
    let k_pin = module.load_function("vi_pin")?;
    let k_pass = module.load_function("vi_pass")?;
    let k_gather = module.load_function("vi_gather")?;

    let d_toff = stream.clone_htod(&t_off)?;
    let d_tdat = stream.clone_htod(&t_dat)?;
    let cells = (side * side) as usize;
    let (nx, ny) = (src.cell_num_x, src.cell_num_y);

    // ── チャンク実行: ジョブ (≤ probs_cap 問題ずつに分割済み) を probs_cap まで詰める。
    let mut ji = 0usize;
    while ji < jobs.len() {
        let mut je = ji;
        let mut n_prob = 0usize;
        while je < jobs.len() {
            let n = jobs[je].jn;
            if n_prob > 0 && n_prob + n > probs_cap {
                break;
            }
            n_prob += n;
            je += 1;
        }
        let chunk = &jobs[ji..je];
        let n_slots = chunk.len();
        let max_watch = chunk.iter().map(|j| j.patches.iter().map(|p| p.len()).sum::<usize>()).max().unwrap();

        // slot 静的データ: free/penalty は θ 不変 (`State::from_occupancy` は
        // θ を見ない) ので xy 平面だけ持つ。地図外は free=0 / pen=PROB_BASE
        // (TileVi::hydrate と同じ)。
        let mut frees = vec![0u8; n_slots * cells];
        let mut pens = vec![PROB_BASE; n_slots * cells];
        let mut prob_slot: Vec<i32> = Vec::with_capacity(n_prob);
        let mut pin_off: Vec<i32> = vec![0];
        let mut pin_states: Vec<i32> = Vec::new();
        let mut watch_off: Vec<i32> = vec![0];
        let mut watch_states: Vec<i32> = Vec::new();
        for (slot, job) in chunk.iter().enumerate() {
            let (tx, ty) = (job.t as i32 % art.tnx, job.t as i32 / art.tnx);
            let (x0, y0) = art.domain_origin(tx, ty);
            for ly in 0..side {
                let gy = y0 + ly;
                for lx in 0..side {
                    let gx = x0 + lx;
                    if gx >= 0 && gx < nx && gy >= 0 && gy < ny {
                        let g = &src.states[to_index_raw(gx, gy, 0, nx, nt) as usize];
                        let li = slot * cells + (ly * side + lx) as usize;
                        frees[li] = g.free as u8;
                        pens[li] = g.penalty;
                    }
                }
            }
            for patch in &job.patches {
                watch_states.extend_from_slice(patch);
            }
            watch_off.push(watch_states.len() as i32);
            for patch in &job.patches[job.j0..job.j0 + job.jn] {
                prob_slot.push(slot as i32);
                pin_states.extend_from_slice(patch);
                pin_off.push(pin_states.len() as i32);
            }
        }

        let d_frees = stream.clone_htod(&frees)?;
        let d_pens = stream.clone_htod(&pens)?;
        let d_pslot = stream.clone_htod(&prob_slot)?;
        let d_poff = stream.clone_htod(&pin_off)?;
        let d_pst = stream.clone_htod(&pin_states)?;
        let d_woff = stream.clone_htod(&watch_off)?;
        let d_wst = stream.clone_htod(&watch_states)?;
        let mut d_values = stream.alloc_zeros::<u64>(n_prob * dom)?;
        let mut d_maxreal = stream.alloc_zeros::<u64>(n_prob)?;
        let mut d_wout = stream.alloc_zeros::<u64>(n_prob * max_watch)?;
        let mut done = vec![0u8; n_prob];
        let mut d_done = stream.clone_htod(&done)?;

        // init
        let total = (n_prob * dom) as i64;
        let dom_i = dom as i32;
        {
            let cfg = LaunchConfig {
                grid_dim: (((total as u64).div_ceil(BLOCK as u64)) as u32, 1, 1),
                block_dim: (BLOCK, 1, 1),
                shared_mem_bytes: 0,
            };
            let mut lb = stream.launch_builder(&k_init);
            lb.arg(&total).arg(&dom_i).arg(&mut d_values);
            unsafe { lb.launch(cfg) }?;
            // 釘付け: 問題ごとのパッチ状態を 0 に。
            let npin = pin_states.len() as i64;
            let cfg2 = LaunchConfig {
                grid_dim: (((npin as u64).div_ceil(BLOCK as u64)) as u32, 1, 1),
                block_dim: (BLOCK, 1, 1),
                shared_mem_bytes: 0,
            };
            let nprob_i = n_prob as i32;
            let mut lb = stream.launch_builder(&k_pin);
            lb.arg(&npin).arg(&nprob_i).arg(&dom_i).arg(&d_poff).arg(&d_pst).arg(&mut d_values);
            unsafe { lb.launch(cfg2) }?;
        }

        // パスループ (ホスト側)。停止則は CPU `solve_gs` の watch 方式の移植:
        // 毎パス vi_gather でポータル値 (clamp 済) を回収し、prev は |Δ|>TOL の
        // ときだけ更新、`same && pass >= 2 && (any_reached || 全域減少 ≤ TOL)`
        // が STABLE 連続で done。全域減少だけの判定は W が読まない内部の磨き
        // 残しの尾まで待ってしまう (house 実測: 16% の問題が数百パスの幾何級数
        // 減衰を磨き続け、全体で 5 倍遅)。早期打ち切りの値は下降途中の上界
        // なので健全性は CPU 同様不変 — 損なうのは W の質だけ。
        let (side_i, nt_i, na_i) = (side, nt, na as i32);
        let bx = (dom as u32).div_ceil(BLOCK);
        let gtotal = (n_prob * max_watch) as i64;
        let mw_i = max_watch as i32;
        let gbx = ((gtotal as u64).div_ceil(BLOCK as u64)) as u32;
        // prev の初期値 = init 直後の clamp 値: 問題 j の pin パッチ (watch 内の
        // 当該区間) が 0、他は MAX。
        let mut prev = vec![MAX_COST; n_prob * max_watch];
        {
            let mut pi = 0usize;
            for job in chunk {
                let mut k_off = vec![0usize];
                for patch in &job.patches {
                    k_off.push(k_off.last().unwrap() + patch.len());
                }
                for jj in 0..job.jn {
                    for k in k_off[job.j0 + jj]..k_off[job.j0 + jj + 1] {
                        prev[pi * max_watch + k] = 0;
                    }
                    pi += 1;
                }
            }
        }
        let mut stable = vec![0u32; n_prob];
        let mut wout: Vec<u64> = vec![MAX_COST; n_prob * max_watch];
        let dbg = std::env::var_os("VI_SCHUR_GPU_DEBUG").is_some();
        let t_chunk = std::time::Instant::now();
        let mut passes_run = 0u32;
        let mut active_pass_sum = 0u64;
        for pass in 0..GPU_MAX_PASSES {
            passes_run = pass + 1;
            active_pass_sum += done.iter().filter(|&&d| d == 0).count() as u64;
            let cfg = LaunchConfig {
                grid_dim: (bx, n_prob as u32, 1),
                block_dim: (BLOCK, 1, 1),
                shared_mem_bytes: 0,
            };
            let mut lb = stream.launch_builder(&k_pass);
            lb.arg(&side_i)
                .arg(&nt_i)
                .arg(&dom_i)
                .arg(&na_i)
                .arg(&d_toff)
                .arg(&d_tdat)
                .arg(&d_frees)
                .arg(&d_pens)
                .arg(&d_pslot)
                .arg(&d_done)
                .arg(&mut d_values)
                .arg(&mut d_maxreal);
            unsafe { lb.launch(cfg) }?;
            let judge = (pass + 1) % JUDGE_EVERY == 0 || pass + 1 == GPU_MAX_PASSES;
            if !judge {
                continue;
            }
            {
                let cfg = LaunchConfig {
                    grid_dim: (gbx, 1, 1),
                    block_dim: (BLOCK, 1, 1),
                    shared_mem_bytes: 0,
                };
                let mut lb = stream.launch_builder(&k_gather);
                lb.arg(&gtotal)
                    .arg(&dom_i)
                    .arg(&mw_i)
                    .arg(&d_pslot)
                    .arg(&d_woff)
                    .arg(&d_wst)
                    .arg(&d_values)
                    .arg(&mut d_wout);
                unsafe { lb.launch(cfg) }?;
            }
            let mr = stream.clone_dtoh(&d_maxreal)?;
            wout = stream.clone_dtoh(&d_wout)?;
            if dbg && (pass % 50 == 0 || pass == GPU_MAX_PASSES - 1) {
                let over = mr.iter().filter(|&&v| v > TOL).count();
                let mx = mr.iter().copied().max().unwrap_or(0);
                eprintln!(
                    "[schur_gpu]   pass {pass}: {}/{} probs with max_real>TOL, global max {}",
                    over, n_prob, mx
                );
            }
            let mut all_done = true;
            let mut changed = false;
            let mut pi = 0usize;
            for job in chunk {
                let n: usize = job.patches.iter().map(|p| p.len()).sum();
                for _ in 0..job.jn {
                    let p = pi;
                    pi += 1;
                    if done[p] == 1 {
                        continue;
                    }
                    let mut same = true;
                    for k in 0..n {
                        let v = wout[p * max_watch + k];
                        let pv = &mut prev[p * max_watch + k];
                        if v.abs_diff(*pv) > TOL {
                            *pv = v;
                            same = false;
                        }
                    }
                    let any_reached =
                        prev[p * max_watch..p * max_watch + n].iter().any(|&v| v < REACH_THRESH);
                    let stable_now = same && pass >= 2 && (any_reached || mr[p] <= TOL);
                    stable[p] = if stable_now { stable[p] + 1 } else { 0 };
                    if stable[p] >= STABLE {
                        done[p] = 1;
                        changed = true;
                    } else {
                        all_done = false;
                    }
                }
            }
            if all_done {
                break;
            }
            if changed {
                stream.memcpy_htod(&done, &mut d_done)?;
            }
            stream.memset_zeros(&mut d_maxreal)?;
        }
        if dbg {
            eprintln!(
                "[schur_gpu] chunk {}..{}: {} probs, passes {}, active-prob-passes {}, {:.0} ms",
                ji, je, n_prob, passes_run, active_pass_sum,
                t_chunk.elapsed().as_secs_f64() * 1e3
            );
        }
        // W[i][j] = パッチ i の最悪状態から パッチ j への初到達コスト (CPU `patch_max`)。
        let agg = crate::solvers::schur::patch_agg();
        let mut pbase = 0usize;
        for job in chunk {
            let n = job.patches.len();
            let w = &mut art.tiles[job.t].w;
            let mut k_off: Vec<usize> = vec![0];
            for patch in &job.patches {
                k_off.push(k_off.last().unwrap() + patch.len());
            }
            for jj in 0..job.jn {
                let j = job.j0 + jj;
                for i in 0..n {
                    if i == j {
                        w[i * n + j] = 0;
                        continue;
                    }
                    let row = &wout[(pbase + jj) * max_watch..];
                    let mut mx = 0u64;
                    let (mut sum, mut cnt) = (0u128, 0u64);
                    let mut any = false;
                    for k in k_off[i]..k_off[i + 1] {
                        any = true;
                        mx = mx.max(row[k]); // gather 済み = clamp 済み
                        if row[k] < MAX_COST {
                            sum += row[k] as u128;
                            cnt += 1;
                        }
                    }
                    w[i * n + j] = match (any, agg) {
                        (false, _) => MAX_COST,
                        (true, crate::solvers::schur::PatchAgg::Mean) => if cnt > 0 { (sum / cnt as u128) as u64 } else { MAX_COST },
                        (true, crate::solvers::schur::PatchAgg::Center) => row[k_off[i]], // パッチ列挙の先頭は中心ではないが近似
                        (true, _) => mx,
                    };
                }
            }
            pbase += job.jn;
        }
        ji = je;
    }
    Ok(art)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solvers::schur::testfix::{cfg, corridor_vi};
    use crate::solvers::schur::{build, schur_solve_with, upper_bound_field};
    use crate::solvers::test_support::{run_reference_to_fixed_point, REACH};
    use crate::solvers::NullObserver;

    /// GPU 成果物の健全性: (1) ポータル配置は CPU と同一、(2) V̂ は全点上界、
    /// (3) exactify 後は Reference 固定点に bit-exact。W そのものは停止則の
    /// 違いで CPU と一致しなくてよい (モジュール docs 参照)。
    #[test]
    fn gpu_artifact_is_sound() {
        let mut exact = corridor_vi();
        run_reference_to_fixed_point(&mut exact);
        let vi0 = corridor_vi();
        let cpu_art = build(&vi0, &cfg());
        let art = match build_gpu(&vi0, &cfg()) {
            Ok(a) => a,
            // GPU の無い環境 (CI 等) では計測不能なのでスキップ扱い。
            Err(e) => {
                eprintln!("skip: CUDA 初期化不可 ({e})");
                return;
            }
        };
        assert_eq!(cpu_art.portals, art.portals);
        for (c, g) in cpu_art.tiles.iter().zip(&art.tiles) {
            assert_eq!(c.portals, g.portals);
        }

        let mut vhat = corridor_vi();
        upper_bound_field(&mut vhat, &art);
        for i in 0..exact.states.len() {
            assert!(
                vhat.states[i].total_cost >= exact.states[i].total_cost,
                "上界破れ @ {i}: {} < {}",
                vhat.states[i].total_cost,
                exact.states[i].total_cost
            );
        }

        let mut vi = corridor_vi();
        let out = schur_solve_with(&mut vi, &art, 100_000, &mut NullObserver);
        assert!(out.converged);
        for i in 0..exact.states.len() {
            if exact.states[i].total_cost < REACH {
                assert_eq!(
                    exact.states[i].total_cost, vi.states[i].total_cost,
                    "値 mismatch @ {i} (ix={},iy={},it={})",
                    exact.states[i].ix, exact.states[i].iy, exact.states[i].it
                );
            }
        }
    }
}
