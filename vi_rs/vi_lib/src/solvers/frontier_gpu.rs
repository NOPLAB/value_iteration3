//! frontier2d の **GPU 版コールドソルブ** (cudarc + NVRTC、feature "cuda")。
//!
//! # なぜこれが「任意ゴールを瞬時に厳密」の本命か
//!
//! tsudanuma s3 (1.57 億状態) のコールド frontier2d は 16 スレッド CPU で 8〜16 s。
//! 内訳を測ると仕事量は 3.3 億更新 (7.8 n) で、1 更新あたり数十回のランダム読み
//! (後継の値 + penalty) — つまり **メモリ律速**。上界場からの warm start (Schur V̂ /
//! 近傍ゴールの場) は光円錐のラウンド数こそ減らすが仕事量は増やす (house: 11 n vs
//! コールド 5.4 n) — 補正が全域の再更新を強いるのに対し、コールドの波は自然にほぼ
//! Dijkstra 順で各状態を確定させる ([`super::ordered`] の実測)。残る手段は**帯域**で、
//! 前計算もディスクも要らない。
//!
//! # 等価性
//!
//! カーネル (`frontier_gpu.cu`) は [`super::frontier2d_pad::action_cost_pad`] の
//! u64 直訳 (2^18 固定小数点・同じ打ち切り・同じ wrapping)。ラウンド内は候補状態の
//! 並列 in-place 更新 (chaotic relaxation) で、MAX_COST 起点の単調降下 +
//! 「1 ラウンド丸ごと無変化で停止」なので、収束値は frontier2d / Reference と
//! bit-exact (`frontier2d_par_unsafe` と同じ Bertsekas–Tsitsiklis 非同期 VI の議論)。
//! 方策は収束場の argmin を GPU で引く (CPU `final_policy` と同じタイブレーク)。
//!
//! # 構造
//!
//! [`GpuSolver::new`] が地図の静的データ (penalty / free / 遷移表) を θ-major
//! レイアウトでデバイスに常駐させ、[`GpuSolver::solve`] はゴールごとに final 列だけ
//! 上げて値をリセットし、ラウンドループ (膨張 → 更新 → 動いた列数の回収、1 launch =
//! 1 ラウンドで WDDM TDR を踏まない) を回す。[`GpuSolver::download`] が値と方策を
//! `vi.states` へ書き戻す (全場が要るときだけ)。

use std::error::Error;
use std::sync::Arc;
use std::time::Instant;

use cudarc::driver::{
    CudaContext, CudaFunction, CudaModule, CudaSlice, CudaStream, LaunchConfig, PushKernelArg,
};
use cudarc::nvrtc::compile_ptx;

use crate::params::MAX_COST;
use crate::value_iterator::ValueIterator;

use super::frontier2d_pad::Padded;

const KERNEL_SRC: &str = include_str!("frontier_gpu.cu");
const BLOCK: u32 = 256;

#[derive(Clone, Copy, Debug, Default)]
pub struct GpuSolveStats {
    pub rounds: u32,
    pub updates: u64,
    pub converged: bool,
    /// ゴールごとの前処理 (final 転送 + 値リセット) [ms]。
    pub t_reset_ms: f64,
    /// ラウンドループ [ms]。
    pub t_solve_ms: f64,
}

/// 地図常駐の GPU ソルバ。
pub struct GpuSolver {
    _ctx: Arc<CudaContext>,
    stream: Arc<CudaStream>,
    _module: Arc<CudaModule>,
    k_reset: CudaFunction,
    k_dilate: CudaFunction,
    k_update: CudaFunction,
    k_policy: CudaFunction,
    nt: i32,
    nx_pad: i32,
    mx: i32,
    my: i32,
    n_pad: usize,
    n_cols: usize,
    na: i32,
    d_val: CudaSlice<u64>,
    d_pen: CudaSlice<u32>,
    d_free: CudaSlice<u8>,
    d_fin: CudaSlice<u8>,
    d_col_free: CudaSlice<u8>,
    d_pc_off: CudaSlice<i64>,
    d_pc_prob: CudaSlice<u32>,
    d_pc_start: CudaSlice<u32>,
    d_chg_flag: CudaSlice<u32>,
    d_cand_flag: CudaSlice<u32>,
    d_chg_list: CudaSlice<u32>,
    d_cand_list: CudaSlice<u32>,
    d_chg_cnt: CudaSlice<u32>,
    d_cand_cnt: CudaSlice<u32>,
    d_updates: CudaSlice<u64>,
    /// 地図常駐化にかかった時間 [ms] (転送込み)。
    pub t_upload_ms: f64,
}

impl GpuSolver {
    /// θ-major 添字: idx = it·plane + col、col = (ix+mx) + (iy+my)·nx_pad。
    #[inline]
    fn idx(&self, ix: i32, iy: i32, it: i32) -> usize {
        let col = ((ix + self.mx) + (iy + self.my) * self.nx_pad) as usize;
        it as usize * self.n_cols + col
    }

    /// 地図 (states 構築済み `vi`) の静的データをデバイスへ常駐させる。ゴールは見ない。
    pub fn new(vi: &ValueIterator) -> Result<Self, Box<dyn Error>> {
        let t0 = Instant::now();
        let m = Padded::build(vi);
        let (nt, nx_pad, mx, my) = (m.nt, m.nx_pad, m.mx, m.my);
        let n_pad = m.hot.len();
        let n_cols = n_pad / nt as usize;
        let na = m.precomp.len();
        let plane = n_cols;

        let mut pens: Vec<u32> = vec![0; n_pad];
        let mut frees: Vec<u8> = vec![0; n_pad];
        for (i, h) in m.hot.iter().enumerate() {
            let p = h[1];
            if p > u32::MAX as u64 {
                return Err(format!("penalty {p} exceeds u32 (GPU kernel stores penalties as u32)").into());
            }
            let j = (i % nt as usize) * plane + i / nt as usize;
            pens[j] = p as u32;
            frees[j] = m.free[i] as u8;
        }
        let mut col_free = vec![0u8; n_cols];
        for c in 0..n_cols {
            let base = c * nt as usize;
            col_free[c] = m.free[base..base + nt as usize].iter().any(|&f| f) as u8;
        }
        // 遷移 (action, θ) → (着地 θ の絶対平面 + 列相対オフセット, prob)。
        let mut pc_start: Vec<u32> = Vec::with_capacity(na * nt as usize + 1);
        let mut pc_off: Vec<i64> = Vec::new();
        let mut pc_prob: Vec<u32> = Vec::new();
        pc_start.push(0);
        for a in &vi.actions {
            for it in 0..nt as usize {
                for tr in &a.state_transitions[it] {
                    let nit = (tr.dit + nt) % nt;
                    let off = nit as i64 * plane as i64 + tr.dix as i64 + tr.diy as i64 * nx_pad as i64;
                    pc_off.push(off);
                    pc_prob.push(tr.prob as u32);
                }
                pc_start.push(pc_off.len() as u32);
            }
        }

        let ctx = CudaContext::new(0)?;
        // 同期はスピン既定のまま: ラウンドごとに列数を読み戻すので、ブロッキング同期
        // (OS の起床遅延 ~ms) だと 1000 ラウンドで数秒の固定費になる。
        let stream = ctx.default_stream();
        let module = ctx.load_module(compile_ptx(KERNEL_SRC)?)?;
        let k_reset = module.load_function("vi_reset")?;
        let k_dilate = module.load_function("vi_dilate")?;
        let k_update = module.load_function("vi_update")?;
        let k_policy = module.load_function("vi_policy")?;

        let d_val = stream.alloc_zeros::<u64>(n_pad)?;
        let d_pen = stream.clone_htod(&pens)?;
        let d_free = stream.clone_htod(&frees)?;
        let d_fin = stream.alloc_zeros::<u8>(n_pad)?;
        let d_col_free = stream.clone_htod(&col_free)?;
        let d_pc_off = stream.clone_htod(&pc_off)?;
        let d_pc_prob = stream.clone_htod(&pc_prob)?;
        let d_pc_start = stream.clone_htod(&pc_start)?;
        let d_chg_flag = stream.alloc_zeros::<u32>(n_cols)?;
        let d_cand_flag = stream.alloc_zeros::<u32>(n_cols)?;
        let d_chg_list = stream.alloc_zeros::<u32>(n_cols)?;
        let d_cand_list = stream.alloc_zeros::<u32>(n_cols)?;
        let d_chg_cnt = stream.alloc_zeros::<u32>(1)?;
        let d_cand_cnt = stream.alloc_zeros::<u32>(1)?;
        let d_updates = stream.alloc_zeros::<u64>(1)?;
        stream.synchronize()?;
        let t_upload_ms = t0.elapsed().as_secs_f64() * 1e3;
        Ok(Self {
            _ctx: ctx,
            stream,
            _module: module,
            k_reset,
            k_dilate,
            k_update,
            k_policy,
            nt,
            nx_pad,
            mx,
            my,
            n_pad,
            n_cols,
            na: na as i32,
            d_val,
            d_pen,
            d_free,
            d_fin,
            d_col_free,
            d_pc_off,
            d_pc_prob,
            d_pc_start,
            d_chg_flag,
            d_cand_flag,
            d_chg_list,
            d_cand_list,
            d_chg_cnt,
            d_cand_cnt,
            d_updates,
            t_upload_ms,
        })
    }

    /// `set_goal` 済み `vi` の final 集合で場をリセットし、固定点まで解く。
    /// 値はデバイスに残る ([`Self::download`] で書き戻す)。
    pub fn solve(&mut self, vi: &ValueIterator, max_rounds: u32) -> Result<GpuSolveStats, Box<dyn Error>> {
        let t0 = Instant::now();
        let (nt, n_cols, n_pad) = (self.nt, self.n_cols, self.n_pad);
        // final: θ-major の u8 配列 + 初期 chg 列。
        let mut fins: Vec<u8> = vec![0; n_pad];
        let mut chg_flag: Vec<u32> = vec![0; n_cols];
        let mut chg0: Vec<u32> = Vec::new();
        for s in &vi.states {
            if s.final_state {
                let j = self.idx(s.ix, s.iy, s.it);
                fins[j] = 1;
                let col = (j % n_cols) as u32;
                if chg_flag[col as usize] == 0 {
                    chg_flag[col as usize] = 1;
                    chg0.push(col);
                }
            }
        }
        let stream = &self.stream;
        stream.memcpy_htod(&fins, &mut self.d_fin)?;
        stream.memcpy_htod(&chg_flag, &mut self.d_chg_flag)?;
        {
            let mut list = chg0.clone();
            list.resize(n_cols, 0);
            stream.memcpy_htod(&list, &mut self.d_chg_list)?;
        }
        stream.memset_zeros(&mut self.d_cand_flag)?;
        stream.memset_zeros(&mut self.d_updates)?;
        // 値リセット: final = 0、他 = MAX_COST。
        {
            let n_pad_i = n_pad as i64;
            let cfg = LaunchConfig {
                grid_dim: ((n_pad as u64).div_ceil(BLOCK as u64) as u32, 1, 1),
                block_dim: (BLOCK, 1, 1),
                shared_mem_bytes: 0,
            };
            let mut lb = stream.launch_builder(&self.k_reset);
            lb.arg(&n_pad_i).arg(&self.d_fin).arg(&mut self.d_val);
            unsafe { lb.launch(cfg) }?;
        }
        stream.synchronize()?;
        let t_reset_ms = t0.elapsed().as_secs_f64() * 1e3;

        // ── ラウンドループ。
        let t1 = Instant::now();
        let (nx_pad_i, mx_i, my_i, nt_i, na_i) = (self.nx_pad, self.mx, self.my, nt, self.na);
        let plane_i = n_cols as i64;
        let mut chg_cnt = chg0.len() as u32;
        let mut rounds = 0u32;
        let mut cnt_host = vec![0u32; 1];
        while chg_cnt > 0 && rounds < max_rounds {
            rounds += 1;
            // 膨張: chg → cand。
            stream.memset_zeros(&mut self.d_cand_cnt)?;
            {
                let cfg = LaunchConfig {
                    grid_dim: (chg_cnt.div_ceil(BLOCK), 1, 1),
                    block_dim: (BLOCK, 1, 1),
                    shared_mem_bytes: 0,
                };
                let mut lb = stream.launch_builder(&self.k_dilate);
                lb.arg(&self.d_chg_list)
                    .arg(&chg_cnt)
                    .arg(&nx_pad_i)
                    .arg(&mx_i)
                    .arg(&my_i)
                    .arg(&self.d_col_free)
                    .arg(&mut self.d_chg_flag)
                    .arg(&mut self.d_cand_flag)
                    .arg(&mut self.d_cand_list)
                    .arg(&mut self.d_cand_cnt);
                unsafe { lb.launch(cfg) }?;
            }
            // 更新: cand → chg。候補数を読み戻して正確な grid で launch する
            // (上界 chg_cnt × 膨張面積で launch すると halo 6 の地図で 98% が空転した)。
            stream.memcpy_dtoh(&self.d_cand_cnt, &mut cnt_host)?;
            let cand_cnt = cnt_host[0];
            if cand_cnt == 0 {
                chg_cnt = 0;
                break;
            }
            stream.memset_zeros(&mut self.d_chg_cnt)?;
            {
                let total = cand_cnt as u64 * nt as u64;
                let cfg = LaunchConfig {
                    grid_dim: (total.div_ceil(BLOCK as u64) as u32, 1, 1),
                    block_dim: (BLOCK, 1, 1),
                    shared_mem_bytes: 0,
                };
                let mut lb = stream.launch_builder(&self.k_update);
                lb.arg(&self.d_cand_list)
                    .arg(&cand_cnt)
                    .arg(&nt_i)
                    .arg(&plane_i)
                    .arg(&mut self.d_val)
                    .arg(&self.d_pen)
                    .arg(&self.d_free)
                    .arg(&self.d_fin)
                    .arg(&self.d_pc_off)
                    .arg(&self.d_pc_prob)
                    .arg(&self.d_pc_start)
                    .arg(&na_i)
                    .arg(&mut self.d_cand_flag)
                    .arg(&mut self.d_chg_flag)
                    .arg(&mut self.d_chg_list)
                    .arg(&mut self.d_chg_cnt)
                    .arg(&mut self.d_updates);
                unsafe { lb.launch(cfg) }?;
            }
            stream.memcpy_dtoh(&self.d_chg_cnt, &mut cnt_host)?;
            chg_cnt = cnt_host[0];
        }
        stream.synchronize()?;
        let t_solve_ms = t1.elapsed().as_secs_f64() * 1e3;
        let updates = stream.clone_dtoh(&self.d_updates)?[0];
        Ok(GpuSolveStats { rounds, updates, converged: chg_cnt == 0, t_reset_ms, t_solve_ms })
    }

    /// デバイスの収束場から値・方策を `vi.states` へ書き戻す。戻り値は所要 [ms]。
    pub fn download(&mut self, vi: &mut ValueIterator) -> Result<f64, Box<dyn Error>> {
        let t0 = Instant::now();
        let stream = &self.stream;
        let n_pad = self.n_pad;
        let mut d_opt = stream.alloc_zeros::<i8>(n_pad)?;
        {
            let n_pad_i = n_pad as i64;
            let plane_i = self.n_cols as i64;
            let cfg = LaunchConfig {
                grid_dim: ((n_pad as u64).div_ceil(BLOCK as u64) as u32, 1, 1),
                block_dim: (BLOCK, 1, 1),
                shared_mem_bytes: 0,
            };
            let mut lb = stream.launch_builder(&self.k_policy);
            lb.arg(&n_pad_i)
                .arg(&self.nt)
                .arg(&plane_i)
                .arg(&self.d_val)
                .arg(&self.d_pen)
                .arg(&self.d_free)
                .arg(&self.d_fin)
                .arg(&self.d_pc_off)
                .arg(&self.d_pc_prob)
                .arg(&self.d_pc_start)
                .arg(&self.na)
                .arg(&mut d_opt);
            unsafe { lb.launch(cfg) }?;
        }
        let vals = stream.clone_dtoh(&self.d_val)?;
        let opt = stream.clone_dtoh(&d_opt)?;
        for s in vi.states.iter_mut() {
            let j = self.idx(s.ix, s.iy, s.it);
            s.total_cost = vals[j];
            s.optimal_action = if s.final_state || vals[j] >= MAX_COST {
                None
            } else {
                (opt[j] >= 0).then_some(opt[j] as usize)
            };
        }
        Ok(t0.elapsed().as_secs_f64() * 1e3)
    }
}

/// 一発呼び: 常駐 → solve → 書き戻し。計測では [`GpuSolver`] を直接使って
/// 常駐コストとゴールごとのコストを分ける。
pub fn frontier_gpu_solve(vi: &mut ValueIterator, max_rounds: u32) -> Result<GpuSolveStats, Box<dyn Error>> {
    let mut g = GpuSolver::new(vi)?;
    let st = g.solve(vi, max_rounds)?;
    g.download(vi)?;
    Ok(st)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solvers::test_support::make_vi;
    use crate::solvers::{solve, U64Solver};

    /// GPU コールドソルブが frontier2d と値・方策とも bit-exact (GPU が無ければ skip)。
    /// 同じ常駐ソルバで 2 つ目のゴールも解けること (リセットの健全性) も見る。
    #[test]
    fn gpu_solve_matches_frontier2d() {
        let (w, h) = (48, 36);
        let mut occ = vec![0i8; (w * h) as usize];
        for y in 6..30 {
            occ[(y * w + 24) as usize] = 100;
        }
        let mut b = make_vi(w, h, occ.clone());
        let mut g = match GpuSolver::new(&b) {
            Ok(g) => g,
            Err(e) => {
                let msg = e.to_string();
                assert!(!msg.contains("CompileError"), "kernel compile failed: {msg}");
                eprintln!("skip (no GPU): {msg}");
                return;
            }
        };
        for (gx, gy) in [(0.10, 0.10), (2.0, 1.5)] {
            let mut a = make_vi(w, h, occ.clone());
            a.set_goal(gx, gy, 0);
            let st = solve(&mut a, U64Solver::Frontier2D, 100_000);
            assert!(st.converged);
            b.set_goal(gx, gy, 0);
            let gs = g.solve(&b, 1_000_000).unwrap();
            assert!(gs.converged, "{gs:?}");
            g.download(&mut b).unwrap();
            let mut mm = 0usize;
            let mut mp = 0usize;
            for (x, y) in a.states.iter().zip(&b.states) {
                if x.total_cost != y.total_cost {
                    mm += 1;
                }
                if x.total_cost < MAX_COST && x.optimal_action != y.optimal_action {
                    mp += 1;
                }
            }
            assert_eq!(mm, 0, "{gs:?}");
            assert_eq!(mp, 0, "{gs:?}");
            eprintln!("gpu goal ({gx},{gy}): {gs:?}");
        }
    }
}
