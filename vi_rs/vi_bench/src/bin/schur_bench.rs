//! `schur_bench` — タイル通過行列 (Schur 補元) ソルバの計測 CLI。
//!
//! 実地図 (PGM+YAML) 上で:
//! 1. frontier2d のコールドソルブをオラクル (基準時間 + bit-exact 比較先) にし、
//! 2. (tile, portal_thetas) の各設定でアーティファクト構築 → ポータル Dijkstra →
//!    lazy 1 タイル復元 → 全場復元 (上界 V̂) → exactify を段階別に計測、
//! 3. V̂ の上界ギャップ (対オラクル相対)、上界破れ数 (期待 0)、exactify 後の
//!    mismatch 数 (期待 0)、V̂ 上の greedy rollout 成功率を報告する。
//!
//! 出力: `--out` に report.json と、先頭設定の可視化 PPM 4 枚
//! (地図+タイル+ポータル / V̂ θ=0 スライス / オラクル同 / 相対ギャップ)。

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use clap::Parser;

use vi_bench::params::{canonical_actions, N_THETA};
use vi_bench::pgm;
use vi_lib::planner::{pose_to_cell, rollout_path_on};
use vi_lib::solvers::ordered::ordered_sweep;
use vi_lib::solvers::schur::{
    build, corridor_tiles, materialize_one, materialize_one_multi, portal_costs,
    portal_costs_masked, upper_bound_field, SchurArtifact, SchurConfig, TileScratch,
};
use vi_lib::solvers::{solve, U64Solver, REACH_THRESH as REACH};
use vi_lib::{OccupancyGrid, Quaternion, State, ValueIterator};

#[derive(Parser)]
#[command(about = "Benchmark the Schur tile-through-matrix solver on a real PGM/YAML map.")]
struct Args {
    /// Map YAML (image: は YAML 相対)。既定は tb3_house。
    #[arg(long)]
    map: Option<PathBuf>,

    /// 整数ダウンサンプル係数 (1 = フル解像度)。
    #[arg(long, default_value_t = 1)]
    scale: usize,

    /// 試すタイル一辺 [セル] (カンマ区切り)。
    #[arg(long, default_value = "32")]
    tiles: String,

    /// 試すポータル heading 数 (カンマ区切り)。
    #[arg(long, default_value = "4,8,12")]
    thetas: String,

    /// 界面上のポータル間隔 [セル] (カンマ区切り、thetas と直積)。
    #[arg(long, default_value = "16")]
    spacings: String,

    /// ゴール世界座標 (省略時は地図中心を free へスナップ)。
    #[arg(long)]
    goal_x: Option<f64>,
    #[arg(long)]
    goal_y: Option<f64>,
    #[arg(long, default_value_t = 90.0)]
    goal_theta_deg: f64,

    /// 第 2 ゴール (近傍ゴール再利用 = B の計測)。省略時はスキップ。
    /// 第 1 ゴールの厳密場 + 定数 c (= ゴール 1 からゴール 2 への最大コスト) を
    /// 上界・掃引順序にして ordered sweep で第 2 ゴールの固定点へ行く。
    #[arg(long)]
    goal2_x: Option<f64>,
    #[arg(long)]
    goal2_y: Option<f64>,
    #[arg(long, default_value_t = 90.0)]
    goal2_theta_deg: f64,

    /// GPU コールドソルブ (vi_lib::solvers::frontier_gpu、要 feature "gpu") も測り、
    /// オラクルと bit-exact か検証する。
    #[arg(long, default_value_t = false)]
    gpu_solve: bool,

    /// 対照: 順序なし (全列 1 バケット = 添字順 GS) のコールド掃引も測る。
    /// 光円錐ぶんのパス数がかかるので小地図向け。
    #[arg(long, default_value_t = false)]
    cold_ordered: bool,

    /// unknown (灰) セルを obstacle 扱いにするか。
    #[arg(long, default_value_t = true)]
    unknown_as_obstacle: bool,

    #[arg(long, default_value_t = 0.2)]
    safety_radius_m: f64,
    #[arg(long, default_value_t = 30.0)]
    safety_penalty: f64,

    /// rollout 標本数 (オラクル場で成功した開始点を V̂ 側と比較)。
    #[arg(long, default_value_t = 200)]
    rollout_samples: usize,

    /// 出力ディレクトリ。
    #[arg(long, default_value = "schur_bench_out")]
    out: PathBuf,

    /// out に保存済みアーティファクトがあれば build せず読み込む
    /// (クエリ側の変更を再検証するときの再ビルド回避)。
    #[arg(long, default_value_t = false)]
    reuse: bool,

    /// 前計算を GPU (vi_lib::solvers::schur_gpu, 要 feature "gpu") で行う。
    /// W は CPU 版とバイト一致しない (停止則の差 — schur_gpu docs 参照) が、
    /// 上界性と exactify の固定点は同一。
    #[arg(long, default_value_t = false)]
    gpu: bool,

    /// 2 段ポータルグラフの計測: この設定を「粗」、--fine-thetas/--fine-spacing の
    /// 成果物を「密」として、粗 全域 Dijkstra → 回廊選択 → 密 回廊限定 Dijkstra →
    /// ロボットタイル復元 (粗のみ / 2 段 / 密全域) を開始点ごとに比較する。
    #[arg(long, default_value_t = false)]
    two_level: bool,
    #[arg(long, default_value_t = 15)]
    fine_thetas: i32,
    #[arg(long, default_value_t = 8)]
    fine_spacing: i32,
    /// 回廊のタイル膨張 [環] (粗最短路木の通過タイルからの拡張)。
    #[arg(long, default_value_t = 1)]
    corridor_dilate: i32,
    /// 2 段計測に使う開始点の最大数 (rollout starts から等間隔サンプル)。
    #[arg(long, default_value_t = 12)]
    tl_starts: usize,

    /// ordered exactify をスキップする (Padded 一式のメモリピークと ~4 分/設定の回避)。
    #[arg(long, default_value_t = false)]
    no_ordered: bool,
}

fn default_map_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/tb3_house/map.yaml")
}

fn parse_list(s: &str) -> Vec<i32> {
    s.split(',').filter_map(|t| t.trim().parse().ok()).collect()
}

/// `set_state` 相当を sweep_orders なしで (行バンド並列)。大地図で 24 B/state を
/// 節約する — frontier2d も schur も sweep_orders を読まない。
fn build_states(grid: &OccupancyGrid, nt: i32, safety_radius: f64, penalty: f64) -> Vec<State> {
    let margin = (safety_radius / grid.resolution).ceil() as i32;
    let (nx, ny) = (grid.width, grid.height);
    let nthr = std::thread::available_parallelism().map(|p| p.get()).unwrap_or(1);
    let rows: Vec<i32> = (0..ny).collect();
    let chunk = rows.len().div_ceil(nthr).max(1);
    let bands: Vec<Vec<State>> = std::thread::scope(|scope| {
        let handles: Vec<_> = rows
            .chunks(chunk)
            .map(|band| {
                scope.spawn(move || {
                    let mut out =
                        Vec::with_capacity(band.len() * nx as usize * nt as usize);
                    for &y in band {
                        for x in 0..nx {
                            for t in 0..nt {
                                out.push(State::from_occupancy(
                                    x, y, t, grid, margin, penalty, nx,
                                ));
                            }
                        }
                    }
                    out
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    bands.concat()
}

fn snap_to_free(occ: &[i8], w: i32, h: i32, gx: i32, gy: i32, max_r: i32) -> Option<(i32, i32)> {
    let at = |x: i32, y: i32| (y * w + x) as usize;
    if gx >= 0 && gx < w && gy >= 0 && gy < h && occ[at(gx, gy)] == 0 {
        return Some((gx, gy));
    }
    for r in 1..=max_r {
        for dy in -r..=r {
            for dx in -r..=r {
                if dx.abs() != r && dy.abs() != r {
                    continue;
                }
                let (nx2, ny2) = (gx + dx, gy + dy);
                if nx2 >= 0 && ny2 >= 0 && nx2 < w && ny2 < h && occ[at(nx2, ny2)] == 0 {
                    return Some((nx2, ny2));
                }
            }
        }
    }
    None
}

// ── PPM 出力 (P6) ──────────────────────────────────────────────────────────

struct Img {
    w: i32,
    h: i32,
    px: Vec<[u8; 3]>,
}

impl Img {
    fn new(w: i32, h: i32, fill: [u8; 3]) -> Self {
        Self { w, h, px: vec![fill; (w * h) as usize] }
    }
    /// グリッド座標 (iy=0 が下) → 画像 (行 0 が上)。
    fn set(&mut self, ix: i32, iy: i32, c: [u8; 3]) {
        if ix < 0 || iy < 0 || ix >= self.w || iy >= self.h {
            return;
        }
        self.px[((self.h - 1 - iy) * self.w + ix) as usize] = c;
    }
    fn save(&self, path: &std::path::Path) -> std::io::Result<()> {
        use std::io::Write;
        let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
        write!(f, "P6\n{} {}\n255\n", self.w, self.h)?;
        for p in &self.px {
            f.write_all(p)?;
        }
        Ok(())
    }
}

/// viridis 風 11 段グラデーション。t ∈ [0,1]。
fn viridis(t: f64) -> [u8; 3] {
    const STOPS: [[f64; 3]; 11] = [
        [0.267, 0.005, 0.329],
        [0.283, 0.141, 0.458],
        [0.254, 0.265, 0.530],
        [0.207, 0.372, 0.553],
        [0.164, 0.471, 0.558],
        [0.128, 0.567, 0.551],
        [0.135, 0.659, 0.518],
        [0.267, 0.749, 0.441],
        [0.478, 0.821, 0.318],
        [0.741, 0.873, 0.150],
        [0.993, 0.906, 0.144],
    ];
    let t = t.clamp(0.0, 1.0) * 10.0;
    let i = (t.floor() as usize).min(9);
    let f = t - i as f64;
    let mix = |a: f64, b: f64| ((a + (b - a) * f) * 255.0) as u8;
    [
        mix(STOPS[i][0], STOPS[i + 1][0]),
        mix(STOPS[i][1], STOPS[i + 1][1]),
        mix(STOPS[i][2], STOPS[i + 1][2]),
    ]
}

/// 値場の θ=0 スライスをヒートマップに描く (到達域の p99 でクリップ)。
fn value_slice_img(vi: &ValueIterator, vals: impl Fn(usize) -> u64) -> Img {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let mut reach: Vec<u64> = Vec::new();
    for iy in 0..ny {
        for ix in 0..nx {
            let v = vals((ix * nt + iy * nt * nx) as usize);
            if v > 0 && v < REACH {
                reach.push(v);
            }
        }
    }
    reach.sort_unstable();
    let p99 = reach.get(reach.len().saturating_mul(99) / 100).copied().unwrap_or(1).max(1);
    let mut img = Img::new(nx, ny, [30, 30, 34]);
    for iy in 0..ny {
        for ix in 0..nx {
            let idx = (ix * nt + iy * nt * nx) as usize;
            let s = &vi.states[idx];
            let v = vals(idx);
            let c = if !s.free {
                [15, 15, 18]
            } else if v == 0 {
                [255, 255, 255]
            } else if v >= REACH {
                [60, 60, 66]
            } else {
                viridis(1.0 - (v as f64 / p99 as f64).min(1.0))
            };
            img.set(ix, iy, c);
        }
    }
    img
}

/// オラクルとの値 mismatch 数 (到達可能セル) と flap 圏外の数。
fn mismatch_vs(vi: &ValueIterator, oracle: &[u64], flap_mask: &[bool], ow: i32) -> (u64, u64) {
    let (mut mm, mut out) = (0u64, 0u64);
    for (i, s) in vi.states.iter().enumerate() {
        if oracle[i] < REACH && s.total_cost != oracle[i] {
            mm += 1;
            if !flap_mask[(s.iy * ow + s.ix) as usize] {
                out += 1;
            }
        }
    }
    (mm, out)
}

/// 上界場の相対ギャップ平均 (到達可能・非ゴール) と上界破れ数。
fn gap_vs(vi: &ValueIterator, oracle: &[u64]) -> (f64, u64) {
    let (mut n, mut sum, mut viol) = (0u64, 0f64, 0u64);
    for (i, s) in vi.states.iter().enumerate() {
        let a = oracle[i];
        if a >= REACH || a == 0 {
            continue;
        }
        n += 1;
        let b = s.total_cost;
        if b < a {
            viol += 1;
        } else if b < REACH {
            sum += (b - a) as f64 / a as f64;
        } else {
            sum += 10.0; // 未到達は 1000% として数える (平均を壊さない程度の飽和)
        }
    }
    (if n > 0 { sum / n as f64 } else { 0.0 }, viol)
}

fn main() -> ExitCode {
    let args = Args::parse();
    let map_path = args.map.clone().unwrap_or_else(default_map_path);
    let tiles = parse_list(&args.tiles);
    let thetas = parse_list(&args.thetas);
    let spacings = parse_list(&args.spacings);
    std::fs::create_dir_all(&args.out).expect("mkdir out");

    eprintln!("loading map: {}", map_path.display());
    let map = match pgm::load(&map_path) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::from(2);
        }
    };
    let res = map.meta.resolution * args.scale as f64;
    let (occ, ow, oh) = pgm::build_occupancy(&map, args.scale, args.unknown_as_obstacle);
    let grid = OccupancyGrid {
        width: ow,
        height: oh,
        resolution: res,
        origin_x: map.meta.origin_x,
        origin_y: map.meta.origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: occ,
    };
    let nt = N_THETA;
    let n_states = (ow as u64) * (oh as u64) * nt as u64;
    eprintln!(
        "grid: {}x{}x{} = {} states ({:.3} m/cell), states mem ~{:.2} GB",
        ow,
        oh,
        nt,
        n_states,
        res,
        n_states as f64 * 56.0 / 1e9
    );

    // ゴール (地図中心 → free スナップ)。
    let goal_x = args.goal_x.unwrap_or(map.meta.origin_x + map.width as f64 * map.meta.resolution / 2.0);
    let goal_y = args.goal_y.unwrap_or(map.meta.origin_y + map.height as f64 * map.meta.resolution / 2.0);
    let req_gx = (((goal_x - grid.origin_x) / res).floor() as i32).clamp(0, ow - 1);
    let req_gy = (((goal_y - grid.origin_y) / res).floor() as i32).clamp(0, oh - 1);
    let Some((gx, gy)) = snap_to_free(&grid.data, ow, oh, req_gx, req_gy, ow.max(oh)) else {
        eprintln!("error: no free cell near goal");
        return ExitCode::from(2);
    };
    let goal_wx = grid.origin_x + (gx as f64 + 0.5) * res;
    let goal_wy = grid.origin_y + (gy as f64 + 0.5) * res;
    let goal_radius_m = (2.0 * res).max(0.5);
    let goal_t = args.goal_theta_deg as i32;
    eprintln!("goal: ({goal_wx:.2}, {goal_wy:.2}) th {goal_t} deg, radius {goal_radius_m:.2} m");

    // VI 構築 (sweep_orders なし)。
    let t0 = Instant::now();
    let mut vi = ValueIterator::new(canonical_actions(), 1);
    vi.set_map_geometry_no_states(&grid, nt, goal_radius_m, 15);
    vi.states = build_states(&grid, nt, args.safety_radius_m, args.safety_penalty);
    let t_setup = t0.elapsed().as_secs_f64() * 1e3;
    vi.set_goal(goal_wx, goal_wy, goal_t);
    eprintln!("setup: {:.0} ms (transitions + states)", t_setup);

    // ── オラクル: frontier2d コールドソルブ。
    let t0 = Instant::now();
    let stats = solve(&mut vi, U64Solver::Frontier2DParUnsafe, 1_000_000);
    let t_cold = t0.elapsed().as_secs_f64() * 1e3;
    assert!(stats.converged, "oracle must converge");
    eprintln!(
        "oracle frontier2d_par_unsafe (parallel): {:.0} ms, iters {}, updates {}",
        t_cold, stats.iters, stats.updates
    );
    let oracle_vals: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
    let oracle_act: Vec<i8> =
        vi.states.iter().map(|s| s.optimal_action.map_or(-1, |a| a as i8)).collect();
    let g1_final: Vec<bool> = vi.states.iter().map(|s| s.final_state).collect();
    let mut flap_xy: std::collections::HashSet<(i32, i32)> = std::collections::HashSet::new();
    {
        // オラクル残差チェック: 収束を宣言した場に全域 Bellman 1 パスを当て、
        // 動くセルがあればオラクル (並列 sparse) の未収束 — schur の上界破れ
        // 判定はそのオラクルに対して無意味になる。パス後は場を復元する。
        let t0 = Instant::now();
        let n = vi.states.len();
        let (mut moved, mut max_d) = (0u64, 0u64);
        for i in 0..n {
            let before = vi.states[i].total_cost;
            vi.value_iteration_at(i);
            let after = vi.states[i].total_cost;
            if after != before && (before < REACH || after < REACH) {
                moved += 1;
                max_d = max_d.max(before.abs_diff(after));
                let s = &vi.states[i];
                flap_xy.insert((s.ix, s.iy));
            }
        }
        eprintln!(
            "oracle residual: {} cells moved (max {} units) in one full sweep [{:.0} ms]",
            moved,
            max_d,
            t0.elapsed().as_secs_f64() * 1e3
        );
        for (i, s) in vi.states.iter_mut().enumerate() {
            s.total_cost = oracle_vals[i];
            s.optimal_action = if oracle_act[i] >= 0 { Some(oracle_act[i] as usize) } else { None };
        }
    }
    let n_reach = oracle_vals.iter().filter(|&&v| v < REACH).count();
    eprintln!("reachable states: {n_reach}");

    // rollout 標本: オラクル場で成功する開始点を集める。
    let reach_idx: Vec<usize> = vi
        .states
        .iter()
        .enumerate()
        .filter(|(_, s)| s.free && !s.final_state && s.total_cost < REACH)
        .map(|(i, _)| i)
        .collect();
    let stride = (reach_idx.len() / args.rollout_samples.max(1)).max(1);
    let mut starts: Vec<(f64, f64, f64, i32, i32)> = Vec::new(); // (x, y, yaw, ix, iy)
    for &i in reach_idx.iter().step_by(stride) {
        let s = &vi.states[i];
        let sx = grid.origin_x + (s.ix as f64 + 0.5) * res;
        let sy = grid.origin_y + (s.iy as f64 + 0.5) * res;
        let yaw = ((s.it as f64 + 0.5) * vi.t_resolution).to_radians();
        if rollout_path_on(&vi, sx, sy, yaw, 100_000, 2).reached_goal() {
            starts.push((sx, sy, yaw, s.ix, s.iy));
        }
    }
    eprintln!("rollout starts (oracle-successful): {}", starts.len());
    // lazy 復元のロボットタイル = 最遠の成功開始点。
    let robot_cell = starts.last().map(|&(_, _, _, ix, iy)| (ix, iy)).unwrap_or((gx, gy));

    {
        // A/B: もう一つの正準ソルバー (sparse) のコールド解と突き合わせる。
        // 両者が食い違えば、wrap 汚染下では忠実モデルの不動点自体が
        // ソルバー依存 (到達不能 free 飛び地の quirk) — schur 固有の破れではない。
        vi.set_goal(goal_wx, goal_wy, goal_t);
        let t0 = Instant::now();
        let st = solve(&mut vi, U64Solver::Frontier2DSparse, 1_000_000);
        let (mut ab_n, mut ab_max) = (0u64, 0u64);
        let mut ab_low = 0u64; // sparse < par_unsafe (「上界破れ」相当の向き)
        for i in 0..oracle_vals.len() {
            if oracle_vals[i] < REACH || vi.states[i].total_cost < REACH {
                let (a, b) = (oracle_vals[i], vi.states[i].total_cost);
                if a != b {
                    ab_n += 1;
                    ab_max = ab_max.max(a.abs_diff(b));
                    if b < a {
                        ab_low += 1;
                    }
                }
            }
        }
        eprintln!(
            "A/B sparse-vs-par_unsafe cold: {} cells differ (max {} units, sparse-lower {}) [{:.0} ms, converged {}]",
            ab_n, ab_max, ab_low, t0.elapsed().as_secs_f64() * 1e3, st.converged
        );
        // 場をオラクルに戻す
        for (i, s) in vi.states.iter_mut().enumerate() {
            s.total_cost = oracle_vals[i];
            s.optimal_action = if oracle_act[i] >= 0 { Some(oracle_act[i] as usize) } else { None };
        }
    }

    // ── GPU コールドソルブ (任意ゴール → 厳密場の実測時間)。地図を常駐させ、
    //    ゴール 1 (オラクルと bit-exact 検証) のあと、地図上の別ゴールを数点解いて
    //    「ゴールごとの時間」を出す (書き戻しは別計上)。
    #[allow(unused_mut)]
    let mut gpu_json = String::from("null");
    if args.gpu_solve {
        #[cfg(feature = "gpu")]
        {
            use vi_lib::solvers::frontier_gpu::GpuSolver;
            vi.set_goal(goal_wx, goal_wy, goal_t);
            match GpuSolver::new(&vi) {
                Ok(mut g) => {
                    eprintln!("GPU resident upload: {:.0} ms", g.t_upload_ms);
                    let st = g.solve(&vi, 1_000_000).expect("gpu solve");
                    let t_dl = g.download(&mut vi).expect("gpu download");
                    let (mut mm, mut mp) = (0u64, 0u64);
                    for (i, s) in vi.states.iter().enumerate() {
                        if oracle_vals[i] < REACH || s.total_cost < REACH {
                            if s.total_cost != oracle_vals[i] {
                                mm += 1;
                            }
                            if s.optimal_action.map_or(-1, |a| a as i8) != oracle_act[i] {
                                mp += 1;
                            }
                        }
                    }
                    eprintln!(
                        "GPU goal1: reset {:.0} ms + solve {:.0} ms ({} rounds, {} updates, converged {}) + download {:.0} ms → mismatch values {} policy {}",
                        st.t_reset_ms, st.t_solve_ms, st.rounds, st.updates, st.converged, t_dl, mm, mp
                    );
                    // 別ゴール: rollout 開始点 (到達可能 free セル) から等間隔に 3 点。
                    let mut extra = Vec::new();
                    let n_extra = 3usize.min(starts.len());
                    for k in 0..n_extra {
                        let (sx, sy, _, _, _) = starts[k * starts.len() / n_extra.max(1)];
                        vi.set_goal(sx, sy, goal_t);
                        let st2 = g.solve(&vi, 1_000_000).expect("gpu solve");
                        eprintln!(
                            "GPU goal@({sx:.1},{sy:.1}): reset {:.0} ms + solve {:.0} ms ({} rounds, {} updates, converged {})",
                            st2.t_reset_ms, st2.t_solve_ms, st2.rounds, st2.updates, st2.converged
                        );
                        extra.push(format!(
                            "{{\"goal\":[{sx:.3},{sy:.3}],\"t_reset_ms\":{:.1},\"t_solve_ms\":{:.1},\"rounds\":{},\"updates\":{}}}",
                            st2.t_reset_ms, st2.t_solve_ms, st2.rounds, st2.updates
                        ));
                    }
                    gpu_json = format!(
                        "{{\"t_upload_ms\":{:.1},\"t_reset_ms\":{:.1},\"t_solve_ms\":{:.1},\"t_download_ms\":{t_dl:.1},\"rounds\":{},\"updates\":{},\"converged\":{},\"mismatch_values\":{mm},\"mismatch_policy\":{mp},\"extra_goals\":[{}]}}",
                        g.t_upload_ms, st.t_reset_ms, st.t_solve_ms, st.rounds, st.updates, st.converged, extra.join(",")
                    );
                }
                Err(e) => eprintln!("GPU cold solve: error {e}"),
            }
            // 場をオラクルに戻す
            vi.set_goal(goal_wx, goal_wy, goal_t);
            for (i, s) in vi.states.iter_mut().enumerate() {
                s.total_cost = oracle_vals[i];
                s.optimal_action = if oracle_act[i] >= 0 { Some(oracle_act[i] as usize) } else { None };
            }
        }
        #[cfg(not(feature = "gpu"))]
        eprintln!("--gpu-solve には --features gpu でのビルドが必要");
    }

    // wrap フラップ圏: 残差で動いたセルの xy を 8 セル膨張した集合。到達不能
    // free 飛び地まわりの wrapping ゴミは全ソルバ共通に揺れる (本家忠実の quirk)
    // ので、上界破れ・mismatch はこの圏の内外に分けて数える。
    let flap_mask: Vec<bool> = {
        let mut m = vec![false; (ow * oh) as usize];
        const R: i32 = 8;
        for &(fx, fy) in &flap_xy {
            for dy in -R..=R {
                for dx in -R..=R {
                    let (x, y) = (fx + dx, fy + dy);
                    if x >= 0 && x < ow && y >= 0 && y < oh {
                        m[(y * ow + x) as usize] = true;
                    }
                }
            }
        }
        m
    };
    eprintln!("flap zone: {} xy cells (dilated x8: {})", flap_xy.len(), flap_mask.iter().filter(|&&b| b).count());

    let mut config_rows: Vec<String> = Vec::new();
    let mut first_imgs = true;

    for &tile in &tiles {
        for &pth in &thetas {
        for &spacing in &spacings {
            let cfg = SchurConfig { tile, portal_thetas: pth, spacing };
            eprintln!("── config tile={tile} thetas={pth} spacing={spacing} ──");
            vi.set_goal(goal_wx, goal_wy, goal_t); // 値リセット

            let art_path = if spacing == 16 {
                args.out.join(format!("schur_t{tile}_h{pth}.bin"))
            } else {
                args.out.join(format!("schur_t{tile}_h{pth}_s{spacing}.bin"))
            };
            let t0 = Instant::now();
            let art = if args.reuse && art_path.exists() {
                let a = vi_lib::solvers::schur::SchurArtifact::load(&art_path).expect("load artifact");
                eprintln!("(reuse: loaded artifact in {:.0} ms)", t0.elapsed().as_secs_f64() * 1e3);
                a
            } else if args.gpu {
                #[cfg(feature = "gpu")]
                {
                    vi_lib::solvers::schur_gpu::build_gpu(&vi, &cfg).expect("GPU build")
                }
                #[cfg(not(feature = "gpu"))]
                panic!("--gpu には --features gpu でのビルドが必要");
            } else {
                build(&vi, &cfg)
            };
            let t_build = t0.elapsed().as_secs_f64() * 1e3;
            art.save(&art_path).expect("save artifact");
            let art_bytes = std::fs::metadata(&art_path).map(|m| m.len()).unwrap_or(0);
            eprintln!(
                "build: {:.0} ms, portals {}, artifact {:.2} MB (halo {})",
                t_build,
                art.portals.len(),
                art_bytes as f64 / 1e6,
                art.halo
            );

            // ポータル Dijkstra (ゴールタイルソルブ込み)。
            let t0 = Instant::now();
            let d = portal_costs(&vi, &art);
            let t_portal = t0.elapsed().as_secs_f64() * 1e3;
            {
                use vi_lib::params::MAX_COST as MC;
                let fin_d = d.iter().filter(|&&v| v < MC).count();
                let (mut fin_w, mut tot_w) = (0usize, 0usize);
                for te in &art.tiles {
                    tot_w += te.w.len();
                    fin_w += te.w.iter().filter(|&&v| v < MC).count();
                }
                eprintln!(
                    "debug: finite D {}/{} , finite W {}/{} ({:.1}%)",
                    fin_d,
                    d.len(),
                    fin_w,
                    tot_w,
                    fin_w as f64 / tot_w.max(1) as f64 * 100.0
                );
            }

            // 診断: ポータル値 D 自体のギャップ (対オラクル)。D が緩ければ原因は
            // ポータルグラフ (W の打ち切り / 間引き) 側、D が正確なら復元側。
            {
                let (mut n, mut sum, mut viol, mut unreached) = (0u64, 0f64, 0u64, 0u64);
                let mut by_dist: Vec<(f64, f64)> = Vec::new(); // (オラクル値 [s], gap)
                for (pid, p) in art.portals.iter().enumerate() {
                    let i = vi.to_index(p.ix, p.iy, p.it) as usize;
                    let a = oracle_vals[i];
                    if a >= REACH || a == 0 {
                        continue;
                    }
                    n += 1;
                    let b = d[pid];
                    if b >= REACH {
                        unreached += 1;
                        continue;
                    }
                    if b < a {
                        viol += 1;
                    } else {
                        let g = (b - a) as f64 / a as f64;
                        sum += g;
                        by_dist.push((a as f64 / vi_lib::params::PROB_BASE as f64, g));
                    }
                }
                by_dist.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
                let q = |f: f64| by_dist.get(((by_dist.len() as f64 * f) as usize).min(by_dist.len().saturating_sub(1))).copied().unwrap_or((0.0, 0.0));
                eprintln!(
                    "portal D gap: mean {:.1}% over {} portals (viol {}, unreached {}); gap at oracle-value quantiles: 10% ({:.0}s: {:.0}%) 50% ({:.0}s: {:.0}%) 90% ({:.0}s: {:.0}%)",
                    sum / n.max(1) as f64 * 100.0, n, viol, unreached,
                    q(0.1).0, q(0.1).1 * 100.0, q(0.5).0, q(0.5).1 * 100.0, q(0.9).0, q(0.9).1 * 100.0
                );
                // W の打ち切り診断: 有限 W の分布 (最小/中央値) と、ゴールタイルのポータル
                // (Dijkstra の種) のギャップ。
                let mut seeds = Vec::new();
                for (pid, p) in art.portals.iter().enumerate() {
                    let i = vi.to_index(p.ix, p.iy, p.it) as usize;
                    let a = oracle_vals[i];
                    if a < REACH && a > 0 && d[pid] < REACH {
                        seeds.push((a, d[pid]));
                    }
                }
                seeds.sort();
                let k = 20.min(seeds.len());
                let near: Vec<String> = seeds[..k].iter().map(|&(a, b)| format!("{:.1}/{:.1}", a as f64 / 262144.0, b as f64 / 262144.0)).collect();
                eprintln!("nearest-to-goal portals (oracle/D [s]): {}", near.join(" "));
            }

            // lazy: ロボットタイル 1 枚だけ復元。
            let finals: Vec<(i32, i32, i32)> = vi
                .states
                .iter()
                .filter(|s| s.final_state)
                .map(|s| (s.ix, s.iy, s.it))
                .collect();
            let mut tv = TileScratch::new(&vi, &art);
            let (rtx, rty) = (robot_cell.0 / tile.max(1), robot_cell.1 / tile.max(1));
            let t0 = Instant::now();
            materialize_one(&mut vi, &art, &d, &finals, &mut tv, rty * art.tnx + rtx);
            let t_lazy = t0.elapsed().as_secs_f64() * 1e3;

            // 全場 V̂。
            vi.set_goal(goal_wx, goal_wy, goal_t);
            let t0 = Instant::now();
            upper_bound_field(&mut vi, &art);
            let t_ub = t0.elapsed().as_secs_f64() * 1e3;
            eprintln!("portal dijkstra: {t_portal:.1} ms, lazy tile: {t_lazy:.1} ms, full V̂: {t_ub:.0} ms");

            // ギャップ統計。
            let (mut n_cmp, mut n_exact, mut viol) = (0u64, 0u64, 0u64);
            let (mut gap_sum, mut gap_max) = (0f64, 0f64);
            let mut n_unreached = 0u64;
            let mut viol_max = 0u64; // 上界破れの最大量 [LSB 単位] — 1-2 なら打ち切り区間
            let mut viol_out = 0u64; // wrap フラップ圏の外での破れ (期待 0)
            for i in 0..oracle_vals.len() {
                let a = oracle_vals[i];
                if a >= REACH || a == 0 {
                    continue;
                }
                let b = vi.states[i].total_cost;
                n_cmp += 1;
                if b < a {
                    viol += 1;
                    viol_max = viol_max.max(a - b);
                    let st = &vi.states[i];
                    if !flap_mask[(st.iy * ow + st.ix) as usize] {
                        viol_out += 1;
                    }
                } else if b == a {
                    n_exact += 1;
                }
                if b >= REACH {
                    n_unreached += 1;
                    continue;
                }
                let g = (b as f64 - a as f64) / a as f64;
                gap_sum += g.max(0.0);
                gap_max = gap_max.max(g);
            }
            let gap_mean = if n_cmp > 0 { gap_sum / n_cmp as f64 } else { 0.0 };
            eprintln!(
                "V̂ gap: mean {:.3}%, max {:.2}%, exact {:.1}%, violations {} (max depth {} units, outside flap zone {}), V̂-unreached {}",
                gap_mean * 100.0,
                gap_max * 100.0,
                n_exact as f64 / n_cmp.max(1) as f64 * 100.0,
                viol,
                viol_max,
                viol_out,
                n_unreached
            );

            // V̂ 上の rollout 成功率。
            let mut vhat_ok = 0usize;
            for &(sx, sy, yaw, _, _) in &starts {
                if rollout_path_on(&vi, sx, sy, yaw, 100_000, 2).reached_goal() {
                    vhat_ok += 1;
                }
            }
            eprintln!("rollout on V̂: {vhat_ok}/{} succeeded", starts.len());
            let vhat_vals: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
            let vhat_act: Vec<i8> =
                vi.states.iter().map(|s| s.optimal_action.map_or(-1, |a| a as i8)).collect();

            // 可視化 (先頭設定のみ)。
            if first_imgs {
                first_imgs = false;
                // 地図 + タイル格子 + ポータル。
                let mut img = Img::new(ow, oh, [255, 255, 255]);
                for iy in 0..oh {
                    for ix in 0..ow {
                        if grid.data[(iy * ow + ix) as usize] != 0 {
                            img.set(ix, iy, [40, 40, 46]);
                        }
                    }
                }
                for k in 1..art.tnx {
                    for iy in 0..oh {
                        let ix = k * tile;
                        if grid.data[(iy * ow + ix) as usize] == 0 {
                            img.set(ix, iy, [210, 220, 235]);
                        }
                    }
                }
                for k in 1..art.tny {
                    for ix in 0..ow {
                        let iy = k * tile;
                        if grid.data[(iy * ow + ix) as usize] == 0 {
                            img.set(ix, iy, [210, 220, 235]);
                        }
                    }
                }
                for p in &art.portals {
                    for dy in -1..=1 {
                        for dx in -1..=1 {
                            img.set(p.ix + dx, p.iy + dy, [225, 60, 50]);
                        }
                    }
                }
                img.set(gx, gy, [30, 160, 70]);
                img.save(&args.out.join("map_portals.ppm")).unwrap();

                value_slice_img(&vi, |i| vi.states[i].total_cost)
                    .save(&args.out.join("vhat_theta0.ppm"))
                    .unwrap();
                value_slice_img(&vi, |i| oracle_vals[i])
                    .save(&args.out.join("oracle_theta0.ppm"))
                    .unwrap();
                // 相対ギャップ (白 = 0, 赤 = p95)。固定クリップだと平均ギャップが
                // 大きい設定で全面赤になり情報が消えるので、自身の p95 で正規化。
                let mut gaps: Vec<f64> = Vec::new();
                for iy in 0..oh {
                    for ix in 0..ow {
                        let idx = (ix * nt + iy * nt * ow) as usize;
                        let a = oracle_vals[idx];
                        if vi.states[idx].free && a > 0 && a < REACH {
                            let b = vi.states[idx].total_cost;
                            gaps.push((b as f64 - a as f64) / a as f64);
                        }
                    }
                }
                gaps.sort_by(f64::total_cmp);
                let g95 = gaps
                    .get(gaps.len().saturating_mul(95) / 100)
                    .copied()
                    .unwrap_or(0.1)
                    .max(1e-9);
                let mut gimg = Img::new(ow, oh, [255, 255, 255]);
                for iy in 0..oh {
                    for ix in 0..ow {
                        let idx = (ix * nt + iy * nt * ow) as usize;
                        let a = oracle_vals[idx];
                        let c = if !vi.states[idx].free {
                            [15, 15, 18]
                        } else if a == 0 || a >= REACH {
                            [235, 235, 238]
                        } else {
                            let b = vi.states[idx].total_cost;
                            let g = ((b as f64 - a as f64) / a as f64 / g95).clamp(0.0, 1.0);
                            [255, (230.0 * (1.0 - g)) as u8, (225.0 * (1.0 - g)) as u8]
                        };
                        gimg.set(ix, iy, c);
                    }
                }
                gimg.save(&args.out.join("gap_theta0.ppm")).unwrap();
                eprintln!("gap image p95 = {:.1}%", g95 * 100.0);
            }

            // exactify: V̂ 起点で frontier2d を回して固定点まで → mismatch。
            let t0 = Instant::now();
            let st = solve(&mut vi, U64Solver::Frontier2DParUnsafe, 1_000_000);
            let t_exact = t0.elapsed().as_secs_f64() * 1e3;
            // flap ゾーン (wrap 振動域) が広い地図では点ごとの固定点が無く、
            // frontier が空にならない (tsudanuma scale1 で実測 375k セル)。
            // 落とすと build 数時間分の計測が消えるので警告して mismatch は出す。
            if !st.converged {
                eprintln!("exactify: NOT converged ({} iters) — flap 振動とみなし残差のまま集計", st.iters);
            }
            let mut mm_v = 0u64;
            let mut mm_p = 0u64;
            let mut mm_max = 0u64;
            let mut mm_out = 0u64;
            for i in 0..oracle_vals.len() {
                if oracle_vals[i] < REACH {
                    if vi.states[i].total_cost != oracle_vals[i] {
                        mm_v += 1;
                        mm_max = mm_max.max(vi.states[i].total_cost.abs_diff(oracle_vals[i]));
                        let st = &vi.states[i];
                        if !flap_mask[(st.iy * ow + st.ix) as usize] {
                            mm_out += 1;
                        }
                    }
                    if vi.states[i].optimal_action.map_or(-1, |a| a as i8) != oracle_act[i] {
                        mm_p += 1;
                    }
                }
            }
            eprintln!(
                "exactify: {:.0} ms ({} iters) → mismatch values {} (max {} units, outside flap zone {}), policy {}",
                t_exact, st.iters, mm_v, mm_max, mm_out, mm_p
            );

            // ordered exactify: 同じ V̂ から値昇順 GS で固定点へ。
            let (t_ord, ord_passes, ord_updates, mm_ord) = if args.no_ordered {
                (-1.0, 0u64, 0u64, 0u64)
            } else {
                for (i, s) in vi.states.iter_mut().enumerate() {
                    s.total_cost = vhat_vals[i];
                    s.optimal_action = (vhat_act[i] >= 0).then_some(vhat_act[i] as usize);
                }
                let t0 = Instant::now();
                let ord = ordered_sweep(&mut vi, 100_000);
                let t_ord = t0.elapsed().as_secs_f64() * 1e3;
                let (mm_ord, mm_ord_out) = mismatch_vs(&vi, &oracle_vals, &flap_mask, ow);
                eprintln!(
                    "ordered exactify: {:.0} ms ({} passes, {} updates, converged {}) → mismatch values {} (outside flap zone {})",
                    t_ord, ord.passes, ord.updates, ord.converged, mm_ord, mm_ord_out
                );
                (t_ord, ord.passes as u64, ord.updates, mm_ord)
            };

            // 対照: 順序なしコールド (全 MAX_COST → 1 バケット = 添字順 GS)。
            let (t_cold_ord, cold_ord_passes) = if args.cold_ordered {
                vi.set_goal(goal_wx, goal_wy, goal_t);
                let t0 = Instant::now();
                let o = ordered_sweep(&mut vi, 100_000);
                let t = t0.elapsed().as_secs_f64() * 1e3;
                let (mm, _) = mismatch_vs(&vi, &oracle_vals, &flap_mask, ow);
                eprintln!("cold ordered (no order): {:.0} ms ({} passes, converged {}) → mismatch {}", t, o.passes, o.converged, mm);
                (t, o.passes)
            } else {
                (-1.0, 0)
            };

            // ── 2 段ポータルグラフ: 粗 (この設定) 全域 + 密 (回廊限定)。
            let mut two_level_json = String::from("null");
            if args.two_level {
                let fcfg = SchurConfig {
                    tile,
                    portal_thetas: args.fine_thetas,
                    spacing: args.fine_spacing,
                };
                let fart_path = if args.fine_spacing == 16 {
                    args.out.join(format!("schur_t{tile}_h{}.bin", args.fine_thetas))
                } else {
                    args.out
                        .join(format!("schur_t{tile}_h{}_s{}.bin", args.fine_thetas, args.fine_spacing))
                };
                let t0 = Instant::now();
                let (art_f, t_fbuild) = if args.reuse && fart_path.exists() {
                    let a = SchurArtifact::load(&fart_path).expect("load fine artifact");
                    eprintln!("(fine reuse: loaded in {:.0} ms)", t0.elapsed().as_secs_f64() * 1e3);
                    (a, -1.0)
                } else {
                    let a = if args.gpu {
                        #[cfg(feature = "gpu")]
                        {
                            vi_lib::solvers::schur_gpu::build_gpu(&vi, &fcfg).expect("GPU fine build")
                        }
                        #[cfg(not(feature = "gpu"))]
                        panic!("--gpu には --features gpu でのビルドが必要");
                    } else {
                        build(&vi, &fcfg)
                    };
                    let t = t0.elapsed().as_secs_f64() * 1e3;
                    a.save(&fart_path).expect("save fine artifact");
                    (a, t)
                };
                let fart_bytes = std::fs::metadata(&fart_path).map(|m| m.len()).unwrap_or(0);
                assert_eq!((art.tile, art.halo), (art_f.tile, art_f.halo), "粗密の幾何不一致");
                eprintln!(
                    "── two-level: fine thetas {} spacing {} → portals {}, {:.1} MB (build {:.0} ms) ──",
                    args.fine_thetas,
                    args.fine_spacing,
                    art_f.portals.len(),
                    fart_bytes as f64 / 1e6,
                    t_fbuild
                );

                // 値リセット (直前の実験の場を消す) — Dijkstra はゴール final だけ読む。
                vi.set_goal(goal_wx, goal_wy, goal_t);
                let t0 = Instant::now();
                let dij_c = portal_costs_masked(&vi, &art, None);
                let t_dijk_c = t0.elapsed().as_secs_f64() * 1e3;
                let t0 = Instant::now();
                let dij_f_full = portal_costs_masked(&vi, &art_f, None);
                let t_dijk_f_full = t0.elapsed().as_secs_f64() * 1e3;
                eprintln!(
                    "dijkstra (single-thread): coarse {t_dijk_c:.0} ms (seed {:.0} + graph {:.0}), fine full {t_dijk_f_full:.0} ms (seed {:.0} + graph {:.0})",
                    dij_c.seed_ms, dij_c.graph_ms, dij_f_full.seed_ms, dij_f_full.graph_ms
                );

                let step = (starts.len() / args.tl_starts.max(1)).max(1);
                let sample: Vec<(f64, f64, f64, i32, i32)> =
                    starts.iter().step_by(step).take(args.tl_starts).cloned().collect();
                let mut start_rows: Vec<String> = Vec::new();
                let (mut sg_c, mut sg_2, mut sg_f) = (0f64, 0f64, 0f64);
                let (mut st_dijk, mut st_mat, mut s_cor) = (0f64, 0f64, 0usize);
                let mut n_rows = 0usize;
                for &(sx, sy, yaw, _, _) in &sample {
                    let (six, siy, sit) = pose_to_cell(&vi, sx, sy, yaw);
                    let sidx = vi.to_index(six, siy, sit) as usize;
                    let a = oracle_vals[sidx];
                    if a == 0 || a >= REACH {
                        continue;
                    }
                    let rt = (siy / tile) * art.tnx + six / tile;
                    let t0 = Instant::now();
                    let mask_cor = corridor_tiles(&art, &dij_c, &[rt], args.corridor_dilate);
                    let t_cor = t0.elapsed().as_secs_f64() * 1e3;
                    let t0 = Instant::now();
                    let dij_f = portal_costs_masked(&vi, &art_f, Some(&mask_cor));
                    let t_dijk = t0.elapsed().as_secs_f64() * 1e3;
                    let n_cor = mask_cor.iter().filter(|&&b| b).count();
                    // 3 通りの復元: 粗のみ / 粗+密回廊 / 密全域。gap は開始状態で測る。
                    let pin_c: Vec<(&SchurArtifact, &[u64])> = vec![(&art, &dij_c.d[..])];
                    let pin_2: Vec<(&SchurArtifact, &[u64])> =
                        vec![(&art, &dij_c.d[..]), (&art_f, &dij_f.d[..])];
                    let pin_f: Vec<(&SchurArtifact, &[u64])> = vec![(&art_f, &dij_f_full.d[..])];
                    let mut gaps = [f64::INFINITY; 3];
                    let mut t_mat = 0f64;
                    for (k, pins) in [&pin_c, &pin_2, &pin_f].into_iter().enumerate() {
                        vi.set_goal(goal_wx, goal_wy, goal_t);
                        let t0 = Instant::now();
                        materialize_one_multi(&mut vi, pins, &finals, &mut tv, rt);
                        if k == 1 {
                            t_mat = t0.elapsed().as_secs_f64() * 1e3;
                        }
                        let b = vi.states[sidx].total_cost;
                        if b < REACH {
                            gaps[k] = (b as f64 - a as f64) / a as f64;
                        }
                    }
                    let d_m = ((sx - goal_wx).powi(2) + (sy - goal_wy).powi(2)).sqrt();
                    let oracle_s = a as f64 / vi_lib::params::PROB_BASE as f64;
                    eprintln!(
                        "TL start ({six},{siy},{sit}) d {d_m:.1} m V* {oracle_s:.1} s: gap coarse {:.1}% / two {:.1}% / fine-full {:.1}%; corridor {n_cor}/{} tiles, cor {t_cor:.1} ms + dijkstra {t_dijk:.1} ms (graph {:.1}) + tile {t_mat:.1} ms",
                        gaps[0] * 100.0,
                        gaps[1] * 100.0,
                        gaps[2] * 100.0,
                        mask_cor.len(),
                        dij_f.graph_ms
                    );
                    start_rows.push(format!(
                        "{{\"cell\":[{six},{siy},{sit}],\"d_m\":{d_m:.2},\"oracle_s\":{oracle_s:.2},\
                         \"gap_coarse\":{:.6},\"gap_two\":{:.6},\"gap_fine_full\":{:.6},\
                         \"corridor_tiles\":{n_cor},\"t_corridor_ms\":{t_cor:.2},\
                         \"t_dijk_corridor_ms\":{t_dijk:.2},\"t_dijk_corridor_graph_ms\":{:.2},\"t_tile_ms\":{t_mat:.2}}}",
                        gaps[0], gaps[1], gaps[2], dij_f.graph_ms
                    ));
                    sg_c += gaps[0];
                    sg_2 += gaps[1];
                    sg_f += gaps[2];
                    st_dijk += t_dijk;
                    st_mat += t_mat;
                    s_cor += n_cor;
                    n_rows += 1;
                }
                let m = n_rows.max(1) as f64;
                eprintln!(
                    "TL mean over {n_rows} starts: gap coarse {:.2}% / two {:.2}% / fine-full {:.2}%; corridor {:.0} tiles, dijkstra {:.1} ms, tile {:.1} ms (vs fine full dijkstra {t_dijk_f_full:.0} ms)",
                    sg_c / m * 100.0,
                    sg_2 / m * 100.0,
                    sg_f / m * 100.0,
                    s_cor as f64 / m,
                    st_dijk / m,
                    st_mat / m
                );
                two_level_json = format!(
                    "{{\"fine_thetas\":{},\"fine_spacing\":{},\"fine_portals\":{},\"fine_artifact_bytes\":{fart_bytes},\
                     \"t_fine_build_ms\":{t_fbuild:.1},\"corridor_dilate\":{},\
                     \"t_dijk_coarse_ms\":{t_dijk_c:.2},\"t_dijk_coarse_graph_ms\":{:.2},\
                     \"t_dijk_fine_full_ms\":{t_dijk_f_full:.2},\"t_dijk_fine_full_graph_ms\":{:.2},\
                     \"mean_gap_coarse\":{:.6},\"mean_gap_two\":{:.6},\"mean_gap_fine_full\":{:.6},\
                     \"mean_t_dijk_corridor_ms\":{:.2},\"mean_t_tile_ms\":{:.2},\"mean_corridor_tiles\":{:.1},\
                     \"starts\":[{}]}}",
                    args.fine_thetas,
                    args.fine_spacing,
                    art_f.portals.len(),
                    args.corridor_dilate,
                    dij_c.graph_ms,
                    dij_f_full.graph_ms,
                    sg_c / m,
                    sg_2 / m,
                    sg_f / m,
                    st_dijk / m,
                    st_mat / m,
                    s_cor as f64 / m,
                    start_rows.join(",")
                );
            }

            // ── B: 近傍ゴール再利用。
            let mut goal2_json = String::from("null");
            if let (Some(g2x), Some(g2y)) = (args.goal2_x, args.goal2_y) {
                let r2x = (((g2x - grid.origin_x) / res).floor() as i32).clamp(0, ow - 1);
                let r2y = (((g2y - grid.origin_y) / res).floor() as i32).clamp(0, oh - 1);
                let (g2cx, g2cy) = snap_to_free(&grid.data, ow, oh, r2x, r2y, ow.max(oh)).expect("goal2 free");
                let g2wx = grid.origin_x + (g2cx as f64 + 0.5) * res;
                let g2wy = grid.origin_y + (g2cy as f64 + 0.5) * res;
                let g2t = args.goal2_theta_deg as i32;
                let dist = ((g2wx - goal_wx).powi(2) + (g2wy - goal_wy).powi(2)).sqrt();
                eprintln!("── goal2 ({g2wx:.2}, {g2wy:.2}) th {g2t}, {dist:.2} m from goal1 ──");

                // コールドオラクル (g2)。
                vi.set_goal(g2wx, g2wy, g2t);
                let t0 = Instant::now();
                let st2 = solve(&mut vi, U64Solver::Frontier2DParUnsafe, 1_000_000);
                let t_cold2 = t0.elapsed().as_secs_f64() * 1e3;
                assert!(st2.converged);
                let oracle2: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
                eprintln!("goal2 cold oracle: {t_cold2:.0} ms ({} iters)", st2.iters);

                // c = max_{f ∈ finals(g1)} V_{g2}(f) — ゴール 1 に着いてからゴール 2 へ行く最大コスト。
                let c = g1_final
                    .iter()
                    .enumerate()
                    .filter(|(_, &f)| f)
                    .map(|(i, _)| oracle2[i])
                    .max()
                    .unwrap_or(vi_lib::params::MAX_COST);
                eprintln!("c = max V_g2 over goal1 finals = {:.2} s", c as f64 / vi_lib::params::PROB_BASE as f64);

                // 上界 B = V_g1 + c (g2 の final は 0)。
                vi.set_goal(g2wx, g2wy, g2t);
                for (i, s) in vi.states.iter_mut().enumerate() {
                    if !s.final_state && oracle_vals[i] < REACH && c < REACH {
                        s.total_cost = oracle_vals[i] + c;
                        s.optimal_action = (oracle_act[i] >= 0).then_some(oracle_act[i] as usize);
                    }
                }
                let (gap_b, viol_b) = gap_vs(&vi, &oracle2);
                let vb: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
                let vba: Vec<i8> = vi.states.iter().map(|s| s.optimal_action.map_or(-1, |a| a as i8)).collect();
                let t0 = Instant::now();
                let ob = ordered_sweep(&mut vi, 100_000);
                let t_b = t0.elapsed().as_secs_f64() * 1e3;
                let (mm_b, mm_b_out) = mismatch_vs(&vi, &oracle2, &flap_mask, ow);
                eprintln!(
                    "B (V_g1 + c) gap {:.2}% viol {} → ordered: {:.0} ms ({} passes, converged {}) mismatch {} (outside flap {})",
                    gap_b * 100.0, viol_b, t_b, ob.passes, ob.converged, mm_b, mm_b_out
                );
                for (i, s) in vi.states.iter_mut().enumerate() {
                    s.total_cost = vb[i];
                    s.optimal_action = (vba[i] >= 0).then_some(vba[i] as usize);
                }
                let t0 = Instant::now();
                let bf = solve(&mut vi, U64Solver::Frontier2DParUnsafe, 1_000_000);
                let t_bf = t0.elapsed().as_secs_f64() * 1e3;
                let (mm_bf, _) = mismatch_vs(&vi, &oracle2, &flap_mask, ow);
                eprintln!("B → frontier exactify: {t_bf:.0} ms ({} iters, {} updates) mismatch {}", bf.iters, bf.updates, mm_bf);

                // Schur V̂ (g2) → ordered / frontier。
                vi.set_goal(g2wx, g2wy, g2t);
                let t0 = Instant::now();
                upper_bound_field(&mut vi, &art);
                let t_ub2 = t0.elapsed().as_secs_f64() * 1e3;
                let (gap_s2, viol_s2) = gap_vs(&vi, &oracle2);
                let vh2: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
                let vh2a: Vec<i8> = vi.states.iter().map(|s| s.optimal_action.map_or(-1, |a| a as i8)).collect();
                let t0 = Instant::now();
                let os2 = ordered_sweep(&mut vi, 100_000);
                let t_s2 = t0.elapsed().as_secs_f64() * 1e3;
                let (mm_s2, _) = mismatch_vs(&vi, &oracle2, &flap_mask, ow);
                for (i, s) in vi.states.iter_mut().enumerate() {
                    s.total_cost = vh2[i];
                    s.optimal_action = (vh2a[i] >= 0).then_some(vh2a[i] as usize);
                }
                let t0 = Instant::now();
                let sf2 = solve(&mut vi, U64Solver::Frontier2DParUnsafe, 1_000_000);
                let t_sf2 = t0.elapsed().as_secs_f64() * 1e3;
                let (mm_sf2, _) = mismatch_vs(&vi, &oracle2, &flap_mask, ow);
                eprintln!(
                    "Schur V̂(g2): {t_ub2:.0} ms gap {:.2}% viol {} → ordered {:.0} ms ({} passes) mismatch {} / frontier {:.0} ms ({} iters) mismatch {}",
                    gap_s2 * 100.0, viol_s2, t_s2, os2.passes, mm_s2, t_sf2, sf2.iters, mm_sf2
                );

                // min(B, Schur): 二つの上界の各点 min も上界。
                vi.set_goal(g2wx, g2wy, g2t);
                for (i, s) in vi.states.iter_mut().enumerate() {
                    if s.final_state {
                        continue;
                    }
                    let b = if oracle_vals[i] < REACH && c < REACH { oracle_vals[i] + c } else { vi_lib::params::MAX_COST };
                    if b <= vh2[i] {
                        s.total_cost = b;
                        s.optimal_action = (oracle_act[i] >= 0).then_some(oracle_act[i] as usize);
                    } else {
                        s.total_cost = vh2[i];
                        s.optimal_action = (vh2a[i] >= 0).then_some(vh2a[i] as usize);
                    }
                }
                let (gap_m, _) = gap_vs(&vi, &oracle2);
                let t0 = Instant::now();
                let om = ordered_sweep(&mut vi, 100_000);
                let t_m = t0.elapsed().as_secs_f64() * 1e3;
                let (mm_m, _) = mismatch_vs(&vi, &oracle2, &flap_mask, ow);
                eprintln!(
                    "min(B, Schur) gap {:.2}% → ordered {:.0} ms ({} passes) mismatch {}",
                    gap_m * 100.0, t_m, om.passes, mm_m
                );

                goal2_json = format!(
                    "{{\"goal2\":[{g2wx:.3},{g2wy:.3},{g2t}],\"dist_m\":{dist:.3},\"c_s\":{:.3},\
                     \"t_cold_ms\":{t_cold2:.1},\"cold_iters\":{},\
                     \"b_gap\":{gap_b:.6},\"b_viol\":{viol_b},\"b_t_ordered_ms\":{t_b:.1},\"b_passes\":{},\"b_mismatch\":{mm_b},\
                     \"b_t_frontier_ms\":{t_bf:.1},\"b_frontier_iters\":{},\"b_frontier_mismatch\":{mm_bf},\
                     \"schur_t_ub_ms\":{t_ub2:.1},\"schur_gap\":{gap_s2:.6},\"schur_t_ordered_ms\":{t_s2:.1},\"schur_passes\":{},\"schur_mismatch\":{mm_s2},\
                     \"schur_t_frontier_ms\":{t_sf2:.1},\"schur_frontier_iters\":{},\"schur_frontier_mismatch\":{mm_sf2},\
                     \"min_gap\":{gap_m:.6},\"min_t_ordered_ms\":{t_m:.1},\"min_passes\":{},\"min_mismatch\":{mm_m}}}",
                    c as f64 / vi_lib::params::PROB_BASE as f64,
                    st2.iters, ob.passes, bf.iters, os2.passes, sf2.iters, om.passes
                );
            }

            config_rows.push(format!(
                "{{\"tile\":{tile},\"thetas\":{pth},\"spacing\":{spacing},\"halo\":{},\"portals\":{},\"artifact_bytes\":{art_bytes},\
                 \"t_build_ms\":{t_build:.1},\"t_portal_ms\":{t_portal:.2},\"t_lazy_ms\":{t_lazy:.2},\
                 \"t_ub_ms\":{t_ub:.1},\"t_exactify_ms\":{t_exact:.1},\
                 \"gap_mean\":{gap_mean:.6},\"gap_max\":{gap_max:.6},\
                 \"frac_exact\":{:.6},\"violations\":{viol},\"vhat_unreached\":{n_unreached},\
                 \"rollout_ok\":{vhat_ok},\"rollout_total\":{},\
                 \"mismatch_values\":{mm_v},\"mismatch_policy\":{mm_p},\
                 \"t_ordered_ms\":{t_ord:.1},\"ordered_passes\":{},\"ordered_updates\":{},\"ordered_mismatch\":{mm_ord},\
                 \"t_cold_ordered_ms\":{t_cold_ord:.1},\"cold_ordered_passes\":{cold_ord_passes},\
                 \"two_level\":{two_level_json},\"goal2\":{goal2_json}}}",
                art.halo,
                art.portals.len(),
                n_exact as f64 / n_cmp.max(1) as f64,
                starts.len(),
                ord_passes,
                ord_updates,
            ));
        }
        }
    }

    let report = format!(
        "{{\n\"map\":\"{}\",\"scale\":{},\"grid\":[{ow},{oh},{nt}],\"resolution_m\":{res},\
         \"goal\":[{goal_wx:.3},{goal_wy:.3},{goal_t}],\"reach_states\":{n_reach},\
         \"t_setup_ms\":{t_setup:.1},\"t_cold_ms\":{t_cold:.1},\"gpu\":{gpu_json},\
         \"oracle_iters\":{},\"rollout_starts\":{},\n\"configs\":[\n{}\n]\n}}\n",
        map_path.display().to_string().replace('\\', "/"),
        args.scale,
        stats.iters,
        starts.len(),
        config_rows.join(",\n"),
    );
    std::fs::write(args.out.join("report.json"), &report).expect("write report");
    eprintln!("report: {}", args.out.join("report.json").display());
    ExitCode::SUCCESS
}
