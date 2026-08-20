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
use vi_lib::planner::rollout_path_on;
use vi_lib::solvers::schur::{
    build, materialize_one, portal_costs, upper_bound_field, SchurConfig, TileScratch,
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

    /// ゴール世界座標 (省略時は地図中心を free へスナップ)。
    #[arg(long)]
    goal_x: Option<f64>,
    #[arg(long)]
    goal_y: Option<f64>,
    #[arg(long, default_value_t = 90.0)]
    goal_theta_deg: f64,

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

fn main() -> ExitCode {
    let args = Args::parse();
    let map_path = args.map.clone().unwrap_or_else(default_map_path);
    let tiles = parse_list(&args.tiles);
    let thetas = parse_list(&args.thetas);
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
            let cfg = SchurConfig { tile, portal_thetas: pth };
            eprintln!("── config tile={tile} thetas={pth} ──");
            vi.set_goal(goal_wx, goal_wy, goal_t); // 値リセット

            let art_path = args.out.join(format!("schur_t{tile}_h{pth}.bin"));
            let t0 = Instant::now();
            let art = if args.reuse && art_path.exists() {
                let a = vi_lib::solvers::schur::SchurArtifact::load(&art_path).expect("load artifact");
                eprintln!("(reuse: loaded artifact in {:.0} ms)", t0.elapsed().as_secs_f64() * 1e3);
                a
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
            assert!(st.converged);
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

            config_rows.push(format!(
                "{{\"tile\":{tile},\"thetas\":{pth},\"halo\":{},\"portals\":{},\"artifact_bytes\":{art_bytes},\
                 \"t_build_ms\":{t_build:.1},\"t_portal_ms\":{t_portal:.2},\"t_lazy_ms\":{t_lazy:.2},\
                 \"t_ub_ms\":{t_ub:.1},\"t_exactify_ms\":{t_exact:.1},\
                 \"gap_mean\":{gap_mean:.6},\"gap_max\":{gap_max:.6},\
                 \"frac_exact\":{:.6},\"violations\":{viol},\"vhat_unreached\":{n_unreached},\
                 \"rollout_ok\":{vhat_ok},\"rollout_total\":{},\
                 \"mismatch_values\":{mm_v},\"mismatch_policy\":{mm_p}}}",
                art.halo,
                art.portals.len(),
                n_exact as f64 / n_cmp.max(1) as f64,
                starts.len(),
            ));
        }
    }

    let report = format!(
        "{{\n\"map\":\"{}\",\"scale\":{},\"grid\":[{ow},{oh},{nt}],\"resolution_m\":{res},\
         \"goal\":[{goal_wx:.3},{goal_wy:.3},{goal_t}],\"reach_states\":{n_reach},\
         \"t_setup_ms\":{t_setup:.1},\"t_cold_ms\":{t_cold:.1},\
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
