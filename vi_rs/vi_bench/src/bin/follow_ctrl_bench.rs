//! `follow_ctrl_bench` — solve 済みの場に対する follow 制御の比較ベンチ。
//!
//! 実地図 (PGM/YAML) を bench_map と同じ規約でロードして u64 ソルバで解き、同じ
//! 収束場から 2 種類のコントローラでシミュレーション走行して品質を比較する:
//!
//! - `greedy` — 現行 `vi_planner` の `decide` 相当 (本家 `ViNode::decision` 準拠)。
//!   セルに丸めて離散 6 行動の方策を引き、`linear.x = delta_fw [m/s]`、
//!   `angular.z = delta_rot [deg/s]` を 1 tick 保持する。
//! - `dwa` — `vi_reference::ctrl` の連続行動 (V̂ 三線形補間 + (v, ω) 候補格子の
//!   軌道サンプリング、終端 V̂ 最小)。棄却全滅時は greedy へフォールバック。
//!
//! ロボットは両者とも同じユニサイクルモデル (定速弧、`--tick-s` 周期で再決定) で
//! 積分する。指標: 到達率 / 所要 tick / 経路長 / 総回転量 / コマンド変動 (Σ|Δω|) /
//! 最小クリアランス / decide 1 回あたりの計算時間。
//!
//! 例 (津田沼 scale 6、既定):
//! ```text
//! cargo run --release -p vi_bench --bin follow_ctrl_bench -- --starts 6
//! ```

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use clap::Parser;

use vi_bench::params::{canonical_actions, N_THETA};
use vi_bench::pgm::{self, Occupancy, PgmMap};
use vi_reference::ctrl::{dwa_decide, unicycle_step, CostView, DwaConfig};
use vi_reference::params::MAX_COST;
use vi_reference::planner::{pose_to_cell, PolicyView};
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::{Action, OccupancyGrid, Quaternion, ValueIterator};

#[derive(Parser)]
#[command(about = "Compare follow controllers (discrete greedy vs continuous DWA) on a solved VI field.")]
struct Args {
    /// Map YAML (bench_map と同じ規約)。既定は同梱の津田沼キャンパス地図。
    #[arg(long)]
    map: Option<PathBuf>,

    /// 整数ダウンサンプル係数。密な states を持つので RAM に注意 (80 B/state —
    /// 津田沼 scale 3 で約 12.6 GB)。前進歩幅 (0.3 m × action_scale) がセルサイズの
    /// 2 倍を切ると値の伝播が退化する (bench_map と同じ制約)。
    #[arg(long, default_value_t = 3)]
    scale: usize,

    /// ゴール X/Y [m] (省略時は地図中心へスナップ)。
    #[arg(long)]
    goal_x: Option<f64>,
    #[arg(long)]
    goal_y: Option<f64>,

    /// ゴール方位 [deg]。
    #[arg(long, default_value_t = 90.0)]
    goal_theta_deg: f64,

    /// ゴール半径 [m] (省略時 max(0.5, 2 セル))。
    #[arg(long)]
    goal_radius_m: Option<f64>,

    /// ゴール方位の半窓 [deg]。
    #[arg(long, default_value_t = 15)]
    goal_margin_theta_deg: i32,

    /// unknown (グレー) セルの扱い。
    #[arg(long, value_enum, default_value_t = UnknownMode::Obstacle)]
    unknown: UnknownMode,

    /// 安全膨張半径 [m]。
    #[arg(long, default_value_t = 0.6)]
    safety_radius_m: f64,

    /// 膨張域ペナルティ (18bit 固定小数点、本家 launch は 100000)。
    #[arg(long, default_value_t = 100000.0)]
    safety_penalty: f64,

    /// 前進歩幅の倍率 (bench_map と同じ意味)。
    #[arg(long, default_value_t = 1.0)]
    action_scale: f64,

    /// ソルバ名 (`U64Solver::from_name`)。frontier2d_sparse は VI_THREADS を読む。
    #[arg(long, default_value = "frontier2d_sparse")]
    solver: String,

    /// ソルバの反復予算。
    #[arg(long, default_value_t = 10_000_000)]
    max_iters: u32,

    /// スタート地点の数 (free かつ到達可能なセルから決定的に乱択)。
    #[arg(long, default_value_t = 6)]
    starts: usize,

    /// 乱択シード。
    #[arg(long, default_value_t = 42)]
    seed: u64,

    /// スタートのゴールからの距離範囲 [m]。
    #[arg(long, default_value_t = 20.0)]
    min_start_m: f64,
    #[arg(long, default_value_t = 60.0)]
    max_start_m: f64,

    /// 制御周期 [s]。
    #[arg(long, default_value_t = 0.1)]
    tick_s: f64,

    /// 1 走行の tick 上限。
    #[arg(long, default_value_t = 6000)]
    max_ticks: usize,

    /// DWA のホライズン [s] / 候補数。
    #[arg(long, default_value_t = 1.0)]
    horizon_s: f64,
    #[arg(long, default_value_t = 7)]
    n_v: usize,
    #[arg(long, default_value_t = 11)]
    n_w: usize,

    /// greedy の近傍借用半径 (チェビシェフ、セル)。vi_planner の action_tolerance 相当。
    #[arg(long, default_value_t = 4)]
    action_tolerance_cells: i32,

    /// CSV 出力先 (省略時は標準出力の表のみ)。
    #[arg(long)]
    out: Option<PathBuf>,
}

#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum UnknownMode {
    Obstacle,
    Free,
}

fn default_map_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/map_tsudanuma.yaml")
}

/// bench_map と同じダウンサンプル占有格子 (`data[x + ow*y]`, 縦反転、obstacle 支配)。
fn build_occupancy(map: &PgmMap, scale: usize, unknown_as_obstacle: bool) -> (Vec<i8>, i32, i32) {
    let w = map.width;
    let h = map.height;
    let ow = w.div_ceil(scale);
    let oh = h.div_ceil(scale);
    let mut occ = vec![0i8; ow * oh];
    for oy in 0..oh {
        for ox in 0..ow {
            let mut blocked = false;
            'blk: for dy in 0..scale {
                let iy = oy * scale + dy;
                if iy >= h {
                    break;
                }
                let src_row = h - 1 - iy;
                for dx in 0..scale {
                    let ix = ox * scale + dx;
                    if ix >= w {
                        break;
                    }
                    let pixel = map.pixels[src_row * w + ix];
                    let c = pgm::classify(pixel, map.negate, map.occupied_thresh, map.free_thresh);
                    let is_obs = matches!(c, Occupancy::Obstacle)
                        || (matches!(c, Occupancy::Unknown) && unknown_as_obstacle);
                    if is_obs {
                        blocked = true;
                        break 'blk;
                    }
                }
            }
            occ[oy * ow + ox] = if blocked { 100 } else { 0 };
        }
    }
    (occ, ow as i32, oh as i32)
}

/// bench_map と同じ「最寄りの free セルへスナップ」。
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
                let (nx, ny) = (gx + dx, gy + dy);
                if nx < 0 || ny < 0 || nx >= w || ny >= h {
                    continue;
                }
                if occ[at(nx, ny)] == 0 {
                    return Some((nx, ny));
                }
            }
        }
    }
    None
}

/// 2 パス chamfer (3-4) 距離変換。障害物セルからの近似ユークリッド距離 (単位 res/3)。
fn chamfer_dist(occ: &[i8], w: i32, h: i32) -> Vec<u32> {
    const INF: u32 = u32::MAX / 2;
    let mut d: Vec<u32> = occ.iter().map(|&c| if c != 0 { 0 } else { INF }).collect();
    let at = |x: i32, y: i32| (y * w + x) as usize;
    for y in 0..h {
        for x in 0..w {
            let mut v = d[at(x, y)];
            if x > 0 {
                v = v.min(d[at(x - 1, y)] + 3);
            }
            if y > 0 {
                v = v.min(d[at(x, y - 1)] + 3);
                if x > 0 {
                    v = v.min(d[at(x - 1, y - 1)] + 4);
                }
                if x < w - 1 {
                    v = v.min(d[at(x + 1, y - 1)] + 4);
                }
            }
            d[at(x, y)] = v;
        }
    }
    for y in (0..h).rev() {
        for x in (0..w).rev() {
            let mut v = d[at(x, y)];
            if x < w - 1 {
                v = v.min(d[at(x + 1, y)] + 3);
            }
            if y < h - 1 {
                v = v.min(d[at(x, y + 1)] + 3);
                if x < w - 1 {
                    v = v.min(d[at(x + 1, y + 1)] + 4);
                }
                if x > 0 {
                    v = v.min(d[at(x - 1, y + 1)] + 4);
                }
            }
            d[at(x, y)] = v;
        }
    }
    d
}

/// xorshift64* (決定的スタート乱択用)。
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n.max(1)
    }
}

fn scaled_actions(scale: f64) -> Vec<Action> {
    canonical_actions()
        .into_iter()
        .enumerate()
        .map(|(i, a)| Action::new(&a.name, a.delta_fw * scale, a.delta_rot, i as i32))
        .collect()
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Ctrl {
    Greedy,
    Dwa,
}
impl Ctrl {
    fn name(self) -> &'static str {
        match self {
            Ctrl::Greedy => "greedy",
            Ctrl::Dwa => "dwa",
        }
    }
}

/// vi_planner `PlannerCore::decide` 相当 (本家 `posToAction` + 近傍借用)。
enum GreedyOut {
    Goal,
    Act(f64, f64),
    NoAction,
}
fn greedy_decide(vi: &ValueIterator, ix: i32, iy: i32, it: i32, tol: i32) -> GreedyOut {
    if vi.is_final(ix, iy, it) {
        return GreedyOut::Goal;
    }
    if let Some(ai) = vi.action_index(ix, iy, it) {
        let (fw, rot) = vi.action_delta(ai);
        return GreedyOut::Act(fw, rot);
    }
    let mut best: Option<(i64, GreedyOut)> = None;
    for dy in -tol..=tol {
        for dx in -tol..=tol {
            if dx == 0 && dy == 0 {
                continue;
            }
            let (nx, ny) = (ix + dx, iy + dy);
            let cand = if vi.is_final(nx, ny, it) {
                GreedyOut::Goal
            } else if let Some(ai) = vi.action_index(nx, ny, it) {
                let (fw, rot) = vi.action_delta(ai);
                GreedyOut::Act(fw, rot)
            } else {
                continue;
            };
            let d2 = (dx as i64) * (dx as i64) + (dy as i64) * (dy as i64);
            if best.as_ref().map(|(bd, _)| d2 < *bd).unwrap_or(true) {
                best = Some((d2, cand));
            }
        }
    }
    best.map(|(_, c)| c).unwrap_or(GreedyOut::NoAction)
}

#[derive(Clone)]
struct RunResult {
    ctrl: &'static str,
    reached: bool,
    collided: bool,
    out_of_map: bool,
    starved: bool,
    ticks: usize,
    path_len_m: f64,
    rot_deg: f64,
    cmd_dw_deg: f64,
    min_clear_m: f64,
    decide_us_sum: f64,
    decide_us_max: f64,
    decide_calls: u64,
    fallbacks: u64,
}

#[allow(clippy::too_many_arguments)]
fn simulate(
    vi: &ValueIterator,
    chamfer: &[u32],
    ow: i32,
    res: f64,
    ctrl: Ctrl,
    dwa_cfg: &DwaConfig,
    start: (f64, f64, f64),
    args: &Args,
) -> RunResult {
    let (mut x, mut y, mut yaw) = start;
    let mut r = RunResult {
        ctrl: ctrl.name(),
        reached: false,
        collided: false,
        out_of_map: false,
        starved: false,
        ticks: 0,
        path_len_m: 0.0,
        rot_deg: 0.0,
        cmd_dw_deg: 0.0,
        min_clear_m: f64::INFINITY,
        decide_us_sum: 0.0,
        decide_us_max: 0.0,
        decide_calls: 0,
        fallbacks: 0,
    };
    let mut prev_cmd: Option<(f64, f64)> = None;
    let mut starve = 0usize;

    for _ in 0..args.max_ticks {
        let (ix, iy, it) = pose_to_cell(vi, x, y, yaw);
        if !PolicyView::in_map_area(vi, ix, iy) || it < 0 || it >= vi.cell_num_t {
            r.out_of_map = true;
            break;
        }
        let clr = chamfer[(iy * ow + ix) as usize] as f64 / 3.0 * res;
        r.min_clear_m = r.min_clear_m.min(clr);
        if !CostView::free_at(vi, ix, iy) {
            r.collided = true;
            break;
        }
        if vi.is_final(ix, iy, it) {
            r.reached = true;
            break;
        }

        let t0 = Instant::now();
        let cmd: Option<(f64, f64)> = match ctrl {
            Ctrl::Greedy => match greedy_decide(vi, ix, iy, it, args.action_tolerance_cells) {
                GreedyOut::Goal => {
                    r.reached = true;
                    break;
                }
                GreedyOut::Act(fw, rot) => Some((fw, rot)),
                GreedyOut::NoAction => None,
            },
            Ctrl::Dwa => match dwa_decide(vi, vi, dwa_cfg, x, y, yaw) {
                Some(c) => Some((c.v, c.w_deg)),
                None => {
                    r.fallbacks += 1;
                    match greedy_decide(vi, ix, iy, it, args.action_tolerance_cells) {
                        GreedyOut::Goal => {
                            r.reached = true;
                            break;
                        }
                        GreedyOut::Act(fw, rot) => Some((fw, rot)),
                        GreedyOut::NoAction => None,
                    }
                }
            },
        };
        let us = t0.elapsed().as_secs_f64() * 1e6;
        r.decide_us_sum += us;
        r.decide_us_max = r.decide_us_max.max(us);
        r.decide_calls += 1;

        let Some((v, w_deg)) = cmd else {
            starve += 1;
            r.ticks += 1;
            if starve > 50 {
                r.starved = true;
                break;
            }
            continue;
        };
        starve = 0;
        if let Some((_, pw)) = prev_cmd {
            r.cmd_dw_deg += (w_deg - pw).abs();
        }
        prev_cmd = Some((v, w_deg));

        let (nx2, ny2, nyaw) = unicycle_step(x, y, yaw, v, w_deg.to_radians(), args.tick_s);
        r.path_len_m += ((nx2 - x).powi(2) + (ny2 - y).powi(2)).sqrt();
        r.rot_deg += (w_deg * args.tick_s).abs();
        x = nx2;
        y = ny2;
        yaw = nyaw;
        r.ticks += 1;
    }
    r
}

fn mean(vals: impl Iterator<Item = f64>) -> f64 {
    let v: Vec<f64> = vals.collect();
    if v.is_empty() {
        f64::NAN
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}

fn main() -> ExitCode {
    let args = Args::parse();
    let map_path = args.map.clone().unwrap_or_else(default_map_path);
    eprintln!("loading map: {}", map_path.display());
    let map = match pgm::load(&map_path) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("error: failed to load map: {e}");
            return ExitCode::from(2);
        }
    };
    let full_res = map.resolution;
    let res = full_res * args.scale as f64;
    let (occ, ow, oh) = build_occupancy(&map, args.scale, args.unknown == UnknownMode::Obstacle);
    eprintln!("grid: {ow}x{oh}x{N_THETA} @ {res:.3} m/cell ({} states)", (ow as u64) * (oh as u64) * N_THETA as u64);

    // ゴール: bench_map と同じ「中心既定 + free スナップ」。
    let goal_x = args.goal_x.unwrap_or(map.origin_x + map.width as f64 * full_res / 2.0);
    let goal_y = args.goal_y.unwrap_or(map.origin_y + map.height as f64 * full_res / 2.0);
    let req_gx = (((goal_x - map.origin_x) / res).floor() as i32).clamp(0, ow - 1);
    let req_gy = (((goal_y - map.origin_y) / res).floor() as i32).clamp(0, oh - 1);
    let Some((gx, gy)) = snap_to_free(&occ, ow, oh, req_gx, req_gy, ow.max(oh)) else {
        eprintln!("error: no free cell near goal");
        return ExitCode::from(2);
    };
    let goal_wx = map.origin_x + (gx as f64 + 0.5) * res;
    let goal_wy = map.origin_y + (gy as f64 + 0.5) * res;
    let goal_radius = args.goal_radius_m.unwrap_or((2.0 * res).max(0.5));
    eprintln!("goal: ({goal_wx:.2}, {goal_wy:.2}) r={goal_radius:.2} m theta={} deg", args.goal_theta_deg);

    let grid = OccupancyGrid {
        width: ow,
        height: oh,
        resolution: res,
        origin_x: map.origin_x,
        origin_y: map.origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: occ.clone(),
    };
    let actions = scaled_actions(args.action_scale);
    let max_fw = actions.iter().map(|a| a.delta_fw).fold(0.0f64, f64::max);
    if max_fw < 2.0 * res - 1e-9 {
        eprintln!(
            "warning: max forward step {max_fw:.2} m < 2 cells ({:.2} m) — value propagation may \
             degenerate; consider --action-scale {:.1}",
            2.0 * res,
            2.0 * res / 0.3
        );
    }
    let mut vi = ValueIterator::new(actions, 1);
    vi.set_map_with_occupancy_grid(
        &grid,
        N_THETA,
        args.safety_radius_m,
        args.safety_penalty,
        goal_radius,
        args.goal_margin_theta_deg,
    );
    vi.set_goal(goal_wx, goal_wy, args.goal_theta_deg as i32);

    let Some(solver) = U64Solver::from_name(&args.solver) else {
        eprintln!("error: unknown solver {}", args.solver);
        return ExitCode::from(2);
    };
    eprintln!("solving with {} ...", args.solver);
    let t0 = Instant::now();
    let stats = solve(&mut vi, solver, args.max_iters);
    eprintln!(
        "solved: iters={} updates={} {:.1} s converged={}",
        stats.iters,
        stats.updates,
        t0.elapsed().as_secs_f64(),
        if stats.converged { "Y" } else { "N" }
    );
    if !stats.converged {
        eprintln!("warning: field not converged; follow comparison may be meaningless");
    }

    let chamfer = chamfer_dist(&occ, ow, oh);

    // スタート乱択: free、障害物から 2 セル以上、ゴール距離 [min,max]、到達可能。
    let mut rng = Rng(args.seed.max(1));
    let mut starts: Vec<(f64, f64, f64)> = Vec::new();
    let mut attempts = 0u64;
    while starts.len() < args.starts && attempts < 2_000_000 {
        attempts += 1;
        let ix = rng.below(ow as u64) as i32;
        let iy = rng.below(oh as u64) as i32;
        if occ[(iy * ow + ix) as usize] != 0 || chamfer[(iy * ow + ix) as usize] < 6 {
            continue;
        }
        let x = map.origin_x + (ix as f64 + 0.5) * res;
        let y = map.origin_y + (iy as f64 + 0.5) * res;
        let dist = ((x - goal_wx).powi(2) + (y - goal_wy).powi(2)).sqrt();
        if dist < args.min_start_m || dist > args.max_start_m {
            continue;
        }
        let yaw_deg = rng.below(360) as f64;
        let it = ((yaw_deg / vi.t_resolution).floor() as i32).clamp(0, N_THETA - 1);
        if vi.value_at(ix, iy, it) >= MAX_COST {
            continue; // このスタートからゴールへ到達不能。
        }
        starts.push((x, y, yaw_deg.to_radians()));
    }
    if starts.len() < args.starts {
        eprintln!(
            "warning: only {} starts found (requested {}) after {attempts} attempts",
            starts.len(),
            args.starts
        );
    }
    if starts.is_empty() {
        eprintln!("error: no valid start found");
        return ExitCode::from(2);
    }

    let mut dwa_cfg = DwaConfig::from_actions(&vi.actions, args.tick_s);
    dwa_cfg.horizon_s = args.horizon_s;
    dwa_cfg.n_v = args.n_v;
    dwa_cfg.n_w = args.n_w;

    let mut results: Vec<(usize, RunResult)> = Vec::new();
    for (si, &start) in starts.iter().enumerate() {
        let dist = ((start.0 - goal_wx).powi(2) + (start.1 - goal_wy).powi(2)).sqrt();
        println!(
            "start {si} @ ({:.2}, {:.2}, {:.0} deg) dist={dist:.1} m",
            start.0,
            start.1,
            start.2.to_degrees()
        );
        for ctrl in [Ctrl::Greedy, Ctrl::Dwa] {
            let r = simulate(&vi, &chamfer, ow, res, ctrl, &dwa_cfg, start, &args);
            let outcome = if r.reached {
                "reached".to_string()
            } else if r.collided {
                "COLLIDED".to_string()
            } else if r.out_of_map {
                "OUT_OF_MAP".to_string()
            } else if r.starved {
                "NO_ACTION".to_string()
            } else {
                "TIMEOUT".to_string()
            };
            println!(
                "  {:6}: {outcome:9} ticks={:5} ({:6.1} s)  len={:6.1} m  rot={:7.1} deg  d|w|={:8.1} deg/s  clr={:5.2} m  decide={:6.1}/{:.0} us{}",
                r.ctrl,
                r.ticks,
                r.ticks as f64 * args.tick_s,
                r.path_len_m,
                r.rot_deg,
                r.cmd_dw_deg,
                r.min_clear_m,
                r.decide_us_sum / r.decide_calls.max(1) as f64,
                r.decide_us_max,
                if r.fallbacks > 0 { format!("  fallback={}", r.fallbacks) } else { String::new() },
            );
            results.push((si, r));
        }
    }

    // 集計 (到達した走行のみの平均 + 到達率)。
    println!();
    println!("| ctrl | reach | ticks | time_s | len_m | rot_deg | cmd_dw | min_clr_m | decide_us | max_us |");
    println!("|------|-------|-------|--------|-------|---------|--------|-----------|-----------|--------|");
    for ctrl in ["greedy", "dwa"] {
        let all: Vec<&RunResult> = results.iter().map(|(_, r)| r).filter(|r| r.ctrl == ctrl).collect();
        let ok: Vec<&&RunResult> = all.iter().filter(|r| r.reached).collect();
        println!(
            "| {ctrl} | {}/{} | {:.0} | {:.1} | {:.1} | {:.1} | {:.1} | {:.2} | {:.1} | {:.0} |",
            ok.len(),
            all.len(),
            mean(ok.iter().map(|r| r.ticks as f64)),
            mean(ok.iter().map(|r| r.ticks as f64 * args.tick_s)),
            mean(ok.iter().map(|r| r.path_len_m)),
            mean(ok.iter().map(|r| r.rot_deg)),
            mean(ok.iter().map(|r| r.cmd_dw_deg)),
            mean(all.iter().map(|r| r.min_clear_m)),
            mean(all.iter().map(|r| r.decide_us_sum / r.decide_calls.max(1) as f64)),
            all.iter().map(|r| r.decide_us_max).fold(0.0f64, f64::max),
        );
    }

    if let Some(path) = &args.out {
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                let _ = std::fs::create_dir_all(parent);
            }
        }
        let mut csv = String::from(
            "start,ctrl,outcome,ticks,time_s,len_m,rot_deg,cmd_dw,min_clr_m,decide_us_avg,decide_us_max,fallbacks\n",
        );
        for (si, r) in &results {
            let outcome = if r.reached {
                "reached"
            } else if r.collided {
                "collided"
            } else if r.out_of_map {
                "out_of_map"
            } else if r.starved {
                "no_action"
            } else {
                "timeout"
            };
            csv.push_str(&format!(
                "{si},{},{outcome},{},{:.1},{:.2},{:.1},{:.1},{:.3},{:.2},{:.1},{}\n",
                r.ctrl,
                r.ticks,
                r.ticks as f64 * args.tick_s,
                r.path_len_m,
                r.rot_deg,
                r.cmd_dw_deg,
                r.min_clear_m,
                r.decide_us_sum / r.decide_calls.max(1) as f64,
                r.decide_us_max,
                r.fallbacks,
            ));
        }
        if let Err(e) = std::fs::write(path, csv) {
            eprintln!("warning: failed to write CSV: {e}");
        } else {
            eprintln!("csv written to {}", path.display());
        }
    }
    ExitCode::SUCCESS
}
