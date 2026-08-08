//! `viola_bench` — VIOLA (Value Iteration with Online Localization and Action)
//! の閉ループ評価。自己位置推定がループに入ると走行がどれだけ劣化するかを測る。
//!
//! 実地図を u64 ソルバで解き (follow_ctrl_bench と同じ規約)、同じ収束場から
//! greedy follow で走行する。ただし世界は **native 解像度**の地図でシミュレート
//! し (真値のレイキャスト・尤度場とも native — vi_planner の実構成と同じ入れ子)、
//! 判断に渡す姿勢だけを 3 way で切り替える:
//!
//! - `truth`  — 真値をそのまま渡す (理想推定の基準線。follow_ctrl_bench の greedy と同条件)。
//! - `dead`   — [`vi_lib::belief::Belief`] を predict のみで回す (デッドレコニング。
//!   correct の寄与を測るアブレーション)。
//! - `belief` — predict + observe (VIOLA 本来の全地図 belief 推定)。
//!
//! 実行 (v, ω) には毎 tick ガウスノイズが乗る (3 モード共通 — 差は推定だけ)。
//! 推定器へは指令値を渡す (ノイズを知らない = 実機と同じ)。ゴール判定は実ノード
//! と同じく**推定姿勢**で行い停止する — 真値が final でなければ GOAL_BEL
//! (信じて停止) としてそのときの真値誤差を記録する。指標: 到達率 / 所要 tick /
//! 位置誤差 RMS・最大 / 方位誤差 RMS / 観測一致度 / 推定 1 tick の計算時間
//! (40 ms 予算比)。
//!
//! 例 (津田沼 scale 3)。既定ゴール (地図中心) は津田沼では閉じた小領域に落ちて
//! スタートが見つからない (follow_ctrl_bench も同じ) ので、ゴールは明示する:
//! ```text
//! cargo run --release -p vi_bench --bin viola_bench -- \
//!     --starts 6 --goal-x 202.73 --goal-y 27.23
//! ```

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use clap::Parser;

use vi_bench::params::N_THETA;
use vi_bench::pgm;
use vi_bench::sim::{
    chamfer_dist, greedy_decide, in_field, mean, scaled_actions, snap_to_free, GreedyOut, Rng,
};
use vi_lib::bridge::PoseView;
use vi_lib::ctrl::{unicycle_step, CostView};
use vi_lib::belief::{cast_scan, Belief, BeliefConfig};
use vi_lib::params::MAX_COST;
use vi_lib::planner::{pose_to_cell, PolicyView};
use vi_lib::solvers::{solve, U64Solver};
use vi_lib::{OccupancyGrid, Quaternion, ValueIterator};

#[derive(Parser)]
#[command(about = "Closed-loop VIOLA benchmark: greedy follow with localization in the loop.")]
struct Args {
    /// Map YAML (bench_map と同じ規約)。既定は同梱の津田沼キャンパス地図。
    #[arg(long)]
    map: Option<PathBuf>,

    /// VI 側の整数ダウンサンプル係数 (密な states を持つので RAM に注意 —
    /// 津田沼 scale 3 で約 12.6 GB)。尤度場・レイキャストは常に native (scale 1)。
    /// scale 6 は粗視化で中心ゴール域が閉塞して不成立 (follow_ctrl_bench と同じ)。
    #[arg(long, default_value_t = 3)]
    scale: usize,

    /// ゴール X/Y [m] (省略時は地図中心へスナップ) と方位 [deg]。
    #[arg(long)]
    goal_x: Option<f64>,
    #[arg(long)]
    goal_y: Option<f64>,
    #[arg(long, default_value_t = 90.0)]
    goal_theta_deg: f64,
    #[arg(long)]
    goal_radius_m: Option<f64>,
    #[arg(long, default_value_t = 15)]
    goal_margin_theta_deg: i32,

    /// unknown (グレー) セルの扱い。
    #[arg(long, value_enum, default_value_t = UnknownMode::Obstacle)]
    unknown: UnknownMode,

    /// 安全膨張半径 [m] / 膨張域ペナルティ。
    #[arg(long, default_value_t = 0.6)]
    safety_radius_m: f64,
    #[arg(long, default_value_t = 100000.0)]
    safety_penalty: f64,

    /// 前進歩幅の倍率 (bench_map と同じ意味)。前進歩幅が 2 セルを切ると
    /// 値伝播が退化する (scale 3 の 0.3 m はちょうど境界で可)。
    #[arg(long, default_value_t = 1.0)]
    action_scale: f64,

    /// ソルバ名 (`U64Solver::from_name`)。
    #[arg(long, default_value = "frontier2d_sparse")]
    solver: String,
    #[arg(long, default_value_t = 10_000_000)]
    max_iters: u32,

    /// スタート地点の数 / 乱択シード / ゴール距離範囲 [m]。
    #[arg(long, default_value_t = 6)]
    starts: usize,
    #[arg(long, default_value_t = 42)]
    seed: u64,
    #[arg(long, default_value_t = 20.0)]
    min_start_m: f64,
    #[arg(long, default_value_t = 60.0)]
    max_start_m: f64,

    /// 制御周期 [s] / 1 走行の tick 上限。
    #[arg(long, default_value_t = 0.1)]
    tick_s: f64,
    #[arg(long, default_value_t = 6000)]
    max_ticks: usize,

    /// greedy の近傍借用半径 (チェビシェフ、セル)。
    #[arg(long, default_value_t = 4)]
    action_tolerance_cells: i32,

    /// 実行ノイズ: 毎 tick の v [m/s] / ω [deg/s] に乗るガウス σ。
    #[arg(long, default_value_t = 0.02)]
    noise_v: f64,
    #[arg(long, default_value_t = 2.0)]
    noise_w_deg: f64,

    /// シードの真値からのずらし量 [m] / [deg] (方向は乱択)。
    #[arg(long, default_value_t = 0.1)]
    seed_offset_m: f64,
    #[arg(long, default_value_t = 5.0)]
    seed_offset_deg: f64,

    /// スキャンのビーム数と最大レンジ [m] (belief の max_range も兼ねる)。
    #[arg(long, default_value_t = 360)]
    scan_beams: usize,
    #[arg(long, default_value_t = 25.0)]
    scan_range: f64,

    /// [`Belief`] のチューニング ([`BeliefConfig`] 対応、実機の校正ノブと同じ)。
    #[arg(long, default_value_t = 0.2)]
    belief_sigma_m: f64,
    #[arg(long, default_value_t = 10)]
    beam_step: usize,
    #[arg(long, default_value_t = 0.03)]
    motion_sigma_xy: f64,
    #[arg(long, default_value_t = 2.0)]
    motion_sigma_theta_deg: f64,
    #[arg(long, default_value_t = 0.3)]
    init_sigma_xy: f64,
    #[arg(long, default_value_t = 10.0)]
    init_sigma_theta_deg: f64,

    /// belief を min-plus (MAP / Viterbi) 更新則で回す。
    #[arg(long)]
    viterbi: bool,

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

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Truth,
    Dead,
    Belief,
}
impl Mode {
    fn name(self) -> &'static str {
        match self {
            Mode::Truth => "truth",
            Mode::Dead => "dead",
            Mode::Belief => "belief",
        }
    }
}

#[derive(Clone)]
struct RunResult {
    mode: &'static str,
    reached: bool,
    /// 推定上のゴール到達で停止した (実ノードの成功宣言と同じ)。truth が
    /// final でなければ「信じて止まった」— そのときの真値誤差が `final_err_m`。
    believed_goal: bool,
    final_err_m: f64,
    collided: bool,
    out_of_map: bool,
    starved: bool,
    ticks: usize,
    path_len_m: f64,
    err_sq_sum: f64,
    err_max_m: f64,
    yaw_sq_sum: f64,
    err_samples: u64,
    quality_sum: f64,
    loc_us_sum: f64,
    loc_us_max: f64,
}

impl RunResult {
    fn err_rms_m(&self) -> f64 {
        (self.err_sq_sum / self.err_samples.max(1) as f64).sqrt()
    }
    fn yaw_rms_deg(&self) -> f64 {
        (self.yaw_sq_sum / self.err_samples.max(1) as f64).sqrt().to_degrees()
    }
    fn quality_mean(&self) -> f64 {
        if self.err_samples == 0 {
            return f64::NAN;
        }
        self.quality_sum / self.err_samples as f64
    }
}

fn wrap_rad(d: f64) -> f64 {
    use std::f64::consts::PI;
    (d + PI).rem_euclid(2.0 * PI) - PI
}

#[allow(clippy::too_many_arguments)]
fn simulate(
    vi: &ValueIterator,
    grid: &OccupancyGrid,
    native: &OccupancyGrid,
    mode: Mode,
    start: (f64, f64, f64),
    goal: (f64, f64),
    seed_pose: PoseView,
    bc: BeliefConfig,
    noise_seed: u64,
    args: &Args,
) -> RunResult {
    let (mut x, mut y, mut yaw) = start;
    let mut rng = Rng(noise_seed.max(1));
    let mut loc = (mode != Mode::Truth).then(|| {
        // 幾何 (belief 格子) は VI と同じスケール後の grid、尤度場だけ native。
        // 実ノードと同じ入れ子 — 旧 GridLocalizer は native 一本だった。
        let mut l = Belief::new(grid, N_THETA, native, bc);
        l.seed(seed_pose);
        l
    });
    let mut r = RunResult {
        mode: mode.name(),
        reached: false,
        believed_goal: false,
        final_err_m: 0.0,
        collided: false,
        out_of_map: false,
        starved: false,
        ticks: 0,
        path_len_m: 0.0,
        err_sq_sum: 0.0,
        err_max_m: 0.0,
        yaw_sq_sum: 0.0,
        err_samples: 0,
        quality_sum: 0.0,
        loc_us_sum: 0.0,
        loc_us_max: 0.0,
    };
    let mut starve = 0usize;

    for _ in 0..args.max_ticks {
        // 物理イベント (地図外・衝突・ゴール到達) は真値で判定する。
        let (tix, tiy, tit) = pose_to_cell(vi, x, y, yaw);
        if !in_field(vi, tix, tiy, tit) {
            r.out_of_map = true;
            break;
        }
        if !CostView::free_at(vi, tix, tiy) {
            r.collided = true;
            break;
        }
        if vi.is_final(tix, tiy, tit) {
            r.reached = true;
            break;
        }

        // 判断は推定姿勢で引く (truth モードは真値がそのまま推定)。
        let est = match &loc {
            None => Some(PoseView { x, y, yaw_rad: yaw }),
            Some(l) => l.pose(),
        };
        if let (Some(e), Some(_)) = (est, &loc) {
            let d = ((e.x - x).powi(2) + (e.y - y).powi(2)).sqrt();
            r.err_sq_sum += d * d;
            r.err_max_m = r.err_max_m.max(d);
            r.yaw_sq_sum += wrap_rad(e.yaw_rad - yaw).powi(2);
            r.err_samples += 1;
            r.quality_sum += loc.as_ref().map(|l| l.quality()).unwrap_or(1.0);
        }

        // 推定姿勢で方策を引く。推定上のゴールなら実ノードと同じく停止して
        // 成功を宣言する — 真値が final でなければ「信じて止まった」となり、
        // そのときの真値のゴール中心距離を final_err_m に残す。
        let mut cmd: Option<(f64, f64)> = None;
        if let Some(e) = est {
            let (ix, iy, it) = pose_to_cell(vi, e.x, e.y, e.yaw_rad);
            if in_field(vi, ix, iy, it) {
                match greedy_decide(vi, ix, iy, it, args.action_tolerance_cells) {
                    GreedyOut::Goal => {
                        r.believed_goal = true;
                        r.final_err_m = ((x - goal.0).powi(2) + (y - goal.1).powi(2)).sqrt();
                        break;
                    }
                    GreedyOut::Act(fw, rot) => cmd = Some((fw, rot)),
                    GreedyOut::NoAction => {}
                }
            }
        }

        let Some((v, w_deg)) = cmd else {
            // 推定が引けない tick はロボットを止める (実機の no-action と同じ)。
            starve += 1;
            r.ticks += 1;
            if starve > 50 {
                r.starved = true;
                break;
            }
            continue;
        };
        starve = 0;

        // 実行 (真値側) にはノイズが乗り、推定器は指令値しか知らない。
        let v_exec = v + args.noise_v * rng.gauss();
        let w_exec = w_deg + args.noise_w_deg * rng.gauss();
        let (nx, ny, nyaw) = unicycle_step(x, y, yaw, v_exec, w_exec.to_radians(), args.tick_s);
        r.path_len_m += ((nx - x).powi(2) + (ny - y).powi(2)).sqrt();
        (x, y, yaw) = (nx, ny, nyaw);

        if let Some(l) = &mut loc {
            let t0 = Instant::now();
            // dead は observe を呼ばないので belief 本体は動かず、pose() が残 pend を
            // 解析加算するだけになる。
            // ponytail: 累積 pend の直線近似 (初期 yaw で 1 本に伸ばす) なので、
            // 旧 GridLocalizer の毎 tick シフトより曲線走行に弱い。correct 無しの
            // 下限を測るアブレーションとしては用途どおり。
            l.predict(v, w_deg, args.tick_s);
            if mode == Mode::Belief {
                let scan = cast_scan(
                    native,
                    PoseView { x, y, yaw_rad: yaw },
                    args.scan_beams,
                    args.scan_range,
                );
                l.observe(&scan);
            }
            let us = t0.elapsed().as_secs_f64() * 1e6;
            r.loc_us_sum += us;
            r.loc_us_max = r.loc_us_max.max(us);
        }
        r.ticks += 1;
    }
    r
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
    let full_res = map.meta.resolution;
    let res = full_res * args.scale as f64;
    let unknown_obst = args.unknown == UnknownMode::Obstacle;
    let (occ, ow, oh) = pgm::build_occupancy(&map, args.scale, unknown_obst);
    let (nocc, nw_, nh_) = pgm::build_occupancy(&map, 1, unknown_obst);
    eprintln!(
        "grid: {ow}x{oh}x{N_THETA} @ {res:.3} m/cell (native {nw_}x{nh_} @ {full_res:.3})"
    );

    let goal_x = args.goal_x.unwrap_or(map.meta.origin_x + map.width as f64 * full_res / 2.0);
    let goal_y = args.goal_y.unwrap_or(map.meta.origin_y + map.height as f64 * full_res / 2.0);
    let req_gx = (((goal_x - map.meta.origin_x) / res).floor() as i32).clamp(0, ow - 1);
    let req_gy = (((goal_y - map.meta.origin_y) / res).floor() as i32).clamp(0, oh - 1);
    let Some((gx, gy)) = snap_to_free(&occ, ow, oh, req_gx, req_gy, ow.max(oh)) else {
        eprintln!("error: no free cell near goal");
        return ExitCode::from(2);
    };
    let goal_wx = map.meta.origin_x + (gx as f64 + 0.5) * res;
    let goal_wy = map.meta.origin_y + (gy as f64 + 0.5) * res;
    let goal_radius = args.goal_radius_m.unwrap_or((2.0 * res).max(0.5));
    eprintln!("goal: ({goal_wx:.2}, {goal_wy:.2}) r={goal_radius:.2} m");

    let grid = OccupancyGrid {
        width: ow,
        height: oh,
        resolution: res,
        origin_x: map.meta.origin_x,
        origin_y: map.meta.origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: occ.clone(),
    };
    let native = OccupancyGrid {
        width: nw_,
        height: nh_,
        resolution: full_res,
        origin_x: map.meta.origin_x,
        origin_y: map.meta.origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: nocc,
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
        "solved: iters={} {:.1} s converged={}",
        stats.iters,
        t0.elapsed().as_secs_f64(),
        if stats.converged { "Y" } else { "N" }
    );

    let bc = BeliefConfig {
        sensor_sigma_m: args.belief_sigma_m,
        beam_step: args.beam_step,
        max_range_m: args.scan_range,
        motion_sigma_xy_m: args.motion_sigma_xy,
        motion_sigma_theta_deg: args.motion_sigma_theta_deg,
        init_sigma_xy_m: args.init_sigma_xy,
        init_sigma_theta_deg: args.init_sigma_theta_deg,
        viterbi: args.viterbi,
        ..BeliefConfig::default()
    };
    {
        let probe = Belief::new(&grid, N_THETA, &native, bc);
        eprintln!("belief: {:.1} MB ({} free cells)", probe.belief_mb(), probe.free_cells());
    }

    // スタート乱択 (follow_ctrl_bench と同じ: free・障害物 2 セル以上・到達可能)。
    let chamfer = chamfer_dist(&occ, ow, oh);
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
        let x = map.meta.origin_x + (ix as f64 + 0.5) * res;
        let y = map.meta.origin_y + (iy as f64 + 0.5) * res;
        let dist = ((x - goal_wx).powi(2) + (y - goal_wy).powi(2)).sqrt();
        if dist < args.min_start_m || dist > args.max_start_m {
            continue;
        }
        let yaw_deg = rng.below(360) as f64;
        let it = ((yaw_deg / vi.t_resolution).floor() as i32).clamp(0, N_THETA - 1);
        if vi.value_at(ix, iy, it) >= MAX_COST {
            continue;
        }
        starts.push((x, y, yaw_deg.to_radians()));
    }
    if starts.is_empty() {
        eprintln!("error: no valid start found");
        return ExitCode::from(2);
    }

    let mut results: Vec<(usize, RunResult)> = Vec::new();
    for (si, &start) in starts.iter().enumerate() {
        // シードのずらし (方向乱択、モード間で共通)。
        let phi = rng.unit() * 2.0 * std::f64::consts::PI;
        let seed_pose = PoseView {
            x: start.0 + args.seed_offset_m * phi.cos(),
            y: start.1 + args.seed_offset_m * phi.sin(),
            yaw_rad: start.2
                + args.seed_offset_deg.to_radians() * if rng.below(2) == 0 { 1.0 } else { -1.0 },
        };
        let noise_seed = rng.next();
        let dist = ((start.0 - goal_wx).powi(2) + (start.1 - goal_wy).powi(2)).sqrt();
        println!(
            "start {si} @ ({:.2}, {:.2}, {:.0} deg) dist={dist:.1} m",
            start.0,
            start.1,
            start.2.to_degrees()
        );
        for mode in [Mode::Truth, Mode::Dead, Mode::Belief] {
            let r = simulate(
                &vi,
                &grid,
                &native,
                mode,
                start,
                (goal_wx, goal_wy),
                seed_pose,
                bc,
                noise_seed,
                &args,
            );
            let outcome = if r.reached {
                "reached".to_string()
            } else if r.believed_goal {
                format!("GOAL_BEL({:.2} m)", r.final_err_m)
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
                "  {:5}: {outcome:16} ticks={:5} ({:6.1} s)  len={:6.1} m  err rms={:5.3}/max={:5.3} m  yaw rms={:5.1} deg  qual={:4.2}  loc={:7.1}/{:.0} us",
                r.mode,
                r.ticks,
                r.ticks as f64 * args.tick_s,
                r.path_len_m,
                r.err_rms_m(),
                r.err_max_m,
                r.yaw_rms_deg(),
                r.quality_mean(),
                r.loc_us_sum / r.ticks.max(1) as f64,
                r.loc_us_max,
            );
            results.push((si, r));
        }
    }

    println!();
    println!("| mode | reach | bel | bel_err_m | ticks | time_s | len_m | err_rms_m | err_max_m | yaw_rms_deg | quality | loc_us | max_us |");
    println!("|------|-------|-----|-----------|-------|--------|-------|-----------|-----------|-------------|---------|--------|--------|");
    for mode in ["truth", "dead", "belief"] {
        let all: Vec<&RunResult> = results.iter().map(|(_, r)| r).filter(|r| r.mode == mode).collect();
        let ok: Vec<&&RunResult> = all.iter().filter(|r| r.reached).collect();
        // 到達扱い (truth final or 信じて停止) の走行。bel_err は後者の真値誤差平均。
        let bel: Vec<&&RunResult> = all.iter().filter(|r| r.believed_goal && !r.reached).collect();
        println!(
            "| {mode} | {}/{} | {} | {:.2} | {:.0} | {:.1} | {:.1} | {:.3} | {:.3} | {:.1} | {:.2} | {:.1} | {:.0} |",
            ok.len(),
            all.len(),
            bel.len(),
            mean(bel.iter().map(|r| r.final_err_m)),
            mean(ok.iter().map(|r| r.ticks as f64)),
            mean(ok.iter().map(|r| r.ticks as f64 * args.tick_s)),
            mean(ok.iter().map(|r| r.path_len_m)),
            mean(all.iter().map(|r| r.err_rms_m())),
            all.iter().map(|r| r.err_max_m).fold(0.0f64, f64::max),
            mean(all.iter().map(|r| r.yaw_rms_deg())),
            mean(all.iter().map(|r| r.quality_mean())),
            mean(all.iter().map(|r| r.loc_us_sum / r.ticks.max(1) as f64)),
            all.iter().map(|r| r.loc_us_max).fold(0.0f64, f64::max),
        );
    }

    if let Some(path) = &args.out {
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                let _ = std::fs::create_dir_all(parent);
            }
        }
        let mut csv = String::from(
            "start,mode,outcome,final_err_m,ticks,time_s,len_m,err_rms_m,err_max_m,yaw_rms_deg,quality,loc_us_avg,loc_us_max\n",
        );
        for (si, r) in &results {
            let outcome = if r.reached {
                "reached"
            } else if r.believed_goal {
                "goal_bel"
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
                "{si},{},{outcome},{:.3},{},{:.1},{:.2},{:.4},{:.4},{:.2},{:.3},{:.2},{:.1}\n",
                r.mode,
                r.final_err_m,
                r.ticks,
                r.ticks as f64 * args.tick_s,
                r.path_len_m,
                r.err_rms_m(),
                r.err_max_m,
                r.yaw_rms_deg(),
                r.quality_mean(),
                r.loc_us_sum / r.ticks.max(1) as f64,
                r.loc_us_max,
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
