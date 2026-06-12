//! `bench_map` — wall-clock benchmark of the u64 (本家忠実) VI solvers on a
//! *real* map.
//!
//! Loads a ROS `map_server` PGM + YAML pair (e.g. the Tsudanuma campus map in
//! `assets/`), builds a `vi_reference::ValueIterator` directly from the
//! occupancy grid (本家 `setMapWithOccupancyGrid` semantics: penalty/goal are
//! computed inside the iterator, in 18-bit fixed point), and runs the Reference
//! and/or Frontier3D u64 solvers to convergence (or until a budget cap),
//! reporting wall-clock per solver. This is the CPU baseline for the `<60 s`
//! FPGA target (see `CLAUDE.md`).
//!
//! The full-resolution Tsudanuma map is 5888×4000×60 ≈ 1.41e9 states; one
//! `ValueIterator::states` vector is therefore tens of GB. Full-res Gauss-Seidel
//! convergence needs sweeps on the order of the map diameter (thousands) and is
//! impractical on CPU — use `--scale` to downsample for tractable convergence
//! numbers. The headline full-res run is a stress test; expect Reference to hit
//! the sweep cap (`converged=N`).
//!
//! ## Valid `--scale` range
//!
//! Downsampling shrinks the grid but the action model has a FIXED physical
//! step (max forward = 0.3 m). Once the cell size exceeds that step
//! (`scale > 6` at 0.05 m/cell), forward moves no longer cross a cell boundary,
//! value stops propagating spatially, and the solver "converges" trivially with
//! near-zero work — a degenerate result, not a smaller version of the same
//! problem. The binary warns when this happens. Meaningful scales are roughly
//! 1–6.
//!
//! Obstacle convention is standard ROS `map_server` (see `pgm.rs`): the image is
//! flipped vertically so grid row `iy=0` is world `y=origin_y`, and obstacle
//! cells become occupancy `100` (the `ValueIterator` treats any non-zero
//! occupancy as blocked: `free = (data == 0)`).
//!
//! ## Reproducing Ueda et al. 2023 (the source paper)
//!
//! The bundled `map_tsudanuma` is the campus map from Ueda et al., "Implementation
//! of Brute-Force Value Iteration..." (J. Robot. Mechatron. 35(6), 2023). The
//! authors' own ROS benchmark min-pools this 0.05 m map ×3, equivalent to our
//! obstacle-dominant `--scale 3 --unknown obstacle`. Matched invocation:
//!
//! ```text
//! bench_map --scale 3 \
//!     --goal-x 0 --goal-y 0 --goal-theta-deg 0 \
//!     --goal-radius-m 0.30 --goal-margin-theta-deg 15 \
//!     --safety-radius-m 0.20 --safety-penalty 100000 --unknown obstacle --solver frontier3d
//! ```
//!
//! Unlike the old u16 port, this binary uses the SAME uint64 18-bit fixed-point
//! cost model as the paper (1 s = 262144), so `--safety-penalty 100000` and the
//! goal geometry are applied exactly as in the paper's launch file.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use clap::Parser;

use vi_bench::fixtures::canonical_actions;
use vi_bench::pgm::{self, Occupancy, PgmMap};
use vi_reference::params::{MAX_COST, PROB_BASE};
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::{OccupancyGrid, Quaternion, State, ValueIterator};

/// Canonical theta cell count (本家 launch / data contract).
const THETA_CELL_NUM: i32 = 60;
/// 最大前進アクション歩幅 [m]（degenerate-dynamics ガード用）。
const MAX_ACTION_FW_M: f64 = 0.3;
/// 到達可能とみなす total_cost 上限（compare.py の value>=1e6 境界と整合）。
const REACH: u64 = 1_000_000u64 * PROB_BASE;

#[derive(Parser)]
#[command(about = "Benchmark u64 VI solvers on a real PGM/YAML map (wall-clock to convergence).")]
struct Args {
    /// Path to the map YAML (resolves its `image:` relative to the YAML dir).
    /// Defaults to the bundled Tsudanuma campus map.
    #[arg(long)]
    map: Option<PathBuf>,

    /// Integer downsample factor. 1 = full resolution. Output dims are
    /// ceil(dim/scale); resolution scales accordingly. Obstacles dominate each
    /// pooled block (conservative).
    #[arg(long, default_value_t = 1)]
    scale: usize,

    /// Goal X in world metres. Defaults to the physical centre of the map.
    #[arg(long)]
    goal_x: Option<f64>,

    /// Goal Y in world metres. Defaults to the physical centre of the map.
    #[arg(long)]
    goal_y: Option<f64>,

    /// Goal heading in degrees.
    #[arg(long, default_value_t = 90.0)]
    goal_theta_deg: f64,

    /// Goal disk radius in metres. Defaults to max(0.5, 2 × cell size).
    #[arg(long)]
    goal_radius_m: Option<f64>,

    /// Goal heading half-window in degrees.
    #[arg(long, default_value_t = 15)]
    goal_margin_theta_deg: i32,

    /// How to treat `map_server` "unknown" (gray) cells.
    #[arg(long, value_enum, default_value_t = UnknownMode::Obstacle)]
    unknown: UnknownMode,

    /// Safety inflation radius in metres (chessboard). 0 disables dilation.
    #[arg(long, default_value_t = 0.0)]
    safety_radius_m: f64,

    /// Penalty applied to free cells within `--safety-radius-m` of an obstacle
    /// (18-bit fixed-point units; the Ueda 2023 launch uses 100000 ≈ 0.38 s).
    #[arg(long, default_value_t = 100000.0)]
    safety_penalty: f64,

    /// Which solver(s) to run.
    #[arg(long, value_enum, default_value_t = SolverSel::Both)]
    solver: SolverSel,

    /// Sweep budget cap for Reference (terminates even without convergence).
    #[arg(long, default_value_t = 2000)]
    max_sweeps: u32,

    /// Iteration budget cap for Frontier3D.
    #[arg(long, default_value_t = 2_000_000)]
    max_iters: u32,

    /// Optional CSV output path (parent dirs created).
    #[arg(long)]
    out: Option<PathBuf>,

    /// Optional path to dump the converged value field (min over theta, seconds)
    /// as a binary `[i32 ow][i32 oh][f32 ow*oh]` (row-major `v[ix + ow*iy]`,
    /// NaN for unreachable). For map-overlay visualisation.
    #[arg(long)]
    dump_value: Option<PathBuf>,

    /// Robot start X/Y in world metres (for `--dump-path`).
    #[arg(long)]
    start_x: Option<f64>,
    #[arg(long)]
    start_y: Option<f64>,

    /// Optional path to dump the optimal robot trajectory (following the converged
    /// policy `optimal_action` from the start pose to the goal) as a binary
    /// `[i32 n][f32 x0][f32 y0]...` of world-metre waypoints. For overlay viz.
    #[arg(long)]
    dump_path: Option<PathBuf>,
}

#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum UnknownMode {
    Obstacle,
    Free,
}

#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum SolverSel {
    Reference,
    Frontier3d,
    /// frontier2d_pad の決定的マルチスレッド版 (本家 並列スイープと対になる CPU 並列ベースライン)。
    #[value(name = "frontier2d_par")]
    Frontier2dPar,
    /// 非同期 (Gauss-Seidel) unsafe 並列版。スレッド間同期を最小化し収束値は bit-exact のまま。
    #[value(name = "frontier2d_par_unsafe")]
    Frontier2dParUnsafe,
    /// 非同期 G-S + penalty 融合レイアウト (バケット 17B→8B)。収束値は bit-exact のまま。
    #[value(name = "frontier2d_fused")]
    Frontier2dFused,
    /// fused + θマスク疎評価 (依存 θ のみ再評価)。収束値は bit-exact のまま。
    #[value(name = "frontier2d_sparse")]
    Frontier2dSparse,
    Both,
}

/// One solver's measurement.
struct Row {
    solver: &'static str,
    iters: u32,
    updates: u64,
    total_ms: f64,
    converged: bool,
}

fn default_map_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/map_tsudanuma.yaml")
}

/// Fractions of obstacle / free / unknown cells in the raw map, for the banner.
fn occupancy_fractions(map: &PgmMap) -> (f64, f64, f64) {
    let (mut obs, mut free, mut unk) = (0u64, 0u64, 0u64);
    for &p in &map.pixels {
        match pgm::classify(p, map.negate, map.occupied_thresh, map.free_thresh) {
            Occupancy::Obstacle => obs += 1,
            Occupancy::Free => free += 1,
            Occupancy::Unknown => unk += 1,
        }
    }
    let n = map.pixels.len().max(1) as f64;
    (obs as f64 / n, free as f64 / n, unk as f64 / n)
}

/// Build a downsampled occupancy grid (row-major, `data[x + ow*y]`, y=0 at world
/// origin). Vertical flip matches `pgm::build_penalty` / `make_goal_mask`.
/// Each output cell is `100` (blocked) if ANY source cell in its `scale×scale`
/// block is an obstacle (conservative pooling), else `0` (free).
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
                let iy = oy * scale + dy; // grid row (world bottom-up)
                if iy >= h {
                    break;
                }
                let src_row = h - 1 - iy; // vertical flip (PGM top-down)
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

/// Find the nearest free cell (`occ == 0`) to `(gx, gy)` by expanding chessboard
/// rings, up to `max_r` cells. Returns the original cell if already free, or
/// `None` if nothing free is found within `max_r`.
fn snap_to_free(occ: &[i8], w: i32, h: i32, gx: i32, gy: i32, max_r: i32) -> Option<(i32, i32)> {
    let at = |x: i32, y: i32| (y * w + x) as usize;
    if gx >= 0 && gx < w && gy >= 0 && gy < h && occ[at(gx, gy)] == 0 {
        return Some((gx, gy));
    }
    for r in 1..=max_r {
        for dy in -r..=r {
            for dx in -r..=r {
                if dx.abs() != r && dy.abs() != r {
                    continue; // ring only
                }
                let nx = gx + dx;
                let ny = gy + dy;
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

fn write_csv(path: &std::path::Path, rows: &[Row]) -> std::io::Result<()> {
    use std::io::Write;
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    let mut f = std::fs::File::create(path)?;
    writeln!(f, "solver,iters,updates,total_ms,converged")?;
    for r in rows {
        writeln!(
            f,
            "{},{},{},{:.3},{}",
            r.solver,
            r.iters,
            r.updates,
            r.total_ms,
            if r.converged { "Y" } else { "N" },
        )?;
    }
    Ok(())
}

fn main() -> ExitCode {
    let args = Args::parse();

    if args.scale == 0 {
        eprintln!("error: --scale must be >= 1");
        return ExitCode::from(2);
    }

    let map_path = args.map.clone().unwrap_or_else(default_map_path);
    eprintln!("loading map: {}", map_path.display());
    let map = match pgm::load(&map_path) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("error: failed to load map: {e}");
            return ExitCode::from(2);
        }
    };

    let (obs_f, free_f, unk_f) = occupancy_fractions(&map);
    let full_res = map.resolution;
    let res = full_res * args.scale as f64;
    let unknown_as_obstacle = args.unknown == UnknownMode::Obstacle;

    // --- Downsampled occupancy grid ---
    let (occ, ow, oh) = build_occupancy(&map, args.scale, unknown_as_obstacle);
    let states = (ow as u64) * (oh as u64) * (THETA_CELL_NUM as u64);
    let free_cells = occ.iter().filter(|&&c| c == 0).count() as u64;
    let free_states = free_cells * THETA_CELL_NUM as u64;

    // --- Goal: default to physical centre of the map, snap to nearest free ---
    let extent_x = map.width as f64 * full_res;
    let extent_y = map.height as f64 * full_res;
    let goal_x = args.goal_x.unwrap_or(map.origin_x + extent_x / 2.0);
    let goal_y = args.goal_y.unwrap_or(map.origin_y + extent_y / 2.0);
    let goal_radius_m = args.goal_radius_m.unwrap_or((2.0 * res).max(0.5));

    let req_gx = (((goal_x - map.origin_x) / res).floor() as i32).clamp(0, ow - 1);
    let req_gy = (((goal_y - map.origin_y) / res).floor() as i32).clamp(0, oh - 1);
    let snap_radius = ow.max(oh).min(2000);
    let (gx, gy) = match snap_to_free(&occ, ow, oh, req_gx, req_gy, snap_radius) {
        Some(c) => c,
        None => {
            eprintln!(
                "error: no free cell within {snap_radius} cells of goal ({goal_x:.2}, {goal_y:.2}); \
                 pass --goal-x/--goal-y onto free space"
            );
            return ExitCode::from(2);
        }
    };
    let goal_wx = map.origin_x + (gx as f64 + 0.5) * res;
    let goal_wy = map.origin_y + (gy as f64 + 0.5) * res;
    let goal_t = args.goal_theta_deg as i32;

    // --- Occupancy grid message (built once; reused across solver rebuilds) ---
    let grid = OccupancyGrid {
        width: ow,
        height: oh,
        resolution: res,
        origin_x: map.origin_x,
        origin_y: map.origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: occ,
    };

    // Build a fresh, fully set-up ValueIterator (map + goal). Cheap relative to
    // the solve; called once per solver so each run starts from a clean state.
    let build = || -> ValueIterator {
        let mut vi = ValueIterator::new(canonical_actions(), 1);
        vi.set_map_with_occupancy_grid(
            &grid,
            THETA_CELL_NUM,
            args.safety_radius_m,
            args.safety_penalty,
            goal_radius_m,
            args.goal_margin_theta_deg,
        );
        vi.set_goal(goal_wx, goal_wy, goal_t);
        vi
    };

    // Goal-mask sanity: count goal (cost-0) cells so an empty goal is visible.
    let goal_cells = {
        let vi = build();
        vi.states.iter().filter(|s| s.total_cost < REACH).count()
    };

    // --- Banner ---
    eprintln!(
        "map: {}x{} px, full-res {:.3} m/cell, origin ({:.1}, {:.1})",
        map.width, map.height, full_res, map.origin_x, map.origin_y
    );
    eprintln!(
        "occupancy (raw): obstacle {:.2}%  free {:.2}%  unknown {:.2}%  (unknown -> {})",
        obs_f * 100.0,
        free_f * 100.0,
        unk_f * 100.0,
        if unknown_as_obstacle { "obstacle" } else { "free" },
    );
    eprintln!(
        "working grid: {}x{}x{} = {} states  (scale {}, {:.3} m/cell)",
        ow, oh, THETA_CELL_NUM, states, args.scale, res,
    );
    eprintln!(
        "free cells: {} ({:.3} m^2)  free states: {}  (Ueda 2023 Actual: 165,076 / 9,904,560)",
        free_cells,
        free_cells as f64 * res * res,
        free_states,
    );
    let states_gb = states as f64 * std::mem::size_of::<State>() as f64 / 1e9;
    eprintln!("est. memory: states {:.2} GB ({} B/state)", states_gb, std::mem::size_of::<State>());
    if req_gx != gx || req_gy != gy {
        eprintln!("goal snapped to nearest free cell");
    }
    eprintln!(
        "goal: world ({:.2}, {:.2}) theta {}deg radius {:.2} m -> cell ({}, {}), {} goal cells",
        goal_wx, goal_wy, goal_t, goal_radius_m, gx, gy, goal_cells,
    );

    if goal_cells == 0 {
        eprintln!(
            "WARNING: goal mask is empty (radius {:.2} m / margin {}deg too tight for this scale); \
             solvers will do no work. Increase --goal-radius-m.",
            goal_radius_m, args.goal_margin_theta_deg,
        );
    }

    // Degenerate-dynamics guard.
    if res > MAX_ACTION_FW_M {
        eprintln!(
            "WARNING: cell size {:.3} m > max action step {:.3} m: moves no longer cross cells. \
             Value will not propagate and convergence is trivial/degenerate. Use --scale <= {}.",
            res,
            MAX_ACTION_FW_M,
            (MAX_ACTION_FW_M / full_res).floor().max(1.0) as u32,
        );
    }

    // --- Build solver schedule ---
    let mut schedule: Vec<(&'static str, U64Solver, u32)> = Vec::new();
    let want_ref = matches!(args.solver, SolverSel::Reference | SolverSel::Both);
    let want_fr = matches!(args.solver, SolverSel::Frontier3d | SolverSel::Both);
    if want_ref {
        schedule.push(("reference", U64Solver::Reference, args.max_sweeps));
    }
    if want_fr {
        schedule.push(("frontier3d", U64Solver::Frontier3D, args.max_iters));
    }
    if matches!(args.solver, SolverSel::Frontier2dPar) {
        schedule.push(("frontier2d_par", U64Solver::Frontier2DPar, args.max_iters));
    }
    if matches!(args.solver, SolverSel::Frontier2dParUnsafe) {
        schedule.push((
            "frontier2d_par_unsafe",
            U64Solver::Frontier2DParUnsafe,
            args.max_iters,
        ));
    }
    if matches!(args.solver, SolverSel::Frontier2dFused) {
        schedule.push(("frontier2d_fused", U64Solver::Frontier2DFused, args.max_iters));
    }
    if matches!(args.solver, SolverSel::Frontier2dSparse) {
        schedule.push(("frontier2d_sparse", U64Solver::Frontier2DSparse, args.max_iters));
    }

    if want_ref && states > 100_000_000 {
        eprintln!(
            "WARNING: reference on {} states will likely hit the --max-sweeps cap ({}) before \
             converging and may run for many minutes/hours. Use --scale to shrink, or \
             --solver frontier3d.",
            states, args.max_sweeps,
        );
    }

    let mut rows: Vec<Row> = Vec::new();
    for (sel, solver, budget) in schedule {
        eprintln!("running {sel} ...");
        let mut vi = build();
        let t0 = Instant::now();
        let stats = solve(&mut vi, solver, budget);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        let row = Row {
            solver: sel,
            iters: stats.iters,
            updates: stats.updates,
            total_ms: ms,
            converged: stats.converged,
        };
        eprintln!(
            "  {} iters={} updates={} total_ms={:.1} converged={}",
            row.solver,
            row.iters,
            row.updates,
            row.total_ms,
            if row.converged { "Y" } else { "N" },
        );
        // Optional value-field dump (min over theta, seconds) for overlay viz.
        if let Some(path) = &args.dump_value {
            use std::io::Write;
            let mut v = vec![f32::NAN; (ow as usize) * (oh as usize)];
            for iy in 0..oh {
                for ix in 0..ow {
                    let mut best = u64::MAX;
                    for it in 0..THETA_CELL_NUM {
                        let idx = vi.to_index(ix, iy, it) as usize;
                        let c = vi.states[idx].total_cost;
                        if c < best {
                            best = c;
                        }
                    }
                    if best < REACH {
                        v[(ix + ow * iy) as usize] = (best as f64 / PROB_BASE as f64) as f32;
                    }
                }
            }
            match std::fs::File::create(path) {
                Ok(mut f) => {
                    let _ = f.write_all(&ow.to_le_bytes());
                    let _ = f.write_all(&oh.to_le_bytes());
                    let bytes: &[u8] = unsafe {
                        std::slice::from_raw_parts(v.as_ptr() as *const u8, v.len() * 4)
                    };
                    let _ = f.write_all(bytes);
                    eprintln!("dumped value field -> {} ({}x{} f32)", path.display(), ow, oh);
                }
                Err(e) => eprintln!("error: failed to write value dump {}: {e}", path.display()),
            }
        }

        // Optional optimal-trajectory dump (follow policy from start to goal).
        if let Some(path_out) = &args.dump_path {
            use std::io::Write;
            let (ox, oy) = (grid.origin_x, grid.origin_y);
            let t_res = 360.0 / THETA_CELL_NUM as f64;
            let sx_w = args.start_x.unwrap_or(goal_wx);
            let sy_w = args.start_y.unwrap_or(goal_wy);
            let six = (((sx_w - ox) / res).floor() as i32).clamp(0, ow - 1);
            let siy = (((sy_w - oy) / res).floor() as i32).clamp(0, oh - 1);
            // best start heading = theta minimising total_cost at the start cell
            let (mut best_it, mut best_c) = (0i32, u64::MAX);
            for it in 0..THETA_CELL_NUM {
                let c = vi.states[vi.to_index(six, siy, it) as usize].total_cost;
                if c < best_c {
                    best_c = c;
                    best_it = it;
                }
            }
            // 本家 vi_node の closed-loop を再現: 10 Hz で姿勢→posToAction→cmd_vel 積分。
            //   cmd_vel.linear.x  = a->_delta_fw            [m/s]  (Δt=1s の前進量を速度として発行)
            //   cmd_vel.angular.z = a->_delta_rot/180*M_PI  [rad/s]
            // 姿勢の離散化も本家 posToAction と同一 (deg の int 切り捨て → %360 → 整数除算)。
            // final_state_ (ゴールマスク) に入ったら "goal" で停止。
            let dt = 0.1f64;
            let t_res_i = (360 / THETA_CELL_NUM) as i32;
            let (mut x, mut y) = (sx_w, sy_w);
            let mut yaw = ((best_it as f64 + 0.5) * t_res).to_radians();
            let mut traj: Vec<(f32, f32)> = Vec::new();
            let mut reached = false;
            for _ in 0..200_000 {
                traj.push((x as f32, y as f32));
                let ix = ((x - ox) / res).floor() as i32;
                let iy = ((y - oy) / res).floor() as i32;
                if ix < 0 || iy < 0 || ix >= ow || iy >= oh {
                    break;
                }
                let t = (180.0 * yaw / std::f64::consts::PI) as i32; // 本家: int 切り捨て
                let it = ((t + 360 * 100) % 360) / t_res_i;
                let s = &vi.states[vi.to_index(ix, iy, it) as usize];
                if s.final_state {
                    reached = true; // 本家: status_="goal"
                    break;
                }
                let a = match s.optimal_action {
                    Some(ai) => &vi.actions[ai],
                    None => break,
                };
                x += a.delta_fw * yaw.cos() * dt;
                y += a.delta_fw * yaw.sin() * dt;
                yaw += a.delta_rot.to_radians() * dt;
                // tf yaw 相当に正規化 ((-π, π])
                yaw = yaw.rem_euclid(2.0 * std::f64::consts::PI);
                if yaw > std::f64::consts::PI {
                    yaw -= 2.0 * std::f64::consts::PI;
                }
            }
            let _ = (gx, gy);
            if !reached {
                // 固着診断: 最終セルの方策テーブル (θごとの optimal_action と V)
                let fx = ((x - ox) / res).floor() as i32;
                let fy = ((y - oy) / res).floor() as i32;
                if fx >= 0 && fy >= 0 && fx < ow && fy < oh {
                    eprintln!("[path debug] stuck at world ({x:.2},{y:.2}) cell ({fx},{fy}) yaw {:.1}deg", yaw.to_degrees());
                    for it in 0..THETA_CELL_NUM {
                        let s = &vi.states[vi.to_index(fx, fy, it) as usize];
                        let an = s.optimal_action.map(|ai| vi.actions[ai].name.as_str()).unwrap_or("-");
                        let qs: Vec<String> = vi.actions.iter()
                            .map(|a| {
                                let q = vi.action_cost(s, a);
                                if q >= MAX_COST { format!("{}=INF", a.name) }
                                else { format!("{}={:.2}", a.name, q as f64 / PROB_BASE as f64) }
                            })
                            .collect();
                        eprintln!("  it={it:2} ({:3}deg) V={:10.2}s a={an}  Q[{}]", it * (360 / THETA_CELL_NUM), s.total_cost as f64 / PROB_BASE as f64, qs.join(" "));
                    }
                }
            }
            match std::fs::File::create(path_out) {
                Ok(mut f) => {
                    let _ = f.write_all(&(traj.len() as i32).to_le_bytes());
                    for (x, y) in &traj {
                        let _ = f.write_all(&x.to_le_bytes());
                        let _ = f.write_all(&y.to_le_bytes());
                    }
                    eprintln!(
                        "dumped path -> {} ({} waypoints; start cell ({},{}) th0={}deg; reached_goal={})",
                        path_out.display(), traj.len(), six, siy, best_it * (t_res as i32), reached,
                    );
                }
                Err(e) => eprintln!("error: failed to write path dump {}: {e}", path_out.display()),
            }
        }

        rows.push(row);
    }

    // --- Markdown table ---
    println!();
    println!("| solver | iters | updates | total_ms | total_s | converged |");
    println!("|--------|-------|---------|----------|---------|-----------|");
    for r in &rows {
        println!(
            "| {} | {} | {} | {:.1} | {:.2} | {} |",
            r.solver,
            r.iters,
            r.updates,
            r.total_ms,
            r.total_ms / 1000.0,
            if r.converged { "Y" } else { "N" },
        );
    }
    if rows.iter().any(|r| r.solver == "reference") {
        println!();
        println!("_note: reference `updates` is always 0 (not tracked); use iters (sweeps) + total_s._");
    }

    if let Some(out) = &args.out {
        if let Err(e) = write_csv(out, &rows) {
            eprintln!("error: failed to write CSV {}: {e}", out.display());
            return ExitCode::from(2);
        }
        eprintln!("wrote {} ({} rows)", out.display(), rows.len());
    }

    ExitCode::SUCCESS
}
