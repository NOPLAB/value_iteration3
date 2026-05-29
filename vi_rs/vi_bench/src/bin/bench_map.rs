//! `bench_map` — wall-clock benchmark of the VI solvers on a *real* map.
//!
//! Loads a ROS `map_server` PGM + YAML pair (e.g. the Tsudanuma campus map in
//! `assets/`), builds a penalty field + goal mask, and runs the Reference
//! and/or Frontier3D solvers to convergence (or until a budget cap), reporting
//! wall-clock per solver. This is the CPU baseline for the `<60 s` FPGA target
//! (see `CLAUDE.md`).
//!
//! The full-resolution Tsudanuma map is 5888×4000×60 ≈ 1.41e9 states; a single
//! `value` table is ~2.83 GB and the goal mask ~1.41 GB, so one context is
//! ~4.3 GB. Full-res Gauss-Seidel convergence needs sweeps on the order of the
//! map diameter (thousands) and is impractical on CPU — use `--scale` to
//! downsample for tractable convergence numbers. The headline full-res run is a
//! stress test; expect Reference to hit the sweep cap (`converged=N`).
//!
//! ## Valid `--scale` range
//!
//! Downsampling shrinks the grid but the action model has a FIXED physical
//! step (max forward = 0.3 m). Once the cell size exceeds that step
//! (`scale > 6` at 0.05 m/cell), forward moves no longer cross a cell boundary,
//! value stops propagating spatially, and the solver "converges" trivially with
//! near-zero work — a degenerate result, not a smaller version of the same
//! problem. The binary warns when this happens. Meaningful scales are roughly
//! 1–6. Reference data point: `--scale 4` (0.2 m/cell, 88 M states) converges
//! with Frontier3D in ~105 s on a typical workstation; `--scale 1` is ~16× that.
//!
//! Obstacle convention is standard ROS `map_server` (see `pgm.rs`), NOT the C
//! `host/src/penalty.c` cost-map reading.
//!
//! ## Reproducing Ueda et al. 2023 (the source paper)
//!
//! The bundled `map_tsudanuma` is the campus map from Ueda et al., "Implementation
//! of Brute-Force Value Iteration..." (J. Robot. Mechatron. 35(6), 2023). Their
//! "Actual" experiment (Table 2) used the SAME 294.3×199.95 m area but at
//! 0.15 m/cell — exactly 3× our 0.05 m PGM. The authors' own ROS benchmark
//! (`ryuichiueda/value_iteration` → `launch/benchmark_tsudanuma.launch` +
//! `test/docker/benchmark/`) takes this same 0.05 m map as input and MIN-pools it
//! ×3, which is equivalent to our obstacle-dominant `--scale 3 --unknown obstacle`.
//! Matched invocation (set RAYON_NUM_THREADS=8 to match their 8 planner threads):
//!
//! ```text
//! RAYON_NUM_THREADS=8 bench_map --scale 3 \
//!     --goal-x 0 --goal-y 0 --goal-theta-deg 0 \
//!     --goal-radius-m 0.30 --goal-margin-theta-deg 15 \
//!     --safety-radius-m 0.20 --safety-penalty 1 --unknown obstacle --solver frontier3d
//! ```
//!
//! This yields 1963×1334×60 = 157,118,520 states (paper: 156,920,760). 6 actions
//! and Nθ=60 match Table 1 exactly; goal geometry matches the launch file
//! (goal (0,0,0°), radius 0.30 m, ±15°).
//!
//! **Irreducible differences (cannot be matched in this value system):**
//! - *Free space*: our open SLAM map has ~730 k free cells vs the paper's 165,076.
//!   The paper restricted the robot to paved areas with hand-drawn black lines on a
//!   private planning map that is NOT in the repo. So solve times are not a
//!   like-for-like workload comparison (we propagate ~44 M free states, paper ~9.9 M).
//! - *Cost quantization*: the paper uses uint64 fixed-point with 18 fractional bits
//!   (1 s = 262144). Our `Value` is u16 with STEP_COST=1, so one step = 1 "second"
//!   and there are no sub-step units. Their `safety_radius_penalty` 100000 ≈ 0.38 s
//!   and their convergence threshold 0.1 s both round to <1 step here — we use
//!   `--safety-penalty 1` (minimal representable) and threshold 0 (exact). These
//!   barely affect convergence time (the reachable set is unchanged).
//! - *Algorithm*: the paper's Node B is multi-threaded full Gauss-Seidel sweeps
//!   (Eq. 13). Frontier3D reaches the SAME fixed point V* via a Dijkstra-like
//!   priority frontier — far fewer updates, so its wall-clock is not the paper's
//!   sweep cost. Use `--solver reference` for the literal sweep algorithm, but on
//!   this 44 M-free-state map exact-convergence Reference is hours on CPU (the very
//!   reason this project exists). Their hardware: i7-11800H, 8 planner threads.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use clap::Parser;
use ndarray::{Array3, Zip};

use vi_algorithm::context::{Budget, MapDims, SolveStats, Solver, VIContext};
use vi_algorithm::{Frontier3D, Reference};
use vi_bench::pgm::{self, Occupancy, PgmMap};
use vi_core::params::{ACTION_FW, MAX_VALUE, N_THETA};
use vi_core::{make_goal_mask, GoalSpec, Penalty, Value, PENALTY_OBSTACLE};
use vi_fixtures::{generate_transitions, TransitionMode};

/// Was the binary built with `--features parallel`? Mirrors `bench_summary`.
#[cfg(feature = "parallel")]
const HAS_PARALLEL: bool = true;
#[cfg(not(feature = "parallel"))]
const HAS_PARALLEL: bool = false;

#[derive(Parser)]
#[command(about = "Benchmark VI solvers on a real PGM/YAML map (wall-clock to convergence).")]
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

    /// Goal disk radius in metres. Defaults to max(0.5, 2 × cell size) so the
    /// goal mask is non-empty even at coarse scales.
    #[arg(long)]
    goal_radius_m: Option<f64>,

    /// Goal heading half-window in degrees.
    #[arg(long, default_value_t = 15.0)]
    goal_margin_theta_deg: f64,

    /// How to treat `map_server` "unknown" (gray) cells.
    #[arg(long, value_enum, default_value_t = UnknownMode::Obstacle)]
    unknown: UnknownMode,

    /// Safety inflation radius in metres (chessboard). 0 disables dilation.
    #[arg(long, default_value_t = 0.0)]
    safety_radius_m: f64,

    /// Penalty applied to free cells within `--safety-radius-m` of an obstacle
    /// (kept below the obstacle sentinel, so the cell stays passable). The Ueda
    /// 2023 benchmark uses 100000 in 18-bit fixed-point ≈ 0.38 step here, which
    /// rounds to 1 in this integer-step value system.
    #[arg(long, default_value_t = 1000)]
    safety_penalty: u16,

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
    Both,
}

/// Read-only inputs shared across solver runs. Moved into a `VIContext` for
/// each run and moved back out afterwards, so the ~1.4 GB goal mask is built
/// once and never cloned.
struct Shared {
    penalty: ndarray::Array2<Penalty>,
    goal_mask: Array3<bool>,
    transitions: vi_core::TransitionModel,
}

/// One solver's measurement.
struct Row {
    solver: &'static str,
    mode: &'static str,
    iters_or_sweeps: u32,
    updates: u64,
    total_ms: f64,
    converged: bool,
}

fn default_map_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/map_tsudanuma.yaml")
}

/// Fractions of obstacle / free / unknown cells in the raw map (pre-flip,
/// pre-downsample), for the startup banner.
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

/// Pin goal cells (mask true) to value 0; everything else to MAX_VALUE.
fn init_value(goal_mask: &Array3<bool>) -> Array3<Value> {
    let mut value = Array3::from_elem(goal_mask.raw_dim(), MAX_VALUE);
    Zip::from(&mut value).and(goal_mask).for_each(|v, &g| {
        if g {
            *v = 0;
        }
    });
    value
}

/// Find the nearest free cell (`penalty == 0`) to `(gx, gy)` by expanding
/// chessboard rings, up to `max_r` cells. Returns the original cell if it is
/// already free, or `None` if nothing free is found within `max_r`.
fn snap_to_free(
    penalty: &ndarray::Array2<Penalty>,
    gx: usize,
    gy: usize,
    max_r: usize,
) -> Option<(usize, usize)> {
    let (h, w) = penalty.dim();
    if gy < h && gx < w && penalty[[gy, gx]] == 0 {
        return Some((gx, gy));
    }
    for r in 1..=max_r {
        let r = r as isize;
        for dy in -r..=r {
            for dx in -r..=r {
                if dx.abs() != r && dy.abs() != r {
                    continue; // ring only
                }
                let ny = gy as isize + dy;
                let nx = gx as isize + dx;
                if ny < 0 || nx < 0 || ny >= h as isize || nx >= w as isize {
                    continue;
                }
                if penalty[[ny as usize, nx as usize]] == 0 {
                    return Some((nx as usize, ny as usize));
                }
            }
        }
    }
    None
}

/// Run one solver: build a fresh value table, move the shared inputs into a
/// context, solve, then move them back out. Returns the row plus the reclaimed
/// shared inputs.
fn run_solver(
    sel: &'static str,
    boxed: Box<dyn Solver>,
    budget: Budget,
    dims: MapDims,
    shared: Shared,
) -> (Row, Shared) {
    let value = init_value(&shared.goal_mask);
    let mut ctx = VIContext {
        dims,
        value,
        penalty: shared.penalty,
        goal_mask: shared.goal_mask,
        transitions: shared.transitions,
    };

    let t0 = Instant::now();
    let stats: SolveStats = boxed.run(&mut ctx, budget);
    let ms = t0.elapsed().as_secs_f64() * 1000.0;

    let capable = matches!(sel, "reference" | "frontier_3d");
    let mode = if HAS_PARALLEL && capable { "parallel" } else { "serial" };

    let row = Row {
        solver: sel,
        mode,
        iters_or_sweeps: stats.iters_or_sweeps,
        updates: stats.updates,
        total_ms: ms,
        converged: stats.converged,
    };

    // Reclaim the shared inputs (value is dropped here).
    let shared = Shared {
        penalty: ctx.penalty,
        goal_mask: ctx.goal_mask,
        transitions: ctx.transitions,
    };
    (row, shared)
}

fn write_csv(path: &std::path::Path, rows: &[Row]) -> std::io::Result<()> {
    use std::io::Write;
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    let mut f = std::fs::File::create(path)?;
    writeln!(f, "solver,mode,iters_or_sweeps,updates,total_ms,converged")?;
    for r in rows {
        writeln!(
            f,
            "{},{},{},{},{:.3},{}",
            r.solver,
            r.mode,
            r.iters_or_sweeps,
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

    // --- Penalty field (full-res classify + dilation, then downsample) ---
    let safety_cells = (args.safety_radius_m / full_res).round() as usize;
    let unknown_as_obstacle = args.unknown == UnknownMode::Obstacle;
    let penalty_full = pgm::build_penalty(&map, unknown_as_obstacle, safety_cells, args.safety_penalty);
    let penalty = pgm::downsample_penalty(&penalty_full, args.scale);
    drop(penalty_full);

    let (my, mx) = penalty.dim();
    let map_x = mx as u32;
    let map_y = my as u32;
    let states = (mx as u64) * (my as u64) * (N_THETA as u64);

    // --- Goal: default to physical centre of the map, snap to nearest free ---
    let extent_x = map.width as f64 * full_res;
    let extent_y = map.height as f64 * full_res;
    let goal_x = args.goal_x.unwrap_or(map.origin_x + extent_x / 2.0);
    let goal_y = args.goal_y.unwrap_or(map.origin_y + extent_y / 2.0);
    let goal_radius_m = args.goal_radius_m.unwrap_or((2.0 * res).max(0.5));

    // Requested goal cell (clamped) and snap to a free cell if needed.
    let req_gx = (((goal_x - map.origin_x) / res).floor() as isize).clamp(0, mx as isize - 1) as usize;
    let req_gy = (((goal_y - map.origin_y) / res).floor() as isize).clamp(0, my as isize - 1) as usize;
    let snap_radius = (mx.max(my)).min(2000);
    let (gx, gy) = match snap_to_free(&penalty, req_gx, req_gy, snap_radius) {
        Some(c) => c,
        None => {
            eprintln!(
                "error: no free cell within {snap_radius} cells of goal ({goal_x:.2}, {goal_y:.2}); \
                 pass --goal-x/--goal-y onto free space"
            );
            return ExitCode::from(2);
        }
    };
    // World coords of the (possibly snapped) goal cell centre.
    let goal_wx = map.origin_x + (gx as f64 + 0.5) * res;
    let goal_wy = map.origin_y + (gy as f64 + 0.5) * res;

    let spec = GoalSpec {
        xy_resolution: res,
        map_origin_x: map.origin_x,
        map_origin_y: map.origin_y,
        goal_x: goal_wx,
        goal_y: goal_wy,
        goal_theta_deg: args.goal_theta_deg,
        goal_radius_m,
        goal_margin_theta_deg: args.goal_margin_theta_deg,
    };
    let mut goal_mask = make_goal_mask(map_x, map_y, &spec);
    let mut goal_cells = goal_mask.iter().filter(|&&b| b).count();

    // Fallback: if the disk/theta window produced nothing (coarse scale or
    // tiny radius), pin the goal cell itself across the theta window so the
    // solvers have a non-empty goal to propagate from.
    if goal_cells == 0 {
        eprintln!("note: goal mask empty for given radius; pinning goal cell across theta window");
        for it in 0..N_THETA {
            goal_mask[[gy, gx, it]] = true;
        }
        goal_cells = goal_mask.iter().filter(|&&b| b).count();
    }

    // --- Banner ---
    eprintln!("map: {}x{} px, full-res {:.3} m/cell, origin ({:.1}, {:.1})",
        map.width, map.height, full_res, map.origin_x, map.origin_y);
    eprintln!(
        "occupancy (raw): obstacle {:.2}%  free {:.2}%  unknown {:.2}%  (unknown -> {})",
        obs_f * 100.0, free_f * 100.0, unk_f * 100.0,
        if unknown_as_obstacle { "obstacle" } else { "free" },
    );
    let free_cells = penalty.iter().filter(|&&p| p != PENALTY_OBSTACLE).count() as u64;
    let free_states = free_cells * N_THETA as u64;
    eprintln!(
        "working grid: {}x{}x{} = {} states  (scale {}, {:.3} m/cell)",
        mx, my, N_THETA, states, args.scale, res,
    );
    eprintln!(
        "free cells: {} ({:.3} m^2)  free states: {}  (Ueda 2023 Actual: 165,076 / 9,904,560)",
        free_cells, free_cells as f64 * res * res, free_states,
    );
    let value_gb = states as f64 * std::mem::size_of::<Value>() as f64 / 1e9;
    let mask_gb = states as f64 / 1e9; // bool = 1 byte
    eprintln!(
        "est. memory: value {:.2} GB + goal_mask {:.2} GB + penalty {:.0} MB",
        value_gb, mask_gb, (mx * my * std::mem::size_of::<Penalty>()) as f64 / 1e6,
    );
    if req_gx != gx || req_gy != gy {
        eprintln!("goal snapped to nearest free cell");
    }
    eprintln!(
        "goal: world ({:.2}, {:.2}) theta {:.0}deg radius {:.2} m -> cell ({}, {}), {} goal cells",
        goal_wx, goal_wy, args.goal_theta_deg, goal_radius_m, gx, gy, goal_cells,
    );
    eprintln!(
        "parallel: {} (built with --features parallel: {})",
        if HAS_PARALLEL { "on" } else { "off" }, HAS_PARALLEL,
    );

    // Degenerate-dynamics guard: if the cell is larger than the longest action
    // step, forward moves never cross a cell boundary, so value cannot
    // propagate and any "convergence" is meaningless. See module docs.
    let max_fw = ACTION_FW.iter().cloned().fold(0.0_f64, f64::max);
    if res > max_fw {
        eprintln!(
            "WARNING: cell size {:.3} m > max action step {:.3} m: moves no longer cross cells. \
             Value will not propagate and convergence is trivial/degenerate. Use --scale <= {}.",
            res, max_fw, (max_fw / full_res).floor().max(1.0) as u32,
        );
    }

    let transitions = generate_transitions(TransitionMode::PaperMonteCarlo { xy_resolution: res }).unpack();
    let dims = MapDims { map_x, map_y };
    let mut shared = Shared { penalty, goal_mask, transitions };

    // --- Build solver schedule ---
    let mut schedule: Vec<(&'static str, Box<dyn Solver>, Budget)> = Vec::new();
    let want_ref = matches!(args.solver, SolverSel::Reference | SolverSel::Both);
    let want_fr = matches!(args.solver, SolverSel::Frontier3d | SolverSel::Both);
    if want_ref {
        schedule.push((
            "reference",
            Box::new(Reference { threshold: 0 }),
            Budget::Sweeps(args.max_sweeps),
        ));
    }
    if want_fr {
        schedule.push(("frontier_3d", Box::new(Frontier3D), Budget::Iterations(args.max_iters)));
    }

    // Reference does a full pass over every state each sweep and needs ~map-
    // diameter sweeps to converge; on a large grid it will almost certainly hit
    // --max-sweeps first. Flag it so a long run isn't mistaken for a hang.
    if want_ref && states > 100_000_000 {
        eprintln!(
            "WARNING: reference on {} states will likely hit the --max-sweeps cap ({}) before \
             converging and may run for many minutes/hours. Use --scale to shrink, or \
             --solver frontier3d.",
            states, args.max_sweeps,
        );
    }

    let mut rows: Vec<Row> = Vec::new();
    for (sel, boxed, budget) in schedule {
        eprintln!("running {sel} ...");
        let (row, reclaimed) = run_solver(sel, boxed, budget, dims, shared);
        shared = reclaimed;
        eprintln!(
            "  {} mode={} iters_or_sweeps={} updates={} total_ms={:.1} converged={}",
            row.solver, row.mode, row.iters_or_sweeps, row.updates, row.total_ms,
            if row.converged { "Y" } else { "N" },
        );
        rows.push(row);
    }

    // --- Markdown table ---
    println!();
    println!("| solver | mode | iters_or_sweeps | updates | total_ms | total_s | converged |");
    println!("|--------|------|-----------------|---------|----------|---------|-----------|");
    for r in &rows {
        println!(
            "| {} | {} | {} | {} | {:.1} | {:.2} | {} |",
            r.solver, r.mode, r.iters_or_sweeps, r.updates, r.total_ms, r.total_ms / 1000.0,
            if r.converged { "Y" } else { "N" },
        );
    }
    if rows.iter().any(|r| r.solver == "reference") {
        println!();
        println!("_note: reference `updates` is always 0 (not tracked, per vi_algorithm spec §4.8); use sweeps + total_s._");
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
