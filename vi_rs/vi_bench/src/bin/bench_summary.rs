//! `bench_summary` — wall-clock + correctness comparison across solvers.
//!
//! Modeled on `vi_matlab/workflows/benchmarks/benchmark_vi.m`. For each
//! `(map_size, map_type)` case, runs every solver from the same initial
//! `VIContext`, records `(iters_or_sweeps, updates, total_ms, mismatch)`
//! against the Reference solver's value table, and emits a Markdown table
//! to stdout plus an optional CSV.
//!
//! Spec: `docs/superpowers/specs/2026-05-22-vi-rs-algorithm-port-design.md` §6.4.
//!
//! `--smoke` collapses to a single 8×8 Empty case with budgets of 1 and the
//! Trivial transition mode so the CLI wires up under CI budgets (<30 s).
//!
//! `--parallel` (only meaningful when compiled with `--features parallel`):
//! for solvers that have a parallel path (Reference, Frontier3D), emit BOTH
//! the serial and parallel rows so the table directly compares wall-clock
//! between the two. Other solvers are always serial and emit one row.
//! Smoke mode honours this too — `--smoke --parallel` yields 2 rows each for
//! Reference and Frontier3D, 1 row for everything else.

use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Instant;

use clap::Parser;
use ndarray::{Array3, Zip};
use vi_algorithm::context::{Budget, Solver, VIContext};
use vi_algorithm::{
    BlockRefine, Frontier2D, Frontier3D, Frontier3DCoarseTheta, Frontier3DTau, Frontier3DTopK,
    FrontierStack, PyramidSweep, Reference, StreamMimic,
};
use vi_bench::fixtures::build_context;
use vi_core::params::MAX_OUTCOMES;
use vi_core::Value;
use vi_fixtures::{MapType, TransitionMode};

/// Compile-time flag: was the binary built with `--features parallel`?
/// Used to decide whether `--parallel` at runtime can actually take effect.
#[cfg(feature = "parallel")]
const HAS_PARALLEL: bool = true;
#[cfg(not(feature = "parallel"))]
const HAS_PARALLEL: bool = false;

/// Solvers whose output must equal the Reference value table cell-for-cell.
/// A nonzero mismatch on any of these causes a nonzero exit code.
const EXACT_SOLVERS: &[&str] = &[
    "reference",
    "frontier_2d",
    "frontier_3d",
    "frontier_stack",
    "block_refine",
    "pyramid_sweep",
    // StreamMimic is documented as "not required to be bit-exact" (spec §4.8),
    // but in practice the (CU, strip, Y, X) scan order is just another
    // Gauss-Seidel ordering — at the fixed point with threshold=0 it must
    // equal Reference. If a real bench ever flags a mismatch, that's the
    // signal to investigate.
    "stream_mimic",
];

/// Solvers that currently have an explicit parallel path. When the binary is
/// compiled with `--features parallel`, `Solver::run` for these dispatches to
/// the parallel implementation; the explicit serial path is reachable via
/// `Reference::run_serial` / `Frontier3D::run_serial`.
///
/// KEEP IN SYNC with `run_serial_for` — every name returning `true` here must
/// have a matching arm in that function, or it will panic at runtime.
fn is_parallel_capable(name: &str) -> bool {
    matches!(name, "reference" | "frontier_3d")
}

#[derive(Parser)]
#[command(
    about = "Run every VI solver across (map_size × map_type) and emit a comparison table."
)]
struct Args {
    /// Comma-separated map sizes (square).
    #[arg(long, value_delimiter = ',', default_value = "8,16,32,64")]
    sizes: Vec<u32>,

    /// Comma-separated map types: empty,obstacle,sentinel,random.
    #[arg(long, value_delimiter = ',', default_value = "empty,obstacle,sentinel,random")]
    types: Vec<String>,

    /// Sweep budget cap for Reference / BlockRefine / PyramidSweep.
    #[arg(long, default_value_t = 200)]
    max_sweeps: u32,

    /// Iteration budget cap for frontier solvers.
    #[arg(long, default_value_t = 4000)]
    max_iters: u32,

    /// CSV output path. Created (and parent dirs) if missing.
    #[arg(long)]
    out: Option<PathBuf>,

    /// Print Markdown table to stdout in addition to per-case progress lines.
    #[arg(long, default_value_t = false)]
    markdown: bool,

    /// CI smoke mode: override sizes to [8], types to [empty], budgets to 1.
    /// Used to verify every solver wires up without a real-time bench cost.
    #[arg(long, default_value_t = false)]
    smoke: bool,

    /// For solvers that have a parallel path (Reference, Frontier3D), run
    /// BOTH variants per case and emit them as separate rows tagged
    /// `serial` / `parallel`. Requires the binary to be built with
    /// `--features parallel`; otherwise we print a warning and proceed as
    /// serial. Other solvers emit a single `serial` row.
    #[arg(long, default_value_t = false)]
    parallel: bool,
}

/// A single (case, solver, mode) measurement.
struct CaseRow {
    case_label: String,
    solver: &'static str,
    /// `"serial"` or `"parallel"`. Indicates which sweep dispatch produced the
    /// row. Solvers without a parallel implementation always emit `"serial"`.
    mode: &'static str,
    iters_or_sweeps: u32,
    updates: u64,
    total_ms: f64,
    mismatch: u64,
    converged: bool,
}

/// Solver instance + the budget flavor it expects. Built once and reused
/// across cases — `Solver::run` only needs `&self`.
struct SolverEntry {
    boxed: Box<dyn Solver>,
    budget: Budget,
}

fn parse_map_type(s: &str) -> Result<MapType, String> {
    match s {
        "empty" => Ok(MapType::Empty),
        "obstacle" => Ok(MapType::Obstacle),
        "sentinel" => Ok(MapType::Sentinel),
        "random" => Ok(MapType::Random { density: 0.15, seed: 42 }),
        other => Err(format!("unknown map type: {other}")),
    }
}

/// Cell-wise count of differences between two value tables. Shapes must match.
fn value_mismatch(a: &Array3<Value>, b: &Array3<Value>) -> u64 {
    Zip::from(a)
        .and(b)
        .fold(0u64, |acc, &x, &y| acc + (x != y) as u64)
}

/// Build the registry of solvers + their budget flavor. Parameters mirror
/// the criterion benches and the spec.
fn build_solver_registry(max_sweeps: u32, max_iters: u32) -> Vec<SolverEntry> {
    let sweeps = Budget::Sweeps(max_sweeps);
    let iters = Budget::Iterations(max_iters);
    // PyramidSweep: cap refine_sweeps at 50 per spec.
    let pyramid_refine_sweeps = max_sweeps.min(50);

    vec![
        SolverEntry { boxed: Box::new(Reference { threshold: 0 }), budget: sweeps },
        SolverEntry { boxed: Box::new(Frontier2D), budget: iters },
        SolverEntry { boxed: Box::new(Frontier3D), budget: iters },
        SolverEntry { boxed: Box::new(FrontierStack), budget: iters },
        SolverEntry { boxed: Box::new(Frontier3DTau { tau: 0 }), budget: iters },
        SolverEntry {
            boxed: Box::new(Frontier3DTopK { k: MAX_OUTCOMES as u32 }),
            budget: iters,
        },
        SolverEntry {
            // CLI scales refine_iters with the budget; benches use a fixed small value.
            boxed: Box::new(Frontier3DCoarseTheta {
                coarse_step: 4,
                refine_iters: max_iters / 2,
            }),
            budget: iters,
        },
        SolverEntry {
            boxed: Box::new(BlockRefine {
                block_w: 8,
                block_h: 8,
                local_sweeps: 2,
                threshold: 0,
            }),
            budget: sweeps,
        },
        SolverEntry {
            boxed: Box::new(PyramidSweep {
                threshold: 0,
                min_size: 4,
                coarse_sweeps: 8,
                refine_sweeps: pyramid_refine_sweeps,
                // descend_tau MUST be 0 for the exact-oracle gate: a nonzero tau
                // prunes cells whose per-sweep delta <= tau from the descent, so the
                // finest level's active mask would miss reachable cells (they'd keep
                // the pessimistic MAX seed) and mismatch Reference. With tau=0 every
                // changed cell descends, guaranteeing full finest-level coverage.
                descend_tau: 0,
            }),
            budget: sweeps,
        },
        SolverEntry { boxed: Box::new(StreamMimic { threshold: 0 }), budget: sweeps },
    ]
}

fn print_markdown(rows: &[CaseRow]) {
    println!("| case | solver | mode | iters_or_sweeps | updates | total_ms | converged | mismatch |");
    println!("|------|--------|------|-----------------|---------|----------|-----------|----------|");
    for r in rows {
        println!(
            "| {} | {} | {} | {} | {} | {:.3} | {} | {} |",
            r.case_label,
            r.solver,
            r.mode,
            r.iters_or_sweeps,
            r.updates,
            r.total_ms,
            if r.converged { "Y" } else { "N" },
            r.mismatch,
        );
    }
}

fn write_csv(path: &Path, rows: &[CaseRow]) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
    }
    let mut f = fs::File::create(path)?;
    writeln!(
        f,
        "case,solver,mode,iters_or_sweeps,updates,total_ms,converged,mismatch"
    )?;
    for r in rows {
        writeln!(
            f,
            "{},{},{},{},{},{:.3},{},{}",
            r.case_label,
            r.solver,
            r.mode,
            r.iters_or_sweeps,
            r.updates,
            r.total_ms,
            if r.converged { "Y" } else { "N" },
            r.mismatch,
        )?;
    }
    Ok(())
}

/// Smoke mode uses Trivial transitions to skip the 64³ Monte-Carlo build
/// cost in CI; normal runs use PaperMonteCarlo. Either way, the
/// `generate_transitions` cache makes repeated calls within a run free.
fn pick_transition_mode(smoke: bool) -> TransitionMode {
    if smoke {
        TransitionMode::Trivial
    } else {
        TransitionMode::PaperMonteCarlo { xy_resolution: 0.05 }
    }
}

/// Run an explicit serial pass for a `parallel_capable` solver and emit a row
/// tagged `"serial"`. `Box<dyn Solver>` doesn't downcast cleanly, so we
/// re-instantiate the small concrete struct by name. This is only called when
/// the binary was compiled with `--features parallel` AND `--parallel` was
/// passed; outside that path the dispatched `Solver::run` is already the
/// serial implementation.
fn run_serial_for(
    solver_name: &str,
    budget: Budget,
    base: &VIContext,
    ref_values: &Array3<Value>,
    case_label: &str,
) -> CaseRow {
    let mut ctx = base.clone_value();
    let t0 = Instant::now();
    // Pick the concrete struct that owns a public serial entry-point. Both
    // structs are tiny / zero-sized, so re-instantiating per case is free.
    // Threshold here MUST match the one used in `build_solver_registry` so the
    // serial row reflects the same configuration as the parallel one.
    let stats = match solver_name {
        "reference" => Reference { threshold: 0 }.run_serial(&mut ctx, budget),
        "frontier_3d" => Frontier3D.run_serial(&mut ctx, budget),
        other => panic!("run_serial_for: solver '{other}' has no parallel-capable serial path"),
    };
    let ms = t0.elapsed().as_secs_f64() * 1000.0;
    let mismatch = value_mismatch(&ctx.value, ref_values);

    // We can't store solver_name (a `&str`) directly into a `&'static str` row;
    // map back to the known static literal.
    let static_name: &'static str = match solver_name {
        "reference" => "reference",
        "frontier_3d" => "frontier_3d",
        _ => unreachable!(),
    };

    eprintln!(
        "  {case_label} solver={static_name} mode=serial iters={} updates={} ms={:.2} mismatch={mismatch}",
        stats.iters_or_sweeps, stats.updates, ms,
    );

    CaseRow {
        case_label: case_label.to_string(),
        solver: static_name,
        mode: "serial",
        iters_or_sweeps: stats.iters_or_sweeps,
        updates: stats.updates,
        total_ms: ms,
        mismatch,
        converged: stats.converged,
    }
}

fn main() -> ExitCode {
    let mut args = Args::parse();

    // Smoke-mode overrides: keep CI cost trivial. `--parallel` is still
    // honoured in smoke mode — it only changes the row-emission strategy,
    // not how much actual work each row does.
    if args.smoke {
        args.sizes = vec![8];
        args.types = vec!["empty".to_string()];
        args.max_sweeps = 1;
        args.max_iters = 1;
    }

    // Validate map-type labels up front so a typo fails fast before any
    // solver runs.
    for s in &args.types {
        if let Err(e) = parse_map_type(s) {
            eprintln!("error: {e}");
            return ExitCode::from(2);
        }
    }

    // Reconcile the runtime --parallel flag with the compile-time feature.
    // If the user asked for --parallel but the binary lacks the feature,
    // there's no parallel code path to call into; warn and proceed serial.
    let want_dual_rows = if args.parallel && !HAS_PARALLEL {
        eprintln!(
            "--parallel passed but binary was not compiled with --features parallel; treating as serial"
        );
        false
    } else {
        args.parallel
    };

    // Solver registry: built once, reused across all cases (Solver::run only
    // needs &self).
    let registry = build_solver_registry(args.max_sweeps, args.max_iters);
    // Sanity check: Reference must be index 0 so we can run it first and
    // capture the oracle value table.
    assert_eq!(registry[0].boxed.name(), "reference");

    let mut rows: Vec<CaseRow> = Vec::new();

    for &size in &args.sizes {
        for type_str in &args.types {
            let case_label = format!("{size}x{size}_{type_str}");

            // Build base context once per case. parse_map_type is cheap and
            // MapType is consumed by build_context, so we re-parse here.
            let map_type = parse_map_type(type_str).expect("validated above");
            let base = build_context(size, size, map_type, pick_transition_mode(args.smoke));

            // Run Reference first to capture the oracle value table.
            // `Solver::run` dispatches to the parallel path under
            // --features parallel; that path's converged value is bit-equal
            // to the serial path at the fixed point, so it's still a valid
            // oracle.
            let ref_entry = &registry[0];
            let mut ref_ctx: VIContext = base.clone_value();
            let t0 = Instant::now();
            let ref_stats = ref_entry.boxed.run(&mut ref_ctx, ref_entry.budget);
            let ref_ms = t0.elapsed().as_secs_f64() * 1000.0;
            let ref_values = ref_ctx.value.clone();

            // Default mode for the dispatched run: "parallel" iff the
            // compile-time feature is on AND this solver has a parallel
            // path; else "serial".
            let ref_mode = if HAS_PARALLEL && is_parallel_capable("reference") {
                "parallel"
            } else {
                "serial"
            };
            eprintln!(
                "  size={size} type={type_str} solver=reference mode={ref_mode} iters={} updates={} ms={:.2} mismatch=0",
                ref_stats.iters_or_sweeps, ref_stats.updates, ref_ms,
            );
            rows.push(CaseRow {
                case_label: case_label.clone(),
                solver: "reference",
                mode: ref_mode,
                iters_or_sweeps: ref_stats.iters_or_sweeps,
                updates: ref_stats.updates,
                total_ms: ref_ms,
                mismatch: 0,
                converged: ref_stats.converged,
            });

            // If --parallel and Reference is parallel-capable, also emit the
            // explicit serial row for direct comparison.
            if want_dual_rows && is_parallel_capable("reference") {
                rows.push(run_serial_for(
                    "reference",
                    ref_entry.budget,
                    &base,
                    &ref_values,
                    &case_label,
                ));
            }

            // Run remaining solvers, mismatch-compare each against Reference.
            for entry in registry.iter().skip(1) {
                let solver_name = entry.boxed.name();
                let mut ctx = base.clone_value();
                let t0 = Instant::now();
                let stats = entry.boxed.run(&mut ctx, entry.budget);
                let ms = t0.elapsed().as_secs_f64() * 1000.0;
                let mismatch = value_mismatch(&ctx.value, &ref_values);

                let mode = if HAS_PARALLEL && is_parallel_capable(solver_name) {
                    "parallel"
                } else {
                    "serial"
                };
                eprintln!(
                    "  size={size} type={type_str} solver={solver_name} mode={mode} iters={} updates={} ms={:.2} mismatch={}",
                    stats.iters_or_sweeps, stats.updates, ms, mismatch,
                );
                rows.push(CaseRow {
                    case_label: case_label.clone(),
                    solver: solver_name,
                    mode,
                    iters_or_sweeps: stats.iters_or_sweeps,
                    updates: stats.updates,
                    total_ms: ms,
                    mismatch,
                    converged: stats.converged,
                });

                // Dual-row for any other parallel-capable solver.
                if want_dual_rows && is_parallel_capable(solver_name) {
                    rows.push(run_serial_for(
                        solver_name,
                        entry.budget,
                        &base,
                        &ref_values,
                        &case_label,
                    ));
                }
            }
        }
    }

    // CSV output (always when --out is set, independent of --markdown).
    if let Some(out_path) = &args.out {
        if let Err(e) = write_csv(out_path, &rows) {
            eprintln!("error: failed to write CSV {}: {e}", out_path.display());
            return ExitCode::from(2);
        }
        eprintln!("wrote {} ({} rows)", out_path.display(), rows.len());
    }

    // Markdown output (stdout).
    if args.markdown {
        print_markdown(&rows);
    }

    // Exact-set correctness gate: any nonzero mismatch on an exact solver
    // fails the run. This considers BOTH serial and parallel rows of exact
    // solvers — a parallel-path regression should fail the gate even if the
    // serial row would have agreed.
    //
    // Skipped in smoke mode — with budget=1 several solvers (notably
    // PyramidSweep, whose internal coarse/refine schedule is not capped by
    // the outer budget) can finish before Reference's 1-sweep pass
    // propagates the goal far enough to agree. Smoke mode is a wiring
    // check, not a correctness gate.
    let mut any_exact_mismatch = false;
    for r in &rows {
        if r.mismatch > 0 && EXACT_SOLVERS.contains(&r.solver) {
            eprintln!(
                "WARNING: {} {} ({}) mismatch={}",
                r.case_label, r.solver, r.mode, r.mismatch,
            );
            any_exact_mismatch = true;
        }
    }

    if any_exact_mismatch && !args.smoke {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    }
}
