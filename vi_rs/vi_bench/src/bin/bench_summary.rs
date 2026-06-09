//! `bench_summary` — u64 (本家忠実) ソルバ群の wall-clock + 正当性比較。
//!
//! `vi_matlab/workflows/benchmarks/benchmark_vi.m` を範に取る。各
//! `(map_size, map_type)` ケースについて、同一の合成マップから全ソルバを走らせ、
//! `(iters, updates, total_ms, converged, mismatch)` を Reference（=本家全走査の
//! 固定点）に対して記録し、Markdown 表（+任意の CSV）を出力する。
//!
//! u64 モデルでは到達可能セルの収束値が更新順に依存しないため、全ソルバ
//! （Frontier/Block/Pyramid/Stream/近似 no-op）が Reference と bit-exact になる。
//! mismatch は到達可能セル（Reference の `total_cost < REACH`）での `total_cost`
//! 不一致数で、厳密性ゲートに使う。
//!
//! `--smoke` は単一 8×8 Empty・budget=1 に潰し、CI でワイヤリングだけ検証する
//! （budget=1 では収束しないのでゲートはスキップ）。
//!
//! `--parallel`: u64 ソルバは現状シリアルのみ。フラグは受け付けるが no-op
//! （`make rs-bench-parallel` 互換のため残す）。

use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Instant;

use clap::Parser;
use vi_bench::fixtures::{build_vi, BenchMap};
use vi_reference::params::PROB_BASE;
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::ValueIterator;

/// 到達可能とみなす total_cost 上限（compare.py の value>=1e6 境界と整合）。
const REACH: u64 = 1_000_000u64 * PROB_BASE;

/// 全ソルバが Reference と bit-exact であるべき（no-op パラメータ）。
/// いずれかで mismatch>0 なら非ゼロ終了。
const EXACT_SOLVERS: &[&str] = &[
    "reference",
    "frontier2d",
    "frontier3d",
    "frontier_stack",
    "block_refine",
    "pyramid_sweep",
    "frontier3d_tau",
    "frontier3d_topk",
    "frontier3d_coarse_theta",
    "stream_mimic",
];

#[derive(Parser)]
#[command(
    about = "Run every u64 VI solver across (map_size × map_type) and emit a comparison table."
)]
struct Args {
    /// Comma-separated map sizes (square).
    #[arg(long, value_delimiter = ',', default_value = "8,16,32,64")]
    sizes: Vec<u32>,

    /// Comma-separated map types: empty,obstacle,sentinel,random.
    #[arg(long, value_delimiter = ',', default_value = "empty,obstacle,sentinel,random")]
    types: Vec<String>,

    /// Sweep budget cap for Reference / BlockRefine / PyramidSweep / StreamMimic.
    #[arg(long, default_value_t = 200)]
    max_sweeps: u32,

    /// Iteration budget cap for the frontier-family solvers.
    #[arg(long, default_value_t = 4000)]
    max_iters: u32,

    /// CSV output path. Created (and parent dirs) if missing.
    #[arg(long)]
    out: Option<PathBuf>,

    /// Print Markdown table to stdout in addition to per-case progress lines.
    #[arg(long, default_value_t = false)]
    markdown: bool,

    /// CI smoke mode: override sizes to [8], types to [empty], budgets to 1.
    #[arg(long, default_value_t = false)]
    smoke: bool,

    /// Accepted for `make rs-bench-parallel` compatibility. u64 solvers are
    /// serial-only, so this is a no-op (a note is printed when set).
    #[arg(long, default_value_t = false)]
    parallel: bool,
}

/// A single (case, solver) measurement.
struct CaseRow {
    case_label: String,
    solver: &'static str,
    iters: u32,
    updates: u64,
    total_ms: f64,
    converged: bool,
    mismatch: u64,
}

/// Solver registry. Reference MUST be first so it produces the oracle table.
/// 近似ソルバは no-op パラメータ（tau=0 / k=全 outcome / step=1）で Frontier3D
/// 等価 → Reference と bit-exact。
fn registry() -> Vec<(&'static str, U64Solver)> {
    use U64Solver::*;
    vec![
        ("reference", Reference),
        ("frontier2d", Frontier2D),
        ("frontier3d", Frontier3D),
        ("frontier_stack", FrontierStack),
        ("block_refine", BlockRefine),
        ("pyramid_sweep", PyramidSweep),
        ("frontier3d_tau", Frontier3DTau { tau: 0 }),
        ("frontier3d_topk", Frontier3DTopK { k: u32::MAX }),
        ("frontier3d_coarse_theta", Frontier3DCoarseTheta { step: 1 }),
        ("stream_mimic", StreamMimic),
    ]
}

/// Sweep-based solvers take `max_sweeps`; frontier-family take `max_iters`.
fn budget_for(name: &str, max_sweeps: u32, max_iters: u32) -> u32 {
    match name {
        "reference" | "block_refine" | "pyramid_sweep" | "stream_mimic" => max_sweeps,
        _ => max_iters,
    }
}

fn parse_map_type(s: &str) -> Result<BenchMap, String> {
    BenchMap::from_name(s).ok_or_else(|| format!("unknown map type: {s}"))
}

/// Count cells where the reference cost is reachable but the solver disagrees.
fn value_mismatch(ref_costs: &[u64], vi: &ValueIterator) -> u64 {
    vi.states
        .iter()
        .enumerate()
        .filter(|(i, s)| ref_costs[*i] < REACH && s.total_cost != ref_costs[*i])
        .count() as u64
}

fn print_markdown(rows: &[CaseRow]) {
    println!("| case | solver | iters | updates | total_ms | converged | mismatch |");
    println!("|------|--------|-------|---------|----------|-----------|----------|");
    for r in rows {
        println!(
            "| {} | {} | {} | {} | {:.3} | {} | {} |",
            r.case_label,
            r.solver,
            r.iters,
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
    writeln!(f, "case,solver,iters,updates,total_ms,converged,mismatch")?;
    for r in rows {
        writeln!(
            f,
            "{},{},{},{},{:.3},{},{}",
            r.case_label,
            r.solver,
            r.iters,
            r.updates,
            r.total_ms,
            if r.converged { "Y" } else { "N" },
            r.mismatch,
        )?;
    }
    Ok(())
}

fn main() -> ExitCode {
    let mut args = Args::parse();

    if args.parallel {
        eprintln!("note: --parallel is a no-op for u64 solvers (serial-only); proceeding serial");
    }

    if args.smoke {
        args.sizes = vec![8];
        args.types = vec!["empty".to_string()];
        args.max_sweeps = 1;
        args.max_iters = 1;
    }

    // Validate map-type labels up front so a typo fails fast.
    for s in &args.types {
        if let Err(e) = parse_map_type(s) {
            eprintln!("error: {e}");
            return ExitCode::from(2);
        }
    }

    let reg = registry();
    assert_eq!(reg[0].0, "reference", "Reference must be index 0 (oracle)");

    let mut rows: Vec<CaseRow> = Vec::new();

    for &size in &args.sizes {
        for type_str in &args.types {
            let case_label = format!("{size}x{size}_{type_str}");
            let map = parse_map_type(type_str).expect("validated above");

            // Oracle: Reference に対して到達可能セルの total_cost を控える。
            let mut ref_vi = build_vi(size, map);
            let t0 = Instant::now();
            let ref_stats = solve(&mut ref_vi, U64Solver::Reference, args.max_sweeps);
            let ref_ms = t0.elapsed().as_secs_f64() * 1000.0;
            let ref_costs: Vec<u64> = ref_vi.states.iter().map(|s| s.total_cost).collect();
            drop(ref_vi);

            eprintln!(
                "  {case_label} solver=reference iters={} updates={} ms={:.2} mismatch=0",
                ref_stats.iters, ref_stats.updates, ref_ms,
            );
            rows.push(CaseRow {
                case_label: case_label.clone(),
                solver: "reference",
                iters: ref_stats.iters,
                updates: ref_stats.updates,
                total_ms: ref_ms,
                converged: ref_stats.converged,
                mismatch: 0,
            });

            // 残りのソルバ。毎回フレッシュな ValueIterator を構築して走らせる。
            for &(name, solver) in reg.iter().skip(1) {
                let budget = budget_for(name, args.max_sweeps, args.max_iters);
                let mut vi = build_vi(size, map);
                let t0 = Instant::now();
                let stats = solve(&mut vi, solver, budget);
                let ms = t0.elapsed().as_secs_f64() * 1000.0;
                let mismatch = value_mismatch(&ref_costs, &vi);

                eprintln!(
                    "  {case_label} solver={name} iters={} updates={} ms={:.2} mismatch={mismatch}",
                    stats.iters, stats.updates, ms,
                );
                rows.push(CaseRow {
                    case_label: case_label.clone(),
                    solver: name,
                    iters: stats.iters,
                    updates: stats.updates,
                    total_ms: ms,
                    converged: stats.converged,
                    mismatch,
                });
            }
        }
    }

    if let Some(out_path) = &args.out {
        if let Err(e) = write_csv(out_path, &rows) {
            eprintln!("error: failed to write CSV {}: {e}", out_path.display());
            return ExitCode::from(2);
        }
        eprintln!("wrote {} ({} rows)", out_path.display(), rows.len());
    }

    if args.markdown {
        print_markdown(&rows);
    }

    // 厳密性ゲート: smoke 以外で、bit-exact 期待のソルバに mismatch があれば失敗。
    let mut any_exact_mismatch = false;
    for r in &rows {
        if r.mismatch > 0 && EXACT_SOLVERS.contains(&r.solver) {
            eprintln!("WARNING: {} {} mismatch={}", r.case_label, r.solver, r.mismatch);
            any_exact_mismatch = true;
        }
    }

    if any_exact_mismatch && !args.smoke {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    }
}
