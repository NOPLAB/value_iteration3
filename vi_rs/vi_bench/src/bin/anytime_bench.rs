//! `anytime_bench` — anytime プロファイル（経過時間 vs 解の正しさ）の実測。
//!
//! 本家 (`value_iteration_worker` = 全状態スイープ) と提案 (`frontier2d_sparse`)
//! を、**提案側の厳密固定点 V\* を真値として**突き合わせる。両者は到達可能セルで
//! bit-exact なので V\* は信頼できる ground truth になる。本家自身の停止基準
//! (1 スイープの最大変化 ΔV < 0.1 s) では測れない「実際の最適解からのずれ」を、
//! この harness だけが測れる。
//!
//! 各サンプル時刻で記録するもの:
//!   - `err_*`      : |V_t − V\*| の最大/平均 [s]（到達可能な自由状態のみ）
//!   - `frac_exact` : V_t == V\* の割合（= 確定済みの割合）
//!   - `frac_0p1`   : |V_t − V\*| < 0.1 s の割合（本家の停止しきい値と同じ尺度）
//!   - `reach_rate` : サンプル開始姿勢から貪欲方策のロールアウトがゴールに届く割合
//!   - `regret`     : 到達した経路の実コスト − V\*(start) の平均 [s]
//!   - `max_delta`  : 直前スイープの最大変化 [s]（本家の停止基準そのもの）
//!
//! 計時はソルバ時間のみ（計測処理と ValueIterator の構築は除外）。
//!
//! 使い方 (house, 論文 §4.4 と同じゴール):
//! ```text
//! anytime_bench --map ../value_iteration/maps/house.yaml \
//!   --goal-x 6.0 --goal-y -2.0 --goal-theta-deg 90 \
//!   --goal-radius-m 0.30 --goal-margin-theta-deg 15 \
//!   --safety-radius-m 0.20 --safety-penalty 100000 \
//!   --out results/anytime_house.csv
//! ```

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use clap::Parser;

use vi_bench::fixtures::canonical_actions;
use vi_bench::pgm::{self, Occupancy, PgmMap};
use vi_reference::params::PROB_BASE;
use vi_reference::planner::{rollout_path, RolloutStatus};
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::{OccupancyGrid, Quaternion, ValueIterator};

const THETA_CELL_NUM: i32 = 60;
/// 到達可能とみなす total_cost 上限（bench_map / compare.py と同一境界）。
const REACH: u64 = 1_000_000u64 * PROB_BASE;
/// ロールアウトの打ち切り歩数（1 歩 = 1 s）。
const MAX_ROLLOUT_STEPS: usize = 20_000;

#[derive(Parser)]
#[command(about = "Anytime profile (error vs wall-clock) of reference sweeps against the active-set solver.")]
struct Args {
    /// 地図 YAML（`image:` は YAML のディレクトリ基準で解決）。
    #[arg(long)]
    map: PathBuf,

    #[arg(long)]
    goal_x: f64,
    #[arg(long)]
    goal_y: f64,
    #[arg(long, default_value_t = 90.0)]
    goal_theta_deg: f64,
    #[arg(long, default_value_t = 0.30)]
    goal_radius_m: f64,
    #[arg(long, default_value_t = 15)]
    goal_margin_theta_deg: i32,
    #[arg(long, default_value_t = 0.20)]
    safety_radius_m: f64,
    #[arg(long, default_value_t = 100000.0)]
    safety_penalty: f64,

    /// 提案ソルバのスレッド数（VI_THREADS）。本家側は常に 1（競合の非決定性を排除）。
    #[arg(long, default_value_t = 1)]
    threads: usize,

    /// ロールアウトを打つ開始姿勢のサンプル数。
    #[arg(long, default_value_t = 64)]
    samples: usize,

    /// 本家側スイープ数の上限。
    #[arg(long, default_value_t = 4000)]
    max_sweeps: u32,

    /// 本家側の打ち切り時間 [s]（ソルバ時間の累計）。
    #[arg(long, default_value_t = 900.0)]
    time_budget_s: f64,

    /// 出力 CSV。
    #[arg(long)]
    out: PathBuf,
}

/// ある時刻における解の質。
struct Metrics {
    err_max: f64,
    err_mean: f64,
    frac_exact: f64,
    frac_0p1: f64,
    frac_finite: f64,
    frac_policy: f64,
    /// 最適方策と行動が一致する自由到達状態の割合。「経路が引けるが正しくない」を
    /// penalty 構造に汚されずに測れる唯一の指標（歩数超過は、最適経路が安全余裕を
    /// 迂回して歩数が増えるぶん、危険域を突っ切る誤方策のほうが短くなり負になりうる）。
    policy_agree: f64,
    reach_rate: f64,
    excess_mean: f64,
    n_reached: usize,
}

/// 開始姿勢サンプル（世界座標 + 最適方策でのロールアウト歩数）。
///
/// 遅れの指標に「実コスト − V\*」を使わないのは、安全半径ペナルティ (R_max = 10^5 s) を
/// 1 歩でも踏むと差が 10^5 のオーダーになり、方策の良し悪しが読めなくなるため。
/// 代わりに **最適方策のロールアウト歩数に対する超過率** を使う（1 歩 = Δt = 1 s）。
struct Sample {
    x: f64,
    y: f64,
    yaw_rad: f64,
    opt_steps: usize,
}

fn build_occupancy(map: &PgmMap) -> (Vec<i8>, i32, i32) {
    let (w, h) = (map.width, map.height);
    let mut occ = vec![0i8; w * h];
    for iy in 0..h {
        let src_row = h - 1 - iy; // PGM は上下反転（world y は下から上）
        for ix in 0..w {
            let pixel = map.pixels[src_row * w + ix];
            let c = pgm::classify(pixel, map.negate, map.occupied_thresh, map.free_thresh);
            // 未知セルは障害物扱い（論文条件）。
            let blocked = matches!(c, Occupancy::Obstacle | Occupancy::Unknown);
            occ[iy * w + ix] = if blocked { 100 } else { 0 };
        }
    }
    (occ, w as i32, h as i32)
}

/// V\* から開始姿勢を決定的にサンプルする（再現性のため RNG ではなく等間隔ストライド）。
fn pick_samples(vi: &ValueIterator, vstar: &[u64], n: usize) -> Vec<Sample> {
    let mut cand: Vec<usize> = Vec::new();
    for (i, s) in vi.states.iter().enumerate() {
        // 方策を辿る意味があるのは「自由・到達可能・ゴール圏外」の状態だけ。
        if s.free && !s.final_state && vstar[i] < REACH {
            cand.push(i);
        }
    }
    if cand.is_empty() {
        return Vec::new();
    }
    // 遠い姿勢ほど方策の誤りが効くので、V\* の降順に並べてから等間隔に拾う
    // （近距離だけを引いて "簡単すぎる" サンプルになるのを避ける）。
    cand.sort_unstable_by(|&a, &b| vstar[b].cmp(&vstar[a]));
    let n = n.min(cand.len());
    let stride = cand.len() / n;
    let mut out = Vec::with_capacity(n);
    for k in 0..n {
        let i = cand[k * stride];
        let s = &vi.states[i];
        let (x, y) = (
            (s.ix as f64 + 0.5) * vi.xy_resolution + vi.map_origin_x,
            (s.iy as f64 + 0.5) * vi.xy_resolution + vi.map_origin_y,
        );
        let yaw_rad = (s.it as f64 * vi.t_resolution).to_radians();
        // 収束済み `vi` (= V*) の方策で辿った歩数が基準値。辿れない開始点は捨てる。
        let r = rollout_path(vi, x, y, yaw_rad, MAX_ROLLOUT_STEPS, 0);
        if let (RolloutStatus::ReachedGoal, steps) = (r.status, rollout_steps(&r.poses, true)) {
            if steps > 0 {
                out.push(Sample { x, y, yaw_rad, opt_steps: steps });
            }
        }
    }
    out
}

/// ロールアウトの歩数（1 歩 = 1 アクション = Δt = 1 s）。
/// 先頭は開始姿勢なので除外。到達時は末尾に厳密なゴール姿勢が足されているのでこれも除外。
fn rollout_steps(poses: &[vi_reference::planner::PathPose], reached: bool) -> usize {
    let end = if reached { poses.len().saturating_sub(1) } else { poses.len() };
    end.saturating_sub(1)
}

fn measure(
    vi: &ValueIterator,
    vstar: &[u64],
    pstar: &[Option<usize>],
    samples: &[Sample],
) -> Metrics {
    let mut n_agree = 0u64;
    let (mut n, mut n_exact, mut n_0p1, mut n_finite) = (0u64, 0u64, 0u64, 0u64);
    let mut n_policy = 0u64;
    let (mut err_max, mut err_sum) = (0.0f64, 0.0f64);
    for (i, s) in vi.states.iter().enumerate() {
        if !s.free || vstar[i] >= REACH {
            continue;
        }
        n += 1;
        // 「行動が引ける」状態の割合。本家は値が未確定でも optimal_action を持ちうるので、
        // これが早期に 1 へ張り付く一方 frac_exact が伸びない、という対比が見える。
        if s.optimal_action.is_some() || s.final_state {
            n_policy += 1;
        }
        if s.final_state || s.optimal_action == pstar[i] {
            n_agree += 1;
        }
        if s.total_cost >= REACH {
            continue; // 未到達: 有限誤差として数えない（frac_finite で見える）
        }
        n_finite += 1;
        let d = (s.total_cost as f64 - vstar[i] as f64).abs() / PROB_BASE as f64;
        if s.total_cost == vstar[i] {
            n_exact += 1;
        }
        if d < 0.1 {
            n_0p1 += 1;
        }
        if d > err_max {
            err_max = d;
        }
        err_sum += d;
    }
    let nf = n.max(1) as f64;

    let (mut reached, mut excess_sum) = (0usize, 0.0f64);
    for sp in samples {
        let r = rollout_path(vi, sp.x, sp.y, sp.yaw_rad, MAX_ROLLOUT_STEPS, 0);
        if matches!(r.status, RolloutStatus::ReachedGoal) {
            reached += 1;
            let steps = rollout_steps(&r.poses, true) as f64;
            excess_sum += steps / sp.opt_steps as f64 - 1.0;
        }
    }

    Metrics {
        err_max,
        err_mean: if n_finite > 0 { err_sum / n_finite as f64 } else { f64::NAN },
        frac_exact: n_exact as f64 / nf,
        frac_0p1: n_0p1 as f64 / nf,
        frac_finite: n_finite as f64 / nf,
        frac_policy: n_policy as f64 / nf,
        policy_agree: n_agree as f64 / nf,
        reach_rate: if samples.is_empty() { f64::NAN } else { reached as f64 / samples.len() as f64 },
        excess_mean: if reached > 0 { excess_sum / reached as f64 } else { f64::NAN },
        n_reached: reached,
    }
}

fn row(w: &mut impl std::io::Write, solver: &str, iter: u32, t: f64, md: f64, m: &Metrics) {
    let _ = writeln!(
        w,
        "{solver},{iter},{t:.4},{md:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.4},{:.4},{}",
        m.err_max, m.err_mean, m.frac_exact, m.frac_0p1, m.frac_finite, m.frac_policy,
        m.policy_agree, m.reach_rate, m.excess_mean, m.n_reached
    );
}

fn main() -> ExitCode {
    let args = Args::parse();
    std::env::set_var("VI_THREADS", args.threads.to_string());

    let map = match pgm::load(&args.map) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("error: failed to load map: {e}");
            return ExitCode::from(2);
        }
    };
    let (occ, ow, oh) = build_occupancy(&map);
    let free_cells = occ.iter().filter(|&&c| c == 0).count();
    eprintln!(
        "map {}x{} res {:.3} m, free cells {}, states {}",
        ow, oh, map.resolution, free_cells,
        (ow as u64) * (oh as u64) * THETA_CELL_NUM as u64
    );

    let grid = OccupancyGrid {
        width: ow,
        height: oh,
        resolution: map.resolution,
        origin_x: map.origin_x,
        origin_y: map.origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: occ,
    };
    let build = || -> ValueIterator {
        let mut vi = ValueIterator::new(canonical_actions(), 1);
        vi.set_map_with_occupancy_grid(
            &grid,
            THETA_CELL_NUM,
            args.safety_radius_m,
            args.safety_penalty,
            args.goal_radius_m,
            args.goal_margin_theta_deg,
        );
        vi.set_goal(args.goal_x, args.goal_y, args.goal_theta_deg as i32);
        vi
    };

    // --- ground truth: 提案ソルバの厳密固定点 ---
    eprintln!("solving ground truth (frontier2d_sparse, {} threads)...", args.threads);
    let mut vi_star = build();
    let t0 = Instant::now();
    let st = solve(&mut vi_star, U64Solver::Frontier2DSparse, u32::MAX);
    let t_star = t0.elapsed().as_secs_f64();
    let vstar: Vec<u64> = vi_star.states.iter().map(|s| s.total_cost).collect();
    let pstar: Vec<Option<usize>> = vi_star.states.iter().map(|s| s.optimal_action).collect();
    let reachable = vstar.iter().filter(|&&v| v < REACH).count();
    eprintln!(
        "  V* in {t_star:.3} s, iters={}, updates={}, converged={}, reachable states={reachable}",
        st.iters, st.updates, st.converged
    );

    let samples = pick_samples(&vi_star, &vstar, args.samples);
    eprintln!("  {} start poses sampled", samples.len());

    if let Some(p) = args.out.parent() {
        let _ = std::fs::create_dir_all(p);
    }
    let mut out = match std::fs::File::create(&args.out) {
        Ok(f) => std::io::BufWriter::new(f),
        Err(e) => {
            eprintln!("error: cannot write {}: {e}", args.out.display());
            return ExitCode::from(2);
        }
    };
    use std::io::Write;
    let _ = writeln!(
        out,
        "solver,iter,t_sec,max_delta_s,err_max_s,err_mean_s,frac_exact,frac_within_0p1s,\
         frac_finite,frac_policy,policy_agree,reach_rate,excess_mean,n_reached"
    );

    // --- 本家: 1 スイープずつ進めて毎回計測 ---
    eprintln!("baseline (reference full sweeps, 1 thread)...");
    let mut vi = build();
    let mut prev: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
    let mut acc = Duration::ZERO;
    for sweep in 1..=args.max_sweeps {
        let t = Instant::now();
        vi.value_iteration_worker(1, 0);
        acc += t.elapsed();

        // 本家の停止基準そのもの: 1 スイープの最大変化。
        let mut max_delta = 0u64;
        let mut changed = false;
        for (i, s) in vi.states.iter().enumerate() {
            if s.total_cost < REACH && s.total_cost != prev[i] {
                changed = true;
                let d = s.total_cost.abs_diff(prev[i]);
                if d > max_delta {
                    max_delta = d;
                }
            }
            prev[i] = s.total_cost;
        }
        let md = max_delta as f64 / PROB_BASE as f64;

        // 序盤は密に、以降は間引いて計測（計測自体は計時に含めない）。
        let sample_now = sweep <= 20 || sweep % 5 == 0 || !changed;
        if sample_now {
            let m = measure(&vi, &vstar, &pstar, &samples);
            row(&mut out, "reference", sweep, acc.as_secs_f64(), md, &m);
            let _ = out.flush();
            eprintln!(
                "  sweep {sweep:4} t={:7.2}s dV={md:12.3}s exact={:6.2}% policy={:6.2}% agree={:6.2}% reach={:5.1}%",
                acc.as_secs_f64(), m.frac_exact * 100.0,
                m.frac_policy * 100.0, m.policy_agree * 100.0, m.reach_rate * 100.0
            );
        }
        if !changed {
            eprintln!("  baseline reached the exact fixed point at sweep {sweep}");
            break;
        }
        if acc.as_secs_f64() > args.time_budget_s {
            eprintln!("  baseline hit the time budget at sweep {sweep}");
            break;
        }
    }

    // --- 提案: ラウンド数を打ち切って再実行し、各点で計測 ---
    // sparse は途中打ち切りでも部分状態を書き戻すので、k ラウンド実行後の値場が得られる。
    eprintln!("proposed (frontier2d_sparse, {} threads)...", args.threads);
    // ウォームアップ: 最初の 1 点だけアロケータ/ページフォルトの初期費用を被って
    // 曲線が非単調になるのを防ぐ（計測値には使わない）。
    {
        let mut vi = build();
        solve(&mut vi, U64Solver::Frontier2DSparse, u32::MAX);
    }
    let mut ks: Vec<u32> = Vec::new();
    let mut k = 1u32;
    while k < st.iters {
        ks.push(k);
        k = ((k as f64 * 1.45).ceil() as u32).max(k + 1);
    }
    ks.push(st.iters);
    for &k in &ks {
        let mut vi = build();
        let t = Instant::now();
        solve(&mut vi, U64Solver::Frontier2DSparse, k);
        let el = t.elapsed().as_secs_f64();
        let m = measure(&vi, &vstar, &pstar, &samples);
        row(&mut out, "sparse", k, el, f64::NAN, &m);
        let _ = out.flush();
        eprintln!(
            "  round {k:5} t={el:7.3}s exact={:6.2}% policy={:6.2}% agree={:6.2}% reach={:5.1}%",
            m.frac_exact * 100.0, m.frac_policy * 100.0, m.policy_agree * 100.0,
            m.reach_rate * 100.0
        );
    }

    eprintln!("wrote {}", args.out.display());
    ExitCode::SUCCESS
}
