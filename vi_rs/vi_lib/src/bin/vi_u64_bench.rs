//! 比較ベンチ用ハーネス: vi_lib の u64 高速ソルバ群を、vi_compare パイプラインと
//! 同一の入力 (map_server 意味論の OccupancyGrid) ・ゴール・パラメータで走らせ、
//! `value_<solver>.npy` / `policy_<solver>.npy` (float64, 形状 (H, W, N_THETA)) と
//! `timing_<solver>.json` を出力する。本家 u64 モデルなので compare.py で ros1 と直接比較でき、
//! 厳密ソルバは bit-exact（RMSE 0）になるはず。
//!
//! 入力 occupancy は別途 Python (u64_bench.py) が ros1 bench_client / ref_bench と同一の
//! `to_occupancy` で生成した raw i8 (h*w, row-major) を渡す。vi_ref_bench と同型で、先頭に
//! `<solver>` 引数を追加し、末尾の `delta_threshold` を除いたもの。
//!
//! 使い方:
//!   vi_u64_bench <solver> <occ_raw> <width> <height> <resolution> <origin_x> <origin_y>
//!                <goal_x> <goal_y> <goal_yaw_deg> <theta_cell_num> <safety_radius>
//!                <safety_radius_penalty> <goal_margin_radius> <goal_margin_theta>
//!                <max_sweeps> <out_dir>
//!   <solver> は `U64Solver::from_name` が受理する名前 (reference / frontier3d / frontier2d /
//!   frontier2d_par / frontier_stack / block_refine / pyramid_sweep / stream_mimic /
//!   prio_ls / prio_lc など)。正典は `solvers::mod` の `from_name`。

#[path = "bench_common/mod.rs"]
mod bench_common;

use std::fs::File;
use std::io::Write;
use std::time::Instant;

use bench_common::{arg, default_actions, extract_value_policy, load_map, write_npy_f64};
use vi_lib::solvers::{solve, U64Solver};
use vi_lib::ValueIterator;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 17 {
        eprintln!(
            "usage: {} <solver> <occ_raw> <width> <height> <resolution> <origin_x> <origin_y> \
             <goal_x> <goal_y> <goal_yaw_deg> <theta_cell_num> <safety_radius> \
             <safety_radius_penalty> <goal_margin_radius> <goal_margin_theta> \
             <max_sweeps> <out_dir>",
            args.first().map(String::as_str).unwrap_or("vi_u64_bench")
        );
        std::process::exit(2);
    }

    let solver_name: String = arg(&args, 1, "solver");
    let solver = U64Solver::from_name(&solver_name)
        .unwrap_or_else(|| panic!("unknown solver '{solver_name}'"));
    let occ_raw: String = arg(&args, 2, "occ_raw");
    let width: i32 = arg(&args, 3, "width");
    let height: i32 = arg(&args, 4, "height");
    let resolution: f64 = arg(&args, 5, "resolution");
    let origin_x: f64 = arg(&args, 6, "origin_x");
    let origin_y: f64 = arg(&args, 7, "origin_y");
    let goal_x: f64 = arg(&args, 8, "goal_x");
    let goal_y: f64 = arg(&args, 9, "goal_y");
    let goal_yaw_deg: f64 = arg(&args, 10, "goal_yaw_deg");
    let theta_cell_num: i32 = arg(&args, 11, "theta_cell_num");
    let safety_radius: f64 = arg(&args, 12, "safety_radius");
    let safety_radius_penalty: f64 = arg(&args, 13, "safety_radius_penalty");
    let goal_margin_radius: f64 = arg(&args, 14, "goal_margin_radius");
    let goal_margin_theta: i32 = arg(&args, 15, "goal_margin_theta");
    let max_sweeps: i32 = arg(&args, 16, "max_sweeps");
    let out_dir: String = arg(&args, 17, "out_dir");

    let map = load_map(&occ_raw, width, height, resolution, origin_x, origin_y);

    // 本家 executeVi: int t = (int)(yaw_rad*180/M_PI) = (int)goal_yaw_deg。
    let goal_t = goal_yaw_deg as i32;

    let mut vi = ValueIterator::new(default_actions(), 1);
    vi.set_map_with_occupancy_grid(
        &map,
        theta_cell_num,
        safety_radius,
        safety_radius_penalty,
        goal_margin_radius,
        goal_margin_theta,
    );
    vi.set_goal(goal_x, goal_y, goal_t);

    let t0 = Instant::now();
    let stats = solve(&mut vi, solver, max_sweeps as u32);
    let elapsed = t0.elapsed().as_secs_f64();

    let (value, policy, shape) = extract_value_policy(&vi);

    std::fs::create_dir_all(&out_dir).expect("mkdir out_dir");
    write_npy_f64(&format!("{out_dir}/value_{solver_name}.npy"), &shape, &value)
        .expect("write value");
    write_npy_f64(&format!("{out_dir}/policy_{solver_name}.npy"), &shape, &policy)
        .expect("write policy");

    let timing = format!(
        "{{\n  \"elapsed_sec\": {},\n  \"sweeps\": {},\n  \"iters\": {},\n  \"updates\": {},\n  \"converged\": {},\n  \"thread_num\": 1,\n  \"side\": \"{}\"\n}}\n",
        elapsed,
        stats.iters,
        stats.iters,
        stats.updates,
        if stats.converged { "true" } else { "false" },
        solver_name
    );
    File::create(format!("{out_dir}/timing_{solver_name}.json"))
        .and_then(|mut f| f.write_all(timing.as_bytes()))
        .expect("write timing");

    eprintln!(
        "[vi_u64_bench] solver={solver_name} iters={} updates={} converged={} elapsed={elapsed:.3}s shape={:?}",
        stats.iters, stats.updates, stats.converged, shape
    );
}
