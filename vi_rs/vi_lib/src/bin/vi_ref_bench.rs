//! 比較ベンチ用ハーネス: vi_lib (本家 u64 忠実移植) を、vi_compare パイプラインと
//! 同一の入力 (map_server 意味論の OccupancyGrid) ・ゴール・パラメータで走らせ、
//! `value_ref.npy` / `policy_ref.npy` (float64, 形状 (H=cell_num_y, W=cell_num_x, N_THETA)) と
//! `timing_ref.json` を出力する。ros1 (本家) と同じ数値モデルなので compare.py で ros1 と
//! 直接比較できる。
//!
//! 入力 occupancy は別途 Python (ref_bench.py) が ros1 bench_client と同一の `to_occupancy`
//! (occ_prob=(255-p)/255, free/occ 閾値, flipud) で生成した raw i8 (h*w, row-major) を渡す。
//!
//! 使い方 (位置引数, ref_bench.py が組み立てる):
//!   vi_ref_bench <occ_raw> <width> <height> <resolution> <origin_x> <origin_y>
//!                <goal_x> <goal_y> <goal_yaw_deg>
//!                <theta_cell_num> <safety_radius> <safety_radius_penalty>
//!                <goal_margin_radius> <goal_margin_theta>
//!                <max_sweeps> <delta_threshold> <out_dir>

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
    if args.len() < 18 {
        eprintln!(
            "usage: {} <occ_raw> <width> <height> <resolution> <origin_x> <origin_y> \
             <goal_x> <goal_y> <goal_yaw_deg> <theta_cell_num> <safety_radius> \
             <safety_radius_penalty> <goal_margin_radius> <goal_margin_theta> \
             <max_sweeps> <delta_threshold> <out_dir>",
            args.first().map(String::as_str).unwrap_or("vi_ref_bench")
        );
        std::process::exit(2);
    }

    let occ_raw: String = arg(&args, 1, "occ_raw");
    let width: i32 = arg(&args, 2, "width");
    let height: i32 = arg(&args, 3, "height");
    let resolution: f64 = arg(&args, 4, "resolution");
    let origin_x: f64 = arg(&args, 5, "origin_x");
    let origin_y: f64 = arg(&args, 6, "origin_y");
    let goal_x: f64 = arg(&args, 7, "goal_x");
    let goal_y: f64 = arg(&args, 8, "goal_y");
    let goal_yaw_deg: f64 = arg(&args, 9, "goal_yaw_deg");
    let theta_cell_num: i32 = arg(&args, 10, "theta_cell_num");
    let safety_radius: f64 = arg(&args, 11, "safety_radius");
    let safety_radius_penalty: f64 = arg(&args, 12, "safety_radius_penalty");
    let goal_margin_radius: f64 = arg(&args, 13, "goal_margin_radius");
    let goal_margin_theta: i32 = arg(&args, 14, "goal_margin_theta");
    let max_sweeps: i32 = arg(&args, 15, "max_sweeps");
    let delta_threshold: f64 = arg(&args, 16, "delta_threshold");
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

    // 収束ループ: 本家 bench_client と同じく、各スイープの reported delta (=max_delta>>18) を
    // 監視し、delta<=threshold で converged、max_sweeps で打ち切り。単スレッド order-0。
    // delta_threshold < 0 → strict モード: 到達可能セル (total_cost < REACH_THRESH) が
    // 1 スイープで全く変化しなくなる「真の固定点」まで回す (= `U64Solver::Reference`
    // = `solvers::original::original_solve_observed` の停止基準そのもの)。本家の soft 収束
    // (delta>>18==0) は確率的アクションのサブステップ精細化を収束後も残すため、
    // bit 一致比較には不向き。
    let strict = delta_threshold < 0.0;
    let t0 = Instant::now();
    let mut sweeps: i32 = 0;
    let converged;
    if strict {
        let stats = solve(&mut vi, U64Solver::Reference, max_sweeps.max(0) as u32);
        sweeps = stats.iters as i32;
        converged = stats.converged;
    } else {
        loop {
            vi.value_iteration_worker(1, 0);
            sweeps += 1;
            let delta = vi.thread_status.get(&0).map(|s| s.delta).unwrap_or(f64::INFINITY);
            if delta <= delta_threshold || sweeps >= max_sweeps {
                converged = delta <= delta_threshold;
                break;
            }
        }
    }
    let elapsed = t0.elapsed().as_secs_f64();

    let (value, policy, shape) = extract_value_policy(&vi);

    std::fs::create_dir_all(&out_dir).expect("mkdir out_dir");
    write_npy_f64(&format!("{out_dir}/value_ref.npy"), &shape, &value).expect("write value_ref");
    write_npy_f64(&format!("{out_dir}/policy_ref.npy"), &shape, &policy).expect("write policy_ref");

    // timing_ref.json (ros1/ros2 と同じスキーマ)
    let timing = format!(
        "{{\n  \"elapsed_sec\": {},\n  \"sweeps\": {},\n  \"converged\": {},\n  \"thread_num\": 1,\n  \"delta_threshold\": {},\n  \"side\": \"ref\"\n}}\n",
        elapsed,
        sweeps,
        if converged { "true" } else { "false" },
        delta_threshold
    );
    File::create(format!("{out_dir}/timing_ref.json"))
        .and_then(|mut f| f.write_all(timing.as_bytes()))
        .expect("write timing_ref");

    eprintln!(
        "[vi_ref_bench] sweeps={sweeps} converged={converged} elapsed={elapsed:.3}s shape={:?}",
        shape
    );
}
