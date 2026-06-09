//! 比較ベンチ用ハーネス: vi_rs の **Frontier3D** ソルバを、vi_compare パイプラインと
//! 同一の入力 (map_server 意味論の OccupancyGrid) ・ゴール・パラメータで走らせ、
//! `value_f3d.npy` / `policy_f3d.npy` (u16 / i16, 形状 (H, W, N_THETA)) と
//! `timing_f3d.json` を出力する。
//!
//! 設計意図:
//!   * `ref` (vi_reference) と同じく **ROS を経由しない直接ハーネス** にすることで、
//!     vi_node (ROS2 node) の feedback ポンプ・100ms ポーリング等のオーバーヘッドを排除し、
//!     ros1 (C++ 単スレッド) / ref (Rust 単スレッド) と公平に速度比較できる。
//!   * VIContext の構築は vi_node の `bridge` を **そのまま再利用** する。よって本ハーネスの
//!     16bit モデルは vi_node (solver=reference) が生成する `value_ros2.npy` と同一の数値モデル
//!     であり、Frontier3D の収束固定点は Reference のそれと bit 一致するはず (f3d ≡ ros2 のクロス
//!     チェックが成立する)。
//!   * `Frontier3D::run_serial` を呼ぶため、crate が `--features parallel` でビルドされていても
//!     必ず単スレッドの Gauss-Seidel フロンティア反復になる (thread_num=1 の公平比較)。
//!
//! NB: vi_node 経由で frontier 系を走らせると、node の収束判定が `final_delta == 0` 固定で
//! ある一方 frontier 系は仕様上常に final_delta=0 を返すため、最初の feedback tick で誤って
//! 「収束」と判定され数反復で打ち切られる (partial solve)。本ハーネスはその経路を使わない。
//!
//! 使い方 (位置引数, f3d_bench.py が組み立てる。vi_ref_bench と同一レイアウト):
//!   vi_f3d_bench <occ_raw> <width> <height> <resolution> <origin_x> <origin_y>
//!                <goal_x> <goal_y> <goal_yaw_deg>
//!                <theta_cell_num> <safety_radius> <safety_radius_penalty>
//!                <goal_margin_radius> <goal_margin_theta>
//!                <max_sweeps> <delta_threshold> <out_dir>

use std::fs::File;
use std::io::Write;
use std::time::Instant;

use ndarray::Array3;

use vi_algorithm::context::{MapDims, VIContext};
use vi_algorithm::{Budget, Frontier3D};
use vi_core::{make_goal_mask, MAX_VALUE, N_THETA};
use vi_fixtures::{generate_transitions, TransitionMode};

use vi_node::bridge::{
    occupancy_to_penalty, pose_to_goal_spec, OccupancyGridView, PenaltyParams, PoseView,
};
use vi_node::npy::{write_i16, write_u16};
use vi_node::sweep_thread::compute_policy;

fn arg<T: std::str::FromStr>(args: &[String], i: usize, name: &str) -> T
where
    <T as std::str::FromStr>::Err: std::fmt::Display,
{
    args.get(i)
        .unwrap_or_else(|| panic!("missing arg {i} ({name})"))
        .parse::<T>()
        .unwrap_or_else(|e| panic!("bad arg {i} ({name}): {e}"))
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 18 {
        eprintln!(
            "usage: {} <occ_raw> <width> <height> <resolution> <origin_x> <origin_y> \
             <goal_x> <goal_y> <goal_yaw_deg> <theta_cell_num> <safety_radius> \
             <safety_radius_penalty> <goal_margin_radius> <goal_margin_theta> \
             <max_sweeps> <delta_threshold> <out_dir>",
            args.first().map(String::as_str).unwrap_or("vi_f3d_bench")
        );
        std::process::exit(2);
    }

    let occ_raw: String = arg(&args, 1, "occ_raw");
    let width: u32 = arg(&args, 2, "width");
    let height: u32 = arg(&args, 3, "height");
    let resolution: f64 = arg(&args, 4, "resolution");
    let origin_x: f64 = arg(&args, 5, "origin_x");
    let origin_y: f64 = arg(&args, 6, "origin_y");
    let goal_x: f64 = arg(&args, 7, "goal_x");
    let goal_y: f64 = arg(&args, 8, "goal_y");
    let goal_yaw_deg: f64 = arg(&args, 9, "goal_yaw_deg");
    let _theta_cell_num: i32 = arg(&args, 10, "theta_cell_num"); // 固定 N_THETA=60 (vi_core 契約)
    let safety_radius: f64 = arg(&args, 11, "safety_radius");
    let safety_radius_penalty: f64 = arg(&args, 12, "safety_radius_penalty");
    let goal_margin_radius: f64 = arg(&args, 13, "goal_margin_radius");
    let goal_margin_theta: f64 = arg(&args, 14, "goal_margin_theta");
    let max_sweeps: u32 = arg(&args, 15, "max_sweeps");
    let delta_threshold: f64 = arg(&args, 16, "delta_threshold");
    let out_dir: String = arg(&args, 17, "out_dir");

    assert_eq!(_theta_cell_num as usize, N_THETA, "theta_cell_num は vi_core::N_THETA と一致が必要");

    // occupancy (raw i8, len=width*height, row-major, ros2 bench_client の to_occupancy と同一)
    let raw = std::fs::read(&occ_raw).expect("read occ_raw");
    let n = (width as usize) * (height as usize);
    assert_eq!(raw.len(), n, "occ_raw size {} != width*height {}", raw.len(), n);
    let data: Vec<i8> = raw.iter().map(|&b| b as i8).collect();

    let grid = OccupancyGridView {
        width,
        height,
        resolution,
        origin_x,
        origin_y,
        data: &data[..],
    };

    // ── VIContext 構築: vi_node main.rs と同一手順 (bridge を再利用) ───────────────
    let pen_params = PenaltyParams {
        safety_radius_m: safety_radius,
        safety_radius_penalty: safety_radius_penalty as u16,
        unknown_as_obstacle: true,
    };
    let penalty = occupancy_to_penalty(&grid, &pen_params);

    let trans = generate_transitions(TransitionMode::PaperMonteCarlo {
        xy_resolution: resolution,
    });

    let pose = PoseView {
        x: goal_x,
        y: goal_y,
        yaw_rad: goal_yaw_deg.to_radians(),
    };
    let goal_spec = pose_to_goal_spec(&pose, &grid, goal_margin_radius, goal_margin_theta);
    let goal_mask = make_goal_mask(width, height, &goal_spec);

    let h = height as usize;
    let w = width as usize;
    let mut value = Array3::<u16>::from_elem((h, w, N_THETA), MAX_VALUE);
    for ((iy, ix, it), &is_goal) in goal_mask.indexed_iter() {
        if is_goal {
            value[[iy, ix, it]] = 0;
        }
    }

    let mut ctx = VIContext {
        dims: MapDims { map_x: width, map_y: height },
        value,
        penalty,
        goal_mask,
        transitions: trans.unpack(),
    };

    // ── 求解: Frontier3D 単スレッド Gauss-Seidel をフロンティアが空になる固定点まで ─────
    // Budget::Iterations(max_sweeps) は上限。Frontier3D はフロンティアが空になった時点で
    // converged=true を返し、それより前に必ず収束する。final_delta は仕様上常に 0。
    let t0 = Instant::now();
    let stats = Frontier3D.run_serial(&mut ctx, Budget::Iterations(max_sweeps));
    let elapsed = t0.elapsed().as_secs_f64();

    let policy = compute_policy(&ctx);

    std::fs::create_dir_all(&out_dir).expect("mkdir out_dir");
    write_u16(&format!("{out_dir}/value_f3d.npy"), &ctx.value).expect("write value_f3d");
    write_i16(&format!("{out_dir}/policy_f3d.npy"), &policy).expect("write policy_f3d");

    // timing_f3d.json (ros1/ros2/ref と同じスキーマ)。"sweeps" は Frontier3D の反復回数
    // (フロンティア拡張ラウンド数) で、ros1/ref の全グリッド sweep とは単位が異なる点に注意。
    let timing = format!(
        "{{\n  \"elapsed_sec\": {},\n  \"sweeps\": {},\n  \"updates\": {},\n  \"converged\": {},\n  \"thread_num\": 1,\n  \"delta_threshold\": {},\n  \"side\": \"f3d\"\n}}\n",
        elapsed,
        stats.iters_or_sweeps,
        stats.updates,
        if stats.converged { "true" } else { "false" },
        delta_threshold
    );
    File::create(format!("{out_dir}/timing_f3d.json"))
        .and_then(|mut f| f.write_all(timing.as_bytes()))
        .expect("write timing_f3d");

    eprintln!(
        "[vi_f3d_bench] iters={} updates={} converged={} elapsed={:.3}s shape=[{}, {}, {}]",
        stats.iters_or_sweeps, stats.updates, stats.converged, elapsed, h, w, N_THETA
    );
}
