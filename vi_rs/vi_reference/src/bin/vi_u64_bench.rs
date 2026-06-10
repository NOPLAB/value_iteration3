//! 比較ベンチ用ハーネス: vi_reference の u64 高速ソルバ群を、vi_compare パイプラインと
//! 同一の入力 (map_server 意味論の OccupancyGrid) ・ゴール・パラメータで走らせ、
//! `value_<solver>.npy` / `policy_<solver>.npy` (float64, 形状 (H, W, N_THETA)) と
//! `timing_<solver>.json` を出力する。本家 u64 モデルなので compare.py で ros1 と直接比較でき、
//! 厳密ソルバは bit-exact（RMSE 0）になるはず。
//!
//! 入力 occupancy は別途 Python (u64_bench.py) が ros2 bench_client / ref_bench と同一の
//! `to_occupancy` で生成した raw i8 (h*w, row-major) を渡す。vi_ref_bench と同型で、先頭に
//! `<solver>` 引数を追加し、末尾の `delta_threshold` を除いたもの。
//!
//! 使い方:
//!   vi_u64_bench <solver> <occ_raw> <width> <height> <resolution> <origin_x> <origin_y>
//!                <goal_x> <goal_y> <goal_yaw_deg> <theta_cell_num> <safety_radius>
//!                <safety_radius_penalty> <goal_margin_radius> <goal_margin_theta>
//!                <max_sweeps> <out_dir>
//!   <solver> は `U64Solver::from_name` が受理する名前 (reference / frontier3d / frontier2d /
//!   frontier2d_soa / frontier2d_pad / frontier2d_par / frontier_stack / block_refine /
//!   pyramid_sweep / frontier3d_tau / frontier3d_topk / frontier3d_coarse_theta / stream_mimic /
//!   prio_ls / prio_lc)。正典は `solvers::mod` の `from_name`。

use std::fs::File;
use std::io::{self, Write};
use std::time::Instant;

use vi_reference::params::PROB_BASE;
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::{Action, OccupancyGrid, Quaternion, ValueIterator};

fn arg<T: std::str::FromStr>(args: &[String], i: usize, name: &str) -> T
where
    <T as std::str::FromStr>::Err: std::fmt::Display,
{
    args.get(i)
        .unwrap_or_else(|| panic!("missing arg {i} ({name})"))
        .parse::<T>()
        .unwrap_or_else(|e| panic!("bad arg {i} ({name}): {e}"))
}

/// 最小の `.npy` ライタ (float64 '<f8', C-order)。numpy が np.load で読める。
fn write_npy_f64(path: &str, shape: &[usize], data: &[f64]) -> io::Result<()> {
    let shape_str = shape.iter().map(|d| d.to_string()).collect::<Vec<_>>().join(", ");
    let dict = format!(
        "{{'descr': '<f8', 'fortran_order': False, 'shape': ({},), }}",
        shape_str
    );
    let prefix = 10usize; // magic(6) + version(2) + header_len(2)
    let mut header = dict;
    let unpadded = prefix + header.len() + 1; // +1 for trailing '\n'
    let pad = (64 - (unpadded % 64)) % 64;
    for _ in 0..pad {
        header.push(' ');
    }
    header.push('\n');
    let hlen = header.len() as u16;
    let mut f = File::create(path)?;
    f.write_all(b"\x93NUMPY")?;
    f.write_all(&[0x01, 0x00])?;
    f.write_all(&hlen.to_le_bytes())?;
    f.write_all(header.as_bytes())?;
    let mut bytes = Vec::with_capacity(data.len() * 8);
    for &v in data {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    f.write_all(&bytes)
}

fn default_actions() -> Vec<Action> {
    // vi_compare の正典 6 アクション (本家 launch と ID 順まで一致)。
    vec![
        Action::new("forward", 0.3, 0.0, 0),
        Action::new("back", -0.2, 0.0, 1),
        Action::new("right", 0.0, -20.0, 2),
        Action::new("rightfw", 0.2, -20.0, 3),
        Action::new("left", 0.0, 20.0, 4),
        Action::new("leftfw", 0.2, 20.0, 5),
    ]
}

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

    let raw = std::fs::read(&occ_raw).expect("read occ_raw");
    let n = (width as usize) * (height as usize);
    assert_eq!(raw.len(), n, "occ_raw size {} != width*height {}", raw.len(), n);
    let data: Vec<i8> = raw.iter().map(|&b| b as i8).collect();

    let map = OccupancyGrid {
        width,
        height,
        resolution,
        origin_x,
        origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data,
    };

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

    // 価値・方策を (H=cell_num_y, W=cell_num_x, theta) C-order で取り出す (vi_ref_bench と同一)。
    let nx = vi.cell_num_x;
    let ny = vi.cell_num_y;
    let nt = vi.cell_num_t;
    let mut value = Vec::with_capacity((nx * ny * nt) as usize);
    let mut policy = Vec::with_capacity((nx * ny * nt) as usize);
    for iy in 0..ny {
        for ix in 0..nx {
            for it in 0..nt {
                let s = &vi.states[vi.to_index(ix, iy, it) as usize];
                // 本家 valueFunctionWriter は total_cost/prob_base を整数除算。
                value.push((s.total_cost / PROB_BASE) as f64);
                let pol = match s.optimal_action {
                    Some(ai) => vi.actions[ai].id as f64,
                    None => -1.0,
                };
                policy.push(pol);
            }
        }
    }

    let shape = [ny as usize, nx as usize, nt as usize];
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
