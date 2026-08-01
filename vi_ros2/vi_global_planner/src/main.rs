//! vi_global_planner entry point — Nav2 の planner_server を置き換える
//! `compute_path_to_pose` action サーバ (全 Rust)。
//!
//! Boot order (vi_node と同型):
//!   1. `Context::default_from_env` + basic executor + node 作成
//!   2. パラメータ宣言・検証 (アクション定数は vi_core と照合し fail-fast)
//!   3. `VI_THREADS` 設定 (vi_threads > 0 のとき; sparse 系ソルバのスレッド数)
//!   4. /map 受信 (transient_local, 初回メッセージまでブロック)
//!   5. PlannerCore 構築 (地図は起動時に一度だけ取り込む静的地図前提)
//!   6. pose トピック購読 + action サーバ + value_function パブリッシャ配線
//!   7. executor.spin()
//!
//! 計画要求ごとの流れ:
//!   - 進行中の solve があれば cancel フラグでプリエンプト
//!   - start は goal.use_start ? goal.start : 自己位置トピックの最新値
//!     (rclrs に tf2 が無いため、emcl2 の `mcl_pose` / AMCL の `amcl_pose` 等の
//!     PoseWithCovarianceStamped トピックで代替する)
//!   - PlannerCore::plan (キャッシュヒット時は rollout のみ) を専用スレッドで
//!     実行し、futures oneshot で async コールバックへ返す (rclrs に tokio は
//!     無い; vi_node と同じブリッジ)
//!   - 成功時: nav_msgs/Path を Result で返し、solve 直後なら value_function
//!     (θ=0 スライス) を配信
//!
//! NOTE: rclrs API は ros2-rust/ros2_rust @ 2c6b926 (rclrs 0.7.0) — Docker
//! イメージがビルドする版 — に合わせている (vi_node と同一)。

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context as ACtx, Result};
use ndarray::Array2;

use vi_core::{ACTION_FW, ACTION_ROT, N_ACTIONS, N_THETA};
use vi_reference::bridge::{
    downsample_occupancy, occupancy_view_to_vi_grid, value_slice_to_occupancy, OccupancyGridView,
    PoseView,
};
use vi_reference::planner::PathPose;
use vi_reference::solvers::U64Solver;
use vi_reference::Action;
// nav_msgs::msg::OccupancyGrid と名前が衝突するので別名で入れる。
use vi_reference::OccupancyGrid as ViOccupancyGrid;

use vi_global_planner::core::{
    value_slice_from_vi, BuildParams, PlanConfig, PlannerCore, ValueSlice,
};

use rclrs::*;

// ──────────────────────────────────────────────────────────────────────────────
// Parameters
// ──────────────────────────────────────────────────────────────────────────────

struct Params {
    solver: String,
    theta_cell_num: i64,
    safety_radius: f64,
    safety_radius_penalty: i64,
    goal_margin_radius: f64,
    goal_margin_theta_deg: f64,
    map_wait_sec: i64,
    allow_action_mismatch: bool,
    action_list: Vec<(String, f64, f64)>,
    // vi_global_planner 固有
    pose_topic: String,
    global_frame: String,
    vi_threads: i64,
    max_solve_iter: i64,
    solve_chunk: i64,
    max_rollout_steps: i64,
    start_tolerance: f64,
    path_spacing: f64,
    goal_tolerance_xy: f64,
    goal_tolerance_deg: f64,
    cost_drawing_threshold: i64,
    publish_value_function: bool,
    /// solve 途中経過の value_function 配信間隔 [ms] (0 = 完了時のみ)。
    value_publish_interval_ms: i64,
    unknown_as_obstacle: bool,
    downsample_policy: String,
    /// プランナ内部で地図を何倍に粗くするか (1 = /map のまま)。
    map_scale: i64,
    /// compact (アウトオブコア) 経路の確定出力を置くディレクトリ ("" = RAM)。
    compact_sink_dir: String,
    /// RAM sink の上限 [MB]。compact 経路で `compact_sink_dir` 未指定かつ推定サイズが
    /// これを超えるとき、自動でディスク sink に逃がす。
    compact_ram_limit_mb: i64,
}

fn read_params(node: &Node) -> Result<Params> {
    macro_rules! p {
        ($name:literal, $ty:ty, $default:expr) => {
            node.declare_parameter::<$ty>($name)
                .default($default)
                .mandatory()
                .map_err(|e| anyhow!(concat!("declare ", $name, ": {}"), e))?
                .get()
        };
    }

    let solver = p!("solver", Arc<str>, "frontier2d_sparse".into()).to_string();
    let theta_cell_num = p!("theta_cell_num", i64, 60);
    let safety_radius = p!("safety_radius", f64, 0.2);
    let safety_radius_penalty = p!("safety_radius_penalty", i64, 30);
    let goal_margin_radius = p!("goal_margin_radius", f64, 0.3);
    let goal_margin_theta_deg = p!("goal_margin_theta", f64, 15.0);
    let map_wait_sec = p!("map_wait_sec", i64, 30);
    let allow_action_mismatch = p!("allow_action_mismatch", bool, false);

    let pose_topic = p!("pose_topic", Arc<str>, "mcl_pose".into()).to_string();
    let global_frame = p!("global_frame", Arc<str>, "map".into()).to_string();
    let vi_threads = p!("vi_threads", i64, 0);
    let max_solve_iter = p!("max_solve_iter", i64, 1_000_000);
    let solve_chunk = p!("solve_chunk", i64, 64);
    let max_rollout_steps = p!("max_rollout_steps", i64, 10_000);
    let start_tolerance = p!("start_tolerance", f64, 0.5);
    let path_spacing = p!("path_spacing", f64, 0.05);
    let goal_tolerance_xy = p!("goal_tolerance_xy", f64, 0.25);
    let goal_tolerance_deg = p!("goal_tolerance_deg", f64, 10.0);
    let cost_drawing_threshold = p!("cost_drawing_threshold", i64, 60);
    let publish_value_function = p!("publish_value_function", bool, true);
    let value_publish_interval_ms = p!("value_publish_interval_ms", i64, 500);
    let unknown_as_obstacle = p!("unknown_as_obstacle", bool, true);
    // "conservative" = 本家 downsample_occupancy (障害物優先)。"optimistic" = ブロック内に free が
    // 1 つでもあれば free。map_scale >= 4 で通路のセル幅を保つために必要 (既定は挙動不変の保守側)。
    let downsample_policy = p!("downsample_policy", Arc<str>, "conservative".into()).to_string();
    let map_scale = p!("map_scale", i64, 1);
    let compact_sink_dir = p!("compact_sink_dir", Arc<str>, "".into()).to_string();
    let compact_ram_limit_mb = p!("compact_ram_limit_mb", i64, 512);

    let names: Vec<String> = node
        .declare_parameter::<Arc<[Arc<str>]>>("action_names")
        .default_string_array(["forward", "back", "right", "rightfw", "left", "leftfw"])
        .mandatory()
        .map_err(|e| anyhow!("declare action_names: {e}"))?
        .get()
        .iter()
        .map(|s| s.to_string())
        .collect();
    let fws: Vec<f64> = node
        .declare_parameter::<Arc<[f64]>>("action_forward_m")
        .default_from_iter([0.3, -0.2, 0.0, 0.2, 0.0, 0.2])
        .mandatory()
        .map_err(|e| anyhow!("declare action_forward_m: {e}"))?
        .get()
        .to_vec();
    let rots: Vec<f64> = node
        .declare_parameter::<Arc<[f64]>>("action_rotation_deg")
        .default_from_iter([0.0, 0.0, -20.0, -20.0, 20.0, 20.0])
        .mandatory()
        .map_err(|e| anyhow!("declare action_rotation_deg: {e}"))?
        .get()
        .to_vec();

    if names.len() != fws.len() || fws.len() != rots.len() {
        return Err(anyhow!(
            "action_names/action_forward_m/action_rotation_deg length mismatch: \
             names={}, fws={}, rots={}",
            names.len(),
            fws.len(),
            rots.len()
        ));
    }
    let action_list =
        names.into_iter().zip(fws).zip(rots).map(|((n, f), r)| (n, f, r)).collect();

    Ok(Params {
        solver,
        theta_cell_num,
        safety_radius,
        safety_radius_penalty,
        goal_margin_radius,
        goal_margin_theta_deg,
        map_wait_sec,
        allow_action_mismatch,
        action_list,
        pose_topic,
        global_frame,
        vi_threads,
        max_solve_iter,
        solve_chunk,
        max_rollout_steps,
        start_tolerance,
        path_spacing,
        goal_tolerance_xy,
        goal_tolerance_deg,
        cost_drawing_threshold,
        publish_value_function,
        value_publish_interval_ms,
        unknown_as_obstacle,
        downsample_policy,
        map_scale,
        compact_sink_dir,
        compact_ram_limit_mb,
    })
}

/// vi_core のコンパイル時定数とパラメータを照合 (vi_node と同じ fail-fast)。
fn validate(p: &Params) -> Result<U64Solver> {
    if p.map_scale < 1 {
        return Err(anyhow!("map_scale must be >= 1, got {}", p.map_scale));
    }
    if p.theta_cell_num != N_THETA as i64 {
        return Err(anyhow!(
            "vi_rs is compiled with N_THETA={}, got theta_cell_num={}",
            N_THETA,
            p.theta_cell_num
        ));
    }
    if p.action_list.len() != N_ACTIONS {
        return Err(anyhow!(
            "vi_rs requires exactly {} actions, got {}",
            N_ACTIONS,
            p.action_list.len()
        ));
    }
    for (i, (_, fw, rot)) in p.action_list.iter().enumerate() {
        if (fw - ACTION_FW[i]).abs() > 1e-6 || (rot - ACTION_ROT[i]).abs() > 1e-6 {
            let msg = format!(
                "action[{i}] differs from vi_rs constants: got (fw={fw}, rot={rot}), \
                 expected (fw={}, rot={})",
                ACTION_FW[i], ACTION_ROT[i]
            );
            if p.allow_action_mismatch {
                eprintln!("WARN: {msg}");
            } else {
                return Err(anyhow!(msg));
            }
        }
    }
    U64Solver::from_name(&p.solver)
        .ok_or_else(|| anyhow!("unknown solver: {} (see U64Solver::from_name)", p.solver))
}

// ──────────────────────────────────────────────────────────────────────────────
// Geometry helpers
// ──────────────────────────────────────────────────────────────────────────────

fn yaw_from_quat(q: &geometry_msgs::msg::Quaternion) -> f64 {
    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    siny_cosp.atan2(cosy_cosp)
}

fn pose_view_from(p: &geometry_msgs::msg::Pose) -> PoseView {
    PoseView { x: p.position.x, y: p.position.y, yaw_rad: yaw_from_quat(&p.orientation) }
}

/// `PathPose` 列 → `nav_msgs::msg::Path`。
fn poses_to_path(
    poses: &[PathPose],
    frame_id: &str,
    stamp: (i32, u32),
) -> nav_msgs::msg::Path {
    let mut path = nav_msgs::msg::Path::default();
    path.header.frame_id = frame_id.into();
    path.header.stamp.sec = stamp.0;
    path.header.stamp.nanosec = stamp.1;
    path.poses = poses
        .iter()
        .map(|p| {
            let mut ps = geometry_msgs::msg::PoseStamped::default();
            ps.header.frame_id = frame_id.into();
            ps.header.stamp.sec = stamp.0;
            ps.header.stamp.nanosec = stamp.1;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.orientation.z = (p.yaw / 2.0).sin();
            ps.pose.orientation.w = (p.yaw / 2.0).cos();
            ps
        })
        .collect();
    path
}

/// solve 済み価値関数の θ=0 スライス (密/compact 共通) を OccupancyGrid に描画。
fn value_function_grid(
    slice: &ValueSlice,
    threshold_steps: u64,
    frame_id: &str,
    stamp: (i32, u32),
) -> nav_msgs::msg::OccupancyGrid {
    let (nx, ny) = (slice.width, slice.height);
    let mut arr = Array2::<u64>::zeros((ny as usize, nx as usize));
    for iy in 0..ny as usize {
        for ix in 0..nx as usize {
            arr[[iy, ix]] = slice.values[iy * nx as usize + ix];
        }
    }
    let mut msg = nav_msgs::msg::OccupancyGrid::default();
    msg.header.frame_id = frame_id.into();
    msg.header.stamp.sec = stamp.0;
    msg.header.stamp.nanosec = stamp.1;
    msg.info.resolution = slice.resolution as f32;
    msg.info.width = nx as u32;
    msg.info.height = ny as u32;
    msg.info.origin.position.x = slice.origin_x;
    msg.info.origin.position.y = slice.origin_y;
    msg.info.origin.orientation.w = 1.0;
    msg.data = value_slice_to_occupancy(&arr, threshold_steps);
    msg
}

/// compact 経路の出力先ディレクトリを決める。
///
/// - compact 以外のソルバでは常に `None` (使われない)。
/// - `compact_sink_dir` が明示されていればそれを使う。
/// - 未指定でも確定出力 (`nstates × 12 B`) が `compact_ram_limit_mb` を超えるなら
///   `/tmp/vi_global_planner_sink` に逃がす。小メモリ機で黙って GB 級の `RamSink` を
///   確保すると OOM killer に落とされるため。
fn compact_sink_dir(params: &Params, solver: U64Solver, nstates: usize) -> Option<PathBuf> {
    if !matches!(solver, U64Solver::Frontier2DSparseCompact { .. }) {
        return None;
    }
    let bytes = nstates as u64 * 12;
    if !params.compact_sink_dir.is_empty() {
        let dir = PathBuf::from(&params.compact_sink_dir);
        eprintln!(
            "vi_global_planner: compact output -> disk mmap {} ({:.2} GB)",
            dir.display(),
            bytes as f64 / 1e9
        );
        return Some(dir);
    }
    let limit = params.compact_ram_limit_mb.max(0) as u64 * 1024 * 1024;
    if bytes > limit {
        let dir = PathBuf::from("/tmp/vi_global_planner_sink");
        eprintln!(
            "WARN: compact output would need {:.2} GB of RAM (> compact_ram_limit_mb={}); \
             spilling to disk mmap {}",
            bytes as f64 / 1e9,
            params.compact_ram_limit_mb,
            dir.display()
        );
        return Some(dir);
    }
    eprintln!("vi_global_planner: compact output -> RAM ({:.2} GB)", bytes as f64 / 1e9);
    None
}

/// `downsample_occupancy` の楽観版。ブロック内に 1 つでも free があれば出力セルを free にする。
///
/// 本家の `downsample_occupancy` は障害物優先 (ブロック内に非 free が 1 つでもあれば占有) なので、
/// 通路が片側最大 `(scale-1)·resolution` 細る。map_tsudanuma は unknown が 68% あり、`map_scale >= 4`
/// では free **面積**は数 % しか減らないのに**通路のセル幅**が落ちる。VI の遷移はサブセル
/// サンプリングによる約 2 セル幅の分布なので、散り先に 1 つでも未到達セルがあると期待値が
/// MAX_COST 側に張り付き、波がゴール近傍で止まる (実測: scale 4 で到達列 1、`--unknown free` に
/// すると完全に伝播)。楽観側にすると通路のセル幅が保たれ、scale 5 まで解けるようになる。
///
/// 安全余裕を地図に焼き込んではいけない点に注意。VI の `safety_radius` は硬い壁ではなく
/// 秒/セルのソフトなペナルティで、これを膨張として焼き込むと scale 3 でも波が死ぬ (実測)。
/// ここでは通路を開けるだけにして、余裕は `safety_radius` / `safety_radius_penalty` に任せる。
fn downsample_occupancy_optimistic(grid: &ViOccupancyGrid, scale: i32) -> ViOccupancyGrid {
    if scale <= 1 {
        return grid.clone();
    }
    let (w, h, s) = (grid.width as usize, grid.height as usize, scale as usize);
    let (ow, oh) = (w.div_ceil(s), h.div_ceil(s));
    let mut data = vec![100i8; ow * oh];
    for oy in 0..oh {
        for ox in 0..ow {
            let mut free = false;
            'blk: for dy in 0..s {
                let iy = oy * s + dy;
                if iy >= h {
                    break;
                }
                for dx in 0..s {
                    let ix = ox * s + dx;
                    if ix >= w {
                        break;
                    }
                    if grid.data[iy * w + ix] == 0 {
                        free = true;
                        break 'blk;
                    }
                }
            }
            data[oy * ow + ox] = if free { 0 } else { 100 };
        }
    }
    ViOccupancyGrid {
        width: ow as i32,
        height: oh as i32,
        resolution: grid.resolution * scale as f64,
        origin_x: grid.origin_x,
        origin_y: grid.origin_y,
        origin_quat: grid.origin_quat.clone(),
        data,
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// main
// ──────────────────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    // 1. ROS context + executor + node.
    let context = Context::default_from_env().context("rclrs context init")?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("vi_global_planner").context("create vi_global_planner")?;

    // 2. Parameters + validation.
    let params = read_params(&node).context("reading parameters")?;
    let solver = validate(&params).context("validating parameters")?;

    // 3. sparse 系ソルバのスレッド数 (n_threads() が VI_THREADS を読む)。
    if params.vi_threads > 0 {
        std::env::set_var("VI_THREADS", params.vi_threads.to_string());
    }

    // 4. Wait for /map.
    let map_msg = wait_for_map(&node, &mut executor, params.map_wait_sec)
        .context("waiting for /map")?;
    eprintln!(
        "vi_global_planner: got map {}x{} @{}m",
        map_msg.info.width, map_msg.info.height, map_msg.info.resolution
    );

    // 5. PlannerCore.
    let grid_view = OccupancyGridView {
        width: map_msg.info.width,
        height: map_msg.info.height,
        resolution: map_msg.info.resolution as f64,
        origin_x: map_msg.info.origin.position.x,
        origin_y: map_msg.info.origin.position.y,
        data: &map_msg.data[..],
    };
    // プランナ内部の作業解像度。VI の状態数は nx·ny·nθ なので、広域地図 (津田沼 5888x4000
    // @0.05m = 14 億状態) は map_scale で粗くしないと解けない。map_server / costmap /
    // 自己位置は元解像度のまま (ここで粗くするのはプランナの内部表現だけ)。
    let binary_grid = occupancy_view_to_vi_grid(&grid_view, params.unknown_as_obstacle);
    let vi_grid = match params.downsample_policy.as_str() {
        "optimistic" => downsample_occupancy_optimistic(&binary_grid, params.map_scale as i32),
        "conservative" => downsample_occupancy(&binary_grid, params.map_scale as i32),
        other => {
            return Err(anyhow!(
                "downsample_policy must be \"conservative\" or \"optimistic\", got {other:?}"
            ))
        }
    };
    drop(binary_grid);
    let nstates =
        vi_grid.width as usize * vi_grid.height as usize * params.theta_cell_num as usize;
    eprintln!(
        "vi_global_planner: planner grid {}x{} @{:.3}m (map_scale={}, downsample={}) x{} theta \
         = {} states",
        vi_grid.width, vi_grid.height, vi_grid.resolution, params.map_scale,
        params.downsample_policy, params.theta_cell_num, nstates,
    );
    // start 近傍スナップの半径はプランナ側の解像度で数える。
    let start_tolerance_cells =
        (params.start_tolerance / vi_grid.resolution).ceil() as i32;

    // compact 経路の出力先。確定出力は nstates × 12 B。未指定でも RAM 上限を超えるなら
    // ディスクへ逃がす (Pi4 で 2GB の RamSink を黙って確保すると OOM kill される)。
    let sink_dir = compact_sink_dir(&params, solver, nstates);

    let build = BuildParams {
        grid: vi_grid,
        actions: params
            .action_list
            .iter()
            .enumerate()
            .map(|(i, (name, fw, rot))| Action::new(name, *fw, *rot, i as i32))
            .collect(),
        theta_cell_num: params.theta_cell_num as i32,
        safety_radius: params.safety_radius,
        safety_radius_penalty: params.safety_radius_penalty as f64,
        goal_margin_radius: params.goal_margin_radius,
        goal_margin_theta: params.goal_margin_theta_deg as i32,
    };
    let cfg = PlanConfig {
        solver,
        max_solve_iter: params.max_solve_iter.max(1) as u32,
        solve_chunk: params.solve_chunk.max(1) as u32,
        max_rollout_steps: params.max_rollout_steps.max(1) as usize,
        start_tolerance_cells,
        path_spacing: params.path_spacing,
        goal_tolerance_xy: params.goal_tolerance_xy,
        goal_tolerance_deg: params.goal_tolerance_deg,
        compact_sink_dir: sink_dir,
        vi_threads: params.vi_threads.max(0) as usize,
    };
    let core = Arc::new(Mutex::new(PlannerCore::new(build, cfg)));

    // 6a. 自己位置トピック購読 (tf2 代替)。
    let latest_pose: Arc<Mutex<Option<PoseView>>> = Arc::new(Mutex::new(None));
    let latest_pose_w = Arc::clone(&latest_pose);
    let _pose_sub = node.create_subscription::<geometry_msgs::msg::PoseWithCovarianceStamped, _>(
        params.pose_topic.as_str().keep_last(1),
        move |msg: geometry_msgs::msg::PoseWithCovarianceStamped| {
            *latest_pose_w.lock().unwrap() = Some(pose_view_from(&msg.pose.pose));
        },
    )?;

    // 6b. value_function デバッグ配信 (solve 中は value_publish_interval_ms
    //     ごとの途中経過 + 完了時に 1 回)。
    let vf_pub = if params.publish_value_function {
        Some(Arc::new(node.create_publisher::<nav_msgs::msg::OccupancyGrid>(
            "value_function".reliable().transient_local().keep_last(1),
        )?))
    } else {
        None
    };

    // 6c. compute_path_to_pose action サーバ。
    // 進行中 solve のプリエンプト用 cancel フラグ置き場。
    let active_cancel: Arc<Mutex<Option<Arc<AtomicBool>>>> = Arc::new(Mutex::new(None));
    let node_clock = node.get_clock();
    let frame_id = params.global_frame.clone();
    let threshold_steps = params.cost_drawing_threshold.max(0) as u64;
    // ロールアウト固着時のヒント表示用 (safety_radius_penalty [秒/セル], safety_radius [m])。
    let params_hint = (params.safety_radius_penalty, params.safety_radius);
    let publish_interval_ms = params.value_publish_interval_ms.max(0) as u64;

    let _server = node.create_action_server::<nav2_msgs::action::ComputePathToPose, _>(
        "compute_path_to_pose",
        move |requested_goal: RequestedGoal<nav2_msgs::action::ComputePathToPose>| {
            let core = Arc::clone(&core);
            let latest_pose = Arc::clone(&latest_pose);
            let active_cancel = Arc::clone(&active_cancel);
            let vf_pub = vf_pub.clone();
            let frame_id = frame_id.clone();
            let node_clock = node_clock.clone();

            async move {
                // ── プリエンプト: 前の solve を止め、自分の cancel を登録 ──
                let my_cancel = Arc::new(AtomicBool::new(false));
                {
                    let mut slot = active_cancel.lock().unwrap();
                    if let Some(prev) = slot.take() {
                        prev.store(true, Ordering::SeqCst);
                    }
                    *slot = Some(Arc::clone(&my_cancel));
                }

                let accepted = requested_goal.accept();
                let goal_msg = accepted.goal();
                let goal = pose_view_from(&goal_msg.goal.pose);
                let start = if goal_msg.use_start {
                    Some(pose_view_from(&goal_msg.start.pose))
                } else {
                    *latest_pose.lock().unwrap()
                };
                let executing = accepted.execute();

                let Some(start) = start else {
                    eprintln!(
                        "ERROR: no robot pose available (use_start=false and nothing \
                         received on the pose topic yet)"
                    );
                    return executing
                        .aborted_with(nav2_msgs::action::ComputePathToPose_Result::default());
                };

                eprintln!(
                    "vi_global_planner: plan ({:.2}, {:.2}) -> ({:.2}, {:.2})",
                    start.x, start.y, goal.x, goal.y
                );

                // ── 計画本体は専用スレッドで実行 (solve は数秒〜数十秒ブロック) ──
                let t0 = Instant::now();
                type PlanOutcome = std::result::Result<
                    (Vec<PathPose>, vi_global_planner::core::PlanStats),
                    vi_global_planner::core::PlanError,
                >;
                let (done_tx, done_rx) = futures::channel::oneshot::channel::<PlanOutcome>();
                let core_thread = Arc::clone(&core);
                let vf_pub_thread = vf_pub.clone();
                let frame_thread = frame_id.clone();
                let clock_thread = node_clock.clone();
                std::thread::spawn(move || {
                    let mut core = core_thread.lock().unwrap();
                    // solve 中の途中経過配信 (チャンク write_back 後の値なので
                    // ゴールから波面が広がる様子が RViz で見える)。
                    let mut last_pub: Option<Instant> = None;
                    let result = core.plan_with_progress(start, goal, &my_cancel, &mut |vi| {
                        let Some(pub_) = &vf_pub_thread else { return };
                        if publish_interval_ms == 0 {
                            return;
                        }
                        let due = last_pub.map_or(true, |t| {
                            t.elapsed() >= Duration::from_millis(publish_interval_ms)
                        });
                        if !due {
                            return;
                        }
                        let stamp = clock_thread.now().to_sec_nanosec().unwrap_or((0, 0));
                        let _ = pub_.publish(value_function_grid(
                            &value_slice_from_vi(vi),
                            threshold_steps,
                            &frame_thread,
                            stamp,
                        ));
                        last_pub = Some(Instant::now());
                    });
                    // solve が走った場合のみ value_function を配信し直す
                    // (compact 経路は途中経過が無いので、この 1 回だけが配信される)。
                    if let (Ok((_, stats)), Some(pub_)) = (&result, &vf_pub_thread) {
                        if stats.solved_now {
                            if let Some(slice) = core.cached_value_slice() {
                                let stamp =
                                    clock_thread.now().to_sec_nanosec().unwrap_or((0, 0));
                                let _ = pub_.publish(value_function_grid(
                                    &slice,
                                    threshold_steps,
                                    &frame_thread,
                                    stamp,
                                ));
                            }
                        }
                    }
                    let _ = done_tx.send(result);
                });

                match done_rx.await {
                    Ok(Ok((poses, stats))) => {
                        let dt = t0.elapsed();
                        eprintln!(
                            "vi_global_planner: path with {} poses in {:.2}s (solved_now={}, iters={})",
                            stats.poses,
                            dt.as_secs_f64(),
                            stats.solved_now,
                            stats.iters
                        );
                        let stamp = node_clock.now().to_sec_nanosec().unwrap_or((0, 0));
                        let mut result = nav2_msgs::action::ComputePathToPose_Result::default();
                        result.path = poses_to_path(&poses, &frame_id, stamp);
                        result.planning_time.sec = dt.as_secs() as i32;
                        result.planning_time.nanosec = dt.subsec_nanos();
                        executing.succeeded_with(result)
                    }
                    Ok(Err(e)) => {
                        eprintln!("ERROR: vi_global_planner: {e}");
                        // ロールアウトの固着は「価値関数の局所的なゆらぎ > 1 手の
                        // 進捗」で起きる。safety_radius_penalty (秒/セル) が 1 手の
                        // コスト 1 秒に対して大きすぎ、かつ経路の大半がペナルティ域に
                        // 入る地図 (細い通路 / 粗いセル) で顕在化する。
                        if matches!(
                            e,
                            vi_global_planner::core::PlanError::Rollout(
                                vi_reference::planner::RolloutStatus::LoopDetected
                            )
                        ) {
                            eprintln!(
                                "HINT: value function converged but the greedy rollout \
                                 oscillated. Lower safety_radius_penalty (currently {}) or \
                                 safety_radius (currently {}) — a penalty much larger than \
                                 the 1s per-step cost makes the value landscape jitter more \
                                 than one step of progress.",
                                params_hint.0, params_hint.1
                            );
                        }
                        executing.aborted_with(
                            nav2_msgs::action::ComputePathToPose_Result::default(),
                        )
                    }
                    Err(_) => executing.aborted_with(
                        nav2_msgs::action::ComputePathToPose_Result::default(),
                    ),
                }
            }
        },
    )?;

    eprintln!("vi_global_planner: ready (solver={}, action=compute_path_to_pose)", params.solver);

    // 7. Spin.
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

// ──────────────────────────────────────────────────────────────────────────────
// wait_for_map — transient_local subscriber, blocks until first message
// ──────────────────────────────────────────────────────────────────────────────

fn wait_for_map(
    node: &Node,
    executor: &mut Executor,
    wait_sec: i64,
) -> Result<nav_msgs::msg::OccupancyGrid> {
    use std::sync::mpsc::sync_channel;

    let (tx, rx) = sync_channel::<nav_msgs::msg::OccupancyGrid>(1);
    let tx_c = tx.clone();

    let _sub = node.create_subscription::<nav_msgs::msg::OccupancyGrid, _>(
        "map".transient_local().reliable().keep_last(1),
        move |msg: nav_msgs::msg::OccupancyGrid| {
            let _ = tx_c.try_send(msg);
        },
    )?;

    let deadline = Instant::now() + Duration::from_secs(wait_sec as u64);
    loop {
        if let Ok(msg) = rx.try_recv() {
            return Ok(msg);
        }
        if Instant::now() > deadline {
            return Err(anyhow!("map not received within {} seconds", wait_sec));
        }
        executor.spin(SpinOptions::default().timeout(Duration::from_millis(100)));
    }
}
