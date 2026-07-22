//! vi_planner entry point — Nav2 の planner_server を置き換える
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

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context as ACtx, Result};
use ndarray::Array2;

use vi_core::{ACTION_FW, ACTION_ROT, N_ACTIONS, N_THETA};
use vi_reference::bridge::{
    occupancy_view_to_vi_grid, value_slice_to_occupancy, OccupancyGridView, PoseView,
};
use vi_reference::planner::PathPose;
use vi_reference::solvers::U64Solver;
use vi_reference::{Action, ValueIterator};

use vi_planner::core::{BuildParams, PlanConfig, PlannerCore};

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
    // vi_planner 固有
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
    })
}

/// vi_core のコンパイル時定数とパラメータを照合 (vi_node と同じ fail-fast)。
fn validate(p: &Params) -> Result<U64Solver> {
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

/// solve 済み ValueIterator の θ=0 スライスを OccupancyGrid data に描画。
fn value_function_grid(
    vi: &ValueIterator,
    threshold_steps: u64,
    frame_id: &str,
    stamp: (i32, u32),
) -> nav_msgs::msg::OccupancyGrid {
    let (nx, ny) = (vi.cell_num_x, vi.cell_num_y);
    let mut slice = Array2::<u64>::zeros((ny as usize, nx as usize));
    for iy in 0..ny {
        for ix in 0..nx {
            slice[[iy as usize, ix as usize]] =
                vi.states[vi.to_index(ix, iy, 0) as usize].total_cost;
        }
    }
    let mut msg = nav_msgs::msg::OccupancyGrid::default();
    msg.header.frame_id = frame_id.into();
    msg.header.stamp.sec = stamp.0;
    msg.header.stamp.nanosec = stamp.1;
    msg.info.resolution = vi.xy_resolution as f32;
    msg.info.width = nx as u32;
    msg.info.height = ny as u32;
    msg.info.origin.position.x = vi.map_origin_x;
    msg.info.origin.position.y = vi.map_origin_y;
    msg.info.origin.orientation.w = 1.0;
    msg.data = value_slice_to_occupancy(&slice, threshold_steps);
    msg
}

// ──────────────────────────────────────────────────────────────────────────────
// main
// ──────────────────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    // 1. ROS context + executor + node.
    let context = Context::default_from_env().context("rclrs context init")?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("vi_planner").context("create vi_planner")?;

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
        "vi_planner: got map {}x{} @{}m",
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
    let vi_grid = occupancy_view_to_vi_grid(&grid_view, params.unknown_as_obstacle);
    let start_tolerance_cells =
        (params.start_tolerance / grid_view.resolution).ceil() as i32;

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
                    "vi_planner: plan ({:.2}, {:.2}) -> ({:.2}, {:.2})",
                    start.x, start.y, goal.x, goal.y
                );

                // ── 計画本体は専用スレッドで実行 (solve は数秒〜数十秒ブロック) ──
                let t0 = Instant::now();
                type PlanOutcome = std::result::Result<
                    (Vec<PathPose>, vi_planner::core::PlanStats),
                    vi_planner::core::PlanError,
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
                            vi,
                            threshold_steps,
                            &frame_thread,
                            stamp,
                        ));
                        last_pub = Some(Instant::now());
                    });
                    // solve が走った場合のみ value_function を配信し直す。
                    if let (Ok((_, stats)), Some(pub_)) = (&result, &vf_pub_thread) {
                        if stats.solved_now {
                            if let Some(vi) = core.cached_vi() {
                                let stamp =
                                    clock_thread.now().to_sec_nanosec().unwrap_or((0, 0));
                                let _ = pub_.publish(value_function_grid(
                                    vi,
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
                            "vi_planner: path with {} poses in {:.2}s (solved_now={}, iters={})",
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
                        eprintln!("ERROR: vi_planner: {e}");
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

    eprintln!("vi_planner: ready (solver={}, action=compute_path_to_pose)", params.solver);

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
