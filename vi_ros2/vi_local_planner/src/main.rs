//! vi_local_planner entry point — Nav2 の controller_server を置き換える
//! `follow_path` action サーバ (全 Rust)。
//!
//! 本家 value_iteration2 の狭域側 (`ValueIteratorLocal` + `ViNode::decision`
//! 100ms タイマ + `/scan` 由来の local_penalty) を Nav2 の作法に載せ替える:
//! bt_navigator は経路追従に `follow_path` (nav2_msgs/action/FollowPath) を
//! 使うので、controller_server を起動せずこのノードを立てればスタックの残りは
//! 無変更で済む (vi_planner が planner_server を置き換えるのと同型)。
//!
//! Boot order (vi_planner と同型):
//!   1. `Context::default_from_env` + basic executor + node 作成
//!   2. パラメータ宣言・検証 (アクション定数は vi_core と照合し fail-fast)
//!   3. `VI_THREADS` 設定 (vi_threads > 0 のとき)
//!   4. /map 受信 (transient_local, 初回メッセージまでブロック)
//!   5. LocalPlannerCore 構築 (静的地図前提)
//!   6. pose / scan 購読 + cmd_vel パブリッシャ + action サーバ配線
//!   7. executor.spin()
//!
//! follow_path ゴールごとの流れ:
//!   - 進行中の追従があれば cancel フラグでプリエンプト
//!   - ゴール姿勢は Goal の path 終端 (controller_id / goal_checker_id は無視)
//!   - 専用スレッドで: 価値関数を prepare (同一ゴールはキャッシュ) → 制御
//!     ループ (control_frequency Hz): ウィンドウ移動 → スキャン penalty 注入 →
//!     ウィンドウ内価値反復 (refine_budget_ms) → 貪欲行動を cmd_vel へ。
//!     本家 `ViNode::decision` と同じく行動の (delta_fw [m], delta_rot [deg])
//!     をそのまま (linear.x [m/s], angular.z [rad/s]) として配信する。
//!   - final_state 到達で succeeded / 方策なし・pose 欠落が続けば aborted /
//!     クライアント cancel は `until_cancel_requested` で観測し cancelled
//!   - 自己位置は tf2 の代わりに pose トピック (emcl2: mcl_pose / AMCL:
//!     amcl_pose) — vi_planner と同じ制約 (rclrs に tf2 が無い)
//!
//! NOTE: rclrs API は ros2-rust/ros2_rust @ 2c6b926 (rclrs 0.7.0) — Docker
//! イメージがビルドする版 — に合わせている (vi_node / vi_planner と同一)。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context as ACtx, Result};

use vi_core::{ACTION_FW, ACTION_ROT, N_ACTIONS, N_THETA};
use vi_reference::bridge::{occupancy_view_to_vi_grid, OccupancyGridView, PoseView};
use vi_reference::msg::LaserScan as ViLaserScan;
use vi_reference::solvers::U64Solver;
use vi_reference::Action;

use vi_local_planner::core::{
    value_grid_of, BuildParams, Decision, FollowConfig, LocalPlannerCore, SolveError,
};

use rclrs::*;

/// 無効レンジ (inf / NaN / 非正) の差し替え値 [m]。ローカルウィンドウ (±1m)
/// から十分遠く、セル座標化しても i32 に収まる。本家 C++ は float→int の
/// 未定義動作に頼っていたが、Rust では添字とビーム角の対応を保ったまま
/// ウィンドウ外へ飛ばして無害化する。
const INVALID_RANGE_M: f64 = 1.0e6;

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
    // vi_local_planner 固有
    pose_topic: String,
    scan_topic: String,
    vi_threads: i64,
    max_solve_iter: i64,
    solve_chunk: i64,
    goal_tolerance_xy: f64,
    goal_tolerance_deg: f64,
    control_frequency: f64,
    refine_budget_ms: i64,
    action_tolerance: f64,
    no_action_timeout_sec: f64,
    unknown_as_obstacle: bool,
    // 可視化 (RViz 向け value function 配信)
    global_frame: String,
    cost_drawing_threshold: i64,
    publish_value_function: bool,
    /// 可視化配信の間隔 [ms] (0 = solve 完了時のみ)。
    value_publish_interval_ms: i64,
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
    let scan_topic = p!("scan_topic", Arc<str>, "scan".into()).to_string();
    let vi_threads = p!("vi_threads", i64, 0);
    let max_solve_iter = p!("max_solve_iter", i64, 1_000_000);
    let solve_chunk = p!("solve_chunk", i64, 64);
    let goal_tolerance_xy = p!("goal_tolerance_xy", f64, 0.25);
    let goal_tolerance_deg = p!("goal_tolerance_deg", f64, 10.0);
    let control_frequency = p!("control_frequency", f64, 10.0);
    let refine_budget_ms = p!("refine_budget_ms", i64, 40);
    let action_tolerance = p!("action_tolerance", f64, 0.2);
    let no_action_timeout_sec = p!("no_action_timeout_sec", f64, 3.0);
    let unknown_as_obstacle = p!("unknown_as_obstacle", bool, true);
    let global_frame = p!("global_frame", Arc<str>, "map".into()).to_string();
    let cost_drawing_threshold = p!("cost_drawing_threshold", i64, 60);
    let publish_value_function = p!("publish_value_function", bool, true);
    let value_publish_interval_ms = p!("value_publish_interval_ms", i64, 500);

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
        scan_topic,
        vi_threads,
        max_solve_iter,
        solve_chunk,
        goal_tolerance_xy,
        goal_tolerance_deg,
        control_frequency,
        refine_budget_ms,
        action_tolerance,
        no_action_timeout_sec,
        unknown_as_obstacle,
        global_frame,
        cost_drawing_threshold,
        publish_value_function,
        value_publish_interval_ms,
    })
}

/// vi_core のコンパイル時定数とパラメータを照合 (vi_planner と同じ fail-fast)。
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
    if p.control_frequency <= 0.0 {
        return Err(anyhow!("control_frequency must be > 0, got {}", p.control_frequency));
    }
    U64Solver::from_name(&p.solver)
        .ok_or_else(|| anyhow!("unknown solver: {} (see U64Solver::from_name)", p.solver))
}

// ──────────────────────────────────────────────────────────────────────────────
// Geometry / message helpers
// ──────────────────────────────────────────────────────────────────────────────

fn yaw_from_quat(q: &geometry_msgs::msg::Quaternion) -> f64 {
    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    siny_cosp.atan2(cosy_cosp)
}

fn pose_view_from(p: &geometry_msgs::msg::Pose) -> PoseView {
    PoseView { x: p.position.x, y: p.position.y, yaw_rad: yaw_from_quat(&p.orientation) }
}

/// sensor_msgs/LaserScan → vi_reference::LaserScan。ビーム角と添字の対応を
/// 保つため無効レンジは取り除かず `INVALID_RANGE_M` に差し替える
/// (`set_local_cost` がウィンドウ外として自然に無視する)。
fn vi_scan_from(msg: &sensor_msgs::msg::LaserScan) -> ViLaserScan {
    ViLaserScan {
        angle_min: msg.angle_min as f64,
        angle_increment: msg.angle_increment as f64,
        ranges: msg
            .ranges
            .iter()
            .map(|&r| {
                let r = r as f64;
                if r.is_finite() && r > 0.0 {
                    r
                } else {
                    INVALID_RANGE_M
                }
            })
            .collect(),
    }
}

fn stop_cmd(pub_cmd: &Publisher<geometry_msgs::msg::Twist>) {
    let _ = pub_cmd.publish(geometry_msgs::msg::Twist::default());
}

// ──────────────────────────────────────────────────────────────────────────────
// Value function visualization (publish_value_function=true)
// ──────────────────────────────────────────────────────────────────────────────

/// value function 可視化の配信一式。
struct Viz {
    /// θ=0 全域スライス (solve の途中経過 + 完了時)。
    vf_pub: Publisher<nav_msgs::msg::OccupancyGrid>,
    /// ローカルウィンドウの現在方位スライス (追従中、スキャン penalty 込み)。
    win_pub: Publisher<nav_msgs::msg::OccupancyGrid>,
    clock: Clock,
    frame_id: String,
    threshold_steps: u64,
    /// 配信間隔。0 で solve 完了時のみ。
    interval: Duration,
}

impl Viz {
    fn stamp(&self) -> (i32, u32) {
        self.clock.now().to_sec_nanosec().unwrap_or((0, 0))
    }
}

/// vi_reference の可視化描画済み OccupancyGrid → ROS メッセージ。
fn ros_grid_from(
    g: &vi_reference::msg::OccupancyGrid,
    frame_id: &str,
    stamp: (i32, u32),
) -> nav_msgs::msg::OccupancyGrid {
    let mut msg = nav_msgs::msg::OccupancyGrid::default();
    msg.header.frame_id = frame_id.into();
    msg.header.stamp.sec = stamp.0;
    msg.header.stamp.nanosec = stamp.1;
    msg.info.resolution = g.resolution as f32;
    msg.info.width = g.width as u32;
    msg.info.height = g.height as u32;
    msg.info.origin.position.x = g.origin_x;
    msg.info.origin.position.y = g.origin_y;
    msg.info.origin.orientation.x = g.origin_quat.x;
    msg.info.origin.orientation.y = g.origin_quat.y;
    msg.info.origin.orientation.z = g.origin_quat.z;
    msg.info.origin.orientation.w = g.origin_quat.w;
    msg.data = g.data.clone();
    msg
}

// ──────────────────────────────────────────────────────────────────────────────
// Follow loop (dedicated thread per goal)
// ──────────────────────────────────────────────────────────────────────────────

/// 制御ループのチューニング (Params から導出)。
#[derive(Clone, Copy)]
struct FollowTuning {
    period: Duration,
    refine_budget: Duration,
    /// pose 欠落 / 方策なしをこの連続 tick 数で追従失敗とみなす。
    failure_ticks_limit: u32,
}

enum Outcome {
    Reached,
    Preempted,
    Failed(String),
}

/// 1 ゴールぶんの追従ループ。`core` のロックを保持し続けるが、cancel は毎 tick
/// (solve 中は solve_chunk ごと) に観測するので、次のゴールのスレッドは
/// 100ms 程度でロックを獲得できる。
#[allow(clippy::too_many_arguments)]
fn run_follow(
    core: &Mutex<LocalPlannerCore>,
    goal: PoseView,
    cancel: &AtomicBool,
    latest_pose: &Mutex<Option<PoseView>>,
    scan_queue: &Mutex<Vec<ViLaserScan>>,
    cmd_pub: &Publisher<geometry_msgs::msg::Twist>,
    feedback: &FeedbackPublisher<nav2_msgs::action::FollowPath>,
    tuning: FollowTuning,
    viz: Option<&Viz>,
) -> Outcome {
    let mut core = core.lock().unwrap();

    // solve 中は静止 (velocity_smoother のタイムアウト任せにしない)。
    stop_cmd(cmd_pub);
    let t0 = Instant::now();
    // solve 途中経過 + 追従中ウィンドウの可視化を共通の間隔で間引く。
    let mut last_viz: Option<Instant> = None;
    let prepared = core.prepare_goal_with_progress(goal, cancel, &mut |vi| {
        let Some(v) = viz else { return };
        if v.interval.is_zero() {
            return;
        }
        if last_viz.map_or(false, |t| t.elapsed() < v.interval) {
            return;
        }
        let g = value_grid_of(vi, v.threshold_steps);
        let _ = v.vf_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
        last_viz = Some(Instant::now());
    });
    match prepared {
        Ok(stats) => {
            if stats.solved_now {
                eprintln!(
                    "vi_local_planner: value function solved in {:.2}s (iters={})",
                    t0.elapsed().as_secs_f64(),
                    stats.iters
                );
                // 収束後の最終状態を必ず 1 回配信する (間引きで最後のチャンク
                // が落ちても RViz は完成形を得る)。
                if let Some(v) = viz {
                    if let Some(g) = core.value_grid(v.threshold_steps) {
                        let _ = v.vf_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
                    }
                }
            }
        }
        Err(SolveError::Cancelled) => return Outcome::Preempted,
        Err(e) => return Outcome::Failed(e.to_string()),
    }

    let mut failure_ticks = 0u32;
    loop {
        let tick_start = Instant::now();
        if cancel.load(Ordering::Relaxed) {
            stop_cmd(cmd_pub);
            return Outcome::Preempted;
        }

        let pose = *latest_pose.lock().unwrap();
        match pose {
            None => {
                stop_cmd(cmd_pub);
                failure_ticks += 1;
            }
            Some(pose) => {
                core.set_window(pose);
                let scans = std::mem::take(&mut *scan_queue.lock().unwrap());
                for scan in &scans {
                    core.observe_scan(scan, pose);
                }
                core.refine_for(tuning.refine_budget);

                // ローカルウィンドウの可視化 (現在方位の θ スライス。スキャン
                // 由来の local_penalty と局所反復の結果が見える)。
                if let Some(v) = viz {
                    if !v.interval.is_zero()
                        && last_viz.map_or(true, |t| t.elapsed() >= v.interval)
                    {
                        if let Some(g) = core.window_value_grid(pose, v.threshold_steps) {
                            let _ =
                                v.win_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
                        }
                        last_viz = Some(Instant::now());
                    }
                }

                let mut speed = 0.0f32;
                match core.decide(pose) {
                    Decision::Goal => {
                        stop_cmd(cmd_pub);
                        return Outcome::Reached;
                    }
                    Decision::Action { fw, rot_deg, .. } => {
                        // 本家 ViNode::decision: delta をそのまま速度指令に。
                        let mut tw = geometry_msgs::msg::Twist::default();
                        tw.linear.x = fw;
                        tw.angular.z = rot_deg.to_radians();
                        let _ = cmd_pub.publish(tw);
                        speed = fw as f32;
                        failure_ticks = 0;
                    }
                    Decision::NoAction => {
                        stop_cmd(cmd_pub);
                        failure_ticks += 1;
                    }
                }

                let dist = core.goal_distance(pose.x, pose.y).unwrap_or(f64::NAN);
                let _ = feedback.publish(nav2_msgs::action::FollowPath_Feedback {
                    distance_to_goal: dist as f32,
                    speed,
                });
            }
        }

        if failure_ticks >= tuning.failure_ticks_limit {
            stop_cmd(cmd_pub);
            return Outcome::Failed(
                "no robot pose / no applicable action for too long".into(),
            );
        }
        if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
            std::thread::sleep(rest);
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// main
// ──────────────────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    // 1. ROS context + executor + node.
    let context = Context::default_from_env().context("rclrs context init")?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("vi_local_planner").context("create vi_local_planner")?;

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
        "vi_local_planner: got map {}x{} @{}m",
        map_msg.info.width, map_msg.info.height, map_msg.info.resolution
    );

    // 5. LocalPlannerCore.
    let grid_view = OccupancyGridView {
        width: map_msg.info.width,
        height: map_msg.info.height,
        resolution: map_msg.info.resolution as f64,
        origin_x: map_msg.info.origin.position.x,
        origin_y: map_msg.info.origin.position.y,
        data: &map_msg.data[..],
    };
    let vi_grid = occupancy_view_to_vi_grid(&grid_view, params.unknown_as_obstacle);
    let action_tolerance_cells =
        (params.action_tolerance / grid_view.resolution).ceil() as i32;

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
    let cfg = FollowConfig {
        solver,
        max_solve_iter: params.max_solve_iter.max(1) as u32,
        solve_chunk: params.solve_chunk.max(1) as u32,
        goal_tolerance_xy: params.goal_tolerance_xy,
        goal_tolerance_deg: params.goal_tolerance_deg,
        action_tolerance_cells,
    };
    let core = Arc::new(Mutex::new(LocalPlannerCore::new(build, cfg)));

    let period = Duration::from_secs_f64(1.0 / params.control_frequency);
    let tuning = FollowTuning {
        period,
        refine_budget: Duration::from_millis(params.refine_budget_ms.max(0) as u64),
        failure_ticks_limit: (params.no_action_timeout_sec.max(0.0)
            * params.control_frequency)
            .ceil()
            .max(1.0) as u32,
    };

    // 6a. 自己位置トピック購読 (tf2 代替)。
    let latest_pose: Arc<Mutex<Option<PoseView>>> = Arc::new(Mutex::new(None));
    let latest_pose_w = Arc::clone(&latest_pose);
    let _pose_sub = node.create_subscription::<geometry_msgs::msg::PoseWithCovarianceStamped, _>(
        params.pose_topic.as_str().keep_last(1),
        move |msg: geometry_msgs::msg::PoseWithCovarianceStamped| {
            *latest_pose_w.lock().unwrap() = Some(pose_view_from(&msg.pose.pose));
        },
    )?;

    // 6b. スキャン購読 (sensor QoS = best effort)。tick 間に届いた分を貯めて
    //     制御ループが順に消化する (本家は scan コールバックで即時注入するが、
    //     pose との対応を tick 時点で取るためキュー方式にする)。
    let scan_queue: Arc<Mutex<Vec<ViLaserScan>>> = Arc::new(Mutex::new(Vec::new()));
    let scan_queue_w = Arc::clone(&scan_queue);
    let _scan_sub = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        params.scan_topic.as_str().best_effort().keep_last(5),
        move |msg: sensor_msgs::msg::LaserScan| {
            let mut q = scan_queue_w.lock().unwrap();
            // 制御ループが止まっていても際限なく溜めない (最新を優先)。
            if q.len() >= 10 {
                q.remove(0);
            }
            q.push(vi_scan_from(&msg));
        },
    )?;

    // 6c. cmd_vel パブリッシャ (Nav2 構成では launch 側で cmd_vel_nav に
    //     リマップし velocity_smoother を経由させる)。Publisher<T> は
    //     Arc エイリアスなのでそのまま clone してスレッドへ渡せる。
    let cmd_pub = node.create_publisher::<geometry_msgs::msg::Twist>("cmd_vel".keep_last(1))?;

    // 6d. value function 可視化 (RViz 向け)。vi_planner の value_function と
    //     衝突しないよう local_ プレフィクスのトピック名にする。
    let viz: Option<Arc<Viz>> = if params.publish_value_function {
        Some(Arc::new(Viz {
            vf_pub: node.create_publisher::<nav_msgs::msg::OccupancyGrid>(
                "local_value_function".reliable().transient_local().keep_last(1),
            )?,
            win_pub: node.create_publisher::<nav_msgs::msg::OccupancyGrid>(
                "local_window_value".reliable().transient_local().keep_last(1),
            )?,
            clock: node.get_clock(),
            frame_id: params.global_frame.clone(),
            threshold_steps: params.cost_drawing_threshold.max(0) as u64,
            interval: Duration::from_millis(params.value_publish_interval_ms.max(0) as u64),
        }))
    } else {
        None
    };

    // 6e. follow_path action サーバ。
    let active_cancel: Arc<Mutex<Option<Arc<AtomicBool>>>> = Arc::new(Mutex::new(None));

    let _server = node.create_action_server::<nav2_msgs::action::FollowPath, _>(
        "follow_path",
        move |requested_goal: RequestedGoal<nav2_msgs::action::FollowPath>| {
            let core = Arc::clone(&core);
            let latest_pose = Arc::clone(&latest_pose);
            let scan_queue = Arc::clone(&scan_queue);
            let cmd_pub = cmd_pub.clone();
            let active_cancel = Arc::clone(&active_cancel);
            let viz = viz.clone();

            async move {
                // ── プリエンプト: 前の追従を止め、自分の cancel を登録 ──
                let my_cancel = Arc::new(AtomicBool::new(false));
                {
                    let mut slot = active_cancel.lock().unwrap();
                    if let Some(prev) = slot.take() {
                        prev.store(true, Ordering::SeqCst);
                    }
                    *slot = Some(Arc::clone(&my_cancel));
                }

                let accepted = requested_goal.accept();
                let goal_pose =
                    accepted.goal().path.poses.last().map(|p| pose_view_from(&p.pose));
                let executing = accepted.execute();

                let Some(goal) = goal_pose else {
                    eprintln!("ERROR: vi_local_planner: follow_path goal has an empty path");
                    return executing
                        .aborted_with(nav2_msgs::action::FollowPath_Result::default());
                };
                eprintln!(
                    "vi_local_planner: follow to ({:.2}, {:.2})",
                    goal.x, goal.y
                );

                // ── 追従本体は専用スレッド (solve + 制御ループがブロックする) ──
                let feedback = executing.feedback_publisher();
                let (done_tx, done_rx) = futures::channel::oneshot::channel::<Outcome>();
                let core_t = Arc::clone(&core);
                let cancel_t = Arc::clone(&my_cancel);
                let latest_pose_t = Arc::clone(&latest_pose);
                let scan_queue_t = Arc::clone(&scan_queue);
                let cmd_pub_t = cmd_pub.clone();
                let viz_t = viz.clone();
                std::thread::spawn(move || {
                    let outcome = run_follow(
                        &core_t,
                        goal,
                        &cancel_t,
                        &latest_pose_t,
                        &scan_queue_t,
                        &cmd_pub_t,
                        &feedback,
                        tuning,
                        viz_t.as_deref(),
                    );
                    let _ = done_tx.send(outcome);
                });

                let mut done_rx = done_rx;
                match executing.until_cancel_requested(&mut done_rx).await {
                    Ok(Ok(Outcome::Reached)) => {
                        eprintln!("vi_local_planner: goal reached");
                        executing
                            .succeeded_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                    Ok(Ok(Outcome::Preempted)) => {
                        eprintln!("vi_local_planner: preempted by a newer follow_path goal");
                        executing
                            .aborted_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                    Ok(Ok(Outcome::Failed(reason))) => {
                        eprintln!("ERROR: vi_local_planner: {reason}");
                        executing
                            .aborted_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                    // 追従スレッドが結果を返さず死んだ (panic 等)。
                    Ok(Err(_)) => executing
                        .aborted_with(nav2_msgs::action::FollowPath_Result::default()),
                    // クライアントからの cancel: ループを止め、停止を待つ。
                    Err(rest) => {
                        my_cancel.store(true, Ordering::SeqCst);
                        let cancelling = executing.begin_cancelling();
                        let _ = rest.await;
                        eprintln!("vi_local_planner: cancelled by client");
                        cancelling
                            .cancelled_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                }
            }
        },
    )?;

    eprintln!(
        "vi_local_planner: ready (solver={}, action=follow_path, {}Hz)",
        params.solver, params.control_frequency
    );

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
