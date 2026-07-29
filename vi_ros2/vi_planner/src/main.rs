//! vi_planner entry point — Nav2 の planner_server と controller_server を
//! **1 ノードで同時に**置き換える全 Rust ノード。
//!
//! `compute_path_to_pose` (nav2_msgs) と `follow_path` (nav2_msgs) の両方を
//! 提供し、どちらも同じ `PlannerCore` = 同じ価値関数を読む。従来の
//! vi_global_planner + vi_local_planner の 2 プロセス構成では、同じ地図・同じ
//! ゴールに対して価値反復を別々に 2 回解いていた (走り出しまでの時間も常駐
//! メモリも 2 倍) 上に、広域側が返した `nav_msgs/Path` は狭域側が終端姿勢しか
//! 読まないので大部分が捨てられていた。ここでは solve をゴールごとに 1 回に
//! 畳み、経路はその価値関数の貪欲ロールアウト、追従は同じ価値関数の
//! ±1m ウィンドウ精密化として扱う。
//!
//! Boot order (vi_global_planner と同型):
//!   1. `Context::default_from_env` + basic executor + node 作成
//!   2. パラメータ宣言・検証 (アクション定数は vi_core と照合し fail-fast)
//!   3. `VI_THREADS` 設定 (vi_threads > 0 のとき)
//!   4. /map 受信 (transient_local, 初回メッセージまでブロック)
//!   5. PlannerCore 構築 (静的地図前提)
//!   6. pose / scan 購読 + cmd_vel パブリッシャ + 2 つの action サーバ配線
//!   7. executor.spin()
//!
//! ## ロック規律 (重要)
//!
//! 2 つのアクションが 1 つの `Mutex<PlannerCore>` を共有するので、追従ループが
//! ロックを握りっぱなしにすると BT の 1Hz リプラン (`compute_path_to_pose`) が
//! 追従の終わりまでブロックされる。そこで:
//!
//!   - solve 中はロックを保持する (cancel は `solve_chunk` ごとに観測するので
//!     プリエンプトは効く)。BT は ComputePathToPose → FollowPath の順に呼ぶので、
//!     実際に solve するのは広域側の 1 回だけになる。
//!   - 追従の制御ループは **tick ごとに取得・解放**する。10Hz・予算 40ms なので、
//!     1Hz のロールアウトは tick の隙間に入る。
//!
//! ## ゴール世代チェック
//!
//! 広域側が別ゴールを解くとキャッシュが差し替わる。追従スレッドは毎 tick
//! `is_cached_goal` で「自分のゴールの価値関数がまだ載っているか」を確認し、
//! 差し替わっていたらプリエンプト扱いで抜ける (別ゴールの方策でロボットを
//! 走らせない)。
//!
//! NOTE: rclrs API は ros2-rust/ros2_rust @ 2c6b926 (rclrs 0.7.0) — Docker
//! イメージがビルドする版 — に合わせている (vi_node / vi_global_planner と同一)。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context as ACtx, Result};

use vi_core::{ACTION_FW, ACTION_ROT, N_ACTIONS, N_THETA};
use vi_reference::bridge::{occupancy_view_to_vi_grid, OccupancyGridView, PoseView};
use vi_reference::msg::LaserScan as ViLaserScan;
use vi_reference::planner::PathPose;
use vi_reference::solvers::U64Solver;
use vi_reference::Action;

use vi_planner::core::{
    value_grid_of, BuildParams, Decision, PlanConfig, PlanError, PlanStats, PlannerCore,
};

use rclrs::*;

/// 無効レンジ (inf / NaN / 非正) の差し替え値 [m]。ローカルウィンドウ (±1m)
/// から十分遠く、セル座標化しても i32 に収まる。本家 C++ は float→int の
/// 未定義動作に頼っていたが、Rust では添字とビーム角の対応を保ったまま
/// ウィンドウ外へ飛ばして無害化する。
const INVALID_RANGE_M: f64 = 1.0e6;

/// 追従ループが `PlannerCore` のロックをこの回数だけ連続で取れなかったら停止指令を出す。
/// 1〜2 tick は同一ゴールのロールアウト (BT の 1Hz リプラン) との競合なので止めない。
/// control_frequency 10Hz なら 3 tick = 300ms。
const BUSY_TICKS_BEFORE_STOP: u32 = 3;

// ──────────────────────────────────────────────────────────────────────────────
// Parameters
// ──────────────────────────────────────────────────────────────────────────────

struct Params {
    // ── 共有 (価値関数の定義そのもの) ──
    solver: String,
    theta_cell_num: i64,
    safety_radius: f64,
    safety_radius_penalty: i64,
    goal_margin_radius: f64,
    goal_margin_theta_deg: f64,
    map_wait_sec: i64,
    allow_action_mismatch: bool,
    action_list: Vec<(String, f64, f64)>,
    unknown_as_obstacle: bool,
    vi_threads: i64,
    max_solve_iter: i64,
    solve_chunk: i64,
    goal_tolerance_xy: f64,
    goal_tolerance_deg: f64,
    pose_topic: String,
    global_frame: String,
    // ── 広域 (compute_path_to_pose) ──
    max_rollout_steps: i64,
    start_tolerance: f64,
    path_spacing: f64,
    // ── 狭域 (follow_path) ──
    scan_topic: String,
    control_frequency: f64,
    refine_budget_ms: i64,
    action_tolerance: f64,
    no_action_timeout_sec: f64,
    // ── 可視化 ──
    publish_value_function: bool,
    value_publish_interval_ms: i64,
    cost_drawing_threshold: i64,
    window_cost_drawing_threshold: i64,
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
    let unknown_as_obstacle = p!("unknown_as_obstacle", bool, true);
    let vi_threads = p!("vi_threads", i64, 0);
    let max_solve_iter = p!("max_solve_iter", i64, 1_000_000);
    let solve_chunk = p!("solve_chunk", i64, 64);
    let goal_tolerance_xy = p!("goal_tolerance_xy", f64, 0.25);
    let goal_tolerance_deg = p!("goal_tolerance_deg", f64, 10.0);
    let pose_topic = p!("pose_topic", Arc<str>, "mcl_pose".into()).to_string();
    let global_frame = p!("global_frame", Arc<str>, "map".into()).to_string();

    let max_rollout_steps = p!("max_rollout_steps", i64, 10_000);
    let start_tolerance = p!("start_tolerance", f64, 0.5);
    let path_spacing = p!("path_spacing", f64, 0.05);

    let scan_topic = p!("scan_topic", Arc<str>, "scan".into()).to_string();
    let control_frequency = p!("control_frequency", f64, 10.0);
    let refine_budget_ms = p!("refine_budget_ms", i64, 40);
    let action_tolerance = p!("action_tolerance", f64, 0.2);
    let no_action_timeout_sec = p!("no_action_timeout_sec", f64, 3.0);

    let publish_value_function = p!("publish_value_function", bool, true);
    let value_publish_interval_ms = p!("value_publish_interval_ms", i64, 500);
    let cost_drawing_threshold = p!("cost_drawing_threshold", i64, 60);
    let window_cost_drawing_threshold = p!("window_cost_drawing_threshold", i64, 60);

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
        unknown_as_obstacle,
        vi_threads,
        max_solve_iter,
        solve_chunk,
        goal_tolerance_xy,
        goal_tolerance_deg,
        pose_topic,
        global_frame,
        max_rollout_steps,
        start_tolerance,
        path_spacing,
        scan_topic,
        control_frequency,
        refine_budget_ms,
        action_tolerance,
        no_action_timeout_sec,
        publish_value_function,
        value_publish_interval_ms,
        cost_drawing_threshold,
        window_cost_drawing_threshold,
    })
}

/// vi_core のコンパイル時定数とパラメータを照合 (vi_node と同じ fail-fast)。
/// 併せて、このノードが扱えないソルバ (アウトオブコア) を弾く。
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
    let solver = U64Solver::from_name(&p.solver)
        .ok_or_else(|| anyhow!("unknown solver: {} (see U64Solver::from_name)", p.solver))?;
    // 追従はローカルウィンドウを ValueIterator::states に直接書き戻すので、
    // states を確保しない compact 経路には載らない。広域地図は
    // vi_global_planner + controller_server の構成を使うこと。
    if matches!(solver, U64Solver::Frontier2DSparseCompact { .. }) {
        return Err(anyhow!(
            "solver '{}' is out-of-core and has no dense state array, which the \
             follow_path control loop writes into. vi_planner cannot use it.\n\
             For maps that need the compact solver, run vi_global_planner (compute_path_to_pose \
             only) together with nav2_controller's controller_server instead.",
            p.solver
        ));
    }
    Ok(solver)
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

/// `PathPose` 列 → `nav_msgs::msg::Path`。
fn poses_to_path(poses: &[PathPose], frame_id: &str, stamp: (i32, u32)) -> nav_msgs::msg::Path {
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
// Value function visualization
// ──────────────────────────────────────────────────────────────────────────────

/// 可視化配信一式。`value_function` は両アクションの solve が共有する θ=0 全域
/// スライス (価値関数は 1 本しかないので、旧 vi_local_planner の
/// `local_value_function` に相当するトピックは無い)。
struct Viz {
    /// θ=0 全域スライス (solve の途中経過 + 完了時)。
    vf_pub: Publisher<nav_msgs::msg::OccupancyGrid>,
    /// ローカルウィンドウの現在方位スライス (追従中、スキャン penalty 込み)。
    win_pub: Publisher<nav_msgs::msg::OccupancyGrid>,
    clock: Clock,
    frame_id: String,
    /// `value_function` のスケール上限 [ステップ数≒秒]。
    threshold_steps: u64,
    /// `local_window_value` のスケール上限 (窓は近傍だけなので別に持つ)。
    window_threshold_steps: u64,
    /// 配信間隔。0 で solve 完了時のみ。
    interval: Duration,
}

impl Viz {
    fn stamp(&self) -> (i32, u32) {
        self.clock.now().to_sec_nanosec().unwrap_or((0, 0))
    }

    /// 間引き判定。`last` を更新した場合のみ true。
    fn due(&self, last: &mut Option<Instant>) -> bool {
        if self.interval.is_zero() {
            return false;
        }
        if last.map_or(false, |t| t.elapsed() < self.interval) {
            return false;
        }
        *last = Some(Instant::now());
        true
    }
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

/// 1 ゴールぶんの追従ループ。
///
/// solve 中だけロックを保持し、制御ループは **tick ごとに取得・解放**する
/// (compute_path_to_pose を待たせないため; ファイル冒頭のロック規律を参照)。
#[allow(clippy::too_many_arguments)]
fn run_follow(
    core: &Mutex<PlannerCore>,
    goal: PoseView,
    cancel: &AtomicBool,
    latest_pose: &Mutex<Option<PoseView>>,
    scan_queue: &Mutex<Vec<ViLaserScan>>,
    cmd_pub: &Publisher<geometry_msgs::msg::Twist>,
    feedback: &FeedbackPublisher<nav2_msgs::action::FollowPath>,
    tuning: FollowTuning,
    viz: Option<&Viz>,
) -> Outcome {
    // ── 1. 価値関数の用意 (広域側が既に解いていれば何もしない) ──
    {
        let mut core = core.lock().unwrap();
        // solve が要るときだけ止める。BT が同じ経路を再送するたびに 0 速度を
        // 挟むと 1Hz のリプラン周期で走行がぎくしゃくするので、キャッシュ
        // ヒット時は現在の指令を保ったまま次の tick へ引き継ぐ。
        if !core.is_cached_goal(goal) {
            stop_cmd(cmd_pub);
        }

        let t0 = Instant::now();
        let mut last_viz: Option<Instant> = None;
        let prepared = core.prepare_goal_with_progress(goal, cancel, &mut |vi| {
            let Some(v) = viz else { return };
            if !v.due(&mut last_viz) {
                return;
            }
            let g = value_grid_of(vi, v.threshold_steps);
            let _ = v.vf_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
        });
        match prepared {
            Ok(stats) => {
                if stats.solved_now {
                    eprintln!(
                        "vi_planner: value function solved in {:.2}s (iters={}) [follow_path]",
                        t0.elapsed().as_secs_f64(),
                        stats.iters
                    );
                    // 収束後の最終状態を必ず 1 回配信する。
                    if let Some(v) = viz {
                        if let Some(g) = core.value_grid(v.threshold_steps) {
                            let _ = v.vf_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
                        }
                    }
                }
            }
            Err(PlanError::Cancelled) => return Outcome::Preempted,
            Err(e) => return Outcome::Failed(e.to_string()),
        }
    } // ここでロックを解放 — 以降は tick ごとに取り直す。

    // ── 2. 制御ループ ──
    let mut failure_ticks = 0u32;
    // ロックを連続で取れなかった tick 数 (下の try_lock を参照)。
    let mut busy_ticks = 0u32;
    let mut last_viz: Option<Instant> = None;
    loop {
        let tick_start = Instant::now();
        if cancel.load(Ordering::Relaxed) {
            stop_cmd(cmd_pub);
            return Outcome::Preempted;
        }

        let pose = *latest_pose.lock().unwrap();
        let Some(pose) = pose else {
            stop_cmd(cmd_pub);
            failure_ticks += 1;
            if failure_ticks >= tuning.failure_ticks_limit {
                return Outcome::Failed("no robot pose for too long".into());
            }
            if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
                std::thread::sleep(rest);
            }
            continue;
        };

        // ── ロックを取るのはこのブロックだけ ──
        //
        // `lock()` ではなく `try_lock()` にしてある。広域側が **別ゴール** の
        // 計画要求を受けると、その solve は数秒〜数十秒ロックを握り続ける。
        // ここでブロックすると、その間ずっと直前の速度指令が出たままロボットが
        // 走り続けることになる (velocity_smoother のタイムアウト任せにしない)。
        let guard = match core.try_lock() {
            Ok(g) => Some(g),
            Err(std::sync::TryLockError::WouldBlock) => None,
            // 他スレッドの panic で毒された場合も、価値関数自体は
            // ValueIterator の内部状態として一貫しているので続行する。
            Err(std::sync::TryLockError::Poisoned(p)) => Some(p.into_inner()),
        };
        let Some(mut core) = guard else {
            // 1〜2 tick 取れないのは同一ゴールのロールアウト待ち (BT の 1Hz
            // リプランはキャッシュヒットでも rollout + densify をロック内で回す)。
            // ここで毎回止めると、取り除いたはずの「1 秒ごとに 0 速度が挟まる」
            // 挙動が戻ってしまうので、直前の指令を保ったまま次の tick を待つ。
            // 連続で取れない = 本当に長い solve が走っているので、そのときだけ止める。
            busy_ticks += 1;
            if busy_ticks >= BUSY_TICKS_BEFORE_STOP {
                stop_cmd(cmd_pub);
            }
            if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
                std::thread::sleep(rest);
            }
            continue;
        };
        busy_ticks = 0;

        // scan の取り込みはロックを取れた tick でだけ行う (取れなかった tick で
        // 捨てると、待っている間のスキャンが丸ごと失われる)。ビジーが続くと
        // キューは 10 件の上限に張り付き、次に取れた tick でまとめて同一 pose の
        // もとに注入される — スキャンコールバック側の上限はこのための蓋。
        let scans = std::mem::take(&mut *scan_queue.lock().unwrap());

        let (decision, dist, window_grid) = {
            // 広域側が別ゴールを解いてキャッシュを差し替えていたら、その方策で
            // 走らせるわけにはいかない。既存のプリエンプトと同じ扱いで抜ける。
            if !core.is_cached_goal(goal) {
                stop_cmd(cmd_pub);
                eprintln!("vi_planner: follow preempted — the cached goal was replaced");
                return Outcome::Preempted;
            }
            core.set_window(pose);
            for scan in &scans {
                core.observe_scan(scan, pose);
            }
            core.refine_for(tuning.refine_budget);

            // 可視化グリッドの作成はロック内 (states を読む) / 配信は外。
            let mut window_grid = None;
            if let Some(v) = viz {
                if v.due(&mut last_viz) {
                    window_grid = core.window_value_grid(pose, v.window_threshold_steps);
                }
            }
            (core.decide(pose), core.goal_distance(pose.x, pose.y), window_grid)
        };
        // ここで手放す。スコープ末尾まで持つと sleep の間もロックを握ったままに
        // なり、広域側の計画要求がほぼ通らなくなる。
        drop(core);

        // 配信はロックの外で (可視化 1 枚は 100 万セル級になり得る)。
        if let (Some(v), Some(g)) = (viz, window_grid) {
            let _ = v.win_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
        }

        // 制御ループは tick ごとにロックを手放すので、プリエンプトした新しい
        // 追従スレッドと一瞬だけ並走し得る (旧 vi_local_planner は追従中ずっと
        // ロックを握っていたので起こらなかった)。古いループの指令が新しい
        // ループの指令を上書きしないよう、publish の直前でもう一度観測する。
        if cancel.load(Ordering::Relaxed) {
            stop_cmd(cmd_pub);
            return Outcome::Preempted;
        }

        let mut speed = 0.0f32;
        match decision {
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

        let _ = feedback.publish(nav2_msgs::action::FollowPath_Feedback {
            distance_to_goal: dist.unwrap_or(f64::NAN) as f32,
            speed,
        });

        if failure_ticks >= tuning.failure_ticks_limit {
            stop_cmd(cmd_pub);
            return Outcome::Failed("no applicable action for too long".into());
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
    let node = executor.create_node("vi_planner").context("create vi_planner")?;

    // 2. Parameters + validation.
    let params = read_params(&node).context("reading parameters")?;
    let solver = validate(&params).context("validating parameters")?;

    // 3. sparse 系ソルバのスレッド数 (n_threads() が VI_THREADS を読む)。
    if params.vi_threads > 0 {
        std::env::set_var("VI_THREADS", params.vi_threads.to_string());
    }

    // 4. Wait for /map.
    let map_msg =
        wait_for_map(&node, &mut executor, params.map_wait_sec).context("waiting for /map")?;
    eprintln!(
        "vi_planner: got map {}x{} @{}m",
        map_msg.info.width, map_msg.info.height, map_msg.info.resolution
    );

    // 5. PlannerCore (広域・狭域が共有する唯一の価値関数)。
    let grid_view = OccupancyGridView {
        width: map_msg.info.width,
        height: map_msg.info.height,
        resolution: map_msg.info.resolution as f64,
        origin_x: map_msg.info.origin.position.x,
        origin_y: map_msg.info.origin.position.y,
        data: &map_msg.data[..],
    };
    let vi_grid = occupancy_view_to_vi_grid(&grid_view, params.unknown_as_obstacle);
    let nstates =
        vi_grid.width as usize * vi_grid.height as usize * params.theta_cell_num as usize;
    eprintln!(
        "vi_planner: planner grid {}x{} @{:.3}m x{} theta = {} states",
        vi_grid.width, vi_grid.height, vi_grid.resolution, params.theta_cell_num, nstates,
    );
    let start_tolerance_cells = (params.start_tolerance / vi_grid.resolution).ceil() as i32;
    let action_tolerance_cells = (params.action_tolerance / vi_grid.resolution).ceil() as i32;

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
        goal_tolerance_xy: params.goal_tolerance_xy,
        goal_tolerance_deg: params.goal_tolerance_deg,
        max_rollout_steps: params.max_rollout_steps.max(1) as usize,
        start_tolerance_cells,
        path_spacing: params.path_spacing,
        action_tolerance_cells,
    };
    let core = Arc::new(Mutex::new(PlannerCore::new(build, cfg)));

    // 6a. 自己位置トピック購読 (tf2 代替)。
    let latest_pose: Arc<Mutex<Option<PoseView>>> = Arc::new(Mutex::new(None));
    let latest_pose_w = Arc::clone(&latest_pose);
    let _pose_sub = node
        .create_subscription::<geometry_msgs::msg::PoseWithCovarianceStamped, _>(
            params.pose_topic.as_str().keep_last(1),
            move |msg: geometry_msgs::msg::PoseWithCovarianceStamped| {
                *latest_pose_w.lock().unwrap() = Some(pose_view_from(&msg.pose.pose));
            },
        )?;

    // 6b. スキャン購読 (sensor QoS = best effort)。tick 間に届いた分を貯めて
    //     制御ループが順に消化する。
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
    //     リマップし velocity_smoother を経由させる)。
    let cmd_pub = node.create_publisher::<geometry_msgs::msg::Twist>("cmd_vel".keep_last(1))?;

    // 6d. 可視化。価値関数は 1 本なので value_function も 1 本。
    let viz: Option<Arc<Viz>> = if params.publish_value_function {
        Some(Arc::new(Viz {
            vf_pub: node.create_publisher::<nav_msgs::msg::OccupancyGrid>(
                "value_function".reliable().transient_local().keep_last(1),
            )?,
            win_pub: node.create_publisher::<nav_msgs::msg::OccupancyGrid>(
                "local_window_value".reliable().transient_local().keep_last(1),
            )?,
            clock: node.get_clock(),
            frame_id: params.global_frame.clone(),
            threshold_steps: params.cost_drawing_threshold.max(0) as u64,
            window_threshold_steps: params.window_cost_drawing_threshold.max(0) as u64,
            interval: Duration::from_millis(params.value_publish_interval_ms.max(0) as u64),
        }))
    } else {
        None
    };

    // 進行中の solve / 追従をプリエンプトするための cancel フラグ置き場。
    // 2 つのアクションは互いを止めない (広域の再計画で追従が死んではいけないし、
    // 追従中でも再計画は受け付ける) ので、スロットは別々に持つ。
    let plan_cancel: Arc<Mutex<Option<Arc<AtomicBool>>>> = Arc::new(Mutex::new(None));
    let follow_cancel: Arc<Mutex<Option<Arc<AtomicBool>>>> = Arc::new(Mutex::new(None));

    let node_clock = node.get_clock();
    let frame_id = params.global_frame.clone();
    // ロールアウト固着時のヒント表示用 (safety_radius_penalty [秒/セル], safety_radius [m])。
    let params_hint = (params.safety_radius_penalty, params.safety_radius);

    // 6e. compute_path_to_pose action サーバ (planner_server の置き換え)。
    let _plan_server = {
        let core = Arc::clone(&core);
        let latest_pose = Arc::clone(&latest_pose);
        let plan_cancel = Arc::clone(&plan_cancel);
        let viz = viz.clone();
        let frame_id = frame_id.clone();
        let node_clock = node_clock.clone();

        node.create_action_server::<nav2_msgs::action::ComputePathToPose, _>(
            "compute_path_to_pose",
            move |requested_goal: RequestedGoal<nav2_msgs::action::ComputePathToPose>| {
                let core = Arc::clone(&core);
                let latest_pose = Arc::clone(&latest_pose);
                let plan_cancel = Arc::clone(&plan_cancel);
                let viz = viz.clone();
                let frame_id = frame_id.clone();
                let node_clock = node_clock.clone();

                async move {
                    // ── プリエンプト: 前の計画を止め、自分の cancel を登録 ──
                    let my_cancel = Arc::new(AtomicBool::new(false));
                    {
                        let mut slot = plan_cancel.lock().unwrap();
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
                            "ERROR: vi_planner: no robot pose available (use_start=false and \
                             nothing received on the pose topic yet)"
                        );
                        return executing.aborted_with(
                            nav2_msgs::action::ComputePathToPose_Result::default(),
                        );
                    };

                    eprintln!(
                        "vi_planner: plan ({:.2}, {:.2}) -> ({:.2}, {:.2})",
                        start.x, start.y, goal.x, goal.y
                    );

                    // ── 計画本体は専用スレッド (solve は数秒〜数十秒ブロック) ──
                    let t0 = Instant::now();
                    type PlanOutcome =
                        std::result::Result<(Vec<PathPose>, PlanStats), PlanError>;
                    let (done_tx, done_rx) = futures::channel::oneshot::channel::<PlanOutcome>();
                    let core_t = Arc::clone(&core);
                    let viz_t = viz.clone();
                    let frame_t = frame_id.clone();
                    std::thread::spawn(move || {
                        let mut core = core_t.lock().unwrap();
                        let mut last_viz: Option<Instant> = None;
                        let result =
                            core.plan_with_progress(start, goal, &my_cancel, &mut |vi| {
                                let Some(v) = &viz_t else { return };
                                if !v.due(&mut last_viz) {
                                    return;
                                }
                                let g = value_grid_of(vi, v.threshold_steps);
                                let _ =
                                    v.vf_pub.publish(ros_grid_from(&g, &frame_t, v.stamp()));
                            });
                        // solve が走った場合のみ完成形を配信し直す。
                        if let (Ok((_, stats)), Some(v)) = (&result, &viz_t) {
                            if stats.solved_now {
                                if let Some(g) = core.value_grid(v.threshold_steps) {
                                    let _ = v
                                        .vf_pub
                                        .publish(ros_grid_from(&g, &frame_t, v.stamp()));
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
                            let mut result =
                                nav2_msgs::action::ComputePathToPose_Result::default();
                            result.path = poses_to_path(&poses, &frame_id, stamp);
                            result.planning_time.sec = dt.as_secs() as i32;
                            result.planning_time.nanosec = dt.subsec_nanos();
                            executing.succeeded_with(result)
                        }
                        Ok(Err(e)) => {
                            eprintln!("ERROR: vi_planner: {e}");
                            // ロールアウトの固着は「価値関数の局所的なゆらぎ >
                            // 1 手の進捗」で起きる。safety_radius_penalty (秒/セル)
                            // が 1 手のコスト 1 秒に対して大きすぎ、かつ経路の
                            // 大半がペナルティ域に入る地図で顕在化する。
                            if matches!(
                                e,
                                PlanError::Rollout(
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
        )?
    };

    // 6f. follow_path action サーバ (controller_server の置き換え)。
    let period = Duration::from_secs_f64(1.0 / params.control_frequency);
    let tuning = FollowTuning {
        period,
        refine_budget: Duration::from_millis(params.refine_budget_ms.max(0) as u64),
        failure_ticks_limit: (params.no_action_timeout_sec.max(0.0) * params.control_frequency)
            .ceil()
            .max(1.0) as u32,
    };

    let _follow_server = node.create_action_server::<nav2_msgs::action::FollowPath, _>(
        "follow_path",
        move |requested_goal: RequestedGoal<nav2_msgs::action::FollowPath>| {
            let core = Arc::clone(&core);
            let latest_pose = Arc::clone(&latest_pose);
            let scan_queue = Arc::clone(&scan_queue);
            let cmd_pub = cmd_pub.clone();
            let follow_cancel = Arc::clone(&follow_cancel);
            let viz = viz.clone();

            async move {
                // ── プリエンプト: 前の追従を止め、自分の cancel を登録 ──
                let my_cancel = Arc::new(AtomicBool::new(false));
                {
                    let mut slot = follow_cancel.lock().unwrap();
                    if let Some(prev) = slot.take() {
                        prev.store(true, Ordering::SeqCst);
                    }
                    *slot = Some(Arc::clone(&my_cancel));
                }

                let accepted = requested_goal.accept();
                // ゴール姿勢は path 終端 (controller_id / goal_checker_id は無視)。
                // この path は同じノードの compute_path_to_pose が返したもので、
                // 追従自体は path ではなく価値関数の方策に従う。
                let goal_pose = accepted.goal().path.poses.last().map(|p| pose_view_from(&p.pose));
                let executing = accepted.execute();

                let Some(goal) = goal_pose else {
                    eprintln!("ERROR: vi_planner: follow_path goal has an empty path");
                    return executing
                        .aborted_with(nav2_msgs::action::FollowPath_Result::default());
                };
                eprintln!("vi_planner: follow to ({:.2}, {:.2})", goal.x, goal.y);

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
                        eprintln!("vi_planner: goal reached");
                        executing.succeeded_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                    Ok(Ok(Outcome::Preempted)) => {
                        eprintln!("vi_planner: preempted by a newer follow_path goal");
                        executing.aborted_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                    Ok(Ok(Outcome::Failed(reason))) => {
                        eprintln!("ERROR: vi_planner: {reason}");
                        executing.aborted_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                    // 追従スレッドが結果を返さず死んだ (panic 等)。
                    Ok(Err(_)) => {
                        executing.aborted_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                    // クライアントからの cancel: ループを止め、停止を待つ。
                    Err(rest) => {
                        my_cancel.store(true, Ordering::SeqCst);
                        let cancelling = executing.begin_cancelling();
                        let _ = rest.await;
                        eprintln!("vi_planner: cancelled by client");
                        cancelling
                            .cancelled_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                }
            }
        },
    )?;

    eprintln!(
        "vi_planner: ready (solver={}, actions=compute_path_to_pose + follow_path, {}Hz)",
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
