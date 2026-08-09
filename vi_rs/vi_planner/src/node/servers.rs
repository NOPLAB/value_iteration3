//! アクションサーバの配線 — `compute_path_to_pose` / `follow_path` は常に、
//! `navigate_to_pose` / `follow_waypoints` と `/goal_pose` 購読は `standalone`
//! のときだけ。
//!
//! どのサーバも中身は同じ形をしている: プリエンプト ([`preempt`]) → 受理 →
//! **専用スレッドで本体** (solve も追従も数秒〜数十秒ブロックするので、rclrs の
//! futures エグゼキュータの上では回せない) → oneshot で結果を受けて終端。
//!
//! 追従を回す 3 つ (`follow_path` / `navigate_to_pose` / `follow_waypoints`) は
//! **cancel の席を共有する** — 同じ 1 台を走らせるので、どれが来ても前のものは
//! 止まらなければならない。広域 (`compute_path_to_pose`) の席だけ別 (再計画で
//! 追従が死んではいけないし、追従中でも再計画は受け付ける)。

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::Result;

use vi_lib::bridge::PoseView;
use vi_lib::planner::PathPose;

use vi_planner::core::{lock, PlanError, PlanStats, Prefetcher};

use rclrs::*;

use super::follow_loop::{
    run_follow, run_goal, run_settle, FollowProgress, FollowTuning, Outcome, RetryTuning,
};
use super::handles::Handles;
use super::msg::{pose_view_from, poses_to_path, ros_grid_from, stop_cmd};

/// アクションサーバと `/goal_pose` 購読が共有する配線材料。ゴールごとに変わる
/// ものは持たない (それは各サーバがゴール受理のたびに作る)。
pub struct Wiring {
    pub handles: Arc<Handles>,
    /// 進行中の solve をプリエンプトする席 (広域)。
    pub plan_cancel: Arc<Mutex<Option<Arc<AtomicBool>>>>,
    /// 追従を回す 3 つのサーバが**共有する**席。
    pub follow_cancel: Arc<Mutex<Option<Arc<AtomicBool>>>>,
    pub frame_id: String,
    pub clock: Clock,
    pub tuning: FollowTuning,
    pub retry: RetryTuning,
    /// ウェイポイント先読み。`follow_waypoints` は受け取った順路をそのまま渡す。
    pub prefetch: Option<Prefetcher>,
    /// follow_waypoints: 1 点失敗したら残りを諦めるか。
    pub stop_on_failure: bool,
    /// 点と点の間の待ち (`run_settle` — 単に寝るのではなく場を更新する)。
    pub pause: Duration,
    /// ロールアウト固着時のヒント表示用 (safety_radius_penalty [秒/セル], safety_radius [m])。
    pub hint: (i64, f64),
}

/// プリエンプト: 前のゴールの cancel を立て、自分の cancel をスロットへ置く。
/// 追従を回す 3 つのサーバはスロットを共有する — 同じ 1 台を走らせるので、
/// どれが来ても前のものは止まらなければならない。
pub fn preempt(slot: &Mutex<Option<Arc<AtomicBool>>>) -> Arc<AtomicBool> {
    let my = Arc::new(AtomicBool::new(false));
    let mut slot = lock(slot);
    if let Some(prev) = slot.take() {
        prev.store(true, Ordering::SeqCst);
    }
    *slot = Some(Arc::clone(&my));
    my
}

/// 巡回 (`follow_waypoints`) の終わり方。飛ばした点そのものは別に返す。
enum TourOutcome {
    /// 最後の点まで回った (途中で失敗した点があっても、進み続けたならこれ)。
    Done,
    /// 中断 (`stop_on_failure: true` での停止 / 新しいゴールか cancel での
    /// プリエンプト)。中身はそのままログへ出すメッセージ。
    Aborted(&'static str),
}

/// `NavigateToPose` の Feedback を 1 tick ぶん作る。
///
/// `builtin_interfaces` を直接 use せずに済むよう、既定値から組み立てて
/// フィールドに代入する形にしてある (Duration の型名を書かなくてよい)。
fn nav_feedback(
    p: &FollowProgress,
    frame_id: &str,
    clock: &Clock,
    started: Instant,
    retries: u64,
) -> nav2_msgs::action::NavigateToPose_Feedback {
    let mut fb = nav2_msgs::action::NavigateToPose_Feedback::default();
    let (sec, nanosec) = clock.now().to_sec_nanosec().unwrap_or((0, 0));
    fb.current_pose.header.frame_id = frame_id.into();
    fb.current_pose.header.stamp.sec = sec;
    fb.current_pose.header.stamp.nanosec = nanosec;
    fb.current_pose.pose.position.x = p.pose.x;
    fb.current_pose.pose.position.y = p.pose.y;
    fb.current_pose.pose.orientation.z = (p.pose.yaw_rad / 2.0).sin();
    fb.current_pose.pose.orientation.w = (p.pose.yaw_rad / 2.0).cos();
    let elapsed = started.elapsed();
    fb.navigation_time.sec = elapsed.as_secs() as i32;
    fb.navigation_time.nanosec = elapsed.subsec_nanos();
    // 投げ直した回数。BT 構成の「リカバリを何回回したか」と同じ枠に入れる
    // (RViz と daifuku_rqt がこの数字を出す)。
    fb.number_of_recoveries = retries.min(i16::MAX as u64) as i16;
    fb.distance_remaining = p.distance_remaining.unwrap_or(f64::NAN) as f32;
    // estimated_time_remaining は出さない (VI の値は秒だが、それは行動 1 手 =
    // 1 秒という模型の中の秒で、実時間ではない。埋めると嘘になる)。
    fb
}

/// compute_path_to_pose サーバ (planner_server の置き換え)。
pub fn plan_server(
    node: &Node,
    w: &Wiring,
) -> Result<ActionServer<nav2_msgs::action::ComputePathToPose>> {
    let handles = Arc::clone(&w.handles);
    let plan_cancel = Arc::clone(&w.plan_cancel);
    let frame_id = w.frame_id.clone();
    let node_clock = w.clock.clone();
    let params_hint = w.hint;

    Ok(node.create_action_server::<nav2_msgs::action::ComputePathToPose, _>(
        "compute_path_to_pose",
        move |requested_goal: RequestedGoal<nav2_msgs::action::ComputePathToPose>| {
            let h = Arc::clone(&handles);
            let plan_cancel = Arc::clone(&plan_cancel);
            let frame_id = frame_id.clone();
            let node_clock = node_clock.clone();

            async move {
                // 前の計画を止め、自分の cancel を登録。
                let my_cancel = preempt(&plan_cancel);

                let accepted = requested_goal.accept();
                let goal_msg = accepted.goal();
                let goal = pose_view_from(&goal_msg.goal.pose);
                let start = if goal_msg.use_start {
                    Some(pose_view_from(&goal_msg.start.pose))
                } else {
                    *lock(&h.latest_pose)
                };
                let executing = accepted.execute();

                let Some(start) = start else {
                    eprintln!(
                        "ERROR: vi_planner: no robot pose available (use_start=false and \
                         nothing received on the pose topic yet)"
                    );
                    return executing
                        .aborted_with(nav2_msgs::action::ComputePathToPose_Result::default());
                };

                eprintln!(
                    "vi_planner: plan ({:.2}, {:.2}) -> ({:.2}, {:.2})",
                    start.x, start.y, goal.x, goal.y
                );

                // ── 計画本体は専用スレッド (solve は数秒〜数十秒ブロック) ──
                let t0 = Instant::now();
                type PlanOutcome = std::result::Result<(Vec<PathPose>, PlanStats), PlanError>;
                let (done_tx, done_rx) = futures::channel::oneshot::channel::<PlanOutcome>();
                let h_t = Arc::clone(&h);
                let frame_t = frame_id.clone();
                std::thread::spawn(move || {
                    let mut core = lock(&h_t.core);
                    let mut last_viz: Option<Instant> = None;
                    let result = core.plan_with_progress(start, goal, &my_cancel, &mut |vi| {
                        let Some(v) = h_t.viz.as_ref() else { return };
                        if !v.due(&mut last_viz) {
                            return;
                        }
                        let g = vi_planner::core::value_grid_on(vi, v.threshold_steps);
                        let _ = v.vf_pub.publish(ros_grid_from(&g, &frame_t, v.stamp()));
                    });
                    // solve が走った場合のみ完成形を配信し直す。
                    if let (Ok((_, stats)), Some(v)) = (&result, h_t.viz.as_ref()) {
                        if stats.solved_now {
                            if let Some(g) = core.value_grid(v.threshold_steps) {
                                let _ =
                                    v.vf_pub.publish(ros_grid_from(&g, &frame_t, v.stamp()));
                            }
                        }
                    }
                    let _ = done_tx.send(result);
                });

                match done_rx.await {
                    Ok(Ok((poses, stats))) => {
                        let dt = t0.elapsed();
                        eprintln!(
                            "vi_planner: path with {} poses in {:.2}s \
                             (solved_now={}, iters={}{}{})",
                            stats.poses,
                            dt.as_secs_f64(),
                            stats.solved_now,
                            stats.iters,
                            // 先読みが効いた回。solve を丸ごと飛ばしたので、
                            // ここが出ている間は点の切り替わりで機体が止まらない。
                            if stats.adopted { ", prefetched" } else { "" },
                            // まだ解き終わっていない場で答えた回 (early_start)。経路の外は
                            // 未確定なので、機体が外れると解き直しが要る。
                            if stats.partial { ", still solving" } else { "" }
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
                            PlanError::Rollout(vi_lib::planner::RolloutStatus::LoopDetected)
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
                        executing
                            .aborted_with(nav2_msgs::action::ComputePathToPose_Result::default())
                    }
                    Err(_) => executing
                        .aborted_with(nav2_msgs::action::ComputePathToPose_Result::default()),
                }
            }
        },
    )?)
}

/// navigate_to_pose サーバ (bt_navigator + behavior_server の置き換え)。
/// **standalone のときだけ立てる** — Nav2 構成で立てると bt_navigator と
/// 2 つになり、クライアントは先に見つけたほうへ繋ぐ (どちらに繋がったかは
/// どこにも出ないので、症状は「ときどき挙動が違う」になる)。
pub fn nav_to_pose_server(
    node: &Node,
    w: &Wiring,
) -> Result<ActionServer<nav2_msgs::action::NavigateToPose>> {
    let handles = Arc::clone(&w.handles);
    let follow_cancel = Arc::clone(&w.follow_cancel);
    let frame_id = w.frame_id.clone();
    let node_clock = w.clock.clone();
    let tuning = w.tuning;
    let retry = w.retry;

    Ok(node.create_action_server::<nav2_msgs::action::NavigateToPose, _>(
        "navigate_to_pose",
        move |requested_goal: RequestedGoal<nav2_msgs::action::NavigateToPose>| {
            let h = Arc::clone(&handles);
            let follow_cancel = Arc::clone(&follow_cancel);
            let frame_id = frame_id.clone();
            let node_clock = node_clock.clone();

            async move {
                // 前の追従を止め、自分の cancel を登録。
                let my_cancel = preempt(&follow_cancel);

                let accepted = requested_goal.accept();
                let goal = pose_view_from(&accepted.goal().pose.pose);
                let executing = accepted.execute();
                eprintln!("vi_planner: navigate to ({:.2}, {:.2})", goal.x, goal.y);

                let feedback = executing.feedback_publisher();
                let (done_tx, done_rx) = futures::channel::oneshot::channel::<Outcome>();
                let cancel_t = Arc::clone(&my_cancel);
                std::thread::spawn(move || {
                    let ctx = h.follow_ctx(tuning, true);
                    let retries = AtomicU64::new(0);
                    let t0 = Instant::now();
                    let outcome = run_goal(&ctx, goal, &cancel_t, retry, &retries, &|p| {
                        let _ = feedback.publish(nav_feedback(
                            p,
                            &frame_id,
                            &node_clock,
                            t0,
                            retries.load(Ordering::Relaxed),
                        ));
                    });
                    let _ = done_tx.send(outcome);
                });

                let mut done_rx = done_rx;
                match executing.until_cancel_requested(&mut done_rx).await {
                    Ok(Ok(Outcome::Reached)) => {
                        eprintln!("vi_planner: goal reached");
                        executing
                            .succeeded_with(nav2_msgs::action::NavigateToPose_Result::default())
                    }
                    Ok(Ok(Outcome::Preempted)) => {
                        eprintln!("vi_planner: preempted by a newer goal");
                        executing
                            .aborted_with(nav2_msgs::action::NavigateToPose_Result::default())
                    }
                    Ok(Ok(Outcome::Failed(reason))) => {
                        eprintln!("ERROR: vi_planner: {reason}");
                        executing
                            .aborted_with(nav2_msgs::action::NavigateToPose_Result::default())
                    }
                    Ok(Err(_)) => executing
                        .aborted_with(nav2_msgs::action::NavigateToPose_Result::default()),
                    Err(rest) => {
                        my_cancel.store(true, Ordering::SeqCst);
                        let cancelling = executing.begin_cancelling();
                        let _ = rest.await;
                        eprintln!("vi_planner: cancelled by client");
                        cancelling
                            .cancelled_with(nav2_msgs::action::NavigateToPose_Result::default())
                    }
                }
            }
        },
    )?)
}

/// `/goal_pose` 購読 (bt_navigator の goal_pose→NavigateToPose 変換の置き換え)。
///
/// RViz の「Nav2 Goal」も「2D Goal Pose」も PoseStamped をこのトピックへ出す
/// だけで、アクションに変換するのは bt_navigator だった — standalone はそれも
/// 置き換えているのでここで直接受ける。経路はアクションと同じ [`run_goal`]、
/// 違いは feedback/result の宛先が無いことだけ (プリエンプトの席も同じ 1 つ)。
pub fn goal_pose_sub(
    node: &Node,
    w: &Wiring,
) -> Result<Subscription<geometry_msgs::msg::PoseStamped>> {
    let handles = Arc::clone(&w.handles);
    let follow_cancel = Arc::clone(&w.follow_cancel);
    let tuning = w.tuning;
    let retry = w.retry;

    Ok(node.create_subscription::<geometry_msgs::msg::PoseStamped, _>(
        "goal_pose".keep_last(1),
        move |msg: geometry_msgs::msg::PoseStamped| {
            let goal = pose_view_from(&msg.pose);
            eprintln!("vi_planner: goal_pose ({:.2}, {:.2})", goal.x, goal.y);
            let my_cancel = preempt(&follow_cancel);
            let h = Arc::clone(&handles);
            std::thread::spawn(move || {
                let ctx = h.follow_ctx(tuning, true);
                match run_goal(&ctx, goal, &my_cancel, retry, &AtomicU64::new(0), &|_| {}) {
                    Outcome::Reached => eprintln!("vi_planner: goal reached"),
                    Outcome::Preempted => eprintln!("vi_planner: preempted by a newer goal"),
                    Outcome::Failed(reason) => eprintln!("ERROR: vi_planner: {reason}"),
                }
            });
        },
    )?)
}

/// follow_waypoints サーバ (nav2_waypoint_follower の置き換え)。
/// **順路は配列の順**に回る (距離で並べ替えたりはしない。nav2 側も同じ)。
///
/// ここで順路を丸ごと受け取れることには、単に 1 ノード減る以上の意味がある:
/// 先読み (`waypoint_prefetch`) は「次の点」を知る手立てが `waypoint_topic` の
/// latch しか無く、そこへ出すものがいない構成では**エラーも警告も出ないまま
/// 何も解かなかった**。ゴールと同じ経路で順路が入るので、その穴が塞がる。
pub fn follow_waypoints_server(
    node: &Node,
    w: &Wiring,
) -> Result<ActionServer<nav2_msgs::action::FollowWaypoints>> {
    let handles = Arc::clone(&w.handles);
    let follow_cancel = Arc::clone(&w.follow_cancel);
    let prefetch = w.prefetch.clone();
    let stop_on_failure = w.stop_on_failure;
    let pause = w.pause;
    let tuning = w.tuning;
    let retry = w.retry;

    Ok(node.create_action_server::<nav2_msgs::action::FollowWaypoints, _>(
        "follow_waypoints",
        move |requested_goal: RequestedGoal<nav2_msgs::action::FollowWaypoints>| {
            let h = Arc::clone(&handles);
            let follow_cancel = Arc::clone(&follow_cancel);
            let prefetch = prefetch.clone();

            async move {
                let my_cancel = preempt(&follow_cancel);

                let accepted = requested_goal.accept();
                let goals: Vec<PoseView> =
                    accepted.goal().poses.iter().map(|p| pose_view_from(&p.pose)).collect();
                let executing = accepted.execute();

                if goals.is_empty() {
                    eprintln!("ERROR: vi_planner: follow_waypoints goal has no poses");
                    return executing
                        .aborted_with(nav2_msgs::action::FollowWaypoints_Result::default());
                }
                eprintln!("vi_planner: waypoint tour of {} poses", goals.len());
                // 先読みへ順路をそのまま渡す (トピック経由と同じ受け口)。
                if let Some(pf) = prefetch.as_ref() {
                    pf.set_waypoints(goals.clone());
                }

                let feedback = executing.feedback_publisher();
                let (done_tx, done_rx) =
                    futures::channel::oneshot::channel::<(TourOutcome, Vec<i32>)>();
                let cancel_t = Arc::clone(&my_cancel);
                std::thread::spawn(move || {
                    let ctx = h.follow_ctx(tuning, true);
                    let retries = AtomicU64::new(0);
                    let mut missed: Vec<i32> = Vec::new();
                    let mut outcome = TourOutcome::Done;
                    for (i, goal) in goals.iter().enumerate() {
                        let mut fb = nav2_msgs::action::FollowWaypoints_Feedback::default();
                        fb.current_waypoint = i as u32;
                        let _ = feedback.publish(fb);
                        eprintln!(
                            "vi_planner: waypoint {}/{} -> ({:.2}, {:.2})",
                            i + 1,
                            goals.len(),
                            goal.x,
                            goal.y
                        );
                        match run_goal(&ctx, *goal, &cancel_t, retry, &retries, &|_| {}) {
                            Outcome::Reached => {}
                            Outcome::Preempted => {
                                outcome = TourOutcome::Aborted("vi_planner: tour preempted");
                                break;
                            }
                            Outcome::Failed(reason) => {
                                eprintln!(
                                    "ERROR: vi_planner: waypoint {} failed: {reason}",
                                    i + 1
                                );
                                missed.push(i as i32);
                                if stop_on_failure {
                                    outcome = TourOutcome::Aborted(
                                        "ERROR: vi_planner: tour stopped at the first \
                                         failure (stop_on_failure: true)",
                                    );
                                    break;
                                }
                            }
                        }
                        // 次の点へ向かうまでの間 (`waypoint_pause_sec`)。単に
                        // 待つのではなく場を更新し続ける (run_settle)。
                        if !pause.is_zero() && !run_settle(&ctx, &cancel_t, pause) {
                            outcome = TourOutcome::Aborted("vi_planner: tour preempted");
                            break;
                        }
                    }
                    stop_cmd(ctx.cmd_pub);
                    let _ = done_tx.send((outcome, missed));
                });

                let mut done_rx = done_rx;
                match executing.until_cancel_requested(&mut done_rx).await {
                    Ok(Ok((outcome, missed))) => {
                        let mut result = nav2_msgs::action::FollowWaypoints_Result::default();
                        result.missed_waypoints = missed;
                        match outcome {
                            TourOutcome::Done => {
                                eprintln!(
                                    "vi_planner: tour finished ({} missed)",
                                    result.missed_waypoints.len()
                                );
                                executing.succeeded_with(result)
                            }
                            TourOutcome::Aborted(msg) => {
                                eprintln!("{msg}");
                                executing.aborted_with(result)
                            }
                        }
                    }
                    Ok(Err(_)) => executing
                        .aborted_with(nav2_msgs::action::FollowWaypoints_Result::default()),
                    Err(rest) => {
                        my_cancel.store(true, Ordering::SeqCst);
                        let cancelling = executing.begin_cancelling();
                        let _ = rest.await;
                        eprintln!("vi_planner: tour cancelled by client");
                        cancelling
                            .cancelled_with(nav2_msgs::action::FollowWaypoints_Result::default())
                    }
                }
            }
        },
    )?)
}

/// follow_path サーバ (controller_server の置き換え)。
/// `follow: false` (nav2_controller と組む構成) では立てない。
pub fn follow_path_server(
    node: &Node,
    w: &Wiring,
) -> Result<ActionServer<nav2_msgs::action::FollowPath>> {
    let handles = Arc::clone(&w.handles);
    let follow_cancel = Arc::clone(&w.follow_cancel);
    let tuning = w.tuning;

    Ok(node.create_action_server::<nav2_msgs::action::FollowPath, _>(
        "follow_path",
        move |requested_goal: RequestedGoal<nav2_msgs::action::FollowPath>| {
            let h = Arc::clone(&handles);
            let follow_cancel = Arc::clone(&follow_cancel);

            async move {
                // 前の追従を止め、自分の cancel を登録。
                let my_cancel = preempt(&follow_cancel);

                let accepted = requested_goal.accept();
                // ゴール姿勢は path 終端 (controller_id / goal_checker_id は無視)。
                // この path は同じノードの compute_path_to_pose が返したもので、
                // 追従自体は path ではなく価値関数の方策に従う。
                let goal_pose =
                    accepted.goal().path.poses.last().map(|p| pose_view_from(&p.pose));
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
                let cancel_t = Arc::clone(&my_cancel);
                std::thread::spawn(move || {
                    // Nav2 構成で `plan` を出すのは compute_path_to_pose の側。
                    let ctx = h.follow_ctx(tuning, false);
                    // BT 構成では投げ直しは BT (RecoveryNode) の仕事なので、
                    // ここは 1 回きり (run_goal を通さない)。
                    let outcome = run_follow(&ctx, goal, &cancel_t, &|p| {
                        let _ = feedback.publish(nav2_msgs::action::FollowPath_Feedback {
                            distance_to_goal: p.distance_remaining.unwrap_or(f64::NAN) as f32,
                            speed: p.speed,
                        });
                    });
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
                        cancelling.cancelled_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                }
            }
        },
    )?)
}
