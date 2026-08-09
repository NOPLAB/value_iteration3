//! vi_planner entry point — Nav2 の planner_server と controller_server を
//! **1 ノードで同時に**置き換える全 Rust ノード。`compute_path_to_pose` と
//! `follow_path` (nav2_msgs) の両方を提供し、どちらも同じ `PlannerCore` =
//! 同じ価値関数を読む。`standalone: true` ならさらに `navigate_to_pose` と
//! `follow_waypoints` も提供し、**Nav2 のノードを 1 つも立てずに**自律移動する
//! (アクション型は nav2_msgs のままなので RViz 等の配線は変わらない。何が
//! 良くなるかはリポジトリ CLAUDE.md を参照。投げ直しの前に
//! [`node::follow_loop::run_settle`] が止まったまま場を更新するのが BT の
//! `Wait` との本質的な違い)。
//!
//! ## ファイルの分かれ方
//!
//! ここ (main.rs) は**起動手順そのもの**だけ。中身は 2 段に分かれている:
//!
//!   - `core` (lib 側) — rclrs 非依存の中核。価値関数・solve・追従の 1 手。
//!   - [`node`] — ROS の口。パラメータ ([`node::params`])、メッセージ変換
//!     ([`node::msg`])、共有ハンドル ([`node::handles`])、追従ループ
//!     ([`node::follow_loop`])、アクションサーバ ([`node::servers`])、
//!     背景の全域掃き ([`node::sweep`])、起動時の組み立て ([`node::boot`])。
//!
//! Boot order:
//!   1. `Context::default_from_env` + basic executor + node 作成
//!   2. パラメータ宣言・検証 (行動集合と θ 数は起動パラメータがそのまま効く)
//!   3. `VI_THREADS` 設定 (vi_threads > 0 のとき)
//!   4. /map 受信 (transient_local, 初回メッセージまでブロック)
//!   5. PlannerCore 構築 (静的地図前提) + 先読みワーカー (`waypoint_prefetch`)
//!   6. pose / scan / waypoints 購読 + cmd_vel パブリッシャ + action サーバ配線
//!      (常に 2 つ、`standalone` ならさらに 2 つ)
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
//!   - 先読みワーカー (`waypoint_prefetch`) は**この Mutex を一度も取らない**。
//!     自分の予備の核を別に持っていて、解けた場だけを受け渡す
//!     (`core::Prefetcher`)。握らせると、無くしたはずの停止がそのまま戻る。
//!
//! ## ゴール世代チェック
//!
//! 広域側が別ゴールを解くとキャッシュが差し替わる。追従スレッドは毎 tick
//! `is_cached_goal` で「自分のゴールの価値関数がまだ載っているか」を確認し、
//! 差し替わっていたらプリエンプト扱いで抜ける (別ゴールの方策でロボットを
//! 走らせない)。
//!
//! NOTE: rclrs API は ros2-rust/ros2_rust @ 2c6b926 (rclrs 0.7.0) — Docker
//! イメージがビルドする版 — に合わせている。

mod node;

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{Context as ACtx, Result};

use vi_lib::bridge::PoseView;

use vi_planner::core::lock;

use rclrs::*;

use node::follow_loop::{FollowTuning, RetryTuning};
use node::handles::{Handles, PlanPub, Viz};
use node::msg::{pose_view_from, vi_scan_from};
use node::params::{read_params, validate};
use node::servers::{self, Wiring};

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
    let map_msg = node::boot::wait_for_map(&node, &mut executor, params.map_wait_sec)
        .context("waiting for /map")?;
    eprintln!(
        "vi_planner: got map {}x{} @{}m",
        map_msg.info.width, map_msg.info.height, map_msg.info.resolution
    );

    // 5. PlannerCore (広域・狭域が共有する唯一の価値関数) + 自己位置推定 +
    //    先読みワーカー。メモリ見積もりと「起動を止める」判断は build_core の中。
    let (core, localizer, prefetch) = node::boot::build_core(&params, solver, &map_msg)?;
    drop(map_msg);

    // 6. 共有ハンドル束 (核・姿勢・スキャン・cmd_vel・可視化・表示用経路)。
    //    cmd_vel は Nav2 構成では launch 側で cmd_vel_nav にリマップし
    //    velocity_smoother を経由させる。可視化は価値関数が 1 本なので
    //    value_function も 1 本。plan はスタンドアロンのみ — あちらでは
    //    `compute_path_to_pose` を誰も呼ばないので、表示用の経路は追従側が
    //    出さないと画面に 1 本も出ない (Nav2 構成では compute_path_to_pose の
    //    成功が出すトピックなので立てない)。
    let handles = Arc::new(Handles {
        core: Mutex::new(core),
        latest_pose: Mutex::new(None),
        localizer: Mutex::new(localizer),
        scan_queue: Mutex::new(Vec::new()),
        cmd_pub: node.create_publisher::<geometry_msgs::msg::Twist>("cmd_vel".keep_last(1))?,
        est_pub: node
            .create_publisher::<geometry_msgs::msg::PoseStamped>("viola_pose".keep_last(1))?,
        est_frame: params.global_frame.clone(),
        est_clock: node.get_clock(),
        tf_pub: if params.publish_tf {
            Some(node.create_publisher::<tf2_msgs::msg::TFMessage>("/tf".keep_last(100))?)
        } else {
            None
        },
        tf_tolerance: Duration::from_secs_f64(params.transform_tolerance.max(0.0)),
        viz: if params.publish_value_function {
            Some(Viz {
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
            })
        } else {
            None
        },
        plan_pub: params
            .standalone
            .then(|| -> Result<PlanPub> {
                Ok(PlanPub {
                    path_pub: node
                        .create_publisher::<nav_msgs::msg::Path>("plan".keep_last(1))?,
                    clock: node.get_clock(),
                    frame_id: params.global_frame.clone(),
                })
            })
            .transpose()?,
    });

    // 6a. 自己位置トピック購読 (tf2 代替)。external ではそのまま採用、belief では
    //     belief の手動シード (どちらも Loc::set_pose 経由)。
    let _pose_sub = {
        let h = Arc::clone(&handles);
        node.create_subscription::<geometry_msgs::msg::PoseWithCovarianceStamped, _>(
            params.pose_topic.as_str().keep_last(1),
            move |msg: geometry_msgs::msg::PoseWithCovarianceStamped| {
                let p = {
                    let mut l = lock(&h.localizer);
                    l.set_pose(pose_view_from(&msg.pose.pose));
                    let p = l.pose();
                    *lock(&h.latest_pose) = p;
                    p
                };
                if let Some(p) = p {
                    h.publish_est(p);
                }
            },
        )?
    };

    // 6a″. odom 購読 (publish_tf のときだけ)。map→odom は推定と odom の合成で、
    //      odom レートで出し直す (map→odom 自体はスキャン補正のときしか動かない
    //      が、スタンプの新鮮さが TF 参照側の生死を分ける)。
    let _odom_sub = if !params.publish_tf {
        None
    } else {
        let h = Arc::clone(&handles);
        Some(node.create_subscription::<nav_msgs::msg::Odometry, _>(
            params.odom_topic.as_str().keep_last(1),
            move |msg: nav_msgs::msg::Odometry| {
                let est = match *lock(&h.latest_pose) {
                    Some(p) => p,
                    None => return, // シード前は map→odom を定義できない
                };
                h.publish_tf(est, pose_view_from(&msg.pose.pose), &msg.header.frame_id);
            },
        )?)
    };

    // 6b. スキャン購読 (sensor QoS = best effort)。tick 間に届いた分を貯めて
    //     制御ループが順に消化する。同じスキャンが自己位置推定の補正にも入る
    //     (external では no-op。belief の補正はアクティブ集合 = belief の広がりに
    //     比例したコストで、収束後は数百セル × 数十ビーム — エグゼキュータ
    //     スレッドで足りる)。
    let _scan_sub = {
        let h = Arc::clone(&handles);
        let invalid_range_m = params.invalid_range_m;
        node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
            params.scan_topic.as_str().best_effort().keep_last(5),
            move |msg: sensor_msgs::msg::LaserScan| {
                let scan = vi_scan_from(&msg, invalid_range_m);
                let est = {
                    let mut l = lock(&h.localizer);
                    l.observe(&scan);
                    let p = l.pose();
                    // 無条件代入: belief はロスト中 None を返すので、latest_pose
                    // ごと消して follow ループを「pose なし」停止経路に乗せる
                    // (external は observe が no-op で Some のまま = 従来どおり)。
                    *lock(&h.latest_pose) = p;
                    p
                };
                if let Some(p) = est {
                    h.publish_est(p);
                }
                let mut q = lock(&h.scan_queue);
                // 制御ループが止まっていても際限なく溜めない (最新を優先)。
                if q.len() >= 10 {
                    q.remove(0);
                }
                q.push(scan);
            },
        )?
    };

    // 6b-2. 先読み対象の並び (nav_msgs/Path)。daifuku_waypoint_manager が
    //       waypoint を編集するたび latch して出す。**受け取っただけでは何も
    //       解かない** — 注文が出るのは走り出して最初のゴールが確定してから
    //       (core::Prefetcher::note_goal) で、そうしないと起動と同時に 1 点目の
    //       solve が走って nav2 の lifecycle 立ち上げと CPU を奪い合う。
    let _waypoints_sub = match prefetch.clone() {
        None => None,
        Some(pf) => {
            let frame = params.global_frame.clone();
            Some(node.create_subscription::<nav_msgs::msg::Path, _>(
                params.waypoint_topic.as_str().reliable().transient_local().keep_last(1),
                move |msg: nav_msgs::msg::Path| {
                    // フレームが違う並びで先読みすると、座標だけがそれらしく
                    // 入って一致判定に永久に掛からない (先読みが黙って効かなく
                    // なるだけなので、必ず出す)。
                    if !msg.header.frame_id.is_empty() && msg.header.frame_id != frame {
                        eprintln!(
                            "WARN: vi_planner: ignoring {} waypoints in frame {:?} \
                             (this planner works in {:?})",
                            msg.poses.len(),
                            msg.header.frame_id,
                            frame
                        );
                        return;
                    }
                    let wps: Vec<PoseView> =
                        msg.poses.iter().map(|p| pose_view_from(&p.pose)).collect();
                    eprintln!("vi_planner: {} waypoints available for prefetch", wps.len());
                    pf.set_waypoints(wps);
                },
            )?)
        }
    };

    // 6c. 狭域 → 広域のフィードバック (背景の全域掃きスレッド)。
    //     `global_sweep: false` なら立たない。詳細は node::sweep の doc。
    node::sweep::spawn(Arc::clone(&handles), &params);

    // 6d. アクションサーバの配線材料。
    //
    // 進行中の solve / 追従をプリエンプトするための cancel フラグ置き場は 2 つ。
    // 2 つのアクションは互いを止めない (広域の再計画で追従が死んではいけないし、
    // 追従中でも再計画は受け付ける) ので、スロットは別々に持つ。追従を回す
    // 3 つのサーバは同じ 1 台を走らせるので follow 側は共有する。
    let wiring = Wiring {
        handles: Arc::clone(&handles),
        plan_cancel: Arc::new(Mutex::new(None::<Arc<AtomicBool>>)),
        follow_cancel: Arc::new(Mutex::new(None::<Arc<AtomicBool>>)),
        frame_id: params.global_frame.clone(),
        clock: node.get_clock(),
        tuning: FollowTuning {
            period: Duration::from_secs_f64(1.0 / params.control_frequency),
            refine_budget: Duration::from_millis(params.refine_budget_ms.max(0) as u64),
            failure_ticks_limit: (params.no_action_timeout_sec.max(0.0)
                * params.control_frequency)
                .ceil()
                .max(1.0) as u32,
            busy_ticks_before_stop: params.busy_ticks_before_stop.max(1) as u32,
            qmdp: params.qmdp,
            scan_quality_gate: params.scan_quality_gate,
            active_reloc: params.active_reloc,
            reloc_ticks_limit: (params.reloc_timeout_sec.max(0.0) * params.control_frequency)
                .ceil()
                .max(1.0) as u32,
        },
        retry: RetryTuning {
            limit: params.goal_retry_limit,
            settle: Duration::from_secs_f64(params.goal_retry_settle_sec.max(0.0)),
        },
        prefetch,
        stop_on_failure: params.waypoint_stop_on_failure,
        pause: Duration::from_secs_f64(params.waypoint_pause_sec.max(0.0)),
        // ロールアウト固着時のヒント表示用 (safety_radius_penalty [秒/セル],
        // safety_radius [m])。
        hint: (params.safety_radius_penalty, params.safety_radius),
    };

    // 6e. アクションサーバ。compute_path_to_pose は常に、follow_path は
    //     `follow: true` のとき、残り 2 つと /goal_pose 購読は standalone のとき。
    let _plan_server = servers::plan_server(&node, &wiring)?;
    let _follow_server =
        params.follow.then(|| servers::follow_path_server(&node, &wiring)).transpose()?;
    let _nav_to_pose_server =
        params.standalone.then(|| servers::nav_to_pose_server(&node, &wiring)).transpose()?;
    let _goal_pose_sub =
        params.standalone.then(|| servers::goal_pose_sub(&node, &wiring)).transpose()?;
    let _follow_waypoints_server = params
        .standalone
        .then(|| servers::follow_waypoints_server(&node, &wiring))
        .transpose()?;

    eprintln!(
        "vi_planner: ready (solver={}, actions=compute_path_to_pose{}{}, {}Hz{})",
        params.solver,
        if params.follow { " + follow_path" } else { " (follow: false — nav2_controller 構成)" },
        if params.standalone {
            " + navigate_to_pose + follow_waypoints (standalone: no Nav2 nodes)"
        } else {
            ""
        },
        params.control_frequency,
        if params.waypoint_prefetch {
            // スタンドアロンでは follow_waypoints のゴールがそのまま順路になるので、
            // トピックは「もう 1 つの入口」でしかない。
            if params.standalone {
                format!(", prefetching from the tour (or {})", params.waypoint_topic)
            } else {
                format!(", prefetching from {}", params.waypoint_topic)
            }
        } else {
            String::new()
        }
    );
    if params.standalone {
        eprintln!(
            "vi_planner: standalone retries a failed goal {} (settling {:.1}s between tries, \
             during which scans keep updating the value function — a Nav2 BT Wait cannot)",
            if params.goal_retry_limit < 0 {
                "without limit".to_string()
            } else {
                format!("up to {} times", params.goal_retry_limit)
            },
            params.goal_retry_settle_sec
        );
    }

    // 7. Spin.
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
