//! vi_planner entry point — Nav2 の planner_server と controller_server を
//! **1 ノードで同時に**置き換える全 Rust ノード。`standalone: true` にすると
//! さらに bt_navigator と waypoint_follower も置き換え、**Nav2 のノードを 1 つも
//! 立てずに**自律移動する (下の「スタンドアロン」を参照)。
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
//! ## スタンドアロン (`standalone: true`)
//!
//! `navigate_to_pose` と `follow_waypoints` を**このノード自身が**提供する。
//! bt_navigator / behavior_server / waypoint_follower / smoother_server と、
//! それらを起こす lifecycle_manager が要らなくなる。アクション型は nav2_msgs の
//! ままなので、RViz の `Nav2 Goal` も daifuku の各パネルも配線を変えずに動く。
//!
//! Nav2 の BT を挟まなくなることで消える制約が 4 つある。どれも「VI の場は
//! 走りながら良くなる」のに BT がゴール単位で諦めていたことに由来する:
//!
//!   * **1 Hz のリプランが消える。** BT は `ComputePathToPose` を毎秒呼び、
//!     キャッシュヒットでもロールアウト + densify を**ロックの中で**回していた。
//!     10 Hz の追従ループは同じ Mutex を `try_lock` するので、これは毎秒の
//!     取り合いだった。ここではロールアウトはゴールにつき 1 回で、しかも
//!     `plan` トピックへの表示専用。
//!   * **ロールアウトの失敗でゴールが死ななくなる。** `LoopDetected` は
//!     「貪欲降下が振動した」であって「方策が無い」ではない。追従は方策を
//!     1 手ずつ引くので、経路が引けなくても走れることがある。BT 構成では
//!     `ComputePathToPose` の失敗がそのままゴールの失敗だった。
//!   * **リカバリが実際に効く。** BT の `Spin` / `BackUp` は
//!     `local_costmap/costmap_raw` を待つので、コストマップを持たないこの構成
//!     では**必ず失敗**し、動くのは `Wait` だけだった。その `Wait` も
//!     follow_path が走っていない間は `observe_scan` も `refine_for` も呼ばない
//!     ので、「待てば通れる」の唯一の仕組み ([`set_local_cost`] の penalty 半減)
//!     が進まない。ここでは投げ直しの前に [`run_settle`] が**止まったまま場を
//!     更新する**。
//!   * **先読みが黙って効かない経路が無くなる。** `waypoint_prefetch` は順路を
//!     `waypoint_topic` から拾うしか手が無く、そこへ出すものがいない構成では
//!     警告も出ずに何もしなかった。`follow_waypoints` を自分で持つと、順路は
//!     ゴールと同じ経路で入ってくる。
//!
//! Boot order (vi_global_planner と同型):
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
//! イメージがビルドする版 — に合わせている (vi_node / vi_global_planner と同一)。

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context as ACtx, Result};

use vi_reference::bridge::{
    downsample_occupancy, occupancy_view_to_vi_grid, OccupancyGridView, PoseView,
};
use vi_reference::msg::LaserScan as ViLaserScan;
use vi_reference::planner::PathPose;
use vi_reference::solvers::U64Solver;
use vi_reference::Action;
// nav_msgs::msg::OccupancyGrid と名前が衝突するので別名で入れる。
use vi_reference::OccupancyGrid as ViOccupancyGrid;

use vi_planner::core::{
    value_grid_of, BuildParams, Decision, PlanConfig, PlanError, PlanStats, PlannerCore,
    Prefetcher, SweepCursor,
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
    action_list: Vec<(String, f64, f64)>,
    unknown_as_obstacle: bool,
    /// プランナ内部で地図を何倍に粗くするか (1 = /map のまま)。
    map_scale: i64,
    /// ダウンサンプルの方針 ("conservative" | "optimistic")。
    downsample_policy: String,
    /// compact ソルバの確定出力を置くディレクトリ (空文字 = RAM)。
    compact_sink_dir: String,
    /// RAM sink の上限 [MB]。compact 経路で `compact_sink_dir` 未指定かつ推定サイズが
    /// これを超えるとき、自動でディスク sink に逃がす。
    compact_ram_limit_mb: i64,
    /// 密ソルバで確保してよい価値関数の上限 [MB] (states + sweep_orders)。
    /// 超えたら起動を止める。
    dense_limit_mb: i64,
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
    // ── スタンドアロン (navigate_to_pose / follow_waypoints) ──
    standalone: bool,
    goal_retry_limit: i64,
    goal_retry_settle_sec: f64,
    waypoint_stop_on_failure: bool,
    waypoint_pause_sec: f64,
    // ── 狭域 → 広域のフィードバック (全域掃き) ──
    global_sweep: bool,
    global_sweep_budget_ms: i64,
    global_sweep_idle_ms: i64,
    // ── ウェイポイントの先読み ──
    waypoint_prefetch: bool,
    waypoint_topic: String,
    waypoint_prefetch_threads: i64,
    // ── 走り出しの短縮 ──
    early_start: bool,
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
    let unknown_as_obstacle = p!("unknown_as_obstacle", bool, true);
    let map_scale = p!("map_scale", i64, 1);
    // "conservative" = 本家 downsample_occupancy (障害物優先)。"optimistic" = ブロック内に free が
    // 1 つでもあれば free。map_scale >= 4 で通路のセル幅を保つために必要 (既定は挙動不変の保守側)。
    let downsample_policy = p!("downsample_policy", Arc<str>, "conservative".into()).to_string();
    // solver = frontier2d_sparse_compact のときだけ効く。空文字なら RAM (RamSink)。
    // 指定先が tmpfs だと結局 RAM に載るので、メモリ退避が目的なら実ディスクを指すこと。
    let compact_sink_dir = p!("compact_sink_dir", Arc<str>, "".into()).to_string();
    // compact_sink_dir 未指定でも、確定出力がこれを超えるならディスクへ逃がす。小メモリ機で
    // 黙って GB 級の RamSink を確保すると OOM killer に落とされるため (vi_global_planner と同じ)。
    let compact_ram_limit_mb = p!("compact_ram_limit_mb", i64, 512);
    // 密ソルバの価値関数 (states 56 B/state + sweep_orders 24 B/state) の上限。
    // 超えたら起動を止める。既定 1500 は 4GB 機 (Pi4) で他のノードと同居できる線
    // — 19F を map_scale 2 で解くと実測 655 MB。
    let dense_limit_mb = p!("dense_limit_mb", i64, 1500);
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

    // navigate_to_pose と follow_waypoints をこのノード自身が提供する
    // (= bt_navigator / behavior_server / waypoint_follower を立てない)。
    // **既定は false**: Nav2 構成でこれを立てると navigate_to_pose のサーバが
    // bt_navigator と 2 つになり、クライアントは先に見つけたほうへ繋ぐ
    // (どちらに繋がったかはログにも出ない)。立てるのは launch の責任。
    let standalone = p!("standalone", bool, false);
    // 追従が失敗したときにゴールを投げ直す上限 (負で無制限)。BT の
    // `RecoveryNode number_of_retries` の置き換えだが、あちらと違って
    // **投げ直しの合間に場が実際に動く** (下の goal_retry_settle_sec)。
    let goal_retry_limit = p!("goal_retry_limit", i64, 3);
    // 投げ直す前に、止まったままスキャンを取り込んで場を精密化する時間。
    // 0 で即座に投げ直す (それだと同じ場で同じ失敗を繰り返しやすい)。
    let goal_retry_settle_sec = p!("goal_retry_settle_sec", f64, 3.0);
    // follow_waypoints: 1 点失敗したら残りを諦めるか。false なら次の点へ進み、
    // 飛ばした番号を result.missed_waypoints で返す (nav2_waypoint_follower と同義)。
    let waypoint_stop_on_failure = p!("stop_on_failure", bool, false);
    // 1 点に着いてから次の点へ向かうまでの待ち (nav2_waypoint_follower の
    // waypoint_pause_duration [ms] に相当。こちらは秒)。
    let waypoint_pause_sec = p!("waypoint_pause_sec", f64, 0.2);

    // 狭域が書いた local_penalty を全域へ伝播させる背景掃き (core::sweep_global)。
    // これを止めると、狭域が「通れない」と判断しても広域の経路は塞がった通路を
    // 指し続ける (旧挙動)。密ソルバは全域 Gauss–Seidel、compact は sink の
    // タイル修復で同じことをする (どちらも `core::sweep_global` の入口)。
    let global_sweep = p!("global_sweep", bool, true);
    // 1 回のロック取得で掃きに使う時間と、そのあとロックを手放して待つ時間。
    // 比がそのまま CPU の取り分になる (既定 20:60 = 1 コアの 25%)。追従ループは
    // 同じ Mutex を try_lock で取り、3 tick 続けて取れないとロボットを止めるので、
    // budget を伸ばすときは idle も一緒に伸ばすこと。
    let global_sweep_budget_ms = p!("global_sweep_budget_ms", i64, 20);
    let global_sweep_idle_ms = p!("global_sweep_idle_ms", i64, 60);

    // 次のウェイポイントの価値関数を、いまの点へ走っている間に解いておく
    // (core::Prefetcher)。巡回では点が変わるたびに solve が丸ごと 1 回走り、その間
    // 機体が止まる (19F で 29 秒、津田沼 compact で 87 秒) — それを走行時間に隠す。
    // **既定は false**: 価値関数が同時に 2 つ生きるので、密ならメモリが、compact
    // なら sink のディスクが最大 2 倍 + 採用待ちの 1 つぶん要る。効くのは
    // waypoint_topic に並びを出すもの (daifuku_waypoint_manager) がいるときだけで、
    // 並びが無ければ立ち上がっていても何も解かない。
    let waypoint_prefetch = p!("waypoint_prefetch", bool, false);
    // 先読み対象の並び (nav_msgs/Path)。transient_local で購読するので、後から
    // 起動しても latch されている並びを拾える。
    let waypoint_topic = p!("waypoint_topic", Arc<str>, "waypoints".into()).to_string();
    // 先読みの solve に使うスレッド数。追従は 10Hz・予算 40ms/tick で回っているので
    // 既定 1 に絞ってある。**効くのは compact 経路だけ** — 密の frontier2d_sparse は
    // スレッド数を環境変数 VI_THREADS から読む (プロセスで 1 つ) ので、先読みだけを
    // 絞ることができない。
    let waypoint_prefetch_threads = p!("waypoint_prefetch_threads", i64, 1);

    // 機体の現在地からゴールまで方策が繋がった時点で solve を打ち切って走り出す
    // (core::PlanConfig::early_start)。**既定は false**: 経路の外は未確定のままに
    // なるので、機体が経路から外れると解き直しが要る (そのときは打ち切った場を
    // 捨ててから最後まで解くので、待ちは合計で長くなる)。先読み
    // (waypoint_prefetch) と違い追加のメモリは要らず、両者は併用できる
    // — 先読みで用意した場は打ち切らないので、受け取れた点では関係しない。
    let early_start = p!("early_start", bool, false);

    let publish_value_function = p!("publish_value_function", bool, true);
    // solve の途中経過の配信間隔。0 は「完了時のみ」。追従中の再配信は掃き
    // スレッドが 2 秒ごとに出すので、ここを 0 にすると追従中は出なくなる。
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
        action_list,
        unknown_as_obstacle,
        map_scale,
        downsample_policy,
        compact_sink_dir,
        compact_ram_limit_mb,
        dense_limit_mb,
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
        standalone,
        goal_retry_limit,
        goal_retry_settle_sec,
        waypoint_stop_on_failure,
        waypoint_pause_sec,
        global_sweep,
        global_sweep_budget_ms,
        global_sweep_idle_ms,
        waypoint_prefetch,
        waypoint_topic,
        waypoint_prefetch_threads,
        early_start,
        publish_value_function,
        value_publish_interval_ms,
        cost_drawing_threshold,
        window_cost_drawing_threshold,
    })
}

/// パラメータの自己整合検査 (vi_node / vi_global_planner と同じ fail-fast)。
///
/// 行動集合も θ 数も `ValueIterator` は実行時に受け取るので、値そのものは launch
/// から自由に決めてよい (かつてここで vi_core のコンパイル時定数と照合していた
/// のは、値を変えたいときの邪魔にしかならなかった)。落とすのは、後段のどこかで
/// 実際に割り算・添字になって壊れるものだけ。
fn validate(p: &Params) -> Result<U64Solver> {
    if p.map_scale < 1 {
        return Err(anyhow!("map_scale must be >= 1, got {}", p.map_scale));
    }
    // 本家 `t_resolution_ = 360/cell_num_t_` は整数除算。割り切れないと
    // `it = (t % 360) / t_resolution` が cell_num_t 以上を返し、states の範囲外を指す。
    if p.theta_cell_num <= 0 || p.theta_cell_num > 360 || 360 % p.theta_cell_num != 0 {
        return Err(anyhow!(
            "theta_cell_num must divide 360 (t_resolution = 360/theta_cell_num is an \
             integer division), got {}",
            p.theta_cell_num
        ));
    }
    if p.action_list.is_empty() {
        return Err(anyhow!("action_names/action_forward_m/action_rotation_deg are empty"));
    }
    if p.control_frequency <= 0.0 {
        return Err(anyhow!("control_frequency must be > 0, got {}", p.control_frequency));
    }
    let solver = U64Solver::from_name(&p.solver)
        .ok_or_else(|| anyhow!("unknown solver: {} (see U64Solver::from_name)", p.solver))?;
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

/// compact 経路の出力先ディレクトリを決める。
///
/// - compact 以外のソルバでは常に `None` (使われない)。
/// - `compact_sink_dir` が明示されていればそれを使う。
/// - 未指定でも確定出力 (`nstates × 12 B`) が `compact_ram_limit_mb` を超えるなら
///   `/tmp/vi_planner_sink` に逃がす。小メモリ機で黙って GB 級の `RamSink` を確保すると
///   OOM killer に落とされるため。
///
/// `vi_global_planner` の同名関数と同じ判断順 (明示指定が先、上限による自動退避が後) だが、
/// **ディスクに置いたときの代償はこちらのほうが大きい**。広域だけを解く
/// `vi_global_planner` と違い、`vi_planner` の追従はパッチを置き直すたびに sink を読むので
/// (10Hz の制御ループ)、SD カード上の sink は追従の遅延に直結する。逃がす先は
/// できるだけ実ディスクでも速いところを `compact_sink_dir` で明示すること。
fn compact_sink_dir(params: &Params, solver: U64Solver, nstates: usize) -> Option<PathBuf> {
    if !matches!(solver, U64Solver::Frontier2DSparseCompact { .. }) {
        return None;
    }
    let bytes = nstates as u64 * 12;
    if !params.compact_sink_dir.is_empty() {
        let dir = PathBuf::from(&params.compact_sink_dir);
        eprintln!(
            "vi_planner: compact output -> disk mmap {} ({:.2} GB)",
            dir.display(),
            bytes as f64 / 1e9
        );
        return Some(dir);
    }
    let limit = params.compact_ram_limit_mb.max(0) as u64 * 1024 * 1024;
    if bytes > limit {
        let dir = PathBuf::from("/tmp/vi_planner_sink");
        eprintln!(
            "WARN: compact output would need {:.2} GB of RAM (> compact_ram_limit_mb={}); \
             spilling to disk mmap {}. The follow loop re-reads the sink on every patch \
             recenter, so point compact_sink_dir at the fastest real disk available.",
            bytes as f64 / 1e9,
            params.compact_ram_limit_mb,
            dir.display()
        );
        return Some(dir);
    }
    eprintln!("vi_planner: compact output -> RAM ({:.2} GB)", bytes as f64 / 1e9);
    None
}

/// `downsample_occupancy` の楽観版。ブロック内に 1 つでも free があれば出力セルを free にする。
///
/// 本家の `downsample_occupancy` は障害物優先なので通路が片側最大 `(scale-1)·resolution` 細る。
/// map_tsudanuma は unknown が 68% あり、`map_scale >= 4` では free **面積**は数 % しか減らないのに
/// **通路のセル幅**が落ちる。VI の遷移はサブセルサンプリングによる約 2 セル幅の分布なので、
/// 散り先に 1 つでも未到達セルがあると期待値が MAX_COST 側に張り付き、波がゴール近傍で止まる。
///
/// 安全余裕を地図に焼き込んではいけない。VI の `safety_radius` は硬い壁ではなく秒/セルのソフトな
/// ペナルティで、膨張として焼き込むと scale 3 でも波が死ぬ (実測)。ここでは通路を開けるだけにする。
///
/// `vi_global_planner` にも同じ関数がある (別クレートなので共有できない)。片方を直したら両方直すこと。
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
    /// θ=0 全域スライス (solve の途中経過 + 完了時 + 追従中の伝播の進み具合)。
    /// 追従中に出しているのは掃きスレッドで、`global_sweep: false` だと
    /// solve した瞬間のまま固まる。
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

/// 追従の 1 tick の様子。アクションごとに Feedback の形が違うので、追従ループ
/// 自体はメッセージ型を知らずにこれを渡す (`follow_path` は距離と速度だけ、
/// `navigate_to_pose` は姿勢と経過時間も要る)。
struct FollowProgress {
    pose: PoseView,
    /// ゴールまでの XY 距離。ゴール未設定なら None。
    distance_remaining: Option<f64>,
    /// この tick に出した前進速度 [m/s] (止めた tick は 0)。
    speed: f32,
}

/// 表示専用の経路パブリッシャ (`plan`)。Nav2 構成ではこれを出すのは
/// planner_server (= `compute_path_to_pose` の側) で、RViz の Path 表示や
/// `daifuku_rqt` が見ているのはそのトピック。スタンドアロンでは
/// `navigate_to_pose` が誰も `compute_path_to_pose` を呼ばないので、
/// ここから出さないと**画面に経路が 1 本も出ない**。
///
/// あくまで表示専用で、ロールアウトが失敗しても走行には影響しない
/// (追従は経路ではなく方策を 1 手ずつ引く)。
struct PlanPub {
    path_pub: Publisher<nav_msgs::msg::Path>,
    clock: Clock,
    frame_id: String,
}

/// 追従ループが触る ROS 側の口。ゴールごとに変わらないものをまとめてある。
/// 全フィールドが参照か Copy なので、分配束縛のために Copy にしてある。
#[derive(Clone, Copy)]
struct FollowCtx<'a> {
    core: &'a Mutex<PlannerCore>,
    latest_pose: &'a Mutex<Option<PoseView>>,
    scan_queue: &'a Mutex<Vec<ViLaserScan>>,
    cmd_pub: &'a Publisher<geometry_msgs::msg::Twist>,
    /// 表示専用の経路。None なら出さない (`follow_path` 構成では BT 側の
    /// `compute_path_to_pose` が出すので不要)。
    plan_pub: Option<&'a PlanPub>,
    viz: Option<&'a Viz>,
    tuning: FollowTuning,
}

/// 1 ゴールぶんの追従ループ。
///
/// solve 中だけロックを保持し、制御ループは **tick ごとに取得・解放**する
/// (compute_path_to_pose を待たせないため; ファイル冒頭のロック規律を参照)。
fn run_follow(
    ctx: &FollowCtx,
    goal: PoseView,
    cancel: &AtomicBool,
    report: &dyn Fn(&FollowProgress),
) -> Outcome {
    let FollowCtx { core, latest_pose, scan_queue, cmd_pub, plan_pub, viz, tuning } = *ctx;
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
        // 早期打ち切り (early_start) の起点。solve のあいだ機体は止まっているので、
        // ここで読んだ姿勢は走り出すときの姿勢そのもの。まだ pose が来ていなければ
        // 打ち切らずに最後まで解く (下の制御ループはどのみち pose を待つ)。
        let from = *latest_pose.lock().unwrap();
        let prepared = core.prepare_goal_with_progress(goal, from, cancel, &mut |vi| {
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
                        // 「追従が用意した場」の意 (広域の plan が解いたぶんと区別する)。
                        // standalone では follow_path ではなく navigate_to_pose の下。
                        "vi_planner: value function {} in {:.2}s (iters={}){} [following]",
                        if stats.adopted { "adopted from prefetch" } else { "solved" },
                        t0.elapsed().as_secs_f64(),
                        stats.iters,
                        // 打ち切った回。ゴールまでの経路は繋がっているが、そこから
                        // 外れた領域は未確定 (early_start)。
                        if stats.truncated {
                            ", cut short at the first path to the goal"
                        } else {
                            ""
                        }
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

        // 表示用の経路を 1 本だけ出す (スタンドアロンのみ)。場は既に載っている
        // ので、ここは solve ではなくロールアウト 1 回。
        //
        // **失敗しても走行には影響させない。** ロールアウトは貪欲降下なので、
        // 値の起伏が 1 手の進捗より大きい地形で振動して `LoopDetected` になる
        // ことがある (HINT を出す条件そのもの)。追従は方策を 1 手ずつ引くだけ
        // なので、経路が引けなくても走れる。BT 構成ではこの失敗が
        // `ComputePathToPose` の失敗 = ゴールの失敗だった。
        if let Some(pp) = plan_pub {
            let from = *latest_pose.lock().unwrap();
            if let Some(from) = from {
                match core.plan(from, goal, cancel) {
                    Ok((poses, _)) => {
                        let stamp = pp.clock.now().to_sec_nanosec().unwrap_or((0, 0));
                        let _ =
                            pp.path_pub.publish(poses_to_path(&poses, &pp.frame_id, stamp));
                    }
                    Err(e) => eprintln!(
                        "WARN: vi_planner: no path to draw ({e}) — following anyway \
                         (the policy is followed one action at a time, not the path)"
                    ),
                }
            }
        }
    } // ここでロックを解放 — 以降は tick ごとに取り直す。

    // ── 2. 制御ループ ──
    // tick の中では取得した guard が `core` を隠すので、guard を手放したあとに
    // 本体へ触るための別名を先に取っておく。
    let shared = core;
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

        report(&FollowProgress { pose, distance_remaining: dist, speed });

        if failure_ticks >= tuning.failure_ticks_limit {
            stop_cmd(cmd_pub);
            // 打ち切った場 (early_start) だったなら捨てる。方策が引けないのは
            // 「経路の外の未確定領域に入った」形で起き得るが、キャッシュを
            // 残したままだと BT が投げ直すたびに同じ場で同じ失敗を繰り返す
            // (prepare_goal はキャッシュヒットで solve に入らない)。捨てておけば
            // 次の要求が最後まで解き直す。収束済みの場なら何もしない。
            if shared.lock().unwrap_or_else(|p| p.into_inner()).discard_truncated() {
                eprintln!(
                    "vi_planner: dropped the truncated value function (early_start) after \
                     {failure_ticks} ticks without an action; the next request solves it \
                     to convergence"
                );
            }
            return Outcome::Failed("no applicable action for too long".into());
        }
        if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
            std::thread::sleep(rest);
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Standalone goal runner (bt_navigator / behavior_server の置き換え)
// ──────────────────────────────────────────────────────────────────────────────

/// 巡回 (`follow_waypoints`) の終わり方。飛ばした点そのものは別に返す。
enum TourOutcome {
    /// 最後の点まで回った (途中で失敗した点があっても、進み続けたならこれ)。
    Done,
    /// `stop_on_failure: true` で 1 点目の失敗のところで止めた。
    Stopped,
    /// 新しいゴールか cancel で止まった。
    Preempted,
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

/// 投げ直しのチューニング (`standalone` のときだけ効く)。
#[derive(Clone, Copy)]
struct RetryTuning {
    /// 追従が失敗したときに投げ直す上限。負で無制限。
    limit: i64,
    /// 投げ直す前に、その場で場を落ち着かせる時間。0 で即座に投げ直す。
    settle: Duration,
}

/// 止まったまま場を更新し続ける「待ち」。投げ直しの前に挟む。
///
/// **Nav2 の BT の `Wait` はこれができない。** あちらが待っている間は
/// `follow_path` が走っていないので `observe_scan` も `refine_for` も呼ばれず、
/// つまり `set_local_cost` の penalty 半減 — 一度«通れない»と塗った場所が
/// «やっぱり通れる»に戻る唯一の経路 — が 1 段も進まない。待ち時間を延ばしても
/// 場が同じままなら、投げ直しは同じ失敗を同じ場所で繰り返すだけになる
/// (2026-08-04 の実機で `no_action_timeout_sec` を 15 秒へ延ばして確認: 粘らず、
/// 1 点飛ばすまでが 2 分に伸びただけだった)。
///
/// ここでは 0 速度を出しながら制御ループと同じ更新 (窓の移動 → スキャン注入 →
/// 精密化) を回す。ロックの持ち方も制御ループと同じ `try_lock` で、掃きスレッド
/// からも先読みからも取り上げない。
///
/// 戻り値は「最後まで待てたか」。false は cancel された。
fn run_settle(ctx: &FollowCtx, cancel: &AtomicBool, dur: Duration) -> bool {
    let FollowCtx { core, latest_pose, scan_queue, cmd_pub, tuning, .. } = *ctx;
    let deadline = Instant::now() + dur;
    while Instant::now() < deadline {
        let tick_start = Instant::now();
        if cancel.load(Ordering::Relaxed) {
            return false;
        }
        stop_cmd(cmd_pub);

        let pose = *latest_pose.lock().unwrap();
        if let Some(pose) = pose {
            let guard = match core.try_lock() {
                Ok(g) => Some(g),
                Err(std::sync::TryLockError::WouldBlock) => None,
                Err(std::sync::TryLockError::Poisoned(p)) => Some(p.into_inner()),
            };
            if let Some(mut c) = guard {
                let scans = std::mem::take(&mut *scan_queue.lock().unwrap());
                c.set_window(pose);
                for scan in &scans {
                    c.observe_scan(scan, pose);
                }
                c.refine_for(tuning.refine_budget);
            }
        }

        if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
            std::thread::sleep(rest);
        }
    }
    true
}

/// スタンドアロンの 1 ゴール = `navigate_to_pose` の中身。
///
/// [`run_follow`] を「失敗したら場を落ち着かせて投げ直す」で包んだだけだが、
/// これが Nav2 の BT (`RecoveryNode` + `RoundRobin` の Spin / Wait / BackUp) の
/// 置き換えになっている。BT との違いは 2 つ:
///
///   * リカバリが**必ず動く**。`Spin` / `BackUp` は `local_costmap/costmap_raw`
///     を待つが、VI 構成にコストマップは無いので必ず失敗していた。
///   * 待つ間に**場が動く** ([`run_settle`])。BT の `Wait` は止まるだけだった。
///
/// `retries` は Feedback の `number_of_recoveries` 用に外から覗くための共有カウンタ。
fn run_goal(
    ctx: &FollowCtx,
    goal: PoseView,
    cancel: &AtomicBool,
    retry: RetryTuning,
    retries: &AtomicU64,
    report: &dyn Fn(&FollowProgress),
) -> Outcome {
    retries.store(0, Ordering::Relaxed);
    loop {
        match run_follow(ctx, goal, cancel, report) {
            Outcome::Failed(reason) => {
                let done = retries.load(Ordering::Relaxed);
                if retry.limit >= 0 && done as i64 >= retry.limit {
                    return Outcome::Failed(format!("{reason} (after {done} retries)"));
                }
                retries.store(done + 1, Ordering::Relaxed);
                eprintln!(
                    "vi_planner: follow failed ({reason}); settling for {:.1}s, then retry {}{}",
                    retry.settle.as_secs_f64(),
                    done + 1,
                    if retry.limit >= 0 {
                        format!("/{}", retry.limit)
                    } else {
                        String::new()
                    }
                );
                if !retry.settle.is_zero() && !run_settle(ctx, cancel, retry.settle) {
                    stop_cmd(ctx.cmd_pub);
                    return Outcome::Preempted;
                }
                if cancel.load(Ordering::Relaxed) {
                    stop_cmd(ctx.cmd_pub);
                    return Outcome::Preempted;
                }
            }
            other => return other,
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
    // プランナ内部の作業解像度。密ソルバは状態配列 (56 B/state) を全域ぶん確保するので、
    // 広域地図では map_scale がそのままメモリを決める。compact ソルバなら確定出力
    // 12 B/state を sink (mmap) に逃がし、追従はロボット近傍のパッチだけを密に起こす。
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
    let use_compact = matches!(solver, U64Solver::Frontier2DSparseCompact { .. });
    // 密が実際に確保するのは `states` (State 56 B/state) だけではない。
    // `set_sweep_orders` が掃き順を 6 本ぶん持つ (`[0..3]` 各 total、`[4]` 1.5×total、
    // `[5]` 0.5×total = 合わせて 6.0×total の i32) ので +24 B/state。
    // 19F を map_scale 2 で解いたときの実測は 654.8 MB (states 444.7 + orders 210.1) で、
    // 56 B/state だけで見積もると 4 割以上足りない。compact は確定出力 12 B/state だけ。
    let states_gb = nstates as f64 * 80.0 / 1e9;
    let sink_gb = nstates as f64 * 12.0 / 1e9;
    // 先読みは価値関数を 2 本持つ (走っている点のと、次の点の)。密ならメモリ、
    // compact ならディスクがそのぶん要る。見積もりは常に 2 倍で語ること。
    let fields = if params.waypoint_prefetch { 2.0 } else { 1.0 };
    eprintln!(
        "vi_planner: planner grid {}x{} @{:.3}m (map_scale={}, downsample={}) x{} theta \
         = {} states{}",
        vi_grid.width,
        vi_grid.height,
        vi_grid.resolution,
        params.map_scale,
        params.downsample_policy,
        params.theta_cell_num,
        nstates,
        if use_compact {
            String::new()
        } else {
            format!(", dense states+sweep_orders {states_gb:.2} GB")
        },
    );
    // sink の置き場と、そこを選んだ理由の 1 行はこの中で出る (明示指定 →
    // compact_ram_limit_mb による自動退避 → RAM)。密ソルバなら None。
    let sink_dir = compact_sink_dir(&params, solver, nstates);
    if params.waypoint_prefetch {
        // 生きる場はちょうど 2 つ (走行中のと、次の) で頭打ちになる。新しい場を
        // 確保する前に必ず古いほうを手放しているため — 走行中の核は solve の前に
        // `cached = None`、先読みは注文を受ける前に採用待ちを捨てる。
        eprintln!(
            "vi_planner: waypoint prefetch on ({} threads) — {} for {:.2} GB \
             (2 x {:.2} GB: the goal being driven to, and the next one)",
            params.waypoint_prefetch_threads.max(1),
            if use_compact { "budget the sink directory" } else { "budget RAM" },
            if use_compact { sink_gb * 2.0 } else { states_gb * 2.0 },
            if use_compact { sink_gb } else { states_gb },
        );
    }
    // 密の見積もりが限度を超えたら**起動を止める**。ここまで来れば地図の実寸が
    // 分かっているので、launch 側の代理判定 (map_scale > 1 なら密を禁止) より正確。
    // 黙って確保して OOM killer に落とされるより、理由を出して止めるほうがよい。
    // 先読みを入れると場が 2 本になるので、限度と突き合わせるのも 2 本ぶん。
    if !use_compact && states_gb * fields * 1000.0 > params.dense_limit_mb.max(0) as f64 {
        return Err(anyhow!(
            "the dense value function needs {:.2} GB (states + sweep_orders) for {} states{}, over \
             dense_limit_mb={}.\n\
             Raise map_scale (halving the resolution quarters this), raise dense_limit_mb if the \
             machine really has the room, or switch to solver: frontier2d_sparse_compact \
             (+ compact_sink_dir), which keeps only {:.2} GB of finalized output and hydrates a \
             small patch around the robot for following. The local -> global feedback \
             (global_sweep) works there too — it repairs the sink tile by tile instead of \
             sweeping a full `states` array.",
            states_gb * fields,
            nstates,
            if params.waypoint_prefetch {
                " x2 value functions (waypoint_prefetch: true)"
            } else {
                ""
            },
            params.dense_limit_mb,
            sink_gb
        ));
    }
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
        compact_sink_dir: sink_dir,
        // 先読みを入れると場が 2 つ同時に生きるので、compact の確定出力は solve
        // ごとに使い捨てのディレクトリ (<compact_sink_dir>/gen<N>) へ分ける。
        // 固定ファイル名のままだと後から解くほうが先の場のファイルを truncate で
        // 潰す (mmap したまま長さが変わる = ゼロを読むか SIGBUS)。カウンタは
        // 先読み側の核と共有すること。
        compact_sink_gen: params.waypoint_prefetch.then(|| Arc::new(AtomicU64::new(0))),
        vi_threads: params.vi_threads.max(0) as usize,
        global_sweep: params.global_sweep,
        early_start: params.early_start,
    };
    if params.early_start {
        eprintln!(
            "vi_planner: early start on — a solve stops as soon as the policy reaches the goal \
             from the robot; everything off that path {}",
            if params.global_sweep {
                "is filled in while driving (dense: the global sweep; compact: the tile repair \
                 the follow loop seeds every time it writes its window back)"
            } else {
                "stays as it was cut — global_sweep is off, so nothing fills it in"
            }
        );
    }

    // 5b. ウェイポイントの先読み (waypoint_prefetch)。予備の核を 1 つ専用スレッドに
    //     持たせ、いまの点へ走っている間に次の点を解かせる。走行中の核のロックは
    //     取らないので、追従ループ (10Hz / try_lock) の邪魔はしない。
    let prefetch = params.waypoint_prefetch.then(|| {
        let cfg = PlanConfig {
            // 先読みの場に狭域の書き込みは無い = 伝播させる仕事が無いので、
            // 修復タイル (数 MB + 遷移表の再計算) を作らせない。
            global_sweep: false,
            // 追従の 40ms/tick を削らないよう既定 1 に絞る (compact のみ有効。
            // 密は VI_THREADS がプロセスで 1 つなので分けられない)。
            vi_threads: params.waypoint_prefetch_threads.max(1) as usize,
            // 先読みは最後まで解く。打ち切りの起点は「いまの機体の姿勢」だが、
            // 次の点に着く頃には機体はそこにいない (打ち切った場は使えない)。
            // ワーカーは起点を渡さないので実際には二重の歯止め。
            early_start: false,
            ..cfg.clone()
        };
        Prefetcher::spawn(build.clone(), cfg)
    });
    let mut core = PlannerCore::new(build, cfg);
    if let Some(pf) = prefetch.clone() {
        core = core.with_prefetch(pf);
    }
    let core = Arc::new(Mutex::new(core));

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

    // 6d-2. 狭域 → 広域のフィードバック: 共有価値関数の全域掃き。
    //
    // 追従 (`observe_scan`) が書いた local_penalty はローカルウィンドウ (±1m) の
    // 中で値を上げるだけで、そこから外へは広がらない。ここで同じ `states` を
    // 全域 Gauss–Seidel で掃き直して初めて、広域の経路が塞がった通路を避ける
    // ようになる (`core::sweep_global` の doc)。
    //
    // compact 経路も同じ入口で回る。向こうは全域の `states` を持てないので、
    // 1 呼び出しで sink のタイルを 1 枚だけ起こして掃いて書き戻す
    // (`core::Repair`)。伝播にかかる時間は地図の大きさではなく変化が及ぶ範囲に
    // 比例するので、最初の反応が遅くても掃き終わるまでの実測ログを見ること。
    //
    // **`value_function` の再配信もこのスレッドが持つ**。追従中は solve が
    // 走らないので、ここで出さないと全域スライスは solve した瞬間のまま固まり、
    // 伝播が効いていても画面では何も起きない (追従ループが出すのは ±1m の窓だけ
    // で、窓は機体と一緒に動くので離れると固まったままの全域が下から見える)。
    //
    // ロックの持ち方が肝。10 Hz の追従ループは同じ Mutex を `try_lock` で取り、
    // 3 tick 続けて取れないとロボットを止める。そこで 1 掃きを走り切らせず、
    // budget ms だけ掃いてロックを手放し、idle ms 待つ形にしてある
    // (既定 20:60 = 1 コアの 25%)。
    if params.global_sweep {
        let core = Arc::clone(&core);
        let viz = viz.clone();
        let budget = Duration::from_millis(params.global_sweep_budget_ms.max(1) as u64);
        let idle = Duration::from_millis(params.global_sweep_idle_ms.max(0) as u64);
        std::thread::spawn(move || {
            // 密経路の 1 チャンクの粒度。budget の中で経過時間を見る刻みでも
            // あるので、Pi4 (実測 1 掃き 7.9M セルで 8-11 秒 = 約 1M cells/s) で
            // 数 ms になる大きさにしてある。compact 経路はこの値を使わず、
            // 1 呼び出し = タイル 1 枚 (これも数十 ms で頭打ち)。
            const CELLS_PER_STEP: usize = 5_000;
            // 伝播が続いている間の報告間隔。**走行中は待ち行列がまず空にならない**
            // ので (壁が窓に入っていれば `set_local_cost` が毎 tick penalty を塗り
            // 直す)、下の「done」の行はほとんど出ない。掃きが動いているかを見る
            // 手掛かりはこの間隔で出る進捗のほうになる。
            //
            // 価値関数の再配信もここに乗せる。追従中は solve が走らないので、
            // ここで出さないと `value_function` は solve した瞬間のまま固まり、
            // **伝播が効いていても画面では何も起きない** (追従ループが出すのは
            // ±1m の窓だけ)。全域 1 枚は 19F (scale 2) で 13 万セル、津田沼
            // (scale 5) で 94 万セル読むので、10 Hz の追従ループには置けない。
            // 作るのはロックの中なので、この間隔の tick だけ追従ループが try_lock に
            // 失敗し得る (2 秒に 1 回なので 3 tick 連続にはならず、機体は止まらない)。
            const REPORT_EVERY: Duration = Duration::from_secs(2);
            let mut cur = SweepCursor::default();
            let mut sweep_start: Option<Instant> = None;
            let mut last_report = Instant::now();
            loop {
                // ロックの中で作って、外で配信する (可視化 1 枚は 100 万セル級)。
                let mut grid = None;
                {
                    let mut c = match core.lock() {
                        Ok(g) => g,
                        // 他スレッドの panic で毒されても価値関数自体は一貫している。
                        Err(p) => p.into_inner(),
                    };
                    if !c.is_dirty() {
                        // 掃く仕事が無い。ロックはすぐ返して、次に狭域が場を動かす
                        // まで長めに待つ (この確認自体もロックを要るので、間隔を
                        // 詰めると追従ループから取り上げてしまう)。
                        drop(c);
                        std::thread::sleep(idle.max(Duration::from_millis(200)));
                        continue;
                    }
                    let t0 = Instant::now();
                    let started = *sweep_start.get_or_insert(t0);
                    let mut done = false;
                    loop {
                        if c.sweep_global(&mut cur, CELLS_PER_STEP).1 {
                            done = true;
                            break;
                        }
                        if t0.elapsed() >= budget {
                            break;
                        }
                    }

                    if done || last_report.elapsed() >= REPORT_EVERY {
                        last_report = Instant::now();
                        if done {
                            // 伝播 1 回に実際かかった時間 (idle を含む実時間 =
                            // 狭域の判断が広域に届くまでの遅れそのもの)。budget/idle を
                            // 実測から詰められるよう必ず出す。`still_dirty=false` なら
                            // 新しい不動点に達したので、次の狭域の書き込みまで止まる。
                            //
                            // compact は 1 回の伝播で掃く量が地図の大きさで決まらない
                            // (変化が及んだ範囲で決まる) ので、経過時間だけでは速いのか
                            // 仕事が少なかったのか読めない。タイル数を必ず添える。
                            let work = match c.sweep_tiles() {
                                Some(n) => format!(", {n} tiles"),
                                None => String::new(),
                            };
                            eprintln!(
                                "vi_planner: global sweep done in {:.1}s{} (still_dirty={})",
                                started.elapsed().as_secs_f64(),
                                work,
                                c.is_dirty()
                            );
                            sweep_start = None;
                        } else if let Some((visits, queued)) = c.repair_progress() {
                            eprintln!(
                                "vi_planner: tile repair running for {:.1}s \
                                 ({visits} visits, {queued} tiles queued)",
                                started.elapsed().as_secs_f64()
                            );
                        }
                        // `interval: 0` は「solve 完了時のみ」なので、途中の
                        // 再配信はしない (伝播 1 回の完了は完了として出す)。
                        if let Some(v) = viz.as_ref() {
                            if done || !v.interval.is_zero() {
                                grid = c.value_grid(v.threshold_steps);
                            }
                        }
                    }
                }
                if let (Some(v), Some(g)) = (viz.as_ref(), grid) {
                    let _ = v.vf_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
                }
                std::thread::sleep(idle);
            }
        });
    }

    // 進行中の solve / 追従をプリエンプトするための cancel フラグ置き場。
    // 2 つのアクションは互いを止めない (広域の再計画で追従が死んではいけないし、
    // 追従中でも再計画は受け付ける) ので、スロットは別々に持つ。
    let plan_cancel: Arc<Mutex<Option<Arc<AtomicBool>>>> = Arc::new(Mutex::new(None));
    let follow_cancel: Arc<Mutex<Option<Arc<AtomicBool>>>> = Arc::new(Mutex::new(None));

    let node_clock = node.get_clock();
    let frame_id = params.global_frame.clone();
    // ロールアウト固着時のヒント表示用 (safety_radius_penalty [秒/セル], safety_radius [m])。
    let params_hint = (params.safety_radius_penalty, params.safety_radius);

    // 制御ループのチューニング。追従を回すサーバが 3 つ (follow_path /
    // navigate_to_pose / follow_waypoints) あるので、サーバの配線より先に作る。
    let period = Duration::from_secs_f64(1.0 / params.control_frequency);
    let tuning = FollowTuning {
        period,
        refine_budget: Duration::from_millis(params.refine_budget_ms.max(0) as u64),
        failure_ticks_limit: (params.no_action_timeout_sec.max(0.0) * params.control_frequency)
            .ceil()
            .max(1.0) as u32,
    };
    let retry = RetryTuning {
        limit: params.goal_retry_limit,
        settle: Duration::from_secs_f64(params.goal_retry_settle_sec.max(0.0)),
    };

    // スタンドアロンでは `compute_path_to_pose` を誰も呼ばないので、表示用の経路は
    // 追従側が出す。Nav2 構成では planner_server (ここでは compute_path_to_pose の
    // 成功) が出すトピックなので、そちらでは立てない。
    let plan_pub: Option<Arc<PlanPub>> = params
        .standalone
        .then(|| -> Result<Arc<PlanPub>> {
            Ok(Arc::new(PlanPub {
                path_pub: node.create_publisher::<nav_msgs::msg::Path>("plan".keep_last(1))?,
                clock: node.get_clock(),
                frame_id: params.global_frame.clone(),
            }))
        })
        .transpose()?;

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
                                "vi_planner: path with {} poses in {:.2}s \
                                 (solved_now={}, iters={}{}{})",
                                stats.poses,
                                dt.as_secs_f64(),
                                stats.solved_now,
                                stats.iters,
                                // 先読みが効いた回。solve を丸ごと飛ばしたので、
                                // ここが出ている間は点の切り替わりで機体が止まらない。
                                if stats.adopted { ", prefetched" } else { "" },
                                // 打ち切った場で答えた回 (early_start)。経路の外は
                                // 未確定なので、機体が外れると解き直しが要る。
                                if stats.truncated { ", truncated" } else { "" }
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

    // 6f. navigate_to_pose action サーバ (bt_navigator + behavior_server の置き換え)。
    //     **standalone のときだけ立てる** — Nav2 構成で立てると bt_navigator と
    //     2 つになり、クライアントは先に見つけたほうへ繋ぐ (どちらに繋がったかは
    //     どこにも出ないので、症状は「ときどき挙動が違う」になる)。
    let _nav_to_pose_server = if !params.standalone {
        None
    } else {
        let core = Arc::clone(&core);
        let latest_pose = Arc::clone(&latest_pose);
        let scan_queue = Arc::clone(&scan_queue);
        let cmd_pub = cmd_pub.clone();
        let follow_cancel = Arc::clone(&follow_cancel);
        let viz = viz.clone();
        let plan_pub = plan_pub.clone();
        let frame_id = frame_id.clone();
        let node_clock = node_clock.clone();

        Some(node.create_action_server::<nav2_msgs::action::NavigateToPose, _>(
            "navigate_to_pose",
            move |requested_goal: RequestedGoal<nav2_msgs::action::NavigateToPose>| {
                let core = Arc::clone(&core);
                let latest_pose = Arc::clone(&latest_pose);
                let scan_queue = Arc::clone(&scan_queue);
                let cmd_pub = cmd_pub.clone();
                let follow_cancel = Arc::clone(&follow_cancel);
                let viz = viz.clone();
                let plan_pub = plan_pub.clone();
                let frame_id = frame_id.clone();
                let node_clock = node_clock.clone();

                async move {
                    // ── プリエンプト: 前の追従を止め、自分の cancel を登録 ──
                    // スロットは follow_path と共有する。3 つのサーバは同じ 1 台を
                    // 走らせるので、どれが来ても前のものは止まらなければならない。
                    let my_cancel = Arc::new(AtomicBool::new(false));
                    {
                        let mut slot = follow_cancel.lock().unwrap();
                        if let Some(prev) = slot.take() {
                            prev.store(true, Ordering::SeqCst);
                        }
                        *slot = Some(Arc::clone(&my_cancel));
                    }

                    let accepted = requested_goal.accept();
                    let goal = pose_view_from(&accepted.goal().pose.pose);
                    let executing = accepted.execute();
                    eprintln!("vi_planner: navigate to ({:.2}, {:.2})", goal.x, goal.y);

                    let feedback = executing.feedback_publisher();
                    let (done_tx, done_rx) = futures::channel::oneshot::channel::<Outcome>();
                    let cancel_t = Arc::clone(&my_cancel);
                    std::thread::spawn(move || {
                        let ctx = FollowCtx {
                            core: &core,
                            latest_pose: &latest_pose,
                            scan_queue: &scan_queue,
                            cmd_pub: &cmd_pub,
                            plan_pub: plan_pub.as_deref(),
                            viz: viz.as_deref(),
                            tuning,
                        };
                        let retries = AtomicU64::new(0);
                        let t0 = Instant::now();
                        let outcome =
                            run_goal(&ctx, goal, &cancel_t, retry, &retries, &|p| {
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
                            executing.succeeded_with(
                                nav2_msgs::action::NavigateToPose_Result::default(),
                            )
                        }
                        Ok(Ok(Outcome::Preempted)) => {
                            eprintln!("vi_planner: preempted by a newer goal");
                            executing.aborted_with(
                                nav2_msgs::action::NavigateToPose_Result::default(),
                            )
                        }
                        Ok(Ok(Outcome::Failed(reason))) => {
                            eprintln!("ERROR: vi_planner: {reason}");
                            executing.aborted_with(
                                nav2_msgs::action::NavigateToPose_Result::default(),
                            )
                        }
                        Ok(Err(_)) => executing
                            .aborted_with(nav2_msgs::action::NavigateToPose_Result::default()),
                        Err(rest) => {
                            my_cancel.store(true, Ordering::SeqCst);
                            let cancelling = executing.begin_cancelling();
                            let _ = rest.await;
                            eprintln!("vi_planner: cancelled by client");
                            cancelling.cancelled_with(
                                nav2_msgs::action::NavigateToPose_Result::default(),
                            )
                        }
                    }
                }
            },
        )?)
    };

    // 6g. follow_waypoints action サーバ (nav2_waypoint_follower の置き換え)。
    //     **順路は配列の順**に回る (距離で並べ替えたりはしない。nav2 側も同じ)。
    //
    //     ここで順路を丸ごと受け取れることには、単に 1 ノード減る以上の意味がある:
    //     先読み (`waypoint_prefetch`) は「次の点」を知る手立てが
    //     `waypoint_topic` の latch しか無く、そこへ出すものがいない構成では
    //     **エラーも警告も出ないまま何も解かなかった**。ゴールと同じ経路で
    //     順路が入るので、その穴が塞がる。
    let _follow_waypoints_server = if !params.standalone {
        None
    } else {
        let core = Arc::clone(&core);
        let latest_pose = Arc::clone(&latest_pose);
        let scan_queue = Arc::clone(&scan_queue);
        let cmd_pub = cmd_pub.clone();
        let follow_cancel = Arc::clone(&follow_cancel);
        let viz = viz.clone();
        let plan_pub = plan_pub.clone();
        let prefetch = prefetch.clone();
        let stop_on_failure = params.waypoint_stop_on_failure;
        let pause = Duration::from_secs_f64(params.waypoint_pause_sec.max(0.0));

        Some(node.create_action_server::<nav2_msgs::action::FollowWaypoints, _>(
            "follow_waypoints",
            move |requested_goal: RequestedGoal<nav2_msgs::action::FollowWaypoints>| {
                let core = Arc::clone(&core);
                let latest_pose = Arc::clone(&latest_pose);
                let scan_queue = Arc::clone(&scan_queue);
                let cmd_pub = cmd_pub.clone();
                let follow_cancel = Arc::clone(&follow_cancel);
                let viz = viz.clone();
                let plan_pub = plan_pub.clone();
                let prefetch = prefetch.clone();

                async move {
                    let my_cancel = Arc::new(AtomicBool::new(false));
                    {
                        let mut slot = follow_cancel.lock().unwrap();
                        if let Some(prev) = slot.take() {
                            prev.store(true, Ordering::SeqCst);
                        }
                        *slot = Some(Arc::clone(&my_cancel));
                    }

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
                        let ctx = FollowCtx {
                            core: &core,
                            latest_pose: &latest_pose,
                            scan_queue: &scan_queue,
                            cmd_pub: &cmd_pub,
                            plan_pub: plan_pub.as_deref(),
                            viz: viz.as_deref(),
                            tuning,
                        };
                        let retries = AtomicU64::new(0);
                        let mut missed: Vec<i32> = Vec::new();
                        let mut outcome = TourOutcome::Done;
                        for (i, goal) in goals.iter().enumerate() {
                            let mut fb =
                                nav2_msgs::action::FollowWaypoints_Feedback::default();
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
                                    outcome = TourOutcome::Preempted;
                                    break;
                                }
                                Outcome::Failed(reason) => {
                                    eprintln!(
                                        "ERROR: vi_planner: waypoint {} failed: {reason}",
                                        i + 1
                                    );
                                    missed.push(i as i32);
                                    if stop_on_failure {
                                        outcome = TourOutcome::Stopped;
                                        break;
                                    }
                                }
                            }
                            // 次の点へ向かうまでの間 (`waypoint_pause_sec`)。単に
                            // 待つのではなく場を更新し続ける (run_settle)。
                            if !pause.is_zero() && !run_settle(&ctx, &cancel_t, pause) {
                                outcome = TourOutcome::Preempted;
                                break;
                            }
                        }
                        stop_cmd(&cmd_pub);
                        let _ = done_tx.send((outcome, missed));
                    });

                    let mut done_rx = done_rx;
                    match executing.until_cancel_requested(&mut done_rx).await {
                        Ok(Ok((outcome, missed))) => {
                            let mut result =
                                nav2_msgs::action::FollowWaypoints_Result::default();
                            result.missed_waypoints = missed;
                            match outcome {
                                TourOutcome::Done => {
                                    eprintln!(
                                        "vi_planner: tour finished ({} missed)",
                                        result.missed_waypoints.len()
                                    );
                                    executing.succeeded_with(result)
                                }
                                TourOutcome::Stopped => {
                                    eprintln!(
                                        "ERROR: vi_planner: tour stopped at the first failure \
                                         (stop_on_failure: true)"
                                    );
                                    executing.aborted_with(result)
                                }
                                TourOutcome::Preempted => {
                                    eprintln!("vi_planner: tour preempted");
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
                            cancelling.cancelled_with(
                                nav2_msgs::action::FollowWaypoints_Result::default(),
                            )
                        }
                    }
                }
            },
        )?)
    };

    // 6h. follow_path action サーバ (controller_server の置き換え)。
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
                    let ctx = FollowCtx {
                        core: &core_t,
                        latest_pose: &latest_pose_t,
                        scan_queue: &scan_queue_t,
                        cmd_pub: &cmd_pub_t,
                        // Nav2 構成で `plan` を出すのは compute_path_to_pose の側。
                        plan_pub: None,
                        viz: viz_t.as_deref(),
                        tuning,
                    };
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
                        cancelling
                            .cancelled_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                }
            }
        },
    )?;

    eprintln!(
        "vi_planner: ready (solver={}, actions=compute_path_to_pose + follow_path{}, {}Hz{})",
        params.solver,
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
