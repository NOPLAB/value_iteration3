//! vi_planner entry point — Nav2 の planner_server と controller_server を
//! **1 ノードで同時に**置き換える全 Rust ノード。`compute_path_to_pose` と
//! `follow_path` (nav2_msgs) の両方を提供し、どちらも同じ `PlannerCore` =
//! 同じ価値関数を読む。`standalone: true` ならさらに `navigate_to_pose` と
//! `follow_waypoints` も提供し、**Nav2 のノードを 1 つも立てずに**自律移動する
//! (アクション型は nav2_msgs のままなので RViz 等の配線は変わらない。何が
//! 良くなるかはリポジトリ CLAUDE.md を参照。投げ直しの前に [`run_settle`] が
//! 止まったまま場を更新するのが BT の `Wait` との本質的な違い)。
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

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context as ACtx, Result};

use vi_lib::bridge::{
    downsample_occupancy, downsample_occupancy_optimistic, occupancy_view_to_vi_grid,
    OccupancyGridView, PoseView,
};
use vi_lib::msg::LaserScan as ViLaserScan;
use vi_lib::planner::PathPose;
use vi_lib::solvers::U64Solver;
use vi_lib::Action;
// nav_msgs::msg::OccupancyGrid と名前が衝突するので別名で入れる。

use vi_planner::core::{
    lock, quality_shift, try_lock, value_grid_on, AdaptiveLocalizer, BeliefConfig, BuildParams,
    Decision, ExternalLocalizer, FollowKind, GridLocalizer, Localizer, PlanConfig, PlanError,
    PlanStats, PlannerCore, Prefetcher, SweepCursor,
};

use rclrs::*;

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
    /// map→odom TF を配信するか (AMCL の契約の置き換え)。外部 localizer が
    /// map→odom を出す構成 (emcl2/AMCL) では off のまま — 出すのは常に 1 人。
    publish_tf: bool,
    /// TF スタンプの未来日付け [s] (AMCL の transform_tolerance 相当)。
    transform_tolerance: f64,
    /// publish_tf 用の odom 購読先 (T_odom→base の出どころ)。
    odom_topic: String,
    // ── 自己位置推定 (core::Localizer) ──
    /// 自己位置の出どころ ("external" | "grid" | "adaptive" | "viterbi")。
    localizer: String,
    /// grid: belief 窓の半径 [m] / 尤度場の σ [m] / ビーム間引き / レンジ上限 [m]。
    belief_radius: f64,
    belief_sensor_sigma: f64,
    belief_beam_step: i64,
    belief_max_range: f64,
    /// grid: predict 1 tick あたりの動作ノイズ σ ([m] / [deg])。
    belief_motion_sigma_xy: f64,
    belief_motion_sigma_theta_deg: f64,
    /// grid: ビームごとの尤度の床 / 補正で読む重みの相対しきい値。
    belief_z_min: f64,
    belief_weight_skip_ratio: f64,
    /// スキャン注入の品質ゲート: localizer の観測一致度がこれを割ると注入
    /// penalty を quality/gate に比例して減衰 (2 冪量子化)。0 で無効。
    scan_quality_gate: f64,
    /// 地図帰属サプレッション半径 [m]: ヒット点からこの距離以内に地図障害物が
    /// あるスキャンヒットは注入しない (壁の再投影ゴースト対策)。0 で無効。
    scan_attribution_m: f64,
    /// footprint クリア半径 [m]: スキャン注入のたびに機体位置の周囲の
    /// local_penalty を消す (真上でゴースト壁が閉じるのを防ぐ)。0 で無効。
    footprint_clear_m: f64,
    // ── 広域 (compute_path_to_pose) ──
    max_rollout_steps: i64,
    start_tolerance: f64,
    path_spacing: f64,
    // ── 狭域 (follow_path) ──
    /// ローカルウィンドウ半径 [m] (本家 ValueIteratorLocal は 1.0 固定)。
    local_xy_range: f64,
    follow: bool,
    scan_topic: String,
    control_frequency: f64,
    refine_budget_ms: i64,
    action_tolerance: f64,
    no_action_timeout_sec: f64,
    /// 無効レンジ (inf / NaN / 非正) の差し替え値 [m]。
    invalid_range_m: f64,
    /// ロックをこの tick 数連続で取れなかったら停止指令を出す。
    busy_ticks_before_stop: i64,
    /// compact パッチの寸法スラック [セル] / 修復タイルの interior の 1 辺 [セル]。
    patch_slack_cells: i64,
    repair_interior_cells: i64,
    /// follow 1 tick の判断器 ("greedy" = 本家 decision / "dwa"・"mppi" = 連続行動)。
    follow_controller: String,
    /// belief が多峰のとき QMDP (Q(b,a) = Σ w·Q(s,a) の argmin) で行動を選ぶ。
    /// grid/adaptive localizer 用 (external は仮説を出さないので実質無効)。
    qmdp: bool,
    /// ロスト中に安全停止で待つ代わりに、仮説を判別する地点への多目標 VI を解いて
    /// QMDP で走る (能動的再定位)。adaptive/viterbi localizer + 密ソルバ用。
    active_reloc: bool,
    /// 能動的再定位を諦めて通常の停止待ちに戻すまでの時間 [s]。
    reloc_timeout_sec: f64,
    /// DWA/MPPI の前方シミュレーション時間 [s] と DWA の (v, ω) 候補数。
    dwa_horizon_s: f64,
    dwa_n_v: i64,
    dwa_n_w: i64,
    /// DWA の致死 penalty しきい値 (PROB_BASE 単位、0 = 無効)。
    dwa_lethal_penalty: f64,
    /// MPPI のサンプル本数 / softmax 温度 / 制御ノイズ標準偏差 (0 = 行動集合から自動)。
    mppi_samples: i64,
    mppi_lambda: f64,
    mppi_sigma_v: f64,
    mppi_sigma_w_deg: f64,
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
    /// 密経路の 1 チャンクの粒度 [セル] / 伝播中の進捗報告と価値関数再配信の間隔 [s]。
    global_sweep_cells_per_step: i64,
    global_sweep_report_sec: f64,
    // ── ウェイポイントの先読み ──
    waypoint_prefetch: bool,
    waypoint_topic: String,
    waypoint_prefetch_threads: i64,
    /// 進行中の先読みを待つときの観測間隔 [ms]。
    waypoint_prefetch_poll_ms: i64,
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

    Ok(Params {
        solver: p!("solver", Arc<str>, "frontier2d_sparse".into()).to_string(),
        theta_cell_num: p!("theta_cell_num", i64, 60),
        safety_radius: p!("safety_radius", f64, 0.2),
        safety_radius_penalty: p!("safety_radius_penalty", i64, 30),
        goal_margin_radius: p!("goal_margin_radius", f64, 0.3),
        goal_margin_theta_deg: p!("goal_margin_theta", f64, 15.0),
        map_wait_sec: p!("map_wait_sec", i64, 30),
        action_list: names.into_iter().zip(fws).zip(rots).map(|((n, f), r)| (n, f, r)).collect(),
        unknown_as_obstacle: p!("unknown_as_obstacle", bool, true),
        map_scale: p!("map_scale", i64, 1),
        // "conservative" = 本家 downsample_occupancy (障害物優先)。"optimistic" = ブロック内に free が
        // 1 つでもあれば free。map_scale >= 4 で通路のセル幅を保つために必要 (既定は挙動不変の保守側)。
        downsample_policy: p!("downsample_policy", Arc<str>, "conservative".into()).to_string(),
        // solver = frontier2d_sparse_compact のときだけ効く。空文字なら RAM (RamSink)。
        // 指定先が tmpfs だと結局 RAM に載るので、メモリ退避が目的なら実ディスクを指すこと。
        compact_sink_dir: p!("compact_sink_dir", Arc<str>, "".into()).to_string(),
        // compact_sink_dir 未指定でも、確定出力がこれを超えるならディスクへ逃がす。小メモリ機で
        // 黙って GB 級の RamSink を確保すると OOM killer に落とされるため。
        compact_ram_limit_mb: p!("compact_ram_limit_mb", i64, 512),
        // 密ソルバの価値関数 (states 56 B/state + sweep_orders 24 B/state) の上限。
        // 超えたら起動を止める。既定 1500 は 4GB 機 (Pi4) で他のノードと同居できる線
        // — 19F を map_scale 2 で解くと実測 655 MB。
        dense_limit_mb: p!("dense_limit_mb", i64, 1500),
        vi_threads: p!("vi_threads", i64, 0),
        max_solve_iter: p!("max_solve_iter", i64, 1_000_000),
        solve_chunk: p!("solve_chunk", i64, 64),
        goal_tolerance_xy: p!("goal_tolerance_xy", f64, 0.25),
        goal_tolerance_deg: p!("goal_tolerance_deg", f64, 10.0),
        pose_topic: p!("pose_topic", Arc<str>, "mcl_pose".into()).to_string(),
        global_frame: p!("global_frame", Arc<str>, "map".into()).to_string(),
        publish_tf: p!("publish_tf", bool, false),
        transform_tolerance: p!("transform_tolerance", f64, 0.5),
        odom_topic: p!("odom_topic", Arc<str>, "odom".into()).to_string(),

        // 自己位置の出どころ。"external" (既定) = pose_topic の推定をそのまま使う
        // (mcl 等の外部推定器)。"grid" = 内蔵ヒストグラム MCL — pose_topic は
        // **手動シード** (initialpose 等) として扱い、メッセージごとに belief を
        // 初期化し直す。以後は scan_topic と自分の出した cmd_vel だけで推定する
        // ので、"grid" のときは pose_topic を mcl の連続出力に向けないこと
        // (毎メッセージでリセットされて素通しと変わらなくなる)。
        // "viterbi" = adaptive の全域レベルを min-plus (MAP) で回す変種
        // (vi_lib の localize/viterbi.rs — 運動整合性で偽仮説を削る)。
        // "adaptive" = grid の多重解像度版 — 観測一致度が落ちると belief を粗い
        // 広域レベルへ広げて再定位する (EMCL の expansion resetting 相当) ので、
        // 誘拐 (持ち上げ移動) から復帰でき、未シードなら大域初期化で立ち上がる。
        // ロスト中は pose を返さず follow ループが安全停止する。シード運用は grid と同じ。
        localizer: p!("localizer", Arc<str>, "external".into()).to_string(),
        // belief 窓は native 解像度 (map_scale をかける前) の 2×radius 四方 × θ。
        // 0.05 m/cell で radius 2.5 なら 100×100×60 = 60 万セル ≈ 5 MB。
        belief_radius: p!("belief_radius", f64, 2.5),
        belief_sensor_sigma: p!("belief_sensor_sigma", f64, 0.2),
        belief_beam_step: p!("belief_beam_step", i64, 10),
        belief_max_range: p!("belief_max_range", f64, 25.0),
        belief_motion_sigma_xy: p!("belief_motion_sigma_xy", f64, 0.03),
        belief_motion_sigma_theta_deg: p!("belief_motion_sigma_theta_deg", f64, 2.0),
        // 尤度の床 (本家 likelihood field の z_rand/z_max 混合に相当) と、補正で
        // 読む belief 重みの相対しきい値 (max との比、小さいほど正確で遅い)。
        belief_z_min: p!("belief_z_min", f64, 0.05),
        belief_weight_skip_ratio: p!("belief_weight_skip_ratio", f64, 1e-4),
        // フィットが怪しいスキャンに満額 (2048) の壁を建てさせない。既定は
        // adaptive の expansion しきい値 (expand_quality) と同じ 0.25。external
        // localizer は quality 1.0 固定なので実質無効。
        scan_quality_gate: p!("scan_quality_gate", f64, 0.25),
        // 地図障害物の近傍に落ちたヒットは壁の再投影 (pose 誤差のゴースト) と
        // みなして注入しない。半径 = pose 誤差の許容幅。副作用: 地図の壁のこの
        // 距離以内に立つ本物の新規障害物も注入されない (静的 penalty 帯 +
        // dwa_lethal_penalty がそこを守る)。
        scan_attribution_m: p!("scan_attribution_m", f64, 0.4),
        // ロボットが現にいる場所は free — 注入のたびに footprint を消し、
        // ゴースト壁が真上で閉じて完全停止する事態を構造的に防ぐ。
        footprint_clear_m: p!("footprint_clear_m", f64, 0.2),

        max_rollout_steps: p!("max_rollout_steps", i64, 10_000),
        start_tolerance: p!("start_tolerance", f64, 0.5),
        path_spacing: p!("path_spacing", f64, 0.05),

        // follow_path サーバを立てるか。false は nav2_controller (controller_server) と
        // 組む構成 — 立てると follow_path のサーバが 2 つになるため。false のとき
        // このノードは compute_path_to_pose 専用になる。
        // 追従が見る・スキャンで補正するウィンドウの半径。広げると 1 tick の
        // refine 対象と compact パッチ (辺 ∝ 2×半径) が大きくなる。
        local_xy_range: p!("local_xy_range", f64, 1.0),
        follow: p!("follow", bool, true),
        scan_topic: p!("scan_topic", Arc<str>, "scan".into()).to_string(),
        control_frequency: p!("control_frequency", f64, 10.0),
        refine_budget_ms: p!("refine_budget_ms", i64, 40),
        action_tolerance: p!("action_tolerance", f64, 0.2),
        no_action_timeout_sec: p!("no_action_timeout_sec", f64, 3.0),
        // 無効レンジの差し替え値。ローカルウィンドウから十分遠く、セル座標化しても
        // i32 に収まること (set_local_cost がウィンドウ外として自然に無視する)。
        invalid_range_m: p!("invalid_range_m", f64, 1.0e6),
        // 1〜2 tick は同一ゴールのロールアウト (BT の 1Hz リプラン) との競合なので
        // 止めない。control_frequency 10Hz なら 3 tick = 300ms。
        busy_ticks_before_stop: p!("busy_ticks_before_stop", i64, 3),
        // compact パッチの寸法スラックと修復タイルの interior (詳細は core の doc)。
        patch_slack_cells: p!("patch_slack_cells", i64, 2),
        repair_interior_cells: p!("repair_interior_cells", i64, 16),
        // follow 1 tick の判断器。"greedy" (既定) は本家 ViNode::decision 準拠の離散
        // 6 行動。"dwa" / "mppi" は同じ価値関数を連続に読む (V̂ 補間 + 軌道サンプリング、
        // core::follow の doc 参照)。指令の速度範囲は行動集合と同じで、候補全滅時は
        // greedy へフォールバックするので、切り替えても失敗の形は変わらない。
        follow_controller: p!("follow_controller", Arc<str>, "greedy".into()).to_string(),
        // DWA/MPPI の前方シミュレーション時間と DWA の候補数 (計算量 ∝ n_v × n_w × horizon)。
        // 既定 (1.0 s, 7×11) の実測は decide ~30 µs — 10 Hz の 40 ms 予算には遠い。
        dwa_horizon_s: p!("dwa_horizon_s", f64, 1.0),
        dwa_n_v: p!("dwa_n_v", i64, 7),
        dwa_n_w: p!("dwa_n_w", i64, 11),
        // 軌道途中に margin 帯 (penalty ≥ 2·PROB_BASE) を踏む候補を棄却する。
        // 0 で無効 (実機で壁を掠める旧挙動に戻る)。
        dwa_lethal_penalty: p!("dwa_lethal_penalty", f64, 2.0),
        // MPPI のサンプル本数・温度・ノイズ (0 = 行動集合から自動)。実測 decide ~0.2 ms。
        mppi_samples: p!("mppi_samples", i64, 256),
        mppi_lambda: p!("mppi_lambda", f64, 1.0),
        mppi_sigma_v: p!("mppi_sigma_v", f64, 0.0),
        mppi_sigma_w_deg: p!("mppi_sigma_w_deg", f64, 0.0),
        // belief が多峰 (grid/adaptive の上位仮説が 2 個以上) の tick は
        // QMDP: Q(b,a) = Σ w·Q(s,a) の argmin で「どの仮説でも悪くない行動」を
        // 選び、有意な仮説が衝突と言う行動しか無ければ止まる。単峰の tick は
        // 従来の follow_controller に退避するので、収束中の挙動だけが変わる。
        qmdp: p!("qmdp", bool, false),
        // ロスト中の能動的再定位: 仮説集合を最も判別する地点 (reloc_targets) を
        // 多目標 VI のゴールにして QMDP で走る。復帰したら本来のゴールを
        // 解き直して走行に戻る (core::prepare_reloc_goal の doc 参照)。
        active_reloc: p!("active_reloc", bool, false),
        reloc_timeout_sec: p!("reloc_timeout_sec", f64, 30.0),

        // navigate_to_pose と follow_waypoints をこのノード自身が提供する
        // (= bt_navigator / behavior_server / waypoint_follower を立てない)。
        // **既定は false**: Nav2 構成でこれを立てると navigate_to_pose のサーバが
        // bt_navigator と 2 つになり、クライアントは先に見つけたほうへ繋ぐ
        // (どちらに繋がったかはログにも出ない)。立てるのは launch の責任。
        standalone: p!("standalone", bool, false),
        // 追従が失敗したときにゴールを投げ直す上限 (負で無制限)。BT の
        // `RecoveryNode number_of_retries` の置き換えだが、あちらと違って
        // **投げ直しの合間に場が実際に動く** (下の goal_retry_settle_sec)。
        goal_retry_limit: p!("goal_retry_limit", i64, 3),
        // 投げ直す前に、止まったままスキャンを取り込んで場を精密化する時間。
        // 0 で即座に投げ直す (それだと同じ場で同じ失敗を繰り返しやすい)。
        goal_retry_settle_sec: p!("goal_retry_settle_sec", f64, 3.0),
        // follow_waypoints: 1 点失敗したら残りを諦めるか。false なら次の点へ進み、
        // 飛ばした番号を result.missed_waypoints で返す (nav2_waypoint_follower と同義)。
        waypoint_stop_on_failure: p!("stop_on_failure", bool, false),
        // 1 点に着いてから次の点へ向かうまでの待ち (nav2_waypoint_follower の
        // waypoint_pause_duration [ms] に相当。こちらは秒)。
        waypoint_pause_sec: p!("waypoint_pause_sec", f64, 0.2),

        // 狭域が書いた local_penalty を全域へ伝播させる背景掃き (core::sweep_global)。
        // これを止めると、狭域が「通れない」と判断しても広域の経路は塞がった通路を
        // 指し続ける (旧挙動)。密ソルバは全域 Gauss–Seidel、compact は sink の
        // タイル修復で同じことをする (どちらも `core::sweep_global` の入口)。
        global_sweep: p!("global_sweep", bool, true),
        // 1 回のロック取得で掃きに使う時間と、そのあとロックを手放して待つ時間。
        // 比がそのまま CPU の取り分になる (既定 20:60 = 1 コアの 25%)。追従ループは
        // 同じ Mutex を try_lock で取り、3 tick 続けて取れないとロボットを止めるので、
        // budget を伸ばすときは idle も一緒に伸ばすこと。
        global_sweep_budget_ms: p!("global_sweep_budget_ms", i64, 20),
        global_sweep_idle_ms: p!("global_sweep_idle_ms", i64, 60),
        // 密経路の 1 チャンクの粒度 (budget の経過時間を見る刻み)。Pi4 (実測 ~1M
        // cells/s) で数 ms になる大きさ。compact 経路では使わない (1 呼び出し =
        // タイル 1 枚)。
        global_sweep_cells_per_step: p!("global_sweep_cells_per_step", i64, 5_000),
        // 伝播が続いている間の進捗報告と value_function 再配信の間隔。可視化 1 枚は
        // 100 万セル級をロックの中で作るので、詰めすぎると追従ループの try_lock が
        // 落ちる (3 tick 連続で機体が止まる)。
        global_sweep_report_sec: p!("global_sweep_report_sec", f64, 2.0),

        // 次のウェイポイントの価値関数を、いまの点へ走っている間に解いておく
        // (core::Prefetcher)。巡回では点が変わるたびに solve が丸ごと 1 回走り、その間
        // 機体が止まる (19F で 29 秒、津田沼 compact で 87 秒) — それを走行時間に隠す。
        // **既定は false**: 価値関数が同時に 2 つ生きるので、密ならメモリが、compact
        // なら sink のディスクが最大 2 倍 + 採用待ちの 1 つぶん要る。効くのは
        // waypoint_topic に並びを出すもの (daifuku_waypoint_manager) がいるときだけで、
        // 並びが無ければ立ち上がっていても何も解かない。
        waypoint_prefetch: p!("waypoint_prefetch", bool, false),
        // 先読み対象の並び (nav_msgs/Path)。transient_local で購読するので、後から
        // 起動しても latch されている並びを拾える。
        waypoint_topic: p!("waypoint_topic", Arc<str>, "waypoints".into()).to_string(),
        // 先読みの solve に使うスレッド数。追従は 10Hz・予算 40ms/tick で回っているので
        // 既定 1 に絞ってある。**効くのは compact 経路だけ** — 密の frontier2d_sparse は
        // スレッド数を環境変数 VI_THREADS から読む (プロセスで 1 つ) ので、先読みだけを
        // 絞ることができない。
        waypoint_prefetch_threads: p!("waypoint_prefetch_threads", i64, 1),
        // 進行中の先読みを待つときの観測間隔。プリエンプトの効きの粒度でもある。
        waypoint_prefetch_poll_ms: p!("waypoint_prefetch_poll_ms", i64, 50),

        // 機体の現在地からゴールまで方策が繋がった時点で solve を打ち切って走り出す
        // (core::PlanConfig::early_start)。**既定は false**: 経路の外は未確定のままに
        // なるので、機体が経路から外れると解き直しが要る (そのときは打ち切った場を
        // 捨ててから最後まで解くので、待ちは合計で長くなる)。先読み
        // (waypoint_prefetch) と違い追加のメモリは要らず、両者は併用できる
        // — 先読みで用意した場は打ち切らないので、受け取れた点では関係しない。
        early_start: p!("early_start", bool, false),

        publish_value_function: p!("publish_value_function", bool, true),
        // solve の途中経過の配信間隔。0 は「完了時のみ」。追従中の再配信は掃き
        // スレッドが 2 秒ごとに出すので、ここを 0 にすると追従中は出なくなる。
        value_publish_interval_ms: p!("value_publish_interval_ms", i64, 500),
        cost_drawing_threshold: p!("cost_drawing_threshold", i64, 60),
        window_cost_drawing_threshold: p!("window_cost_drawing_threshold", i64, 60),
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
    if p.local_xy_range <= 0.0 {
        return Err(anyhow!("local_xy_range must be > 0, got {}", p.local_xy_range));
    }
    // standalone は navigate_to_pose / follow_waypoints が追従本体を回すので
    // follow を切る組み合わせは成立しない。
    if p.standalone && !p.follow {
        return Err(anyhow!("standalone: true requires follow: true"));
    }
    if FollowKind::from_name(&p.follow_controller).is_none() {
        return Err(anyhow!(
            "unknown follow_controller: {} (expected \"greedy\", \"dwa\" or \"mppi\")",
            p.follow_controller
        ));
    }
    if !matches!(p.localizer.as_str(), "external" | "grid" | "adaptive" | "viterbi") {
        return Err(anyhow!(
            "unknown localizer: {} (expected \"external\", \"grid\", \"adaptive\" or \"viterbi\")",
            p.localizer
        ));
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

/// sensor_msgs/LaserScan → vi_lib::LaserScan。ビーム角と添字の対応を
/// 保つため無効レンジは取り除かず `invalid_range_m` に差し替える
/// (`set_local_cost` がウィンドウ外として自然に無視する)。
fn vi_scan_from(msg: &sensor_msgs::msg::LaserScan, invalid_range_m: f64) -> ViLaserScan {
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
                    invalid_range_m
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
/// 判断順は明示指定が先、上限による自動退避が後。追従はパッチを置き直すたびに
/// sink を読むので (10Hz の制御ループ)、SD カード上の sink は追従の遅延に直結する。
/// 逃がす先はできるだけ実ディスクでも速いところを `compact_sink_dir` で明示すること。
fn compact_sink_dir(params: &Params, solver: U64Solver, nstates: usize) -> Option<PathBuf> {
    if !solver.caps().out_of_core {
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

/// vi_lib の可視化描画済み OccupancyGrid → ROS メッセージ。
fn ros_grid_from(
    g: &vi_lib::msg::OccupancyGrid,
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
    /// ロックをこの tick 数連続で取れなかったら停止指令を出す。
    busy_ticks_before_stop: u32,
    /// belief が多峰のとき QMDP (`decide_qmdp`) で行動を選ぶ (単峰は従来どおり)。
    qmdp: bool,
    /// スキャン注入の品質ゲート ([`quality_shift`] の gate)。0 で無効。
    scan_quality_gate: f64,
    /// ロスト中に判別点へ走る能動的再定位 (`prepare_reloc_goal` + QMDP)。
    active_reloc: bool,
    /// 能動的再定位を諦めて通常の停止待ちに戻すまでの tick 数。
    reloc_ticks_limit: u32,
}

/// QMDP に渡す belief 仮説の上限。ヒストグラムの上位セルだけで質量の大半を
/// 覆える (それ以下は veto 判定も動かさない微小仮説)。
const QMDP_TOP_K: usize = 64;
/// 能動的再定位中の近接停止 [m]。姿勢が無く local_penalty を置けないので、
/// 生の最近接レンジで守る。
// ponytail: 定数 1 個 — 機体寸法に合わせるならパラメータへ昇格。
const RELOC_STOP_RANGE: f64 = 0.35;

/// 能動的再定位 (`active_reloc`) の段階 (run_follow のロスト分岐が使う)。
enum Reloc {
    /// ロストしていない / まだ再定位場を解いていない。
    Idle,
    /// 再定位場の上を QMDP で走行中 (このときキャッシュ = 再定位場)。
    Driving,
    /// この構成では使えない (compact 等) — このゴールでは以後試さない。
    GaveUp,
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
    localizer: &'a Mutex<Box<dyn Localizer>>,
    scan_queue: &'a Mutex<Vec<ViLaserScan>>,
    cmd_pub: &'a Publisher<geometry_msgs::msg::Twist>,
    /// 表示専用の経路。None なら出さない (`follow_path` 構成では BT 側の
    /// `compute_path_to_pose` が出すので不要)。
    plan_pub: Option<&'a PlanPub>,
    viz: Option<&'a Viz>,
    tuning: FollowTuning,
}

/// 4 つのアクションサーバ・購読コールバック・掃きスレッドが共有するハンドル束。
/// 各サーバはこれを 1 つ clone するだけでよい (以前は 6〜10 個の Arc を
/// closure 用と内側 async 用に二重 clone していた)。
struct Handles {
    core: Mutex<PlannerCore>,
    latest_pose: Mutex<Option<PoseView>>,
    /// 自己位置推定器 (core::Localizer)。`latest_pose` はこれの出力キャッシュ —
    /// 読む側 (follow ループ・plan サーバ) は従来どおり latest_pose だけを見る。
    localizer: Mutex<Box<dyn Localizer>>,
    scan_queue: Mutex<Vec<ViLaserScan>>,
    cmd_pub: Publisher<geometry_msgs::msg::Twist>,
    viz: Option<Viz>,
    /// 表示専用の経路 (`plan`)。standalone のときだけ Some。
    plan_pub: Option<PlanPub>,
    /// 推定姿勢の表示用出力 (`viola_pose`)。シードとスキャン補正のたびに出す。
    est_pub: Publisher<geometry_msgs::msg::PoseStamped>,
    est_frame: String,
    est_clock: Clock,
    /// map→odom TF (`publish_tf`)。odom が届くたびに latest_pose と合成して出す
    /// (odom レートで出すので、スキャン間も TF が新鮮なまま)。
    tf_pub: Option<Publisher<tf2_msgs::msg::TFMessage>>,
    tf_tolerance: Duration,
}

impl Handles {
    /// 推定姿勢を `viola_pose` へ。
    fn publish_est(&self, p: PoseView) {
        let mut msg = geometry_msgs::msg::PoseStamped::default();
        msg.header.frame_id = self.est_frame.as_str().into();
        let (sec, nanosec) = self.est_clock.now().to_sec_nanosec().unwrap_or((0, 0));
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nanosec;
        msg.pose.position.x = p.x;
        msg.pose.position.y = p.y;
        msg.pose.orientation.z = (p.yaw_rad / 2.0).sin();
        msg.pose.orientation.w = (p.yaw_rad / 2.0).cos();
        let _ = self.est_pub.publish(msg);
    }

    /// map→odom = T_map→base(推定) · T_odom→base⁻¹ を `/tf` へ (AMCL の契約)。
    /// スキャン時刻より新しい参照にも答えられるよう、スタンプは
    /// transform_tolerance だけ未来へ日付ける (AMCL と同じ手当て)。
    fn publish_tf(&self, est: PoseView, odo: PoseView, odom_frame: &str) {
        let Some(tf_pub) = self.tf_pub.as_ref() else { return };
        let th = est.yaw_rad - odo.yaw_rad;
        let (s, c) = th.sin_cos();
        let mut t = geometry_msgs::msg::TransformStamped::default();
        t.header.frame_id = self.est_frame.as_str().into();
        t.child_frame_id = odom_frame.into();
        let (sec, nanosec) = self.est_clock.now().to_sec_nanosec().unwrap_or((0, 0));
        let total =
            sec as i64 * 1_000_000_000 + nanosec as i64 + self.tf_tolerance.as_nanos() as i64;
        t.header.stamp.sec = (total / 1_000_000_000) as i32;
        t.header.stamp.nanosec = (total % 1_000_000_000) as u32;
        t.transform.translation.x = est.x - (c * odo.x - s * odo.y);
        t.transform.translation.y = est.y - (s * odo.x + c * odo.y);
        t.transform.rotation.z = (th / 2.0).sin();
        t.transform.rotation.w = (th / 2.0).cos();
        let mut msg = tf2_msgs::msg::TFMessage::default();
        msg.transforms = vec![t];
        let _ = tf_pub.publish(msg);
    }

    /// 追従ループ用の借用ビュー。`with_plan: false` は Nav2 構成の follow_path
    /// (表示用の `plan` は BT 側の compute_path_to_pose が出す)。
    fn follow_ctx(&self, tuning: FollowTuning, with_plan: bool) -> FollowCtx<'_> {
        FollowCtx {
            core: &self.core,
            latest_pose: &self.latest_pose,
            localizer: &self.localizer,
            scan_queue: &self.scan_queue,
            cmd_pub: &self.cmd_pub,
            plan_pub: if with_plan { self.plan_pub.as_ref() } else { None },
            viz: self.viz.as_ref(),
            tuning,
        }
    }
}

/// プリエンプト: 前のゴールの cancel を立て、自分の cancel をスロットへ置く。
/// 追従を回す 3 つのサーバはスロットを共有する — 同じ 1 台を走らせるので、
/// どれが来ても前のものは止まらなければならない。
fn preempt(slot: &Mutex<Option<Arc<AtomicBool>>>) -> Arc<AtomicBool> {
    let my = Arc::new(AtomicBool::new(false));
    let mut slot = lock(slot);
    if let Some(prev) = slot.take() {
        prev.store(true, Ordering::SeqCst);
    }
    *slot = Some(Arc::clone(&my));
    my
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
    let FollowCtx { core, latest_pose, localizer, scan_queue, cmd_pub, plan_pub, viz, tuning } =
        *ctx;
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
            let g = value_grid_on(vi, v.threshold_steps);
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
    let mut reloc = Reloc::Idle;
    let mut reloc_ticks = 0u32;
    loop {
        let tick_start = Instant::now();
        if cancel.load(Ordering::Relaxed) {
            stop_cmd(cmd_pub);
            return Outcome::Preempted;
        }

        let pose = *latest_pose.lock().unwrap();
        // 能動的再定位から復帰した直後: キャッシュには再定位場が載っている。
        // 下の「goal replaced」プリエンプトと誤認する前に本来のゴールを解き直す
        // (再定位中に本当に別の追従ゴールが来る形は cancel 経由で上で抜けている)。
        if matches!(reloc, Reloc::Driving) {
            if let Some(p) = pose {
                let Some(mut core) = try_lock(core) else {
                    if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
                        std::thread::sleep(rest);
                    }
                    continue;
                };
                stop_cmd(cmd_pub); // 解き直しの数秒、直前の指令で這わせない
                if !core.is_cached_goal(goal) {
                    eprintln!(
                        "vi_planner: re-localized at ({:.2}, {:.2}) — re-solving the goal \
                         [active_reloc]",
                        p.x, p.y
                    );
                    if let Err(e) = core.prepare_goal(goal, cancel) {
                        return match e {
                            PlanError::Cancelled => Outcome::Preempted,
                            e => Outcome::Failed(e.to_string()),
                        };
                    }
                }
                reloc = Reloc::Idle;
                reloc_ticks = 0;
                failure_ticks = 0;
            }
        }
        let Some(pose) = pose else {
            // ── ロスト (pose なし)。既定は安全停止で受動復帰 (expansion resetting)
            // を待つ。active_reloc なら仮説を判別する地点の多目標場を解き、
            // QMDP で走って復帰を早める (core::prepare_reloc_goal の doc)。
            let mut acted = false;
            if tuning.active_reloc
                && reloc_ticks < tuning.reloc_ticks_limit
                && !matches!(reloc, Reloc::GaveUp)
            {
                reloc_ticks += 1;
                acted = 'reloc: {
                    // localizer → core の順 (入れ子ロックを作らない)。
                    let (hyps, targets) = {
                        let l = lock(localizer);
                        (l.top_cells(QMDP_TOP_K), l.reloc_targets())
                    };
                    if hyps.len() < 2 {
                        break 'reloc false;
                    }
                    // 近接ガード: 姿勢が無く local_penalty を置けないので、生の
                    // 最近接レンジで守る (スキャンはコールバックで localizer に
                    // 反映済みなので捨ててよい)。
                    let scans = std::mem::take(&mut *scan_queue.lock().unwrap());
                    if scans.last().is_some_and(|s| {
                        s.ranges
                            .iter()
                            .filter(|r| r.is_finite() && **r > 0.0)
                            .fold(f64::INFINITY, |a, &r| a.min(r))
                            < RELOC_STOP_RANGE
                    }) {
                        stop_cmd(cmd_pub);
                        break 'reloc true; // 止まって観測を待つのも再定位のうち
                    }
                    let Some(mut core) = try_lock(core) else { break 'reloc false };
                    if matches!(reloc, Reloc::Idle) {
                        if targets.is_empty() {
                            break 'reloc false;
                        }
                        match core.prepare_reloc_goal(&targets, cancel) {
                            Ok(_) => {
                                eprintln!(
                                    "vi_planner: lost — driving toward {} disambiguation \
                                     target(s) [active_reloc]",
                                    targets.len()
                                );
                                reloc = Reloc::Driving;
                            }
                            Err(PlanError::Cancelled) => break 'reloc false,
                            Err(e) => {
                                // compact 構成など — 受動復帰に任せる。
                                eprintln!("vi_planner: active_reloc unavailable ({e})");
                                reloc = Reloc::GaveUp;
                                break 'reloc false;
                            }
                        }
                    }
                    match core.decide_qmdp(&hyps) {
                        Decision::Action { fw, rot_deg, .. } => {
                            drop(core); // localizer を取る前に手放す (ロック順序)
                            let mut tw = geometry_msgs::msg::Twist::default();
                            tw.linear.x = fw;
                            tw.angular.z = rot_deg.to_radians();
                            let _ = cmd_pub.publish(tw);
                            let mut l = lock(localizer);
                            l.predict(fw, rot_deg, tuning.period.as_secs_f64());
                            *lock(latest_pose) = l.pose();
                            true
                        }
                        // Goal = 判別点に到着 / NoAction — 止まって観測を待つ。
                        _ => {
                            stop_cmd(cmd_pub);
                            true
                        }
                    }
                };
            }
            if acted {
                failure_ticks = 0;
            } else {
                stop_cmd(cmd_pub);
                failure_ticks += 1;
                if failure_ticks >= tuning.failure_ticks_limit {
                    return Outcome::Failed("no robot pose for too long".into());
                }
            }
            if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
                std::thread::sleep(rest);
            }
            continue;
        };
        // QMDP 用の belief 仮説と、スキャン注入の品質ゲート (直近補正の観測
        // 一致度 → 減衰段数)。core のロックの前に取る (localizer と core の
        // 入れ子ロックを作らない)。external localizer は空 = 点推定へ退避。
        let (hyps, scan_shift) = {
            let l = lock(localizer);
            (
                if tuning.qmdp { l.top_cells(QMDP_TOP_K) } else { Vec::new() },
                quality_shift(l.quality(), tuning.scan_quality_gate),
            )
        };

        // ── ロックを取るのはこのブロックだけ ──
        //
        // `lock()` ではなく `try_lock()` にしてある。広域側が **別ゴール** の
        // 計画要求を受けると、その solve は数秒〜数十秒ロックを握り続ける。
        // ここでブロックすると、その間ずっと直前の速度指令が出たままロボットが
        // 走り続けることになる (velocity_smoother のタイムアウト任せにしない)。
        let Some(mut core) = try_lock(core) else {
            // 1〜2 tick 取れないのは同一ゴールのロールアウト待ち (BT の 1Hz
            // リプランはキャッシュヒットでも rollout + densify をロック内で回す)。
            // ここで毎回止めると、取り除いたはずの「1 秒ごとに 0 速度が挟まる」
            // 挙動が戻ってしまうので、直前の指令を保ったまま次の tick を待つ。
            // 連続で取れない = 本当に長い solve が走っているので、そのときだけ止める。
            busy_ticks += 1;
            if busy_ticks >= tuning.busy_ticks_before_stop {
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
                core.observe_scan_gated(scan, pose, scan_shift);
            }
            core.refine_for(tuning.refine_budget);

            // 可視化グリッドの作成はロック内 (states を読む) / 配信は外。
            let mut window_grid = None;
            if let Some(v) = viz {
                if v.due(&mut last_viz) {
                    window_grid = core.window_value_grid(pose, v.window_threshold_steps);
                }
            }
            // 多峰 belief (仮説 2 個以上) は QMDP — どの仮説でも悪くない行動を
            // 選び、有意な仮説が衝突と言う行動しか無ければ止まる。単峰は従来の
            // decide (follow_controller 経由) と一致するので点推定に退避。
            let decision = if hyps.len() >= 2 {
                core.decide_qmdp(&hyps)
            } else {
                core.decide(pose)
            };
            (decision, core.goal_distance(pose.x, pose.y), window_grid)
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
                // 自分が出した指令がそのまま推定器の動作モデル (external では
                // no-op)。dt は制御周期 — 実際の適用時間との差は motion ノイズが
                // 吸収する。停止指令は動きゼロなので predict しない。代入は
                // 無条件 — adaptive がロストしたら latest_pose ごと消え、次 tick
                // から「pose なし」の安全停止に入る。
                {
                    let mut l = lock(localizer);
                    l.predict(fw, rot_deg, tuning.period.as_secs_f64());
                    *lock(latest_pose) = l.pose();
                }
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
            if lock(shared).discard_truncated() {
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
    let FollowCtx { core, latest_pose, localizer, scan_queue, cmd_pub, tuning, .. } = *ctx;
    let deadline = Instant::now() + dur;
    while Instant::now() < deadline {
        let tick_start = Instant::now();
        if cancel.load(Ordering::Relaxed) {
            return false;
        }
        stop_cmd(cmd_pub);

        let pose = *latest_pose.lock().unwrap();
        if let Some(pose) = pose {
            if let Some(mut c) = try_lock(core) {
                let scan_shift =
                    quality_shift(lock(localizer).quality(), tuning.scan_quality_gate);
                let scans = std::mem::take(&mut *scan_queue.lock().unwrap());
                c.set_window(pose);
                for scan in &scans {
                    c.observe_scan_gated(scan, pose, scan_shift);
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
    // 自己位置推定器。grid は native 解像度の占有格子から尤度場を起こすので、
    // ダウンサンプル前の binary_grid をここで使い切ってから捨てる。
    let localizer: Box<dyn Localizer> = match params.localizer.as_str() {
        kind @ ("grid" | "adaptive" | "viterbi") => {
            let bc = BeliefConfig {
                half_m: params.belief_radius.max(0.5),
                sensor_sigma_m: params.belief_sensor_sigma.max(0.01),
                beam_step: params.belief_beam_step.max(1) as usize,
                max_range_m: params.belief_max_range.max(0.1),
                motion_sigma_xy_m: params.belief_motion_sigma_xy.max(0.0),
                motion_sigma_theta_deg: params.belief_motion_sigma_theta_deg.max(0.0),
                z_min: params.belief_z_min.clamp(0.0, 1.0),
                weight_skip_ratio: params.belief_weight_skip_ratio.max(0.0) as f32,
                // "viterbi" = adaptive の全域レベルを min-plus (MAP) で回す変種。
                viterbi: kind == "viterbi",
                ..BeliefConfig::default()
            };
            if kind == "grid" {
                let g = GridLocalizer::new(&binary_grid, params.theta_cell_num as i32, bc);
                eprintln!(
                    "vi_planner: localizer = grid ({}m window @{}m x{} theta, {:.1} MB belief; \
                     seed it via {} — e.g. remap to initialpose)",
                    params.belief_radius * 2.0,
                    binary_grid.resolution,
                    params.theta_cell_num,
                    g.belief_mb(),
                    params.pose_topic
                );
                Box::new(g)
            } else {
                let g = AdaptiveLocalizer::new(&binary_grid, params.theta_cell_num as i32, bc);
                eprintln!(
                    "vi_planner: localizer = {} ({} levels up to whole-map, {:.1} MB \
                     belief; seed via {} or leave unseeded for global init; lost → pose \
                     withheld until re-localized)",
                    kind,
                    g.num_levels(),
                    g.belief_mb(),
                    params.pose_topic
                );
                Box::new(g)
            }
        }
        _ => Box::new(ExternalLocalizer::default()),
    };
    drop(binary_grid);
    let nstates =
        vi_grid.width as usize * vi_grid.height as usize * params.theta_cell_num as usize;
    let use_compact = solver.caps().out_of_core;
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
        local_xy_range: params.local_xy_range,
        scan_attribution_m: params.scan_attribution_m,
        patch_slack_cells: params.patch_slack_cells.max(0) as i32,
        repair_interior_cells: params.repair_interior_cells.max(1) as i32,
    };
    // validate() 済みなので必ず解ける。
    let follow_kind = FollowKind::from_name(&params.follow_controller)
        .ok_or_else(|| anyhow!("unknown follow_controller: {}", params.follow_controller))?;
    if follow_kind == FollowKind::Dwa {
        eprintln!(
            "vi_planner: follow controller = dwa (continuous; horizon {:.2} s, {}x{} candidates, \
             greedy fallback)",
            params.dwa_horizon_s, params.dwa_n_v, params.dwa_n_w
        );
    } else if follow_kind == FollowKind::Mppi {
        eprintln!(
            "vi_planner: follow controller = mppi (continuous; horizon {:.2} s, {} samples, \
             lambda {:.2}, greedy fallback)",
            params.dwa_horizon_s, params.mppi_samples, params.mppi_lambda
        );
    }
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
        follow_controller: follow_kind,
        footprint_clear_m: params.footprint_clear_m,
        dwa_tick_s: 1.0 / params.control_frequency,
        dwa_horizon_s: params.dwa_horizon_s,
        dwa_n_v: params.dwa_n_v.max(2) as usize,
        dwa_n_w: params.dwa_n_w.max(3) as usize,
        dwa_lethal_penalty: params.dwa_lethal_penalty.max(0.0),
        mppi_samples: params.mppi_samples.max(2) as usize,
        mppi_lambda: params.mppi_lambda.max(1e-3),
        mppi_sigma_v: params.mppi_sigma_v.max(0.0),
        mppi_sigma_w_deg: params.mppi_sigma_w_deg.max(0.0),
        compact_sink_dir: sink_dir,
        // 先読みを入れると場が 2 つ同時に生きるので、compact の確定出力は solve
        // ごとに使い捨てのディレクトリ (<compact_sink_dir>/gen<N>) へ分ける。
        // 固定ファイル名のままだと後から解くほうが先の場のファイルを truncate で
        // 潰す (mmap したまま長さが変わる = ゼロを読むか SIGBUS)。カウンタは
        // 先読み側の核と共有すること。
        compact_sink_gen: params.waypoint_prefetch.then(|| Arc::new(AtomicU64::new(0))),
        vi_threads: params.vi_threads.max(0) as usize,
        prefetch_poll_ms: params.waypoint_prefetch_poll_ms.max(1) as u64,
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

    // 6a. 自己位置トピック購読 (tf2 代替)。external ではそのまま採用、grid では
    //     belief の手動シード (どちらも Localizer::set_pose 経由)。
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
    //     (external では no-op。grid の補正は belief の広がりに比例したコストで、
    //     収束後は数百セル × 数十ビーム — エグゼキュータスレッドで足りる)。
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
                    // 無条件代入: adaptive はロスト中 None を返すので、latest_pose
                    // ごと消して follow ループを「pose なし」停止経路に乗せる
                    // (external / grid は observe 後も Some のままで従来どおり)。
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

    // 6c. 狭域 → 広域のフィードバック: 共有価値関数の全域掃き。
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
        let h = Arc::clone(&handles);
        let budget = Duration::from_millis(params.global_sweep_budget_ms.max(1) as u64);
        let idle = Duration::from_millis(params.global_sweep_idle_ms.max(0) as u64);
        // 密経路の 1 チャンクの粒度。budget の中で経過時間を見る刻みでもあるので、
        // Pi4 (実測 1 掃き 7.9M セルで 8-11 秒 = 約 1M cells/s) で数 ms になる
        // 大きさにしてある。compact 経路はこの値を使わず、1 呼び出し = タイル 1 枚
        // (これも数十 ms で頭打ち)。
        let cells_per_step = params.global_sweep_cells_per_step.max(1) as usize;
        // 伝播が続いている間の報告間隔。**走行中は待ち行列がまず空にならない**
        // ので (壁が窓に入っていれば `set_local_cost` が毎 tick penalty を塗り
        // 直す)、下の「done」の行はほとんど出ない。掃きが動いているかを見る
        // 手掛かりはこの間隔で出る進捗のほうになる。
        //
        // 価値関数の再配信もここに乗せる。追従中は solve が走らないので、
        // ここで出さないと `value_function` は solve した瞬間のまま固まり、
        // **伝播が効いていても画面では何も起きない** (追従ループが出すのは
        // ローカルウィンドウだけ)。全域 1 枚は 19F (scale 2) で 13 万セル、津田沼
        // (scale 5) で 94 万セル読むので、10 Hz の追従ループには置けない。
        // 作るのはロックの中なので、この間隔の tick だけ追従ループが try_lock に
        // 失敗し得る (2 秒に 1 回なら 3 tick 連続にはならず、機体は止まらない)。
        let report_every = Duration::from_secs_f64(params.global_sweep_report_sec.max(0.1));
        std::thread::spawn(move || {
            let mut cur = SweepCursor::default();
            let mut sweep_start: Option<Instant> = None;
            let mut last_report = Instant::now();
            loop {
                // ロックの中で作って、外で配信する (可視化 1 枚は 100 万セル級)。
                let mut grid = None;
                {
                    let mut c = lock(&h.core);
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
                        if c.sweep_global(&mut cur, cells_per_step).1 {
                            done = true;
                            break;
                        }
                        if t0.elapsed() >= budget {
                            break;
                        }
                    }

                    if done || last_report.elapsed() >= report_every {
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
                        if let Some(v) = h.viz.as_ref() {
                            if done || !v.interval.is_zero() {
                                grid = c.value_grid(v.threshold_steps);
                            }
                        }
                    }
                }
                if let (Some(v), Some(g)) = (h.viz.as_ref(), grid) {
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
        busy_ticks_before_stop: params.busy_ticks_before_stop.max(1) as u32,
        qmdp: params.qmdp,
        scan_quality_gate: params.scan_quality_gate,
        active_reloc: params.active_reloc,
        reloc_ticks_limit: (params.reloc_timeout_sec.max(0.0) * params.control_frequency)
            .ceil() as u32,
    };
    let retry = RetryTuning {
        limit: params.goal_retry_limit,
        settle: Duration::from_secs_f64(params.goal_retry_settle_sec.max(0.0)),
    };

    // 6d. compute_path_to_pose action サーバ (planner_server の置き換え)。
    let _plan_server = {
        let handles = Arc::clone(&handles);
        let plan_cancel = Arc::clone(&plan_cancel);
        let frame_id = frame_id.clone();
        let node_clock = node_clock.clone();

        node.create_action_server::<nav2_msgs::action::ComputePathToPose, _>(
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
                    let h_t = Arc::clone(&h);
                    let frame_t = frame_id.clone();
                    std::thread::spawn(move || {
                        let mut core = lock(&h_t.core);
                        let mut last_viz: Option<Instant> = None;
                        let result =
                            core.plan_with_progress(start, goal, &my_cancel, &mut |vi| {
                                let Some(v) = h_t.viz.as_ref() else { return };
                                if !v.due(&mut last_viz) {
                                    return;
                                }
                                let g = value_grid_on(vi, v.threshold_steps);
                                let _ =
                                    v.vf_pub.publish(ros_grid_from(&g, &frame_t, v.stamp()));
                            });
                        // solve が走った場合のみ完成形を配信し直す。
                        if let (Ok((_, stats)), Some(v)) = (&result, h_t.viz.as_ref()) {
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
                                    vi_lib::planner::RolloutStatus::LoopDetected
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

    // 6e. navigate_to_pose action サーバ (bt_navigator + behavior_server の置き換え)。
    //     **standalone のときだけ立てる** — Nav2 構成で立てると bt_navigator と
    //     2 つになり、クライアントは先に見つけたほうへ繋ぐ (どちらに繋がったかは
    //     どこにも出ないので、症状は「ときどき挙動が違う」になる)。
    let _nav_to_pose_server = if !params.standalone {
        None
    } else {
        let handles = Arc::clone(&handles);
        let follow_cancel = Arc::clone(&follow_cancel);
        let frame_id = frame_id.clone();
        let node_clock = node_clock.clone();

        Some(node.create_action_server::<nav2_msgs::action::NavigateToPose, _>(
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

    // 6e′. /goal_pose 購読 (bt_navigator の goal_pose→NavigateToPose 変換の
    //      置き換え)。RViz の「Nav2 Goal」も「2D Goal Pose」も PoseStamped を
    //      このトピックへ出すだけで、アクションに変換するのは bt_navigator
    //      だった — standalone はそれも置き換えているのでここで直接受ける。
    //      経路はアクションと同じ run_goal、違いは feedback/result の宛先が
    //      無いことだけ (プリエンプトの席も同じ 1 つを使う)。
    let _goal_pose_sub = if !params.standalone {
        None
    } else {
        let handles = Arc::clone(&handles);
        let follow_cancel = Arc::clone(&follow_cancel);
        Some(node.create_subscription::<geometry_msgs::msg::PoseStamped, _>(
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
                        Outcome::Preempted => {
                            eprintln!("vi_planner: preempted by a newer goal")
                        }
                        Outcome::Failed(reason) => eprintln!("ERROR: vi_planner: {reason}"),
                    }
                });
            },
        )?)
    };

    // 6f. follow_waypoints action サーバ (nav2_waypoint_follower の置き換え)。
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
        let handles = Arc::clone(&handles);
        let follow_cancel = Arc::clone(&follow_cancel);
        let prefetch = prefetch.clone();
        let stop_on_failure = params.waypoint_stop_on_failure;
        let pause = Duration::from_secs_f64(params.waypoint_pause_sec.max(0.0));

        Some(node.create_action_server::<nav2_msgs::action::FollowWaypoints, _>(
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
                            cancelling.cancelled_with(
                                nav2_msgs::action::FollowWaypoints_Result::default(),
                            )
                        }
                    }
                }
            },
        )?)
    };

    // 6g. follow_path action サーバ (controller_server の置き換え)。
    // `follow: false` (nav2_controller と組む構成) では立てない。
    let _follow_server = params.follow.then(|| node.create_action_server::<nav2_msgs::action::FollowPath, _>(
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
                        cancelling
                            .cancelled_with(nav2_msgs::action::FollowPath_Result::default())
                    }
                }
            }
        },
    )).transpose()?;

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

    let _sub = node.create_subscription::<nav_msgs::msg::OccupancyGrid, _>(
        "map".transient_local().reliable().keep_last(1),
        move |msg: nav_msgs::msg::OccupancyGrid| {
            let _ = tx.try_send(msg);
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
