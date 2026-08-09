//! 起動パラメータ — 宣言 ([`read_params`]) と自己整合検査 ([`validate`])。
//!
//! 既定値と「なぜその既定か」はフィールドのコメントに全部書いてある
//! (launch から上書きするときの判断材料はここが一次情報)。
//!
//! **ここに無いものは意図的に無い。** 正しい値が 1 つしかない内部の粒度と
//! センチネル (掃きのチャンク幅、パッチのスラック、無効レンジの差し替え値 …)、
//! マシンから測れるもの (密の価値関数の上限 = `MemAvailable`)、同じことを 2 回
//! 言っていたもの (窓と全域で別々のカラースケール上限) は、それぞれの使用箇所の
//! `const` へ落とした — 探すなら [`super::boot`] / [`super::sweep`] /
//! [`super::follow_loop`] / [`super::msg`] の中。

use anyhow::{anyhow, Result};

use std::sync::Arc;

use vi_lib::solvers::U64Solver;

use vi_planner::core::FollowKind;

use rclrs::*;

pub struct Params {
    // ── 共有 (価値関数の定義そのもの) ──
    pub solver: String,
    pub theta_cell_num: i64,
    pub safety_radius: f64,
    pub safety_radius_penalty: i64,
    pub goal_margin_radius: f64,
    pub goal_margin_theta_deg: f64,
    pub map_wait_sec: i64,
    pub action_list: Vec<(String, f64, f64)>,
    pub unknown_as_obstacle: bool,
    /// プランナ内部で地図を何倍に粗くするか (1 = /map のまま)。
    pub map_scale: i64,
    /// ダウンサンプルの方針 ("conservative" | "optimistic")。
    pub downsample_policy: String,
    /// compact ソルバの確定出力を置くディレクトリ (空文字 = 空きメモリ次第で自動)。
    pub compact_sink_dir: String,
    /// RAM sink の上限 [MB]。0 = 空きメモリの半分で自動判定。
    pub compact_ram_limit_mb: i64,
    pub vi_threads: i64,
    pub goal_tolerance_xy: f64,
    pub goal_tolerance_deg: f64,
    pub pose_topic: String,
    pub global_frame: String,
    /// map→odom TF を配信するか (AMCL の契約の置き換え)。外部 localizer が
    /// map→odom を出す構成 (emcl2/AMCL) では off のまま — 出すのは常に 1 人。
    pub publish_tf: bool,
    /// TF スタンプの未来日付け [s] (AMCL の transform_tolerance 相当)。
    pub transform_tolerance: f64,
    /// publish_tf 用の odom 購読先 (T_odom→base の出どころ)。
    pub odom_topic: String,
    // ── 自己位置推定 (窓つき core::Localizer / 全地図 vi_lib::belief::Belief) ──
    /// 自己位置の出どころ ("external" | "grid" | "adaptive" | "belief" | "viterbi")。
    pub localizer: String,
    /// grid/adaptive: belief 窓の半径 [m] (全地図の belief/viterbi では使わない)。
    pub belief_radius: f64,
    /// 尤度場の σ [m] / ビーム間引き / レンジ上限 [m]。
    pub belief_sensor_sigma: f64,
    pub belief_beam_step: i64,
    pub belief_max_range: f64,
    /// predict 1 tick あたりの動作ノイズ σ ([m] / [deg])。
    pub belief_motion_sigma_xy: f64,
    pub belief_motion_sigma_theta_deg: f64,
    /// スキャン注入の品質ゲート: localizer の観測一致度がこれを割ると注入
    /// penalty を quality/gate に比例して減衰 (2 冪量子化)。0 で無効。
    pub scan_quality_gate: f64,
    /// footprint クリア半径 [m]: スキャン注入のたびに機体位置の周囲の
    /// local_penalty を消す (真上でゴースト壁が閉じるのを防ぐ)。0 で無効。
    pub footprint_clear_m: f64,
    /// 自己位置の広がり σ [m] × この係数 を、壁際マージンの膨張量にする
    /// (上田ら 2023 4·2·2 の「マージン m に σ を足す」)。0 で無効。
    /// σ は localizer の上位仮説の RMS 半径なので、`localizer` が external の
    /// ときは常に 0 = 無効。
    pub sigma_margin_gain: f64,
    pub map_clear_from_scan: bool,
    // ── 広域 (compute_path_to_pose) ──
    pub max_rollout_steps: i64,
    pub path_spacing: f64,
    // ── 狭域 (follow_path) ──
    /// ローカルウィンドウ半径 [m] (本家 ValueIteratorLocal は 1.0 固定)。
    pub local_xy_range: f64,
    pub follow: bool,
    pub scan_topic: String,
    pub control_frequency: f64,
    pub refine_budget_ms: i64,
    pub no_action_timeout_sec: f64,
    /// follow 1 tick の判断器 ("greedy" = 本家 decision / "dwa"・"mppi" = 連続行動)。
    pub follow_controller: String,
    /// belief が多峰のとき QMDP (Q(b,a) = Σ w·Q(s,a) の argmin) で行動を選ぶ。
    /// 内蔵推定器 (grid/adaptive/belief/viterbi) 用 — external は仮説を出さないので
    /// 実質無効。
    pub qmdp: bool,
    /// ロスト中に安全停止で待つ代わりに、仮説を判別する地点への多目標 VI を解いて
    /// QMDP で走る (能動的再定位)。判別点を出せる adaptive localizer + 密ソルバ用。
    pub active_reloc: bool,
    /// DWA/MPPI の前方シミュレーション時間 [s]。
    pub dwa_horizon_s: f64,
    /// DWA の致死 penalty しきい値 (PROB_BASE 単位、0 = 無効)。
    pub dwa_lethal_penalty: f64,
    /// MPPI のサンプル本数。
    pub mppi_samples: i64,
    // ── スタンドアロン (navigate_to_pose / follow_waypoints) ──
    pub standalone: bool,
    pub goal_retry_limit: i64,
    pub waypoint_stop_on_failure: bool,
    pub waypoint_pause_sec: f64,
    // ── 狭域 → 広域のフィードバック (全域掃き) ──
    pub global_sweep: bool,
    /// 全域掃きに渡す CPU の割合 [%]。
    pub global_sweep_duty: i64,
    // ── ウェイポイントの先読み ──
    pub waypoint_prefetch: bool,
    pub waypoint_topic: String,
    pub waypoint_prefetch_threads: i64,
    // ── 走り出しの短縮 ──
    pub early_start: bool,
    // ── 可視化 ──
    /// 価値関数の配信間隔 [ms]。負で配信そのものを止め、0 で solve 完了時のみ。
    pub value_publish_interval_ms: i64,
    pub cost_drawing_threshold: i64,
}

pub fn read_params(node: &Node) -> Result<Params> {
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

    let mut params = Params {
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
        // solver = frontier2d_sparse_compact のときだけ効く。空文字なら空きメモリを見て
        // 自動で決める (RAM に収まらなければディスクへ退避 — node::boot::compact_sink_dir)。
        // 指定先が tmpfs だと結局 RAM に載るので、メモリ退避が目的なら実ディスクを指すこと。
        compact_sink_dir: p!("compact_sink_dir", Arc<str>, "".into()).to_string(),
        // 確定出力を RAM (RamSink) に置いてよい上限。既定 0 = このマシンの空きメモリの
        // 半分で自動判定。密の価値関数の上限と違ってこちらは**明示できる**ようにしてある
        // — sink は追従ループが毎パッチ読み直す常時アクセス先で、ディスクへ逃がすか
        // どうかは同居プロセスと保存先の速さ次第、つまり運用側にしか分からない。
        compact_ram_limit_mb: p!("compact_ram_limit_mb", i64, 0),
        goal_tolerance_xy: p!("goal_tolerance_xy", f64, 0.25),
        goal_tolerance_deg: p!("goal_tolerance_deg", f64, 10.0),
        pose_topic: p!("pose_topic", Arc<str>, "mcl_pose".into()).to_string(),
        global_frame: p!("global_frame", Arc<str>, "map".into()).to_string(),
        publish_tf: p!("publish_tf", bool, false),
        transform_tolerance: p!("transform_tolerance", f64, 0.5),
        odom_topic: p!("odom_topic", Arc<str>, "odom".into()).to_string(),

        // 自己位置の出どころ。"external" (既定) = pose_topic の推定をそのまま使う
        // (mcl 等の外部推定器)。内蔵推定器は 2 系統あり、どちらも pose_topic を
        // **手動シード** (initialpose 等) として扱ってメッセージごとに belief を
        // 張り直す。以後は scan_topic と自分の出した cmd_vel だけで推定するので、
        // 内蔵のときは pose_topic を mcl の連続出力に向けないこと (毎メッセージで
        // リセットされて素通しと変わらなくなる)。ロスト中は pose を返さず follow
        // ループが安全停止する。
        //
        //   窓つき (belief_radius の窓だけに belief を持つ):
        //     "grid"     = 内蔵ヒストグラム MCL。
        //     "adaptive" = grid の多重解像度版 — 観測一致度が落ちると belief を粗い
        //                  広域レベルへ広げて再定位する (EMCL の expansion resetting
        //                  相当) ので、誘拐から復帰でき、未シードなら大域初期化で
        //                  立ち上がる。能動的再定位 (active_reloc) の判別点を出せる
        //                  のもこれだけ。
        //   全地図 (窓もレベル機構も無く、プランナと同じ格子に belief を全域で持つ):
        //     "belief"   = 和積 (sum-product)。未シードなら最初のスキャンで free
        //                  一様から立ち上がり、観測が合わなくなれば一様混合でリセット。
        //     "viterbi"  = 同じ belief を min-plus (MAP) 半環で回す変種 (運動整合性で
        //                  偽仮説を削る。全域を掃くので 1 observe ≈ 183 ms — 40 ms の
        //                  追従予算の外)。
        //
        // 尤度の床・枝刈りしきい値・リセット/ロストの判定値は vi_lib 側の既定
        // (`BeliefConfig::default()`) をそのまま使う。詰めるなら viola_bench で。
        localizer: p!("localizer", Arc<str>, "external".into()).to_string(),
        // grid/adaptive の belief 窓は native 解像度 (map_scale をかける前) の
        // 2×radius 四方 × θ。0.05 m/cell で radius 2.5 なら 100×100×60 = 60 万セル
        // ≈ 5 MB。全地図の belief/viterbi では使わない。
        belief_radius: p!("belief_radius", f64, 2.0),
        belief_sensor_sigma: p!("belief_sensor_sigma", f64, 0.2),
        belief_beam_step: p!("belief_beam_step", i64, 10),
        belief_max_range: p!("belief_max_range", f64, 25.0),
        belief_motion_sigma_xy: p!("belief_motion_sigma_xy", f64, 0.03),
        belief_motion_sigma_theta_deg: p!("belief_motion_sigma_theta_deg", f64, 2.0),
        // フィットが怪しいスキャンに満額 (2048) の壁を建てさせない。既定は
        // vi_lib の reset_quality (と adaptive の expand しきい値) と同じ 0.25。
        // external localizer は quality 1.0 固定なので実質無効。
        scan_quality_gate: p!("scan_quality_gate", f64, 0.25),
        // ロボットが現にいる場所は free — 注入のたびに footprint を消し、
        // ゴースト壁が真上で閉じて完全停止する事態を構造的に防ぐ。
        footprint_clear_m: p!("footprint_clear_m", f64, 0.2),
        // 自己位置が曖昧なほど壁から離れて通る (文献のマージン膨張)。挙動が
        // 変わるので既定 off — 内蔵 localizer と併せて使うこと。
        sigma_margin_gain: p!("sigma_margin_gain", f64, 0.0),
        // 同じ理屈をビームが貫通した範囲まで広げる: 地図が壁と言っていても
        // レーザが通り抜けたなら通れる。幽霊壁に囲まれた停止を解くが、自己位置が
        // ずれていると本物の壁を消し得るので既定 off。
        map_clear_from_scan: p!("map_clear_from_scan", bool, false),

        max_rollout_steps: p!("max_rollout_steps", i64, 10_000),
        path_spacing: p!("path_spacing", f64, 0.05),

        // 追従が見る・スキャンで補正するウィンドウの半径。広げると 1 tick の
        // refine 対象と compact パッチ (辺 ∝ 2×半径) が大きくなる。
        local_xy_range: p!("local_xy_range", f64, 1.0),
        // follow_path サーバを立てるか。false は nav2_controller (controller_server) と
        // 組む構成 — 立てると follow_path のサーバが 2 つになるため。false のとき
        // このノードは compute_path_to_pose 専用になる。
        follow: p!("follow", bool, true),
        scan_topic: p!("scan_topic", Arc<str>, "scan".into()).to_string(),
        control_frequency: p!("control_frequency", f64, 10.0),
        refine_budget_ms: p!("refine_budget_ms", i64, 40),
        no_action_timeout_sec: p!("no_action_timeout_sec", f64, 3.0),
        // follow 1 tick の判断器。"greedy" (既定) は本家 ViNode::decision 準拠の離散
        // 6 行動。"dwa" / "mppi" は同じ価値関数を連続に読む (V̂ 補間 + 軌道サンプリング、
        // core::follow の doc 参照)。指令の速度範囲は行動集合と同じで、候補全滅時は
        // greedy へフォールバックするので、切り替えても失敗の形は変わらない。
        follow_controller: p!("follow_controller", Arc<str>, "greedy".into()).to_string(),
        // belief が多峰の tick は QMDP: Q(b,a) = Σ w·Q(s,a) の argmin で
        // 「どの仮説でも悪くない行動」を選び、有意な仮説が衝突と言う行動しか
        // 無ければ止まる。単峰の tick は従来の follow_controller に退避するので、
        // 収束中の挙動だけが変わる。多峰性は**セル数ではなく峰の数**で測る
        // (core::mode_count — 全地図 belief は収束していても非ゼロセルが数千残る
        // ので、セル数だと常に多峰になり follow_controller が一度も動かない)。
        qmdp: p!("qmdp", bool, false),
        // ロスト中の能動的再定位: 仮説集合を最も判別する地点 (reloc_targets) を
        // 多目標 VI のゴールにして QMDP で走る。復帰したら本来のゴールを
        // 解き直して走行に戻る (core::prepare_reloc_goal の doc 参照)。
        active_reloc: p!("active_reloc", bool, false),
        // DWA/MPPI の前方シミュレーション時間。候補数と温度は固定 (node::boot)。
        // 既定 (1.0 s, 7×11) の実測は decide ~30 µs — 10 Hz の 40 ms 予算には遠い。
        dwa_horizon_s: p!("dwa_horizon_s", f64, 1.0),
        // 軌道途中に margin 帯 (penalty ≥ 2·PROB_BASE) を踏む候補を棄却する。
        // 0 で無効 (実機で壁を掠める旧挙動に戻る)。
        dwa_lethal_penalty: p!("dwa_lethal_penalty", f64, 2.0),
        // MPPI のサンプル本数。実測 decide ~0.2 ms。
        mppi_samples: p!("mppi_samples", i64, 256),

        // navigate_to_pose と follow_waypoints をこのノード自身が提供する
        // (= bt_navigator / behavior_server / waypoint_follower を立てない)。
        // **既定は false**: Nav2 構成でこれを立てると navigate_to_pose のサーバが
        // bt_navigator と 2 つになり、クライアントは先に見つけたほうへ繋ぐ
        // (どちらに繋がったかはログにも出ない)。立てるのは launch の責任。
        standalone: p!("standalone", bool, false),
        // 追従が失敗したときにゴールを投げ直す上限 (負で無制限)。BT の
        // `RecoveryNode number_of_retries` の置き換えだが、あちらと違って
        // **投げ直しの合間に場が実際に動く** (node::boot::GOAL_RETRY_SETTLE)。
        goal_retry_limit: p!("goal_retry_limit", i64, 3),
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
        // 掃きに渡す CPU の割合。掃きスレッドは「この割合だけロックを握って掃き、
        // 残りは手放して待つ」を繰り返す。追従ループは同じ Mutex を try_lock で取り、
        // 3 tick 続けて取れないとロボットを止めるので、上げるほど伝播は速く、
        // 追従が譲られる隙間は狭くなる (100 = 掃きっぱなし)。
        global_sweep_duty: p!("global_sweep_duty", i64, 25),

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

        // 機体の現在地からゴールまで方策が繋がった時点で走り出す
        // (core::PlanConfig::early_start)。**solve をやめるわけではなく**、残りは
        // 走りながら背景 (global_sweep) で解き切る。**既定は false**: 走り出して
        // から解き終わるまでの間は経路の外が未確定なので、そこへ機体が外れると
        // 解き直しが要る (そのときは場を捨てて最後まで解くので、待ちは合計で
        // 長くなる)。先読み (waypoint_prefetch) と違い追加のメモリは要らず、
        // 両者は併用できる — 先読みで用意した場は最後まで解いてあるので、
        // 受け取れた点では関係しない。
        early_start: p!("early_start", bool, false),

        // 価値関数 (全域スライス・ローカルウィンドウ・belief) の配信間隔。
        // 負で可視化そのものを立てない、0 は「solve 完了時のみ」。追従中の再配信は
        // 掃きスレッドが出すので、0 にすると追従中は出なくなる。
        value_publish_interval_ms: p!("value_publish_interval_ms", i64, 500),
        // カラースケールの上限 [ステップ数≒秒] (全域スライスとローカルウィンドウで共通)。
        cost_drawing_threshold: p!("cost_drawing_threshold", i64, 60),
    };
    // 早期走り出しは背景の解き切りとセット。走り出した時点の場はまだ解き終わって
    // いないので、掃きスレッドが無いと未確定のまま走り続けることになる (経路から
    // 外れたら解き直し、が常態になる)。黙って落とさず立ててから知らせる。
    if params.early_start && !params.global_sweep {
        eprintln!(
            "vi_planner: early_start needs the background solve to finish the field — \
             turning global_sweep on"
        );
        params.global_sweep = true;
    }
    Ok(params)
}

/// パラメータの自己整合検査 (vi_node / vi_global_planner と同じ fail-fast)。
///
/// 行動集合も θ 数も `ValueIterator` は実行時に受け取るので、値そのものは launch
/// から自由に決めてよい (かつてここで vi_core のコンパイル時定数と照合していた
/// のは、値を変えたいときの邪魔にしかならなかった)。落とすのは、後段のどこかで
/// 実際に割り算・添字になって壊れるものだけ。
pub fn validate(p: &Params) -> Result<U64Solver> {
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
    if !matches!(p.localizer.as_str(), "external" | "grid" | "adaptive" | "belief" | "viterbi") {
        return Err(anyhow!(
            "unknown localizer: {} (expected \"external\", \"grid\", \"adaptive\", \"belief\" \
             or \"viterbi\")",
            p.localizer
        ));
    }
    let solver = U64Solver::from_name(&p.solver)
        .ok_or_else(|| anyhow!("unknown solver: {} (see U64Solver::from_name)", p.solver))?;
    // 能動的再定位は判別点を出せる推定器 (adaptive) と密経路が要る。ここで
    // 落とさないと、ロストした瞬間に初めて Unsupported が返って原因が読めない。
    if p.active_reloc {
        if !matches!(p.localizer.as_str(), "adaptive") {
            return Err(anyhow!(
                "active_reloc needs a localizer that proposes disambiguating targets, and \
                 localizer={} does not (only \"adaptive\" implements reloc_targets). Set \
                 localizer:=adaptive, or leave active_reloc at false.",
                p.localizer
            ));
        }
        if solver.caps().out_of_core {
            return Err(anyhow!(
                "active_reloc needs the dense path, but solver={} is out-of-core (the compact \
                 sink solver is single-goal). Use a dense solver, or leave active_reloc at false.",
                p.solver
            ));
        }
    }
    Ok(solver)
}
