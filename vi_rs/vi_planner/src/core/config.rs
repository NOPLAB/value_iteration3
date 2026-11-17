//! Public configuration and result types for the shared planning core.
use super::*;

/// ゴールごとの ValueIterator 構築入力 (地図は起動時に一度だけ取り込む)。
/// vi_global_planner::core::BuildParams と同型 — クレート間依存を避けるための
/// 意図的な重複定義。
#[derive(Clone)]
pub struct BuildParams {
    pub grid: OccupancyGrid,
    pub actions: Vec<Action>,
    pub theta_cell_num: i32,
    pub map_penalty_radius: f64,
    pub map_penalty: f64,
    pub goal_margin_radius: f64,
    pub goal_margin_theta: i32,
    /// 追従のローカルウィンドウ半径 [m] (本家 `ValueIteratorLocal` は 1.0 固定)。
    /// `local_shape.range_m` と同じ値 — compact パッチの寸法計算が読む。
    pub local_xy_range: f64,
    /// 追従ウィンドウの形状・寸法・変形制限 ([`LocalShapeConfig`])。
    /// 既定は本家と同じ地図軸の等方矩形。
    pub local_shape: LocalShapeConfig,
    /// compact パッチの寸法スラック [セル] (`half = 2*win + reach + slack`)。
    pub patch_slack_cells: i32,
    /// 修復タイルの interior の 1 辺 [セル]。大きいほど 1 訪問あたりの halo の
    /// 割り増し (`(I+2h)²/I²`) が減って効率が上がり、代わりにタイル 1 枚の
    /// メモリとロックを握る時間が増える。16 は 0.25 m/cell (halo 2) で
    /// 20x20x60 = 1.9 MB・ハイドレート 288 KB あたり。
    pub repair_interior_cells: i32,
}

/// アウトオブコア (compact) 経路の確定出力の置き場。`None` = RAM (`RamSink`)、
/// `Some(dir)` = そのディレクトリ上の mmap ファイル。必要量は
/// `nx·ny·theta_cell_num × 12 B` なので、小メモリ機ではディスクに逃がす。
pub type SinkDir = Option<std::path::PathBuf>;

/// compact 出力を solve ごとに使い捨てのディレクトリ (`<dir>/gen<N>`) へ置くための
/// 世代カウンタ。`None` = 従来どおり `compact_sink_dir` 直下の固定ファイル名。
///
/// 先読み ([`Prefetcher`]) を使うときは**必須**で、しかも先読み側の核と**同じ
/// カウンタを共有すること**。場が 2 つ同時に生きるので、固定ファイル名のままだと
/// 後から解くほうが先の場のファイルを `truncate` で潰す。
pub type SinkGen = Option<Arc<AtomicU64>>;

/// 計画・追従の設定。前半は solve とキャッシュ、後半は用途別
/// (`plan_*` = 広域ロールアウト / `decide` = 狭域追従)。
#[derive(Clone)]
pub struct PlanConfig {
    /// スキャンのヒットセル周囲に塗るペナルティ帯の半幅 [m]。
    pub scan_penalty_radius: f64,
    pub scan_penalty: u64,
    pub solver: U64Solver,
    /// solve 総イテレーション上限 (発散ガード)。
    pub max_solve_iter: u32,
    /// cancel 観測間隔 (イテレーション数)。密経路のみ (compact は 1 回で走り切る)。
    pub solve_chunk: u32,

    // ── 広域 (compute_path_to_pose) ──
    pub max_rollout_steps: usize,
    /// start が方策なしセルのとき近傍探索する範囲 (セル数)。
    pub start_tolerance_cells: i32,
    /// 経路の最大姿勢間隔 (m)。0 以下で補間なし。
    pub path_spacing: f64,

    // ── 狭域 (follow_path) ──
    /// 現在セルに方策が無いとき近傍から行動を借りる範囲 (セル数)。
    pub action_tolerance_cells: i32,
    /// follow 1 tick の判断器 ([`FollowKind::Greedy`] = 本家 decision /
    /// [`FollowKind::Dwa`] / [`FollowKind::Mppi`] = 連続行動)。詳細は
    /// [`follow`] モジュールの doc。
    pub follow_controller: FollowKind,
    /// 自己位置の広がり σ [m] に掛けてマージン膨張量にする係数 (0 以下で無効、既定 0)。
    /// 上田ら 2023 (4·2·2) の「マージン `m` に σ を足す」を、状態空間を拡張せずに
    /// ウィンドウの `local_penalty` で作る ([`PlannerCore::inflate_by_sigma`])。
    /// σ は [`vi_lib::belief::spread_m`] が localizer の上位仮説から測る。
    pub sigma_margin_gain: f64,
    /// レーザーが貫通したセルの**地図由来**コスト (`free` / 膨張帯 `penalty`) も
    /// 反証するか (既定 false)。ビームが通り抜けた以上そこは空いている、という
    /// 反証。効くのは「地図には壁があるが実際には無い」— 自己位置がずれている /
    /// 地図が古い / ドアが開いた — のケースで、ロボットが幽霊壁に囲まれて止まる
    /// のを解く。
    ///
    /// スキャンが空いていると言った所しか開かないので「地図を無視して突っ切る」
    /// にはならない。逆に、自己位置がずれたまま開けた穴は本物の壁を消し得るので、
    /// 開いた分は `global_sweep` 経由で広域の場にも伝わる (寿命は local_penalty と
    /// 同じ = そのゴールの間だけ)。
    pub map_clear_from_scan: bool,
    /// DWA/MPPI: 制御周期 [s] (= 1/control_frequency)。候補評価の時間刻みの上限。
    pub dwa_tick_s: f64,
    /// DWA/MPPI: 前方シミュレーション時間 [s]。
    pub dwa_horizon_s: f64,
    /// DWA: 並進 / 角速度の候補数 (格子は n_v × n_w)。
    pub dwa_n_v: usize,
    pub dwa_n_w: usize,
    /// DWA: 軌道途中のセルを致死とみなす penalty しきい値 (PROB_BASE 単位、
    /// 既定 0 = 無効)。2.0 で margin 帯とレーザ注入セルが候補棄却になる
    /// (`DwaConfig::lethal_penalty` 参照)。
    pub dwa_lethal_penalty: f64,
    /// MPPI: サンプル本数 / softmax 温度 / 制御ノイズ標準偏差 (0 = 行動集合から
    /// 自動 — σ_v は速度幅の 1/4、σ_ω は上限の 1/4)。
    pub mppi_samples: usize,
    pub mppi_lambda: f64,
    pub mppi_sigma_v: f64,
    pub mppi_sigma_w_deg: f64,

    // ── アウトオブコア (compact) 経路 ──
    /// 確定出力の置き場。`solver` が `frontier2d_sparse_compact` のときだけ参照する。
    pub compact_sink_dir: SinkDir,
    /// solve ごとに使い捨てのディレクトリを切るか (先読みを使うときは必須)。
    pub compact_sink_gen: SinkGen,
    /// compact 経路のワーカースレッド数 (0 = `default_threads()`)。
    pub vi_threads: usize,

    // ── ウェイポイントの先読み ──
    /// 進行中の先読みを待つときの観測間隔 [ms] (cancel を見る刻みでもある)。
    pub prefetch_poll_ms: u64,

    // ── 狭域 → 広域の全域伝播 ──
    /// 全域伝播 ([`PlannerCore::sweep_global`]) を回すか。compact 経路では
    /// これが false のとき修復タイル ([`compact::Repair`], 数 MB) を確保しない。
    pub global_sweep: bool,

    // ── 走り出しの短縮 ──
    /// **機体の現在地からゴールまで方策が繋がった時点で走り出せるようにする**か
    /// (`false` = 収束するまで機体を止めておく)。**場を解くのをやめるわけではない**
    /// — 繋がった時点で場をキャッシュに載せて追従へ渡し、残りは走りながら背景で
    /// 解き切る (密は全域掃き、compact はタイル修復。どちらも同じ Bellman 更新)。
    /// 判定と代償は [`PlannerCore::prepare_goal_with_progress`] の「早期走り出し」。
    /// 起点が要るので、`from: None` で呼ばれた solve はこれが true でも
    /// 収束するまで返らない。
    ///
    /// **背景の解き切りは `global_sweep` が担う**ので、これを立てるなら
    /// `global_sweep: true` も要る (ノード側は early_start で自動的に立てる)。
    pub early_start: bool,
}

impl PlanConfig {
    /// アウトオブコア (`states` を作らない) 経路を使うか (ソルバの能力宣言
    /// [`vi_lib::solvers::SolverCaps::out_of_core`] を読む — ソルバ名の
    /// ハードコードはしない)。
    pub fn use_compact(&self) -> bool {
        self.solver.caps().out_of_core
    }
}

/// solve 単体の統計 (ログ用)。
#[derive(Clone, Copy, Debug)]
pub struct SolveStats {
    /// この呼び出しでゴールの価値関数が新しく載ったか (false = キャッシュヒット)。
    /// 先読みから受け取ったときも true — 場が入れ替わったのは同じなので。
    pub solved_now: bool,
    /// 実行した solve イテレーション数 (キャッシュヒットと先読み採用では 0 —
    /// 解いたのは先読みワーカーなので、この呼び出しの仕事ではない)。
    pub iters: u32,
    /// 先読み ([`Prefetcher`]) が用意しておいた場を受け取ったか。この呼び出しは
    /// solve していない = 走り出しまでの待ちが無かった、という意味。
    pub adopted: bool,
    /// いま載っている場が**まだ解き終わっていない**か (`early_start`)。ゴールまでの
    /// 経路は繋がっているが、そこから外れた領域は未確定 (compact なら sink が
    /// `MAX_COST` のまま、密なら値が上振れしたまま)。キャッシュヒットのときも
    /// 載っている場の状態を映す。
    ///
    /// **永続的な印ではない**: 背景の解き切り ([`PlannerCore::sweep_global`]) が
    /// 不動点に達した時点で降りる。降りたあとの場は収束まで解いたものと同じ。
    pub partial: bool,
}

/// 1 回の plan の統計 (ログ/Feedback 用)。
#[derive(Clone, Copy, Debug)]
pub struct PlanStats {
    pub solved_now: bool,
    pub iters: u32,
    pub adopted: bool,
    pub partial: bool,
    pub poses: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlanError {
    /// cancel フラグにより中断された (プリエンプト / client cancel)。
    Cancelled,
    /// `max_solve_iter` 内に収束しなかった。
    NotConverged,
    /// 価値関数は収束したがロールアウトが失敗した (`plan_*` のみ)。
    Rollout(RolloutStatus),
    /// compact 経路の出力先 (mmap ファイル) を用意できなかった。
    Sink(String),
    /// compact 経路の追従用パッチを構成できなかった (幾何が破綻している)。
    Patch(String),
    /// この構成では提供できない機能 (compact での能動的再定位など)。
    Unsupported(&'static str),
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlanError::Cancelled => write!(f, "cancelled (preempted)"),
            PlanError::NotConverged => write!(f, "value iteration did not converge"),
            PlanError::Rollout(s) => write!(f, "policy rollout failed: {s:?}"),
            PlanError::Sink(e) => write!(f, "compact output sink unavailable: {e}"),
            PlanError::Patch(e) => write!(f, "follow patch cannot be built: {e}"),
            PlanError::Unsupported(e) => write!(f, "unsupported in this configuration: {e}"),
        }
    }
}

/// 1 制御周期の判断。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Decision {
    /// 現在姿勢がゴール圏 (`final_state`)。
    Goal,
    /// 実行する行動。`fw` は前進量 [m]、`rot_deg` は回転量 [deg]
    /// (本家 `ViNode::decision` はこれをそのまま速度指令として配信する)。
    /// `id` は離散方策の行動 id — 連続コントローラ ([`DwaController`]) の指令は
    /// 離散行動に対応しないので `None`。
    Action { id: Option<usize>, fw: f64, rot_deg: f64 },
    /// 方策なし (地図外 / 障害物セル / 未到達セル)。
    NoAction,
}
