//! rclrs 非依存の統合プランナ中核。**1 本の価値関数**を広域 (経路計画) と
//! 狭域 (経路追従) の両方で共有する:
//!
//!   - solve はゴールごとに 1 回だけ (`prepare_goal_with_progress`)
//!   - `plan_*` は解決済み価値関数の貪欲ロールアウト
//!   - 追従は同じ価値関数の ±1m ウィンドウをスキャンで補正しながら回す
//!
//! 狭域 → 広域の伝播 (全域掃き) は [`PlannerCore::sweep_global`]、収束前に
//! 走り出す早期打ち切りは [`PlannerCore::prepare_goal_with_progress`] の doc に
//! それぞれ詳細がある (経緯と実測値はリポジトリ CLAUDE.md)。
//!
//! # ファイルの分かれ方
//!
//! - `core/mod.rs` (ここ) — 公開型と [`PlannerCore`] の本体。密 (dense) 経路の
//!   solve・ロールアウト・窓の精密化・全域掃きはすべてここにある。
//! - [`compact`] — アウトオブコア経路だけの機構 (sink・追従パッチ・penalty 表・
//!   タイル修復)。`PlannerCore` の compact 側メソッドもそちらに置いてある。
//! - [`prefetch`] — 次のウェイポイントを走行中に解いておく先読み ([`Prefetcher`])。
//! - `core/tests.rs` — 両経路をまたぐテスト。
//!
//! # 2 つの経路: 密 (dense) とアウトオブコア (compact)
//!
//! `PlanConfig::solver` が out-of-core かどうかで価値関数の持ち方が変わる。
//! 広域と狭域が 1 本の場を共有するという上の性質はどちらでも同じ。
//!
//! - **密**: `ValueIteratorLocal` を全域ぶん確保する。狭域はその `states` を
//!   その場で書き換える。小〜中規模の地図はこちら。
//! - **compact**: `solve_compact_mapped` が `states` を作らずに解き、確定出力
//!   (12 B/state) だけを `CompactSink` (既定は mmap ファイル) に置く。追従は
//!   `states` を必要とするので、**ロボット近傍だけを compact の場から起こした
//!   小さな密パッチ** ([`compact::Patch`]) の上で回す。
//!
//! `BuildParams` の重複定義は意図的 (クレート間依存を避ける)。このモジュールは
//! vi_lib のみに依存し、ホストで `cargo test --lib` できる (分離クレート
//! 方式; リポジトリ CLAUDE.md 参照)。

mod compact;
mod follow;
mod prefetch;
#[cfg(test)]
mod tests;

pub use follow::{DwaController, FollowController, FollowKind, GreedyController, MppiController};
// 自己位置推定は vi_lib::belief の全地図 Belief (アルゴリズムは vi_lib、配線は
// ノードの分担)。窓つきの旧 localize::* はもう使わない。
pub use vi_lib::belief::{Belief, BeliefConfig};
pub use vi_lib::value_iterator::BeliefModel;

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, TryLockError};
use std::time::{Duration, Instant};

use vi_lib::bridge::{value_slice_to_occupancy, yaw_to_goal_theta_deg, PoseView};
use vi_lib::msg::{LaserScan, OccupancyGrid};
use vi_lib::planner::{
    densify, pose_to_cell, qmdp_decide, rollout_path_on, PathPose, PolicyView, QmdpDecision,
    Rollout, RolloutStatus,
};
use vi_lib::solvers::{solve_observed, SolveFlow, SolveObserver, SolveProbe, U64Solver};
use vi_lib::value_iterator::ValueIterator;
use vi_lib::{Action, ValueIteratorLocal};

use compact::{new_patch, new_repair, CompactField, Patch, PenaltyOverlay, Repair};
pub use prefetch::Prefetcher;

/// 他スレッドの panic で毒された Mutex もそのまま使い続けて取る。この核の場も
/// 先読みの state も、フィールドが独立していて panic を跨ぐ不変条件を持たない
/// (価値関数は `ValueIterator` の内部状態として一貫している) ため。
pub fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|p| p.into_inner())
}

/// [`lock`] の try 版。取れなければ `None` (WouldBlock)。毒は同じく無視する。
pub fn try_lock<T>(m: &Mutex<T>) -> Option<MutexGuard<'_, T>> {
    match m.try_lock() {
        Ok(g) => Some(g),
        Err(TryLockError::WouldBlock) => None,
        Err(TryLockError::Poisoned(p)) => Some(p.into_inner()),
    }
}

/// 観測一致度 → スキャン注入の減衰段数 ([`PlannerCore::observe_scan_gated`] の
/// `shift`)。`gate` 以上は 0 (満額 2048)、下回ると注入 ∝ quality/gate の 2 冪
/// 量子化 (PenaltyOverlay が指数しか持てないため冪に丸める)。上限 11
/// (2048 → 1)。`gate <= 0` でゲート無効。External localizer は quality 1.0 を
/// 返すので常に 0 — 挙動は従来と変わらない。
pub fn quality_shift(quality: f64, gate: f64) -> u32 {
    if gate <= 0.0 || quality >= gate {
        return 0;
    }
    if quality <= 0.0 {
        return 11;
    }
    ((gate / quality).log2().ceil() as u32).min(11)
}

/// ゴールごとの ValueIterator 構築入力 (地図は起動時に一度だけ取り込む)。
/// vi_global_planner::core::BuildParams と同型 — クレート間依存を避けるための
/// 意図的な重複定義。
#[derive(Clone)]
pub struct BuildParams {
    pub grid: OccupancyGrid,
    pub actions: Vec<Action>,
    pub theta_cell_num: i32,
    pub safety_radius: f64,
    pub safety_radius_penalty: f64,
    pub goal_margin_radius: f64,
    pub goal_margin_theta: i32,
    /// 追従のローカルウィンドウ半径 [m] (本家 `ValueIteratorLocal` は 1.0 固定)。
    pub local_xy_range: f64,
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
    pub solver: U64Solver,
    /// solve 総イテレーション上限 (発散ガード)。
    pub max_solve_iter: u32,
    /// cancel 観測間隔 (イテレーション数)。密経路のみ (compact は 1 回で走り切る)。
    pub solve_chunk: u32,
    /// キャッシュ再利用とみなすゴール位置差 (m)。
    pub goal_tolerance_xy: f64,
    /// キャッシュ再利用とみなすゴール方位差 (度)。
    pub goal_tolerance_deg: f64,

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
    /// footprint クリアの半径 [m] (0 以下で無効)。スキャン注入のたびに機体位置の
    /// 周囲の local_penalty を消す — ロボットが現にいる場所は free という反証
    /// 不能な証拠で、ゴースト壁が機体の真上で閉じて完全停止するのを防ぐ。
    pub footprint_clear_m: f64,
    /// DWA/MPPI: 制御周期 [s] (= 1/control_frequency)。候補評価の時間刻みの上限。
    pub dwa_tick_s: f64,
    /// DWA/MPPI: 前方シミュレーション時間 [s]。
    pub dwa_horizon_s: f64,
    /// DWA: 並進 / 角速度の候補数 (格子は n_v × n_w)。
    pub dwa_n_v: usize,
    pub dwa_n_w: usize,
    /// DWA: 軌道途中のセルを致死とみなす penalty しきい値 (PROB_BASE 単位、
    /// 0 = 無効)。既定 2.0 で margin 帯とレーザ注入セルが候補棄却になる
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
    /// 全域掃き ([`PlannerCore::sweep_global`]) を回すか。compact 経路では
    /// これが false のとき修復タイル ([`compact::Repair`], 数 MB) を確保しない。
    pub global_sweep: bool,

    // ── 走り出しの短縮 ──
    /// **機体の現在地からゴールまで方策が繋がった時点で solve を打ち切る**か
    /// (`false` = 従来どおり収束まで解く)。判定と代償は
    /// [`PlannerCore::prepare_goal_with_progress`] の「早期打ち切り」を参照。
    /// 打ち切りには起点が要るので、`from: None` で呼ばれた solve は
    /// これが true でも最後まで解く。
    pub early_start: bool,

    // ── belief 次元 (x, y, θ, b) ──
    /// 価値関数へ足す belief 不確かさ次元。既定 (`nb_levels: 1`) は従来の 3D VI と
    /// 構成的に同一。`nb_levels > 1` は密経路 + `caps().belief` のソルバ限定
    /// ([`PlannerCore::prepare_goal_with_progress`] が `Unsupported` で弾く)。
    pub belief: BeliefModel,
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
    /// いま載っている場が**収束前に打ち切ったもの**か (`early_start`)。ゴールまでの
    /// 経路は繋がっているが、そこから外れた領域は未確定 (compact なら sink が
    /// `MAX_COST` のまま、密なら値が上振れしたまま)。キャッシュヒットのときも
    /// 載っている場の状態を映す。
    pub truncated: bool,
}

/// 1 回の plan の統計 (ログ/Feedback 用)。
#[derive(Clone, Copy, Debug)]
pub struct PlanStats {
    pub solved_now: bool,
    pub iters: u32,
    pub adopted: bool,
    pub truncated: bool,
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
    /// この構成では提供できない機能 (compact / Group B ソルバでの belief 次元など)。
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

/// solve 済み価値関数の実体。
enum Field {
    /// 密経路: 全域の `ValueIteratorLocal`。広域も狭域もこれを直接読む。
    Dense(Box<ValueIteratorLocal>),
    /// compact 経路: 確定出力 (sink) + 幾何。狭域は [`compact::Patch`] を経由する。
    Compact(Box<CompactField>),
}

struct CachedGoal {
    goal_x: f64,
    goal_y: f64,
    goal_t_deg: i32,
    field: Field,
    /// 収束前に打ち切った場か ([`PlanConfig::early_start`])。**捨てて解き直す判断に
    /// 要る**: 打ち切った場でロールアウトが通らなかったとき、この印が無いと
    /// 同じキャッシュを何度でも返して同じ失敗を繰り返す (BT の 1Hz リプランは
    /// キャッシュヒットなので solve に入らない = 自然には直らない)。
    truncated: bool,
}

/// 全域掃き ([`PlannerCore::sweep_global`]) の再開位置。10 Hz の追従ループと同じ
/// `Mutex<PlannerCore>` を共有するので、掃きはチャンクに切ってロックを手放しながら
/// 進める。その「続き」を呼び出し側が持つための値。
#[derive(Clone, Copy, Debug, Default)]
pub struct SweepCursor {
    /// `sweep_orders` のどれを掃いているか。1 掃き終わるごとに次へ回す。
    order: usize,
    /// その掃き順の中の位置。
    pos: usize,
    /// 今の 1 掃きでこれまでに積んだ Δ 合計 (1 掃き丸ごとで 0 なら収束)。
    delta: u64,
}

pub struct PlannerCore {
    build: BuildParams,
    cfg: PlanConfig,
    cached: Option<CachedGoal>,
    /// compact 経路の追従用パッチ (ゴール非依存なのでキャッシュの外に置く)。
    /// 密経路では使わない。
    patch: Option<Patch>,
    /// compact 経路で観測した `local_penalty` の全域表 (密経路では None)。
    /// 密経路の `states.local_penalty` に対応するので、寿命も同じ
    /// (ゴールを解き直すと `states` ごと作り直される = ここでは `clear`)。
    penalty: Option<PenaltyOverlay>,
    /// compact 経路の全域伝播の作業場 (密経路と `global_sweep: false` では None)。
    repair: Option<Repair>,
    /// 狭域が共有場を動かしたか。`refine_for` が Δ>0 を出すと立ち、全域掃きが
    /// Δ=0 で 1 周し終えると落ちる。全域掃きを回すかの唯一の判断材料
    /// (「狭域が通れないと言っている」を別途検出する必要はない — 通れないなら
    /// 必ず値が動くので、これがその信号そのものになる)。
    dirty: bool,
    /// 次のウェイポイントの先読み ([`Prefetcher`])。`None` = 先読みなし
    /// (`waypoint_prefetch: false`) で、そのときこの核の挙動は従来と同じ。
    prefetch: Option<Prefetcher>,
    /// 追従の道具 (パッチ・penalty 表・修復タイル) を持たず solve だけをする核か。
    /// 先読みワーカーが抱える予備の核がこれ ([`PlannerCore::new_solve_only`])。
    solve_only: bool,
    /// follow 1 tick の判断器 ([`PlanConfig::follow_controller`] から組む)。
    /// solve・ロールアウトには関与しない — `decide` だけがこれを通る。
    follow: Box<dyn FollowController>,
}

/// 円環上の角度差 (度、0..=180)。
fn circ_deg_diff(a: i32, b: i32) -> i32 {
    let d = (a - b).rem_euclid(360);
    d.min(360 - d)
}

/// 2 つのゴール `(x, y, θ[deg])` を「同じ」とみなせるか。キャッシュの再利用判定と
/// 先読み ([`Prefetcher`]) の照合が同じ規則を使うためにここに 1 つだけ置く
/// (別々にすると、先読みが用意した場を採用できないゴールが静かに生まれる)。
fn goal_matches(a: (f64, f64, i32), b: (f64, f64, i32), tol_xy: f64, tol_deg: f64) -> bool {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt() <= tol_xy
        && circ_deg_diff(a.2, b.2) as f64 <= tol_deg
}

/// `from` からゴール圏まで、いまの場の方策だけで辿り着けるか。早期打ち切り
/// ([`PlanConfig::early_start`]) の判定はこれ 1 つで、密経路と compact 経路が同じ
/// 規則を使う (どちらも [`rollout_path_on`] = `plan` が実際に返す経路の作り方そのもの
/// なので、「打ち切ったのに `plan` が失敗する」がこの判定を通った場では起きない)。
fn reaches_goal(p: &dyn PolicyView, cfg: &PlanConfig, from: PoseView) -> bool {
    rollout_path_on(
        p,
        from.x,
        from.y,
        from.yaw_rad,
        cfg.max_rollout_steps,
        cfg.start_tolerance_cells,
    )
    .reached_goal()
}

/// solve の進行制御 ([`SolveObserver`] 実装)。密・compact の両経路がこれ 1 つを使う —
/// 旧実装の「密: 手書きチャンクループ / compact: stop クロージャ」の置き換え。
/// 境界 (ソルバの自然な粒度: 密系はラウンド/スイープ × `solve_chunk`、compact は
/// バンド finalize) ごとに呼ばれ、
///   1. cancel (プリエンプト) を観測し、
///   2. 途中経過を `on_progress` へ流し (probe の場は要求時にだけ具現化される)、
///   3. `from` があれば早期打ち切り ([`PlanConfig::early_start`]) を判定する。
/// 判定は [`reaches_goal`] そのもの = `plan` が実際に返す経路の作り方なので、密でも
/// compact でも「打ち切ったのに `plan` が失敗する」形の外し方をしない。
struct SolveDirector<'a> {
    cfg: &'a PlanConfig,
    cancel: &'a AtomicBool,
    /// 早期打ち切りの起点 (None = 打ち切らない)。
    from: Option<PoseView>,
    on_progress: &'a mut dyn FnMut(&dyn PolicyView),
}

impl SolveObserver for SolveDirector<'_> {
    fn interval(&self) -> u32 {
        self.cfg.solve_chunk.max(1)
    }
    fn boundary(&mut self, probe: &mut dyn SolveProbe) -> SolveFlow {
        if self.cancel.load(Ordering::Relaxed) {
            return SolveFlow::Cancel;
        }
        let p = probe.policy();
        (self.on_progress)(p);
        if let Some(from) = self.from {
            if reaches_goal(p, self.cfg, from) {
                return SolveFlow::Stop;
            }
        }
        SolveFlow::Continue
    }
}

/// `(ix, iy, it)` を `ib` 層で見て方策があれば Decision::Action を返す (読み取り専用)。
/// `nb_levels == 1` では `ib` は常に 0 = 従来と同一。
fn action_at(vi: &ValueIterator, ix: i32, iy: i32, it: i32, ib: i32) -> Option<Decision> {
    let ai = PolicyView::action_index_b(vi, ix, iy, it, ib)?;
    let a = &vi.actions[ai];
    Some(Decision::Action { id: Some(a.id as usize), fw: a.delta_fw, rot_deg: a.delta_rot })
}

/// 範囲内で final_state か (境界チェック込み)。`ib` 層で見たゴール圏 — b が
/// `ib_goal` より不確かな層ではゴールが終端にならない (coastal navigation)。
fn is_final(vi: &ValueIterator, ix: i32, iy: i32, it: i32, ib: i32) -> bool {
    PolicyView::is_final_b(vi, ix, iy, it, ib)
}

/// solve 済み場の θ=0 全域スライスを可視化用 OccupancyGrid に描画する (0..=100、
/// 未到達 -1)。ビューは [`PolicyView`] — 密 (`&ValueIterator`) も compact (sink ビュー)
/// も同じ実装で描く。solve 中の途中経過 (`*_with_progress` のコールバック) からも呼べる。
pub fn value_grid_on(p: &dyn PolicyView, threshold_steps: u64) -> OccupancyGrid {
    let (nx, ny, _) = p.cell_num();
    let (w, h) = (nx as usize, ny as usize);
    let mut slice = vec![0u64; w * h];
    for iy in 0..ny {
        for ix in 0..nx {
            slice[iy as usize * w + ix as usize] = p.value_at(ix, iy, 0);
        }
    }
    let (ox, oy) = p.map_origin();
    OccupancyGrid {
        width: nx,
        height: ny,
        resolution: p.xy_resolution(),
        origin_x: ox,
        origin_y: oy,
        origin_quat: p.map_origin_quat(),
        data: value_slice_to_occupancy(&slice, w, h, threshold_steps),
    }
}

impl PlannerCore {
    pub fn new(build: BuildParams, cfg: PlanConfig) -> Self {
        // penalty 表は compact だけ (密は states がそのまま持つ)。1 B/セルなので
        // 津田沼の 0.25 m/cell (1177x800) でも 0.9 MB。
        let penalty = cfg
            .use_compact()
            .then(|| PenaltyOverlay::new(build.grid.width, build.grid.height));
        // 修復タイルは遷移表の再計算が要るので、パッチと一緒に最初の solve で作る
        // (`prepare_goal_with_progress`)。掃きスレッドの中で作ると、数秒かかる
        // 再計算をロックの中でやることになる。
        let follow = follow::make_controller(&cfg, &build.actions);
        Self {
            build,
            cfg,
            follow,
            cached: None,
            patch: None,
            penalty,
            repair: None,
            dirty: false,
            prefetch: None,
            solve_only: false,
        }
    }

    /// 先読み ([`Prefetcher`]) を付ける。付けると `prepare_goal_*` が 2 つのことを
    /// するようになる: ゴールの場が先読みで用意できていれば solve を飛ばして
    /// 受け取り、ゴールが確定するたびに**並びの次の点**を先読みへ注文する。
    /// 付けなければ (既定) この核の挙動は従来のまま。
    pub fn with_prefetch(mut self, prefetch: Prefetcher) -> Self {
        self.prefetch = Some(prefetch);
        self
    }

    /// 先読みワーカーが抱える予備の核。追従はしないので、パッチも penalty 表も
    /// 修復タイルも作らない (作ると遷移表の再計算ぶんだけ最初の先読みが遅くなる)。
    fn new_solve_only(build: BuildParams, cfg: PlanConfig) -> Self {
        Self { solve_only: true, penalty: None, ..Self::new(build, cfg) }
    }

    /// キャッシュ中のゴールが `goal` と同一 (許容差内) か。追従スレッドが
    /// 「自分のゴールの価値関数がまだ載っているか」を毎 tick 確認するのに使う
    /// (広域側の計画要求が別ゴールで solve し直すとキャッシュが差し替わるため)。
    pub fn is_cached_goal(&self, goal: PoseView) -> bool {
        self.cache_matches(&goal, yaw_to_goal_theta_deg(goal.yaw_rad))
    }

    fn cache_matches(&self, goal: &PoseView, goal_t_deg: i32) -> bool {
        let Some(c) = self.cached.as_ref() else { return false };
        goal_matches(
            (c.goal_x, c.goal_y, c.goal_t_deg),
            (goal.x, goal.y, goal_t_deg),
            self.cfg.goal_tolerance_xy,
            self.cfg.goal_tolerance_deg,
        )
    }

    // ──────────────────────────────────────────────────────────────────────
    // 狭域が読み書きする密な局所 VI
    //
    // 密経路は全域の ValueIteratorLocal そのもの、compact 経路はハイドレート済み
    // パッチ。追従側 (observe_scan / refine_* / decide / window_value_grid) は
    // この 2 つを区別しない。
    // ──────────────────────────────────────────────────────────────────────

    fn local(&self) -> Option<&ValueIteratorLocal> {
        match &self.cached.as_ref()?.field {
            Field::Dense(vi) => Some(vi),
            Field::Compact(_) => {
                let p = self.patch.as_ref()?;
                p.at.map(|_| &p.vi)
            }
        }
    }

    fn local_mut(&mut self) -> Option<&mut ValueIteratorLocal> {
        let compact = matches!(self.cached.as_ref()?.field, Field::Compact(_));
        if compact {
            let p = self.patch.as_mut()?;
            p.at?;
            return Some(&mut p.vi);
        }
        match &mut self.cached.as_mut()?.field {
            Field::Dense(vi) => Some(vi),
            Field::Compact(_) => None,
        }
    }

    // ──────────────────────────────────────────────────────────────────────
    // solve (広域・狭域で共有する唯一の価値関数)
    //
    // 密は `solve_dense` (すぐ下)、compact は `compact::` 側の `solve_compact`。
    // ──────────────────────────────────────────────────────────────────────

    /// ゴールへ向けた価値関数を用意する。同一ゴール (許容差内) なら solve を
    /// スキップする (このとき直近スキャン由来の `local_penalty` も温存される)。
    /// `cancel` は solve 中に `solve_chunk` ごとに観測される。
    pub fn prepare_goal(
        &mut self,
        goal: PoseView,
        cancel: &AtomicBool,
    ) -> Result<SolveStats, PlanError> {
        self.prepare_goal_with_progress(goal, None, cancel, &mut |_| {})
    }

    /// `prepare_goal` と同じだが、solve の境界 (密: `solve_chunk` 反復ごと /
    /// compact: バンド finalize ごと) に `on_chunk` を呼ぶ (途中経過の value_function
    /// 可視化用 — [`value_grid_on`] がそのまま受け取れる `&dyn PolicyView` を渡す)。
    /// キャッシュヒット時は呼ばれない。
    ///
    /// 先読み ([`Prefetcher`]) が付いていれば、solve の前にそちらを見る。用意が
    /// できていれば受け取って solve を飛ばし (`stats.adopted`)、まだ解いている
    /// 最中なら終わるまで待つ。どちらにせよ最後に**並びの次の点**を注文するので、
    /// 巡回中は「いまのゴールへ走っている間に次のゴールが解けている」状態になる。
    ///
    /// # 早期打ち切り (`early_start`)
    ///
    /// `from` は**機体のいまの姿勢**。[`PlanConfig::early_start`] が true でこれが
    /// 与えられていると、solve を収束まで回さず「`from` からゴールまで方策が
    /// 繋がった」時点で止める。判定はロールアウトそのもの — 途中の場の上で
    /// [`rollout_path_on`] を試し、ゴール圏に着いたら打ち切る。「値がロボットの
    /// 近くまで来たか」を距離や値で近似せず、**使う経路が引けたかを直接見る**
    /// (近似だと、経路上の 1 セルだけ未確定で `plan` が失敗する形の外し方をする)。
    ///
    /// 止めた場でよい理由は経路の側にある。compact 経路の finalize は値の昇順に
    /// 進むので、sink に載っている列は**最後まで解いたときと同じ値** (未確定の列が
    /// `MAX_COST` のまま残っているだけ)。密経路は値が上から単調に下がるので、途中の
    /// 場は常に Bellman の上界 = 貪欲降下は必ず値を下げながらゴールに着く (循環しない)
    /// — ただし**最短とは限らない**。密で打ち切った場は `global_sweep: true` なら
    /// 走りながら全域掃きが最後まで詰める。
    ///
    /// 代償は「ゴールまでの経路の外は未確定」であること。機体が経路から外れると
    /// 未確定領域に入り得るので、`plan` のロールアウトが通らなかったときは
    /// [`PlannerCore::discard_truncated`] で捨てて解き直す道を必ず残しておくこと
    /// (キャッシュヒットは solve に入らないので、放っておくと同じ失敗が続く)。
    pub fn prepare_goal_with_progress(
        &mut self,
        goal: PoseView,
        from: Option<PoseView>,
        cancel: &AtomicBool,
        on_chunk: &mut dyn FnMut(&dyn PolicyView),
    ) -> Result<SolveStats, PlanError> {
        let goal_t_deg = yaw_to_goal_theta_deg(goal.yaw_rad);
        let mut stats =
            SolveStats { solved_now: false, iters: 0, adopted: false, truncated: false };

        // belief 次元の適用範囲 (ノード側の起動時検証と同じ条件のベルト。素通しすると
        // vi_lib の solve_observed が assert で落ちる)。
        if self.cfg.belief.nb_levels > 1 {
            if self.cfg.use_compact() {
                return Err(PlanError::Unsupported(
                    "belief_levels > 1 needs the dense path (the compact sink is 3D)",
                ));
            }
            if !self.cfg.solver.caps().belief {
                return Err(PlanError::Unsupported(
                    "belief_levels > 1 needs a solver with caps().belief (solver: frontier2d)",
                ));
            }
        }

        if self.cache_matches(&goal, goal_t_deg) {
            // ここも注文の入口。BT は追従中も 1Hz で計画を投げ直すので、最初の
            // 1 回で注文できていなくても (並びがまだ届いていない等) 次の tick で
            // 拾える。同じ点の注文は 2 回目以降 no-op。
            self.request_next(goal);
            stats.truncated = self.cached.as_ref().is_some_and(|c| c.truncated);
            return Ok(stats);
        }

        self.cached = None; // 旧キャッシュ (数 GB になり得る) を先に解放
        if let Some(p) = self.patch.as_mut() {
            p.at = None; // 別ゴールの値が残ったパッチで走らせない
        }
        // 密経路は `states` を作り直すので local_penalty がここで消える。compact の
        // penalty 表も同じ寿命にそろえる (「消したければゴールを取り直す」の実体)。
        if let Some(ov) = self.penalty.as_mut() {
            ov.clear();
        }
        // 解き直した直後は収束済み。前のゴールで積んだ「伝播させる仕事がある」印は
        // 持ち越さない (持ち越すと最初の 1 掃きが必ず無駄に回る)。待ち行列に残った
        // タイルも同じで、前のゴールの場を修復しても意味がない。
        self.dirty = false;
        if let Some(r) = self.repair.as_mut() {
            r.clear();
        }

        // パッチも修復タイルも幾何だけで決まるので初回だけ作る (遷移表の再計算は
        // 64^3 サブセルサンプリング x 行動 x θ で重い)。**solve の外に出してある**
        // のは、先読みから受け取る経路が solve を通らないため — 巡回の 1 点目から
        // 受け取ると、ここが solve の中にあると追従の道具が無いまま走り出す。
        if self.cfg.use_compact() && !self.solve_only && self.patch.is_none() {
            let p = new_patch(&self.build)?;
            if self.cfg.global_sweep && self.repair.is_none() {
                self.repair = Some(new_repair(&self.build, p.reach)?);
            }
            self.patch = Some(p);
        }

        let adopted = match self.prefetch.as_ref() {
            Some(pf) => pf.adopt(goal, goal_t_deg, cancel),
            None => None,
        };
        // 打ち切りの起点。`early_start` でも起点が無ければ打ち切らない (先読みの
        // ワーカーがここを通る道でもある — あちらは `prepare_goal` 経由で
        // `from: None`。次の点を解いている間に機体は動くので、いまの姿勢を起点に
        // 打ち切った場は着いた頃には使えない)。
        let from = self.cfg.early_start.then_some(from).flatten();

        let cached = match adopted {
            Some(c) => {
                stats.adopted = true;
                c
            }
            None => {
                let field = if self.cfg.use_compact() {
                    self.solve_compact(&goal, goal_t_deg, from, &mut stats, cancel, on_chunk)?
                } else {
                    self.solve_dense(&goal, goal_t_deg, from, &mut stats, cancel, on_chunk)?
                };
                CachedGoal {
                    goal_x: goal.x,
                    goal_y: goal.y,
                    goal_t_deg,
                    field,
                    truncated: stats.truncated,
                }
            }
        };

        stats.solved_now = true;
        // 打ち切った密の場は、走りながら全域掃きが最後まで詰める。掃きを回すかの
        // 判断材料は `dirty` 1 つなので、ここで立てるだけでよい。
        //
        // compact で立てないのは、あちらの伝播が「変化した範囲」から広げる形だから。
        // 立てても待ち行列が空で 1 回空振りするだけで、実際の起点は追従が始まって
        // 最初の `commit_window` (窓が未確定域をまたいで値が動く) になる。
        if stats.truncated && !self.cfg.use_compact() {
            self.dirty = true;
        }
        self.cached = Some(cached);
        self.request_next(goal);
        Ok(stats)
    }

    /// 打ち切った (収束前に止めた) 場を捨てる。捨てたら true、収束済みの場や
    /// キャッシュ無しなら false。
    ///
    /// **これが `early_start` の唯一の逃げ道**。打ち切った場でロールアウトが
    /// 通らない / 方策が引けないとき、キャッシュが載ったままだと BT が何度
    /// 投げ直しても `prepare_goal_*` はキャッシュヒットで返って同じ失敗を繰り返す。
    /// 捨てておけば次の要求が最後まで解き直す (`from` を渡さなければ打ち切らない)。
    pub fn discard_truncated(&mut self) -> bool {
        if !self.cached.as_ref().is_some_and(|c| c.truncated) {
            return false;
        }
        self.cached = None;
        if let Some(p) = self.patch.as_mut() {
            p.at = None; // 打ち切った場から起こしたパッチで走らせない
        }
        true
    }

    /// 先読みへ「いまのゴールはこれ」と伝える (並びの次の点の注文がここで出る)。
    /// 先読みが付いていなければ何もしない。
    fn request_next(&self, goal: PoseView) {
        if let Some(pf) = self.prefetch.as_ref() {
            pf.note_goal(goal);
        }
    }

    /// 密経路: `ValueIterator::states` を確保し、[`solve_observed`] + [`SolveDirector`]
    /// で解く。cancel の観測・途中経過・早期打ち切り (`from` があるとき) はすべて
    /// solve 内部の境界 (`solve_chunk` 反復ごと) で行われる — 旧実装のチャンク再入
    /// (毎チャンクの再ビルド + 全セル write_back) はもう無い。境界で場を読めることは
    /// ソルバ側の契約 (`SolveProbe::policy`) で、全 `U64Solver` が conformance テストで
    /// これをゲートされている。
    fn solve_dense(
        &self,
        goal: &PoseView,
        goal_t_deg: i32,
        from: Option<PoseView>,
        stats: &mut SolveStats,
        cancel: &AtomicBool,
        on_chunk: &mut dyn FnMut(&dyn PolicyView),
    ) -> Result<Field, PlanError> {
        if cancel.load(Ordering::Relaxed) {
            return Err(PlanError::Cancelled);
        }
        let mut vi = ValueIteratorLocal::new(self.build.actions.clone(), 1);
        // **set_map より前**に入れること — states の層数と info はそこで確定する。
        vi.base.belief = self.cfg.belief.clone();
        vi.set_map_with_occupancy_grid(
            &self.build.grid,
            self.build.theta_cell_num,
            self.build.safety_radius,
            self.build.safety_radius_penalty,
            self.build.goal_margin_radius,
            self.build.goal_margin_theta,
        );
        vi.set_local_xy_range(self.build.local_xy_range);
        vi.base.set_goal(goal.x, goal.y, goal_t_deg);

        let mut director =
            SolveDirector { cfg: &self.cfg, cancel, from, on_progress: on_chunk };
        let out =
            solve_observed(&mut vi.base, self.cfg.solver, self.cfg.max_solve_iter, &mut director);
        stats.iters = out.iters;
        stats.truncated = out.stopped;
        if out.cancelled {
            return Err(PlanError::Cancelled);
        }
        if !out.converged && !out.stopped {
            return Err(PlanError::NotConverged);
        }
        Ok(Field::Dense(Box::new(vi)))
    }

    // ──────────────────────────────────────────────────────────────────────
    // 広域: compute_path_to_pose
    // ──────────────────────────────────────────────────────────────────────

    /// start から goal への経路を計画する。価値関数は `prepare_goal` と共有され、
    /// 同一ゴールなら solve をスキップしてロールアウトだけを行う。
    pub fn plan(
        &mut self,
        start: PoseView,
        goal: PoseView,
        cancel: &AtomicBool,
    ) -> Result<(Vec<PathPose>, PlanStats), PlanError> {
        self.plan_with_progress(start, goal, cancel, &mut |_| {})
    }

    /// `plan` の途中経過コールバック付き版 (`prepare_goal_with_progress` と同じ規約)。
    ///
    /// `start` は早期打ち切り (`early_start`) の起点でもある。打ち切った場で
    /// ロールアウトが通らなかったときは**その場で捨てて解き直す** — 失敗を
    /// そのまま返すと、キャッシュに載ったままの場を BT が何度でも引き直して
    /// 同じ失敗を繰り返す。
    pub fn plan_with_progress(
        &mut self,
        start: PoseView,
        goal: PoseView,
        cancel: &AtomicBool,
        on_chunk: &mut dyn FnMut(&dyn PolicyView),
    ) -> Result<(Vec<PathPose>, PlanStats), PlanError> {
        let mut s = self.prepare_goal_with_progress(goal, Some(start), cancel, on_chunk)?;

        let mut r = self.rollout(start);
        if !r.reached_goal() && self.discard_truncated() {
            eprintln!(
                "vi_planner: the truncated value function (early_start) does not roll out from \
                 ({:.2}, {:.2}): {:?} — solving this goal to convergence instead",
                start.x, start.y, r.status
            );
            // 起点を渡さない = 今度は打ち切らない。先読みが次の点を解いている
            // 最中ならここで取り消されて注文し直しになる (走っている間に解き直す
            // 時間はあるので、失う仕事より繰り返す失敗のほうが高い)。
            s = self.prepare_goal_with_progress(goal, None, cancel, on_chunk)?;
            r = self.rollout(start);
        }
        if !r.reached_goal() {
            return Err(PlanError::Rollout(r.status));
        }
        let poses = if self.cfg.path_spacing > 0.0 {
            densify(&r.poses, self.cfg.path_spacing)
        } else {
            r.poses
        };
        Ok((
            poses.clone(),
            PlanStats {
                solved_now: s.solved_now,
                iters: s.iters,
                adopted: s.adopted,
                truncated: s.truncated,
                poses: poses.len(),
            },
        ))
    }

    /// いま載っている場の貪欲ロールアウト。ゴール未設定なら `NoAction`。
    ///
    /// 密経路は追従で注入された local_penalty 込みの値を読む。compact 経路は
    /// sink を読む。sink には狭域が `commit_window` で返した値も入っているので、
    /// こちらも動的障害物込みの値になる (モジュール冒頭)。
    fn rollout(&self, start: PoseView) -> Rollout {
        let Some(cached) = self.cached.as_ref() else {
            return Rollout { poses: Vec::new(), status: RolloutStatus::NoAction };
        };
        let go = |p: &dyn PolicyView| {
            rollout_path_on(
                p,
                start.x,
                start.y,
                start.yaw_rad,
                self.cfg.max_rollout_steps,
                self.cfg.start_tolerance_cells,
            )
        };
        match &cached.field {
            Field::Dense(vi) => go(&vi.base),
            Field::Compact(f) => go(&f.policy()),
        }
    }

    // ──────────────────────────────────────────────────────────────────────
    // 狭域: follow_path
    // ──────────────────────────────────────────────────────────────────────

    /// 現在ゴールまでの XY 距離 (Feedback 用)。ゴール未設定なら None。
    pub fn goal_distance(&self, x: f64, y: f64) -> Option<f64> {
        self.cached
            .as_ref()
            .map(|c| ((c.goal_x - x).powi(2) + (c.goal_y - y).powi(2)).sqrt())
    }

    /// θ=0 全域スライスの可視化グリッド。ゴール未設定なら None。
    /// compact 経路では sink を全走査するので、広域地図では相応に重い
    /// (`publish_value_function: false` を推奨)。
    pub fn value_grid(&self, threshold_steps: u64) -> Option<OccupancyGrid> {
        match &self.cached.as_ref()?.field {
            Field::Dense(vi) => Some(value_grid_on(&vi.base, threshold_steps)),
            Field::Compact(f) => Some(value_grid_on(&f.policy(), threshold_steps)),
        }
    }

    /// ローカルウィンドウ範囲だけを現在方位の θ スライスで描画した可視化グリッド。
    /// `set_window` 後に呼ぶこと。地図端ではクランプ後の実範囲を使うので、本家
    /// `makeLocalValueFunctionMap` と違い幅とデータ長が常に一致する。
    // ponytail: 可視化は b=0 スライス固定 ([`value_grid_on`] も同じ)。b̂ 層を見たく
    // なったら to_index4 に ib を通す。
    pub fn window_value_grid(
        &self,
        pose: PoseView,
        threshold_steps: u64,
    ) -> Option<OccupancyGrid> {
        let vi = self.local()?;
        let (_, _, it) = pose_to_cell(&vi.base, pose.x, pose.y, pose.yaw_rad);
        let it = it.clamp(0, vi.base.cell_num_t - 1);
        let (x0, x1) = (vi.local_ix_min, vi.local_ix_max);
        let (y0, y1) = (vi.local_iy_min, vi.local_iy_max);
        if x1 < x0 || y1 < y0 {
            return None;
        }
        let (w, h) = ((x1 - x0 + 1) as usize, (y1 - y0 + 1) as usize);
        let mut slice = vec![0u64; w * h];
        for iy in y0..=y1 {
            for ix in x0..=x1 {
                slice[(iy - y0) as usize * w + (ix - x0) as usize] =
                    vi.base.states[vi.base.to_index(ix, iy, it) as usize].total_cost;
            }
        }
        Some(OccupancyGrid {
            width: w as i32,
            height: h as i32,
            resolution: vi.base.xy_resolution,
            origin_x: vi.base.map_origin_x + x0 as f64 * vi.base.xy_resolution,
            origin_y: vi.base.map_origin_y + y0 as f64 * vi.base.xy_resolution,
            origin_quat: vi.base.map_origin_quat.clone(),
            data: value_slice_to_occupancy(&slice, w, h, threshold_steps),
        })
    }

    /// ローカルウィンドウをロボット位置中心へ移動 (本家 `setLocalWindow`)。
    /// compact 経路では、必要ならその前にパッチを置き直して compact の場から
    /// 起こし直す。前の位置での成果は `commit_window` が毎 tick sink へ返して
    /// あるので、置き直しで失われるものは無い。
    pub fn set_window(&mut self, pose: PoseView) {
        // build / cached / patch / penalty は別フィールドなので分割して借りられる。
        let build = &self.build;
        let patch = &mut self.patch;
        let overlay = self.penalty.as_ref();
        let Some(c) = self.cached.as_mut() else { return };
        match &mut c.field {
            Field::Dense(vi) => vi.set_local_window(pose.x, pose.y),
            Field::Compact(f) => {
                let Some(p) = patch.as_mut() else { return };
                let res = f.resolution;
                let gx = ((pose.x - f.origin.0) / res).floor() as i32;
                let gy = ((pose.y - f.origin.1) / res).floor() as i32;
                if p.needs_recenter(gx, gy) {
                    p.hydrate(f, build, overlay, (gx - p.half, gy - p.half));
                }
                p.vi.set_local_window(pose.x, pose.y);
            }
        }
    }

    /// スキャンのヒット点周辺に local_penalty を注入 (本家 `setLocalCost`)。
    /// `set_window` 後に呼ぶこと (ウィンドウ外のヒットは無視される)。
    pub fn observe_scan(&mut self, scan: &LaserScan, pose: PoseView) {
        self.observe_scan_gated(scan, pose, 0);
    }

    /// 品質ゲート付きの [`Self::observe_scan`]: 注入 penalty を `2048 >> shift` に
    /// 減衰する。自己位置のフィットが怪しい tick は localizer 自身がそれを
    /// `quality()` で知っている — そのスキャンの投影は地図とずれており、満額で
    /// 塗るとゴースト壁がロボットを global 勾配との間に挟んで止める。shift は
    /// [`quality_shift`] で quality から量子化する。0 は `observe_scan` と同一。
    pub fn observe_scan_gated(&mut self, scan: &LaserScan, pose: PoseView, shift: u32) {
        let clear = self.cfg.footprint_clear_m;
        if let Some(vi) = self.local_mut() {
            vi.set_local_cost_attenuated(scan, pose.x, pose.y, pose.yaw_rad, shift);
            // 注入の後に footprint を消す — 機体の真上に塗られたゴーストが勝たない
            // ように (harvest より前なので compact の penalty 表にも 0 が写る)。
            if clear > 0.0 {
                vi.clear_local_penalty_around(pose.x, pose.y, clear);
            }
        }
        self.harvest_penalties();
    }

    /// ローカルウィンドウ内の価値反復を `budget` の範囲で回す (本家
    /// `localValueIterationWorker` の常駐スレッドを制御周期内の時間予算に
    /// 置き換えたもの)。1 パスの Δ 合計が 0 になったら予算を残して早期リターン
    /// する。戻り値は最後のパスの Δ 合計。
    pub fn refine_for(&mut self, budget: Duration) -> u64 {
        let t0 = Instant::now();
        loop {
            let (pass_delta, stopped) = self.refine_pass_until(|| t0.elapsed() >= budget);
            // 狭域が共有場を動かした。全域掃きに伝播させる仕事があると印を付ける。
            // compact は窓を書き戻して初めて共有場が動くので、印は `commit_window`
            // が (書けた範囲を待ち行列へ入れるのと同時に) 立てる。ここで立てると、
            // 書き戻す先が無い Δ でも掃きスレッドを起こしてしまう。
            if pass_delta > 0 && !self.cfg.use_compact() {
                self.dirty = true;
            }
            if stopped || pass_delta == 0 {
                // compact 経路は窓を sink へ返して初めて共有場になる。予算切れでも
                // 返す (次の tick で続きから精密化する)。密経路では何もしない。
                self.commit_window();
                return pass_delta;
            }
        }
    }

    /// ウィンドウ全体を `n` パス回す (決定的テスト用)。
    #[cfg(test)]
    pub fn refine_passes(&mut self, n: usize) {
        for _ in 0..n {
            let _ = self.refine_pass_until(|| false);
        }
        self.commit_window();
    }

    // ──────────────────────────────────────────────────────────────────────
    // 狭域 → 広域: 共有場の全域掃き
    //
    // 入口は `sweep_global` 1 つ。密はここで全域 Gauss–Seidel、compact は
    // `compact::` 側の `repair_one_tile` (タイル修復) へ分岐する。
    // ──────────────────────────────────────────────────────────────────────

    /// 狭域が共有場を動かしてから、まだ全域へ伝播させ切っていないか。
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// compact 経路: 直近の伝播 1 回で処理したタイル数 (密経路では None)。
    ///
    /// compact の「1 掃き」は地図の大きさで決まらず、変化が及んだ範囲で決まる。
    /// 経過時間だけ出しても速いのか仕事が少なかったのか読めないので、ログには
    /// これを添えること。
    pub fn sweep_tiles(&self) -> Option<usize> {
        self.repair.as_ref().map(|r| r.last_visits)
    }

    /// compact 経路: 進行中の伝播の進み具合 `(これまでの訪問数, 残りタイル数)`。
    /// 伝播していないときと密経路では None。
    ///
    /// **走行中は待ち行列がまず空にならない**。壁が窓 (±1m) に入っていれば
    /// `set_local_cost` が毎 tick penalty を塗り直すので `commit_window` が動き、
    /// 次の伝播を積む (実測: 壁が窓の中にある通路を 100 秒走って、待ち行列が
    /// 空だったのは 1000 tick 中 13 回だけ)。つまり `sweep_tiles` の「1 回終わった」
    /// ログはほぼ出ない — 掃きが動いているかはこちらで見ること。
    pub fn repair_progress(&self) -> Option<(usize, usize)> {
        let r = self.repair.as_ref()?;
        (!r.queue.is_empty()).then_some((r.visits, r.queue.len()))
    }

    /// 共有価値関数を全域で最大 `max_cells` セルぶん掃き進める (Gauss–Seidel)。
    /// 戻り値は `(このチャンクの Δ 合計, 1 掃きを終えたか)`。
    ///
    /// # これが狭域 → 広域のフィードバック経路そのもの
    ///
    /// 密経路の `states` は最初から共有場になっている。狭域は `observe_scan` で
    /// `local_penalty` を書き込み、広域は `plan_*` のロールアウトで同じ配列を読む。
    /// 足りていなかったのは**伝播**だけで、`refine_pass_until` が掃くのは
    /// ローカルウィンドウ (±1m) の中だけなので、局所で上がった値はそこで止まって
    /// いた。20m 先から降りてくるロールアウトは塞がった通路へ降り続け、着いてから
    /// 初めて気づいて `decide` が NoAction を返すか貪欲降下が往復する。
    ///
    /// ここで同じ `states` を全域 Gauss–Seidel で掃けば、その値の上昇が外へ
    /// 広がり、広域の経路が自然に迂回へ変わる。新しい Bellman 更新は書かず
    /// `value_iteration_at` をそのまま使うので、狭域・広域・solve の 3 者は
    /// 同一の更新式のままになる (vi_rs 側には手を入れない)。
    ///
    /// # 呼び出し規約
    ///
    /// 10 Hz の追従ループと同じ `Mutex<PlannerCore>` を共有するので、**1 掃きを
    /// ロックの中で走り切らせないこと**。`cur` を持ち回してチャンクごとに
    /// ロックを手放す (`run_follow` は `try_lock` に 3 回続けて失敗するとロボットを
    /// 止める)。1 掃き終わるごとに掃き順を次へ回すので、伝播方向は偏らない。
    ///
    /// # compact 経路ではタイル 1 枚を修復する
    ///
    /// compact に全域の `states` は無いので、[`compact::Repair`] のタイルを 1 枚だけ処理して
    /// 返る (`max_cells` と `cur` は使わない)。`done` は待ち行列が空になったとき。
    /// 呼び出し規約は密経路と同じ — 呼び出し側は `done` か予算切れまで繰り返す。
    ///
    /// 1 呼び出しの仕事は「ハイドレート + 高々 2 パス + 書き戻し」で頭打ちなので、
    /// ロックを握る時間は予算 + タイル 1 枚ぶんに収まる (0.25 m/cell で数十 ms、
    /// 追従ループが `try_lock` に 3 回失敗する 300 ms には余裕がある)。
    pub fn sweep_global(&mut self, cur: &mut SweepCursor, max_cells: usize) -> (u64, bool) {
        if self.cfg.use_compact() {
            return self.repair_one_tile();
        }
        let Some(c) = self.cached.as_mut() else { return (0, true) };
        let Field::Dense(vi) = &mut c.field else {
            self.dirty = false;
            return (0, true);
        };

        let orders = vi.base.sweep_orders.len();
        if orders == 0 {
            self.dirty = false;
            return (0, true);
        }
        // ゴールが変わってもセル数は変わらない (地図が同じ) ので、持ち越した
        // カーソルは掃きの途中から再開するだけで害はない。念のため丸める。
        let order = cur.order % orders;
        let len = vi.base.sweep_orders[order].len();
        let start = cur.pos.min(len);
        let end = start.saturating_add(max_cells.max(1)).min(len);

        let mut delta = 0u64;
        for k in start..end {
            let i = vi.base.sweep_orders[order][k] as usize;
            delta = delta.saturating_add(vi.base.value_iteration_at(i));
        }
        cur.delta = cur.delta.saturating_add(delta);
        cur.pos = end;
        if end < len {
            return (delta, false);
        }

        // 1 掃き完了。丸ごと Δ=0 なら新しい不動点に達している。
        let swept_delta = cur.delta;
        cur.order = (order + 1) % orders;
        cur.pos = 0;
        cur.delta = 0;
        if swept_delta == 0 {
            self.dirty = false;
        }
        (delta, true)
    }

    /// ウィンドウ 1 パス。`should_stop` は x 列ごとに観測し、途中打ち切り時は
    /// `(それまでの Δ 合計, true)` を返す。
    fn refine_pass_until(&mut self, should_stop: impl Fn() -> bool) -> (u64, bool) {
        let Some(vi) = self.local_mut() else { return (0, true) };
        let nt = vi.base.cell_num_t;
        // b 層も舐める (本家 `local_value_iteration_loop` と同じ。nb_levels==1 では
        // 1 周 = 従来と同一)。窓の中だけ 4D で精密化しないと、b̂ 層の値だけが
        // スキャンの penalty を知らないまま残る。
        let nb = vi.base.belief_levels();
        let mut delta = 0u64;
        for iix in vi.local_ix_min..=vi.local_ix_max {
            if should_stop() {
                return (delta, true);
            }
            for iiy in vi.local_iy_min..=vi.local_iy_max {
                for iit in 0..nt {
                    for ib in 0..nb {
                        let i = vi.base.to_index4(iix, iiy, iit, ib);
                        delta = delta.saturating_add(vi.value_iteration_local(i));
                    }
                }
            }
        }
        (delta, false)
    }

    /// 現在姿勢の判断 (読み取り専用)。実体は [`PlanConfig::follow_controller`] で
    /// 選んだ [`FollowController`] — 既定の greedy は本家 `ViNode::decision` の
    /// `posToAction` 相当 (方策が無ければ近傍借用)、dwa は連続行動。
    /// compact 経路では `set_window` でパッチを起こしてから呼ぶこと。
    ///
    /// `ib_hat` は belief 不確かさレベルの推定値 ([`Belief::b_hat`])。`nb_levels == 1`
    /// では 0 を渡せば従来と同一。
    pub fn decide(&self, pose: PoseView, ib_hat: i32) -> Decision {
        let Some(local) = self.local() else { return Decision::NoAction };
        let ib = ib_hat.clamp(0, local.base.belief_levels() - 1);
        self.follow.decide(&local.base, &self.cfg, pose, ib)
    }

    /// belief 仮説集合での判断 (QMDP — [`qmdp_decide`])。読む場は [`Self::decide`]
    /// と同じ密な局所場 (compact 経路では `set_window` 済みパッチ — パッチ外の
    /// 仮説は評価対象外)。仮説が割れていても「どの仮説でも悪くない行動」を選び、
    /// 有意な仮説が衝突と言う行動しか無ければ `NoAction` (停止)。
    /// `follow_controller` (dwa/mppi) は経由しない — 多峰時は離散 QMDP が優先で、
    /// 単峰時は呼び出し側が [`Self::decide`] へフォールバックする分担。
    pub fn decide_qmdp(&self, hyps: &[(PoseView, f64)], ib_hat: i32) -> Decision {
        let Some(local) = self.local() else { return Decision::NoAction };
        match qmdp_decide(&local.base, hyps, ib_hat) {
            QmdpDecision::Goal => Decision::Goal,
            QmdpDecision::Action(ai) => {
                let a = &local.base.actions[ai];
                Decision::Action { id: Some(a.id as usize), fw: a.delta_fw, rot_deg: a.delta_rot }
            }
            QmdpDecision::NoAction => Decision::NoAction,
        }
    }
}
