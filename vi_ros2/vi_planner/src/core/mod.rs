//! rclrs 非依存の統合プランナ中核。**1 本の価値関数**を広域 (経路計画) と
//! 狭域 (経路追従) の両方で共有する。
//!
//! vi_global_planner (`compute_path_to_pose`) と vi_local_planner
//! (`follow_path`) を別プロセスで動かすと、同じ地図・同じゴールに対して同じ
//! 価値反復を 2 回解くことになる (走り出しまでの時間も、常駐する価値関数の
//! メモリも 2 倍)。しかも広域側が作った `nav_msgs/Path` は狭域側が終端姿勢しか
//! 読まないので、ロールアウトの成果はほぼ捨てられていた。ここではその 2 つを
//! 1 つの価値関数の上に載せ直す:
//!
//!   - solve はゴールごとに 1 回だけ (`prepare_goal_with_progress`)
//!   - `plan_*` は解決済み価値関数の貪欲ロールアウト = 旧実装の
//!     「キャッシュヒット経路」そのもの
//!   - 追従は同じ価値関数の ±1m ウィンドウをスキャンで補正しながら回す
//!
//! # ファイルの分かれ方
//!
//! - `core/mod.rs` (ここ) — 公開型と [`PlannerCore`] の本体。密 (dense) 経路の
//!   solve・ロールアウト・窓の精密化・全域掃きはすべてここにある。
//! - [`compact`] — アウトオブコア経路だけの機構 (sink・追従パッチ・penalty 表・
//!   タイル修復)。`PlannerCore` の compact 側メソッドもそちらに置いてある。
//! - `core/tests.rs` — 両経路をまたぐテスト。
//!
//! # 狭域 → 広域のフィードバック ([`PlannerCore::sweep_global`])
//!
//! 上の 3 つ目で狭域が書き込む `local_penalty` は、密経路では**同じ `states` に
//! 載る**。共有場はここで既に成立している。足りないのは伝播で、局所精密化
//! (`refine_pass_until`) が掃くのはウィンドウの中だけなので、上がった値はそこで
//! 止まり広域のロールアウトは塞がった通路へ降り続ける。`sweep_global` が同じ
//! `states` を全域 Gauss–Seidel で掃き直してそれを外へ広げる (詳細はそちらの doc)。
//!
//! `local_penalty` はウィンドウの外では**誰も消さない** (`set_local_cost` は
//! `in_local_area` の中しか触らない)。障害物の脇を通り過ぎるとその penalty は
//! そのゴールの間ずっと `states` に残り、全域掃きのたびに広域の場を歪め続ける。
//! これは本家 `ViNode` から引き継いだ挙動で、意図的にそのままにしてある
//! (「一度通れないと分かった場所は覚えておく」= 望ましい側の効果でもあるため)。
//! 消したければゴールを取り直すこと。
//!
//! ## 効いているかを見るときに引っかかる 2 つ
//!
//! - **塞ぎ方で桁が変わる。** `set_local_cost` が置くのは壁ではなくコストなので、
//!   通路の一部だけを塞いでも脇を抜けられれば遠方の値はほぼ動かない (幅 2m の
//!   通路を幅 0.4m 塞いで +0.75 ステップ = `cost_drawing_threshold: 60` の色 1 段)。
//!   幅いっぱい塞げば桁が変わる (実測 13 → 38 ステップ、12 秒相当で収束。
//!   `tests::a_full_width_block_raises_the_value_far_outside_the_window`)。
//! - **走行中は待ち行列がまず空にならない** (compact 経路)。壁が窓 (±1m) に
//!   入っていれば `set_local_cost` が毎 tick penalty を塗り直すので、次の伝播が
//!   積まれ続ける (実測 1000 tick 中 987 tick が dirty)。「1 回終わった」ログを
//!   待っても出ないので、[`PlannerCore::repair_progress`] のほうを見ること。
//!
//! # 2 つの経路: 密 (dense) とアウトオブコア (compact)
//!
//! `PlanConfig::solver` が `frontier2d_sparse_compact` かどうかで価値関数の持ち方が
//! 変わる。広域と狭域が 1 本の場を共有するという上の性質はどちらでも同じ。
//!
//! - **密**: `ValueIteratorLocal` を全域ぶん確保する (`State` は 56 B/state)。
//!   狭域はその `states` をその場で書き換える。小〜中規模の地図はこちら。
//! - **compact**: `solve_compact_mapped` が `states` を作らずに解き、確定出力
//!   (12 B/state) だけを `CompactSink` (既定は mmap ファイル) に置く。追従は
//!   `states` を必要とするので、**ロボット近傍だけを compact の場から起こした
//!   小さな密パッチ** ([`compact::Patch`]) の上で回す。津田沼のような広域地図
//!   (0.25 m/cell で 5650 万状態 = 密なら 3.17 GB) を Pi4 4GB に載せるための経路。
//!
//! compact 側の作り (sink がどう共有場になるか、全域伝播をタイル修復でどう回すか)
//! は [`compact`] の doc にある。
//!
//! `BuildParams` が vi_global_planner::core と重複しているのは意図的
//! (クレート間依存を避けるため; vi_local_planner から引き継いだ規約)。
//!
//! このモジュールは vi_reference のみに依存し、ホストで `cargo test --lib`
//! できる (分離クレート方式; リポジトリ CLAUDE.md 参照)。

mod compact;
#[cfg(test)]
mod tests;

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use ndarray::Array2;

use vi_reference::bridge::{value_slice_to_occupancy, yaw_to_goal_theta_deg, PoseView};
use vi_reference::msg::{LaserScan, OccupancyGrid};
use vi_reference::planner::{
    densify, optimal_action_at, pose_to_cell, rollout_path_on, PathPose, RolloutStatus,
};
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::value_iterator::ValueIterator;
use vi_reference::{Action, ValueIteratorLocal};

use compact::{new_patch, new_repair, CompactField, Patch, PenaltyOverlay, Repair};

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
}

/// アウトオブコア (compact) 経路の確定出力の置き場。`None` = RAM (`RamSink`)、
/// `Some(dir)` = そのディレクトリ上の mmap ファイル。必要量は
/// `nx·ny·theta_cell_num × 12 B` なので、小メモリ機ではディスクに逃がす。
pub type SinkDir = Option<std::path::PathBuf>;

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

    // ── アウトオブコア (compact) 経路 ──
    /// 確定出力の置き場。`solver` が `frontier2d_sparse_compact` のときだけ参照する。
    pub compact_sink_dir: SinkDir,
    /// compact 経路のワーカースレッド数 (0 = `default_threads()`)。
    pub vi_threads: usize,

    // ── 狭域 → 広域の全域伝播 ──
    /// 全域掃き ([`PlannerCore::sweep_global`]) を回すか。compact 経路では
    /// これが false のとき修復タイル ([`compact::Repair`], 数 MB) を確保しない。
    pub global_sweep: bool,
}

impl PlanConfig {
    /// アウトオブコア (`states` を作らない) 経路を使うか。
    pub fn use_compact(&self) -> bool {
        matches!(self.solver, U64Solver::Frontier2DSparseCompact { .. })
    }
}

/// solve 単体の統計 (ログ用)。
#[derive(Clone, Copy, Debug)]
pub struct SolveStats {
    /// この呼び出しで solve を実行したか (false = キャッシュヒット)。
    pub solved_now: bool,
    /// 実行した solve イテレーション数 (キャッシュヒット時 0)。
    pub iters: u32,
}

/// 1 回の plan の統計 (ログ/Feedback 用)。
#[derive(Clone, Copy, Debug)]
pub struct PlanStats {
    pub solved_now: bool,
    pub iters: u32,
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
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlanError::Cancelled => write!(f, "cancelled (preempted)"),
            PlanError::NotConverged => write!(f, "value iteration did not converge"),
            PlanError::Rollout(s) => write!(f, "policy rollout failed: {s:?}"),
            PlanError::Sink(e) => write!(f, "compact output sink unavailable: {e}"),
            PlanError::Patch(e) => write!(f, "follow patch cannot be built: {e}"),
        }
    }
}

/// 1 制御周期の判断。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Decision {
    /// 現在姿勢がゴール圏 (`final_state`)。
    Goal,
    /// 最適行動。`fw` は前進量 [m]、`rot_deg` は回転量 [deg]
    /// (本家 `ViNode::decision` はこれをそのまま速度指令として配信する)。
    Action { id: usize, fw: f64, rot_deg: f64 },
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
}

/// 円環上の角度差 (度、0..=180)。
fn circ_deg_diff(a: i32, b: i32) -> i32 {
    let d = (a - b).rem_euclid(360);
    d.min(360 - d)
}

/// `(ix, iy, it)` に方策があれば Decision::Action を返す (読み取り専用)。
fn action_at(vi: &ValueIterator, ix: i32, iy: i32, it: i32) -> Option<Decision> {
    let id = optimal_action_at(vi, ix, iy, it);
    if id < 0 {
        return None;
    }
    let a = vi.actions.iter().find(|a| a.id == id)?;
    Some(Decision::Action { id: id as usize, fw: a.delta_fw, rot_deg: a.delta_rot })
}

/// 範囲内で final_state か (境界チェック込み)。
fn is_final(vi: &ValueIterator, ix: i32, iy: i32, it: i32) -> bool {
    vi.in_map_area(ix, iy)
        && it >= 0
        && it < vi.cell_num_t
        && vi.states[vi.to_index(ix, iy, it) as usize].final_state
}

/// solve 済み ValueIterator の θ=0 全域スライスを可視化用 OccupancyGrid に描画する
/// (0..=100、未到達 -1)。solve 中の途中経過 (`*_with_progress` のコールバック) からも
/// 呼べるよう自由関数にしてある。
pub fn value_grid_of(vi: &ValueIterator, threshold_steps: u64) -> OccupancyGrid {
    let (nx, ny) = (vi.cell_num_x, vi.cell_num_y);
    let mut slice = Array2::<u64>::zeros((ny as usize, nx as usize));
    for iy in 0..ny {
        for ix in 0..nx {
            slice[[iy as usize, ix as usize]] =
                vi.states[vi.to_index(ix, iy, 0) as usize].total_cost;
        }
    }
    OccupancyGrid {
        width: nx,
        height: ny,
        resolution: vi.xy_resolution,
        origin_x: vi.map_origin_x,
        origin_y: vi.map_origin_y,
        origin_quat: vi.map_origin_quat.clone(),
        data: value_slice_to_occupancy(&slice, threshold_steps),
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
        Self { build, cfg, cached: None, patch: None, penalty, repair: None, dirty: false }
    }

    /// キャッシュ中のゴールが `goal` と同一 (許容差内) か。追従スレッドが
    /// 「自分のゴールの価値関数がまだ載っているか」を毎 tick 確認するのに使う
    /// (広域側の計画要求が別ゴールで solve し直すとキャッシュが差し替わるため)。
    pub fn is_cached_goal(&self, goal: PoseView) -> bool {
        self.cache_matches(&goal, yaw_to_goal_theta_deg(goal.yaw_rad))
    }

    fn cache_matches(&self, goal: &PoseView, goal_t_deg: i32) -> bool {
        let Some(c) = self.cached.as_ref() else { return false };
        let d2 = (c.goal_x - goal.x).powi(2) + (c.goal_y - goal.y).powi(2);
        d2.sqrt() <= self.cfg.goal_tolerance_xy
            && circ_deg_diff(c.goal_t_deg, goal_t_deg) as f64 <= self.cfg.goal_tolerance_deg
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
        self.prepare_goal_with_progress(goal, cancel, &mut |_| {})
    }

    /// `prepare_goal` と同じだが、`solve_chunk` ごとの write_back 後に `on_chunk`
    /// を呼ぶ (途中経過の value_function 可視化用)。キャッシュヒット時は呼ばれない。
    /// compact 経路は 1 回で走り切るので `on_chunk` は呼ばれない。
    pub fn prepare_goal_with_progress(
        &mut self,
        goal: PoseView,
        cancel: &AtomicBool,
        on_chunk: &mut dyn FnMut(&ValueIterator),
    ) -> Result<SolveStats, PlanError> {
        let goal_t_deg = yaw_to_goal_theta_deg(goal.yaw_rad);
        let mut stats = SolveStats { solved_now: false, iters: 0 };

        if self.cache_matches(&goal, goal_t_deg) {
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

        let field = if self.cfg.use_compact() {
            // パッチも修復タイルも幾何だけで決まるので初回だけ作る
            // (遷移表の再計算は 64^3 サブセルサンプリング x 行動 x θ で重い)。
            if self.patch.is_none() {
                let p = new_patch(&self.build)?;
                if self.cfg.global_sweep && self.repair.is_none() {
                    self.repair = Some(new_repair(&self.build, p.reach)?);
                }
                self.patch = Some(p);
            }
            self.solve_compact(&goal, goal_t_deg, &mut stats, cancel)?
        } else {
            self.solve_dense(&goal, goal_t_deg, &mut stats, cancel, on_chunk)?
        };

        stats.solved_now = true;
        self.cached = Some(CachedGoal { goal_x: goal.x, goal_y: goal.y, goal_t_deg, field });
        Ok(stats)
    }

    /// 密経路: `ValueIterator::states` を確保し、`solve_chunk` ごとに cancel を観測しながら解く。
    fn solve_dense(
        &self,
        goal: &PoseView,
        goal_t_deg: i32,
        stats: &mut SolveStats,
        cancel: &AtomicBool,
        on_chunk: &mut dyn FnMut(&ValueIterator),
    ) -> Result<Field, PlanError> {
        let mut vi = ValueIteratorLocal::new(self.build.actions.clone(), 1);
        vi.set_map_with_occupancy_grid(
            &self.build.grid,
            self.build.theta_cell_num,
            self.build.safety_radius,
            self.build.safety_radius_penalty,
            self.build.goal_margin_radius,
            self.build.goal_margin_theta,
        );
        vi.base.set_goal(goal.x, goal.y, goal_t_deg);

        let mut remaining = self.cfg.max_solve_iter;
        let converged = loop {
            if cancel.load(Ordering::Relaxed) {
                return Err(PlanError::Cancelled);
            }
            if remaining == 0 {
                break false;
            }
            let chunk = remaining.min(self.cfg.solve_chunk.max(1));
            let s = solve(&mut vi.base, self.cfg.solver, chunk);
            stats.iters = stats.iters.saturating_add(s.iters);
            remaining -= chunk;
            on_chunk(&vi.base);
            if s.converged {
                break true;
            }
        };
        if !converged {
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
    pub fn plan_with_progress(
        &mut self,
        start: PoseView,
        goal: PoseView,
        cancel: &AtomicBool,
        on_chunk: &mut dyn FnMut(&ValueIterator),
    ) -> Result<(Vec<PathPose>, PlanStats), PlanError> {
        let s = self.prepare_goal_with_progress(goal, cancel, on_chunk)?;

        let cached = self.cached.as_ref().expect("cache filled above");
        // 密経路は追従で注入された local_penalty 込みの値を読む。compact 経路は
        // sink を読む。sink には狭域が `commit_window` で返した値も入っている
        // ので、こちらも動的障害物込みの値になる (モジュール冒頭)。
        let r = match &cached.field {
            Field::Dense(vi) => rollout_path_on(
                &vi.base,
                start.x,
                start.y,
                start.yaw_rad,
                self.cfg.max_rollout_steps,
                self.cfg.start_tolerance_cells,
            ),
            Field::Compact(f) => rollout_path_on(
                &f.policy(),
                start.x,
                start.y,
                start.yaw_rad,
                self.cfg.max_rollout_steps,
                self.cfg.start_tolerance_cells,
            ),
        };
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
            PlanStats { solved_now: s.solved_now, iters: s.iters, poses: poses.len() },
        ))
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
            Field::Dense(vi) => Some(value_grid_of(&vi.base, threshold_steps)),
            Field::Compact(f) => {
                let (nx, ny) = (f.cell_num.0, f.cell_num.1);
                let slice = Array2::from_shape_vec(
                    (ny as usize, nx as usize),
                    f.policy().value_slice_theta0(),
                )
                .ok()?;
                Some(OccupancyGrid {
                    width: nx,
                    height: ny,
                    resolution: f.resolution,
                    origin_x: f.origin.0,
                    origin_y: f.origin.1,
                    origin_quat: f.origin_quat.clone(),
                    data: value_slice_to_occupancy(&slice, threshold_steps),
                })
            }
        }
    }

    /// ローカルウィンドウ範囲だけを現在方位の θ スライスで描画した可視化グリッド。
    /// `set_window` 後に呼ぶこと。地図端ではクランプ後の実範囲を使うので、本家
    /// `makeLocalValueFunctionMap` と違い幅とデータ長が常に一致する。
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
        let mut slice = Array2::<u64>::zeros((h, w));
        for iy in y0..=y1 {
            for ix in x0..=x1 {
                slice[[(iy - y0) as usize, (ix - x0) as usize]] =
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
            data: value_slice_to_occupancy(&slice, threshold_steps),
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
        if let Some(vi) = self.local_mut() {
            vi.set_local_cost(scan, pose.x, pose.y, pose.yaw_rad);
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
        let mut delta = 0u64;
        for iix in vi.local_ix_min..=vi.local_ix_max {
            if should_stop() {
                return (delta, true);
            }
            for iiy in vi.local_iy_min..=vi.local_iy_max {
                for iit in 0..nt {
                    let i = vi.base.to_index(iix, iiy, iit) as usize;
                    delta = delta.saturating_add(vi.value_iteration_local(i));
                }
            }
        }
        (delta, false)
    }

    /// 現在姿勢の判断 (本家 `ViNode::decision` の `posToAction` 相当、読み取り
    /// 専用)。現在セルに方策が無ければ同一 θ の近傍 (チェビシェフ距離
    /// `action_tolerance_cells` 以内) から最近傍の行動 / ゴールセルを借りる。
    /// compact 経路では `set_window` でパッチを起こしてから呼ぶこと。
    pub fn decide(&self, pose: PoseView) -> Decision {
        let Some(local) = self.local() else { return Decision::NoAction };
        let vi = &local.base;
        let (ix, iy, it) = pose_to_cell(vi, pose.x, pose.y, pose.yaw_rad);

        if is_final(vi, ix, iy, it) {
            return Decision::Goal;
        }
        if let Some(d) = action_at(vi, ix, iy, it) {
            return d;
        }

        let tol = self.cfg.action_tolerance_cells;
        let mut best: Option<(i64, Decision)> = None;
        for dy in -tol..=tol {
            for dx in -tol..=tol {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let (nx, ny) = (ix + dx, iy + dy);
                let cand = if is_final(vi, nx, ny, it) {
                    Some(Decision::Goal)
                } else {
                    action_at(vi, nx, ny, it)
                };
                let Some(cand) = cand else { continue };
                let d2 = (dx as i64) * (dx as i64) + (dy as i64) * (dy as i64);
                if best.as_ref().map(|(bd, _)| d2 < *bd).unwrap_or(true) {
                    best = Some((d2, cand));
                }
            }
        }
        best.map(|(_, d)| d).unwrap_or(Decision::NoAction)
    }
}
