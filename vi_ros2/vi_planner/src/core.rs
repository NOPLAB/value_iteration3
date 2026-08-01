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
//!   小さな密パッチ** ([`Patch`]) の上で回す。津田沼のような広域地図 (0.25 m/cell で
//!   5650 万状態 = 密なら 3.17 GB) を Pi4 4GB に載せるための経路。
//!
//! パッチは「ローカルウィンドウ (±1m) + 遷移が届く距離 + 動ける余裕」の大きさで、
//! ウィンドウの外側は compact の値のまま凍結して境界条件に使う。凍結境界が
//! 成り立つ条件は「ウィンドウ内のセルの遷移先がパッチ内に収まる」ことで、
//! これは遷移表の実測値で起動時に検証する ([`transition_reach`])。
//!
//! compact 経路には密経路と 2 点だけ差異がある (どちらも意図的):
//!
//! 1. `plan_*` のロールアウトは sink (静的地図の解) を読む。密経路では追従中に
//!    注入されたスキャン由来の `local_penalty` 込みの値を読むので、動的障害物が
//!    経路にも反映されていた。compact ではされない (走行はパッチ側の方策に従うので
//!    避けること自体はできる)。
//! 2. パッチを置き直すとその周期のローカル精密化は捨てられる (密経路は全域 `states`
//!    に書くので残る)。スキャンは毎 tick 注入されるので次の tick で復元される。
//!
//! `BuildParams` が vi_global_planner::core と重複しているのは意図的
//! (クレート間依存を避けるため; vi_local_planner から引き継いだ規約)。
//!
//! このモジュールは vi_reference のみに依存し、ホストで `cargo test --lib`
//! できる (分離クレート方式; リポジトリ CLAUDE.md 参照)。

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use ndarray::Array2;

use vi_reference::bridge::{value_slice_to_occupancy, yaw_to_goal_theta_deg, PoseView};
use vi_reference::msg::{LaserScan, OccupancyGrid, Quaternion};
use vi_reference::params::{MAX_COST, PROB_BASE};
use vi_reference::planner::{
    densify, optimal_action_at, pose_to_cell, rollout_path_on, CompactPolicy, PathPose,
    RolloutStatus,
};
use vi_reference::solvers::frontier2d_sparse_compact::{
    default_threads, solve_compact_mapped, CompactSink, RamSink,
};
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::state::State;
use vi_reference::value_iterator::ValueIterator;
use vi_reference::{Action, ValueIteratorLocal};

/// `ValueIteratorLocal::set_map_with_occupancy_grid` が固定で入れるローカル
/// ウィンドウ半径 [m]。パッチの寸法を決めるのに先に知る必要があるので写しを持つ
/// (vi_rs 側は変更しない。ズレたら `new_patch` の検証が落ちる)。
const LOCAL_XY_RANGE_M: f64 = 1.0;

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
    /// compact 経路: 確定出力 (sink) + 幾何。狭域は [`Patch`] を経由する。
    Compact(Box<CompactField>),
}

/// compact 経路の solve 結果。`sink` は orig 索引で `(total_cost, action)` を返す確定出力。
struct CompactField {
    sink: Box<dyn CompactSink + Send>,
    actions: Vec<Action>,
    cell_num: (i32, i32, i32),
    resolution: f64,
    origin: (f64, f64),
    origin_quat: Quaternion,
    goal: (f64, f64, i32),
}

impl CompactField {
    fn policy(&self) -> CompactPolicy<'_> {
        CompactPolicy::new(
            self.sink.as_ref(),
            &self.actions,
            self.cell_num,
            self.resolution,
            self.origin,
            self.goal,
        )
    }

    /// グローバルセル `(ix,iy,it)` の sink 索引 (usize 演算; 広域地図で i32 が溢れる)。
    fn orig(&self, ix: i32, iy: i32, it: i32) -> usize {
        it as usize
            + ix as usize * self.cell_num.2 as usize
            + iy as usize * self.cell_num.0 as usize * self.cell_num.2 as usize
    }
}

/// compact 経路の追従用パッチ: ロボット近傍だけを compact の場から起こした密な
/// `ValueIteratorLocal`。幾何 (解像度・θ 数・行動) だけで決まりゴールには依らないので、
/// ゴールが変わってもパッチ自体は作り直さず、中身 (`at` とハイドレート値) だけ入れ替える。
///
/// 寸法: `half = 2*ウィンドウ半径 + 遷移到達距離 + 2` セル。ウィンドウの外側は
/// compact の値のまま凍結して境界条件に使う。ロボットがパッチの縁に近づいたら
/// 置き直す (`needs_recenter`)。
struct Patch {
    vi: ValueIteratorLocal,
    /// パッチ半径 [セル] (パッチは `2*half+1` 辺の正方形)。
    half: i32,
    /// 遷移が届く最大セル数 (遷移表の実測値)。凍結境界の成立条件に使う。
    reach: i32,
    /// パッチ左下のグローバルセル座標。`None` = 未ハイドレート。
    at: Option<(i32, i32)>,
}

struct CachedGoal {
    goal_x: f64,
    goal_y: f64,
    goal_t_deg: i32,
    field: Field,
}

pub struct PlannerCore {
    build: BuildParams,
    cfg: PlanConfig,
    cached: Option<CachedGoal>,
    /// compact 経路の追従用パッチ (ゴール非依存なのでキャッシュの外に置く)。
    /// 密経路では使わない。
    patch: Option<Patch>,
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

/// 遷移表が届く最大セル数 (x/y のチェビシェフ距離)。遷移は解像度と θ 数だけで
/// 決まるので、パッチの凍結境界が成り立つかはこの実測値で判定できる。
fn transition_reach(vi: &ValueIterator) -> i32 {
    vi.actions
        .iter()
        .flat_map(|a| a.state_transitions.iter().flatten())
        .map(|t| t.dix.abs().max(t.diy.abs()))
        .max()
        .unwrap_or(0)
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

/// compact 経路の追従用パッチを 1 つ作る (ゴール非依存)。
///
/// 中身はまだ空 (`at: None`)。寸法は「ローカルウィンドウ半径 + 遷移到達距離 +
/// 動ける余裕 (= ウィンドウ半径ぶん)」で、余裕を使い切ったら `hydrate` で置き直す。
fn new_patch(build: &BuildParams) -> Result<Patch, PlanError> {
    let res = build.grid.resolution;
    if res <= 0.0 {
        return Err(PlanError::Patch(format!("planner grid resolution is {res}")));
    }
    let win = (LOCAL_XY_RANGE_M / res) as i32; // ValueIteratorLocal と同じ式
                                               // 遷移の x/y 変位の上界。`cell_delta` はセル内オフセット (0..res) を足してから
                                               // floor するので、正側は floor(|fw|/res)、負側は -floor(|fw|/res)-1 まで届く。
    let max_fw = build.actions.iter().map(|a| a.delta_fw.abs()).fold(0.0f64, f64::max);
    let reach_bound = (max_fw / res).floor() as i32 + 1;
    let half = 2 * win + reach_bound + 2;
    let side = 2 * half + 1;

    let mut vi = ValueIteratorLocal::new(build.actions.clone(), 1);
    // 中身は hydrate が全部上書きするので、地図は全 free のダミーでよい
    // (ここで確定するのは幾何と遷移表と sweep_orders)。
    let dummy = OccupancyGrid {
        width: side,
        height: side,
        resolution: res,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: build.grid.origin_quat.clone(),
        data: vec![0i8; (side as usize) * (side as usize)],
    };
    vi.set_map_with_occupancy_grid(
        &dummy,
        build.theta_cell_num,
        build.safety_radius,
        build.safety_radius_penalty,
        build.goal_margin_radius,
        build.goal_margin_theta,
    );

    // 凍結境界の成立条件: ウィンドウ内のセルの遷移先がパッチに収まること。
    // 解析上界ではなく遷移表の実測で確かめる (1 セル足りないと、ある方位でだけ
    // ウィンドウ端の値が MAX_COST に見えて NoAction で止まる — 気付きにくい)。
    let reach = transition_reach(&vi.base);
    let win = vi.local_ixy_range;
    if win + reach >= half {
        return Err(PlanError::Patch(format!(
            "window {win} + transition reach {reach} cells does not fit in the follow patch \
             (half = {half} cells) at {res:.3} m/cell; action_forward_m is too large"
        )));
    }
    Ok(Patch { vi, half, reach, at: None })
}

impl Patch {
    /// ロボットのグローバルセル `(gx, gy)` に対してパッチを置き直す必要があるか。
    /// 判定は寸法ではなく凍結境界の条件そのもの: ウィンドウ (±`local_ixy_range`) と
    /// そこから遷移が届く先 (±`reach`) がパッチに収まらなくなったら置き直す。
    fn needs_recenter(&self, gx: i32, gy: i32) -> bool {
        let Some((p0x, p0y)) = self.at else { return true };
        let need = self.vi.local_ixy_range + self.reach;
        let side_max = 2 * self.half;
        gx - need < p0x
            || gy - need < p0y
            || gx - p0x + need > side_max
            || gy - p0y + need > side_max
    }

    /// compact の場と静的地図からパッチの `states` を起こす。
    /// `p0` はパッチ左下のグローバルセル座標 (地図外へ食い込んでよい — その分は
    /// 占有セル扱いになり、`action_cost` が MAX_COST を返す = 元の地図外判定と同じ)。
    fn hydrate(&mut self, f: &CompactField, build: &BuildParams, p0: (i32, i32)) {
        let (gnx, gny, nt) = f.cell_num;
        let res = f.resolution;
        let side = 2 * self.half + 1;
        let margin = (build.safety_radius / res).ceil() as i32;

        self.at = Some(p0);
        let base = &mut self.vi.base;
        base.map_origin_x = f.origin.0 + p0.0 as f64 * res;
        base.map_origin_y = f.origin.1 + p0.1 as f64 * res;
        base.map_origin_quat = f.origin_quat.clone();
        // ゴールは sink 側の規約 (value == 0) で判定するので final_state の再計算は
        // 要らないが、幾何の一貫性のために持たせておく。
        base.goal_x = f.goal.0;
        base.goal_y = f.goal.1;
        base.goal_t = f.goal.2;

        for py in 0..side {
            let gy = p0.1 + py;
            for px in 0..side {
                let gx = p0.0 + px;
                let inside = gx >= 0 && gx < gnx && gy >= 0 && gy < gny;
                // free / penalty は**グローバル座標・グローバル幅**で評価する。
                // `State::from_occupancy` の margin ループは行跨ぎバグを持つので、
                // パッチを切り出してから評価すると compact solve が見た値とズレる。
                let proto = if inside {
                    State::from_occupancy(
                        gx,
                        gy,
                        0,
                        &build.grid,
                        margin,
                        build.safety_radius_penalty,
                        gnx,
                    )
                } else {
                    // 地図外。`from_occupancy` の非 free 早期リターンと同じ形。
                    State {
                        total_cost: MAX_COST,
                        penalty: PROB_BASE,
                        local_penalty: 0,
                        ix: 0,
                        iy: 0,
                        it: 0,
                        free: false,
                        final_state: false,
                        optimal_action: None,
                    }
                };
                let orig0 = if inside { Some(f.orig(gx, gy, 0)) } else { None };
                for it in 0..nt {
                    let (v, a) = match orig0 {
                        Some(o) => f.sink.read(o + it as usize),
                        None => (MAX_COST, -1),
                    };
                    let idx = base.to_index(px, py, it) as usize;
                    let s = &mut base.states[idx];
                    s.ix = px;
                    s.iy = py;
                    s.it = it;
                    s.free = proto.free;
                    s.penalty = proto.penalty;
                    s.local_penalty = 0;
                    s.total_cost = v;
                    s.optimal_action = if a >= 0 { Some(a as usize) } else { None };
                    // sink の規約: 未到達 = MAX_COST、ゴール圏 = 0 (`CompactPolicy::is_final`
                    // と同じ判定)。`value_iteration_raw` は final_state を更新しないので、
                    // ゴール圏はローカル精密化でも 0 に留まる。
                    s.final_state = v == 0;
                }
            }
        }
    }
}

/// compact 出力 sink を作る。`dir` 指定時はディスク mmap、無指定は RAM。
fn make_sink(nstates: usize, dir: &SinkDir) -> Result<Box<dyn CompactSink + Send>, PlanError> {
    match dir {
        Some(dir) => crate::sink::MmapSink::new(dir, nstates)
            .map(|s| Box::new(s) as Box<dyn CompactSink + Send>)
            .map_err(|e| PlanError::Sink(e.to_string())),
        None => Ok(Box::new(RamSink::new(nstates))),
    }
}

impl PlannerCore {
    pub fn new(build: BuildParams, cfg: PlanConfig) -> Self {
        Self { build, cfg, cached: None, patch: None }
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

        let field = if self.cfg.use_compact() {
            // パッチは幾何だけで決まるので初回だけ作る (遷移表の再計算を避ける)。
            if self.patch.is_none() {
                self.patch = Some(new_patch(&self.build)?);
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

    /// compact (アウトオブコア) 経路: `states` を作らず地図とゴールから直接解き、
    /// 確定出力を sink に置く。solve は 1 回で走り切るので `solve_chunk` は使わず、
    /// `cancel` は `solve_compact_mapped` 内のラウンド境界で観測される。
    fn solve_compact(
        &self,
        goal: &PoseView,
        goal_t_deg: i32,
        stats: &mut SolveStats,
        cancel: &AtomicBool,
    ) -> Result<Field, PlanError> {
        let g = &self.build.grid;
        let nt = self.build.theta_cell_num;
        let nstates = g.width as usize * g.height as usize * nt as usize;
        let mut sink = make_sink(nstates, &self.cfg.compact_sink_dir)?;
        let nthreads =
            if self.cfg.vi_threads > 0 { self.cfg.vi_threads } else { default_threads() };

        let s = solve_compact_mapped(
            self.build.actions.clone(),
            1,
            g,
            nt,
            self.build.safety_radius,
            self.build.safety_radius_penalty,
            self.build.goal_margin_radius,
            self.build.goal_margin_theta,
            goal.x,
            goal.y,
            goal_t_deg,
            self.cfg.max_solve_iter,
            None,
            sink.as_mut(),
            nthreads,
            cancel,
        );
        stats.iters = s.iters;
        if s.cancelled {
            return Err(PlanError::Cancelled);
        }
        if !s.converged {
            return Err(PlanError::NotConverged);
        }
        Ok(Field::Compact(Box::new(CompactField {
            sink,
            actions: self.build.actions.clone(),
            cell_num: (g.width, g.height, nt),
            resolution: g.resolution,
            origin: (g.origin_x, g.origin_y),
            origin_quat: g.origin_quat.clone(),
            goal: (goal.x, goal.y, goal_t_deg),
        })))
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
        // sink (静的地図の解) を読む — モジュール冒頭の差異 1。
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
    /// 起こし直す (置き直した周期のローカル精密化は捨てられる — 冒頭の差異 2)。
    pub fn set_window(&mut self, pose: PoseView) {
        // build / cached / patch は別フィールドなので分割して借りられる。
        let build = &self.build;
        let patch = &mut self.patch;
        let Some(c) = self.cached.as_mut() else { return };
        match &mut c.field {
            Field::Dense(vi) => vi.set_local_window(pose.x, pose.y),
            Field::Compact(f) => {
                let Some(p) = patch.as_mut() else { return };
                let res = f.resolution;
                let gx = ((pose.x - f.origin.0) / res).floor() as i32;
                let gy = ((pose.y - f.origin.1) / res).floor() as i32;
                if p.needs_recenter(gx, gy) {
                    p.hydrate(f, build, (gx - p.half, gy - p.half));
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
    }

    /// ローカルウィンドウ内の価値反復を `budget` の範囲で回す (本家
    /// `localValueIterationWorker` の常駐スレッドを制御周期内の時間予算に
    /// 置き換えたもの)。1 パスの Δ 合計が 0 になったら予算を残して早期リターン
    /// する。戻り値は最後のパスの Δ 合計。
    pub fn refine_for(&mut self, budget: Duration) -> u64 {
        let t0 = Instant::now();
        loop {
            let (pass_delta, stopped) = self.refine_pass_until(|| t0.elapsed() >= budget);
            if stopped || pass_delta == 0 {
                return pass_delta;
            }
        }
    }

    /// ウィンドウ全体を `n` パス回す (決定的テスト用)。
    pub fn refine_passes(&mut self, n: usize) {
        for _ in 0..n {
            let _ = self.refine_pass_until(|| false);
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;
    use vi_reference::params::PROB_BASE_BIT;

    const RES: f64 = 0.05;

    fn actions() -> Vec<Action> {
        vec![
            Action::new("forward", 0.3, 0.0, 0),
            Action::new("back", -0.2, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2),
            Action::new("rightfw", 0.2, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4),
            Action::new("leftfw", 0.2, 20.0, 5),
        ]
    }

    fn build(size: i32) -> BuildParams {
        BuildParams {
            grid: OccupancyGrid {
                width: size,
                height: size,
                resolution: RES,
                origin_x: 0.0,
                origin_y: 0.0,
                origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                data: vec![0i8; (size * size) as usize],
            },
            actions: actions(),
            theta_cell_num: 60,
            safety_radius: 0.1,
            safety_radius_penalty: 30.0,
            goal_margin_radius: 0.2,
            goal_margin_theta: 180,
        }
    }

    fn cfg() -> PlanConfig {
        PlanConfig {
            solver: U64Solver::Frontier2DSparse,
            max_solve_iter: 100_000,
            solve_chunk: 16,
            goal_tolerance_xy: 0.25,
            goal_tolerance_deg: 10.0,
            max_rollout_steps: 10_000,
            start_tolerance_cells: 10,
            path_spacing: RES,
            action_tolerance_cells: 4,
            compact_sink_dir: None,
            vi_threads: 1,
        }
    }

    /// アウトオブコア経路 (states を作らない) の設定。
    fn cfg_compact() -> PlanConfig {
        PlanConfig { solver: U64Solver::Frontier2DSparseCompact { band: 0 }, ..cfg() }
    }

    fn pose(x: f64, y: f64, yaw: f64) -> PoseView {
        PoseView { x, y, yaw_rad: yaw }
    }

    /// この統合の本題: 広域 (plan) と狭域 (decide) が **1 回の solve** を
    /// 共有すること。旧構成 (vi_global_planner + vi_local_planner) では
    /// 同じゴールに対して別プロセスで 2 回解いていた。
    #[test]
    fn plan_and_follow_share_a_single_solve() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(false);
        let goal = pose(2.0, 2.0, 0.0);

        // 広域計画が最初の (そして唯一の) solve を走らせる。
        let (path, s1) = core.plan(pose(0.6, 0.6, 0.0), goal, &cancel).expect("plan");
        assert!(s1.solved_now && s1.iters > 0);
        assert!(path.len() > 2);

        // BT が同じゴールで FollowPath を送ってきた相当: solve は走らない。
        let s2 = core.prepare_goal(goal, &cancel).expect("prepare for follow");
        assert!(!s2.solved_now && s2.iters == 0, "follow must reuse the planner's solve");

        // その価値関数のまま追従判断が下せる。
        let robot = pose(0.6, 0.6, 0.0);
        core.set_window(robot);
        assert!(matches!(core.decide(robot), Decision::Action { .. }));

        // 1Hz リプラン相当も solve なし (ロールアウトのみ)。
        let (_, s3) = core.plan(pose(0.8, 0.9, 0.3), goal, &cancel).expect("replan");
        assert!(!s3.solved_now && s3.iters == 0);
    }

    /// decide → 行動適用 (並進→回転、no_noise_state_transition と同じ) を
    /// 繰り返してゴール圏へ到達できること = 制御ループの中核が閉じていること。
    #[test]
    fn follows_policy_to_goal_on_empty_map() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(false);
        let goal = pose(2.0, 2.0, 0.0);
        let stats = core.prepare_goal(goal, &cancel).expect("solve");
        assert!(stats.solved_now && stats.iters > 0);

        let (mut x, mut y, mut yaw) = (0.6f64, 0.6f64, 0.0f64);
        for _ in 0..500 {
            let p = pose(x, y, yaw);
            core.set_window(p);
            core.refine_passes(1);
            match core.decide(p) {
                Decision::Goal => {
                    let d = core.goal_distance(x, y).unwrap();
                    assert!(d <= 0.3, "goal margin: d = {d}");
                    return;
                }
                Decision::Action { fw, rot_deg, .. } => {
                    x += fw * yaw.cos();
                    y += fw * yaw.sin();
                    yaw += rot_deg.to_radians();
                }
                Decision::NoAction => panic!("no action at ({x:.2}, {y:.2})"),
            }
        }
        panic!("did not reach the goal in 500 steps");
    }

    /// **compact 経路の本題**: `states` を確保せずに解いた場でも、追従ループ
    /// (set_window → refine → decide) がゴール圏まで走り切れること。パッチは
    /// 走行中に何度も置き直される (`needs_recenter`)。
    #[test]
    fn compact_follows_policy_to_goal() {
        let mut core = PlannerCore::new(build(96), cfg_compact());
        let cancel = AtomicBool::new(false);
        let goal = pose(4.0, 4.0, 0.0);
        let stats = core.prepare_goal(goal, &cancel).expect("compact solve");
        assert!(stats.solved_now && stats.iters > 0);

        let (mut x, mut y, mut yaw) = (0.6f64, 0.6f64, 0.0f64);
        let mut hydrations = 0usize;
        let mut last_at = None;
        for _ in 0..500 {
            let p = pose(x, y, yaw);
            core.set_window(p);
            let at = core.patch.as_ref().and_then(|p| p.at);
            if at != last_at {
                hydrations += 1;
                last_at = at;
            }
            core.refine_passes(1);
            match core.decide(p) {
                Decision::Goal => {
                    let d = core.goal_distance(x, y).unwrap();
                    assert!(d <= 0.3, "goal margin: d = {d}");
                    assert!(hydrations > 1, "the patch must have moved along the way");
                    return;
                }
                Decision::Action { fw, rot_deg, .. } => {
                    x += fw * yaw.cos();
                    y += fw * yaw.sin();
                    yaw += rot_deg.to_radians();
                }
                Decision::NoAction => panic!("no action at ({x:.2}, {y:.2})"),
            }
        }
        panic!("did not reach the goal in 500 steps");
    }

    /// 上のテストは 0.05m セルなのでパッチ (99 セル角) が地図より大きく、凍結境界は
    /// ほぼ地図外の壁になる。こちらは **map_tsudanuma と同じ幾何** (0.25m セル、
    /// 歩幅 0.5m → パッチ 27 セル角) で、パッチが地図の内側に完全に収まったまま
    /// 何度も置き直される経路を走らせる。凍結境界の 4 辺すべてに compact の実値が
    /// 入る唯一のケースで、1 セルずれると「1.5m 走るごとに、ある方位でだけ NoAction」
    /// という気付きにくい壊れ方をする。
    #[test]
    fn compact_recenters_repeatedly_with_an_interior_patch() {
        // 120x120 @0.25m = 30m x 30m。
        let mut b = build(120);
        b.grid.resolution = 0.25;
        b.actions = vec![
            Action::new("forward", 0.5, 0.0, 0),
            Action::new("back", -0.3333, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2),
            Action::new("rightfw", 0.3333, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4),
            Action::new("leftfw", 0.3333, 20.0, 5),
        ];
        b.goal_margin_radius = 0.5; // セルサイズに比例 (overrides と同じ)
        b.safety_radius_penalty = 1.0;

        let mut core = PlannerCore::new(b, cfg_compact());
        let cancel = AtomicBool::new(false);
        let goal = pose(25.0, 25.0, 0.0);
        core.prepare_goal(goal, &cancel).expect("compact solve");

        let (mut x, mut y, mut yaw) = (4.0f64, 4.0f64, 0.0f64);
        let (mut hydrations, mut interior) = (0usize, 0usize);
        let mut last_at = None;
        for _ in 0..500 {
            let p = pose(x, y, yaw);
            core.set_window(p);
            let patch = core.patch.as_ref().unwrap();
            if patch.at != last_at {
                hydrations += 1;
                last_at = patch.at;
                let (p0x, p0y) = patch.at.unwrap();
                let side_max = 2 * patch.half;
                if p0x >= 0 && p0y >= 0 && p0x + side_max < 120 && p0y + side_max < 120 {
                    interior += 1;
                }
            }
            core.refine_passes(1);
            match core.decide(p) {
                Decision::Goal => {
                    assert!(hydrations >= 3, "patch must move several times: {hydrations}");
                    assert!(
                        interior >= 3,
                        "the patch must sit strictly inside the map at least a few times, \
                         so all four frozen edges carry real compact values: {interior}"
                    );
                    return;
                }
                Decision::Action { fw, rot_deg, .. } => {
                    x += fw * yaw.cos();
                    y += fw * yaw.sin();
                    yaw += rot_deg.to_radians();
                }
                Decision::NoAction => panic!(
                    "no action at ({x:.2}, {y:.2}) yaw={:.0}deg after {hydrations} hydrations",
                    yaw.to_degrees()
                ),
            }
        }
        panic!("did not reach the goal in 500 steps ({hydrations} hydrations)");
    }

    /// compact 経路の広域側が密経路と同じ経路を返すこと (ロールアウトは sink を読む)。
    #[test]
    fn compact_plan_matches_dense() {
        let cancel = AtomicBool::new(false);
        let goal = pose(2.0, 2.0, 0.0);
        let start = pose(0.6, 0.6, 0.0);

        let (dense, _) =
            PlannerCore::new(build(64), cfg()).plan(start, goal, &cancel).expect("dense plan");

        let mut core = PlannerCore::new(build(64), cfg_compact());
        let (p1, s1) = core.plan(start, goal, &cancel).expect("compact plan");
        assert!(s1.solved_now && s1.iters > 0);
        assert_eq!(p1.len(), dense.len(), "compact path length differs from dense");
        for (a, b) in p1.iter().zip(dense.iter()) {
            assert!(
                (a.x - b.x).abs() < 1e-12 && (a.y - b.y).abs() < 1e-12,
                "compact pose {a:?} != dense pose {b:?}"
            );
        }
        // 同一ゴールの再計画は solve なし (キャッシュヒット)。
        let (_, s2) = core.plan(pose(0.4, 1.8, 1.0), goal, &cancel).expect("compact replan");
        assert!(!s2.solved_now);
    }

    /// パッチのハイドレートが compact の場を忠実に写していること。
    /// ウィンドウ内の各セルについて、パッチの `(total_cost, optimal_action, free)` が
    /// sink / 静的地図と一致する = 凍結境界とローカル反復の前提。
    #[test]
    fn hydrated_patch_matches_the_compact_field() {
        let mut core = PlannerCore::new(build(64), cfg_compact());
        let cancel = AtomicBool::new(false);
        core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("compact solve");
        let robot = pose(1.0, 1.0, 0.0);
        core.set_window(robot);

        let p = core.patch.as_ref().expect("patch built");
        let (p0x, p0y) = p.at.expect("patch hydrated");
        // 遷移がパッチからはみ出さないこと (凍結境界の成立条件)。
        assert!(p.vi.local_ixy_range + p.reach < p.half);

        let Some(CachedGoal { field: Field::Compact(f), .. }) = core.cached.as_ref() else {
            panic!("compact field expected");
        };
        let nt = f.cell_num.2;
        let mut checked = 0usize;
        for py in 0..=(2 * p.half) {
            for px in 0..=(2 * p.half) {
                let (gx, gy) = (p0x + px, p0y + py);
                if gx < 0 || gy < 0 || gx >= f.cell_num.0 || gy >= f.cell_num.1 {
                    continue;
                }
                for it in [0, nt / 3, nt - 1] {
                    let (v, a) = f.sink.read(f.orig(gx, gy, it));
                    let s = &p.vi.base.states[p.vi.base.to_index(px, py, it) as usize];
                    assert_eq!(s.total_cost, v, "value at ({gx},{gy},{it})");
                    assert_eq!(
                        s.optimal_action,
                        if a >= 0 { Some(a as usize) } else { None },
                        "policy at ({gx},{gy},{it})"
                    );
                    assert_eq!(s.final_state, v == 0, "final_state at ({gx},{gy},{it})");
                    assert!(s.free, "empty map: every in-map cell is free");
                    checked += 1;
                }
            }
        }
        assert!(checked > 100, "the patch must overlap the map");
    }

    /// ディスク mmap sink 経由でも追従判断が出ること (Pi4 のような小メモリ機の経路)。
    #[test]
    fn compact_with_mmap_sink_follows() {
        let dir = std::env::temp_dir().join("vi_planner_core_mmap_test");
        let cfg = PlanConfig { compact_sink_dir: Some(dir.clone()), ..cfg_compact() };
        let mut core = PlannerCore::new(build(64), cfg);
        let cancel = AtomicBool::new(false);
        core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");
        let robot = pose(0.6, 0.6, 0.0);
        core.set_window(robot);
        assert!(matches!(core.decide(robot), Decision::Action { .. }));
        drop(core);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn plans_and_caches_for_same_goal() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(false);
        let goal = pose(2.0, 2.0, 0.0);

        let (p1, s1) = core.plan(pose(0.6, 0.6, 0.0), goal, &cancel).expect("first plan");
        assert!(s1.solved_now && s1.iters > 0);
        assert!(p1.len() > 2);

        let (p2, s2) = core.plan(pose(0.4, 1.8, 1.0), goal, &cancel).expect("replan");
        assert!(!s2.solved_now && s2.iters == 0);
        assert!(p2.len() > 2);

        let (_, s3) =
            core.plan(pose(0.6, 0.6, 0.0), pose(0.8, 2.4, 0.0), &cancel).expect("new goal");
        assert!(s3.solved_now);
    }

    /// `is_cached_goal` は追従スレッドの世代チェックの土台。別ゴールの計画で
    /// キャッシュが差し替わったら false になること。
    #[test]
    fn is_cached_goal_tracks_the_replacement() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(false);
        let followed = pose(2.0, 2.0, 0.0);

        assert!(!core.is_cached_goal(followed), "nothing solved yet");
        core.prepare_goal(followed, &cancel).expect("solve");
        assert!(core.is_cached_goal(followed));
        // 許容差内のゆらぎは同一ゴール扱い。
        assert!(core.is_cached_goal(pose(2.05, 2.0, 0.0)));

        // 広域側が別ゴールを解くとキャッシュが差し替わる。
        core.plan(pose(0.6, 0.6, 0.0), pose(0.8, 2.4, 0.0), &cancel).expect("new goal");
        assert!(!core.is_cached_goal(followed));
    }

    /// ゴールが変わったらパッチは無効化され、次の `set_window` で新しい場から
    /// 起こし直されること (古いゴールの方策で走らせない)。
    #[test]
    fn compact_patch_is_invalidated_on_a_new_goal() {
        let mut core = PlannerCore::new(build(64), cfg_compact());
        let cancel = AtomicBool::new(false);
        let robot = pose(0.6, 0.6, 0.0);

        core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");
        core.set_window(robot);
        assert!(core.patch.as_ref().unwrap().at.is_some());

        core.prepare_goal(pose(0.8, 2.4, 0.0), &cancel).expect("new goal");
        assert!(core.patch.as_ref().unwrap().at.is_none(), "stale patch must be dropped");
        assert_eq!(core.decide(robot), Decision::NoAction, "no policy before set_window");
        core.set_window(robot);
        assert!(matches!(core.decide(robot), Decision::Action { .. }));
    }

    #[test]
    fn densified_path_spacing_bounded() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(false);
        let (p, _) = core.plan(pose(0.6, 0.6, 0.0), pose(2.0, 2.0, 0.0), &cancel).unwrap();
        for w in p.windows(2) {
            let d = ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt();
            assert!(d <= RES + 1e-9);
        }
    }

    #[test]
    fn pre_raised_cancel_aborts_solve() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(true);
        assert_eq!(
            core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).unwrap_err(),
            PlanError::Cancelled
        );
        let mut core = PlannerCore::new(build(64), cfg());
        assert_eq!(
            core.plan(pose(0.6, 0.6, 0.0), pose(2.0, 2.0, 0.0), &cancel).unwrap_err(),
            PlanError::Cancelled
        );
        // compact 経路も同じ (中断はソルバ内部のラウンド境界で観測される)。
        let mut core = PlannerCore::new(build(64), cfg_compact());
        assert_eq!(
            core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).unwrap_err(),
            PlanError::Cancelled
        );
    }

    /// 進捗コールバックは新規 solve でのみ発火し、キャッシュヒットでは発火しない
    /// (途中経過の value_function 配信の前提)。
    #[test]
    fn progress_callback_fires_only_when_solving() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(false);
        let goal = pose(2.0, 2.0, 0.0);

        let mut calls = 0usize;
        core.plan_with_progress(pose(0.6, 0.6, 0.0), goal, &cancel, &mut |vi| {
            calls += 1;
            assert!(vi.cell_num_x > 0);
        })
        .expect("first plan");
        assert!(calls > 0);

        let mut calls_cached = 0usize;
        core.prepare_goal_with_progress(goal, &cancel, &mut |_| calls_cached += 1)
            .expect("follow");
        assert_eq!(calls_cached, 0);
    }

    /// value_grid は全域、window_value_grid はクランプ後の実ウィンドウと
    /// 寸法・原点・データ長が一致し、値は OccupancyGrid の規約 (-1..=100) に収まる。
    #[test]
    fn visualization_grids_match_geometry() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(false);
        core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");

        let g = core.value_grid(60).expect("full grid");
        assert_eq!((g.width, g.height), (64, 64));
        assert_eq!(g.data.len(), 64 * 64);

        // 地図端 (原点) にウィンドウ → min 側がクランプされ 21x21
        // (local_ixy_range = 1.0m / 0.05m = 20)。
        let robot = pose(0.0, 0.0, 0.0);
        core.set_window(robot);
        let w = core.window_value_grid(robot, 60).expect("window grid");
        assert_eq!((w.width, w.height), (21, 21));
        assert_eq!(w.data.len(), (w.width * w.height) as usize);
        assert_eq!((w.origin_x, w.origin_y), (0.0, 0.0));
        assert!(w.data.iter().all(|&v| (-1..=100).contains(&i32::from(v))));

        let empty = PlannerCore::new(build(64), cfg());
        assert!(empty.value_grid(60).is_none());
        assert!(empty.window_value_grid(robot, 60).is_none());
    }

    /// compact 経路でも可視化グリッドが同じ幾何で出ること (全域は sink 走査)。
    #[test]
    fn compact_visualization_grids_match_geometry() {
        let mut core = PlannerCore::new(build(64), cfg_compact());
        let cancel = AtomicBool::new(false);
        core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");

        let g = core.value_grid(60).expect("full grid");
        assert_eq!((g.width, g.height), (64, 64));
        assert_eq!(g.data.len(), 64 * 64);

        let robot = pose(1.0, 1.0, 0.0);
        core.set_window(robot);
        let w = core.window_value_grid(robot, 60).expect("window grid");
        // ウィンドウはパッチの内側に丸ごと収まるのでクランプされない
        // (local_ixy_range = 1.0m / 0.05m = 20 → 41x41)。密経路で地図端に寄せたとき
        // だけ 21x21 にクランプされる (visualization_grids_match_geometry 参照)。
        assert_eq!((w.width, w.height), (41, 41));
        assert!(w.data.iter().all(|&v| (-1..=100).contains(&i32::from(v))));
    }

    /// スキャンで注入された local_penalty が、ローカル反復を経て「ヒット帯へ
    /// 踏み込む行動を持つ上流セル」の価値を引き上げること (障害物回避の根拠)。
    #[test]
    fn scan_penalty_raises_upstream_value() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(false);
        // ゴールは東 (x 正方向) の先。前進が最短。
        core.prepare_goal(pose(2.5, 1.0, 0.0), &cancel).expect("solve");

        let robot = pose(1.0, 1.0, 0.0);
        core.set_window(robot);

        // 前進 1 ステップ (0.3m) でヒット帯 (1.5, 1.0)±2 セルに着地する上流セル。
        let (uix, uiy, uit) = {
            let vi = core.local().unwrap();
            pose_to_cell(&vi.base, 1.2, 1.0, 0.0)
        };
        let before = {
            let vi = core.local().unwrap();
            vi.base.states[vi.base.to_index(uix, uiy, uit) as usize].total_cost
        };

        // 正面 0.5m にヒット 1 ビーム → (1.5, 1.0) 周辺へ 2048<<PROB_BASE_BIT。
        let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
        core.observe_scan(&scan, robot);
        {
            let vi = core.local().unwrap();
            let (hx, hy, _) = pose_to_cell(&vi.base, 1.5, 1.0, 0.0);
            let hit = vi.base.to_index(hx, hy, 0) as usize;
            assert_eq!(vi.base.states[hit].local_penalty, 2048u64 << PROB_BASE_BIT);
        }

        core.refine_passes(5);
        let after = {
            let vi = core.local().unwrap();
            vi.base.states[vi.base.to_index(uix, uiy, uit) as usize].total_cost
        };
        assert!(after > before, "upstream value must rise: before={before}, after={after}");
    }

    /// 同じことが compact 経路のパッチ上でも成り立つこと (狭域が compact の場の
    /// 上でも障害物回避として機能する = この実装の目的)。
    #[test]
    fn compact_scan_penalty_raises_upstream_value() {
        let mut core = PlannerCore::new(build(64), cfg_compact());
        let cancel = AtomicBool::new(false);
        core.prepare_goal(pose(2.5, 1.0, 0.0), &cancel).expect("solve");

        let robot = pose(1.0, 1.0, 0.0);
        core.set_window(robot);

        let (uix, uiy, uit) = {
            let vi = core.local().unwrap();
            pose_to_cell(&vi.base, 1.2, 1.0, 0.0)
        };
        let before = {
            let vi = core.local().unwrap();
            vi.base.states[vi.base.to_index(uix, uiy, uit) as usize].total_cost
        };
        assert!(before < MAX_COST, "the patch must carry the compact solution");

        let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
        core.observe_scan(&scan, robot);
        core.refine_passes(5);
        let after = {
            let vi = core.local().unwrap();
            vi.base.states[vi.base.to_index(uix, uiy, uit) as usize].total_cost
        };
        assert!(after > before, "upstream value must rise: before={before}, after={after}");
    }

    /// 障害物・ペナルティ変化の無いウィンドウは 1 パスで Δ=0 になり、
    /// refine_for が予算を使い切らず早期リターンすること。
    #[test]
    fn refine_early_exits_when_window_converged() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(false);
        core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");
        core.set_window(pose(1.0, 1.0, 0.0));
        core.refine_passes(2); // 念のため均しておく
        let t0 = Instant::now();
        let delta = core.refine_for(Duration::from_secs(10));
        assert_eq!(delta, 0);
        assert!(t0.elapsed() < Duration::from_secs(5), "must not burn the whole budget");
    }

    /// 方策なしセル (障害物の膨張内) からの近傍救済。
    #[test]
    fn decide_borrows_action_from_neighbors() {
        let size = 64;
        let mut b = build(size);
        // (20, 20) 付近に小さな障害物ブロック。
        for y in 18..=22 {
            for x in 18..=22 {
                b.grid.data[(y * size + x) as usize] = 100;
            }
        }
        let mut core = PlannerCore::new(b, cfg());
        let cancel = AtomicBool::new(false);
        core.prepare_goal(pose(2.5, 2.5, 0.0), &cancel).expect("solve");

        let on_obstacle = pose(20.5 * RES, 20.5 * RES, 0.0);
        // tolerance 0 だと NoAction。
        let mut strict = cfg();
        strict.action_tolerance_cells = 0;
        let strict_core = PlannerCore {
            build: core.build.clone(),
            cfg: strict,
            cached: core.cached.take(),
            patch: core.patch.take(),
        };
        assert_eq!(strict_core.decide(on_obstacle), Decision::NoAction);
        // tolerance 4 (0.2m) なら近傍の行動を借りられる。
        let relaxed_core = PlannerCore {
            build: strict_core.build.clone(),
            cfg: cfg(),
            cached: strict_core.cached,
            patch: strict_core.patch,
        };
        assert!(matches!(relaxed_core.decide(on_obstacle), Decision::Action { .. }));
    }

    #[test]
    fn unreachable_goal_fails_rollout() {
        // ゴール周辺だけ厚壁で囲む (中は free のままゴールを置く)。
        let size = 64;
        let mut b = build(size);
        for y in 32..size {
            for x in 40..48 {
                b.grid.data[(y * size + x) as usize] = 100;
            }
        }
        for y in 40..48 {
            for x in 40..size {
                b.grid.data[(y * size + x) as usize] = 100;
            }
        }
        let mut core = PlannerCore::new(b, cfg());
        let cancel = AtomicBool::new(false);
        let err = core.plan(pose(0.6, 0.6, 0.0), pose(2.8, 2.8, 0.0), &cancel).unwrap_err();
        assert!(matches!(err, PlanError::Rollout(RolloutStatus::NoAction)), "{err:?}");
    }

    /// パッチの寸法は「ウィンドウ + 遷移到達距離」を必ず超えること
    /// (凍結境界が成り立つ条件そのもの)。粗いセルでも成り立つ。
    #[test]
    fn patch_geometry_covers_the_transition_reach() {
        for res in [0.05, 0.15, 0.25, 0.5] {
            let mut b = build(8);
            b.grid.resolution = res;
            // 津田沼構成と同じく歩幅をセルサイズに比例させた場合も見る。
            let k = res / 0.05;
            b.actions = actions()
                .into_iter()
                .enumerate()
                .map(|(i, a)| Action::new(&a.name, a.delta_fw * k, a.delta_rot, i as i32))
                .collect();
            let p = new_patch(&b).unwrap_or_else(|e| panic!("res={res}: {e}"));
            assert!(
                p.vi.local_ixy_range + p.reach < p.half,
                "res={res}: window {} + reach {} must fit in half {}",
                p.vi.local_ixy_range,
                p.reach,
                p.half
            );
        }
    }
}
