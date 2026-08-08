//! 収束済み `ValueIterator` の方策から経路 (世界座標の姿勢列) を生成するプランナ層。
//!
//! ソルバ非依存: `solvers::solve` がどの `U64Solver` でも、収束後は
//! `states[i].optimal_action` に方策が書き戻されている (fused/sparse 系は
//! `write_back_fused` が、sweep 系は `value_iteration_raw` が書く) ことを前提に、
//! 貪欲ロールアウトで方策を辿る。
//!
//! 動作モデルは本家の実行系と同一:
//! - 世界座標→セルの変換は本家 `posToAction` の式 (度は i32 切り捨て、`+360*100` で正規化)
//! - 1 ステップの変位は `no_noise_state_transition` と同じ「現在向きで delta_fw 並進 →
//!   delta_rot 回転」
//!
//! `rollout_path_on` は読み取り専用 API (本家 `posToAction` と違い `status` を
//! 書き換えない) ので、solve 済みのイテレータを複数ゴール間で使い回す
//! プランナサーバから安全に呼べる。

use std::collections::HashMap;
use std::f64::consts::PI;

use crate::params::MAX_COST;
use crate::solvers::frontier2d_sparse_compact::CompactSink;
use crate::value_iterator::ValueIterator;

/// ロールアウト終了理由。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RolloutStatus {
    /// ゴール領域 (`final_state`) に到達。`poses` の末尾はゴール姿勢そのもの。
    ReachedGoal,
    /// 方策が無い (未到達セル / 障害物セル / ゴール不能)。
    NoAction,
    /// 地図外に出た。
    OutOfMap,
    /// 同一離散状態への再訪が閾値を超えた (方策の巡回)。
    LoopDetected,
    /// `max_steps` を使い切った。
    StepLimit,
}

/// 経路上の 1 姿勢 (世界座標)。`yaw` はラジアン。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathPose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

/// ロールアウト結果。`status == ReachedGoal` 以外でも、それまでに辿れた
/// `poses` は診断用に返す (経路としては不完全)。
#[derive(Clone, Debug)]
pub struct Rollout {
    pub poses: Vec<PathPose>,
    pub status: RolloutStatus,
}

impl Rollout {
    pub fn reached_goal(&self) -> bool {
        self.status == RolloutStatus::ReachedGoal
    }
}

/// 同一離散状態の再訪許容回数。連続空間ロールアウトではセル内オフセットが
/// 毎回異なるため 1 度の再訪は巡回と断定できないが、これを超えたら打ち切る。
const REVISIT_LIMIT: u32 = 8;

/// ロールアウトが読む「解けた価値関数」の最小ビュー。
///
/// `ValueIterator`（密な `states` を持つ通常経路）と、`solve_compact_mapped` の出力
/// `CompactSink`（states を作らないアウトオブコア経路）の両方を同じロールアウトで辿るための
/// 抽象。geometry とゴールはビュー側が持ち、per-state に必要なのは「ゴール圏か」「最適行動は何か」
/// の 2 つだけ。
pub trait PolicyView {
    /// `(cell_num_x, cell_num_y, cell_num_t)`。
    fn cell_num(&self) -> (i32, i32, i32);
    fn xy_resolution(&self) -> f64;
    /// `(map_origin_x, map_origin_y)`。
    fn map_origin(&self) -> (f64, f64);
    /// `(goal_x, goal_y, goal_t[deg])`。
    fn goal(&self) -> (f64, f64, i32);
    /// `(ix,iy,it)` がゴール圏 (`final_state`) か。範囲外は false。
    fn is_final(&self, ix: i32, iy: i32, it: i32) -> bool;
    /// `(ix,iy,it)` の最適行動 index。範囲外 / 非 free / final / 方策なしは `None`。
    fn action_index(&self, ix: i32, iy: i32, it: i32) -> Option<usize>;
    /// 行動 index → `(delta_fw[m], delta_rot[deg])`。
    fn action_delta(&self, ai: usize) -> (f64, f64);
    /// `(ix,iy,it)` の値 (`total_cost`)。範囲外・未到達は `MAX_COST`。solve 途中の
    /// 観測 (value_function 可視化・上界不変条件の検証) が同じビューで値も読めるように
    /// するための口。
    fn value_at(&self, ix: i32, iy: i32, it: i32) -> u64;
    /// 地図原点の姿勢 (可視化グリッド生成用)。既定は単位クォータニオン —
    /// 回転付き地図を扱うビューは override すること (`ValueIterator` は地図から、
    /// `CompactPolicy` は `with_origin_quat` で与える)。
    fn map_origin_quat(&self) -> crate::msg::Quaternion {
        crate::msg::Quaternion::default()
    }

    /// 本家 `t_resolution_ = 360/cell_num_t_`（整数除算後に f64 化）。
    fn t_resolution(&self) -> f64 {
        (360 / self.cell_num().2) as f64
    }
    fn in_map_area(&self, ix: i32, iy: i32) -> bool {
        let (nx, ny, _) = self.cell_num();
        ix >= 0 && ix < nx && iy >= 0 && iy < ny
    }
}

impl PolicyView for ValueIterator {
    fn cell_num(&self) -> (i32, i32, i32) {
        (self.cell_num_x, self.cell_num_y, self.cell_num_t)
    }
    fn xy_resolution(&self) -> f64 {
        self.xy_resolution
    }
    fn map_origin(&self) -> (f64, f64) {
        (self.map_origin_x, self.map_origin_y)
    }
    fn goal(&self) -> (f64, f64, i32) {
        (self.goal_x, self.goal_y, self.goal_t)
    }
    fn t_resolution(&self) -> f64 {
        self.t_resolution
    }
    fn is_final(&self, ix: i32, iy: i32, it: i32) -> bool {
        self.in_map_area(ix, iy)
            && it >= 0
            && it < self.cell_num_t
            && self.states[self.to_index(ix, iy, it) as usize].final_state
    }
    fn action_index(&self, ix: i32, iy: i32, it: i32) -> Option<usize> {
        if !PolicyView::in_map_area(self, ix, iy) || it < 0 || it >= self.cell_num_t {
            return None;
        }
        let s = &self.states[self.to_index(ix, iy, it) as usize];
        if !s.free || s.final_state {
            return None;
        }
        s.optimal_action
    }
    fn action_delta(&self, ai: usize) -> (f64, f64) {
        let a = &self.actions[ai];
        (a.delta_fw, a.delta_rot)
    }
    fn value_at(&self, ix: i32, iy: i32, it: i32) -> u64 {
        if !PolicyView::in_map_area(self, ix, iy) || it < 0 || it >= self.cell_num_t {
            return MAX_COST;
        }
        self.states[self.to_index(ix, iy, it) as usize].total_cost
    }
    fn map_origin_quat(&self) -> crate::msg::Quaternion {
        self.map_origin_quat.clone()
    }
}

/// `solve_compact_mapped` が確定した `CompactSink` を方策ビューとして読む。
///
/// sink は orig 索引 `it + ix·nt + iy·nt·nx` に `(total_cost, action)` を持ち、
/// - 未到達 (障害物 / ゴールから到達不能) = `(MAX_COST, -1)`
/// - ゴール圏 (`final_state`) = `(0, -1)`   ← `set_state_values` が値を 0 にピン留めするため
/// - それ以外の到達セル = `(cost, action>=0)`
///
/// なので `final_state` の再計算 (距離+向き判定) は不要で、`value == 0` が一意にゴール圏を表す。
/// 津田沼地図 (1963x1334x60) で実測確認済み: `value == 0` は 28 状態のみ、いずれも action = -1、
/// 到達状態 41,879,880 のうち action < 0 はその 28 状態だけ。
pub struct CompactPolicy<'a> {
    sink: &'a dyn CompactSink,
    actions: &'a [crate::action::Action],
    cell_num_x: i32,
    cell_num_y: i32,
    cell_num_t: i32,
    xy_resolution: f64,
    map_origin_x: f64,
    map_origin_y: f64,
    goal_x: f64,
    goal_y: f64,
    goal_t: i32,
    /// 地図原点の姿勢 (可視化用)。`new` では単位クォータニオン、
    /// 回転付き地図は [`CompactPolicy::with_origin_quat`] で与える。
    origin_quat: crate::msg::Quaternion,
}

impl<'a> CompactPolicy<'a> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        sink: &'a dyn CompactSink,
        actions: &'a [crate::action::Action],
        cell_num: (i32, i32, i32),
        xy_resolution: f64,
        map_origin: (f64, f64),
        goal: (f64, f64, i32),
    ) -> Self {
        Self {
            sink,
            actions,
            cell_num_x: cell_num.0,
            cell_num_y: cell_num.1,
            cell_num_t: cell_num.2,
            xy_resolution,
            map_origin_x: map_origin.0,
            map_origin_y: map_origin.1,
            goal_x: goal.0,
            goal_y: goal.1,
            goal_t: goal.2,
            origin_quat: crate::msg::Quaternion::default(),
        }
    }

    /// 地図原点の姿勢を与える (可視化グリッド生成用)。
    pub fn with_origin_quat(mut self, q: crate::msg::Quaternion) -> Self {
        self.origin_quat = q;
        self
    }

    /// orig 索引 (usize 演算。巨大マップで i32 が溢れるため)。
    #[inline]
    fn orig(&self, ix: i32, iy: i32, it: i32) -> usize {
        it as usize
            + ix as usize * self.cell_num_t as usize
            + iy as usize * self.cell_num_x as usize * self.cell_num_t as usize
    }

    /// `(ix,iy,it)` の `(total_cost, action)`。範囲外は未到達扱い。
    #[inline]
    fn read(&self, ix: i32, iy: i32, it: i32) -> (u64, i32) {
        if !self.in_map_area(ix, iy) || it < 0 || it >= self.cell_num_t {
            return (MAX_COST, -1);
        }
        self.sink.read(self.orig(ix, iy, it))
    }
}

impl PolicyView for CompactPolicy<'_> {
    fn cell_num(&self) -> (i32, i32, i32) {
        (self.cell_num_x, self.cell_num_y, self.cell_num_t)
    }
    fn xy_resolution(&self) -> f64 {
        self.xy_resolution
    }
    fn map_origin(&self) -> (f64, f64) {
        (self.map_origin_x, self.map_origin_y)
    }
    fn goal(&self) -> (f64, f64, i32) {
        (self.goal_x, self.goal_y, self.goal_t)
    }
    fn is_final(&self, ix: i32, iy: i32, it: i32) -> bool {
        self.read(ix, iy, it).0 == 0
    }
    fn action_index(&self, ix: i32, iy: i32, it: i32) -> Option<usize> {
        let (v, a) = self.read(ix, iy, it);
        if v == 0 || v >= MAX_COST || a < 0 {
            return None; // ゴール圏 / 未到達 / 方策なし。
        }
        Some(a as usize)
    }
    fn action_delta(&self, ai: usize) -> (f64, f64) {
        let a = &self.actions[ai];
        (a.delta_fw, a.delta_rot)
    }
    fn value_at(&self, ix: i32, iy: i32, it: i32) -> u64 {
        self.read(ix, iy, it).0
    }
    fn map_origin_quat(&self) -> crate::msg::Quaternion {
        self.origin_quat.clone()
    }
}

/// `pose_to_cell` の `PolicyView` 版（式は本家 `posToAction` と同一）。
pub fn pose_to_cell_on(p: &dyn PolicyView, x: f64, y: f64, yaw_rad: f64) -> (i32, i32, i32) {
    let (ox, oy) = p.map_origin();
    let res = p.xy_resolution();
    let ix = ((x - ox) / res).floor() as i32;
    let iy = ((y - oy) / res).floor() as i32;
    let t = (180.0 * yaw_rad / PI) as i32;
    let it = (((t + 360 * 100) % 360) as f64 / p.t_resolution()).floor() as i32;
    (ix, iy, it)
}

/// 世界座標をセル (ix, iy, it) へ変換する。本家 `posToAction` の変換式を逐語再現
/// (度は i32 切り捨て、`+360*100 (mod 360)` 正規化、`/t_resolution` の floor)。
/// 範囲チェックはしない (呼び出し側で `in_map_area` / it 範囲を確認すること)。
pub fn pose_to_cell(vi: &ValueIterator, x: f64, y: f64, yaw_rad: f64) -> (i32, i32, i32) {
    pose_to_cell_on(vi, x, y, yaw_rad)
}

/// `(ix, iy, it)` の最適行動 id。範囲外 / 非 free / final_state / 方策なしは -1。
/// (vi_node の sweep_thread から移設した読み取り専用ヘルパ。)
pub fn optimal_action_at(vi: &ValueIterator, ix: i32, iy: i32, it: i32) -> i32 {
    if ix < 0 || iy < 0 || it < 0 || ix >= vi.cell_num_x || iy >= vi.cell_num_y
        || it >= vi.cell_num_t
    {
        return -1;
    }
    let s = &vi.states[vi.to_index(ix, iy, it) as usize];
    if !s.free || s.final_state {
        return -1;
    }
    match s.optimal_action {
        Some(ai) => vi.actions[ai].id,
        None => -1,
    }
}

/// start が方策を持たないセルに落ちた場合に、同一 θ で xy 近傍 (チェビシェフ距離
/// `tolerance_cells` 以内) から方策を持つ最近傍セルを探す。ロボットが膨張領域や
/// 未知セルの縁に僅かに掛かった状態からでも計画できるようにするための救済。
/// 戻り値はセル中心の世界座標。見つからなければ None。
fn find_plannable_start(
    p: &dyn PolicyView,
    x: f64,
    y: f64,
    yaw_rad: f64,
    tolerance_cells: i32,
) -> Option<(f64, f64)> {
    let (ix0, iy0, it) = pose_to_cell_on(p, x, y, yaw_rad);
    let mut best: Option<(i64, i32, i32)> = None;
    for dy in -tolerance_cells..=tolerance_cells {
        for dx in -tolerance_cells..=tolerance_cells {
            let (ix, iy) = (ix0 + dx, iy0 + dy);
            if p.action_index(ix, iy, it).is_none() {
                // final_state セル (既にゴール圏内) も救済対象に含める。
                if !p.is_final(ix, iy, it) {
                    continue;
                }
            }
            let d2 = (dx as i64) * (dx as i64) + (dy as i64) * (dy as i64);
            if best.map(|(bd, _, _)| d2 < bd).unwrap_or(true) {
                best = Some((d2, ix, iy));
            }
        }
    }
    let (ox, oy) = p.map_origin();
    let res = p.xy_resolution();
    best.map(|(_, ix, iy)| ((ix as f64 + 0.5) * res + ox, (iy as f64 + 0.5) * res + oy))
}

/// 貪欲方策ロールアウト。solve 済み (`set_goal` 後に収束させた) 場の方策を start
/// 姿勢から辿り、世界座標の姿勢列を返す。密な `ValueIterator` でも compact sink
/// ([`CompactPolicy`]) でも同じ経路生成を行う (どちらも同じ方策・同じ遷移モデル)。
///
/// - 1 ステップ = 現在セルの最適行動を 1 回適用 (`no_noise_state_transition` と同じ
///   「並進→回転」)。姿勢間隔はアクション設計 (既定 0.2–0.3 m / 20°) に従う。
///   Nav2 等で密な経路が要る場合は [`densify`] を併用する。
/// - `final_state` セルに入った時点で成功とし、末尾に正確なゴール姿勢
///   (`goal_x`, `goal_y`, `goal_t`) を追加する (goal_margin ぶん手前で経路が
///   途切れないようにするため)。
/// - `start_tolerance_cells > 0` なら、start が方策を持たないセルのとき同一 θ の
///   xy 近傍から計画可能な最近傍セル中心へスナップして開始する。
pub fn rollout_path_on(
    p: &dyn PolicyView,
    start_x: f64,
    start_y: f64,
    start_yaw_rad: f64,
    max_steps: usize,
    start_tolerance_cells: i32,
) -> Rollout {
    let (mut x, mut y) = (start_x, start_y);
    // 内部では度で保持 (本家の遷移生成・セル変換が度基準のため)。
    let mut yaw_deg = normalize_deg(start_yaw_rad.to_degrees());
    let (nt, goal) = (p.cell_num().2, p.goal());

    // start 救済: 現セルに方策が無ければ近傍の計画可能セル中心へスナップ。
    {
        let (ix, iy, it) = pose_to_cell_on(p, x, y, start_yaw_rad);
        let on_final = p.is_final(ix, iy, it);
        if !on_final && p.action_index(ix, iy, it).is_none() && start_tolerance_cells > 0 {
            if let Some((sx, sy)) =
                find_plannable_start(p, x, y, start_yaw_rad, start_tolerance_cells)
            {
                x = sx;
                y = sy;
            }
        }
    }

    let mut poses = vec![PathPose { x, y, yaw: yaw_deg.to_radians() }];
    let mut visits: HashMap<(i32, i32, i32), u32> = HashMap::new();

    for _ in 0..max_steps {
        let (ix, iy, it) = pose_to_cell_on(p, x, y, yaw_deg.to_radians());
        if !p.in_map_area(ix, iy) || it < 0 || it >= nt {
            return Rollout { poses, status: RolloutStatus::OutOfMap };
        }
        if p.is_final(ix, iy, it) {
            // ゴール圏に入った: 末尾を正確なゴール姿勢で締める。
            poses.push(PathPose { x: goal.0, y: goal.1, yaw: (goal.2 as f64).to_radians() });
            return Rollout { poses, status: RolloutStatus::ReachedGoal };
        }
        let Some(ai) = p.action_index(ix, iy, it) else {
            return Rollout { poses, status: RolloutStatus::NoAction };
        };

        let count = visits.entry((ix, iy, it)).or_insert(0);
        *count += 1;
        if *count > REVISIT_LIMIT {
            return Rollout { poses, status: RolloutStatus::LoopDetected };
        }

        // no_noise_state_transition と同じ: 現在向きで並進 → 回転。
        let (delta_fw, delta_rot) = p.action_delta(ai);
        let ang = yaw_deg.to_radians();
        x += delta_fw * ang.cos();
        y += delta_fw * ang.sin();
        yaw_deg = normalize_deg(yaw_deg + delta_rot);
        poses.push(PathPose { x, y, yaw: yaw_deg.to_radians() });
    }

    Rollout { poses, status: RolloutStatus::StepLimit }
}

/// 姿勢列を最大間隔 `max_spacing` (m) で線形補間して密にする。yaw は区間の
/// 始点値を引き継ぎ、各元姿勢は必ず保持される。Nav2 の経路追従器 (DWB 等) が
/// セル解像度並みの点列を期待する場合に使う。
pub fn densify(poses: &[PathPose], max_spacing: f64) -> Vec<PathPose> {
    if poses.len() < 2 || max_spacing <= 0.0 {
        return poses.to_vec();
    }
    let mut out = Vec::with_capacity(poses.len());
    for w in poses.windows(2) {
        let (p, q) = (w[0], w[1]);
        out.push(p);
        let dist = ((q.x - p.x).powi(2) + (q.y - p.y).powi(2)).sqrt();
        if dist > max_spacing {
            let n = (dist / max_spacing).ceil() as usize;
            for k in 1..n {
                let r = k as f64 / n as f64;
                out.push(PathPose {
                    x: p.x + (q.x - p.x) * r,
                    y: p.y + (q.y - p.y) * r,
                    yaw: p.yaw,
                });
            }
        }
    }
    out.push(*poses.last().unwrap());
    out
}

/// 度を [0, 360) へ正規化。
fn normalize_deg(d: f64) -> f64 {
    d.rem_euclid(360.0)
}

// ═══ QMDP — belief 加重の行動選択 ═══

/// [`qmdp_decide`] の拒否権しきい値: 勝った行動でも「衝突/場外 (Q = MAX_COST)」と
/// 言う仮説の質量比がこれを超えたら走らない (None = 停止)。仮説間で行動が割れる
/// 場面で期待値だけ見ると、有意な仮説の下で衝突する行動を選び得るための安全弁。
// ponytail: 定数 1 個。地図・センサごとの調整が要るなら PlanConfig へ昇格。
pub const QMDP_VETO_MASS: f64 = 0.2;

/// [`qmdp_decide`] の結果。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QmdpDecision {
    /// 評価できた質量の過半がゴール圏 (`final_state`)。
    Goal,
    /// `vi.actions` への添字。
    Action(usize),
    /// 評価できる仮説が無い / 全行動が拒否権で落ちた。呼び出し側は停止。
    NoAction,
}

/// QMDP 行動選択: belief の仮説集合 `{(pose, w)}` で Q(b,a) = Σᵢ wᵢ·Q(sᵢ,a) を
/// 評価し、argmin の行動を返す。Q(s,a) は本家 `actionCost` (遷移テーブル期待値 —
/// solve と同じ [`action_cost_raw`]) なので、方策 (`optimal_action`) が無いセルの
/// 仮説も価値関数から直接評価できる (greedy の近傍借用は不要)。
///
/// - 重み `w` は非正規化でよい (質量比しか使わない)。
/// - 場外 / 非 free (VI 解像度で壁の中) の仮説は評価対象外 — compact 経路の
///   パッチで呼ぶときは「パッチ外の仮説は無視される」ことを意味する。
/// - Q = MAX_COST の項はそのまま和に入る (衝突質量が僅かでも argmin はまず
///   衝突ゼロの行動を選ぶ)。勝者の衝突質量が [`QMDP_VETO_MASS`] を超えたら
///   `NoAction` — 全行動が危険なら止まるのが正解。
/// - 単峰 belief (仮説 1 個) では argmin_a Q(s,a) = 本家 greedy と同じ行動になる。
pub fn qmdp_decide(vi: &ValueIterator, hyps: &[(crate::bridge::PoseView, f64)]) -> QmdpDecision {
    let n_act = vi.actions.len();
    let mut qb = vec![0.0f64; n_act];
    let mut veto = vec![0.0f64; n_act];
    let mut usable = 0.0f64;
    let mut final_mass = 0.0f64;
    for &(p, w) in hyps {
        if !(w > 0.0) {
            continue;
        }
        let (ix, iy, it) = pose_to_cell(vi, p.x, p.y, p.yaw_rad);
        if !vi.in_map_area(ix, iy) || it < 0 || it >= vi.cell_num_t {
            continue;
        }
        let s = &vi.states[vi.to_index(ix, iy, it) as usize];
        if !s.free {
            continue;
        }
        if s.final_state {
            final_mass += w;
            continue;
        }
        usable += w;
        for (ai, a) in vi.actions.iter().enumerate() {
            let q = crate::value_iterator::action_cost_raw(
                &vi.states,
                a,
                s,
                vi.cell_num_x,
                vi.cell_num_y,
                vi.cell_num_t,
            );
            qb[ai] += w * q as f64;
            if q == MAX_COST {
                veto[ai] += w;
            }
        }
    }
    let total = usable + final_mass;
    if total <= 0.0 {
        return QmdpDecision::NoAction;
    }
    if final_mass >= 0.5 * total {
        return QmdpDecision::Goal;
    }
    if usable <= 0.0 {
        return QmdpDecision::NoAction;
    }
    let best = (0..n_act).min_by(|&a, &b| qb[a].total_cmp(&qb[b])).unwrap();
    if veto[best] > QMDP_VETO_MASS * usable {
        return QmdpDecision::NoAction;
    }
    QmdpDecision::Action(best)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::{OccupancyGrid, Quaternion};
    use crate::solvers::test_support::actions;
    use crate::solvers::{solve, U64Solver};

    const RES: f64 = 0.05;
    const NT: i32 = 60;

    /// data 指定つきの ValueIterator を組み立てて solve まで済ませる。
    fn solved_vi(
        width: i32,
        height: i32,
        data: Vec<i8>,
        goal: (f64, f64, i32),
        solver: U64Solver,
    ) -> ValueIterator {
        let grid = OccupancyGrid {
            width,
            height,
            resolution: RES,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data,
        };
        let mut vi = ValueIterator::new(actions(), 1);
        vi.set_map_with_occupancy_grid(&grid, NT, 0.1, 30.0, 0.2, 180);
        vi.set_goal(goal.0, goal.1, goal.2);
        let stats = solve(&mut vi, solver, 100_000);
        assert!(stats.converged, "test map must converge");
        vi
    }

    fn empty_map(size: i32) -> Vec<i8> {
        vec![0i8; (size * size) as usize]
    }

    /// 前進 1 ステップ (0.3 m = 6 セル) では飛び越えられない厚さの壁。
    /// 本家アルゴリズムは遷移の着地セルしか判定しないため、薄壁はすり抜け得る。
    const WALL_THICK: i32 = 8;

    /// 中央に縦壁 (WALL_THICK セル厚、上端だけ開口) を持つ地図。
    fn walled_map(size: i32) -> Vec<i8> {
        let mut data = empty_map(size);
        let wall_x = size / 2;
        for y in 0..(size - 8) {
            for dx in 0..WALL_THICK {
                data[(y * size + wall_x + dx) as usize] = 100;
            }
        }
        data
    }

    #[test]
    fn reaches_goal_on_empty_map() {
        let goal = (2.0, 2.0, 0);
        let vi = solved_vi(64, 64, empty_map(64), goal, U64Solver::Frontier3D);
        let r = rollout_path_on(&vi, 0.6, 0.6, 0.0, 10_000, 0);
        assert!(r.reached_goal(), "status = {:?}", r.status);
        let last = r.poses.last().unwrap();
        assert!((last.x - goal.0).abs() < 1e-9 && (last.y - goal.1).abs() < 1e-9);
        assert!(r.poses.len() >= 3, "path must contain intermediate poses");
    }

    #[test]
    fn sparse_solver_policy_is_rolloutable() {
        // Phase 2 の実運用ソルバ (frontier2d_sparse) でも write_back された方策で
        // ロールアウトできること。
        let goal = (2.0, 2.0, 0);
        let vi = solved_vi(64, 64, empty_map(64), goal, U64Solver::Frontier2DSparse);
        let r = rollout_path_on(&vi, 0.6, 0.6, 0.0, 10_000, 0);
        assert!(r.reached_goal(), "status = {:?}", r.status);
    }

    #[test]
    fn path_avoids_obstacle_cells() {
        let size = 64;
        let goal = (2.8, 0.6, 0);
        let vi = solved_vi(size, size, walled_map(size), goal, U64Solver::Frontier3D);
        let r = rollout_path_on(&vi, 0.4, 0.4, 0.0, 10_000, 0);
        assert!(r.reached_goal(), "status = {:?}", r.status);
        // 全経由点 (ゴール姿勢を除く) のセルが free であること。
        for p in &r.poses[..r.poses.len() - 1] {
            let (ix, iy, _) = pose_to_cell(&vi, p.x, p.y, p.yaw);
            let s = &vi.states[vi.to_index(ix, iy, 0) as usize];
            assert!(s.free, "pose ({}, {}) fell on an obstacle cell", p.x, p.y);
        }
    }

    #[test]
    fn unreachable_goal_reports_no_action() {
        // 開口の無い厚壁でゴール側を完全に仕切る。
        let size = 64;
        let mut data = empty_map(size);
        let wall_x = size / 2;
        for y in 0..size {
            for dx in 0..WALL_THICK {
                data[(y * size + wall_x + dx) as usize] = 100;
            }
        }
        let goal = (2.8, 0.6, 0);
        let vi = solved_vi(size, size, data, goal, U64Solver::Frontier3D);
        let r = rollout_path_on(&vi, 0.4, 0.4, 0.0, 10_000, 0);
        assert_eq!(r.status, RolloutStatus::NoAction);
    }

    #[test]
    fn start_outside_map_reports_out_of_map() {
        let vi = solved_vi(32, 32, empty_map(32), (1.0, 1.0, 0), U64Solver::Frontier3D);
        let r = rollout_path_on(&vi, -5.0, -5.0, 0.0, 100, 0);
        assert_eq!(r.status, RolloutStatus::OutOfMap);
    }

    #[test]
    fn start_on_obstacle_recovers_with_tolerance() {
        // start を障害物セル上に置く。tolerance 0 では失敗、tolerance 有りでは
        // 近傍 free セルへスナップして成功する。
        let size = 64;
        let mut data = empty_map(size);
        // (8, 8) 付近に小さな障害物ブロック。
        for y in 6..=10 {
            for x in 6..=10 {
                data[(y * size + x) as usize] = 100;
            }
        }
        let goal = (2.0, 2.0, 0);
        let vi = solved_vi(size, size, data, goal, U64Solver::Frontier3D);
        let (sx, sy) = (8.0 * RES + 0.5 * RES, 8.0 * RES + 0.5 * RES); // 障害物中心
        let fail = rollout_path_on(&vi, sx, sy, 0.0, 10_000, 0);
        assert_eq!(fail.status, RolloutStatus::NoAction);
        let ok = rollout_path_on(&vi, sx, sy, 0.0, 10_000, 20);
        assert!(ok.reached_goal(), "status = {:?}", ok.status);
    }

    #[test]
    fn start_inside_goal_region_returns_immediately() {
        let goal = (1.0, 1.0, 0);
        let vi = solved_vi(32, 32, empty_map(32), goal, U64Solver::Frontier3D);
        let r = rollout_path_on(&vi, 1.0, 1.0, 0.0, 100, 0);
        assert!(r.reached_goal());
        assert_eq!(r.poses.len(), 2, "start pose + exact goal pose");
    }

    #[test]
    fn densify_bounds_spacing_and_keeps_endpoints() {
        let poses = vec![
            PathPose { x: 0.0, y: 0.0, yaw: 0.0 },
            PathPose { x: 0.3, y: 0.0, yaw: 0.0 },
            PathPose { x: 0.3, y: 0.3, yaw: 1.0 },
        ];
        let dense = densify(&poses, RES);
        assert_eq!(dense.first().unwrap(), &poses[0]);
        assert_eq!(dense.last().unwrap(), &poses[2]);
        for w in dense.windows(2) {
            let d = ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt();
            assert!(d <= RES + 1e-9, "spacing {d} exceeds max");
        }
    }

    /// compact(mapped) sink 経由のロールアウトが、密な `ValueIterator` 経由と同一経路を返すこと。
    /// mapped 経路は `states` を作らないので `final_state` フラグが無く、`CompactPolicy` は
    /// 「value == 0 がゴール圏」という sink の規約で終端判定する。その規約の回帰ガード。
    #[test]
    fn compact_sink_rollout_matches_dense() {
        use crate::solvers::frontier2d_sparse_compact::{solve_compact_mapped, RamSink};
        use std::sync::atomic::AtomicBool;

        let size = 64;
        let data = walled_map(size);
        let goal = (2.8, 0.6, 0);
        let grid = OccupancyGrid {
            width: size,
            height: size,
            resolution: RES,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: data.clone(),
        };

        // 密経路 (基準)。set_map_with_occupancy_grid と同じ safety/goal パラメータを使う。
        let mut vi = ValueIterator::new(actions(), 1);
        vi.set_map_with_occupancy_grid(&grid, NT, 0.1, 30.0, 0.2, 180);
        vi.set_goal(goal.0, goal.1, goal.2);
        assert!(solve(&mut vi, U64Solver::Frontier2DSparseCompact { band: 0 }, 100_000).converged);
        let dense = rollout_path_on(&vi, 0.4, 0.4, 0.0, 10_000, 0);
        assert!(dense.reached_goal(), "dense status = {:?}", dense.status);

        // mapped 経路 (states を作らず sink に確定)。
        let mut sink = RamSink::new((size * size * NT) as usize);
        let s = solve_compact_mapped(
            actions(), 1, &grid, NT, 0.1, 30.0, 0.2, 180, goal.0, goal.1, goal.2, 100_000, None,
            &mut sink, 1, &AtomicBool::new(false),
        );
        assert!(s.converged && !s.cancelled);

        let acts = actions();
        let policy = CompactPolicy::new(
            &sink,
            &acts,
            (size, size, NT),
            RES,
            (0.0, 0.0),
            (goal.0, goal.1, goal.2),
        );
        let compact = rollout_path_on(&policy, 0.4, 0.4, 0.0, 10_000, 0);
        assert!(compact.reached_goal(), "compact status = {:?}", compact.status);
        assert_eq!(compact.poses.len(), dense.poses.len(), "path length differs");
        for (a, b) in compact.poses.iter().zip(dense.poses.iter()) {
            assert_eq!(a, b, "pose differs between compact sink and dense rollout");
        }
    }

    /// 中断フラグを立てた mapped solve は `cancelled` を立てて未収束で戻る。
    #[test]
    fn compact_mapped_solve_honours_cancel() {
        use crate::solvers::frontier2d_sparse_compact::{solve_compact_mapped, RamSink};
        use std::sync::atomic::AtomicBool;

        let size = 64;
        let grid = OccupancyGrid {
            width: size,
            height: size,
            resolution: RES,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: empty_map(size),
        };
        let mut sink = RamSink::new((size * size * NT) as usize);
        let s = solve_compact_mapped(
            actions(), 1, &grid, NT, 0.1, 30.0, 0.2, 180, 2.0, 2.0, 0, 100_000, None, &mut sink, 1,
            &AtomicBool::new(true),
        );
        assert!(!s.converged && s.cancelled);
    }

    #[test]
    fn optimal_action_at_bounds_and_semantics() {
        let vi = solved_vi(32, 32, empty_map(32), (1.0, 1.0, 0), U64Solver::Frontier3D);
        assert_eq!(optimal_action_at(&vi, -1, 0, 0), -1);
        assert_eq!(optimal_action_at(&vi, 0, 0, NT), -1);
        assert_eq!(optimal_action_at(&vi, 0, 0, -1), -1);
        // 到達可能な free セルは 0..6 の行動 id を持つ。
        let a = optimal_action_at(&vi, 4, 4, 0);
        assert!((0..6).contains(&a), "action id = {a}");
    }

    // ═══ QMDP ═══

    use crate::bridge::PoseView;

    fn hyp(x: f64, y: f64, yaw: f64) -> PoseView {
        PoseView { x, y, yaw_rad: yaw }
    }

    /// 仮説セルでの Q(s,a)。衝突チェック用 (テスト専用の薄い読み)。
    fn q_at(vi: &ValueIterator, p: PoseView, ai: usize) -> u64 {
        let (ix, iy, it) = pose_to_cell(vi, p.x, p.y, p.yaw_rad);
        let s = &vi.states[vi.to_index(ix, iy, it) as usize];
        crate::value_iterator::action_cost_raw(
            &vi.states,
            &vi.actions[ai],
            s,
            vi.cell_num_x,
            vi.cell_num_y,
            vi.cell_num_t,
        )
    }

    /// set_goal_region: 2 点ゴールの多目標場が収束し、どちらの側からの貪欲降下も
    /// ゴールに入ること (能動的再定位の行き先マーキング)。
    #[test]
    fn set_goal_region_solves_a_multi_goal_field() {
        let grid = OccupancyGrid {
            width: 64,
            height: 64,
            resolution: RES,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: empty_map(64),
        };
        let mut vi = ValueIterator::new(actions(), 1);
        vi.set_map_with_occupancy_grid(&grid, NT, 0.1, 30.0, 0.2, 180);
        vi.set_goal_region(&[(0.8, 0.8), (2.4, 2.4)], 0.2);
        let stats = solve(&mut vi, U64Solver::Frontier3D, 100_000);
        assert!(stats.converged);
        for start in [(0.5, 0.5), (2.7, 2.7)] {
            let r = rollout_path_on(&vi, start.0, start.1, 0.0, 10_000, 0);
            assert!(r.reached_goal(), "start {start:?}: {:?}", r.status);
        }
    }

    #[test]
    fn qmdp_single_hypothesis_matches_greedy() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0), U64Solver::Frontier3D);
        let p = hyp(0.6, 0.6, 0.0);
        let (ix, iy, it) = pose_to_cell(&vi, p.x, p.y, p.yaw_rad);
        match qmdp_decide(&vi, &[(p, 1.0)]) {
            QmdpDecision::Action(ai) => {
                assert_eq!(vi.actions[ai].id, optimal_action_at(&vi, ix, iy, it))
            }
            d => panic!("expected Action, got {d:?}"),
        }
    }

    #[test]
    fn qmdp_multimodal_avoids_collision_for_significant_hypothesis() {
        // 仮説 A は開けた場所で前進がゴール方向、仮説 B は壁の直前で前進 = 衝突。
        // どちらも質量 0.5 — 選ばれた行動は両仮説で衝突であってはならない
        // (衝突項は MAX_COST そのものなので argmin がまず衝突ゼロの行動を選ぶ)。
        let size = 64;
        let vi = solved_vi(size, size, walled_map(size), (2.8, 0.6, 0), U64Solver::Frontier3D);
        let a = hyp(1.0, 2.0, 0.0);
        // 壁は x セル 32..40。3 セル手前から東向き → 前進 6 セルは壁の中。
        let b = hyp(1.475, 2.0, 0.0);
        assert_eq!(q_at(&vi, b, 0), MAX_COST, "premise: forward from B collides");
        match qmdp_decide(&vi, &[(a, 0.5), (b, 0.5)]) {
            QmdpDecision::Action(ai) => {
                assert_ne!(q_at(&vi, a, ai), MAX_COST, "chosen action collides for A");
                assert_ne!(q_at(&vi, b, ai), MAX_COST, "chosen action collides for B");
            }
            d => panic!("expected Action, got {d:?}"),
        }
    }

    #[test]
    fn qmdp_boxed_hypothesis_never_picks_collision() {
        // 周囲を塞がれたポケットの中の仮説: 並進系 4 行動は全て衝突 (MAX_COST)、
        // その場旋回は自セル着地 (V = MAX_COST → 本家の u64 折り返しで巨大値だが
        // MAX_COST ではない)。argmin は旋回を選び、衝突には決して踏み込まない。
        let size = 64;
        let mut data = empty_map(size);
        for y in 4..19 {
            for x in 4..19 {
                data[(y * size + x) as usize] = 100;
            }
        }
        data[(11 * size + 11) as usize] = 0;
        let vi = solved_vi(size, size, data, (2.0, 2.0, 0), U64Solver::Frontier3D);
        let p = hyp(0.575, 0.575, 0.0);
        assert_eq!(q_at(&vi, p, 0), MAX_COST, "premise: forward collides");
        match qmdp_decide(&vi, &[(p, 1.0)]) {
            QmdpDecision::Action(ai) => {
                assert_ne!(q_at(&vi, p, ai), MAX_COST, "picked a colliding action");
                assert_eq!(vi.actions[ai].delta_fw, 0.0, "only in-place turns are safe");
            }
            d => panic!("expected Action, got {d:?}"),
        }
    }

    #[test]
    fn qmdp_goal_majority_reports_goal() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0), U64Solver::Frontier3D);
        // ゴール中心の仮説 (final_state) が過半、残りは通常セル。
        let hyps = [(hyp(2.0, 2.0, 0.0), 0.6), (hyp(0.6, 0.6, 0.0), 0.4)];
        assert_eq!(qmdp_decide(&vi, &hyps), QmdpDecision::Goal);
    }

    #[test]
    fn qmdp_no_usable_hypotheses_is_no_action() {
        let vi = solved_vi(32, 32, empty_map(32), (1.0, 1.0, 0), U64Solver::Frontier3D);
        // 地図外と重みゼロだけ → 評価できる質量なし。
        let hyps = [(hyp(-5.0, -5.0, 0.0), 1.0), (hyp(0.6, 0.6, 0.0), 0.0)];
        assert_eq!(qmdp_decide(&vi, &hyps), QmdpDecision::NoAction);
        assert_eq!(qmdp_decide(&vi, &[]), QmdpDecision::NoAction);
    }
}
