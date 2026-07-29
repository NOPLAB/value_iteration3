//! rclrs 非依存の統合プランナ中核。**1 本の価値関数**を広域 (経路計画) と
//! 狭域 (経路追従) の両方で共有する。
//!
//! vi_global_planner (`compute_path_to_pose`) と vi_local_planner
//! (`follow_path`) を別プロセスで動かすと、同じ地図・同じゴールに対して同じ
//! 価値反復を 2 回解くことになる (走り出しまでの時間も、常駐する価値関数の
//! メモリも 2 倍)。しかも広域側が作った `nav_msgs/Path` は狭域側が終端姿勢しか
//! 読まないので、ロールアウトの成果はほぼ捨てられていた。ここではその 2 つを
//! 1 つの `ValueIteratorLocal` の上に載せ直す:
//!
//!   - solve はゴールごとに 1 回だけ (`prepare_goal_with_progress`)
//!   - `plan_*` は解決済み価値関数の貪欲ロールアウト = 旧実装の
//!     「キャッシュヒット経路」そのもの
//!   - 追従は同じ価値関数の ±1m ウィンドウをスキャンで補正しながら回す
//!
//! 副次的な性質として、ロールアウトは追従中にスキャン由来の `local_penalty` が
//! 注入された後の値を読む。旧構成では広域側が静的地図しか知らなかったので、
//! これは退化ではなく改善方向の差異だが、「経路は毎回同じ」ではなくなる。
//!
//! アウトオブコア (compact) 経路と `map_scale` はここには無い。狭域側は
//! `ValueIterator::states` を直接書き換えるので、状態配列を持たない compact 解には
//! 載らない。広域地図 (津田沼など) は従来どおり vi_global_planner +
//! controller_server の構成を使うこと。
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
use vi_reference::msg::{LaserScan, OccupancyGrid};
use vi_reference::planner::{
    densify, optimal_action_at, pose_to_cell, rollout_path_on, PathPose, RolloutStatus,
};
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::value_iterator::ValueIterator;
use vi_reference::{Action, ValueIteratorLocal};

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

/// 計画・追従の設定。前半は solve とキャッシュ、後半は用途別
/// (`plan_*` = 広域ロールアウト / `decide` = 狭域追従)。
#[derive(Clone, Copy)]
pub struct PlanConfig {
    pub solver: U64Solver,
    /// solve 総イテレーション上限 (発散ガード)。
    pub max_solve_iter: u32,
    /// cancel 観測間隔 (イテレーション数)。
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanError {
    /// cancel フラグにより中断された (プリエンプト / client cancel)。
    Cancelled,
    /// `max_solve_iter` 内に収束しなかった。
    NotConverged,
    /// 価値関数は収束したがロールアウトが失敗した (`plan_*` のみ)。
    Rollout(RolloutStatus),
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlanError::Cancelled => write!(f, "cancelled (preempted)"),
            PlanError::NotConverged => write!(f, "value iteration did not converge"),
            PlanError::Rollout(s) => write!(f, "policy rollout failed: {s:?}"),
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

struct CachedGoal {
    goal_x: f64,
    goal_y: f64,
    goal_t_deg: i32,
    vi: ValueIteratorLocal,
}

pub struct PlannerCore {
    build: BuildParams,
    cfg: PlanConfig,
    cached: Option<CachedGoal>,
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
        Self { build, cfg, cached: None }
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
        stats.solved_now = true;
        self.cached = Some(CachedGoal { goal_x: goal.x, goal_y: goal.y, goal_t_deg, vi });
        Ok(stats)
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

        let vi = &self.cached.as_ref().expect("cache filled above").vi;
        let r = rollout_path_on(
            &vi.base,
            start.x,
            start.y,
            start.yaw_rad,
            self.cfg.max_rollout_steps,
            self.cfg.start_tolerance_cells,
        );
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

    /// θ=0 全域スライスの可視化グリッド (`value_grid_of`)。ゴール未設定なら None。
    pub fn value_grid(&self, threshold_steps: u64) -> Option<OccupancyGrid> {
        self.cached.as_ref().map(|c| value_grid_of(&c.vi.base, threshold_steps))
    }

    /// ローカルウィンドウ範囲だけを現在方位の θ スライスで描画した可視化グリッド。
    /// `set_window` 後に呼ぶこと。地図端ではクランプ後の実範囲を使うので、本家
    /// `makeLocalValueFunctionMap` と違い幅とデータ長が常に一致する。
    pub fn window_value_grid(
        &self,
        pose: PoseView,
        threshold_steps: u64,
    ) -> Option<OccupancyGrid> {
        let c = self.cached.as_ref()?;
        let vi = &c.vi;
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
    pub fn set_window(&mut self, pose: PoseView) {
        if let Some(c) = self.cached.as_mut() {
            c.vi.set_local_window(pose.x, pose.y);
        }
    }

    /// スキャンのヒット点周辺に local_penalty を注入 (本家 `setLocalCost`)。
    /// `set_window` 後に呼ぶこと (ウィンドウ外のヒットは無視される)。
    pub fn observe_scan(&mut self, scan: &LaserScan, pose: PoseView) {
        if let Some(c) = self.cached.as_mut() {
            c.vi.set_local_cost(scan, pose.x, pose.y, pose.yaw_rad);
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
        let Some(c) = self.cached.as_mut() else { return (0, true) };
        let vi = &mut c.vi;
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
    pub fn decide(&self, pose: PoseView) -> Decision {
        let Some(c) = self.cached.as_ref() else { return Decision::NoAction };
        let vi = &c.vi.base;
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
    use vi_reference::Quaternion;

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
        }
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
            let c = core.cached.as_ref().unwrap();
            pose_to_cell(&c.vi.base, 1.2, 1.0, 0.0)
        };
        let before = {
            let c = core.cached.as_ref().unwrap();
            c.vi.base.states[c.vi.base.to_index(uix, uiy, uit) as usize].total_cost
        };

        // 正面 0.5m にヒット 1 ビーム → (1.5, 1.0) 周辺へ 2048<<PROB_BASE_BIT。
        let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
        core.observe_scan(&scan, robot);
        {
            let c = core.cached.as_ref().unwrap();
            let (hx, hy, _) = pose_to_cell(&c.vi.base, 1.5, 1.0, 0.0);
            let hit = c.vi.base.to_index(hx, hy, 0) as usize;
            assert_eq!(c.vi.base.states[hit].local_penalty, 2048u64 << PROB_BASE_BIT);
        }

        core.refine_passes(5);
        let after = {
            let c = core.cached.as_ref().unwrap();
            c.vi.base.states[c.vi.base.to_index(uix, uiy, uit) as usize].total_cost
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
        };
        assert_eq!(strict_core.decide(on_obstacle), Decision::NoAction);
        // tolerance 4 (0.2m) なら近傍の行動を借りられる。
        let relaxed_core = PlannerCore {
            build: strict_core.build.clone(),
            cfg: cfg(),
            cached: strict_core.cached,
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
}
