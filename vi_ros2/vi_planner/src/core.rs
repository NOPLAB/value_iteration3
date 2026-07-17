//! rclrs 非依存のプランナ中核。地図とパラメータを保持し、ゴールごとに
//! ValueIterator を構築 → キャンセル可能なチャンク solve → 方策ロールアウト
//! で経路 (世界座標の姿勢列) を返す。
//!
//! - 直近ゴールの solve 済み ValueIterator をキャッシュし、同一ゴールの
//!   リプラン (Nav2 BT の周期的 ComputePathToPose) はロールアウトのみで返す。
//! - solve は `solve_chunk` イテレーションごとに cancel フラグを観測する。
//!   fused/sparse 系は非収束打ち切りでも states へ write_back するので、
//!   チャンク再入で進捗は保存される (frontier 系はシード再構築のみ冗長)。
//!
//! このモジュールは vi_reference のみに依存し、ホストで `cargo test` できる
//! (分離クレート方式; リポジトリ CLAUDE.md 参照)。

use std::sync::atomic::{AtomicBool, Ordering};

use vi_reference::bridge::{yaw_to_goal_theta_deg, PoseView};
use vi_reference::msg::OccupancyGrid;
use vi_reference::planner::{densify, rollout_path, PathPose, RolloutStatus};
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::{Action, ValueIterator};

/// ゴールごとの ValueIterator 構築入力 (地図は起動時に一度だけ取り込む)。
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

/// 計画動作の設定。
#[derive(Clone, Copy)]
pub struct PlanConfig {
    pub solver: U64Solver,
    /// solve 総イテレーション上限 (発散ガード)。
    pub max_solve_iter: u32,
    /// cancel 観測間隔 (イテレーション数)。
    pub solve_chunk: u32,
    pub max_rollout_steps: usize,
    /// start が方策なしセルのとき近傍探索する範囲 (セル数)。
    pub start_tolerance_cells: i32,
    /// 経路の最大姿勢間隔 (m)。0 以下で補間なし。
    pub path_spacing: f64,
    /// キャッシュ再利用とみなすゴール位置差 (m)。
    pub goal_tolerance_xy: f64,
    /// キャッシュ再利用とみなすゴール方位差 (度)。
    pub goal_tolerance_deg: f64,
}

/// 1 回の plan の統計 (ログ/Feedback 用)。
#[derive(Clone, Copy, Debug)]
pub struct PlanStats {
    /// この呼び出しで solve を実行したか (false = キャッシュヒット)。
    pub solved_now: bool,
    /// 実行した solve イテレーション数 (キャッシュヒット時 0)。
    pub iters: u32,
    pub poses: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanError {
    /// cancel フラグにより中断された (新ゴールによるプリエンプト)。
    Cancelled,
    /// `max_solve_iter` 内に収束しなかった。
    NotConverged,
    /// 価値関数は収束したがロールアウトが失敗した。
    Rollout(RolloutStatus),
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlanError::Cancelled => write!(f, "planning cancelled (preempted)"),
            PlanError::NotConverged => write!(f, "value iteration did not converge"),
            PlanError::Rollout(s) => write!(f, "policy rollout failed: {s:?}"),
        }
    }
}

struct CachedSolve {
    goal_x: f64,
    goal_y: f64,
    goal_t_deg: i32,
    vi: ValueIterator,
}

pub struct PlannerCore {
    build: BuildParams,
    cfg: PlanConfig,
    cached: Option<CachedSolve>,
}

/// 円環上の角度差 (度、0..=180)。
fn circ_deg_diff(a: i32, b: i32) -> i32 {
    let d = (a - b).rem_euclid(360);
    d.min(360 - d)
}

impl PlannerCore {
    pub fn new(build: BuildParams, cfg: PlanConfig) -> Self {
        Self { build, cfg, cached: None }
    }

    /// solve 済みキャッシュ (value_function 配信などの読み取り用)。
    pub fn cached_vi(&self) -> Option<&ValueIterator> {
        self.cached.as_ref().map(|c| &c.vi)
    }

    fn cache_matches(&self, goal: &PoseView, goal_t_deg: i32) -> bool {
        let Some(c) = self.cached.as_ref() else { return false };
        let d2 = (c.goal_x - goal.x).powi(2) + (c.goal_y - goal.y).powi(2);
        d2.sqrt() <= self.cfg.goal_tolerance_xy
            && circ_deg_diff(c.goal_t_deg, goal_t_deg) as f64 <= self.cfg.goal_tolerance_deg
    }

    /// start から goal への経路を計画する。同一ゴールなら solve をスキップし
    /// ロールアウトのみ。`cancel` は solve 中に `solve_chunk` ごとに観測される。
    pub fn plan(
        &mut self,
        start: PoseView,
        goal: PoseView,
        cancel: &AtomicBool,
    ) -> Result<(Vec<PathPose>, PlanStats), PlanError> {
        let goal_t_deg = yaw_to_goal_theta_deg(goal.yaw_rad);

        let mut stats = PlanStats { solved_now: false, iters: 0, poses: 0 };

        if !self.cache_matches(&goal, goal_t_deg) {
            self.cached = None; // 旧キャッシュ (数 GB になり得る) を先に解放
            let mut vi = ValueIterator::new(self.build.actions.clone(), 1);
            vi.set_map_with_occupancy_grid(
                &self.build.grid,
                self.build.theta_cell_num,
                self.build.safety_radius,
                self.build.safety_radius_penalty,
                self.build.goal_margin_radius,
                self.build.goal_margin_theta,
            );
            vi.set_goal(goal.x, goal.y, goal_t_deg);

            let mut remaining = self.cfg.max_solve_iter;
            let converged = loop {
                if cancel.load(Ordering::Relaxed) {
                    return Err(PlanError::Cancelled);
                }
                if remaining == 0 {
                    break false;
                }
                let chunk = remaining.min(self.cfg.solve_chunk.max(1));
                let s = solve(&mut vi, self.cfg.solver, chunk);
                stats.iters = stats.iters.saturating_add(s.iters);
                remaining -= chunk;
                if s.converged {
                    break true;
                }
            };
            if !converged {
                return Err(PlanError::NotConverged);
            }
            stats.solved_now = true;
            self.cached =
                Some(CachedSolve { goal_x: goal.x, goal_y: goal.y, goal_t_deg, vi });
        }

        let vi = &self.cached.as_ref().expect("cache filled above").vi;
        let r = rollout_path(
            vi,
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
        stats.poses = poses.len();
        Ok((poses, stats))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;
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
            max_rollout_steps: 10_000,
            start_tolerance_cells: 10,
            path_spacing: RES,
            goal_tolerance_xy: 0.25,
            goal_tolerance_deg: 10.0,
        }
    }

    fn pose(x: f64, y: f64, yaw: f64) -> PoseView {
        PoseView { x, y, yaw_rad: yaw }
    }

    #[test]
    fn plans_and_caches_for_same_goal() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(false);
        let goal = pose(2.0, 2.0, 0.0);

        let (p1, s1) = core.plan(pose(0.6, 0.6, 0.0), goal, &cancel).expect("first plan");
        assert!(s1.solved_now && s1.iters > 0);
        assert!(p1.len() > 2);

        // 同一ゴールの再計画 (別 start) はキャッシュヒット。
        let (p2, s2) = core.plan(pose(0.4, 1.8, 1.0), goal, &cancel).expect("replan");
        assert!(!s2.solved_now && s2.iters == 0);
        assert!(p2.len() > 2);

        // ゴール移動で再 solve。
        let (_, s3) = core.plan(pose(0.6, 0.6, 0.0), pose(0.8, 2.4, 0.0), &cancel).expect("new goal");
        assert!(s3.solved_now);
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
    fn pre_raised_cancel_aborts() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(true);
        let err = core.plan(pose(0.6, 0.6, 0.0), pose(2.0, 2.0, 0.0), &cancel).unwrap_err();
        assert_eq!(err, PlanError::Cancelled);
    }

    #[test]
    fn unreachable_goal_fails_rollout() {
        // ゴール周辺だけ厚壁で囲む (中は free のままゴールを置く)。
        let size = 64;
        let mut b = build(size);
        // x=40..48 の縦壁と y=40..48 の横壁で右上区画を仕切る。
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
        let err = core
            .plan(pose(0.6, 0.6, 0.0), pose(2.8, 2.8, 0.0), &cancel)
            .unwrap_err();
        assert!(matches!(err, PlanError::Rollout(RolloutStatus::NoAction)), "{err:?}");
    }
}
