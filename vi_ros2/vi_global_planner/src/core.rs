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
use vi_reference::params::MAX_COST;
use vi_reference::planner::{
    densify, rollout_path_on, CompactPolicy, PathPose, PolicyView, Rollout, RolloutStatus,
};
use vi_reference::solvers::frontier2d_sparse_compact::{
    default_threads, solve_compact_mapped_observed, CompactSink, RamSink,
};
use vi_reference::solvers::{
    solve_observed, SolveFlow, SolveObserver, SolveProbe, U64Solver,
};
use vi_reference::{Action, ValueIterator};

/// 巨大マップ用アウトオブコア経路の出力先。`None` = RAM (`RamSink`)、`Some(dir)` = その
/// ディレクトリ上の mmap ファイル。必要量は `nx·ny·theta_cell_num × 12 B`
/// (津田沼 0.15 m/cell で約 1.9 GB) なので、Pi4 のような小メモリ機ではディスクに逃がす。
pub type SinkDir = Option<std::path::PathBuf>;

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
#[derive(Clone)]
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
    /// アウトオブコア (compact) 経路の出力先。`solver` が
    /// `frontier2d_sparse_compact` のときのみ参照される。
    pub compact_sink_dir: SinkDir,
    /// compact 経路のワーカースレッド数 (0 = `default_threads()`)。
    pub vi_threads: usize,
}

impl PlanConfig {
    /// アウトオブコア (states を作らない) 経路を使うか。
    ///
    /// `frontier2d_sparse_compact` を選んだときだけ `solve_compact_mapped` に切り替える。
    /// 他のソルバは `ValueIterator::states` (56 B × nx·ny·nθ) を密に確保するので、
    /// 津田沼のような広域地図では確保だけで数十 GB になり起動不能。
    pub fn use_compact(&self) -> bool {
        self.solver.caps().out_of_core
    }
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlanError {
    /// cancel フラグにより中断された (新ゴールによるプリエンプト)。
    Cancelled,
    /// `max_solve_iter` 内に収束しなかった。
    NotConverged,
    /// 価値関数は収束したがロールアウトが失敗した。
    Rollout(RolloutStatus),
    /// compact 経路の出力先 (mmap ファイル) を用意できなかった。
    Sink(String),
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlanError::Cancelled => write!(f, "planning cancelled (preempted)"),
            PlanError::NotConverged => write!(f, "value iteration did not converge"),
            PlanError::Rollout(s) => write!(f, "policy rollout failed: {s:?}"),
            PlanError::Sink(e) => write!(f, "compact output sink unavailable: {e}"),
        }
    }
}

/// solve 済み価値関数の実体。密経路は `ValueIterator` をそのまま、compact 経路は
/// `solve_compact_mapped` が確定した sink (+ 幾何) を保持する。
enum SolvedField {
    Dense(Box<ValueIterator>),
    Compact(CompactField),
}

/// compact 経路の solve 結果。`sink` は orig 索引で `(total_cost, action)` を返す確定出力で、
/// `CompactPolicy` がロールアウト用の方策ビューとして読む。
struct CompactField {
    sink: Box<dyn CompactSink + Send>,
    actions: Vec<Action>,
    cell_num: (i32, i32, i32),
    resolution: f64,
    origin: (f64, f64),
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
}

/// solve の進行制御 ([`SolveObserver`] 実装)。境界 (密: `solve_chunk` 反復ごと /
/// compact: バンド finalize ごと) で cancel を観測し、途中経過を `on_progress` へ流す。
/// 旧実装の手書きチャンクループの置き換え (vi_planner::core の SolveDirector と同型 —
/// クレート間依存を避ける意図的な重複)。
struct SolveDirector<'a> {
    interval: u32,
    cancel: &'a AtomicBool,
    on_progress: &'a mut dyn FnMut(&dyn PolicyView),
}

impl SolveObserver for SolveDirector<'_> {
    fn interval(&self) -> u32 {
        self.interval.max(1)
    }
    fn boundary(&mut self, probe: &mut dyn SolveProbe) -> SolveFlow {
        if self.cancel.load(Ordering::Relaxed) {
            return SolveFlow::Cancel;
        }
        (self.on_progress)(probe.policy());
        SolveFlow::Continue
    }
}

struct CachedSolve {
    goal_x: f64,
    goal_y: f64,
    goal_t_deg: i32,
    field: SolvedField,
}

/// value_function 配信用の θ=0 スライス (密/compact 共通)。
pub struct ValueSlice {
    pub width: i32,
    pub height: i32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    /// 長さ `width*height`、`total_cost` の生値 (未到達は `MAX_COST`)。
    pub values: Vec<u64>,
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

    /// solve 済みキャッシュ (密経路のみ; compact 経路は `states` を持たないので `None`)。
    pub fn cached_vi(&self) -> Option<&ValueIterator> {
        match self.cached.as_ref()?.field {
            SolvedField::Dense(ref vi) => Some(vi),
            SolvedField::Compact(_) => None,
        }
    }

    /// solve 済みキャッシュの θ=0 スライス (value_function 配信用。密/compact 共通)。
    pub fn cached_value_slice(&self) -> Option<ValueSlice> {
        let field = &self.cached.as_ref()?.field;
        Some(match field {
            SolvedField::Dense(vi) => value_slice_from_vi(vi),
            SolvedField::Compact(c) => ValueSlice {
                width: c.cell_num.0,
                height: c.cell_num.1,
                resolution: c.resolution,
                origin_x: c.origin.0,
                origin_y: c.origin.1,
                values: c.policy().value_slice_theta0(),
            },
        })
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
        self.plan_with_progress(start, goal, cancel, &mut |_| {})
    }

    /// `plan` と同じだが、solve の境界 (密: `solve_chunk` 反復ごと / compact: バンド
    /// finalize ごと) に `on_chunk` を呼ぶ (途中経過の value_function 可視化用 —
    /// [`value_slice_on`] がそのまま受け取れる `&dyn PolicyView` を渡す)。
    /// キャッシュヒット時 (solve なし) は呼ばれない。
    pub fn plan_with_progress(
        &mut self,
        start: PoseView,
        goal: PoseView,
        cancel: &AtomicBool,
        on_chunk: &mut dyn FnMut(&dyn PolicyView),
    ) -> Result<(Vec<PathPose>, PlanStats), PlanError> {
        let goal_t_deg = yaw_to_goal_theta_deg(goal.yaw_rad);

        let mut stats = PlanStats { solved_now: false, iters: 0, poses: 0 };

        if !self.cache_matches(&goal, goal_t_deg) {
            self.cached = None; // 旧キャッシュ (数 GB になり得る) を先に解放
            let field = if self.cfg.use_compact() {
                self.solve_compact(&goal, goal_t_deg, cancel, &mut stats, on_chunk)?
            } else {
                self.solve_dense(&goal, goal_t_deg, cancel, &mut stats, on_chunk)?
            };
            stats.solved_now = true;
            self.cached =
                Some(CachedSolve { goal_x: goal.x, goal_y: goal.y, goal_t_deg, field });
        }

        let cached = self.cached.as_ref().expect("cache filled above");
        let r = match &cached.field {
            SolvedField::Dense(vi) => self.rollout(vi.as_ref(), start),
            SolvedField::Compact(c) => self.rollout(&c.policy(), start),
        };
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

    fn rollout(&self, policy: &dyn PolicyView, start: PoseView) -> Rollout {
        rollout_path_on(
            policy,
            start.x,
            start.y,
            start.yaw_rad,
            self.cfg.max_rollout_steps,
            self.cfg.start_tolerance_cells,
        )
    }

    /// 密経路: `ValueIterator::states` を確保し、[`solve_observed`] + [`SolveDirector`]
    /// で解く。cancel の観測と途中経過は solve 内部の境界 (`solve_chunk` 反復ごと) で
    /// 行われる — 旧実装のチャンク再入 (毎チャンクの再ビルド + 全セル write_back) は
    /// もう無い。
    fn solve_dense(
        &self,
        goal: &PoseView,
        goal_t_deg: i32,
        cancel: &AtomicBool,
        stats: &mut PlanStats,
        on_chunk: &mut dyn FnMut(&dyn PolicyView),
    ) -> Result<SolvedField, PlanError> {
        if cancel.load(Ordering::Relaxed) {
            return Err(PlanError::Cancelled);
        }
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

        let mut director = SolveDirector {
            interval: self.cfg.solve_chunk,
            cancel,
            on_progress: on_chunk,
        };
        let out = solve_observed(&mut vi, self.cfg.solver, self.cfg.max_solve_iter, &mut director);
        stats.iters = out.iters;
        if out.cancelled {
            return Err(PlanError::Cancelled);
        }
        if !out.converged {
            return Err(PlanError::NotConverged);
        }
        Ok(SolvedField::Dense(Box::new(vi)))
    }

    /// compact (アウトオブコア) 経路: `states` を作らず地図とゴールから直接解き、確定出力を
    /// sink に置く。境界はバンド finalize ごとで、そこで `on_chunk` (sink ビューの
    /// `&dyn PolicyView`) も呼ばれる (`cancel` はソルバ内部のラウンド境界でも従来
    /// どおり観測される)。
    fn solve_compact(
        &self,
        goal: &PoseView,
        goal_t_deg: i32,
        cancel: &AtomicBool,
        stats: &mut PlanStats,
        on_chunk: &mut dyn FnMut(&dyn PolicyView),
    ) -> Result<SolvedField, PlanError> {
        let g = &self.build.grid;
        let nt = self.build.theta_cell_num;
        let nstates = g.width as usize * g.height as usize * nt as usize;
        let mut sink = make_sink(nstates, &self.cfg.compact_sink_dir)?;
        let nthreads = if self.cfg.vi_threads > 0 { self.cfg.vi_threads } else { default_threads() };

        let mut director = SolveDirector {
            interval: self.cfg.solve_chunk,
            cancel,
            on_progress: on_chunk,
        };
        let s = solve_compact_mapped_observed(
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
            &mut director,
        );
        stats.iters = s.iters;
        if s.cancelled {
            return Err(PlanError::Cancelled);
        }
        if !s.converged {
            return Err(PlanError::NotConverged);
        }
        Ok(SolvedField::Compact(CompactField {
            sink,
            actions: self.build.actions.clone(),
            cell_num: (g.width, g.height, nt),
            resolution: g.resolution,
            origin: (g.origin_x, g.origin_y),
            goal: (goal.x, goal.y, goal_t_deg),
        }))
    }
}

/// 場の θ=0 スライスを取り出す (solve 途中経過の配信にも使う)。ビューは
/// [`PolicyView`] なので密 (`&ValueIterator`) も compact (sink ビュー) も同じ実装。
pub fn value_slice_on(p: &dyn PolicyView) -> ValueSlice {
    let (nx, ny, _) = p.cell_num();
    let mut values = vec![MAX_COST; nx as usize * ny as usize];
    for iy in 0..ny {
        for ix in 0..nx {
            values[iy as usize * nx as usize + ix as usize] = p.value_at(ix, iy, 0);
        }
    }
    let (ox, oy) = p.map_origin();
    ValueSlice {
        width: nx,
        height: ny,
        resolution: p.xy_resolution(),
        origin_x: ox,
        origin_y: oy,
        values,
    }
}

/// `value_slice_on` の `&ValueIterator` 版 (後方互換の別名)。
pub fn value_slice_from_vi(vi: &ValueIterator) -> ValueSlice {
    value_slice_on(vi)
}

/// compact 出力 sink を作る。`dir` 指定時はディスク mmap、無指定は RAM。
fn make_sink(
    nstates: usize,
    dir: &SinkDir,
) -> Result<Box<dyn CompactSink + Send>, PlanError> {
    match dir {
        Some(dir) => crate::sink::MmapSink::new(dir, nstates)
            .map(|s| Box::new(s) as Box<dyn CompactSink + Send>)
            .map_err(|e| PlanError::Sink(e.to_string())),
        None => Ok(Box::new(RamSink::new(nstates))),
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

    /// compact 経路が密経路と同じ経路を返し、キャッシュ規約も同じであること。
    /// (広域地図では密経路が states の確保だけで数十 GB になるので、この経路が唯一の選択肢になる。)
    #[test]
    fn compact_path_matches_dense_and_caches() {
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
        // compact 経路は states を持たないので cached_vi は None、value スライスは取れる。
        assert!(core.cached_vi().is_none());
        let slice = core.cached_value_slice().expect("value slice");
        assert_eq!((slice.width, slice.height), (64, 64));
    }

    /// compact 経路でも事前に立てた cancel でプリエンプトされること
    /// (solve_compact_mapped はチャンク分割できないので、中断はソルバ内部の観測に依る)。
    #[test]
    fn compact_pre_raised_cancel_aborts() {
        let mut core = PlannerCore::new(build(64), cfg_compact());
        let cancel = AtomicBool::new(true);
        let err = core.plan(pose(0.6, 0.6, 0.0), pose(2.0, 2.0, 0.0), &cancel).unwrap_err();
        assert_eq!(err, PlanError::Cancelled);
    }

    /// ディスク mmap sink 経由でも同じ経路が出ること (Pi4 のような小メモリ機の経路)。
    #[test]
    fn compact_with_mmap_sink_plans() {
        let dir = std::env::temp_dir().join("vi_global_planner_core_mmap_test");
        let cfg = PlanConfig { compact_sink_dir: Some(dir.clone()), ..cfg_compact() };
        let cancel = AtomicBool::new(false);
        let mut core = PlannerCore::new(build(64), cfg);
        let (p, s) = core.plan(pose(0.6, 0.6, 0.0), pose(2.0, 2.0, 0.0), &cancel).expect("plan");
        assert!(s.solved_now && p.len() > 2);
        drop(core);
        let _ = std::fs::remove_dir_all(&dir);
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

    /// 進捗コールバックは新規 solve でのみ発火し、キャッシュヒットでは
    /// 発火しない (途中経過の value_function 配信の前提)。
    #[test]
    fn progress_callback_fires_only_when_solving() {
        let mut core = PlannerCore::new(build(64), cfg());
        let cancel = AtomicBool::new(false);
        let goal = pose(2.0, 2.0, 0.0);

        let mut calls = 0usize;
        core.plan_with_progress(pose(0.6, 0.6, 0.0), goal, &cancel, &mut |vi| {
            calls += 1;
            assert!(vi.cell_num().0 > 0);
        })
        .expect("first plan");
        assert!(calls > 0);

        let mut calls_cached = 0usize;
        core.plan_with_progress(pose(0.4, 1.8, 0.0), goal, &cancel, &mut |_| {
            calls_cached += 1;
        })
        .expect("replan");
        assert_eq!(calls_cached, 0);
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
