//! 収束済み `ValueIterator` の方策から経路 (世界座標の姿勢列) を生成するプランナ層。
//!
//! ソルバ非依存: `solvers::solve` がどの `U64Solver` でも、収束後は
//! `states[i].optimal_action` に方策が書き戻されている (fused/sparse 系は
//! `write_back_fused` が、sweep 系は `value_iteration_raw` が書く) ことを前提に、
//! 貪欲ロールアウトで方策を辿る。
//!
//! 動作モデルは本家の実行系と同一:
//! - 世界座標→セルの変換は `pos_to_action` の式 (度は i32 切り捨て、`+360*100` で正規化)
//! - 1 ステップの変位は `no_noise_state_transition` と同じ「現在向きで delta_fw 並進 →
//!   delta_rot 回転」
//!
//! `rollout_path` は `&ValueIterator` のみ借用する読み取り専用 API
//! (`pos_to_action` と違い `status` を書き換えない) ので、solve 済みのイテレータを
//! 複数ゴール間で使い回すプランナサーバから安全に呼べる。

use std::collections::HashMap;
use std::f64::consts::PI;

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

/// 世界座標をセル (ix, iy, it) へ変換する。本家 `posToAction` の変換式を逐語再現
/// (度は i32 切り捨て、`+360*100 (mod 360)` 正規化、`/t_resolution` の floor)。
/// 範囲チェックはしない (呼び出し側で `in_map_area` / it 範囲を確認すること)。
pub fn pose_to_cell(vi: &ValueIterator, x: f64, y: f64, yaw_rad: f64) -> (i32, i32, i32) {
    let ix = ((x - vi.map_origin_x) / vi.xy_resolution).floor() as i32;
    let iy = ((y - vi.map_origin_y) / vi.xy_resolution).floor() as i32;
    let t = (180.0 * yaw_rad / PI) as i32;
    let it = (((t + 360 * 100) % 360) as f64 / vi.t_resolution).floor() as i32;
    (ix, iy, it)
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
    vi: &ValueIterator,
    x: f64,
    y: f64,
    yaw_rad: f64,
    tolerance_cells: i32,
) -> Option<(f64, f64)> {
    let (ix0, iy0, it) = pose_to_cell(vi, x, y, yaw_rad);
    let mut best: Option<(i64, i32, i32)> = None;
    for dy in -tolerance_cells..=tolerance_cells {
        for dx in -tolerance_cells..=tolerance_cells {
            let (ix, iy) = (ix0 + dx, iy0 + dy);
            if optimal_action_at(vi, ix, iy, it) < 0 {
                // final_state セル (既にゴール圏内) も救済対象に含める。
                let inside = vi.in_map_area(ix, iy)
                    && it >= 0
                    && it < vi.cell_num_t
                    && vi.states[vi.to_index(ix, iy, it) as usize].final_state;
                if !inside {
                    continue;
                }
            }
            let d2 = (dx as i64) * (dx as i64) + (dy as i64) * (dy as i64);
            if best.map(|(bd, _, _)| d2 < bd).unwrap_or(true) {
                best = Some((d2, ix, iy));
            }
        }
    }
    best.map(|(_, ix, iy)| {
        (
            (ix as f64 + 0.5) * vi.xy_resolution + vi.map_origin_x,
            (iy as f64 + 0.5) * vi.xy_resolution + vi.map_origin_y,
        )
    })
}

/// 貪欲方策ロールアウト。solve 済み (`set_goal` 後に収束させた) `ValueIterator` の
/// 方策を start 姿勢から辿り、世界座標の姿勢列を返す。
///
/// - 1 ステップ = 現在セルの最適行動を 1 回適用 (`no_noise_state_transition` と同じ
///   「並進→回転」)。姿勢間隔はアクション設計 (既定 0.2–0.3 m / 20°) に従う。
///   Nav2 等で密な経路が要る場合は [`densify`] を併用する。
/// - `final_state` セルに入った時点で成功とし、末尾に正確なゴール姿勢
///   (`goal_x`, `goal_y`, `goal_t`) を追加する (goal_margin ぶん手前で経路が
///   途切れないようにするため)。
/// - `start_tolerance_cells > 0` なら、start が方策を持たないセルのとき同一 θ の
///   xy 近傍から計画可能な最近傍セル中心へスナップして開始する。
pub fn rollout_path(
    vi: &ValueIterator,
    start_x: f64,
    start_y: f64,
    start_yaw_rad: f64,
    max_steps: usize,
    start_tolerance_cells: i32,
) -> Rollout {
    let (mut x, mut y) = (start_x, start_y);
    // 内部では度で保持 (本家の遷移生成・セル変換が度基準のため)。
    let mut yaw_deg = normalize_deg(start_yaw_rad.to_degrees());

    // start 救済: 現セルに方策が無ければ近傍の計画可能セル中心へスナップ。
    {
        let (ix, iy, it) = pose_to_cell(vi, x, y, start_yaw_rad);
        let on_final = vi.in_map_area(ix, iy)
            && it >= 0
            && it < vi.cell_num_t
            && vi.states[vi.to_index(ix, iy, it) as usize].final_state;
        if !on_final && optimal_action_at(vi, ix, iy, it) < 0 && start_tolerance_cells > 0 {
            if let Some((sx, sy)) = find_plannable_start(vi, x, y, start_yaw_rad, start_tolerance_cells)
            {
                x = sx;
                y = sy;
            }
        }
    }

    let mut poses = vec![PathPose { x, y, yaw: yaw_deg.to_radians() }];
    let mut visits: HashMap<(i32, i32, i32), u32> = HashMap::new();

    for _ in 0..max_steps {
        let (ix, iy, it) = pose_to_cell(vi, x, y, yaw_deg.to_radians());
        if !vi.in_map_area(ix, iy) || it < 0 || it >= vi.cell_num_t {
            return Rollout { poses, status: RolloutStatus::OutOfMap };
        }
        let s = &vi.states[vi.to_index(ix, iy, it) as usize];
        if s.final_state {
            // ゴール圏に入った: 末尾を正確なゴール姿勢で締める。
            poses.push(PathPose {
                x: vi.goal_x,
                y: vi.goal_y,
                yaw: (vi.goal_t as f64).to_radians(),
            });
            return Rollout { poses, status: RolloutStatus::ReachedGoal };
        }
        let Some(ai) = s.optimal_action.filter(|_| s.free) else {
            return Rollout { poses, status: RolloutStatus::NoAction };
        };

        let count = visits.entry((ix, iy, it)).or_insert(0);
        *count += 1;
        if *count > REVISIT_LIMIT {
            return Rollout { poses, status: RolloutStatus::LoopDetected };
        }

        // no_noise_state_transition と同じ: 現在向きで並進 → 回転。
        let a = &vi.actions[ai];
        let ang = yaw_deg.to_radians();
        x += a.delta_fw * ang.cos();
        y += a.delta_fw * ang.sin();
        yaw_deg = normalize_deg(yaw_deg + a.delta_rot);
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
fn normalize_deg(mut d: f64) -> f64 {
    d %= 360.0;
    if d < 0.0 {
        d += 360.0;
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action::Action;
    use crate::msg::{OccupancyGrid, Quaternion};
    use crate::solvers::{solve, U64Solver};

    const RES: f64 = 0.05;
    const NT: i32 = 60;

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
        let r = rollout_path(&vi, 0.6, 0.6, 0.0, 10_000, 0);
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
        let r = rollout_path(&vi, 0.6, 0.6, 0.0, 10_000, 0);
        assert!(r.reached_goal(), "status = {:?}", r.status);
    }

    #[test]
    fn path_avoids_obstacle_cells() {
        let size = 64;
        let goal = (2.8, 0.6, 0);
        let vi = solved_vi(size, size, walled_map(size), goal, U64Solver::Frontier3D);
        let r = rollout_path(&vi, 0.4, 0.4, 0.0, 10_000, 0);
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
        let r = rollout_path(&vi, 0.4, 0.4, 0.0, 10_000, 0);
        assert_eq!(r.status, RolloutStatus::NoAction);
    }

    #[test]
    fn start_outside_map_reports_out_of_map() {
        let vi = solved_vi(32, 32, empty_map(32), (1.0, 1.0, 0), U64Solver::Frontier3D);
        let r = rollout_path(&vi, -5.0, -5.0, 0.0, 100, 0);
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
        let fail = rollout_path(&vi, sx, sy, 0.0, 10_000, 0);
        assert_eq!(fail.status, RolloutStatus::NoAction);
        let ok = rollout_path(&vi, sx, sy, 0.0, 10_000, 20);
        assert!(ok.reached_goal(), "status = {:?}", ok.status);
    }

    #[test]
    fn start_inside_goal_region_returns_immediately() {
        let goal = (1.0, 1.0, 0);
        let vi = solved_vi(32, 32, empty_map(32), goal, U64Solver::Frontier3D);
        let r = rollout_path(&vi, 1.0, 1.0, 0.0, 100, 0);
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
}
