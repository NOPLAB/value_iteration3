//! 解けた値関数を「連続に読む」制御層 — V̂ 三線形補間 + DWA 型軌道サンプリング。
//!
//! ソルバ・方策の意味論 (bit-exact) には一切触れない読み取り専用の層。solve 済みの
//! [`PolicyView`] から cmd_vel 相当の連続行動 `(v [m/s], ω [deg/s])` を引く。
//! 大域最適性は V̂ (終端コスト) が、滑らかさは連続候補集合が担保する二層構成で、
//! 6 離散行動の argmin (`vi_planner` の `decide`) の置き換え候補。
//!
//! - [`v_hat`] — (x,y) 双線形 × θ 周期線形の三線形補間。8 頂点に未到達/障害物
//!   (`MAX_COST`) が混ざる点は「評価不能 = `None`」(障害物際で安全側に倒す)。
//! - [`dwa_decide`] — (v, ω) 候補格子をユニサイクルモデルで `horizon_s` 前進
//!   シミュレーションし、**終端 V̂ 最小** (ゴール圏到達候補は最優先・早い到達優先)
//!   の候補を返す。占有セルに触れる候補・終端 V̂ が評価不能な候補は棄却。全滅
//!   なら `None` (呼び出し側が従来の離散 greedy へフォールバックする)。
//! - 実行系は 1 tick (`tick_s`) ごとに再決定する前提 (follow ループと同じ)。
//!
//! 評価に走行中の penalty 積算項を**入れない**のは意図的: V̂ は margin penalty を
//! 含む将来コストを既に符号化しており (VI は action の着地セルにだけ penalty を
//! 課すので、0.3 m の 1 手で margin 帯を跳び越える経路は VI 会計では無料)、走行中
//! の滞在を tick 課金で再徴収すると V̂ が推す経路を DWA が拒否して帯の縁で膠着
//! する (壁角テストで実測)。安全 (占有セル非侵入) は衝突棄却が担う。margin 帯の
//! 一時的な通過露出は本家 greedy (方策追従) も同様に持つ性質。

use crate::action::Action;
use crate::params::MAX_COST;
use crate::planner::PolicyView;
use crate::value_iterator::ValueIterator;

/// 衝突・ペナルティ判定に使う地図コスト面。`free`/`penalty` は θ 非依存なので
/// 2 次元 (ix, iy) で読む。compact 経路ではパッチ/静的地図から実装できる。
pub trait CostView {
    /// (ix, iy) が free (非占有) か。範囲外は false。
    fn free_at(&self, ix: i32, iy: i32) -> bool;
    /// (ix, iy) の penalty + local_penalty (18bit 固定小数点)。範囲外は 0
    /// (`free_at` が先に落とす前提)。
    fn penalty_at(&self, ix: i32, iy: i32) -> u64;
}

impl CostView for ValueIterator {
    fn free_at(&self, ix: i32, iy: i32) -> bool {
        self.in_map_area(ix, iy) && self.states[self.to_index(ix, iy, 0) as usize].free
    }
    fn penalty_at(&self, ix: i32, iy: i32) -> u64 {
        if !self.in_map_area(ix, iy) {
            return 0;
        }
        // free/penalty は θ 非依存 (State::from_occupancy は (x,y) だけで決まる) ので
        // θ=0 の State を読む。
        let s = &self.states[self.to_index(ix, iy, 0) as usize];
        s.penalty + s.local_penalty
    }
}

/// V̂ — 値関数の三線形補間 (x,y 双線形 × θ 周期線形)。
///
/// セル中心 (`origin + (i+0.5)·res`、θ は `(it+0.5)·t_res` 度) を格子点として
/// 補間する。8 頂点のどれかが `MAX_COST` 以上 (未到達・障害物・地図外) なら
/// `None`。ゴール圏は値 0 なので自然に引き込む。
pub fn v_hat(p: &dyn PolicyView, x: f64, y: f64, yaw_rad: f64) -> Option<f64> {
    let (ox, oy) = p.map_origin();
    let res = p.xy_resolution();
    let nt = p.cell_num().2;
    let t_res = p.t_resolution();

    let ux = (x - ox) / res - 0.5;
    let uy = (y - oy) / res - 0.5;
    let fxf = ux.floor();
    let fyf = uy.floor();
    let (ix0, fx) = (fxf as i32, ux - fxf);
    let (iy0, fy) = (fyf as i32, uy - fyf);

    let deg = normalize_deg(yaw_rad.to_degrees());
    let ut = deg / t_res - 0.5;
    let ftf = ut.floor();
    let ft = ut - ftf;
    let it0 = (((ftf as i32) % nt) + nt) % nt;
    let it1 = (it0 + 1) % nt;

    let mut acc = 0.0f64;
    for (dix, wx) in [(0, 1.0 - fx), (1, fx)] {
        for (diy, wy) in [(0, 1.0 - fy), (1, fy)] {
            for (it, wt) in [(it0, 1.0 - ft), (it1, ft)] {
                let v = p.value_at(ix0 + dix, iy0 + diy, it);
                if v >= MAX_COST {
                    return None;
                }
                acc += (v as f64) * wx * wy * wt;
            }
        }
    }
    Some(acc)
}

/// DWA 型サンプリングの設定。
#[derive(Clone, Debug)]
pub struct DwaConfig {
    /// 並進速度の候補範囲 [m/s] (負 = 後退許可)。
    pub v_min: f64,
    pub v_max: f64,
    /// 角速度の候補範囲 ±w_max_deg [deg/s]。
    pub w_max_deg: f64,
    /// v 候補数 (`v_min..=v_max` 等間隔)。
    pub n_v: usize,
    /// ω 候補数 (`-w_max..=w_max` 等間隔、奇数なら 0 を含む)。
    pub n_w: usize,
    /// 前進シミュレーション時間 [s]。
    pub horizon_s: f64,
    /// 制御周期 [s]。penalty の「1 action」換算単位。
    pub tick_s: f64,
    /// 衝突判定の弧長サンプリング間隔 [m]。0 = auto (セル解像度の半分)。
    pub collide_step_m: f64,
}

impl DwaConfig {
    /// 行動集合から速度範囲を取る (v = delta_fw [m/s]、ω = delta_rot [deg/s] —
    /// 本家 `decision` の cmd_vel 変換と同じ読み方)。候補数とホライズンは既定値。
    pub fn from_actions(actions: &[Action], tick_s: f64) -> Self {
        let v_max = actions.iter().map(|a| a.delta_fw).fold(0.0f64, f64::max);
        let v_min = actions.iter().map(|a| a.delta_fw).fold(0.0f64, f64::min);
        let w_max_deg = actions.iter().map(|a| a.delta_rot.abs()).fold(0.0f64, f64::max);
        Self {
            v_min,
            v_max,
            w_max_deg,
            n_v: 7,
            n_w: 11,
            horizon_s: 1.0,
            tick_s,
            collide_step_m: 0.0,
        }
    }
}

/// [`dwa_decide`] の選択結果。
#[derive(Clone, Copy, Debug)]
pub struct DwaChoice {
    /// 並進速度 [m/s]。
    pub v: f64,
    /// 角速度 [deg/s]。
    pub w_deg: f64,
    /// 評価値。非ゴール候補は終端 V̂ (18bit 固定小数点、≥0)。ゴール到達候補は
    /// `到達時刻 [s] − GOAL_BONUS` (負) — 常に非ゴール候補より優先され、
    /// ゴール候補同士は早く着く方が勝つ。
    pub cost: f64,
    /// ホライズン内にゴール圏 (`final_state`) へ入る候補か。
    pub hits_goal: bool,
}

/// ゴール到達候補を非ゴール候補より必ず優先させる番兵 (V̂ ≥ 0 なのでこれで足りる)。
const GOAL_BONUS: f64 = 1e18;

/// ユニサイクルモデルの 1 ステップ (定速 (v, ω) の閉形式弧積分)。
pub fn unicycle_step(x: f64, y: f64, yaw_rad: f64, v: f64, w_rad_s: f64, dt: f64) -> (f64, f64, f64) {
    if w_rad_s.abs() < 1e-9 {
        (x + v * dt * yaw_rad.cos(), y + v * dt * yaw_rad.sin(), yaw_rad)
    } else {
        let r = v / w_rad_s;
        let yaw2 = yaw_rad + w_rad_s * dt;
        (x + r * (yaw2.sin() - yaw_rad.sin()), y - r * (yaw2.cos() - yaw_rad.cos()), yaw2)
    }
}

/// (v, ω) 候補格子を評価して最良の連続行動を返す。
///
/// 現在セルが free でない、または全候補が棄却された (衝突・終端評価不能) 場合は
/// `None` — 呼び出し側は従来の離散 greedy (`decide` 相当) へフォールバックすること。
pub fn dwa_decide(
    p: &dyn PolicyView,
    cost: &dyn CostView,
    cfg: &DwaConfig,
    x: f64,
    y: f64,
    yaw_rad: f64,
) -> Option<DwaChoice> {
    let (ox, oy) = p.map_origin();
    let res = p.xy_resolution();
    let cix = ((x - ox) / res).floor() as i32;
    let ciy = ((y - oy) / res).floor() as i32;
    if !cost.free_at(cix, ciy) {
        return None;
    }
    let step = if cfg.collide_step_m > 0.0 { cfg.collide_step_m } else { res * 0.5 };

    let mut best: Option<DwaChoice> = None;
    for i in 0..cfg.n_v.max(1) {
        let rv = if cfg.n_v > 1 { i as f64 / (cfg.n_v - 1) as f64 } else { 1.0 };
        let v = cfg.v_min + (cfg.v_max - cfg.v_min) * rv;
        for j in 0..cfg.n_w.max(1) {
            let rw = if cfg.n_w > 1 { j as f64 / (cfg.n_w - 1) as f64 } else { 0.5 };
            let w_deg = -cfg.w_max_deg + 2.0 * cfg.w_max_deg * rw;
            let Some(c) = eval_candidate(p, cost, cfg, x, y, yaw_rad, v, w_deg, step) else {
                continue;
            };
            if best.map(|b| c.cost < b.cost).unwrap_or(true) {
                best = Some(c);
            }
        }
    }
    best
}

/// 1 候補 (v, ω) の前進シミュレーション評価。棄却は `None`。
#[allow(clippy::too_many_arguments)]
fn eval_candidate(
    p: &dyn PolicyView,
    cost: &dyn CostView,
    cfg: &DwaConfig,
    x: f64,
    y: f64,
    yaw_rad: f64,
    v: f64,
    w_deg: f64,
    step_m: f64,
) -> Option<DwaChoice> {
    let (ox, oy) = p.map_origin();
    let res = p.xy_resolution();
    // 時間刻み: 弧長 step_m を超えず、かつ tick_s 以下 (回転のみでもゴール判定を刻む)。
    let dt = if v.abs() > 1e-9 { (step_m / v.abs()).min(cfg.tick_s) } else { cfg.tick_s };
    let n = (cfg.horizon_s / dt).ceil().max(1.0) as usize;
    let w_rad = w_deg.to_radians();

    let (mut px, mut py, mut pyaw) = (x, y, yaw_rad);
    for k in 0..n {
        let (nx2, ny2, nyaw) = unicycle_step(px, py, pyaw, v, w_rad, dt);
        px = nx2;
        py = ny2;
        pyaw = nyaw;
        let ix = ((px - ox) / res).floor() as i32;
        let iy = ((py - oy) / res).floor() as i32;
        if !cost.free_at(ix, iy) {
            return None; // 地図外 or 占有セル。
        }
        if p.is_final(ix, iy, theta_cell(p, pyaw)) {
            // ゴール圏に入れる候補: そこで停まれる。早い到達ほど低コスト。
            let t_hit = (k + 1) as f64 * dt;
            return Some(DwaChoice { v, w_deg, cost: t_hit - GOAL_BONUS, hits_goal: true });
        }
    }
    let vh = v_hat(p, px, py, pyaw)?;
    Some(DwaChoice { v, w_deg, cost: vh, hits_goal: false })
}

/// yaw [rad] → θ セル (本家 `posToAction` の θ 変換と同じ i32 切り捨て + 正規化)。
fn theta_cell(p: &dyn PolicyView, yaw_rad: f64) -> i32 {
    let t = (180.0 * yaw_rad / std::f64::consts::PI) as i32;
    ((((t + 360 * 100) % 360) as f64) / p.t_resolution()).floor() as i32
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
    use crate::planner::pose_to_cell;
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

    fn solved_vi(width: i32, height: i32, data: Vec<i8>, goal: (f64, f64, i32)) -> ValueIterator {
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
        let stats = solve(&mut vi, U64Solver::Frontier3D, 100_000);
        assert!(stats.converged, "test map must converge");
        vi
    }

    fn empty_map(size: i32) -> Vec<i8> {
        vec![0i8; (size * size) as usize]
    }

    /// 前進 1 ステップ (0.3 m = 6 セル) で飛び越えられない厚さの縦壁 (上端だけ開口)。
    fn walled_map(size: i32) -> Vec<i8> {
        let mut data = empty_map(size);
        let wall_x = size / 2;
        for y in 0..(size - 8) {
            for dx in 0..8 {
                data[(y * size + wall_x + dx) as usize] = 100;
            }
        }
        data
    }

    /// DWA を 1 tick ごとに再決定しながらユニサイクルで実行する (テスト用 follow ループ)。
    /// 経路上の全セルが free であることも検証する。
    fn dwa_follow(
        vi: &ValueIterator,
        start: (f64, f64, f64),
        max_ticks: usize,
    ) -> (bool, Vec<(f64, f64, f64)>) {
        use crate::planner::PolicyView as _;
        let cfg = DwaConfig::from_actions(&vi.actions, 0.1);
        let (mut x, mut y, mut yaw) = start;
        let mut traj = vec![start];
        for _ in 0..max_ticks {
            let (ix, iy, it) = pose_to_cell(vi, x, y, yaw);
            assert!(
                CostView::free_at(vi, ix, iy),
                "trajectory entered a non-free cell at ({x:.3}, {y:.3})"
            );
            if vi.is_final(ix, iy, it) {
                return (true, traj);
            }
            let Some(c) = dwa_decide(vi, vi, &cfg, x, y, yaw) else {
                return (false, traj);
            };
            let (nx2, ny2, nyaw) = unicycle_step(x, y, yaw, c.v, c.w_deg.to_radians(), cfg.tick_s);
            x = nx2;
            y = ny2;
            yaw = nyaw;
            traj.push((x, y, yaw));
        }
        (false, traj)
    }

    #[test]
    fn v_hat_matches_cell_value_at_centers() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0));
        use crate::planner::PolicyView as _;
        for (ix, iy, it) in [(10, 10, 5), (30, 20, 0), (5, 40, 59)] {
            let x = (ix as f64 + 0.5) * RES;
            let y = (iy as f64 + 0.5) * RES;
            let yaw = ((it as f64 + 0.5) * vi.t_resolution).to_radians();
            let exact = vi.value_at(ix, iy, it) as f64;
            let interp = v_hat(&vi, x, y, yaw).expect("interior cell must interpolate");
            let tol = exact.abs() * 1e-9 + 1e-3;
            assert!(
                (interp - exact).abs() <= tol,
                "cell ({ix},{iy},{it}): interp {interp} != exact {exact}"
            );
        }
    }

    #[test]
    fn v_hat_is_periodic_in_theta() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0));
        let (x, y) = (1.0, 1.0);
        for yaw in [0.1, 1.0, 3.0, -2.0] {
            let a = v_hat(&vi, x, y, yaw).unwrap();
            let b = v_hat(&vi, x, y, yaw + 2.0 * std::f64::consts::PI).unwrap();
            assert!((a - b).abs() <= a.abs() * 1e-9 + 1e-6, "yaw {yaw}: {a} != {b}");
        }
        // θ ラップ (it0=59, it1=0) を跨ぐ点も評価できること。
        assert!(v_hat(&vi, x, y, 0.5f64.to_radians()).is_some());
    }

    #[test]
    fn v_hat_rejects_points_next_to_unreached_cells() {
        let size = 64;
        let vi = solved_vi(size, size, walled_map(size), (2.8, 0.6, 0));
        // 壁 (x セル 32..40) の左隣 (セル 31) で壁向きの重みが立つ点 → 頂点に
        // 障害物セル (MAX_COST) が入り None。
        let x = 31.8 * RES;
        let y = 1.0;
        assert!(v_hat(&vi, x, y, 0.0).is_none());
        // 壁から離れた点は評価できる。
        assert!(v_hat(&vi, 20.0 * RES, y, 0.0).is_some());
    }

    #[test]
    fn dwa_picks_a_candidate_and_descends_v_hat() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0));
        let cfg = DwaConfig::from_actions(&vi.actions, 0.1);
        let start = (0.6, 0.6, 0.0);
        let c = dwa_decide(&vi, &vi, &cfg, start.0, start.1, start.2).expect("open space");
        assert!(c.cost.is_finite());
        // 50 tick 実行して V̂ が下がっていること (ゴールへ向かう)。
        let (_, traj) = dwa_follow(&vi, start, 50);
        let v0 = v_hat(&vi, start.0, start.1, start.2).unwrap();
        let last = traj.last().unwrap();
        let v1 = v_hat(&vi, last.0, last.1, last.2).unwrap();
        assert!(v1 < v0, "V̂ did not decrease: {v0} -> {v1}");
    }

    #[test]
    fn dwa_follow_reaches_goal_on_empty_map() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0));
        let (reached, traj) = dwa_follow(&vi, (0.6, 0.6, 0.0), 3000);
        assert!(reached, "did not reach goal in 3000 ticks ({} poses)", traj.len());
    }

    #[test]
    fn dwa_follow_avoids_wall_and_reaches_goal() {
        let size = 64;
        let vi = solved_vi(size, size, walled_map(size), (2.8, 0.6, 0));
        // 壁の左側から出発 → 上端の開口を回ってゴールへ。free 検証は dwa_follow 内。
        let (reached, traj) = dwa_follow(&vi, (0.4, 0.4, 0.0), 6000);
        assert!(reached, "did not reach goal in 6000 ticks ({} poses)", traj.len());
    }

    #[test]
    fn dwa_on_non_free_cell_returns_none() {
        let size = 64;
        let vi = solved_vi(size, size, walled_map(size), (2.8, 0.6, 0));
        // 壁セルの中心に置く → 判断不能 (呼び出し側が greedy 救済へ)。
        let cfg = DwaConfig::from_actions(&vi.actions, 0.1);
        let x = 34.5 * RES;
        let y = 1.0;
        assert!(dwa_decide(&vi, &vi, &cfg, x, y, 0.0).is_none());
    }
}
