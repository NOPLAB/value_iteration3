//! 全 `U64Solver` 変種の共通 conformance テスト。散在していた per-solver
//! `parity_standard_maps_*` テストの置き換えで、**全ソルバ**を同じ観点でゲートする:
//!
//! 1. **収束**: 標準 3 マップ (empty / obstacle / sentinel) で `converged`。
//! 2. **正しさ** (caps 駆動):
//!    - `caps().exact` → Reference 固定点と bit-exact (値 + 方策)。
//!    - 近似 (`exact == false`) → Reference が到達する全セルへ到達し、
//!      `partial == UpperBound` なら全セルで V ≥ v* (妥当な Bellman 更新のみ =
//!      上界不変条件)。
//! 3. **境界観測**: `boundary` が少なくとも 1 回呼ばれ、`iters` が単調非減少で、
//!    プローブの場が Partiality の不変条件 (UpperBound: V ≥ v* / ExactPrefix:
//!    V ∈ {v*, MAX_COST}) を**solve の途中でも**満たす。
//! 4. **cancel 即時性**: 最初の境界で `Cancel` → `cancelled=true` で即座に戻る
//!    (priority 系は従来これが不可能だった)。
//! 5. **Stop 健全性 (early_start)**: 「起点からゴールへ方策が繋がった」時点で `Stop`
//!    した場の上で、実際に `rollout_path_on` がゴールへ着く。
//!
//! Reference 固定点 (本家全走査) が全ソルバ共通のオラクル。

use crate::params::MAX_COST;
use crate::planner::rollout_path_on;
use crate::solvers::observe::{SolveFlow, SolveObserver, SolveProbe};
use crate::solvers::test_support::{make_vi, run_reference_to_fixed_point, REACH};
use crate::solvers::{solve, solve_observed, Partiality, U64Solver};
use crate::value_iterator::ValueIterator;

/// テスト対象の全変種。
fn all_solvers() -> Vec<(&'static str, U64Solver)> {
    use U64Solver::*;
    vec![
        ("reference", Reference),
        ("frontier3d", Frontier3D),
        ("frontier2d", Frontier2D),
        ("frontier2d_par", Frontier2DPar),
        ("frontier2d_par_unsafe", Frontier2DParUnsafe),
        ("frontier2d_fused", Frontier2DFused),
        ("frontier2d_sparse", Frontier2DSparse),
        ("frontier2d_sparse_compact", Frontier2DSparseCompact { band: 0 }),
        ("frontier_stack", FrontierStack),
        ("block_refine", BlockRefine),
        ("pyramid_sweep", PyramidSweep),
        ("stream_mimic", StreamMimic),
        ("prio_ls", PriorityLabelSetting),
        ("prio_lc", PriorityLabelCorrecting),
    ]
}

/// 標準 3 マップ (test_support::parity_standard_maps と同じ構成)。
fn standard_maps() -> Vec<(&'static str, i32, i32, Vec<i8>)> {
    let empty = vec![0i8; 64];
    let mut obstacle = vec![0i8; 64];
    for iy in 0..8 {
        obstacle[(iy * 8 + 5) as usize] = 100;
    }
    obstacle[5] = 0;
    let mut sentinel = vec![0i8; 64];
    sentinel[(1 * 8 + 2) as usize] = 100;
    sentinel[(3 * 8 + 2) as usize] = 100;
    sentinel[(2 * 8 + 1) as usize] = 100;
    vec![
        ("empty", 8, 8, empty),
        ("obstacle", 8, 8, obstacle),
        ("sentinel", 8, 8, sentinel),
    ]
}

fn exact_fixed_point(w: i32, h: i32, occ: &[i8]) -> ValueIterator {
    let mut vi = make_vi(w, h, occ.to_vec());
    run_reference_to_fixed_point(&mut vi);
    vi
}

/// 到達可能セルの最遠点 (Reference 固定点で最大値) をセル中心のポーズにして返す。
/// Stop 健全性テストのロールアウト起点。
fn farthest_reachable_pose(exact: &ValueIterator) -> (f64, f64, f64) {
    let s = exact
        .states
        .iter()
        .filter(|s| s.free && !s.final_state && s.total_cost < REACH)
        .max_by_key(|s| s.total_cost)
        .expect("到達可能セルが存在するはず");
    let res = exact.xy_resolution;
    let x = exact.map_origin_x + (s.ix as f64 + 0.5) * res;
    let y = exact.map_origin_y + (s.iy as f64 + 0.5) * res;
    let yaw = (s.it as f64 * exact.t_resolution).to_radians();
    (x, y, yaw)
}

const MAX_ITER: u32 = 4000;
const ROLLOUT_STEPS: usize = 4000;
const START_TOL: i32 = 2;

// ──────────────────────────────────────────────────────────────────────────────
// 1+2. 収束と正しさ (caps 駆動)
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn conformance_converges_and_matches_caps() {
    for (map_name, w, h, occ) in standard_maps() {
        let exact = exact_fixed_point(w, h, &occ);
        for (name, solver) in all_solvers() {
            let caps = solver.caps();
            let mut vi = make_vi(w, h, occ.clone());
            let stats = solve(&mut vi, solver, MAX_ITER);
            assert!(stats.converged, "{name}@{map_name}: must converge");

            let mut n_reach = 0u64;
            for i in 0..exact.states.len() {
                let a = &exact.states[i];
                let b = &vi.states[i];
                if a.total_cost < REACH {
                    n_reach += 1;
                    if caps.exact {
                        assert_eq!(
                            a.total_cost, b.total_cost,
                            "{name}@{map_name}: total_cost mismatch @ state {i} \
                             (ix={},iy={},it={})",
                            a.ix, a.iy, a.it
                        );
                        assert_eq!(
                            a.optimal_action, b.optimal_action,
                            "{name}@{map_name}: policy mismatch @ state {i} \
                             (ix={},iy={},it={})",
                            a.ix, a.iy, a.it
                        );
                    } else {
                        // 近似でも「Reference が到達する集合」へは到達すること。
                        assert!(
                            b.total_cost < REACH,
                            "{name}@{map_name}: 近似ソルバが到達可能セルへ未到達 @ {i} \
                             (ix={},iy={},it={})",
                            a.ix, a.iy, a.it
                        );
                    }
                }
                // 上界不変条件 (UpperBound を宣言する全ソルバ、近似含む)。
                if caps.partial == Partiality::UpperBound {
                    assert!(
                        b.total_cost >= a.total_cost,
                        "{name}@{map_name}: V < v* (上界破れ) @ state {i}: {} < {}",
                        b.total_cost,
                        a.total_cost
                    );
                }
            }
            assert!(n_reach > 0, "{map_name}: 到達可能セルが存在するはず");
        }
    }
}

/// 並列ソルバは行バンド分割・work stealing が効く広いマップでも parity を確認する
/// (旧 fused/sparse/par_unsafe の larger-map テストの置き換え)。
#[test]
fn conformance_parallel_solvers_larger_map() {
    let (w, h) = (32, 24);
    let occ = vec![0i8; (w * h) as usize];
    let exact = exact_fixed_point(w, h, &occ);
    for (name, solver) in all_solvers() {
        if !solver.caps().parallel {
            continue;
        }
        let mut vi = make_vi(w, h, occ.clone());
        let stats = solve(&mut vi, solver, MAX_ITER);
        assert!(stats.converged, "{name}: must converge");
        for i in 0..exact.states.len() {
            let a = &exact.states[i];
            if a.total_cost < REACH {
                assert_eq!(a.total_cost, vi.states[i].total_cost, "{name}: value @ {i}");
                assert_eq!(
                    a.optimal_action, vi.states[i].optimal_action,
                    "{name}: policy @ {i}"
                );
            }
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// 3. 境界観測 (プローブの場が途中でも不変条件を満たす)
// ──────────────────────────────────────────────────────────────────────────────

struct InvariantObserver<'a> {
    exact: &'a ValueIterator,
    partial: Partiality,
    calls: u32,
    prev_iters: u32,
    violations: Vec<String>,
}

impl SolveObserver for InvariantObserver<'_> {
    fn interval(&self) -> u32 {
        1
    }
    fn boundary(&mut self, probe: &mut dyn SolveProbe) -> SolveFlow {
        self.calls += 1;
        let iters = probe.iters();
        if iters < self.prev_iters {
            self.violations
                .push(format!("iters 逆行: {} -> {}", self.prev_iters, iters));
        }
        self.prev_iters = iters;

        // 場の不変条件は数回に 1 回だけ検証 (毎境界で具現化すると O(n·境界数) が嵩む)。
        if self.calls % 3 != 1 {
            return SolveFlow::Continue;
        }
        let p = probe.policy();
        let (nx, ny, nt) = p.cell_num();
        for iy in 0..ny {
            for ix in 0..nx {
                for it in 0..nt {
                    let idx = (it + ix * nt + iy * nt * nx) as usize;
                    let a = self.exact.states[idx].total_cost;
                    let b = p.value_at(ix, iy, it);
                    let ok = match self.partial {
                        Partiality::UpperBound => b >= a,
                        Partiality::ExactPrefix => b == MAX_COST || b == a,
                    };
                    if !ok {
                        self.violations.push(format!(
                            "boundary#{}: ({ix},{iy},{it}) exact={a} probe={b} ({:?})",
                            self.calls, self.partial
                        ));
                        if self.violations.len() > 4 {
                            return SolveFlow::Cancel; // 打ち切って早期報告
                        }
                    }
                }
            }
        }
        SolveFlow::Continue
    }
}

#[test]
fn conformance_boundary_observation() {
    let (_, w, h, occ) = standard_maps().remove(0); // empty
    let exact = exact_fixed_point(w, h, &occ);
    for (name, solver) in all_solvers() {
        let mut vi = make_vi(w, h, occ.clone());
        let mut obs = InvariantObserver {
            exact: &exact,
            partial: solver.caps().partial,
            calls: 0,
            prev_iters: 0,
            violations: Vec::new(),
        };
        let out = solve_observed(&mut vi, solver, MAX_ITER, &mut obs);
        assert!(
            obs.violations.is_empty(),
            "{name}: 境界不変条件違反: {:?}",
            obs.violations
        );
        assert!(out.converged, "{name}: must converge (cancelled={})", out.cancelled);
        assert!(obs.calls >= 1, "{name}: boundary が一度も呼ばれていない");
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// 4. cancel 即時性
// ──────────────────────────────────────────────────────────────────────────────

struct CancelAtFirstBoundary {
    calls: u32,
}

impl SolveObserver for CancelAtFirstBoundary {
    fn interval(&self) -> u32 {
        1
    }
    fn boundary(&mut self, _probe: &mut dyn SolveProbe) -> SolveFlow {
        self.calls += 1;
        SolveFlow::Cancel
    }
}

#[test]
fn conformance_cancel_is_prompt() {
    let (_, w, h, occ) = standard_maps().remove(0); // empty
    for (name, solver) in all_solvers() {
        // 完走時の反復数 (単位はソルバ固有だが、同一ソルバ内では比較可能)。
        let mut vi_full = make_vi(w, h, occ.clone());
        let full = solve(&mut vi_full, solver, MAX_ITER);
        assert!(full.converged, "{name}: full run must converge");

        let mut vi = make_vi(w, h, occ.clone());
        let mut obs = CancelAtFirstBoundary { calls: 0 };
        let out = solve_observed(&mut vi, solver, MAX_ITER, &mut obs);
        assert_eq!(obs.calls, 1, "{name}: 最初の境界で止まるはず");
        assert!(out.cancelled, "{name}: cancelled が立つはず");
        assert!(!out.converged && !out.stopped, "{name}: cancel は converge/stop と排他");
        assert!(
            out.iters <= full.iters,
            "{name}: cancel 後の反復 {} が完走 {} を超えている",
            out.iters,
            full.iters
        );
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// 5. Stop 健全性 (early_start の核心不変条件)
// ──────────────────────────────────────────────────────────────────────────────

struct StopWhenReachable {
    start: (f64, f64, f64),
    stopped_at: Option<u32>,
}

impl SolveObserver for StopWhenReachable {
    fn interval(&self) -> u32 {
        1
    }
    fn boundary(&mut self, probe: &mut dyn SolveProbe) -> SolveFlow {
        let p = probe.policy();
        let r = rollout_path_on(p, self.start.0, self.start.1, self.start.2, ROLLOUT_STEPS, START_TOL);
        if r.reached_goal() {
            self.stopped_at = Some(probe.iters());
            SolveFlow::Stop
        } else {
            SolveFlow::Continue
        }
    }
}

#[test]
fn conformance_stop_yields_rollout_ready_field() {
    for (map_name, w, h, occ) in standard_maps() {
        let exact = exact_fixed_point(w, h, &occ);
        let start = farthest_reachable_pose(&exact);
        for (name, solver) in all_solvers() {
            let mut vi = make_vi(w, h, occ.clone());
            let mut obs = StopWhenReachable { start, stopped_at: None };
            let out = solve_observed(&mut vi, solver, MAX_ITER, &mut obs);
            assert!(
                out.stopped || out.converged,
                "{name}@{map_name}: stop も converge もしなかった"
            );
            // Stop した場 (または収束した場) の上で、実際に使う経路が引けること。
            // これは vi_planner の early_start が依存する不変条件そのもの。
            let r = rollout_path_on(&vi, start.0, start.1, start.2, ROLLOUT_STEPS, START_TOL);
            assert!(
                r.reached_goal(),
                "{name}@{map_name}: stopped={} の場からロールアウトが失敗: {:?}",
                out.stopped,
                r.status
            );
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// caps の明示的な期待 (実装とドキュメントのずれ検知)
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn caps_expectations() {
    use U64Solver::*;
    assert!(Reference.caps().exact);
    assert!(!PriorityLabelSetting.caps().exact);
    assert!(PriorityLabelCorrecting.caps().exact);
    let compact = Frontier2DSparseCompact { band: 0 }.caps();
    assert!(compact.exact && compact.out_of_core);
    assert_eq!(compact.partial, Partiality::ExactPrefix);
    assert!(Frontier2DSparse.caps().parallel);
    assert!(!Frontier2D.caps().out_of_core);
}
