//! u64 コストモデル上で動く高速 VI ソルバ群。各ソルバは本家の per-cell 更新
//! `value_iteration_raw` を活性集合に対して呼ぶ。コスト数式は不変なので、到達可能
//! セルの収束値は Reference (全走査) = 本家と bit-exact。
//! 設計: `docs/superpowers/specs/2026-06-09-vi-u64-fast-solvers-design.md`

use crate::params::MAX_COST;
use crate::value_iterator::ValueIterator;

// フロンティアには実績ある word 並列 Bitboard を再利用する（u16 frontier の高速化の源）。
// Bitboard は値の型に非依存なので u64 モデルでもそのまま使える。dilate は theta periodic。
pub(crate) use vi_algorithm::bitboard::{Bitboard2D, Bitboard3D};

pub mod block;
pub mod coarse_theta;
pub mod frontier2d;
pub mod frontier2d_pad;
pub mod frontier2d_par;
pub mod frontier2d_soa;
#[cfg(test)]
mod measure;
pub mod frontier3d;
pub mod pyramid;
pub mod stack;
pub mod stream;
pub mod tau;
pub mod topk;

/// dilation 変位 `(mx, my, mt)` を `actions` の全遷移から算出する。`dit` は絶対 θ なので、
/// 各 (action, source theta `t`) について循環距離 `min(|dit-t|, nt-|dit-t|)` を取り `mt` とする。
/// これは「あるセルが変化したとき再評価が必要な前駆セル集合」の正しい上位集合を与える。
pub(crate) fn displacement(vi: &ValueIterator) -> (i32, i32, i32) {
    let nt = vi.cell_num_t;
    let (mut mx, mut my, mut mt) = (0i32, 0i32, 0i32);
    for a in &vi.actions {
        for (t, trans) in a.state_transitions.iter().enumerate() {
            for st in trans {
                mx = mx.max(st.dix.abs());
                my = my.max(st.diy.abs());
                let raw = (st.dit - t as i32).rem_euclid(nt);
                let circ = raw.min(nt - raw);
                mt = mt.max(circ);
            }
        }
    }
    (mx.max(1), my.max(1), mt)
}

/// 初期フロンティア種: `total_cost < MAX_COST` のセル（`set_goal` 後の `final_state` セル）。
pub(crate) fn seed_frontier(vi: &ValueIterator) -> Bitboard3D {
    let mut bb = Bitboard3D::new(vi.cell_num_x as u32, vi.cell_num_y as u32, vi.cell_num_t as u32);
    for s in &vi.states {
        if s.total_cost < MAX_COST {
            bb.set(s.ix as u32, s.iy as u32, s.it as u32);
        }
    }
    bb
}

/// 初期フロンティア種 (2D): いずれかの θ で `total_cost < MAX_COST` の (ix,iy)。
pub(crate) fn seed_frontier_2d(vi: &ValueIterator) -> Bitboard2D {
    let mut bb = Bitboard2D::new(vi.cell_num_x as u32, vi.cell_num_y as u32);
    for s in &vi.states {
        if s.total_cost < MAX_COST {
            bb.set(s.ix as u32, s.iy as u32);
        }
    }
    bb
}

/// 到達可能とみなす total_cost 上限（compare.py の value>=1e6 境界と整合）。
pub(crate) const REACH_THRESH: u64 = 1_000_000u64 * crate::params::PROB_BASE;

/// u64 高速ソルバの種別。近似ソルバは no-op パラメータ（tau=0 / k=全 outcome / step=1）で
/// Frontier3D と等価（bit-exact）になり、移植の正しさを検証できる。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum U64Solver {
    Reference,
    Frontier3D,
    Frontier2D,
    Frontier2DSoA,
    Frontier2DPad,
    Frontier2DPar,
    FrontierStack,
    BlockRefine,
    PyramidSweep,
    Frontier3DTau { tau: u64 },
    Frontier3DTopK { k: u32 },
    Frontier3DCoarseTheta { step: u32 },
    StreamMimic,
}

impl U64Solver {
    pub fn from_name(s: &str) -> Option<Self> {
        Some(match s {
            "reference" => U64Solver::Reference,
            "frontier3d" => U64Solver::Frontier3D,
            "frontier2d" => U64Solver::Frontier2D,
            "frontier2d_soa" => U64Solver::Frontier2DSoA,
            "frontier2d_pad" => U64Solver::Frontier2DPad,
            "frontier2d_par" => U64Solver::Frontier2DPar,
            "frontier_stack" => U64Solver::FrontierStack,
            "block_refine" => U64Solver::BlockRefine,
            "pyramid_sweep" => U64Solver::PyramidSweep,
            // 近似ソルバ: 既定は no-op（= Frontier3D 等価）。実用近似は param 指定で。
            "frontier3d_tau" => U64Solver::Frontier3DTau { tau: 0 },
            "frontier3d_topk" => U64Solver::Frontier3DTopK { k: u32::MAX },
            "frontier3d_coarse_theta" => U64Solver::Frontier3DCoarseTheta { step: 1 },
            "stream_mimic" => U64Solver::StreamMimic,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct U64SolveStats {
    pub iters: u32,
    pub updates: u64,
    pub converged: bool,
}

/// Reference は全走査を strict 固定点（到達可能セルが不変）まで回す。
pub(crate) fn reference_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let mut prev: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
    let mut iters = 0u32;
    let converged = loop {
        vi.value_iteration_worker(1, 0);
        iters += 1;
        let mut changed = false;
        for (i, s) in vi.states.iter().enumerate() {
            if s.total_cost < REACH_THRESH && s.total_cost != prev[i] {
                changed = true;
            }
            prev[i] = s.total_cost;
        }
        if !changed {
            break true;
        }
        if iters >= max_iter {
            break false;
        }
    };
    (iters, 0, converged)
}

/// セット済み `ValueIterator` を指定ソルバで収束まで解く。
pub fn solve(vi: &mut ValueIterator, solver: U64Solver, max_iter: u32) -> U64SolveStats {
    let (iters, updates, converged) = match solver {
        U64Solver::Reference => reference_solve(vi, max_iter),
        U64Solver::Frontier3D => frontier3d::frontier3d_solve(vi, max_iter),
        U64Solver::Frontier2D => frontier2d::frontier2d_solve(vi, max_iter),
        U64Solver::Frontier2DSoA => frontier2d_soa::frontier2d_soa_solve(vi, max_iter),
        U64Solver::Frontier2DPad => frontier2d_pad::frontier2d_pad_solve(vi, max_iter),
        U64Solver::Frontier2DPar => frontier2d_par::frontier2d_par_solve(vi, max_iter),
        U64Solver::FrontierStack => stack::frontier_stack_solve(vi, max_iter),
        U64Solver::BlockRefine => block::block_refine_solve(vi, max_iter),
        U64Solver::PyramidSweep => pyramid::pyramid_sweep_solve(vi, max_iter),
        U64Solver::Frontier3DTau { tau } => tau::frontier3d_tau_solve(vi, tau, max_iter),
        U64Solver::Frontier3DTopK { k } => topk::frontier3d_topk_solve(vi, k, max_iter),
        U64Solver::Frontier3DCoarseTheta { step } => {
            coarse_theta::frontier3d_coarse_theta_solve(vi, step, max_iter)
        }
        U64Solver::StreamMimic => stream::stream_mimic_solve(vi, max_iter),
    };
    U64SolveStats { iters, updates, converged }
}

/// フロンティア/ブロック系ソルバの parity テスト共有ヘルパ。
#[cfg(test)]
pub(crate) mod test_support {
    use crate::action::Action;
    use crate::msg::OccupancyGrid;
    use crate::params::PROB_BASE;
    use crate::value_iterator::ValueIterator;

    pub(crate) const REACH: u64 = 1_000_000u64 * PROB_BASE;

    pub(crate) fn actions() -> Vec<Action> {
        vec![
            Action::new("forward", 0.3, 0.0, 0),
            Action::new("back", -0.2, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2),
            Action::new("rightfw", 0.2, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4),
            Action::new("leftfw", 0.2, 20.0, 5),
        ]
    }

    pub(crate) fn make_vi(w: i32, h: i32, occ: Vec<i8>) -> ValueIterator {
        let mut vi = ValueIterator::new(actions(), 1);
        let map = OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: occ,
        };
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.3, 15);
        vi.set_goal(0.10, 0.10, 0);
        vi
    }

    /// Reference 全走査を strict 固定点（到達可能セルが変化しなくなる）まで回す。
    pub(crate) fn run_reference_to_fixed_point(vi: &mut ValueIterator) {
        let mut prev: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
        for _ in 0..2000 {
            vi.value_iteration_worker(1, 0);
            let mut changed = false;
            for (i, s) in vi.states.iter().enumerate() {
                if s.total_cost < REACH && s.total_cost != prev[i] {
                    changed = true;
                }
                prev[i] = s.total_cost;
            }
            if !changed {
                break;
            }
        }
    }

    /// `solve_fn` で解いた結果が Reference 固定点と到達可能セルで bit 一致することを assert。
    pub(crate) fn assert_parity<F>(w: i32, h: i32, occ: Vec<i8>, solve_fn: F)
    where
        F: Fn(&mut ValueIterator) -> (u32, u64, bool),
    {
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);
        let (_i, _u, converged) = solve_fn(&mut b);
        assert!(converged, "solver must converge");
        let mut n_reach = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH {
                n_reach += 1;
                assert_eq!(
                    a.states[i].total_cost, b.states[i].total_cost,
                    "total_cost mismatch @ state {i} (ix={},iy={},it={})",
                    a.states[i].ix, a.states[i].iy, a.states[i].it
                );
                assert_eq!(
                    a.states[i].optimal_action, b.states[i].optimal_action,
                    "policy mismatch @ state {i} (ix={},iy={},it={})",
                    a.states[i].ix, a.states[i].iy, a.states[i].it
                );
            }
        }
        assert!(n_reach > 0, "到達可能セルが存在するはず");
    }

    /// 標準の3マップ (empty / obstacle / sentinel) で parity を検証する共通テスト本体。
    pub(crate) fn parity_standard_maps<F>(solve_fn: F)
    where
        F: Fn(&mut ValueIterator) -> (u32, u64, bool) + Copy,
    {
        // empty 8x8
        assert_parity(8, 8, vec![0i8; 64], solve_fn);
        // obstacle: x=5 の縦壁 (隙間あり)
        let mut occ = vec![0i8; 64];
        for iy in 0..8 {
            occ[(iy * 8 + 5) as usize] = 100;
        }
        occ[5] = 0;
        assert_parity(8, 8, occ, solve_fn);
        // sentinel: goal(2,2) を3方向で囲む
        let mut occ = vec![0i8; 64];
        occ[(1 * 8 + 2) as usize] = 100;
        occ[(3 * 8 + 2) as usize] = 100;
        occ[(2 * 8 + 1) as usize] = 100;
        assert_parity(8, 8, occ, solve_fn);
    }
}

#[cfg(test)]
mod helper_tests {
    use super::*;
    use crate::action::Action;
    use crate::msg::OccupancyGrid;
    use crate::value_iterator::ValueIterator;

    fn small_vi() -> ValueIterator {
        let actions = vec![
            Action::new("forward", 0.3, 0.0, 0),
            Action::new("back", -0.2, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2),
            Action::new("rightfw", 0.2, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4),
            Action::new("leftfw", 0.2, 20.0, 5),
        ];
        let mut vi = ValueIterator::new(actions, 1);
        let map = OccupancyGrid {
            width: 5,
            height: 5,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: vec![0i8; 25],
        };
        // theta_cell_num=60 (production と同じ)。粗いと goal の向き判定が成立しない。
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.3, 15);
        vi.set_goal(0.10, 0.10, 0);
        vi
    }

    #[test]
    fn displacement_is_bounded_and_positive() {
        let vi = small_vi();
        let (mx, my, mt) = displacement(&vi);
        assert!(mx >= 1 && my >= 1);
        assert!(mt >= 0 && mt < vi.cell_num_t);
    }

    #[test]
    fn seed_contains_goal_cells() {
        let vi = small_vi();
        let seed = seed_frontier(&vi);
        let n_final = vi.states.iter().filter(|s| s.total_cost < crate::params::MAX_COST).count();
        assert!(n_final > 0, "goal セルが存在するはず");
        assert_eq!(seed.popcount(), n_final as u64);
    }

    #[test]
    fn solve_reference_and_frontier3d_agree() {
        let mut a = small_vi();
        let mut b = small_vi();
        solve(&mut a, U64Solver::Reference, 2000);
        solve(&mut b, U64Solver::Frontier3D, 2000);
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH_THRESH {
                assert_eq!(a.states[i].total_cost, b.states[i].total_cost, "@ {i}");
                assert_eq!(a.states[i].optimal_action, b.states[i].optimal_action, "@ {i}");
            }
        }
    }

    #[test]
    fn solver_from_str() {
        assert!(matches!(U64Solver::from_name("frontier3d"), Some(U64Solver::Frontier3D)));
        assert!(matches!(U64Solver::from_name("reference"), Some(U64Solver::Reference)));
        assert!(U64Solver::from_name("nope").is_none());
    }
}
