//! u64 コストモデル上で動く高速 VI ソルバ群。各ソルバは本家の per-cell 更新
//! `value_iteration_raw` を活性集合に対して呼ぶ。コスト数式は不変なので、到達可能
//! セルの収束値は Reference (全走査) = 本家と bit-exact。
//! 設計: `docs/superpowers/specs/2026-06-09-vi-u64-fast-solvers-design.md`

use crate::params::MAX_COST;
use crate::value_iterator::ValueIterator;

pub mod frontier3d;

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
pub(crate) fn seed_frontier(vi: &ValueIterator) -> Bitset3D {
    let mut bb = Bitset3D::new(vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    for s in &vi.states {
        if s.total_cost < MAX_COST {
            bb.set(s.ix, s.iy, s.it);
        }
    }
    bb
}

/// 到達可能とみなす total_cost 上限（compare.py の value>=1e6 境界と整合）。
pub(crate) const REACH_THRESH: u64 = 1_000_000u64 * crate::params::PROB_BASE;

/// u64 高速ソルバの種別。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum U64Solver {
    Reference,
    Frontier3D,
    Frontier2D,
    FrontierStack,
    BlockRefine,
    PyramidSweep,
}

impl U64Solver {
    pub fn from_name(s: &str) -> Option<Self> {
        Some(match s {
            "reference" => U64Solver::Reference,
            "frontier3d" => U64Solver::Frontier3D,
            "frontier2d" => U64Solver::Frontier2D,
            "frontier_stack" => U64Solver::FrontierStack,
            "block_refine" => U64Solver::BlockRefine,
            "pyramid_sweep" => U64Solver::PyramidSweep,
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
fn reference_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
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
        // Phase 2/3 で実装次第、各 *_solve に差し替える。
        other => panic!("u64 solver not yet implemented: {other:?}"),
    };
    U64SolveStats { iters, updates, converged }
}

/// 索引 `it + ix*nt + iy*nt*nx`（本家 `to_index` と整合）のビット集合。
/// フロンティアの活性セル集合を表現する。
pub(crate) struct Bitset3D {
    nx: i32,
    ny: i32,
    nt: i32,
    words: Vec<u64>,
}

impl Bitset3D {
    pub(crate) fn new(nx: i32, ny: i32, nt: i32) -> Self {
        let n = (nx * ny * nt) as usize;
        Bitset3D { nx, ny, nt, words: vec![0u64; n.div_ceil(64)] }
    }
    #[inline]
    fn index(&self, ix: i32, iy: i32, it: i32) -> usize {
        (it + ix * self.nt + iy * self.nt * self.nx) as usize
    }
    pub(crate) fn set(&mut self, ix: i32, iy: i32, it: i32) {
        let i = self.index(ix, iy, it);
        self.words[i / 64] |= 1u64 << (i % 64);
    }
    pub(crate) fn test(&self, ix: i32, iy: i32, it: i32) -> bool {
        let i = self.index(ix, iy, it);
        (self.words[i / 64] >> (i % 64)) & 1 == 1
    }
    pub(crate) fn popcount(&self) -> u64 {
        self.words.iter().map(|w| w.count_ones() as u64).sum()
    }
    pub(crate) fn enumerate(&self) -> impl Iterator<Item = (i32, i32, i32)> + '_ {
        let (nx, ny, nt) = (self.nx, self.ny, self.nt);
        self.words.iter().enumerate().flat_map(move |(wi, &w)| {
            (0..64).filter_map(move |bit| {
                if (w >> bit) & 1 == 1 {
                    let i = (wi * 64 + bit) as i32;
                    let it = i % nt;
                    let ix = (i / nt) % nx;
                    let iy = i / (nt * nx);
                    if iy < ny { Some((ix, iy, it)) } else { None }
                } else {
                    None
                }
            })
        })
    }
    /// 空間 ±dx,±dy（境界クリップ）と θ ±dt（循環 wrap）で膨張した集合を返す。
    pub(crate) fn dilate(&self, dx: i32, dy: i32, dt: i32) -> Bitset3D {
        let mut out = Bitset3D::new(self.nx, self.ny, self.nt);
        for (ix, iy, it) in self.enumerate() {
            for ddx in -dx..=dx {
                let jx = ix + ddx;
                if jx < 0 || jx >= self.nx {
                    continue;
                }
                for ddy in -dy..=dy {
                    let jy = iy + ddy;
                    if jy < 0 || jy >= self.ny {
                        continue;
                    }
                    for ddt in -dt..=dt {
                        let jt = (it + ddt + self.nt) % self.nt;
                        out.set(jx, jy, jt);
                    }
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod bitset_tests {
    use super::Bitset3D;
    #[test]
    fn set_test_popcount_enumerate() {
        let mut b = Bitset3D::new(3, 2, 4); // nx=3, ny=2, nt=4
        assert_eq!(b.popcount(), 0);
        b.set(2, 1, 3);
        b.set(0, 0, 0);
        assert!(b.test(2, 1, 3));
        assert!(b.test(0, 0, 0));
        assert!(!b.test(1, 1, 1));
        assert_eq!(b.popcount(), 2);
        let mut cells: Vec<(i32, i32, i32)> = b.enumerate().collect();
        cells.sort();
        assert_eq!(cells, vec![(0, 0, 0), (2, 1, 3)]);
    }
    #[test]
    fn dilate_spatial_and_theta_wrap() {
        let mut b = Bitset3D::new(5, 5, 4);
        b.set(2, 2, 0);
        let d = b.dilate(1, 1, 1); // ±1 in x,y; ±1 in theta (wrap)
        assert!(d.test(2, 2, 0));
        assert!(d.test(1, 1, 0) && d.test(3, 3, 0));
        assert!(d.test(2, 2, 1) && d.test(2, 2, 3)); // theta wrap 0→{3,1}
        assert!(!d.test(4, 4, 0)); // 距離2は入らない
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
