//! Frontier3D の u64 版。`vi_algorithm/src/frontier/f3d.rs` の `run_serial_inner` を
//! 本家 u64 モデル（`value_iteration_raw`）へ移植。コスト数式は不変なので、到達可能セルの
//! 収束値・方策は Reference (全走査) = 本家と bit-exact。

use crate::solvers::{displacement, seed_frontier, Bitboard3D};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// セット済み `ValueIterator` を Frontier3D で収束まで解く。`(iters, updates, converged)` を返す。
///
/// 各反復: フロンティアを `(mx,my,mt)` で膨張 → 候補セルを `value_iteration_raw` で更新 →
/// `total_cost` が**厳密減少**したセルを次フロンティアに入れる。`final_state`/非 `free` セルは
/// `value_iteration_raw` が更新せず据置くので、候補に混ざっても安全に無視される。
pub fn frontier3d_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let (mx, my, mt) = displacement(vi);
    let (dx, dy, dt) = (mx as u32, my as u32, mt as u32);
    let mut frontier = seed_frontier(vi);
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let candidates = frontier.dilate(dx, dy, dt);
        let mut new_frontier = Bitboard3D::new(nx as u32, ny as u32, nt as u32);
        for (ix, iy, it) in candidates.enumerate() {
            let idx = vi.to_index(ix as i32, iy as i32, it as i32) as usize;
            let before = vi.states[idx].total_cost;
            value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
            let after = vi.states[idx].total_cost;
            if after < before {
                updates += 1;
                new_frontier.set(ix, iy, it);
            }
        }
        frontier = new_frontier;
    }
    (iters, updates, frontier.popcount() == 0)
}

#[cfg(test)]
mod tests {
    use super::frontier3d_solve;
    use crate::action::Action;
    use crate::msg::OccupancyGrid;
    use crate::params::PROB_BASE;
    use crate::value_iterator::ValueIterator;

    const REACH: u64 = 1_000_000u64 * PROB_BASE;

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

    fn make_vi(w: i32, h: i32, occ: Vec<i8>) -> ValueIterator {
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
    fn run_reference_to_fixed_point(vi: &mut ValueIterator) {
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

    fn assert_parity(w: i32, h: i32, occ: Vec<i8>) {
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);
        let (_iters, _updates, converged) = frontier3d_solve(&mut b, 2000);
        assert!(converged, "Frontier3D must converge");
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

    #[test]
    fn parity_empty_8x8() {
        assert_parity(8, 8, vec![0i8; 64]);
    }

    #[test]
    fn parity_obstacle_8x8() {
        // 中央付近に縦壁（goal セル (2,2) は空けておく）。
        let mut occ = vec![0i8; 64];
        for iy in 0..8 {
            occ[(iy * 8 + 5) as usize] = 100; // x=5 の縦壁
        }
        occ[(0 * 8 + 5) as usize] = 0; // 壁に隙間
        assert_parity(8, 8, occ);
    }

    #[test]
    fn parity_sentinel_8x8() {
        // goal を3方向障害物で囲み、片側のみ通路 → 到達不能セルが生じる構成。
        let mut occ = vec![0i8; 64];
        occ[(1 * 8 + 2) as usize] = 100; // goal(2,2) の下
        occ[(3 * 8 + 2) as usize] = 100; // 上
        occ[(2 * 8 + 1) as usize] = 100; // 左
        assert_parity(8, 8, occ);
    }
}
