//! Frontier3DTopK の u64 版。`vi_algorithm/src/frontier/topk.rs` を本家 u64 モデルへ移植。
//! 各 (action, theta) の遷移 outcome を確率上位 k 個に枝刈りし、枝刈り後の確率和で正規化した
//! 期待コストで Bellman 更新する近似。outcome 数 ≤ k のアクションは枝刈り無し＝厳密コスト
//! （`action_cost_raw`）に委譲するので、k が全 outcome 数以上なら Frontier3D と bit-exact。

use crate::params::MAX_COST;
use crate::solvers::{displacement, seed_frontier, Bitboard3D};
use crate::value_iterator::{action_cost_raw, to_index_raw, ValueIterator};
use crate::{Action, State};

/// 1 アクションの top-k 正規化コスト。枝刈り後の outcome のいずれかが範囲外/障害物なら
/// `MAX_COST`。outcome 数 ≤ k なら枝刈り無し → `action_cost_raw` と一致（厳密）。
fn action_cost_topk(
    states: &[State], a: &Action, s: &State, k: u32, nx: i32, ny: i32, nt: i32,
) -> u64 {
    let trans = &a.state_transitions[s.it as usize];
    let n = trans.len();
    if n <= k as usize {
        // 枝刈り不要 → 厳密コスト（>> PROB_BASE_BIT）。
        return action_cost_raw(states, a, s, nx, ny, nt);
    }
    // 確率上位 k 個を選択（first-wins ties: 厳密 > で元順序を保つ）。
    let mut idx: Vec<usize> = (0..n).collect();
    // 安定的な部分選択: prob 降順、同値はインデックス昇順。
    idx.sort_by(|&i, &j| trans[j].prob.cmp(&trans[i].prob).then(i.cmp(&j)));
    idx.truncate(k as usize);

    let mut cost: u64 = 0;
    let mut psum: u64 = 0;
    for &i in &idx {
        let tran = &trans[i];
        let ix = s.ix + tran.dix;
        if ix < 0 || ix >= nx {
            return MAX_COST;
        }
        let iy = s.iy + tran.diy;
        if iy < 0 || iy >= ny {
            return MAX_COST;
        }
        let it = (tran.dit + nt) % nt;
        let after = &states[to_index_raw(ix, iy, it, nx, nt) as usize];
        if !after.free {
            return MAX_COST;
        }
        cost = cost.wrapping_add(
            after
                .total_cost
                .wrapping_add(after.penalty)
                .wrapping_add(after.local_penalty)
                .wrapping_mul(tran.prob as u64),
        );
        psum += tran.prob as u64;
    }
    if psum == 0 {
        return MAX_COST;
    }
    // 枝刈り後確率和で正規化（k=全 outcome なら psum = PROB_BASE → >> PROB_BASE_BIT と一致）。
    cost / psum
}

/// セット済み `ValueIterator` を Frontier3DTopK で収束まで解く。`(iters, updates, converged)`。
pub fn frontier3d_topk_solve(vi: &mut ValueIterator, k: u32, max_iter: u32) -> (u32, u64, bool) {
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
            if !vi.states[idx].free || vi.states[idx].final_state {
                continue;
            }
            let old = vi.states[idx].total_cost;
            // min over アクションの top-k コスト（書き込まず計算）。
            let (mut min_cost, mut min_a) = (MAX_COST, None);
            {
                let s = &vi.states[idx];
                for (ai, a) in vi.actions.iter().enumerate() {
                    let c = action_cost_topk(&vi.states, a, s, k, nx, ny, nt);
                    if c < min_cost {
                        min_cost = c;
                        min_a = Some(ai);
                    }
                }
            }
            // value_iteration_raw と同じく **無条件**に total_cost/optimal_action を書く
            // （k=全 outcome のとき policy まで bit-exact になる）。フロンティア追加は減少時のみ。
            vi.states[idx].total_cost = min_cost;
            vi.states[idx].optimal_action = min_a;
            if min_cost < old {
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
    use super::frontier3d_topk_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn topk_full_parity_standard_maps() {
        // k=u32::MAX → 枝刈り無し → Frontier3D 等価 → Reference と bit-exact。
        parity_standard_maps(|vi| frontier3d_topk_solve(vi, u32::MAX, 2000));
    }
}
