//! VI を SSP と捉えた優先順序伝播ソルバの共有基盤。本家 per-cell 更新
//! `value_iteration_raw` を「値の昇順」に呼ぶ。コスト数式は不変なので、到達可能
//! セルの収束値は Reference (全走査) = 本家と一致（厳密版 prio_lc）。
//! 設計: `docs/superpowers/specs/2026-06-09-vi-ssp-priority-acceleration-design.md`

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::params::MAX_COST;
use crate::value_iterator::{to_index_raw, value_iteration_raw, ValueIterator};

/// 逆θ写像。`rev[it']` = 確定セル `(.., it')` の前駆を列挙する `(dix, diy, t_src)` 列。
/// 全 (action, source θ `t`, 遷移 `δ`) を走査し、着地 θ `it' = (dit + nt) % nt` をキーに
/// `(dix, diy, t)` を積む。前駆は `(ix' - dix, iy' - diy, t)`。重複は dedup（過剰列挙抑制）。
pub(crate) fn build_rev_theta(vi: &ValueIterator) -> Vec<Vec<(i32, i32, i32)>> {
    let nt = vi.cell_num_t;
    let mut rev: Vec<Vec<(i32, i32, i32)>> = vec![Vec::new(); nt as usize];
    for a in &vi.actions {
        for (t, trans) in a.state_transitions.iter().enumerate() {
            for st in trans {
                let itp = (((st.dit % nt) + nt) % nt) as usize;
                rev[itp].push((st.dix, st.diy, t as i32));
            }
        }
    }
    for bucket in rev.iter_mut() {
        bucket.sort_unstable();
        bucket.dedup();
    }
    rev
}

/// セル `idx` を本家 Bellman で再評価・書込。改善（厳密減少）したら新ラベルを返す。
#[inline]
pub(crate) fn relax_cell(
    vi: &mut ValueIterator,
    idx: usize,
    nx: i32,
    ny: i32,
    nt: i32,
) -> Option<u64> {
    let before = vi.states[idx].total_cost;
    value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
    let after = vi.states[idx].total_cost;
    if after < before {
        Some(after)
    } else {
        None
    }
}

/// 優先順序ソルバの拡張統計。`repops` は確定済みセルの再処理回数（単調性違反の指標、
/// label-setting では常に 0、label-correcting で >0 なら Dial 化に注意）。
#[derive(Clone, Copy, Debug)]
pub struct PrioStats {
    pub iters: u32,
    pub updates: u64,
    pub converged: bool,
    pub repops: u64,
}

/// 共有の優先順序伝播。`label_setting=true`→Dijkstra 流 settle-once（近似・最速）、
/// `false`→label-correcting（厳密・bit-exact）。`total_cost` を tentative ラベルに流用し、
/// 二分ヒープで値の昇順に確定 → 前駆を逆θ隣接で relax。
pub fn priority_solve(vi: &mut ValueIterator, max_iter: u32, label_setting: bool) -> PrioStats {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let rev = build_rev_theta(vi);
    let n = vi.states.len();
    // label-setting は settled、label-correcting は popped を使う（他方は空 Vec）。
    let mut settled = vec![false; if label_setting { n } else { 0 }];
    let mut popped = vec![false; if label_setting { 0 } else { n }];

    let mut heap: BinaryHeap<Reverse<(u64, usize)>> = BinaryHeap::new();
    for (i, s) in vi.states.iter().enumerate() {
        if s.total_cost < MAX_COST {
            heap.push(Reverse((s.total_cost, i))); // 種: final セル (V=0)
        }
    }

    let pop_cap = (n as u64).saturating_mul(max_iter.max(1) as u64); // 暴走ガード（実質無限）
    let mut pops = 0u64;
    let mut iters = 0u32;
    let mut updates = 0u64;
    let mut repops = 0u64;

    while let Some(Reverse((lab, s_star))) = heap.pop() {
        pops += 1;
        if pops > pop_cap {
            return PrioStats { iters, updates, converged: false, repops };
        }
        // 遅延 decrease-key の stale 破棄。
        if lab != vi.states[s_star].total_cost {
            continue;
        }
        if label_setting {
            if settled[s_star] {
                continue;
            }
            settled[s_star] = true;
        } else if popped[s_star] {
            repops += 1;
        } else {
            popped[s_star] = true;
        }
        iters += 1;

        let (ix, iy, it) = (vi.states[s_star].ix, vi.states[s_star].iy, vi.states[s_star].it);
        for &(dix, diy, t) in &rev[it as usize] {
            let px = ix - dix;
            let py = iy - diy;
            if px < 0 || px >= nx || py < 0 || py >= ny {
                continue;
            }
            let pidx = to_index_raw(px, py, t, nx, nt) as usize;
            if label_setting && settled[pidx] {
                continue;
            }
            if !vi.states[pidx].free || vi.states[pidx].final_state {
                continue;
            }
            if let Some(newlab) = relax_cell(vi, pidx, nx, ny, nt) {
                updates += 1;
                heap.push(Reverse((newlab, pidx)));
            }
        }
    }

    PrioStats { iters, updates, converged: true, repops }
}

/// (A1) Priority Label-Setting（近似・最速）。`solve()` 用の軽量タプル。
pub fn prio_ls_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let st = priority_solve(vi, max_iter, true);
    (st.iters, st.updates, st.converged)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solvers::test_support::make_vi;

    #[test]
    fn rev_theta_round_trips_forward_transitions() {
        // 全 (action, θ, 遷移) について、着地θのバケットに (dix,diy,t) が含まれること。
        let vi = make_vi(8, 8, vec![0i8; 64]);
        let rev = build_rev_theta(&vi);
        let nt = vi.cell_num_t;
        assert_eq!(rev.len(), nt as usize);
        for a in &vi.actions {
            for (t, trans) in a.state_transitions.iter().enumerate() {
                for st in trans {
                    let itp = (((st.dit % nt) + nt) % nt) as usize;
                    assert!(
                        rev[itp].contains(&(st.dix, st.diy, t as i32)),
                        "rev[{itp}] must contain ({},{},{})",
                        st.dix,
                        st.diy,
                        t
                    );
                }
            }
        }
        // dedup 済み（各バケットは昇順ユニーク）。
        for bucket in &rev {
            let mut sorted = bucket.clone();
            sorted.sort_unstable();
            sorted.dedup();
            assert_eq!(&sorted, bucket);
        }
    }
}
