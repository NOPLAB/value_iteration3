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
    pub iters: u64,
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

    let pop_cap = (n as u64).saturating_mul(max_iter.max(1) as u64); // 暴走ガード: LC は最大 n*max_iter pops で打ち切り（LS は 1 パス ≤ n pops）
    let mut pops = 0u64;
    let mut iters = 0u64;
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
        iters += 1; // LS: 各セル1回（≤ n）。LC: 再処理も計上（= 初回確定数 + repops）

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
    (st.iters.min(u32::MAX as u64) as u32, st.updates, st.converged)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::PROB_BASE;
    use crate::solvers::test_support::{make_vi, run_reference_to_fixed_point, REACH};
    use crate::state_transition::StateTransition;
    use crate::value_iterator::ValueIterator;

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

    /// 各 action・θ の遷移分布を最頻 outcome 1点 (prob=PROB_BASE) に潰し、決定論化する。
    fn collapse_to_deterministic(vi: &mut ValueIterator) {
        let b = PROB_BASE as i32;
        for a in vi.actions.iter_mut() {
            for trans in a.state_transitions.iter_mut() {
                if trans.is_empty() {
                    continue;
                }
                let top = trans.iter().max_by_key(|s| s.prob).unwrap().clone();
                *trans = vec![StateTransition::new(top.dix, top.diy, top.dit, b)];
            }
        }
    }

    #[test]
    fn prio_ls_exact_on_deterministic_transitions() {
        // 決定論遷移では単調性違反が起き得ず、prio_ls (settle-once) も Reference と bit-exact。
        let mut a = make_vi(8, 8, vec![0i8; 64]);
        let mut b = make_vi(8, 8, vec![0i8; 64]);
        collapse_to_deterministic(&mut a);
        collapse_to_deterministic(&mut b);
        run_reference_to_fixed_point(&mut a);
        let (_i, _u, conv) = super::prio_ls_solve(&mut b, 3000);
        assert!(conv, "prio_ls must converge");

        let mut n_reach = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH {
                n_reach += 1;
                assert_eq!(a.states[i].total_cost, b.states[i].total_cost, "value @ {i}");
                assert_eq!(
                    a.states[i].optimal_action, b.states[i].optimal_action,
                    "policy @ {i}"
                );
            }
        }
        assert!(n_reach > 0, "決定論グラフでも到達可能セルが存在するはず");
    }

    fn standard_occ() -> Vec<(&'static str, Vec<i8>)> {
        let empty = vec![0i8; 64];
        let mut wall = vec![0i8; 64];
        for iy in 0..8 {
            wall[(iy * 8 + 5) as usize] = 100;
        }
        wall[5] = 0;
        let mut sentinel = vec![0i8; 64];
        sentinel[(1 * 8 + 2) as usize] = 100;
        sentinel[(3 * 8 + 2) as usize] = 100;
        sentinel[(2 * 8 + 1) as usize] = 100;
        vec![("empty", empty), ("obstacle", wall), ("sentinel", sentinel)]
    }

    #[test]
    fn prio_ls_characterization_vs_reference() {
        // prio_ls の近似度を実測（RMSE/方策一致）。ゆるい上限で回帰ガード（厳密値は出力で観察）。
        for (name, occ) in standard_occ() {
            let mut a = make_vi(8, 8, occ.clone());
            let mut b = make_vi(8, 8, occ);
            run_reference_to_fixed_point(&mut a);
            super::prio_ls_solve(&mut b, 3000);

            let (mut se, mut n, mut agree) = (0f64, 0u64, 0u64);
            for i in 0..a.states.len() {
                if a.states[i].total_cost < REACH {
                    let va = (a.states[i].total_cost / PROB_BASE) as f64;
                    let vb = (b.states[i].total_cost / PROB_BASE) as f64;
                    se += (va - vb) * (va - vb);
                    n += 1;
                    if a.states[i].optimal_action == b.states[i].optimal_action {
                        agree += 1;
                    }
                }
            }
            let rmse = (se / n as f64).sqrt();
            let pa = agree as f64 / n as f64;
            eprintln!("[prio_ls characterization] map={name} rmse={rmse:.3} policy={pa:.4} n={n}");
            assert!(n > 0, "到達セルが存在するはず ({name})");
            assert!(rmse <= 10.0, "prio_ls RMSE {rmse} exceeds loose bound ({name})");
            assert!(pa >= 0.85, "prio_ls policy agreement {pa} below loose bound ({name})");
        }
    }
}
