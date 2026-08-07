//! A1: frontier2d の SoA (Structure-of-Arrays) 版。Step0 計測で frontier2d が
//! メモリ流量律速 (隣接 State 56B ロードが時間の 99.9%) と判明したため、Bellman 更新が
//! 実際に読む hot フィールドだけを連続配列に切り出して per-bucket footprint を 56B→16B に縮める。
//!
//! - `hot: Vec<[u64; 2]>`   `[total_cost(動的), penalty +ʷ local_penalty(静的・事前合成)]` を 16B 隣接配置
//! - `free` / `finals`      ターゲットガード/障害物判定用 (per-bucket は free のみ参照)
//!
//! コスト数式・演算順序・早期 return は `value_iterator::action_cost_raw` と完全一致させる。
//! `penalty +ʷ local_penalty` の事前合成は `wrapping_add` の結合則で bit一致
//! (`tc +ʷ (pen +ʷ lp) == (tc +ʷ pen) +ʷ lp`)。収束値・方策は Reference = 本家と bit-exact。

use crate::params::{MAX_COST, PROB_BASE_BIT};
use crate::solvers::observe::{
    MaterializeProbe, NullObserver, SolveFlow, SolveObserver, SolveOutcome,
};
use crate::state_transition::StateTransition;
use crate::value_iterator::ValueIterator;

use super::{displacement, frontier2d_driver, seed_frontier_2d, Frontier2DSweep};

/// 本家 `actionCost` の SoA 版。`trans` はソースセルの θ の遷移リスト。
#[inline]
fn action_cost_soa(
    hot: &[[u64; 2]],
    free: &[bool],
    trans: &[StateTransition],
    ix: i32,
    iy: i32,
    nx: i32,
    ny: i32,
    nt: i32,
) -> u64 {
    let mut cost: u64 = 0;
    for tran in trans {
        let nix = ix + tran.dix;
        if nix < 0 || nix >= nx {
            return MAX_COST;
        }
        let niy = iy + tran.diy;
        if niy < 0 || niy >= ny {
            return MAX_COST;
        }
        let nit = (tran.dit + nt) % nt;
        let nidx = (nit + nix * nt + niy * (nt * nx)) as usize;
        if !free[nidx] {
            return MAX_COST;
        }
        let h = hot[nidx]; // [total_cost, penalty +ʷ local_penalty]
        cost = cost.wrapping_add(h[0].wrapping_add(h[1]).wrapping_mul(tran.prob as u64));
    }
    cost >> PROB_BASE_BIT
}

/// SoA モデル: hot/free/finals/opt を所有し、境界では要求時にだけ states へ書き戻す。
struct SoaSweep<'a> {
    vi: &'a mut ValueIterator,
    hot: Vec<[u64; 2]>,
    free: Vec<bool>,
    finals: Vec<bool>,
    opt: Vec<Option<usize>>,
}

impl SoaSweep<'_> {
    /// hot/opt を `vi.states` へ書き戻す (値と方策)。境界プローブの具現化と最終書き戻しで共用。
    fn write_back(vi: &mut ValueIterator, hot: &[[u64; 2]], opt: &[Option<usize>]) {
        for (i, s) in vi.states.iter_mut().enumerate() {
            s.total_cost = hot[i][0];
            s.optimal_action = opt[i];
        }
    }
}

impl Frontier2DSweep for SoaSweep<'_> {
    fn cell(&mut self, ixu: u32, iyu: u32) -> u64 {
        let (nx, ny, nt) =
            (self.vi.cell_num_x, self.vi.cell_num_y, self.vi.cell_num_t);
        let (ix, iy) = (ixu as i32, iyu as i32);
        let mut upd = 0u64;
        for it in 0..nt {
            let idx = (it + ix * nt + iy * (nt * nx)) as usize;
            // 本家 valueIteration: 非 free / final_state は更新しない。
            if !self.free[idx] || self.finals[idx] {
                continue;
            }
            let before = self.hot[idx][0];
            let mut min_cost = MAX_COST;
            let mut min_action: Option<usize> = None;
            for (ai, a) in self.vi.actions.iter().enumerate() {
                let c = action_cost_soa(
                    &self.hot,
                    &self.free,
                    &a.state_transitions[it as usize],
                    ix,
                    iy,
                    nx,
                    ny,
                    nt,
                );
                if c < min_cost {
                    min_cost = c;
                    min_action = Some(ai);
                }
            }
            self.hot[idx][0] = min_cost;
            self.opt[idx] = min_action;
            if min_cost < before {
                upd += 1;
            }
        }
        upd
    }

    fn boundary(&mut self, obs: &mut dyn SolveObserver, iters: u32, updates: u64) -> SolveFlow {
        let (hot, opt, vi) = (&self.hot, &self.opt, &mut *self.vi);
        let mut mat = |vi: &mut ValueIterator| Self::write_back(vi, hot, opt);
        let mut probe = MaterializeProbe::new(vi, &mut mat, iters, updates);
        obs.boundary(&mut probe)
    }
}

/// セット済み `ValueIterator` を Frontier2D-SoA で収束まで解く。
pub fn frontier2d_soa_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    let (nx, ny) = (vi.cell_num_x, vi.cell_num_y);
    let n = vi.states.len();

    // ── SoA 構築: hot=[tc, pen]、free/finals。pen は静的なので一度だけ合成。──
    let mut hot: Vec<[u64; 2]> = Vec::with_capacity(n);
    let mut free: Vec<bool> = Vec::with_capacity(n);
    let mut finals: Vec<bool> = Vec::with_capacity(n);
    for s in &vi.states {
        hot.push([s.total_cost, s.penalty.wrapping_add(s.local_penalty)]);
        free.push(s.free);
        finals.push(s.final_state);
    }
    let opt: Vec<Option<usize>> = vi.states.iter().map(|s| s.optimal_action).collect();

    let (mx, my, _mt) = displacement(vi);
    let seed = seed_frontier_2d(vi);

    let mut model = SoaSweep { vi, hot, free, finals, opt };
    let out = frontier2d_driver(nx, ny, seed, mx as u32, my as u32, max_iter, obs, &mut model);

    // ── 結果を vi.states へ書き戻し (ハーネス出力・parity 比較が読む)。cancel 時は
    // 場を保証しない契約だが、上界値なので書いても害はない (簡潔さ優先)。──
    SoaSweep::write_back(model.vi, &model.hot, &model.opt);
    out
}

/// 従来 API (observer なし)。`(iters, updates, converged)`。
pub fn frontier2d_soa_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    frontier2d_soa_solve_observed(vi, max_iter, &mut NullObserver).tuple()
}
