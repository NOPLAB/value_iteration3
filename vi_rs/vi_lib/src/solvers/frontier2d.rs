//! Frontier2D の u64 版。`vi_algorithm/src/frontier/f2d.rs` を本家 u64 モデルへ移植。
//! 空間 2D フロンティア: 活性 (ix,iy) が現れたら全 θ 層を再評価する。dilation は空間のみで
//! 安い代わりに per-cell 仕事量が N_THETA 倍。収束値・方策は Reference = 本家と bit-exact。
//!
//! # 伝播は値が「動いた」ら行う (下がったときだけ、ではない)
//!
//! 白紙 (`MAX_COST`) から解く限り値は単調に下がるので、伝播の条件を「下がった」に
//! しても「動いた」にしても挙動は同じ。**違うのは解き終わった場を掃き直すとき**で、
//! 走行中に注入される `local_penalty` は値を上げる向きにも働く
//! (`ValueIteratorLocal::set_local_cost` はヒット点にコストを足し、光線が抜けた
//! 自由空間のコストは半減させる)。条件が「下がった」だと上昇は黙って伝播せず、
//! エラーも出ないまま遠方の場が古い値のまま残る。
//!
//! 能動集合の健全性はどちらの向きでも同じ理屈で立つ: あるセルの値が動くのは
//! 遷移先の値かペナルティが動いたときだけなので、動いたセルを `displacement` だけ
//! 膨張させた範囲を再評価すれば取りこぼさない。空フロンティア = 不動点。
//!
//! なお同じ「下がったときだけ」の条件は frontier3d / stack / block / sparse / fused /
//! par 系にも残っている。あちらは白紙から解く用途しか無いので実害は無いが、
//! **収束後の場を掃き直す用途に転用するならここと同じ直しが要る**。

use crate::solvers::observe::{
    BoundaryPacer, InPlaceProbe, SolveFlow, SolveObserver, SolveOutcome,
};
use crate::solvers::{displacement, seed_frontier_2d, Bitboard2D};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// セット済み `ValueIterator` を Frontier2D で収束まで解く。
///
/// 「seed → (空間膨張 → 候補 (ix,iy) ごとに全 θ 層を `value_iteration_raw` で再評価 →
/// 減少があれば次フロンティアへ) を収束まで」。ラウンド境界ごとに `obs.boundary` を呼ぶ
/// (in-place なので観測は無料)。
pub fn frontier2d_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let (mx, my, _mt) = displacement(vi);
    let (dx, dy) = (mx as u32, my as u32);
    let mut pacer = BoundaryPacer::new(obs);
    let mut frontier = seed_frontier_2d(vi);
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let candidates = frontier.dilate(dx, dy);
        let mut new_frontier = Bitboard2D::new(nx as u32, ny as u32);
        for (ix, iy) in candidates.enumerate() {
            let mut u = 0u64;
            for it in 0..nt {
                let idx = vi.to_index(ix as i32, iy as i32, it) as usize;
                // `value_iteration_raw` の戻り値は |Δ| なので、上下どちらの変化も拾う
                // (モジュール doc 参照)。白紙からの solve では `< before` と完全に
                // 同じ挙動 = Reference と bit-exact のまま。
                if value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt) > 0 {
                    u += 1;
                }
            }
            if u > 0 {
                updates += u;
                new_frontier.set(ix, iy);
            }
        }
        frontier = new_frontier;
        if frontier.popcount() > 0 && pacer.due(iters as u64) {
            let mut probe = InPlaceProbe { vi, iters, updates };
            match obs.boundary(&mut probe) {
                SolveFlow::Continue => {}
                SolveFlow::Stop => return SolveOutcome::stopped(iters, updates),
                SolveFlow::Cancel => return SolveOutcome::cancelled(iters, updates),
            }
        }
    }
    SolveOutcome::running(iters, updates, frontier.popcount() == 0)
}
