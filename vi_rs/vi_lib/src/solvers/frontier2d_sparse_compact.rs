//! B5: `frontier2d_sparse` のアウトオブコア版 (`compact`)。メモリ制約下で巨大マップを解く。
//!
//! 設計（2026-06-27 合意・spec 省略・直接実装）:
//! - **健全性 = 値ウォーターマーク finalization**: 値を昇順で前進させ、しきい値 `T` 未満を毎波
//!   Gauss–Seidel で収束させる。`watermark = T − Δ_band` 以下に収束したセルは、その直接依存
//!   （窓内 outcome, 値は `v + 1セル分` 以内）が全て収束済み区間 `< T` に入るため v* 到達済み =
//!   以後不変 → final 化する。本家ダイナミクスは決定論 (`no_noise_state_transition`) + サブセル
//!   サンプリングなので outcome は隣接1セル幅のタイトクラスタ＝結合は浅く、`Δ_band` は控えめで
//!   足りる（parity テストで検証）。
//! - **メモリ = ブロックタイル + 値バンド + 遅延確保 + 退避**: パディング済みグリッドを行ブロックに
//!   分割。ブロックは frontier 到達時に states から**遅延確保**し、interior-final（自±halo が全
//!   final）になったら cp/pen/eval を**退避（解放）**。常駐ブロック ∝ 等高線×バンド（総面積でない）。
//!   出力(value/policy)は finalize 時に別配列へ確定保存し store から切り離す。
//!
//! 実装ステージ（TDD・parity 先行）: S1 ブロックストレージ。S2 finalization。S3 値バンド閾値
//! スキャン + ウォーターマーク finalization + ブロック退避。**S3b(本コミット)** 遅延確保で peak
//! RAM を band に抑える。後続: ディスク mmap → 並列化 → CLI。

use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::action::Action;
use crate::msg::OccupancyGrid;
use crate::params::MAX_COST;
use crate::state::State;
use crate::value_iterator::ValueIterator;

use super::frontier2d_fused::{action_cost_fused, CpCells, Geom, UNREACHED};
use super::frontier2d_par::n_threads;
use super::frontier2d_sparse::rot_dilate;
use super::{displacement, Bitboard2D};
use crate::planner::{CompactPolicy, PolicyView};
use crate::solvers::observe::{NullObserver, SolveFlow, SolveObserver, SolveProbe};

/// compact が per-state に読む情報の抽象源。O(total) の `states` 配列を常駐させる `SliceSource` と、
/// マップから 2D の free/penalty（θ 非依存）＋ ゴール近傍の final 集合だけを保持する `MapSource` を
/// 差し替えられる。`MapSource` は O(total) を持たないので、巨大マップでもメモリ床が O(nx·ny) になる。
///
/// compact が読むのは「(x,y) の penalty/free（θ 非依存）」「(x,y,θ) の final_state（ゴール局所）」
/// だけ。seed（total_cost<MAX = final）は `for_each_seed` で列挙、free 列は `for_each_free` で列挙する
/// （どちらも MapSource では全走査を避けられる）。本家 `State::from_occupancy` / `setStateValues` の式を
/// そのまま再利用するので、両源は per-cell で bit-exact 一致する（parity テストで検証）。
pub(crate) trait StateSource {
    /// セル (ix,iy) の `penalty +ʷ local_penalty`（θ 非依存、local_penalty は常に 0）。
    fn pen(&self, ix: i32, iy: i32) -> u64;
    /// セル (ix,iy) が自由か（`map.data==0`、θ 非依存）。
    fn free(&self, ix: i32, iy: i32) -> bool;
    /// (ix,iy,it) が goal final セルか（本家 `setStateValues` の距離+向き判定）。
    fn is_final(&self, ix: i32, iy: i32, it: i32) -> bool;
    /// 全セルの penalty 最大（値バンド幅算出用、占有 PROB_BASE も含む）。
    fn max_pen(&self) -> u64;
    /// seed（final）セル (ix,iy,it) を列挙する（ゴール局所）。
    fn for_each_seed(&self, f: &mut dyn FnMut(i32, i32, i32));
    /// free な (ix,iy) を列挙する（2D、n_eval 算出用）。
    fn for_each_free(&self, f: &mut dyn FnMut(i32, i32));
    /// 源自身の常駐ヒープバイト数（O(nx·ny) 床の計測用）。states 借用の SliceSource は
    /// 自前確保がないので 0。
    fn floor_bytes(&self) -> u64 {
        0
    }
}

/// orig 索引 `it + ix·nt + iy·nx·nt` を **usize で**計算する。i32 演算だと nstates が i32::MAX
/// (≈2.1e9) を超える巨大マップ（フル解像度 tsukuba 13250×7100×60 = 5.6e9 states）で `iy·nt·nx` が
/// オーバーフローして負にラップ → スライス範囲外 panic になるため、必ずこの関数を通す。
#[inline]
fn orig_index(ix: i32, iy: i32, it: i32, nx: i32, nt: i32) -> usize {
    it as usize + ix as usize * nt as usize + iy as usize * nx as usize * nt as usize
}

/// `states` 配列をそのまま源にする（既存挙動）。full states 常駐・write_back あり経路で使う。
pub(crate) struct SliceSource<'a> {
    states: &'a [State],
    nx: i32,
    nt: i32,
    max_pen: u64,
}

impl<'a> SliceSource<'a> {
    fn new(states: &'a [State], nx: i32, nt: i32) -> Self {
        let mut max_pen = 0u64;
        for s in states {
            let p = s.penalty.wrapping_add(s.local_penalty);
            if p > max_pen {
                max_pen = p;
            }
        }
        Self { states, nx, nt, max_pen }
    }
    #[inline]
    fn orig(&self, ix: i32, iy: i32, it: i32) -> usize {
        orig_index(ix, iy, it, self.nx, self.nt)
    }
}

impl StateSource for SliceSource<'_> {
    #[inline]
    fn pen(&self, ix: i32, iy: i32) -> u64 {
        let s = &self.states[self.orig(ix, iy, 0)];
        s.penalty.wrapping_add(s.local_penalty)
    }
    #[inline]
    fn free(&self, ix: i32, iy: i32) -> bool {
        self.states[self.orig(ix, iy, 0)].free
    }
    #[inline]
    fn is_final(&self, ix: i32, iy: i32, it: i32) -> bool {
        self.states[self.orig(ix, iy, it)].final_state
    }
    #[inline]
    fn max_pen(&self) -> u64 {
        self.max_pen
    }
    fn for_each_seed(&self, f: &mut dyn FnMut(i32, i32, i32)) {
        for s in self.states {
            if s.total_cost < MAX_COST {
                f(s.ix, s.iy, s.it);
            }
        }
    }
    fn for_each_free(&self, f: &mut dyn FnMut(i32, i32)) {
        for s in self.states {
            if s.it == 0 && s.free {
                f(s.ix, s.iy);
            }
        }
    }
}

/// マップから 2D の free/penalty とゴール近傍の final 集合だけを構築する源（O(nx·ny) メモリ）。
/// O(total) の states を持たないので巨大マップでもメモリ床が下がる。
///
/// 表現は可逆圧縮済み: free は 1 bit/セル、penalty は distinct 値表 + u8 索引（本家
/// `from_occupancy` の式は {PROB_BASE, margin_penalty·PROB_BASE+PROB_BASE} の 2 値しか
/// 生まない）。素朴な bool + u64 の 9 B/セルが約 1.13 B/セルになる。
pub(crate) struct MapSource {
    nx: i32,
    ny: i32,
    nt: i32,
    /// (ix,iy) の free（`map.data==0`）。線形 xy 順で 64 セル/語のビットセット。
    free_bits: Vec<u64>,
    /// (ix,iy) の `penalty`（local_penalty=0）のコードブック索引。値本体は `pen_table`。
    pen_code: Vec<u8>,
    /// penalty の distinct 値表（`pen_code` が索引）。
    pen_table: Vec<u64>,
    max_pen: u64,
    /// final セルの orig 索引集合（ゴール局所で小さい）。
    finals: HashSet<i64>,
}

impl MapSource {
    /// マップ + 設定済み `vi`（geometry/goal、states は不要）から構築する。`safety_radius` /
    /// `safety_radius_penalty` は本家 penalty 式の引数（vi には保持されないので明示的に受ける）。
    pub(crate) fn build(
        map: &OccupancyGrid,
        vi: &ValueIterator,
        safety_radius: f64,
        safety_radius_penalty: f64,
    ) -> Self {
        let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
        let margin = (safety_radius / vi.xy_resolution).ceil() as i32;
        let n2d = (nx as usize) * (ny as usize);
        let mut free_bits = vec![0u64; n2d.div_ceil(64)];
        let mut pen_code = vec![0u8; n2d];
        let mut pen_table: Vec<u64> = Vec::new();
        let mut max_pen = 0u64;
        // 2D: free/penalty は θ 非依存なので θ=0 で 1 回だけ本家式を回す。
        for y in 0..ny {
            for x in 0..nx {
                let s = State::from_occupancy(x, y, 0, map, margin, safety_radius_penalty, nx);
                let i = (y * nx + x) as usize;
                if s.free {
                    free_bits[i >> 6] |= 1u64 << (i & 63);
                }
                // local_penalty = 0。distinct 値は構造上ごく少数なので線形探索で十分。
                pen_code[i] = match pen_table.iter().position(|&p| p == s.penalty) {
                    Some(c) => c as u8,
                    None => {
                        pen_table.push(s.penalty);
                        assert!(
                            pen_table.len() <= 256,
                            "MapSource pen codebook overflow: from_occupancy が 256 種超の \
                             penalty を生んだ（式を変えたら pen_code を広げること）"
                        );
                        (pen_table.len() - 1) as u8
                    }
                };
                if s.penalty > max_pen {
                    max_pen = s.penalty;
                }
            }
        }
        let finals = compute_finals(vi, |ix, iy| {
            let i = (iy * nx + ix) as usize;
            (free_bits[i >> 6] >> (i & 63)) & 1 == 1
        });
        Self { nx, ny, nt, free_bits, pen_code, pen_table, max_pen, finals }
    }
    #[inline]
    fn xy(&self, ix: i32, iy: i32) -> usize {
        (iy * self.nx + ix) as usize
    }
    #[inline]
    fn orig(&self, ix: i32, iy: i32, it: i32) -> i64 {
        orig_index(ix, iy, it, self.nx, self.nt) as i64
    }
}

impl StateSource for MapSource {
    #[inline]
    fn pen(&self, ix: i32, iy: i32) -> u64 {
        self.pen_table[self.pen_code[self.xy(ix, iy)] as usize]
    }
    #[inline]
    fn free(&self, ix: i32, iy: i32) -> bool {
        let i = self.xy(ix, iy);
        (self.free_bits[i >> 6] >> (i & 63)) & 1 == 1
    }
    #[inline]
    fn is_final(&self, ix: i32, iy: i32, it: i32) -> bool {
        self.finals.contains(&self.orig(ix, iy, it))
    }
    #[inline]
    fn max_pen(&self) -> u64 {
        self.max_pen
    }
    fn for_each_seed(&self, f: &mut dyn FnMut(i32, i32, i32)) {
        let (nx, nt) = (self.nx as i64, self.nt as i64);
        for &orig in &self.finals {
            let it = (orig % nt) as i32;
            let rem = orig / nt;
            let ix = (rem % nx) as i32;
            let iy = (rem / nx) as i32;
            f(ix, iy, it);
        }
    }
    fn for_each_free(&self, f: &mut dyn FnMut(i32, i32)) {
        for iy in 0..self.ny {
            for ix in 0..self.nx {
                if self.free(ix, iy) {
                    f(ix, iy);
                }
            }
        }
    }
    fn floor_bytes(&self) -> u64 {
        (self.free_bits.len() * 8
            + self.pen_code.len()
            + self.pen_table.len() * 8
            + self.finals.len() * 8) as u64
    }
}

/// 本家 `setStateValues` の final 判定を**ゴール近傍 bbox のみ**で再現し、final セルの orig 集合を返す。
/// final_xy は両角 (x0,y0)/(x1,y1) がともに goal から `goal_margin_radius` 内を要求するので、bbox の
/// 外側のセルは決して final にならない（健全な絞り込み）。bit-exact のため式は本家と完全一致させる。
fn compute_finals(vi: &ValueIterator, free_at: impl Fn(i32, i32) -> bool) -> HashSet<i64> {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let (xy_res, ox, oy) = (vi.xy_resolution, vi.map_origin_x, vi.map_origin_y);
    let (gx, gy, gt, gm) = (vi.goal_x, vi.goal_y, vi.goal_t, vi.goal_margin_theta);
    let r2 = vi.goal_margin_radius * vi.goal_margin_radius;
    let t_res = vi.t_resolution;
    let rad = vi.goal_margin_radius;
    let mut set = HashSet::new();
    if rad <= 0.0 || xy_res <= 0.0 {
        return set;
    }
    // ゴール ± (rad + 1cell) の bbox（両角が rad 内 → 中心も rad 内なので必ず包含）。
    let lo_x = (((gx - rad - ox) / xy_res).floor() as i32 - 1).max(0);
    let hi_x = (((gx + rad - ox) / xy_res).ceil() as i32 + 1).min(nx - 1);
    let lo_y = (((gy - rad - oy) / xy_res).floor() as i32 - 1).max(0);
    let hi_y = (((gy + rad - oy) / xy_res).ceil() as i32 + 1).min(ny - 1);
    for iy in lo_y..=hi_y {
        for ix in lo_x..=hi_x {
            if !free_at(ix, iy) {
                continue;
            }
            let x0 = ix as f64 * xy_res + ox;
            let y0 = iy as f64 * xy_res + oy;
            let r0 = (x0 - gx) * (x0 - gx) + (y0 - gy) * (y0 - gy);
            let x1 = x0 + xy_res;
            let y1 = y0 + xy_res;
            let r1 = (x1 - gx) * (x1 - gx) + (y1 - gy) * (y1 - gy);
            if !(r0 < r2 && r1 < r2) {
                continue; // final_xy 不成立。
            }
            for it in 0..nt {
                let t0 = (it as f64 * t_res) as i32;
                let t1 = ((it + 1) as f64 * t_res) as i32;
                let goal_t_2 = if gt > 180 { gt - 360 } else { gt + 360 };
                let ok = (gt - gm <= t0 && t1 <= gt + gm)
                    || (goal_t_2 - gm <= t0 && t1 <= goal_t_2 + gm);
                if ok {
                    set.insert(orig_index(ix, iy, it, nx, nt) as i64);
                }
            }
        }
    }
    set
}

/// 1 ブロック = この本数のパディング行。複数ブロックに跨る gather を刺激しつつ、退避粒度を保つ。
const BLOCK_ROWS: usize = 8;

/// 値バンド幅 `Δ_band = COUPLE_SAFETY · max(mx,my) · max_pen` の結合深さ安全係数。本家は決定論
/// ダイナミクス + サブセルサンプリングで outcome が隣接1セル幅 → 結合は浅いので小さめで足りる。
/// 値が小さすぎると bit-exact が壊れる（parity テストが検出）ので、テストで妥当性を担保する。
const COUPLE_SAFETY: u64 = 4;

/// 1 ブロック分の cp/pen/eval/fin。退避時に `BlockStore` 側で `None` に落として解放する。
///
/// メモリレイアウト: `cp` だけが per-cell（8 B）。`pen` は θ 非依存（`StateSource::pen` が
/// (ix,iy) にしか依らない）ので**列単位** 1 本（per-cell 換算 8/nt B）、`eval`/`fin` は列単位の
/// θ ビットマスク（`nt ≤ 64` 前提、per-cell 換算 2/nt B）。旧レイアウト（pen per-cell 8 B +
/// bool 2 B）の 18 B/cell から ~8.4 B/cell へ半減する。ブロック境界は `chunk = BLOCK_ROWS ·
/// row_stride` で `row_stride % nt == 0` なので列がブロックを跨ぐことはない。
struct Block {
    cp: Vec<u64>,
    /// 列 (=`off/nt`) ごとのペナルティ（θ 非依存）。
    pen: Vec<u64>,
    /// 列ごとの eval_ok θ ビットマスク。
    eval: Vec<u64>,
    /// 列ごとの final 化済み θ ビットマスク。構造的 final（`!eval_ok`）は構築時 1。
    fin: Vec<u64>,
}

/// 行ブロック化＋遅延確保＋退避するストア。
///
/// パディング済みフラット座標（`Geom`、`cp[it + (ix+mx)·nt + (iy+my)·row_stride]`）を
/// `chunk = BLOCK_ROWS·row_stride` ごとの行ブロックへ分割。`blocks[b]` は frontier 到達時に
/// `ensure_block` で states から遅延構築し、interior-final になったら `evict_block` で `None` に
/// 落として解放する。ブロック単位 metadata（`n_eval`/`n_final`/`evicted`）は確保/退避をまたいで
/// 残すため並列 Vec で保持する。gather は flat 索引を `(block, offset)` に分解して読む。
pub(crate) struct BlockStore {
    blocks: Vec<Option<Block>>,
    /// ブロックごとの eval_ok セル数（構築前に states スキャンで確定）。
    n_eval: Vec<usize>,
    /// ブロックごとの final 化済み eval_ok セル数。`n_final == n_eval` で全 final。
    n_final: Vec<usize>,
    /// 退避済みフラグ（退避後も全 final 判定に使うため残す）。
    evicted: Vec<bool>,
    chunk: usize,
    n_pad: usize,
    // 遅延構築の座標復元用。
    row_stride: usize,
    nt: i32,
    nx: i32,
    ny: i32,
    mx: i32,
    my: i32,
    /// 全 free セルの最大ペナルティ（値バンド幅算出用）。
    max_pen: u64,
}

impl CpCells for BlockStore {
    #[inline(always)]
    fn get(&self, i: usize) -> u64 {
        let b = i / self.chunk;
        debug_assert!(!self.evicted[b], "read from evicted block {b}");
        self.blocks[b].as_ref().expect("block not allocated").cp[i - b * self.chunk]
    }
}

impl BlockStore {
    /// `src` から `n_eval`/`max_pen` だけ確定する（cp は materialize しない）。n_eval は「free·nt −
    /// final」をブロック別に集計する（1 列の全 θ は同一ブロックに収まり、final ⊆ free なので非負）。
    /// MapSource ならこの集計は O(nx·ny)（O(total) の states 走査が不要）。
    fn new(g: &Geom, src: &dyn StateSource) -> Self {
        let chunk = BLOCK_ROWS * g.row_stride as usize;
        let n_pad = g.n_pad();
        let nblk = n_pad.div_ceil(chunk);
        let nt = g.nt as i64;
        let mut acc = vec![0i64; nblk];
        src.for_each_free(&mut |ix, iy| {
            let blk = (g.pad_col(ix, iy) as usize) / chunk;
            acc[blk] += nt;
        });
        src.for_each_seed(&mut |ix, iy, _it| {
            let blk = (g.pad_col(ix, iy) as usize) / chunk;
            acc[blk] -= 1;
        });
        let n_eval: Vec<usize> = acc.into_iter().map(|v| v as usize).collect();
        let max_pen = src.max_pen();
        Self {
            blocks: (0..nblk).map(|_| None).collect(),
            n_final: vec![0; nblk],
            evicted: vec![false; nblk],
            n_eval,
            chunk,
            n_pad,
            row_stride: g.row_stride as usize,
            nt: g.nt,
            nx: g.nx,
            ny: g.ny,
            mx: g.mx,
            my: g.my,
            max_pen,
        }
    }

    /// ブロック `b` を `src` から遅延構築する（既に確保済み/退避済みなら no-op）。`Fused::build_direct`
    /// と同一規約: `pen = penalty +ʷ local_penalty`、`cp = pen`（final セル= seed value 0、`0+ʷpen`）
    /// さもなくば UNREACHED、`eval = free && !final`、構造的 `fin = !eval`。free/pen は θ 非依存
    /// なので src への問い合わせは列ごと 1 回で済む（旧実装は per-cell に 60 回引いていた）。
    fn ensure_block(&mut self, b: usize, src: &dyn StateSource) {
        if self.blocks[b].is_some() || self.evicted[b] {
            return;
        }
        let start = b * self.chunk;
        let len = self.chunk.min(self.n_pad - start);
        let (nt, nx, ny, mx, my) = (self.nt, self.nx, self.ny, self.mx, self.my);
        let ntu = nt as usize;
        debug_assert_eq!(len % ntu, 0, "ブロックは列境界に揃うはず");
        let ncols = len / ntu;
        let mut cp = vec![UNREACHED; len];
        let mut pen = vec![0u64; ncols];
        let mut eval = vec![0u64; ncols];
        let mut fin = vec![u64::MAX; ncols]; // パディング/非 eval は構造的 final。
        for c in 0..ncols {
            let pad_base = start + c * ntu;
            let iy_pad = pad_base / self.row_stride;
            let ix_pad = (pad_base % self.row_stride) / ntu;
            let ix = ix_pad as i32 - mx;
            let iy = iy_pad as i32 - my;
            if ix < 0 || ix >= nx || iy < 0 || iy >= ny {
                continue;
            }
            let free = src.free(ix, iy);
            let p = src.pen(ix, iy);
            pen[c] = p;
            if !free {
                continue; // eval=0 / fin=全1（構造的 final）のまま。
            }
            let mut ev = 0u64;
            for it in 0..nt {
                if src.is_final(ix, iy, it) {
                    // seed: final セルは total_cost=0 → cp = 0 +ʷ p = p。
                    cp[c * ntu + it as usize] = p;
                } else {
                    ev |= 1u64 << it;
                }
            }
            eval[c] = ev;
            fin[c] = !ev;
        }
        self.blocks[b] = Some(Block { cp, pen, eval, fin });
    }

    #[inline(always)]
    fn pen(&self, i: usize) -> u64 {
        let b = i / self.chunk;
        let off = i - b * self.chunk;
        self.blocks[b].as_ref().expect("block not allocated").pen[off / self.nt as usize]
    }

    #[inline(always)]
    fn eval_ok(&self, i: usize) -> bool {
        let b = i / self.chunk;
        let off = i - b * self.chunk;
        let ntu = self.nt as usize;
        let blk = self.blocks[b].as_ref().expect("block not allocated");
        (blk.eval[off / ntu] >> (off % ntu)) & 1 == 1
    }

    #[inline(always)]
    fn set_final(&mut self, i: usize) {
        let b = i / self.chunk;
        let off = i - b * self.chunk;
        let ntu = self.nt as usize;
        let (c, bit) = (off / ntu, off % ntu);
        let blk = self.blocks[b].as_mut().expect("block not allocated");
        if (blk.fin[c] >> bit) & 1 == 0 {
            blk.fin[c] |= 1u64 << bit;
            if (blk.eval[c] >> bit) & 1 == 1 {
                self.n_final[b] += 1;
            }
        }
    }

    /// ブロック `b` の eval_ok セルが全て final 化済みか（退避済みも「全 final」のまま）。
    #[inline(always)]
    fn block_full(&self, b: usize) -> bool {
        self.n_final[b] == self.n_eval[b]
    }

    /// ブロック `b` を退避（cp/pen/eval/fin を解放）。metadata は近傍判定用に残す。
    fn evict_block(&mut self, b: usize) {
        self.blocks[b] = None;
        self.evicted[b] = true;
    }

    fn nblocks(&self) -> usize {
        self.blocks.len()
    }

    /// 現在常駐している（確保済み・未退避の）ブロック数。
    fn resident_blocks(&self) -> usize {
        self.blocks.iter().filter(|b| b.is_some()).count()
    }

    /// 常駐ブロックのヒープ実測バイト数（cp + pen + eval + fin）。peak RAM 指標用。
    fn resident_bytes(&self) -> u64 {
        self.blocks
            .iter()
            .flatten()
            .map(|blk| 8 * (blk.cp.len() + blk.pen.len() + blk.eval.len() + blk.fin.len()) as u64)
            .sum()
    }

    /// 並列非同期 G-S 用に、確保済みブロックの cp/pen/eval の生ポインタ表を作る（未確保は null）。
    /// 返り値の `AtomicCpView` は当該ラウンド中（ブロックの確保/退避が起きない間）のみ有効。cp は
    /// `AtomicU64` として in-place 原子書き込みし、pen/eval は読み取り専用（列単位配列）。
    /// `Block::cp` は同ラウンド中に再確保（move）されないので生ポインタは安定（`Vec<u64>` と
    /// `AtomicU64` は同一表現）。
    fn atomic_view(&mut self) -> AtomicCpView {
        let nb = self.blocks.len();
        let mut cp = Vec::with_capacity(nb);
        let mut pen = Vec::with_capacity(nb);
        let mut eval = Vec::with_capacity(nb);
        for b in self.blocks.iter_mut() {
            match b {
                Some(blk) => {
                    cp.push(blk.cp.as_mut_ptr().cast::<AtomicU64>());
                    pen.push(blk.pen.as_ptr());
                    eval.push(blk.eval.as_ptr());
                }
                None => {
                    cp.push(std::ptr::null_mut());
                    pen.push(std::ptr::null());
                    eval.push(std::ptr::null());
                }
            }
        }
        AtomicCpView { cp, pen, eval, chunk: self.chunk, nt: self.nt as usize }
    }
}

// AtomicU64 は u64 と同一表現（cp の生ポインタを AtomicU64 として読み書きする前提）。
const _: () = assert!(
    std::mem::size_of::<AtomicU64>() == std::mem::size_of::<u64>()
        && std::mem::align_of::<AtomicU64>() == std::mem::align_of::<u64>()
);

/// `BlockStore` のブロック別 cp/pen/eval を生ポインタ表で束ねた並列ビュー（非同期 G-S 用）。cp は
/// `Relaxed` 原子で in-place 読み書き、pen/eval は読み取り専用。フラット索引 `i` を `(block, offset)`
/// に分解してアクセスする。`CpCells` を実装するので `action_cost_fused` がそのまま使える。
///
/// SAFETY: 当該ラウンド中はブロックの確保/退避が無く `Block::cp/pen/eval` のバッファは move されない。
/// 列ごとに書き手は 1 スレッド（frontier をチャンク分割し列を排他割り当て）なので、同一セルへの
/// 競合書き込みは無い。隣接読みは別列の原子 load（Relaxed）で、G-S は stale read を許容する。よって
/// `frontier2d_fused` の `&[AtomicU64]` ビューと同じ健全性が成り立つ（固定点は単調降下で一意）。
struct AtomicCpView {
    cp: Vec<*mut AtomicU64>,
    /// 列単位（index `off/nt`）のペナルティ配列。
    pen: Vec<*const u64>,
    /// 列単位の eval_ok θ ビットマスク配列。
    eval: Vec<*const u64>,
    chunk: usize,
    nt: usize,
}

// SAFETY: 全アクセスは原子（cp）または読み取り専用（pen/eval）で、列ごと単一書き手の規律を守る。
unsafe impl Send for AtomicCpView {}
unsafe impl Sync for AtomicCpView {}

impl AtomicCpView {
    #[inline(always)]
    fn store(&self, i: usize, v: u64) {
        let b = i / self.chunk;
        // SAFETY: 確保済みブロックのみアクセス（ensure_window 済）、offset は chunk 内、AtomicU64==u64。
        unsafe { (*self.cp[b].add(i - b * self.chunk)).store(v, Ordering::Relaxed) }
    }
    #[inline(always)]
    fn pen(&self, i: usize) -> u64 {
        let b = i / self.chunk;
        let off = i - b * self.chunk;
        unsafe { *self.pen[b].add(off / self.nt) }
    }
    #[inline(always)]
    fn eval_ok(&self, i: usize) -> bool {
        let b = i / self.chunk;
        let off = i - b * self.chunk;
        unsafe { ((*self.eval[b].add(off / self.nt)) >> (off % self.nt)) & 1 == 1 }
    }
}

impl CpCells for AtomicCpView {
    #[inline(always)]
    fn get(&self, i: usize) -> u64 {
        let b = i / self.chunk;
        unsafe { (*self.cp[b].add(i - b * self.chunk)).load(Ordering::Relaxed) }
    }
}

/// 値バンド幅 `Δ_band`。`COUPLE_SAFETY · max(mx,my) · max_pen`。
fn couple_margin(g: &Geom, max_pen: u64) -> u64 {
    let r = (g.mx.max(g.my)).max(1) as u64;
    COUPLE_SAFETY.saturating_mul(r).saturating_mul(max_pen.max(1))
}

/// 列 (ix,iy) の窓（±my 行）に重なる行ブロックを全て確保する。relax/finalize で近傍 gather する前に
/// 呼び、退避済み以外の窓ブロックを常駐させる（退避済みは interior-final なので読まれない）。
fn ensure_window(store: &mut BlockStore, ix: i32, iy: i32, src: &dyn StateSource) {
    let _ = ix; // 行ブロックは全 x を含むので x 窓はブロック境界を跨がない。
    let nb = store.nblocks();
    // 列のセルはパディング行 iy+my、窓は ±my → パディング行 [iy, iy+2my]、ブロック = 行/BLOCK_ROWS。
    let b_lo = iy as usize / BLOCK_ROWS;
    let b_hi = (((iy + 2 * store.my) as usize) / BLOCK_ROWS).min(nb - 1);
    for b in b_lo..=b_hi {
        store.ensure_block(b, src);
    }
}

/// 列 (ix,iy) の `eval_mask` で立つ θ だけを Bellman 更新し、減少分を `view` へ **in-place 原子
/// 書き込み**する（θ マスク疎評価 × 非同期 G-S）。`eval_mask` 外の θ は入力不変が保証されている
/// （マスクは窓 OR + 円環膨張による依存集合の保守的上位集合）ので現在値のまま据え置く＝評価を省く。
/// 隣接は他スレッドの最新書き込みも見え得る（async = G-S 同等の速い収束）。列 (ix,iy) のセルの
/// 書き手は常にこのスレッドだけ（frontier をチャンク排他割り当て）なので競合書き込みは無い。
/// 戻り値 `(更新後 min, 更新後 max, 減少 θ 数, 減少 θ マスク)`。`mn`/`mx` は**全 θ**から集計
/// （非評価 θ は据え置き値で参入）＝ finalize の col_max 判定に使える真の列値域。窓ブロックは
/// 呼び出し前に `ensure_window` 済み前提。
fn relax_column_async_masked(
    view: &AtomicCpView,
    g: &Geom,
    ix: i32,
    iy: i32,
    eval_mask: u64,
) -> (u64, u64, u64, u64) {
    let pad_col = g.pad_col(ix, iy);
    let mut ups = 0u64;
    let mut cmask = 0u64;
    let (mut mn, mut mx) = (MAX_COST, MAX_COST);
    let mut first = true;
    for it in 0..g.nt {
        let pad_idx = (pad_col + it as i64) as usize;
        let cp_self = view.get(pad_idx);
        let pen_self = view.pen(pad_idx);
        let cur_v = if cp_self == UNREACHED {
            MAX_COST
        } else {
            cp_self.wrapping_sub(pen_self)
        };
        let eval_this = (eval_mask >> it) & 1 == 1;
        let new_v = if eval_this && view.eval_ok(pad_idx) {
            let mut min_cost = MAX_COST;
            for per_theta in g.precomp.iter() {
                let c = action_cost_fused(view, &per_theta[it as usize], pad_col);
                if c < min_cost {
                    min_cost = c;
                }
            }
            if min_cost < cur_v {
                view.store(pad_idx, min_cost.wrapping_add(pen_self)); // in-place 原子書き込み。
                ups += 1;
                cmask |= 1u64 << it;
                min_cost
            } else {
                cur_v
            }
        } else {
            cur_v // 非評価 θ（マスク外）/ 非 eval（ゴール/障害）: 値は不変。
        };
        if new_v != MAX_COST {
            if first {
                mn = new_v;
                mx = new_v;
                first = false;
            } else {
                mn = mn.min(new_v);
                mx = mx.max(new_v);
            }
        }
    }
    (mn, mx, ups, cmask)
}

/// 列 (ix,iy) を final 化する。到達済みセルの (value, policy) を出力配列へ確定保存し（退避後に
/// 近傍が無く再計算できないため必須）、eval_ok セルを final 化してその数を返す。policy 算出時は
/// 近傍ブロックがまだ常駐（退避は finalize の後）。`out_*` は orig 索引 `it + ix·nt + iy·nt·nx`。
fn finalize_column(
    store: &mut BlockStore,
    g: &Geom,
    ix: i32,
    iy: i32,
    src: &dyn StateSource,
    sink: &mut dyn CompactSink,
) -> u64 {
    ensure_window(store, ix, iy, src);
    let pad_col = g.pad_col(ix, iy);
    let (nt, nx) = (g.nt, g.nx);
    let ntu = nt as usize;
    // 列の全 θ を一旦バッファに作り、sink へ 1 回で書く（ディスク sink でも列連続 write になる）。
    let mut buf_v = vec![MAX_COST; ntu];
    let mut buf_a = vec![-1i32; ntu];
    let mut cnt = 0u64;
    for it in 0..nt {
        let pad_idx = (pad_col + it as i64) as usize;
        let cp = store.get(pad_idx);
        if cp == UNREACHED {
            continue; // 未到達: バッファは初期 (MAX_COST, -1) のまま。
        }
        let pen = store.pen(pad_idx);
        buf_v[it as usize] = cp.wrapping_sub(pen);
        if store.eval_ok(pad_idx) {
            let mut min_cost = MAX_COST;
            let mut min_action = -1i32;
            for (ai, per_theta) in g.precomp.iter().enumerate() {
                let c = action_cost_fused(store, &per_theta[it as usize], pad_col);
                if c < min_cost {
                    min_cost = c;
                    min_action = ai as i32;
                }
            }
            buf_a[it as usize] = min_action;
            store.set_final(pad_idx);
            cnt += 1;
        }
        // 非 eval（ゴール）: action は -1（None）、value はピン留め値。
    }
    let base = orig_index(ix, iy, 0, nx, nt); // orig(it=0)
    sink.write_column(base, &buf_v, &buf_a);
    cnt
}

/// 確定出力（value, policy）の格納先を抽象化する sink。finalize 時に列単位で書き込み、
/// write_back 時に orig 単位で読む。RAM 実装（`RamSink`）と、呼び出し側のディスク mmap 実装
/// （`vi_bench::MmapSink`）を差し替えられる。これで出力の O(total) RAM をディスクへ外せる
/// （巨大マップ対応）。vi_lib は依存軽量なので mmap 実装はここには置かない。
pub trait CompactSink {
    /// 連続 orig 範囲 `[base, base+values.len())` へ value/action を書く（列の全 θ を 1 回で）。
    fn write_column(&mut self, base: usize, values: &[u64], actions: &[i32]);
    /// orig セルの (value, action) を読む。action<0 は None。
    fn read(&self, orig: usize) -> (u64, i32);
}

/// 既定の RAM 実装（2 本の `Vec`）。未書き込みセル（到達不能/パディング）は `(MAX_COST, -1)`。
pub struct RamSink {
    total: Vec<u64>,
    action: Vec<i32>,
}

impl RamSink {
    pub fn new(nstates: usize) -> Self {
        Self { total: vec![MAX_COST; nstates], action: vec![-1; nstates] }
    }
}

impl CompactSink for RamSink {
    #[inline]
    fn write_column(&mut self, base: usize, values: &[u64], actions: &[i32]) {
        self.total[base..base + values.len()].copy_from_slice(values);
        self.action[base..base + actions.len()].copy_from_slice(actions);
    }
    #[inline]
    fn read(&self, orig: usize) -> (u64, i32) {
        (self.total[orig], self.action[orig])
    }
}

/// 確定出力をディスク mmap に置く [`CompactSink`] 実装。value (u64 LE) と policy (i32 LE) を
/// 2 ファイル (`compact_value.bin` / `compact_action.bin`) に分け、finalize 時は列連続で
/// write、ロールアウト時は orig 単位で read する。未書き込み = 到達不能 → `(MAX_COST, -1)`。
///
/// 用途: 広域地図では確定出力が `nx·ny·nθ × 12 B` になる (0.15 m/cell の津田沼で
/// 157M states = 1.9 GB)。Pi4 4GB では RAM に置けないのでディスクへ外す。
/// かつて vi_planner / vi_global_planner / bench_map が各自コピーを持っていた実装の一本化。
pub struct MmapSink {
    value: memmap2::MmapMut,  // nstates * 8 bytes
    action: memmap2::MmapMut, // nstates * 4 bytes
    /// 捨てるときにディレクトリごと消すか (`new_owned` で作ったときだけ Some)。
    /// 先読み (waypoint_prefetch) は場を 2 つ同時に生かすので solve ごとに別
    /// ディレクトリを使う。誰も消さないと 1 巡ごとに sink が 1 つ積み上がる。
    owned_dir: Option<std::path::PathBuf>,
}

impl MmapSink {
    pub fn new(dir: &std::path::Path, nstates: usize) -> std::io::Result<Self> {
        Self::open(dir, nstates, None)
    }

    /// `new` と同じだが、**この sink がディレクトリを所有する** — 捨てるときに
    /// `dir` ごと消す。solve のたびに使い捨てのディレクトリを切る呼び出し用。
    pub fn new_owned(dir: &std::path::Path, nstates: usize) -> std::io::Result<Self> {
        Self::open(dir, nstates, Some(dir.to_path_buf()))
    }

    fn open(
        dir: &std::path::Path,
        nstates: usize,
        owned_dir: Option<std::path::PathBuf>,
    ) -> std::io::Result<Self> {
        std::fs::create_dir_all(dir)?;
        let map = |name: &str, bytes: usize| -> std::io::Result<memmap2::MmapMut> {
            let f = std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(true)
                .open(dir.join(name))?;
            f.set_len(bytes as u64)?;
            unsafe { memmap2::MmapMut::map_mut(&f) }
        };
        let mut value = map("compact_value.bin", nstates * 8)?;
        let mut action = map("compact_action.bin", nstates * 4)?;
        // 初期化: 未書き込み (到達不能) セルが (MAX_COST, None) と読めるように。
        let le = MAX_COST.to_le_bytes();
        for rec in value.chunks_exact_mut(8) {
            rec.copy_from_slice(&le);
        }
        action.fill(0xFF); // i32 -1 = 全バイト 0xFF。
        Ok(Self { value, action, owned_dir })
    }
}

impl Drop for MmapSink {
    fn drop(&mut self) {
        let Some(dir) = self.owned_dir.as_ref() else { return };
        // マッピングはこの後 (フィールドの drop で) 外れるが、Linux では
        // mmap 中のファイルを unlink してよい (inode は写像が生きている間残る)。
        // Windows では拒否されるので、握り潰して残す。
        let _ = std::fs::remove_dir_all(dir);
    }
}

impl CompactSink for MmapSink {
    fn write_column(&mut self, base: usize, values: &[u64], actions: &[i32]) {
        let vb = &mut self.value[base * 8..(base + values.len()) * 8];
        for (i, &v) in values.iter().enumerate() {
            vb[i * 8..i * 8 + 8].copy_from_slice(&v.to_le_bytes());
        }
        let ab = &mut self.action[base * 4..(base + actions.len()) * 4];
        for (i, &a) in actions.iter().enumerate() {
            ab[i * 4..i * 4 + 4].copy_from_slice(&a.to_le_bytes());
        }
    }

    fn read(&self, orig: usize) -> (u64, i32) {
        let v = u64::from_le_bytes(self.value[orig * 8..orig * 8 + 8].try_into().unwrap());
        let a = i32::from_le_bytes(self.action[orig * 4..orig * 4 + 4].try_into().unwrap());
        (v, a)
    }
}

/// 詳細統計（テスト/退避判定/ベンチで使う）。
pub struct CompactStats {
    pub iters: u32,
    pub updates: u64,
    pub converged: bool,
    /// `converged == false` の内訳: `cancel` フラグで中断されたなら true、
    /// `max_iter` 打ち切りなら false。中断時の `sink` の内容は未完成なので使えない。
    pub cancelled: bool,
    /// `converged == false` のもう 1 つの内訳: observer の `Stop` が打ち切ったなら true
    /// （observed 系入口を使ったときだけ立つ）。**`cancelled` と違い
    /// `sink` はそのまま使える** — 書いてあるのは finalize 済みの列だけで、それらは
    /// 定義から v* 到達済み＝収束解と bit-exact に同じ値。未 finalize の列が
    /// `(MAX_COST, -1)` のまま残っているだけ。
    pub stopped: bool,
    /// final 化された eval_ok セル数。
    pub finalized: u64,
    /// 到達可能（eval_ok かつ `cp != UNREACHED`）セル数。
    pub reachable: u64,
    /// 退避後の常駐列数（＝非 final の到達列＝値バンド）のピーク。
    pub peak_resident_cols: u64,
    /// 到達した列数（値バンドの削減効果の比較基準）。
    pub reachable_cols: u64,
    /// 退避したブロック数。
    pub freed_blocks: u64,
    /// 常駐ブロック数（確保済み・未退避）のピーク。遅延確保＋退避が peak RAM を抑える指標。
    pub peak_resident_blocks: u64,
    /// 常駐ブロックのヒープ実測バイト数（cp/pen/eval/fin）のピーク。ブロック数と違い
    /// レイアウト（per-cell/列単位）の違いが数字に出るので、RAM 削減の比較はこちらで行う。
    pub peak_resident_block_bytes: u64,
    /// 総ブロック数。
    pub total_blocks: u64,
    /// 源（MapSource）の常駐ヒープバイト数 = O(nx·ny) メモリ床。SliceSource 経路は 0。
    pub map_floor_bytes: u64,
}

/// 波内バンドを**並列非同期 Gauss–Seidel × θ マスク疎評価**で収束させる
/// （`nthreads==1` なら単一ワーカー = 直列 G-S と同じ評価順、収束値は常に bit-exact）。
/// `frontier2d_fused`/`frontier2d_sparse` の async + θマスクを out-of-core
/// ブロックへ移植したもの。各ラウンド: ① 直列で frontier 全列の窓ブロックを `ensure_window` 確保
/// ② 確保済みブロックの cp/pen/eval 生ポインタ表（`AtomicCpView`）を作り、各ワーカーが担当列を
/// 評価して cp を **in-place 原子書き込み**。評価する θ は、窓 (±mx,±my) の前ラウンド変化マスクを
/// OR gather → `rot_dilate(mt)` で得た依存集合だけ（`acc==0` の列は丸ごとスキップ）③ join 後に直列で
/// col_min/col_max/live/変化マスク/次フロンティアを構築。
///
/// **波ごと初回フル評価**: `mask` は波の収束で自然にクリアされる（最終ラウンドは伝播変化ゼロ →
/// store なし、前ラウンド分は zero 化）。各波の round 1 は `first=true` で全 θ を評価（依存集合の
/// 保守的上位集合）し、マスクを育ててから round 2+ で疎評価に入る。マスク store は **in-band 変化**
/// （`cmask!=0 && mn<t`）の列のみ＝バンド外変化（高値、in-band を下げられない=正コスト SSP）は
/// 伝播させない。これにより評価集合は真の依存集合の保守的上位集合のままなので、「1 ラウンド丸ごと
/// 無変化」での収束時に全 (cell,θ) が Bellman 整合 = 一意固定点 = 本家と bit-exact。
#[allow(clippy::too_many_arguments)]
fn converge_band_async(
    store: &mut BlockStore,
    g: &Geom,
    src: &dyn StateSource,
    nthreads: usize,
    nx: i32,
    ny: i32,
    dx: u32,
    dy: u32,
    t: u64,
    max_iter: u32,
    cancel: &AtomicBool,
    frontier: &mut Vec<(u32, u32)>,
    col_min: &mut [u64],
    col_max: &mut [u64],
    col_final: &[bool],
    reached: &mut [bool],
    live: &mut Vec<usize>,
    iters: &mut u32,
    total_updates: &mut u64,
    mask: &mut [u64],
    mw: usize,
    mt: i32,
    full_mask: u64,
) -> bool {
    // 並列 compute がワーカーから返す 1 列分の結果 `(列index i, mn, mx, ups, 変化θマスク)`。
    type ColEval = (usize, u64, u64, u64, u64);
    let cidx = |ix: i32, iy: i32| (iy * nx + ix) as usize;
    let (mx, my, nt) = (g.mx, g.my, g.nt);
    let midx = |ix: i32, iy: i32| (ix + mx) as usize + (iy + my) as usize * mw;
    // 前ラウンドにマスクを立てたセル（次ラウンド冒頭でゼロ化する。波末でも掃除）。
    let mut prev_cells: Vec<(u32, u32)> = Vec::new();
    let mut first = true;
    let outcome = loop {
        if cancel.load(Ordering::Relaxed) {
            break false;
        }
        // ① 直列: frontier 全列の窓ブロックを確保（並列 compute 中は確保/退避できないため）。
        for &(ixu, iyu) in frontier.iter() {
            ensure_window(store, ixu as i32, iyu as i32, src);
        }
        // ② 並列 async G-S × θ疎評価。各列は 1 チャンク = 1 スレッドが排他担当（単一書き手）。
        //    戻り値は評価した列の `(i, mn, mx, ups, cmask)`。view/mask は本スコープ中 store を
        //    可変借用しない（確保/退避なし）。mask は読み取り専用（書きは③直列）。
        let view = store.atomic_view();
        const MIN_COLS_PER_THREAD: usize = 64;
        let eff = nthreads.min((frontier.len() / MIN_COLS_PER_THREAD).max(1));
        let chunk = frontier.len().div_ceil(eff).max(1);
        let results: Vec<Vec<ColEval>> = {
            let view_ref = &view;
            let mask_ref: &[u64] = mask;
            std::thread::scope(|scope| {
                let handles: Vec<_> = frontier
                    .chunks(chunk)
                    .map(|part| {
                        scope.spawn(move || {
                            let mut out: Vec<ColEval> = Vec::new();
                            for &(ixu, iyu) in part {
                                let (ix, iy) = (ixu as i32, iyu as i32);
                                let i = (iy * nx + ix) as usize;
                                if col_final[i] {
                                    continue;
                                }
                                // θ マスク gather: 窓 (±mx,±my) の前ラウンド変化マスクを OR。
                                let eval_mask = if first {
                                    full_mask // 波初回はマスク未育成 → 全 θ（保守的）。
                                } else {
                                    let mut acc = 0u64;
                                    for dy2 in -my..=my {
                                        let base = ix as usize + (iy + dy2 + my) as usize * mw;
                                        for k in 0..(2 * mx + 1) as usize {
                                            acc |= mask_ref[base + k];
                                        }
                                    }
                                    if acc == 0 {
                                        continue; // 依存入力に変化なし → 列ごとスキップ。
                                    }
                                    rot_dilate(acc, mt, nt)
                                };
                                let (mn, mx_, ups, cmask) =
                                    relax_column_async_masked(view_ref, g, ix, iy, eval_mask);
                                out.push((i, mn, mx_, ups, cmask));
                            }
                            out
                        })
                    })
                    .collect();
                handles.into_iter().map(|h| h.join().unwrap()).collect()
            })
        };
        drop(view); // 生ポインタ表を破棄してから store/mask を可変アクセスする。
        // ③ 直列 apply: 前ラウンドのマスクをゼロ化 → 今ラウンドの in-band 変化マスクを store。
        for &(x, y) in &prev_cells {
            mask[midx(x as i32, y as i32)] = 0;
        }
        prev_cells.clear();
        let mut changed = Bitboard2D::new(nx as u32, ny as u32);
        let mut any = false;
        for part in &results {
            for &(i, mn, mx_, ups, cmask) in part {
                col_min[i] = mn;
                col_max[i] = mx_;
                if mn != MAX_COST && !reached[i] {
                    reached[i] = true;
                    live.push(i);
                }
                *total_updates += ups;
                // in-band 変化のみ伝播（バンド外＝高値は in-band を下げられない＝正コスト SSP）。
                if cmask != 0 && mn != MAX_COST && mn < t {
                    any = true;
                    let ix = (i % nx as usize) as i32;
                    let iy = (i / nx as usize) as i32;
                    mask[midx(ix, iy)] = cmask;
                    prev_cells.push((ix as u32, iy as u32));
                    changed.set(ix as u32, iy as u32);
                }
            }
        }
        first = false;
        *iters += 1;
        if *iters >= max_iter {
            break false;
        }
        if !any {
            break true;
        }
        *frontier = changed
            .dilate(dx, dy)
            .enumerate()
            .filter(|&(ixu, iyu)| !col_final[cidx(ixu as i32, iyu as i32)])
            .collect();
    };
    // 波末: 残った変化マスクをゼロ化して次の波に持ち越さない（収束時はほぼ空、max_iter 時も掃除）。
    for &(x, y) in &prev_cells {
        mask[midx(x as i32, y as i32)] = 0;
    }
    outcome
}

/// 値バンド閾値スキャン + during-solve ウォーターマーク finalization + 遅延確保 + 退避の
/// slice (states あり) 入口、スレッド数明示版。`nthreads == 1` は直列 G-S、`>= 2` は波内 relax を
/// 並列非同期 G-S（`converge_band_async`、cp を in-place 原子書き込み）で回す。固定点は単調降下で
/// 一意なので、収束値・方策はスレッド数に依らず本家と bit-exact（iters は非決定的）。テストが
/// nthreads 1/4 両方を固定して value+policy parity を検証する
/// (現在の呼び出し元はテストのみ — 実行系の slice 入口は `solve_compact_observed`)。
#[cfg(test)]
fn solve_compact_into_nthreads(
    vi: &mut ValueIterator,
    max_iter: u32,
    band_override: Option<u64>,
    sink: &mut dyn CompactSink,
    nthreads: usize,
) -> CompactStats {
    let g = Geom::build(vi);
    let mt = displacement(vi).2;
    let never = AtomicBool::new(false); // slice 経路に中断はない（`solve_compact_mapped` のみ対応）。
    // states を源にバンドスキャンを回す（src の借用は core 内で完結し、write_back と競合しない）。
    let mut stats = {
        let src = SliceSource::new(&vi.states, g.nx, g.nt);
        solve_compact_core(
            &g,
            mt,
            &src,
            max_iter,
            band_override,
            sink,
            nthreads,
            &never,
            &mut |_| false,
        )
    };
    write_back_sink(vi, &g, sink);
    // slice 経路は states があるので reachable を厳密にカウント（mapped 経路は core 内の finalized）。
    stats.reachable = vi
        .states
        .iter()
        .filter(|s| s.free && !s.final_state && s.total_cost < MAX_COST)
        .count() as u64;
    stats
}

/// compact の境界プローブ: finalize 済み sink を [`CompactPolicy`] として観測する。
/// finalize は値の昇順に進むので、見える列は最後まで解いたときと bit-exact に同じ
/// (`Partiality::ExactPrefix`)。`iters` は境界呼び出し回数 (バンド finalize 回数)。
/// `updates` は core が境界へ運ばないため 0 (best effort)。
struct CompactProbe<'a> {
    policy: CompactPolicy<'a>,
    iters: u32,
}

impl SolveProbe for CompactProbe<'_> {
    fn iters(&self) -> u32 {
        self.iters
    }
    fn updates(&self) -> u64 {
        0
    }
    fn policy(&mut self) -> &dyn PolicyView {
        &self.policy
    }
}

/// observer を core の `stop` クロージャへ写すアダプタ (observed 系入口の共通部)。
/// 境界はバンド finalize ごと (compact の自然な粒度で、`interval` ヒントより粗い)。
/// [`SolveFlow::Cancel`] は `obs_cancel` に立てて打ち切る (呼び出し側が `cancelled` へ写す)。
fn observer_stop<'a>(
    vi: &'a ValueIterator,
    obs: &'a mut dyn SolveObserver,
    obs_cancel: &'a AtomicBool,
) -> impl FnMut(&dyn CompactSink) -> bool + 'a {
    let cell_num = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let res = vi.xy_resolution;
    let origin = (vi.map_origin_x, vi.map_origin_y);
    let goal = (vi.goal_x, vi.goal_y, vi.goal_t);
    let actions = &vi.actions;
    let mut boundaries = 0u32;
    move |sink: &dyn CompactSink| -> bool {
        boundaries += 1;
        let mut probe = CompactProbe {
            policy: CompactPolicy::new(sink, actions, cell_num, res, origin, goal),
            iters: boundaries,
        };
        match obs.boundary(&mut probe) {
            SolveFlow::Continue => false,
            SolveFlow::Stop => true,
            SolveFlow::Cancel => {
                obs_cancel.store(true, Ordering::Relaxed);
                true
            }
        }
    }
}

/// slice (states あり) 入口の observer 付き版 (`U64Solver::Frontier2DSparseCompact` の
/// `solve_observed` 経路)。observer が `Cancel` を返したら core を止め、`cancelled=true` を
/// 返す (このとき sink / states は未完成なので write_back しない)。`Stop` は sink がそのまま
/// 使えるので、確定分を states へ書き戻して返す。
pub fn solve_compact_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    band_override: Option<u64>,
    obs: &mut dyn SolveObserver,
) -> CompactStats {
    let mut sink = RamSink::new(vi.states.len());
    let g = Geom::build(vi);
    let mt = displacement(vi).2;
    let obs_cancel = AtomicBool::new(false);
    let never = AtomicBool::new(false); // slice 経路に外部中断フラグはない。
    let mut stats = {
        let src = SliceSource::new(&vi.states, g.nx, g.nt);
        let mut stop = observer_stop(vi, obs, &obs_cancel);
        solve_compact_core(
            &g,
            mt,
            &src,
            max_iter,
            band_override,
            &mut sink,
            n_threads(),
            &never,
            &mut stop,
        )
    };
    if obs_cancel.load(Ordering::Relaxed) {
        stats.cancelled = true;
        stats.stopped = false;
        return stats; // sink は未完成 — states には書き戻さない (場は破棄扱い)。
    }
    write_back_sink(vi, &g, &sink);
    stats.reachable = vi
        .states
        .iter()
        .filter(|s| s.free && !s.final_state && s.total_cost < MAX_COST)
        .count() as u64;
    stats
}

/// マップ + ゴールから **states を構築せず**解く mapped 入口の observer 付き版
/// (vi_planner の統一入口)。geometry/transitions だけの `ValueIterator` と `MapSource` を
/// 組み、出力は `sink` に確定する (write_back なし)。`cancel` (外部プリエンプトフラグ、
/// バンド内ラウンド境界で観測) はそのまま生かし、observer の `Cancel` はバンド finalize
/// 境界で `cancelled=true` に写す。observer の `Stop` で打ち切った sink はそのまま使える
/// (finalize は値の昇順に進み、確定列は v* 到達済みで bit-exact — 未確定列は
/// `(MAX_COST, -1)` のまま)。
#[allow(clippy::too_many_arguments)]
pub fn solve_compact_mapped_observed(
    actions: Vec<Action>,
    thread_num: i32,
    map: &OccupancyGrid,
    theta_cell_num: i32,
    safety_radius: f64,
    safety_radius_penalty: f64,
    goal_margin_radius: f64,
    goal_margin_theta: i32,
    goal_x: f64,
    goal_y: f64,
    goal_t: i32,
    max_iter: u32,
    band_override: Option<u64>,
    sink: &mut dyn CompactSink,
    nthreads: usize,
    cancel: &AtomicBool,
    obs: &mut dyn SolveObserver,
) -> CompactStats {
    let mut vi = ValueIterator::new(actions, thread_num);
    // geometry + transitions のみ（states / sweep_orders は作らない = O(total) 確保なし）。
    vi.set_map_geometry_no_states(map, theta_cell_num, goal_margin_radius, goal_margin_theta);
    // ゴール設定（states は空なので set_state_values は no-op、goal フィールドだけ更新）。
    vi.set_goal(goal_x, goal_y, goal_t);
    let g = Geom::build(&vi);
    let mt = displacement(&vi).2;
    let src = MapSource::build(map, &vi, safety_radius, safety_radius_penalty);
    let obs_cancel = AtomicBool::new(false);
    let mut stats = {
        let mut stop = observer_stop(&vi, obs, &obs_cancel);
        solve_compact_core(
            &g, mt, &src, max_iter, band_override, sink, nthreads, cancel, &mut stop,
        )
    };
    if obs_cancel.load(Ordering::Relaxed) {
        stats.cancelled = true;
        stats.stopped = false;
    }
    stats
}

/// マップ + ゴールから **states を構築せず**ブロックタイル・アウトオブコアで解く（メモリ床を O(nx·ny)
/// に下げる）。`vi.states`（O(total)）を一切確保せず、geometry/transitions だけ持つ `ValueIterator`
/// を作り `MapSource` を源にする。出力は `sink`（ディスク mmap 推奨）に確定し、write_back はしない
/// （結果は sink にある）。到達可能セルの収束値・方策は slice 経路（= 本家）と bit-exact。
///
/// `cancel` はバンド内ラウンド境界で観測される中断フラグ。立てると `converged=false,
/// cancelled=true` で即座に戻る（`sink` の内容は未完成なので破棄すること）。中断を使わない
/// 呼び出し側は `&AtomicBool::new(false)` を渡す。ROS プランナがゴール変更でプリエンプトする
/// ために必要（solve は巨大マップで数十秒〜数分ブロックする）。
///
/// **途中まで解けた出力を使いたい**（走り出しを早める等）なら observer で `Stop` を返す
/// [`solve_compact_mapped_observed`] のほう。`cancel` で止めた `sink` は使えないが、
/// あちらで止めた `sink` は使える。
#[allow(clippy::too_many_arguments)]
pub fn solve_compact_mapped(
    actions: Vec<Action>,
    thread_num: i32,
    map: &OccupancyGrid,
    theta_cell_num: i32,
    safety_radius: f64,
    safety_radius_penalty: f64,
    goal_margin_radius: f64,
    goal_margin_theta: i32,
    goal_x: f64,
    goal_y: f64,
    goal_t: i32,
    max_iter: u32,
    band_override: Option<u64>,
    sink: &mut dyn CompactSink,
    nthreads: usize,
    cancel: &AtomicBool,
) -> CompactStats {
    solve_compact_mapped_observed(
        actions,
        thread_num,
        map,
        theta_cell_num,
        safety_radius,
        safety_radius_penalty,
        goal_margin_radius,
        goal_margin_theta,
        goal_x,
        goal_y,
        goal_t,
        max_iter,
        band_override,
        sink,
        nthreads,
        cancel,
        &mut NullObserver,
    )
}

/// 既定スレッド数（環境変数 `VI_THREADS` 上書き可）。`solve_compact_mapped` にスレッド数を渡すための
/// 公開ヘルパ（内部 `n_threads()` は crate 内可視のため）。
pub fn default_threads() -> usize {
    n_threads()
}

/// states を作らず goal(final) セル数を数える（bench のサニティ表示用、O(nx·ny)）。geometry-only vi +
/// MapSource を構築して final 集合サイズを返す。
#[allow(clippy::too_many_arguments)]
pub fn mapped_goal_cell_count(
    actions: Vec<Action>,
    map: &OccupancyGrid,
    theta_cell_num: i32,
    safety_radius: f64,
    safety_radius_penalty: f64,
    goal_margin_radius: f64,
    goal_margin_theta: i32,
    goal_x: f64,
    goal_y: f64,
    goal_t: i32,
) -> usize {
    let mut vi = ValueIterator::new(actions, 1);
    vi.set_map_geometry_no_states(map, theta_cell_num, goal_margin_radius, goal_margin_theta);
    vi.set_goal(goal_x, goal_y, goal_t);
    MapSource::build(map, &vi, safety_radius, safety_radius_penalty).finals.len()
}

/// バンドスキャン本体（源 `src` 非依存）。`slice`/`mapped` 両入口が共有する。出力は `sink` に確定し、
/// write_back はしない（呼び出し側の責務）。`reachable` は finalized 数で近似（slice 入口が上書きする）。
///
/// `stop` は**波の finalize が終わるたび**に呼ばれ、`true` を返すとそこで打ち切る
/// （`converged=false, stopped=true`）。渡されるのはその時点の `sink` = finalize 済みの
/// 確定出力だけなので、呼び出し側は「もう十分か」を出力そのもので判定できる。使わない
/// 呼び出し側は `&mut |_| false` を渡す。
#[allow(clippy::too_many_arguments)]
fn solve_compact_core(
    g: &Geom,
    mt: i32,
    src: &dyn StateSource,
    max_iter: u32,
    band_override: Option<u64>,
    sink: &mut dyn CompactSink,
    nthreads: usize,
    cancel: &AtomicBool,
    stop: &mut dyn FnMut(&dyn CompactSink) -> bool,
) -> CompactStats {
    let (nx, ny) = (g.nx, g.ny);
    let (dx, dy) = (g.mx as u32, g.my as u32);
    let ncol = (nx * ny) as usize;
    let cidx = |ix: i32, iy: i32| (iy * nx + ix) as usize;

    let halo_blocks = (g.my as usize).div_ceil(BLOCK_ROWS);

    // θ マスク疎評価（並列 async 経路のみ）。変化 θ ビットマスク (u64) の padding 付き 2D 配列。
    // O(ncol) u64（cp の 1/nt）で、compact が常駐させる col_* 配列と同オーダー。`mt` は θ 円環膨張半径。
    assert!(g.nt <= 64, "θ マスクは u64 前提 (nt={})", g.nt);
    let mw = (nx + 2 * g.mx) as usize;
    let full_mask: u64 = if g.nt >= 64 { u64::MAX } else { (1u64 << g.nt) - 1 };
    let mut mask: Vec<u64> = vec![0u64; mw * (ny + 2 * g.my) as usize];

    let mut iters = 0u32;
    let mut total_updates = 0u64;
    let mut finalized = 0u64;
    let mut peak_resident_cols = 0u64;
    let mut peak_resident_blocks = 0u64;
    let mut peak_resident_block_bytes = 0u64;
    let mut freed_blocks = 0u64;

    // 初期フロンティア（ゴール列を依存窓で膨張）。seed は src.for_each_seed（ゴール局所）から作る。
    let mut seed_bb = Bitboard2D::new(nx as u32, ny as u32);
    src.for_each_seed(&mut |ix, iy, _it| seed_bb.set(ix as u32, iy as u32));
    let mut frontier: Vec<(u32, u32)> = seed_bb.dilate(dx, dy).enumerate().collect();

    let mut store = BlockStore::new(g, src);
    let band = band_override.unwrap_or_else(|| couple_margin(g, store.max_pen));

    // 列ごとの現在値域と final フラグ。seed セル（final、value=0）で初期化する。
    // `live` = 到達済み（`col_min != MAX`）かつ非 final の列インデックス集合。波ごとの
    // finalize/再活性/終了判定/常駐ピークは全列 O(ncol) 走査ではなく `live` の反復で行う
    // （バンド ≪ 全グリッドなので走査が O(band) になる）。`reached` で二重 push を防ぐ。
    let mut col_min = vec![MAX_COST; ncol];
    let mut col_max = vec![MAX_COST; ncol];
    let mut col_final = vec![false; ncol];
    let mut reached = vec![false; ncol];
    let mut live: Vec<usize> = Vec::new();
    src.for_each_seed(&mut |ix, iy, _it| {
        let i = cidx(ix, iy);
        if !reached[i] {
            reached[i] = true;
            live.push(i);
            col_min[i] = 0; // seed value = total_cost = 0
            col_max[i] = 0;
        }
    });

    let mut t = band;
    let mut stopped = false;
    let converged = 'outer: loop {
        // ── 波: バンド [.., t) を収束。relax は隣接（未到達含む）を発見、伝播は in-band 列のみ。
        //    async G-S × θマスク疎評価（nthreads==1 は単一ワーカー）。max_iter 到達なら
        //    'outer を false で抜ける。 ──
        let band_ok = converge_band_async(
            &mut store, g, src, nthreads, nx, ny, dx, dy, t, max_iter, cancel, &mut frontier,
            &mut col_min, &mut col_max, &col_final, &mut reached, &mut live, &mut iters,
            &mut total_updates, &mut mask, mw, mt, full_mask,
        );
        if !band_ok {
            break 'outer false;
        }

        // ── ウォーターマーク finalize: col_max ≤ T − band。live のみ走査。 ──
        // finalize した列が属する行ブロック（列 (ix,iy) → ブロック (iy+my)/BLOCK_ROWS）を
        // 記録し、退避はその近傍ブロックだけ調べる（O(nblocks) 全走査を回避）。
        let wm = t.saturating_sub(band);
        let mut touched_blocks: Vec<usize> = Vec::new();
        for &i in &live {
            if col_max[i] <= wm {
                let ix = (i % nx as usize) as i32;
                let iy = (i / nx as usize) as i32;
                finalized += finalize_column(&mut store, g, ix, iy, src, sink);
                col_final[i] = true;
                touched_blocks.push((iy + store.my) as usize / BLOCK_ROWS);
            }
        }
        live.retain(|&i| !col_final[i]);

        // ── 退避: finalize 列の近傍ブロック（±halo）のうち interior-final のものを解放。 ──
        // ブロック b が新たに退避可能になるのは b±halo の最後の非 full ブロックが full 化した瞬間 =
        // b±halo 内の列が finalize した波。その列の b_col は b±halo に入る ⟺ b は b_col±halo に入る
        // ので、touched_blocks±halo を候補にすれば取りこぼさない（健全性は元の全走査と同一）。
        if !touched_blocks.is_empty() {
            let nb = store.nblocks();
            touched_blocks.sort_unstable();
            touched_blocks.dedup();
            let mut cand: Vec<usize> = Vec::new();
            for &b in &touched_blocks {
                let lo = b.saturating_sub(halo_blocks);
                let hi = (b + halo_blocks).min(nb - 1);
                cand.extend(lo..=hi);
            }
            cand.sort_unstable();
            cand.dedup();
            for b in cand {
                if store.evicted[b] || store.blocks[b].is_none() {
                    continue; // 退避済み or 未確保（空/パディング）は対象外。
                }
                let lo = b.saturating_sub(halo_blocks);
                let hi = (b + halo_blocks).min(nb - 1);
                if (lo..=hi).all(|k| store.block_full(k)) {
                    store.evict_block(b);
                    freed_blocks += 1;
                }
            }
        }

        peak_resident_blocks = peak_resident_blocks.max(store.resident_blocks() as u64);
        peak_resident_block_bytes = peak_resident_block_bytes.max(store.resident_bytes());
        peak_resident_cols = peak_resident_cols.max(live.len() as u64);

        // ── 終了判定: 到達済み非 final 列（= live）が残っていない ──
        if live.is_empty() {
            break true;
        }

        // ── 打ち切り判定: ここまでの確定出力で足りるか呼び出し側に訊く ──
        // 全部解けたかどうかの判定 (直上) の**後**に置く。最後の波で解き切ったなら
        // `stopped` ではなく `converged` を返したい（打ち切りは「まだ残っているのに
        // 止めた」ときだけの状態）。
        if stop(&*sink) {
            stopped = true;
            break 'outer false;
        }

        // T を次バンドへ進め、deferred（live かつ col_min < 新 T）を膨張して再活性。
        t = t.saturating_add(band);
        let mut react = Bitboard2D::new(nx as u32, ny as u32);
        for &i in &live {
            if col_min[i] < t {
                let ix = (i % nx as usize) as u32;
                let iy = (i / nx as usize) as u32;
                react.set(ix, iy);
            }
        }
        frontier = react
            .dilate(dx, dy)
            .enumerate()
            .filter(|&(ixu, iyu)| !col_final[cidx(ixu as i32, iyu as i32)])
            .collect();
    };

    let total_blocks = store.nblocks() as u64;
    drop(store); // 残常駐ブロックも解放（出力は sink に確定済み）。src 借用もここで終わる。

    let reachable_cols = (0..ncol).filter(|&i| col_max[i] != MAX_COST).count() as u64;

    CompactStats {
        iters,
        updates: total_updates,
        converged,
        // 非収束の内訳: stop が打ち切ったならそれ、cancel が立っていれば中断、
        // どちらでもなければ max_iter 打ち切り。
        cancelled: !converged && !stopped && cancel.load(Ordering::Relaxed),
        stopped,
        finalized,
        // 収束時は finalized == 到達可能 eval セル数。slice 入口は states から厳密値で上書きする。
        reachable: finalized,
        peak_resident_cols,
        reachable_cols,
        freed_blocks,
        peak_resident_blocks,
        peak_resident_block_bytes,
        total_blocks,
        map_floor_bytes: src.floor_bytes(),
    }
}

/// 確定出力 sink（finalize 時に保存済み）から states へ値・方策を書き戻す。退避でブロックが解放
/// されていても sink に残るので store には触れない。orig 索引 `it + ix·nt + iy·nt·nx`。
fn write_back_sink(vi: &mut ValueIterator, g: &Geom, sink: &dyn CompactSink) {
    let (nt, nx) = (g.nt, g.nx);
    for s in vi.states.iter_mut() {
        let orig = orig_index(s.ix, s.iy, s.it, nx, nt);
        let (v, a) = sink.read(orig);
        s.total_cost = v;
        s.optimal_action = if a < 0 { None } else { Some(a as usize) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 旧 pub `solve_compact` 相当のテスト用ラッパ (RAM sink + 既定スレッド数)。
    fn solve_compact(
        vi: &mut ValueIterator,
        max_iter: u32,
        band_override: Option<u64>,
    ) -> CompactStats {
        let mut sink = RamSink::new(vi.states.len());
        solve_compact_into_nthreads(vi, max_iter, band_override, &mut sink, n_threads())
    }

    /// 多数の行ブロックに跨る大きめ空マップで遅延 gather + 値バンドを刺激する。
    /// (標準3マップの parity は conformance が全ソルバ横断でゲートする。)
    #[test]
    fn parity_larger_empty_compact() {
        crate::solvers::test_support::assert_parity(32, 24, vec![0i8; 32 * 24], |vi| {
            let s = solve_compact(vi, 4000, None);
            (s.iters, s.updates, s.converged)
        });
    }

    /// nthreads==1（async 経路の単一ワーカー）を機械のコア数に依らず固定して
    /// value+policy parity を検証。
    #[test]
    fn parity_serial_nthreads1_compact() {
        crate::solvers::test_support::parity_standard_maps(|vi| {
            let mut sink = RamSink::new(vi.states.len());
            let s = solve_compact_into_nthreads(vi, 4000, None, &mut sink, 1);
            (s.iters, s.updates, s.converged)
        });
    }

    /// 並列非同期 G-S パス（nthreads==4）を固定して value+policy parity を検証。固定点は単調降下で
    /// 一意なのでスレッド数・チャンク分割・スケジュールに依らず収束値・方策は本家と bit-exact。
    #[test]
    fn parity_parallel_nthreads4_compact() {
        crate::solvers::test_support::parity_standard_maps(|vi| {
            let mut sink = RamSink::new(vi.states.len());
            let s = solve_compact_into_nthreads(vi, 4000, None, &mut sink, 4);
            (s.iters, s.updates, s.converged)
        });
    }

    /// 複数行ブロックに跨る大きめ空マップ × 並列非同期 G-S（nthreads==4）で、チャンク境界を跨ぐ
    /// 原子 in-place 書き込み・隣接読み（別スレッド担当列）と退避を同時に刺激し bit-exact を確認。
    #[test]
    fn parity_parallel_larger_nthreads4_compact() {
        use crate::solvers::test_support::{make_vi, run_reference_to_fixed_point, REACH};
        let (w, h) = (32, 24);
        let occ = vec![0i8; (w * h) as usize];
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);

        let mut sink = RamSink::new(b.states.len());
        let s = solve_compact_into_nthreads(&mut b, 4000, None, &mut sink, 4);
        assert!(s.converged, "parallel Jacobi must converge");
        let mut mism = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH {
                if a.states[i].total_cost != b.states[i].total_cost {
                    mism += 1;
                }
                assert_eq!(
                    a.states[i].optimal_action, b.states[i].optimal_action,
                    "policy mismatch @ state {i}"
                );
            }
        }
        assert_eq!(mism, 0, "parallel Jacobi value must be bit-exact with reference");
    }

    /// 収束後、到達可能な eval_ok セルは全て final 化されるべき。
    #[test]
    fn finalizes_all_reachable_cells() {
        let mut vi = crate::solvers::test_support::make_vi(8, 8, vec![0i8; 64]);
        let stats = solve_compact(&mut vi, 4000, None);
        assert!(stats.converged, "solver must converge");
        assert!(stats.reachable > 0, "到達可能セルが存在するはず");
        assert_eq!(
            stats.finalized, stats.reachable,
            "全到達可能セルが final 化されるべき (finalized={}, reachable={})",
            stats.finalized, stats.reachable
        );
    }

    /// 小さい値バンドでも bit-exact を保ちつつ、退避後の常駐列を到達列より小さく抑える。
    #[test]
    fn value_band_bounds_and_exact() {
        use crate::params::PROB_BASE;
        use crate::solvers::test_support::{make_vi, run_reference_to_fixed_point, REACH};
        let (w, h) = (64, 8);
        let occ = vec![0i8; (w * h) as usize];
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);

        let band = 3 * PROB_BASE;
        let stats = solve_compact(&mut b, 8000, Some(band));
        assert!(stats.converged, "must converge");

        let mut mism = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH && a.states[i].total_cost != b.states[i].total_cost {
                mism += 1;
            }
        }
        assert_eq!(mism, 0, "bit-exact mismatch (band={band})");
        assert!(
            stats.peak_resident_cols < stats.reachable_cols,
            "band should bound resident (peak={}, reachable_cols={}, band={})",
            stats.peak_resident_cols, stats.reachable_cols, band
        );
    }

    /// θ マスク疎評価 × 値バンド × 退避を **nthreads=4 固定**で同時に刺激し bit-exact を確認。
    /// 小バンド（多波）＋縦長マップで「波ごとマスク育成→疎評価→波末クリア→再活性」の往復と、
    /// θ マスク gather/rot_dilate/acc==0 スキップを機械のコア数に依らず決定的に踏む。
    #[test]
    fn theta_mask_band_eviction_parallel_exact() {
        use crate::params::PROB_BASE;
        use crate::solvers::test_support::{make_vi, run_reference_to_fixed_point, REACH};
        let (w, h) = (16, 96);
        let occ = vec![0i8; (w * h) as usize];
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);

        let mut sink = RamSink::new(b.states.len());
        let s = solve_compact_into_nthreads(&mut b, 30000, Some(PROB_BASE), &mut sink, 4);
        assert!(s.converged, "must converge (θ-mask + band, parallel)");
        let mut mism = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH {
                if a.states[i].total_cost != b.states[i].total_cost {
                    mism += 1;
                }
                assert_eq!(
                    a.states[i].optimal_action, b.states[i].optimal_action,
                    "policy mismatch @ state {i}"
                );
            }
        }
        assert_eq!(mism, 0, "θ-mask sparse eval must stay bit-exact under band scan");
        assert!(s.freed_blocks > 0, "退避が起きるべき（多波・小バンド）");
    }

    /// 遅延確保＋退避が peak RAM（常駐ブロック）を band に抑え、かつ出力経由で bit-exact。
    #[test]
    fn lazy_alloc_and_eviction_bound_peak_and_exact() {
        use crate::params::PROB_BASE;
        use crate::solvers::test_support::{make_vi, run_reference_to_fixed_point, REACH};
        // 縦長マップ＋小バンドで、band がブロックの一部だけを覆い peak < total を顕在化させる。
        let (w, h) = (8, 128);
        let occ = vec![0i8; (w * h) as usize];
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);

        let stats = solve_compact(&mut b, 20000, Some(PROB_BASE));
        assert!(stats.converged, "must converge");

        let mut mism = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH && a.states[i].total_cost != b.states[i].total_cost {
                mism += 1;
            }
        }
        assert_eq!(mism, 0, "bit-exact mismatch after lazy alloc + eviction");
        assert!(stats.freed_blocks > 0, "ブロック退避が起きるべき");
        // 遅延確保＋退避で常駐ブロックのピークが総ブロックを下回る（= peak RAM 削減）。
        assert!(
            stats.peak_resident_blocks < stats.total_blocks,
            "peak 常駐ブロックが総ブロックを下回るべき (peak={}, total={})",
            stats.peak_resident_blocks, stats.total_blocks
        );
    }

    /// 回帰: フル解像度 tsukuba 規模（nx=13250, ny=7100, nt=60, nstates=5.6e9 > i32::MAX）で orig
    /// 索引が i32 オーバーフローしないこと。pve1ubuntu のフル解像度ランが旧 i32 演算で `iy·nt·nx` が
    /// 負にラップして MmapSink スライス範囲外 panic した実バグの回帰ガード。
    #[test]
    fn orig_index_no_i32_overflow_large_map() {
        let (nx, nt) = (13250i32, 60i32);
        let iy = 7099i32; // 最終行付近
        let got = orig_index(0, iy, 0, nx, nt);
        let want = iy as usize * nx as usize * nt as usize; // 5,643,705,000
        assert_eq!(got, want);
        assert!(got > i32::MAX as usize, "index は i32::MAX (={}) を超えるはず", i32::MAX);
        // it/ix 成分も含めた一般形。
        assert_eq!(
            orig_index(13249, 7099, 59, nx, nt),
            59usize + 13249 * 60 + 7099 * 13250 * 60
        );
    }

    /// MapSource（2D free/penalty + ゴール局所 final）が materialized states と per-cell で bit-exact
    /// 一致することを検証（compact のメモリ床撤去の健全性の土台）。pen/free/is_final/max_pen/seed/free
    /// 集合が全 (ix,iy,it) で SliceSource と一致する。
    #[test]
    fn mapsource_matches_materialized_states() {
        use crate::msg::OccupancyGrid;
        use crate::solvers::test_support::actions;
        use crate::value_iterator::ValueIterator;

        // 障害物入りマップ（penalty margin / free を非自明にする）。
        let (w, h) = (10i32, 8i32);
        let mut data = vec![0i8; (w * h) as usize];
        data[(2 * w + 3) as usize] = 100;
        data[(5 * w + 6) as usize] = 100;
        let map = OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data,
        };
        let (sr, srp, gmr, gmt) = (0.2, 30.0, 0.3, 15);

        let mut vi = ValueIterator::new(actions(), 1);
        vi.set_map_with_occupancy_grid(&map, 60, sr, srp, gmr, gmt);
        vi.set_goal(0.10, 0.10, 0);

        let slice = SliceSource::new(&vi.states, vi.cell_num_x, vi.cell_num_t);
        let mapsrc = MapSource::build(&map, &vi, sr, srp);
        let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);

        for iy in 0..ny {
            for ix in 0..nx {
                assert_eq!(mapsrc.free(ix, iy), slice.free(ix, iy), "free @ ({ix},{iy})");
                assert_eq!(mapsrc.pen(ix, iy), slice.pen(ix, iy), "pen @ ({ix},{iy})");
                for it in 0..nt {
                    assert_eq!(
                        mapsrc.is_final(ix, iy, it),
                        slice.is_final(ix, iy, it),
                        "is_final @ ({ix},{iy},{it})"
                    );
                }
            }
        }
        assert_eq!(mapsrc.max_pen(), slice.max_pen(), "max_pen");

        let mut sset: HashSet<(i32, i32, i32)> = HashSet::new();
        slice.for_each_seed(&mut |ix, iy, it| {
            sset.insert((ix, iy, it));
        });
        let mut mset: HashSet<(i32, i32, i32)> = HashSet::new();
        mapsrc.for_each_seed(&mut |ix, iy, it| {
            mset.insert((ix, iy, it));
        });
        assert_eq!(mset, sset, "seed set");
        assert!(!sset.is_empty(), "goal final セルが存在するはず");

        let mut sf: HashSet<(i32, i32)> = HashSet::new();
        slice.for_each_free(&mut |ix, iy| {
            sf.insert((ix, iy));
        });
        let mut mf: HashSet<(i32, i32)> = HashSet::new();
        mapsrc.for_each_free(&mut |ix, iy| {
            mf.insert((ix, iy));
        });
        assert_eq!(mf, sf, "free col set");
    }

    /// `set_map_geometry_no_states` が geometry/transitions を整えつつ states/sweep_orders を一切
    /// 確保しないこと（メモリ床撤去の核）。
    #[test]
    fn geometry_no_states_leaves_states_empty() {
        use crate::msg::OccupancyGrid;
        use crate::value_iterator::ValueIterator;
        let map = OccupancyGrid {
            width: 6,
            height: 5,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: vec![0i8; 30],
        };
        let mut vi = ValueIterator::new(crate::solvers::test_support::actions(), 1);
        vi.set_map_geometry_no_states(&map, 60, 0.3, 15);
        assert!(vi.states.is_empty(), "states を確保してはいけない");
        assert!(vi.sweep_orders.is_empty(), "sweep_orders を確保してはいけない");
        assert_eq!(vi.cell_num_x, 6);
        assert_eq!(vi.cell_num_y, 5);
        assert_eq!(vi.cell_num_t, 60);
        // 遷移テーブルは構築済み（Geom::build / displacement に必要）。
        assert!(vi.actions.iter().all(|a| !a.state_transitions.is_empty()));
    }

    /// mapped 経路（states を作らない）が Reference 固定点と到達可能セルで bit-exact（value+policy）。
    /// 出力は sink から読む（write_back しない）。nthreads を引数で固定（1=直列 / 4=並列 async）。
    fn assert_mapped_parity(w: i32, h: i32, occ: Vec<i8>, nthreads: usize) {
        use crate::msg::OccupancyGrid;
        use crate::solvers::test_support::{actions, make_vi, run_reference_to_fixed_point, REACH};

        let mut a = make_vi(w, h, occ.clone());
        run_reference_to_fixed_point(&mut a);

        let map = OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: occ,
        };
        let mut sink = RamSink::new((w * h * 60) as usize);
        // make_vi と同一パラメータ: theta=60, sr=0.2, srp=30.0, gmr=0.3, gmt=15, goal(0.10,0.10,0)。
        let s = solve_compact_mapped(
            actions(), 1, &map, 60, 0.2, 30.0, 0.3, 15, 0.10, 0.10, 0, 4000, None, &mut sink,
            nthreads, &AtomicBool::new(false),
        );
        assert!(s.converged, "mapped solver must converge (nthreads={nthreads})");

        let mut n_reach = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH {
                n_reach += 1;
                let (v, act) = sink.read(i);
                assert_eq!(
                    v, a.states[i].total_cost,
                    "value mismatch @ state {i} (ix={},iy={},it={}, nthreads={nthreads})",
                    a.states[i].ix, a.states[i].iy, a.states[i].it
                );
                let act_opt = if act < 0 { None } else { Some(act as usize) };
                assert_eq!(
                    act_opt, a.states[i].optimal_action,
                    "policy mismatch @ state {i} (ix={},iy={},it={}, nthreads={nthreads})",
                    a.states[i].ix, a.states[i].iy, a.states[i].it
                );
            }
        }
        assert!(n_reach > 0, "到達可能セルが存在するはず");
    }

    /// 標準3マップで mapped == Reference（直列）。
    #[test]
    fn parity_mapped_standard_maps_nthreads1() {
        assert_mapped_parity(8, 8, vec![0i8; 64], 1);
        let mut occ = vec![0i8; 64];
        for iy in 0..8 {
            occ[(iy * 8 + 5) as usize] = 100;
        }
        occ[5] = 0;
        assert_mapped_parity(8, 8, occ, 1);
        let mut occ = vec![0i8; 64];
        occ[8 + 2] = 100;
        occ[3 * 8 + 2] = 100;
        occ[2 * 8 + 1] = 100;
        assert_mapped_parity(8, 8, occ, 1);
    }

    /// 標準3マップで mapped == Reference（並列 async G-S, nthreads=4）。
    #[test]
    fn parity_mapped_standard_maps_nthreads4() {
        assert_mapped_parity(8, 8, vec![0i8; 64], 4);
        let mut occ = vec![0i8; 64];
        for iy in 0..8 {
            occ[(iy * 8 + 5) as usize] = 100;
        }
        occ[5] = 0;
        assert_mapped_parity(8, 8, occ, 4);
    }

    /// 複数行ブロック × 並列 async（nthreads=4）で mapped == Reference（チャンク境界・退避を刺激）。
    #[test]
    fn parity_mapped_larger_nthreads4() {
        let (w, h) = (32, 24);
        assert_mapped_parity(w, h, vec![0i8; (w * h) as usize], 4);
    }
}
