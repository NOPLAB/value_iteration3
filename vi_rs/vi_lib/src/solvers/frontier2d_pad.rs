//! パディング + オフセット事前計算の frontier2d 共有モデル (`Padded`)。
//! ソルバ本体は `frontier2d_par` / `frontier2d_par_unsafe` / `frontier2d_fused` が
//! このモデルの上に載る (単独の直列 pad ソルバは削除済み)。
//!
//! - **パディング**: グリッドを最大変位 (mx,my) 分だけ非 free セルで囲う。範囲外アクセスは
//!   パディング (free=false) を読んで `!free → MAX_COST` となり、本家の「範囲外で即 MAX_COST」と
//!   返り値・短絡位置まで一致する (bit-exact)。境界分岐が不要になる。
//! - **オフセット事前計算**: 絶対 θ + 列レイアウトより、隣接フラット index =
//!   `col_base + (dix·nt + diy·nt·nx_pad + nit)`。括弧内は `(action, source θ)` ごとに定数なので
//!   `(offset, prob)` を事前計算し、inner loop から乗算/剰余を消す。
//!
//! コスト数式・演算順序・短絡は `action_cost_raw` と一致。収束値・方策は本家と bit-exact。
//! パディングモデル (`Padded`) は B1 並列版 (`frontier2d_par`) でも共有する (bit-exact 演算の単一ソース)。

use std::sync::atomic::{AtomicU64, Ordering};

use crate::params::{MAX_COST, PROB_BASE_BIT};
use crate::value_iterator::ValueIterator;

use super::displacement;

/// `hot` 配列の読み出し抽象。直列/Jacobi 版は素の `[[u64; 2]]` スライス、非同期版
/// (`frontier2d_par_unsafe`) は Relaxed atomic ビューを渡す。Relaxed load は
/// x86-64/aarch64 で素の load と同一命令なので、monomorphize 後の直列版コードは
/// 従来と一致する (コスト数式の単一ソースを generic 化だけで保つ)。
pub(crate) trait HotCells {
    fn get(&self, i: usize) -> [u64; 2];
}

impl HotCells for [[u64; 2]] {
    #[inline(always)]
    fn get(&self, i: usize) -> [u64; 2] {
        self[i]
    }
}

impl HotCells for [[AtomicU64; 2]] {
    #[inline(always)]
    fn get(&self, i: usize) -> [u64; 2] {
        let c = &self[i];
        [c[0].load(Ordering::Relaxed), c[1].load(Ordering::Relaxed)]
    }
}

/// frontier2d 用パディング SoA モデル。`hot=[total_cost, penalty +ʷ local_penalty]`、
/// `free`/`finals` は境界が false。`precomp[a][it]` は隣接の `(相対オフセット, prob)`。
pub(crate) struct Padded {
    pub hot: Vec<[u64; 2]>,
    pub free: Vec<bool>,
    pub finals: Vec<bool>,
    pub precomp: Vec<Vec<Vec<(i64, u64)>>>,
    pub nx: i32,
    pub ny: i32,
    pub nt: i32,
    pub mx: i32,
    pub my: i32,
    pub nx_pad: i32,
    pub row_stride: i64,
}

impl Padded {
    pub(crate) fn build(vi: &ValueIterator) -> Self {
        let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
        let (mx, my, _mt) = displacement(vi);
        let (nx_pad, ny_pad) = (nx + 2 * mx, ny + 2 * my);
        let row_stride = (nt * nx_pad) as i64;
        let n_pad = (nx_pad * ny_pad * nt) as usize;

        let mut hot: Vec<[u64; 2]> = vec![[MAX_COST, 0]; n_pad];
        let mut free: Vec<bool> = vec![false; n_pad];
        let mut finals: Vec<bool> = vec![false; n_pad];
        for s in &vi.states {
            let idx = (s.it + (s.ix + mx) * nt + (s.iy + my) * (nt * nx_pad)) as usize;
            hot[idx] = [s.total_cost, s.penalty.wrapping_add(s.local_penalty)];
            free[idx] = s.free;
            finals[idx] = s.final_state;
        }

        let precomp = build_precomp(vi, nt, row_stride);

        Self { hot, free, finals, precomp, nx, ny, nt, mx, my, nx_pad, row_stride }
    }

    /// セル (ix,iy) の θ=0 列ベース (パディング座標フラット)。
    #[inline]
    pub(crate) fn pad_col(&self, ix: i32, iy: i32) -> i64 {
        (ix + self.mx) as i64 * self.nt as i64 + (iy + self.my) as i64 * self.row_stride
    }

    /// hot を vi.states へ書き戻す。`opt` が Some なら optimal_action も。
    pub(crate) fn write_back(&self, vi: &mut ValueIterator, opt: Option<&[Option<usize>]>) {
        self.write_back_hot(vi, self.hot.as_slice(), opt)
    }

    /// `write_back` の hot 分離版。`frontier2d_par_unsafe` は hot を atomic ビューとして
    /// 取り出したまま (self.hot は空) 境界プローブを具現化するため、hot を外から渡す。
    pub(crate) fn write_back_hot<H: HotCells + ?Sized>(
        &self,
        vi: &mut ValueIterator,
        hot: &H,
        opt: Option<&[Option<usize>]>,
    ) {
        let (nx, nt, mx, my, nx_pad) = (self.nx, self.nt, self.mx, self.my, self.nx_pad);
        for s in vi.states.iter_mut() {
            let pad_idx = (s.it + (s.ix + mx) * nt + (s.iy + my) * (nt * nx_pad)) as usize;
            s.total_cost = hot.get(pad_idx)[0];
            if let Some(opt) = opt {
                let orig = (s.it + s.ix * nt + s.iy * (nt * nx)) as usize;
                s.optimal_action = opt[orig];
            }
        }
    }
}

/// 隣接フラット = col_base + dix·nt + diy·(nt·nx_pad) + nit、nit=(dit+nt)%nt。
/// `(action, source θ)` ごとの `(相対オフセット, prob)` テーブル (`Padded`/`Geom` 共有)。
pub(crate) fn build_precomp(
    vi: &ValueIterator,
    nt: i32,
    row_stride: i64,
) -> Vec<Vec<Vec<(i64, u64)>>> {
    vi.actions
        .iter()
        .map(|a| {
            (0..nt as usize)
                .map(|it| {
                    a.state_transitions[it]
                        .iter()
                        .map(|tr| {
                            let nit = (tr.dit + nt) % nt;
                            let off =
                                tr.dix as i64 * nt as i64 + tr.diy as i64 * row_stride + nit as i64;
                            (off, tr.prob as u64)
                        })
                        .collect()
                })
                .collect()
        })
        .collect()
}

/// 本家 `actionCost` のパディング + 事前計算オフセット版。`col_base` はソースセルの θ=0 列ベース。
/// `buckets` は `(隣接フラット相対オフセット, prob)`。`hot[n]=[total_cost, pen]`。
///
/// 本家との意図的な差分が 1 つ: 隣接が**未確定 (初期値 MAX_COST のまま)** なら折返し計算せず
/// MAX_COST を返す。本家は (MAX_COST+pen)·prob の u64 折返しゴミを Q として返し、毎 sweep の
/// 無条件代入で後から自己修正するが、frontier 系の「減少時のみ書く」更新ではその偽低 Q を
/// ラッチして下流全体を汚染する (実マップで観測)。このクランプにより hot には実数値しか
/// 書かれず (帰納)、固定点では到達可能セルの argmin 連鎖は実数値のみなので収束値は本家と一致。
#[inline]
pub(crate) fn action_cost_pad<H: HotCells + ?Sized>(
    hot: &H,
    free: &[bool],
    buckets: &[(i64, u64)],
    col_base: i64,
) -> u64 {
    let mut cost: u64 = 0;
    for &(off, prob) in buckets {
        let n = (col_base + off) as usize; // パディングにより常に有効域
        if !free[n] {
            return MAX_COST;
        }
        let h = hot.get(n);
        if h[0] == MAX_COST {
            return MAX_COST; // 未確定隣接: 折返しゴミ Q を作らない
        }
        cost = cost.wrapping_add(h[0].wrapping_add(h[1]).wrapping_mul(prob));
    }
    cost >> PROB_BASE_BIT
}
