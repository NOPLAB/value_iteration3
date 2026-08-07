//! solve の進行を外から観測・制御する共通フック (制御反転)。
//! 設計: `docs/superpowers/specs/2026-08-08-vi-solver-observe-design.md`
//!
//! 各ソルバは自分の**自然な境界** (フロンティアラウンド / 全域スイープ / ブロックパス /
//! バンド finalize / 一定 pop 数) ごとに [`SolveObserver::boundary`] を呼ぶ。呼び出し側は
//! そこで cancel (プリエンプト)・途中経過の観測・早期打ち切り (early_start) を行う。
//! これは vi_planner の手書きチャンクループ・compact の `stop` クロージャ・sparse の
//! Snapshotter が別々にやっていたことの一般化で、チャンク再入 (毎チャンクの再ビルド +
//! 全セル write_back) を不要にする。
//!
//! observer は**常に呼び出しスレッドで呼ばれる** (並列ソルバはリーダーを spawn せず
//! 呼び出しスレッドでインライン実行する) ので、`Send` 境界は不要。

use crate::planner::PolicyView;
use crate::value_iterator::ValueIterator;

/// 境界で呼び出し側が返す指示。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolveFlow {
    /// 続行。
    Continue,
    /// ここで打ち切る (early_start)。場は観測可能な整合状態のまま戻る
    /// (`SolveOutcome::stopped`)。
    Stop,
    /// プリエンプト。最短経路で戻る。場の内容は保証しない (`SolveOutcome::cancelled`)。
    Cancel,
}

/// 境界での場の観測窓。**`policy()` を呼んだときだけ**内部表現からの具現化
/// (write_back + argmin) が走る — 呼ばなければ無料。観測は非破壊で、solve は
/// そのまま続行できる。
pub trait SolveProbe {
    /// 進捗 (ソルバの反復単位: ラウンド / スイープ / 確定セル数など。単調非減少)。
    fn iters(&self) -> u32;
    /// 進捗 (更新セル数。単調非減少、ソルバによっては概算)。
    fn updates(&self) -> u64;
    /// 現在の場の読み取りビュー。in-place 系は `&ValueIterator` そのもの、
    /// 内部表現を持つ系は要求時 write_back 済みの `&ValueIterator`、
    /// compact は sink ビュー ([`crate::planner::CompactPolicy`])。
    fn policy(&mut self) -> &dyn PolicyView;
}

/// solve の進行を外から制御するフック。
pub trait SolveObserver {
    /// 境界呼び出しの間隔ヒント (ソルバの反復単位)。ラウンド粒度のソルバは
    /// `interval` ラウンドごと、priority 系は文書化された pop 数 × これ、に相当する
    /// 頻度で `boundary` を呼ぶ (best effort — ソルバの粒度より細かくはならない)。
    fn interval(&self) -> u32 {
        1
    }
    /// 境界。`probe` から現時点の場を (必要なときだけ) 観測できる。
    fn boundary(&mut self, probe: &mut dyn SolveProbe) -> SolveFlow;
}

/// 何もしない observer。`solve()` (従来 API) はこれを渡す。
pub struct NullObserver;

impl SolveObserver for NullObserver {
    fn interval(&self) -> u32 {
        u32::MAX
    }
    fn boundary(&mut self, _probe: &mut dyn SolveProbe) -> SolveFlow {
        SolveFlow::Continue
    }
}

/// `solve_observed` の結果。従来の `(iters, updates, converged)` に境界フックの
/// 終端状態を加えたもの。
#[derive(Clone, Copy, Debug)]
pub struct SolveOutcome {
    pub iters: u32,
    pub updates: u64,
    /// 固定点に到達したか。`stopped` / `cancelled` のときは false。
    pub converged: bool,
    /// [`SolveFlow::Stop`] で打ち切った。場は観測可能な整合状態のまま
    /// (dense: Bellman 上界 + その時点の argmin 方策 / compact: 確定列 = v*)。
    pub stopped: bool,
    /// [`SolveFlow::Cancel`] で中断した。場の内容は保証しない (破棄すること)。
    pub cancelled: bool,
}

impl SolveOutcome {
    pub(crate) fn running(iters: u32, updates: u64, converged: bool) -> Self {
        Self { iters, updates, converged, stopped: false, cancelled: false }
    }
    pub(crate) fn stopped(iters: u32, updates: u64) -> Self {
        Self { iters, updates, converged: false, stopped: true, cancelled: false }
    }
    pub(crate) fn cancelled(iters: u32, updates: u64) -> Self {
        Self { iters, updates, converged: false, stopped: false, cancelled: true }
    }
    /// 従来のタプル形 (iters, updates, converged)。
    pub fn tuple(&self) -> (u32, u64, bool) {
        (self.iters, self.updates, self.converged)
    }
}

/// `interval` 刻みの境界呼び出しを管理するカウンタ。ラウンド粒度のソルバが
/// 「前回の境界から interval 反復進んだか」を判定するのに使う。
pub(crate) struct BoundaryPacer {
    next: u64,
    interval: u64,
}

impl BoundaryPacer {
    pub(crate) fn new(obs: &dyn SolveObserver) -> Self {
        let interval = obs.interval().max(1) as u64;
        Self { next: interval, interval }
    }
    /// 反復カウンタ `iters` が次の境界に達したか。達したら次の境界を予約する。
    pub(crate) fn due(&mut self, iters: u64) -> bool {
        if iters >= self.next {
            self.next = iters.saturating_add(self.interval);
            true
        } else {
            false
        }
    }
}

/// 多フェーズソルバ (coarse→refine の coarse_theta、粗→細スケジュールの pyramid) が
/// 後段フェーズへ observer を渡すためのラッパ。後段の probe が報告する iters/updates に
/// 前フェーズ分を上乗せし、呼び出し側から見た進捗を単調非減少に保つ。
pub(crate) struct OffsetObserver<'a> {
    pub inner: &'a mut dyn SolveObserver,
    pub iters_offset: u32,
    pub updates_offset: u64,
}

struct OffsetProbe<'a, 'b> {
    inner: &'a mut (dyn SolveProbe + 'b),
    iters_offset: u32,
    updates_offset: u64,
}

impl SolveProbe for OffsetProbe<'_, '_> {
    fn iters(&self) -> u32 {
        self.inner.iters().saturating_add(self.iters_offset)
    }
    fn updates(&self) -> u64 {
        self.inner.updates().saturating_add(self.updates_offset)
    }
    fn policy(&mut self) -> &dyn PolicyView {
        self.inner.policy()
    }
}

impl SolveObserver for OffsetObserver<'_> {
    fn interval(&self) -> u32 {
        self.inner.interval()
    }
    fn boundary(&mut self, probe: &mut dyn SolveProbe) -> SolveFlow {
        let mut p = OffsetProbe {
            inner: probe,
            iters_offset: self.iters_offset,
            updates_offset: self.updates_offset,
        };
        self.inner.boundary(&mut p)
    }
}

/// in-place 系 (states を直接更新するソルバ) の境界プローブ。観測は無料。
pub(crate) struct InPlaceProbe<'a> {
    pub vi: &'a ValueIterator,
    pub iters: u32,
    pub updates: u64,
}

impl SolveProbe for InPlaceProbe<'_> {
    fn iters(&self) -> u32 {
        self.iters
    }
    fn updates(&self) -> u64 {
        self.updates
    }
    fn policy(&mut self) -> &dyn PolicyView {
        self.vi
    }
}

/// 内部表現を持つソルバ (SoA / Pad / Par / Fused / Sparse) の境界プローブ。
/// `policy()` の初回呼び出しでだけ `materialize` (内部表現 → `vi.states` への
/// write_back + argmin) を実行する。観測されなければ具現化コストは払わない。
pub(crate) struct MaterializeProbe<'a, 'b> {
    vi: &'a mut ValueIterator,
    materialize: &'b mut dyn FnMut(&mut ValueIterator),
    done: bool,
    pub iters: u32,
    pub updates: u64,
}

impl<'a, 'b> MaterializeProbe<'a, 'b> {
    pub(crate) fn new(
        vi: &'a mut ValueIterator,
        materialize: &'b mut dyn FnMut(&mut ValueIterator),
        iters: u32,
        updates: u64,
    ) -> Self {
        Self { vi, materialize, done: false, iters, updates }
    }
}

impl SolveProbe for MaterializeProbe<'_, '_> {
    fn iters(&self) -> u32 {
        self.iters
    }
    fn updates(&self) -> u64 {
        self.updates
    }
    fn policy(&mut self) -> &dyn PolicyView {
        if !self.done {
            (self.materialize)(self.vi);
            self.done = true;
        }
        &*self.vi
    }
}
