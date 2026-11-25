//! Shared-field propagation entry points and dense frontier seeding.
use super::*;

impl PlannerCore {
    /// 狭域が共有場を動かしてから、まだ全域へ伝播させ切っていないか。
    pub fn is_dirty(&self) -> bool {
        self.dirty
            || self.global.in_flight.is_some()
            || self.global.local_in_flight.is_some()
            || self.global.local_pending()
            || !self.global.deferred.is_empty()
    }

    /// 直近の伝播 1 回で処理した量と、その単位。ログ用。
    ///
    /// どちらの経路でも「1 回の伝播」の仕事量は**地図の大きさで決まらず、変化が
    /// 及んだ範囲で決まる**。経過時間だけ出しても速いのか仕事が少なかったのか
    /// 読めないので、ログにはこれを添えること。
    pub fn sweep_work(&self) -> (usize, &'static str) {
        match self.repair.as_ref() {
            Some(r) => (r.last_visits, "tiles"),
            None => (self.sweep.last_visits, "cells"),
        }
    }

    /// 進行中の伝播の進み具合 `(これまでの処理数, 残り)`。伝播していないときは None。
    ///
    /// **走行中は待ち行列がまず空にならない**。壁が窓 (±1m) に入っていれば
    /// `set_local_cost` が毎 tick penalty を塗り直すので次の伝播が積まれる
    /// (実測: 壁が窓の中にある通路を 100 秒走って、待ち行列が空だったのは
    /// 1000 tick 中 13 回だけ)。つまり「1 回終わった」ログはほぼ出ない —
    /// 掃きが動いているかはこちらで見ること。
    pub fn sweep_progress(&self) -> Option<(usize, usize)> {
        let outstanding = usize::from(self.global.in_flight.is_some())
            + usize::from(self.global.local_in_flight.is_some())
            + self.global.deferred.len();
        if let Some(r) = self.repair.as_ref() {
            let queued = r.queue.len() + outstanding;
            return (queued > 0).then_some((r.visits, queued));
        }
        let sw = &self.sweep;
        let queued = sw.cands.len() - sw.pos + sw.pending_len() + outstanding;
        (queued > 0).then_some((sw.visits, queued))
    }

    /// Synchronous wrapper; threaded callers run the job outside the core mutex.
    pub fn sweep_global(&mut self, max_states: usize) -> (u64, bool) {
        let Some(mut job) = self.prepare_global_job(max_states) else {
            return (0, !self.is_dirty());
        };
        job.run();
        self.commit_global_job(job)
    }

    /// 密経路: ローカルウィンドウを全域伝播の種に積む。狭域が共有場を書き換えた
    /// とき (`refine_for` で Δ>0) に呼ぶ。
    ///
    /// 積むのは動いたセルではなく**窓の矩形まるごと**。膨張がその外周 ±reach を
    /// 拾うので、「窓の縁のセルの penalty が変わったせいで値が動く、窓の外の
    /// 前駆セル」も取りこぼさない (窓の中だけを積むと、その前駆が種から外れる)。
    pub(super) fn seed_sweep_window(&mut self) {
        let sw = &mut self.sweep;
        let Some(c) = self.cached.as_ref() else { return };
        let Field::Dense(vi) = &c.field else { return };
        let (nx, ny) = (vi.base.cell_num_x, vi.base.cell_num_y);
        let rect = (vi.local_ix_min, vi.local_ix_max, vi.local_iy_min, vi.local_iy_max);
        sw.seed_rect(nx, ny, rect);
    }

    /// 密経路: 地図を丸ごと種に積む。早期走り出し ([`PlanConfig::early_start`]) で
    /// 未確定のまま走り出した場を背景で解き切るための起点で、そこでしか呼ばない
    /// (compact 側の [`compact::Repair::enqueue_all`] と同じ役割)。
    ///
    /// 通常の伝播は「値が動いた範囲」から広げるが、解き終わっていない場では
    /// 未確定域が地図のどこにでもあるので起点が要る。確定済みのセルは 1 ラウンドで
    /// Δ=0 になって抜けるだけなので、余分な仕事は地図 1 周ぶんに収まる。
    pub(super) fn seed_sweep_all(&mut self) {
        let sw = &mut self.sweep;
        let Some(c) = self.cached.as_ref() else { return };
        let Field::Dense(vi) = &c.field else { return };
        let (nx, ny) = (vi.base.cell_num_x, vi.base.cell_num_y);
        sw.seed_rect(nx, ny, (0, nx - 1, 0, ny - 1));
    }

}
