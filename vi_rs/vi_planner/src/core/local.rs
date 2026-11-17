//! Local field access, scan updates, refinement, and control decisions.
use super::*;

impl PlannerCore {
    // ──────────────────────────────────────────────────────────────────────
    // 狭域が読み書きする密な局所 VI
    //
    // 密経路は全域の ValueIteratorLocal そのもの、compact 経路はハイドレート済み
    // パッチ。追従側 (observe_scan / refine_* / decide / window_value_grid) は
    // この 2 つを区別しない。
    // ──────────────────────────────────────────────────────────────────────

    pub(super) fn local(&self) -> Option<&ValueIteratorLocal> {
        match &self.cached.as_ref()?.field {
            Field::Dense(vi) => Some(vi),
            Field::Compact(_) => {
                let p = self.patch.as_ref()?;
                p.at.map(|_| &p.vi)
            }
        }
    }

    pub(super) fn local_mut(&mut self) -> Option<&mut ValueIteratorLocal> {
        let compact = matches!(self.cached.as_ref()?.field, Field::Compact(_));
        if compact {
            let p = self.patch.as_mut()?;
            p.at?;
            return Some(&mut p.vi);
        }
        match &mut self.cached.as_mut()?.field {
            Field::Dense(vi) => Some(vi),
            Field::Compact(_) => None,
        }
    }

    /// ローカルウィンドウをロボット位置中心へ移動 (本家 `setLocalWindow`)。
    /// compact 経路では、必要ならその前にパッチを置き直して compact の場から
    /// 起こし直す。前の位置での成果は `commit_window` が毎 tick sink へ返して
    /// あるので、置き直しで失われるものは無い。
    pub fn set_window(&mut self, pose: PoseView) {
        let old_origin = self.patch.as_ref().and_then(|p| p.at);
        // build / cached / patch / penalty は別フィールドなので分割して借りられる。
        let build = &self.build;
        let patch = &mut self.patch;
        let overlay = self.penalty.as_ref();
        let Some(c) = self.cached.as_mut() else { return };
        match &mut c.field {
            Field::Dense(vi) => vi.set_local_window(pose.x, pose.y, pose.yaw_rad),
            Field::Compact(f) => {
                let Some(p) = patch.as_mut() else { return };
                let res = f.resolution;
                let gx = ((pose.x - f.origin.0) / res).floor() as i32;
                let gy = ((pose.y - f.origin.1) / res).floor() as i32;
                if p.needs_recenter(gx, gy) {
                    p.hydrate(f, build, overlay, (gx - p.half, gy - p.half));
                }
                p.vi.set_local_window(pose.x, pose.y, pose.yaw_rad);
            }
        }
        if self.patch.as_ref().and_then(|p| p.at) != old_origin {
            self.invalidate_local_window();
        }
        self.update_local_ownership();
    }

    /// スキャンのヒット点周辺に local_penalty を注入 (本家 `setLocalCost`)。
    /// `set_window` 後に呼ぶこと (ウィンドウ外のヒットは無視される)。
    pub fn observe_scan(&mut self, scan: &LaserScan, pose: PoseView) {
        self.observe_scan_gated(scan, pose, 0);
    }

    /// 品質ゲート付きの [`Self::observe_scan`]: 注入 penalty を `scan_penalty >> shift` に
    /// 減衰する。自己位置のフィットが怪しい tick は localizer 自身がそれを
    /// `quality()` で知っている — そのスキャンの投影は地図とずれており、満額で
    /// 塗るとゴースト壁がロボットを global 勾配との間に挟んで止める。shift は
    /// [`quality_shift`] で quality から量子化する。0 は `observe_scan` と同一。
    pub fn observe_scan_gated(&mut self, scan: &LaserScan, pose: PoseView, shift: u32) {
        let map_clear = self.cfg.map_clear_from_scan;
        let radius = self.cfg.scan_penalty_radius;
        let penalty = self.cfg.scan_penalty;
        let before = self.local_inputs();
        if let Some(vi) = self.local_mut() {
            vi.clear_map_from_scan = map_clear;
            vi.scan_penalty_radius = radius;
            vi.scan_penalty = penalty;
            vi.set_local_cost_attenuated(scan, pose.x, pose.y, pose.yaw_rad, shift);
        }
        self.harvest_penalties();
        self.invalidate_local_inputs(before);
    }

    /// 自己位置の広がり `sigma_m` [m] に応じて壁際のマージンを広げる
    /// (上田ら 2023 4·2·2 のマージン膨張 — [`PlanConfig::sigma_margin_gain`])。
    /// `set_window` の後、`refine_for` の前に 1 tick 1 回呼ぶ。
    ///
    /// 膨張量はウィンドウの**最小**半軸でクランプする。窓が丸ごと帯になると
    /// ロボットが出口を失い、`decide` が NoAction を返し続けて停止するため。
    /// 異方・変形する形状では一番細い方向が先に埋まるので、そこで測る。
    pub fn inflate_by_sigma(&mut self, sigma_m: f64) {
        let gain = self.cfg.sigma_margin_gain;
        if gain <= 0.0 || sigma_m <= 0.0 {
            return;
        }
        let extra = (sigma_m * gain).min(self.build.local_shape.min_radius_m());
        let before = self.local_inputs();
        if let Some(vi) = self.local_mut() {
            vi.inflate_by_sigma(extra);
        }
        self.harvest_penalties();
        self.invalidate_local_inputs(before);
    }

    /// ローカルウィンドウ内の価値反復を `budget` の範囲で回す (本家
    /// `localValueIterationWorker` の常駐スレッドを制御周期内の時間予算に
    /// 置き換えたもの)。1 パスの Δ 合計が 0 になったら予算を残して早期リターン
    /// する。戻り値は最後のパスの Δ 合計。
    pub fn refine_for(&mut self, budget: Duration) -> u64 {
        let t0 = Instant::now();
        while !self.advance_invalidations(512) {
            if t0.elapsed() >= budget {
                return 0;
            }
        }
        loop {
            let (pass_delta, stopped) = self.refine_pass_until(|| t0.elapsed() >= budget);
            if pass_delta > 0 {
                self.global.certified = false;
            }
            // 狭域が共有場を動かした。全域掃きに伝播させる仕事があると印を付ける。
            // compact は窓を書き戻して初めて共有場が動くので、印は `commit_window`
            // が (書けた範囲を待ち行列へ入れるのと同時に) 立てる。ここで立てると、
            // 書き戻す先が無い Δ でも掃きスレッドを起こしてしまう。
            if pass_delta > 0 && !self.cfg.use_compact() {
                self.dirty = true;
                // ponytail: `global_sweep: false` だと掃きスレッドが立たないので、
                // ここで積んだ種は誰も引き取らない。地図 1 枚ぶんのビットボード
                // (19F scale 2 で 16 KB) が飽和するだけの有界な空振りなので、
                // `dirty` を立てるほうと揃えてそのままにしてある。気になるなら
                // 両方まとめて `cfg.global_sweep` で塞ぐこと。
                self.seed_sweep_window();
            }
            if stopped || pass_delta == 0 {
                // compact 経路は窓を sink へ返して初めて共有場になる。予算切れでも
                // 返す (次の tick で続きから精密化する)。密経路では何もしない。
                self.commit_window();
                if !stopped && pass_delta == 0 {
                    self.certify_local();
                }
                return pass_delta;
            }
        }
    }

    /// ウィンドウ全体を `n` パス回す (決定的テスト用)。
    #[cfg(test)]
    pub fn refine_passes(&mut self, n: usize) {
        while !self.advance_invalidations(512) {}
        for _ in 0..n {
            let (delta, stopped) = self.refine_pass_until(|| false);
            if delta > 0 {
                self.global.certified = false;
            }
            if delta > 0 && !self.cfg.use_compact() {
                self.dirty = true;
                self.seed_sweep_window();
            }
            if !stopped && delta == 0 {
                self.certify_local();
            }
        }
        self.commit_window();
    }

    /// ウィンドウ 1 パス。`should_stop` は x 列ごとに観測し、途中打ち切り時は
    /// `(それまでの Δ 合計, true)` を返す。
    pub(super) fn refine_pass_until(&mut self, should_stop: impl Fn() -> bool) -> (u64, bool) {
        let Some(vi) = self.local_mut() else {
            return (0, true);
        };
        let nt = vi.base.cell_num_t;
        let reach = transition_reach(&vi.base);
        let mut boundary = Vec::new();
        for y in
            (vi.local_iy_min - reach).max(0)..=(vi.local_iy_max + reach).min(vi.base.cell_num_y - 1)
        {
            for x in
                (vi.local_ix_min - reach).max(0)..=(vi.local_ix_max + reach).min(vi.base.cell_num_x - 1)
            {
                let outside = !vi.in_local_area(x, y);
                for t in 0..nt {
                    let idx = vi.base.to_index(x, y, t) as usize;
                    let s = &mut vi.base.states[idx];
                    if s.policy_invalidated() {
                        if outside {
                            boundary.push((idx, s.total_cost));
                        }
                        s.total_cost = MAX_COST;
                    }
                }
            }
        }
        let mut delta = 0u64;
        let mut stopped = false;
        for iix in vi.local_ix_min..=vi.local_ix_max {
            if should_stop() {
                stopped = true;
                break;
            }
            for iiy in vi.local_iy_min..=vi.local_iy_max {
                // 窓の形は `in_local_area` 一点で決まる (スキャン注入と同じ判定)。
                if !vi.in_local_area(iix, iiy) {
                    continue;
                }
                for iit in 0..nt {
                    let i = vi.base.to_index(iix, iiy, iit) as usize;
                    delta = delta.saturating_add(vi.value_iteration_local(i));
                }
            }
        }
        // The halo participates in this calculation, but only window states are
        // published. Far-field values are committed by the global worker.
        for (idx, value) in boundary {
            vi.base.states[idx].total_cost = value;
        }
        (delta, stopped)
    }

    /// 現在姿勢の判断 (読み取り専用)。実体は [`PlanConfig::follow_controller`] で
    /// 選んだ [`FollowController`] — 既定の greedy は本家 `ViNode::decision` の
    /// `posToAction` 相当 (方策が無ければ近傍借用)、dwa は連続行動。
    /// compact 経路では `set_window` でパッチを起こしてから呼ぶこと。
    pub fn decide(&self, pose: PoseView) -> Decision {
        let Some(local) = self.local() else { return Decision::NoAction };
        let (x, y, t) = pose_to_cell(&local.base, pose.x, pose.y, pose.yaw_rad);
        if is_final(&local.base, x, y, t) {
            return Decision::Goal;
        }
        if self.global.invalidating() {
            return Decision::NoAction;
        }
        self.follow.decide(&local.base, &self.cfg, pose)
    }

    /// belief 仮説集合での判断 (QMDP — [`qmdp_decide`])。読む場は [`Self::decide`]
    /// と同じ密な局所場 (compact 経路では `set_window` 済みパッチ — パッチ外の
    /// 仮説は評価対象外)。仮説が割れていても「どの仮説でも悪くない行動」を選び、
    /// 有意な仮説が衝突と言う行動しか無ければ `NoAction` (停止)。
    /// `follow_controller` (dwa/mppi) は経由しない — 多峰時は離散 QMDP が優先で、
    /// 単峰時は呼び出し側が [`Self::decide`] へフォールバックする分担。
    pub fn decide_qmdp(&self, hyps: &[(PoseView, f64)]) -> Decision {
        let Some(local) = self.local() else { return Decision::NoAction };
        match qmdp_decide(&local.base, hyps) {
            QmdpDecision::Goal => Decision::Goal,
            QmdpDecision::Action(ai) => {
                if self.global.invalidating() {
                    return Decision::NoAction;
                }
                let a = &local.base.actions[ai];
                Decision::Action { id: Some(a.id as usize), fw: a.delta_fw, rot_deg: a.delta_rot }
            }
            QmdpDecision::NoAction => Decision::NoAction,
        }
    }}
