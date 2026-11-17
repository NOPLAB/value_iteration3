//! Dense solves and the multi-goal relocalization field.
use super::*;

impl PlannerCore {
    /// 密経路: `ValueIterator::states` を確保し、[`solve_observed`] + [`SolveDirector`]
    /// で解く。cancel の観測・途中経過・早期走り出し (`from` があるとき) はすべて
    /// solve 内部の境界 (`solve_chunk` 反復ごと) で行われる — 旧実装のチャンク再入
    /// (毎チャンクの再ビルド + 全セル write_back) はもう無い。境界で場を読めることは
    /// ソルバ側の契約 (`SolveProbe::policy`) で、全 `U64Solver` が conformance テストで
    /// これをゲートされている。
    pub(super) fn solve_dense(
        &self,
        goal: &PoseView,
        goal_t_deg: i32,
        from: Option<PoseView>,
        stats: &mut SolveStats,
        cancel: &AtomicBool,
        on_chunk: &mut dyn FnMut(&dyn PolicyView),
    ) -> Result<Field, PlanError> {
        if cancel.load(Ordering::Relaxed) {
            return Err(PlanError::Cancelled);
        }
        let mut vi = ValueIteratorLocal::new(self.build.actions.clone(), 1);
        vi.set_map_with_occupancy_grid(
            &self.build.grid,
            self.build.theta_cell_num,
            self.build.map_penalty_radius,
            self.build.map_penalty,
            self.build.goal_margin_radius,
            self.build.goal_margin_theta,
        );
        vi.set_local_shape(self.build.local_shape);
        vi.base.set_goal(goal.x, goal.y, goal_t_deg);

        let mut director =
            SolveDirector { cfg: &self.cfg, cancel, from, on_progress: on_chunk };
        let out =
            solve_observed(&mut vi.base, self.cfg.solver, self.cfg.max_solve_iter, &mut director);
        stats.iters = out.iters;
        stats.partial = out.stopped;
        if out.cancelled {
            return Err(PlanError::Cancelled);
        }
        if !out.converged && !out.stopped {
            return Err(PlanError::NotConverged);
        }
        Ok(Field::Dense(Box::new(vi)))
    }

    /// 能動的再定位の多目標場を用意する: ロスト中の仮説集合の判別点
    /// ([`Localizer::reloc_targets`]) を**全 θ** の final_state にマークして
    /// (`ValueIterator::set_goal_region`) 収束まで解き、キャッシュに載せる。
    /// 以後の [`Self::decide_qmdp`] はこの場を読む — どの仮説が真でも、その仮説に
    /// とっての判別点へ向かう行動になる。
    ///
    /// - キャッシュのゴールは**意図的に本来のゴールと一致しない** (先頭の判別点):
    ///   復帰後の最初の `prepare_goal` / 計画要求がキャッシュミスで本来のゴールを
    ///   解き直すので、専用の後始末は要らない。
    /// - 密経路のみ。compact の sink ソルバは単一ゴール前提なので `Unsupported`
    ///   を返す — 呼び出し側は受動復帰 (expansion resetting のみ) に任せること。
    // ponytail: compact 対応は多シードの frontier finalize が要る — 必要になったら。
    pub fn prepare_reloc_goal(
        &mut self,
        targets: &[(f64, f64)],
        cancel: &AtomicBool,
    ) -> Result<SolveStats, PlanError> {
        if self.cfg.use_compact() {
            return Err(PlanError::Unsupported("active relocalization needs the dense path"));
        }
        if targets.is_empty() {
            return Err(PlanError::Unsupported("no relocalization targets"));
        }
        if cancel.load(Ordering::Relaxed) {
            return Err(PlanError::Cancelled);
        }
        let mut stats =
            SolveStats { solved_now: false, iters: 0, adopted: false, partial: false };
        self.invalidate_global_jobs();
        self.cached = None; // 旧キャッシュを先に解放 (prepare_goal と同じ規律)
        if let Some(ov) = self.penalty.as_mut() {
            ov.clear();
        }
        self.dirty = false;
        self.sweep.clear();

        let mut vi = ValueIteratorLocal::new(self.build.actions.clone(), 1);
        vi.set_map_with_occupancy_grid(
            &self.build.grid,
            self.build.theta_cell_num,
            self.build.map_penalty_radius,
            self.build.map_penalty,
            self.build.goal_margin_radius,
            self.build.goal_margin_theta,
        );
        vi.set_local_shape(self.build.local_shape);
        // 半径はゴール margin と同じ、ただし最低 1 セルは確実にマークする。
        let radius = self.build.goal_margin_radius.max(vi.base.xy_resolution);
        vi.base.set_goal_region(targets, radius);

        let mut director =
            SolveDirector { cfg: &self.cfg, cancel, from: None, on_progress: &mut |_| {} };
        let out =
            solve_observed(&mut vi.base, self.cfg.solver, self.cfg.max_solve_iter, &mut director);
        stats.iters = out.iters;
        if out.cancelled {
            return Err(PlanError::Cancelled);
        }
        if !out.converged {
            return Err(PlanError::NotConverged);
        }
        self.cached = Some(CachedGoal {
            goal_x: targets[0].0,
            goal_y: targets[0].1,
            goal_t_deg: 0,
            field: Field::Dense(Box::new(vi)),
            partial: false,
        });
        stats.solved_now = true;
        Ok(stats)
    }

}
