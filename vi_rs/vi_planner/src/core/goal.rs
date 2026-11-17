//! Goal lifecycle, cache adoption, and rollout over the shared value function.
use super::*;

impl PlannerCore {
    /// キャッシュ中のゴールが `goal` と同一か。追従スレッドが
    /// 「自分のゴールの価値関数がまだ載っているか」を毎 tick 確認するのに使う
    /// (広域側の計画要求が別ゴールで solve し直すとキャッシュが差し替わるため)。
    pub fn is_cached_goal(&self, goal: PoseView) -> bool {
        self.cache_matches(&goal, yaw_to_goal_theta_deg(goal.yaw_rad))
    }

    pub(super) fn cache_matches(&self, goal: &PoseView, goal_t_deg: i32) -> bool {
        let Some(c) = self.cached.as_ref() else { return false };
        goal_matches(
            (c.goal_x, c.goal_y, c.goal_t_deg),
            (goal.x, goal.y, goal_t_deg),
        )
    }

    /// ゴールへ向けた価値関数を用意する。同一ゴール (許容差内) なら solve を
    /// スキップする (このとき直近スキャン由来の `local_penalty` も温存される)。
    /// `cancel` は solve 中に `solve_chunk` ごとに観測される。
    pub fn prepare_goal(
        &mut self,
        goal: PoseView,
        cancel: &AtomicBool,
    ) -> Result<SolveStats, PlanError> {
        self.prepare_goal_with_progress(goal, None, cancel, &mut |_| {})
    }

    /// `prepare_goal` と同じだが、solve の境界 (密: `solve_chunk` 反復ごと /
    /// compact: バンド finalize ごと) に `on_chunk` を呼ぶ (途中経過の value_function
    /// 可視化用 — [`value_grid_on`] がそのまま受け取れる `&dyn PolicyView` を渡す)。
    /// キャッシュヒット時は呼ばれない。
    ///
    /// 先読み ([`Prefetcher`]) が付いていれば、solve の前にそちらを見る。用意が
    /// できていれば受け取って solve を飛ばし (`stats.adopted`)、まだ解いている
    /// 最中なら終わるまで待つ。どちらにせよ最後に**並びの次の点**を注文するので、
    /// 巡回中は「いまのゴールへ走っている間に次のゴールが解けている」状態になる。
    ///
    /// # 早期走り出し (`early_start`)
    ///
    /// `from` は**機体のいまの姿勢**。[`PlanConfig::early_start`] が true でこれが
    /// 与えられていると、「`from` からゴールまで方策が繋がった」時点でこの呼び出しが
    /// 返り、機体が走り出せるようになる。判定はロールアウトそのもの — 途中の場の
    /// 上で [`rollout_path_on`] を試し、ゴール圏に着いたら返す。「値がロボットの
    /// 近くまで来たか」を距離や値で近似せず、**使う経路が引けたかを直接見る**
    /// (近似だと、経路上の 1 セルだけ未確定で `plan` が失敗する形の外し方をする)。
    ///
    /// **場を解くのはここでやめない。** 返る時点の場は `partial` の印付きで
    /// キャッシュに載り、残りは機体が走っている間に背景で解き切られる — 密は
    /// [`PlannerCore::sweep_global`] の能動集合、compact は [`compact::Repair`] の
    /// タイル修復で、どちらも solve と同じ `value_iteration_at`。能動集合が空に
    /// なった時点で [`PlannerCore::mark_complete`] が印を降ろし、その場は収束まで
    /// 解いたものと同一になる。**伝播は「動いたところ」から広げる形なので、印を
    /// 立てるだけでは空振りする** — 未確定域は地図のどこにでもあるので、密は地図を
    /// 丸ごと種に積み (`seed_sweep_all`)、compact は丸ごと修復の待ち行列へ入れる。
    ///
    /// 走り出した直後の場でよい理由は経路の側にある。compact 経路の finalize は値の
    /// 昇順に進むので、sink に載っている列は**最後まで解いたときと同じ値** (未確定の
    /// 列が `MAX_COST` のまま残っているだけ)。密経路は値が上から単調に下がるので、
    /// 途中の場は常に Bellman の上界 = 貪欲降下は必ず値を下げながらゴールに着く
    /// (循環しない) — ただし解き終わるまでは**最短とは限らない**。
    ///
    /// 代償は「解き終わるまでゴールまでの経路の外は未確定」であること。機体が経路から
    /// 外れると未確定領域に入り得るので、`plan` のロールアウトが通らなかったときは
    /// [`PlannerCore::discard_partial`] で捨てて解き直す道を必ず残しておくこと
    /// (キャッシュヒットは solve に入らないので、放っておくと同じ失敗が続く)。
    pub fn prepare_goal_with_progress(
        &mut self,
        goal: PoseView,
        from: Option<PoseView>,
        cancel: &AtomicBool,
        on_chunk: &mut dyn FnMut(&dyn PolicyView),
    ) -> Result<SolveStats, PlanError> {
        let goal_t_deg = yaw_to_goal_theta_deg(goal.yaw_rad);
        let mut stats =
            SolveStats { solved_now: false, iters: 0, adopted: false, partial: false };

        if self.cache_matches(&goal, goal_t_deg) {
            // ここも注文の入口。BT は追従中も 1Hz で計画を投げ直すので、最初の
            // 1 回で注文できていなくても (並びがまだ届いていない等) 次の tick で
            // 拾える。同じ点の注文は 2 回目以降 no-op。
            self.request_next(goal);
            stats.partial = self.cached.as_ref().is_some_and(|c| c.partial);
            return Ok(stats);
        }

        self.invalidate_global_jobs();
        self.cached = None; // 旧キャッシュ (数 GB になり得る) を先に解放
        if let Some(p) = self.patch.as_mut() {
            p.at = None; // 別ゴールの値が残ったパッチで走らせない
        }
        // 密経路は `states` を作り直すので local_penalty がここで消える。compact の
        // penalty 表も同じ寿命にそろえる (「消したければゴールを取り直す」の実体)。
        if let Some(ov) = self.penalty.as_mut() {
            ov.clear();
        }
        // 解き直した直後は収束済み。前のゴールで積んだ「伝播させる仕事がある」印は
        // 持ち越さない (持ち越すと最初の 1 掃きが必ず無駄に回る)。待ち行列に残った
        // タイルも同じで、前のゴールの場を修復しても意味がない。
        self.dirty = false;
        self.sweep.clear();
        if let Some(r) = self.repair.as_mut() {
            r.clear();
        }

        // パッチも修復タイルも幾何だけで決まるので初回だけ作る (遷移表の再計算は
        // 64^3 サブセルサンプリング x 行動 x θ で重い)。**solve の外に出してある**
        // のは、先読みから受け取る経路が solve を通らないため — 巡回の 1 点目から
        // 受け取ると、ここが solve の中にあると追従の道具が無いまま走り出す。
        if self.cfg.use_compact() && !self.solve_only && self.patch.is_none() {
            let p = new_patch(&self.build)?;
            if self.cfg.global_sweep && self.repair.is_none() {
                self.repair = Some(new_repair(&self.build, p.reach)?);
            }
            self.patch = Some(p);
        }

        let adopted = match self.prefetch.as_ref() {
            Some(pf) => pf.adopt(goal, goal_t_deg, cancel),
            None => None,
        };
        // 早期走り出しの起点。`early_start` でも起点が無ければ収束まで解いてから
        // 返す (先読みのワーカーがここを通る道でもある — あちらは `prepare_goal`
        // 経由で `from: None`。次の点を解いている間に機体は動くので、いまの姿勢を
        // 起点にした「繋がった」判定は着いた頃には意味がない)。
        let from = self.cfg.early_start.then_some(from).flatten();

        let cached = match adopted {
            Some(c) => {
                stats.adopted = true;
                c
            }
            None => {
                let field = if self.cfg.use_compact() {
                    self.solve_compact(&goal, goal_t_deg, from, &mut stats, cancel, on_chunk)?
                } else {
                    self.solve_dense(&goal, goal_t_deg, from, &mut stats, cancel, on_chunk)?
                };
                CachedGoal {
                    goal_x: goal.x,
                    goal_y: goal.y,
                    goal_t_deg,
                    field,
                    partial: stats.partial,
                }
            }
        };

        stats.solved_now = true;
        self.cached = Some(cached);
        // 走り出せる形になっただけで、場はまだ解き終わっていない。**残りは走りながら
        // 背景で解き切る** — 掃きを回すかの判断材料は `dirty` 1 つなので、密経路は
        // ここで立てるだけでよい。
        //
        // compact の伝播は「変化した範囲」から広げる形なので、立てるだけでは待ち行列が
        // 空で空振りする。未確定域は地図のどこにでもあるので、ここで地図を丸ごと
        // 待ち行列へ入れて起点にする (確定済みのタイルは 1 パス Δ=0 で抜けるだけ)。
        //
        // ponytail: 全タイル投入は「未確定域の輪郭だけ入れる」より無駄がある。
        // 広域地図で背景の掃きが長すぎたら、finalize 済みの範囲を solve から
        // 受け取って輪郭のタイルだけ積むこと。
        if stats.partial {
            self.dirty = true;
            match self.repair.as_mut() {
                Some(r) => r.enqueue_all(),
                None => self.seed_sweep_all(),
            }
        }
        self.request_next(goal);
        Ok(stats)
    }

    /// まだ解き終わっていない場を捨てる。捨てたら true、解き終わった場や
    /// キャッシュ無しなら false。
    ///
    /// **これが `early_start` の唯一の逃げ道**。解き終わる前の場でロールアウトが
    /// 通らない / 方策が引けないとき、キャッシュが載ったままだと BT が何度
    /// 投げ直しても `prepare_goal_*` はキャッシュヒットで返って同じ失敗を繰り返す。
    /// 捨てておけば次の要求が解き直す (`from` を渡さなければ収束まで返らない)。
    pub fn discard_partial(&mut self) -> bool {
        if !self.cached.as_ref().is_some_and(|c| c.partial) {
            return false;
        }
        self.invalidate_global_jobs();
        self.cached = None;
        if let Some(p) = self.patch.as_mut() {
            p.at = None; // 解き終わっていない場から起こしたパッチで走らせない
        }
        true
    }

    /// いま載っている場がまだ解き終わっていないか ([`PlanConfig::early_start`])。
    pub fn is_partial(&self) -> bool {
        self.cached.as_ref().is_some_and(|c| c.partial)
    }

    /// 先読みへ「いまのゴールはこれ」と伝える (並びの次の点の注文がここで出る)。
    /// 先読みが付いていなければ何もしない。
    pub(super) fn request_next(&self, goal: PoseView) {
        if let Some(pf) = self.prefetch.as_ref() {
            pf.note_goal(goal);
        }
    }

    /// start から goal への経路を計画する。価値関数は `prepare_goal` と共有され、
    /// 同一ゴールなら solve をスキップしてロールアウトだけを行う。
    pub fn plan(
        &mut self,
        start: PoseView,
        goal: PoseView,
        cancel: &AtomicBool,
    ) -> Result<(Vec<PathPose>, PlanStats), PlanError> {
        self.plan_with_progress(start, goal, cancel, &mut |_| {})
    }

    /// `plan` の途中経過コールバック付き版 (`prepare_goal_with_progress` と同じ規約)。
    ///
    /// `start` は早期走り出し (`early_start`) の起点でもある。解き終わる前の場で
    /// ロールアウトが通らなかったときは**その場で捨てて解き直す** — 失敗を
    /// そのまま返すと、キャッシュに載ったままの場を BT が何度でも引き直して
    /// 同じ失敗を繰り返す。
    pub fn plan_with_progress(
        &mut self,
        start: PoseView,
        goal: PoseView,
        cancel: &AtomicBool,
        on_chunk: &mut dyn FnMut(&dyn PolicyView),
    ) -> Result<(Vec<PathPose>, PlanStats), PlanError> {
        let mut s = self.prepare_goal_with_progress(goal, Some(start), cancel, on_chunk)?;

        let mut r = self.rollout(start);
        if !r.reached_goal() && self.discard_partial() {
            eprintln!(
                "vi_planner: the not-yet-finished value function (early_start) does not roll out \
                 from ({:.2}, {:.2}): {:?} — solving this goal to convergence instead",
                start.x, start.y, r.status
            );
            // 起点を渡さない = 今度は収束まで返らない。先読みが次の点を解いている
            // 最中ならここで取り消されて注文し直しになる (走っている間に解き直す
            // 時間はあるので、失う仕事より繰り返す失敗のほうが高い)。
            s = self.prepare_goal_with_progress(goal, None, cancel, on_chunk)?;
            r = self.rollout(start);
        }
        if !r.reached_goal() {
            return Err(PlanError::Rollout(r.status));
        }
        let poses = if self.cfg.path_spacing > 0.0 {
            densify(&r.poses, self.cfg.path_spacing)
        } else {
            r.poses
        };
        Ok((
            poses.clone(),
            PlanStats {
                solved_now: s.solved_now,
                iters: s.iters,
                adopted: s.adopted,
                partial: s.partial,
                poses: poses.len(),
            },
        ))
    }

    /// いま載っている場の貪欲ロールアウト。ゴール未設定なら `NoAction`。
    ///
    /// 密経路は追従で注入された local_penalty 込みの値を読む。compact 経路は
    /// sink を読む。sink には狭域が `commit_window` で返した値も入っているので、
    /// こちらも動的障害物込みの値になる (モジュール冒頭)。
    pub(super) fn rollout(&self, start: PoseView) -> Rollout {
        let Some(cached) = self.cached.as_ref() else {
            return Rollout { poses: Vec::new(), status: RolloutStatus::NoAction };
        };
        let go = |p: &dyn PolicyView| {
            rollout_path_on(
                p,
                start.x,
                start.y,
                start.yaw_rad,
                self.cfg.max_rollout_steps,
                self.cfg.start_tolerance_cells,
            )
        };
        match &cached.field {
            Field::Dense(vi) => go(&vi.base),
            Field::Compact(f) => go(&f.policy()),
        }
    }

}
