//! ウェイポイントの先読み: **次の点の価値関数を、いまの点へ走っている間に解いて
//! おく**機構 ([`Prefetcher`])。`waypoint_prefetch: true` のときだけ立ち上がる
//! (背景と実測はリポジトリ CLAUDE.md)。
//!
//! 予備の [`PlannerCore`] ([`PlannerCore::new_solve_only`]) を専用スレッドに 1 つ
//! 持たせ、注文が来たら解かせる。要点:
//!
//! - **共有ロックを一切握らない。** ワーカーが触るのはこのモジュールの `state`
//!   だけで、走行中の核の `Mutex<PlannerCore>` には手を出さない。握ったら 10Hz の
//!   追従ループが `try_lock` に落ち続けて機体が止まる。
//! - **注文は「いまのゴール」から引く** ([`Prefetcher::note_goal`])。並び
//!   ([`Prefetcher::set_waypoints`]) の中で今のゴールに当たる点を探し、その次を
//!   解かせる。並びに無いゴール (単発の Nav2 Goal など) では何もしない。
//! - **採用は場ごと移す** ([`Prefetcher::adopt`])。解けた場は
//!   `prepare_goal_with_progress` がそのままキャッシュに載せる。
//! - **先読みした場は静的地図だけで解いてある。** 走行中に `observe_scan` が
//!   積んだ `local_penalty` は次の点の場には入らない (自分で解き直すときと同じ)。
//! - **場が 2 つ同時に生きる。** メモリ (密) とディスク (compact の sink) が
//!   2 倍要る。**ちょうど 2 つで頭打ち**なのは、新しい場を確保する前に必ず古い
//!   ほうを手放しているから — 走行中の核は solve の前に `cached = None`、
//!   こちらは注文を受ける前に採用待ちを捨てる ([`discard`])。ここを崩すと
//!   3 つ目が生まれる。
//! - **compact では solve ごとに使い捨てのディレクトリを切る必要がある**
//!   ([`super::SinkGen`])。sink ディレクトリの中のファイル名は固定で
//!   `MmapSink::new` が `truncate` するので、2 つの場が同じディレクトリを
//!   使うと後から解くほうが**走行中の場を潰す**。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use vi_reference::bridge::{yaw_to_goal_theta_deg, PoseView};

use super::{goal_matches, lock, BuildParams, CachedGoal, PlanConfig, PlanError, PlannerCore};

/// 先読みの取っ手。中身は共有なので clone しても同じワーカーを指す。
pub struct Prefetcher(Arc<Shared>);

impl Clone for Prefetcher {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

struct Shared {
    /// ゴールの一致判定 ([`goal_matches`])。走行中の核と同じ値を使うこと —
    /// 違うと「先読みは持っているのに採用されない」が静かに起きる。
    tol_xy: f64,
    tol_deg: f64,
    /// 進行中の先読みを待つときの観測間隔 ([`PlanConfig::prefetch_poll_ms`])。
    /// 呼び出し側の cancel (プリエンプト) を見る刻みでもある。
    wait_tick: Duration,
    state: Mutex<State>,
    cv: Condvar,
}

#[derive(Default)]
struct State {
    /// 先読み対象の並び (ノード側が `/waypoints` の購読で入れる)。
    waypoints: Vec<PoseView>,
    /// ワーカーにこれから解いてほしいゴール (ワーカーが拾って None にする)。
    want: Option<PoseView>,
    /// ワーカーがいま解いているゴール。
    solving: Option<PoseView>,
    /// 解き終わって採用待ちの場。**数百 MB になり得る**ので、別のゴールを
    /// 注文するときは必ず捨てる。
    ready: Option<CachedGoal>,
    /// 進行中の solve を止める旗 (`solving` と対で立つ)。
    cancel: Option<Arc<AtomicBool>>,
}

/// 並びの中で「いまのゴール」がどこにいたか。
enum Position {
    /// 次の点がある。
    Next(PoseView),
    /// 並びの最後だった (先読みするものはもう無い)。
    Last,
    /// 並びに無い (単発ゴール、あるいは並びがまだ届いていない)。
    Unknown,
}

impl Prefetcher {
    /// 予備の核とワーカースレッドを立ち上げる。`cfg` は走行中の核のものから
    /// 変えて渡すこと — 少なくとも `global_sweep: false` (先読みの場に狭域の
    /// 書き込みは無いので掃く仕事が無い)、`vi_threads` は絞った値、そして
    /// compact なら `compact_sink_gen` を走行中の核と**共有**したもの。
    pub fn spawn(build: BuildParams, cfg: PlanConfig) -> Self {
        let shared = Arc::new(Shared {
            tol_xy: cfg.goal_tolerance_xy,
            tol_deg: cfg.goal_tolerance_deg,
            wait_tick: Duration::from_millis(cfg.prefetch_poll_ms.max(1)),
            state: Mutex::new(State::default()),
            cv: Condvar::new(),
        });
        let worker_shared = Arc::clone(&shared);
        std::thread::spawn(move || worker(worker_shared, build, cfg));
        Self(shared)
    }

    /// 先読み対象の並びを差し替える (`/waypoints` を受けるたびに呼ぶ)。
    ///
    /// 受け取っただけでは何も解き始めない。**注文は走り出してから**
    /// ([`Self::note_goal`]) で、それは意図的 — 並びが latch されている構成では
    /// 起動と同時に 1 点目の solve が走り、nav2 の lifecycle 立ち上げと CPU を
    /// 奪い合うことになる。
    pub fn set_waypoints(&self, waypoints: Vec<PoseView>) {
        let mut st = lock(&self.0.state);
        st.waypoints = waypoints;
    }

    /// いま確定したゴールを伝える。並びの中でその次に当たる点を注文する。
    ///
    /// 同じ点を何度伝えてもよい (2 回目以降は no-op)。走行中の核は BT の 1Hz
    /// リプランのたびにこれを呼ぶので、並びが後から届いても取りこぼさない。
    pub(super) fn note_goal(&self, goal: PoseView) {
        let mut st = lock(&self.0.state);
        match self.position_of(&st, goal) {
            Position::Next(next) => self.request(&mut st, next),
            // 最後の点まで来た。抱えている場はもう誰も使わないので手放す
            // (compact の sink はディレクトリごと消える)。
            Position::Last => discard(&mut st),
            // 並びに無いゴール。進行中の先読みには触らない — 巡回の途中に
            // 単発ゴールが 1 つ挟まっただけかもしれない。
            Position::Unknown => {}
        }
    }

    /// 注文済み・進行中の先読みのゴール (無ければ None)。テストが「注文したか」を
    /// 見るためだけのもので、走行中に読む人はいない。
    #[cfg(test)]
    pub(super) fn pending(&self) -> Option<PoseView> {
        let st = lock(&self.0.state);
        st.want.or(st.solving)
    }

    /// `goal` の場を先読みから受け取る。
    ///
    /// - 用意できていれば場ごと返す (呼び出し側は solve を飛ばす)。
    /// - **まだ解いている最中なら終わるまで待つ。** 待っても、その場で解き直す
    ///   より遅くはならない (残り時間 ≤ 丸ごと 1 回) し、呼び出し側はどちらの
    ///   道でも同じだけロックを握る。`cancel` はこの待ちにも効く。
    /// - 別のゴールを解いていたら取り消して `None` を返す (呼び出し側が自分で
    ///   解く。そのまま走らせておくと、これから走り出すための solve と CPU を
    ///   奪い合う)。
    ///
    /// # 取り消してよい理由 (destructive なので明記する)
    ///
    /// **進行中の先読みは常に「いまキャッシュに載っているゴールの次」**になる。
    /// 注文を出すのは [`Self::note_goal`] だけで、それを呼ぶのは
    /// `prepare_goal_with_progress` がゴールを載せ終えた直後 (キャッシュヒットの
    /// ときも呼ぶ) だから。したがってここへ来る = 要求されたゴールが
    /// 「載っているゴール」でも「その次」でもない、ということで、いま進行中の
    /// 先読みは誰も待っていない。取り消し損ねは起きない — 呼び出し側はこの直後に
    /// `note_goal` を呼び、新しいゴールの次を注文し直す。
    ///
    /// 逆に言うと、**巡回中に「いまの点」を計画し直しても取り消しは起きない**
    /// (キャッシュヒットなのでここまで来ない)。BT の 1Hz リプランや復帰行動の
    /// たびに次の点の先読みが振り出しに戻る、ということはない。
    /// (`tests::replanning_the_current_goal_leaves_the_prefetch_running`)
    ///
    /// 並びが走行中に差し替わるとこの対応は一時的に崩れるが、そのときは
    /// 取り消して新しい並びで注文し直すのが正しい。
    pub(super) fn adopt(
        &self,
        goal: PoseView,
        goal_t_deg: i32,
        cancel: &AtomicBool,
    ) -> Option<CachedGoal> {
        let mut st = lock(&self.0.state);
        loop {
            if let Some(c) = st.ready.as_ref() {
                if goal_matches(
                    (c.goal_x, c.goal_y, c.goal_t_deg),
                    (goal.x, goal.y, goal_t_deg),
                    self.0.tol_xy,
                    self.0.tol_deg,
                ) {
                    return st.ready.take();
                }
                // 別ゴールの場。これから解く邪魔にしかならないので捨てる。
                st.ready = None;
            }
            let waiting_for_it = [st.solving, st.want]
                .iter()
                .flatten()
                .any(|g| self.same_goal(*g, (goal.x, goal.y, goal_t_deg)));
            if !waiting_for_it {
                cancel_in_flight(&mut st);
                return None;
            }
            if cancel.load(Ordering::Relaxed) {
                return None;
            }
            st = self.0.cv.wait_timeout(st, self.0.wait_tick).map(|(g, _)| g).unwrap_or_else(|e| {
                let (g, _) = e.into_inner();
                g
            });
        }
    }

    /// 並びの中での `goal` の位置。
    fn position_of(&self, st: &State, goal: PoseView) -> Position {
        let goal_t = yaw_to_goal_theta_deg(goal.yaw_rad);
        let Some(i) = st.waypoints.iter().position(|w| self.same_goal(*w, (goal.x, goal.y, goal_t)))
        else {
            return Position::Unknown;
        };
        match st.waypoints.get(i + 1) {
            Some(next) => Position::Next(*next),
            None => Position::Last,
        }
    }

    fn same_goal(&self, a: PoseView, b: (f64, f64, i32)) -> bool {
        goal_matches(
            (a.x, a.y, yaw_to_goal_theta_deg(a.yaw_rad)),
            b,
            self.0.tol_xy,
            self.0.tol_deg,
        )
    }

    /// `next` を注文する。既に用意できている / 解いている点なら何もしない。
    fn request(&self, st: &mut State, next: PoseView) {
        let key = (next.x, next.y, yaw_to_goal_theta_deg(next.yaw_rad));
        let have = st.ready.as_ref().is_some_and(|c| {
            goal_matches((c.goal_x, c.goal_y, c.goal_t_deg), key, self.0.tol_xy, self.0.tol_deg)
        });
        if have || [st.solving, st.want].iter().flatten().any(|g| self.same_goal(*g, key)) {
            return;
        }
        discard(st);
        st.want = Some(next);
        self.0.cv.notify_all();
    }
}

/// 進行中の先読みを止め、抱えている場を手放す。
fn discard(st: &mut State) {
    st.ready = None;
    st.want = None;
    cancel_in_flight(st);
}

/// 進行中の solve に止まれと伝える (`solving` はワーカーが自分で畳む)。
fn cancel_in_flight(st: &mut State) {
    st.want = None;
    if let Some(c) = st.cancel.take() {
        c.store(true, Ordering::SeqCst);
    }
}

/// 先読みワーカー。注文を待ち、予備の核で解き、解けた場を `ready` へ置く。
///
/// 走行中の核のロックはここでは**一度も取らない**。
fn worker(shared: Arc<Shared>, build: BuildParams, cfg: PlanConfig) {
    let mut spare = PlannerCore::new_solve_only(build, cfg);
    loop {
        // 1. 注文を待つ。
        let (goal, cancel) = {
            let mut st = lock(&shared.state);
            loop {
                if let Some(g) = st.want.take() {
                    let c = Arc::new(AtomicBool::new(false));
                    st.cancel = Some(Arc::clone(&c));
                    st.solving = Some(g);
                    break (g, c);
                }
                st = shared.cv.wait(st).unwrap_or_else(|e| e.into_inner());
            }
        };

        // 2. 解く。ここはロックの外 — 数十秒かかるので、握ったままだと
        //    `adopt` (走行中の核が呼ぶ) が丸ごとその時間ブロックされる。
        let t0 = Instant::now();
        let solved = spare.prepare_goal(goal, &cancel);
        let dt = t0.elapsed().as_secs_f64();

        // 3. 結果を置く。取り消されていた場合、予備の核のキャッシュは
        //    `prepare_goal` が先に空にしているので拾うものは無い。
        let mut st = lock(&shared.state);
        st.solving = None;
        st.cancel = None;
        match solved {
            Ok(_) => {
                st.ready = spare.cached.take();
                // 追従の tick が遅れていないかを後から突き合わせられるよう、
                // かかった時間は必ず出す (先読みは try_lock を邪魔しないので、
                // CPU を取られた症状は「制御周期がずれる」としてしか現れない)。
                eprintln!(
                    "vi_planner: prefetched the value function for ({:.2}, {:.2}) in {dt:.2}s",
                    goal.x, goal.y
                );
            }
            Err(PlanError::Cancelled) => {}
            Err(e) => {
                // 先読みの失敗は走行を止めない (そのゴールに着いたら普通に
                // 解き直すだけ)。ただし黙ると「先読みが効いていない」としか
                // 見えないので理由を出す。
                eprintln!(
                    "WARN: vi_planner: prefetch for ({:.2}, {:.2}) failed after {dt:.2}s: {e}; \
                     the goal will be solved on arrival as before",
                    goal.x, goal.y
                );
            }
        }
        shared.cv.notify_all();
    }
}
