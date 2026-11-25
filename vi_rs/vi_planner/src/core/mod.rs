//! rclrs 非依存の統合プランナ中核。**1 本の価値関数**を広域 (経路計画) と
//! 狭域 (経路追従) の両方で共有する:
//!
//!   - solve はゴールごとに 1 回だけ (`prepare_goal_with_progress`)
//!   - `plan_*` は解決済み価値関数の貪欲ロールアウト
//!   - 追従は同じ価値関数の ±1m ウィンドウをスキャンで補正しながら回す
//!
//! 狭域 → 広域の伝播は [`PlannerCore::prepare_global_job`] で仕事を取り出し、
//! ロック外で計算して [`PlannerCore::commit_global_job`] で反映する。収束前に
//! 走り出す早期走り出しは [`PlannerCore::prepare_goal_with_progress`] の doc に
//! それぞれ詳細がある (経緯と実測値はリポジトリ CLAUDE.md)。
//!
//! # ファイルの分かれ方
//!
//! - `core/mod.rs` (ここ) — 共有場の所有と核の構築。
//! - `config` — 公開設定・結果型 (このモジュールから再公開)。
//! - `goal` / `solve` — ゴールの寿命・キャッシュ・経路生成と密経路の solve。
//! - `local` / `propagation` — 局所更新と全域伝播の入口。
//! - [`compact`] — アウトオブコア経路だけの機構 (sink・追従パッチ・penalty 表・
//!   タイル修復)。`PlannerCore` の compact 側メソッドもそちらに置いてある。
//! - [`prefetch`] — 次のウェイポイントを走行中に解いておく先読み ([`Prefetcher`])。
//! - [`global`] — 領域の担当管理と、境界を含む広域ジョブの取り出し・反映。
//! - `value_grid` — 可視化の分割読み出し。
//! - `core/tests.rs` — 両経路をまたぐテスト。
//!
//! # 2 つの経路: 密 (dense) とアウトオブコア (compact)
//!
//! `PlanConfig::solver` が out-of-core かどうかで価値関数の持ち方が変わる。
//! 広域と狭域が 1 本の場を共有するという上の性質はどちらでも同じ。
//!
//! - **密**: `ValueIteratorLocal` を全域ぶん確保する。狭域はその `states` を
//!   その場で書き換える。小〜中規模の地図はこちら。
//! - **compact**: `solve_compact_mapped` が `states` を作らずに解き、確定出力
//!   (12 B/state) だけを `CompactSink` (既定は mmap ファイル) に置く。追従は
//!   `states` を必要とするので、**ロボット近傍だけを compact の場から起こした
//!   小さな密パッチ** ([`compact::Patch`]) の上で回す。
//!
//! `BuildParams` の重複定義は意図的 (クレート間依存を避ける)。このモジュールは
//! vi_lib のみに依存し、ホストで `cargo test --lib` できる (分離クレート
//! 方式; リポジトリ CLAUDE.md 参照)。

mod compact;
mod config;
mod goal;
mod local;
mod propagation;
mod solve;
mod follow;
mod global;
mod prefetch;
mod scan_collision;
mod value_grid;
#[cfg(test)]
mod tests;

pub use config::{BuildParams, Decision, PlanConfig, PlanError, PlanStats, SinkDir, SinkGen, SolveStats};
pub use follow::{DwaController, FollowController, FollowKind, GreedyController, MppiController};
pub use global::{GlobalJob, LocalJob};
// 自己位置推定は 2 系統が併存する (アルゴリズムは vi_lib、配線はノードの分担):
//   - 窓つき: vi_lib::localize の `Localizer` トレイト (external / grid / adaptive)。
//   - 全地図: vi_lib::belief の `Belief` (belief / viterbi)。窓もレベル機構も無い。
// `BeliefConfig` が両側にあるので、全地図側は別名で入れる。
pub use vi_lib::belief::{
    mode_count, spread_m, weighted_modes, Belief, BeliefConfig as WholeMapBeliefConfig,
};
pub use vi_lib::localize::{
    AdaptiveLocalizer, BeliefConfig, ExternalLocalizer, GridLocalizer, Localizer,
};

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, TryLockError};
use std::time::{Duration, Instant};

use vi_lib::bridge::{value_slice_to_occupancy, yaw_to_goal_theta_deg, PoseView};
use vi_lib::msg::{LaserScan, OccupancyGrid};
use vi_lib::params::MAX_COST;
use vi_lib::planner::{
    densify, optimal_action_at, pose_to_cell, qmdp_decide, rollout_path_on, PathPose, PolicyView,
    QmdpDecision, Rollout, RolloutStatus,
};
use vi_lib::solvers::{solve_observed, SolveFlow, SolveObserver, SolveProbe, U64Solver};
use vi_lib::value_iterator::ValueIterator;
use vi_lib::{Action, LocalShapeConfig, ValueIteratorLocal};

use vi_lib::frontier::Frontier2D;

use compact::{
    new_patch, new_repair, transition_reach, CompactField, Patch, PenaltyOverlay, Repair,
};
pub use prefetch::Prefetcher;
use scan_collision::ScanCollision;

/// Placeholder for an invalid scan range, preserving its beam index.
pub const INVALID_RANGE_M: f64 = 1.0e6;

/// 他スレッドの panic で毒された Mutex もそのまま使い続けて取る。この核の場も
/// 先読みの state も、フィールドが独立していて panic を跨ぐ不変条件を持たない
/// (価値関数は `ValueIterator` の内部状態として一貫している) ため。
pub fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|p| p.into_inner())
}

/// [`lock`] の try 版。取れなければ `None` (WouldBlock)。毒は同じく無視する。
pub fn try_lock<T>(m: &Mutex<T>) -> Option<MutexGuard<'_, T>> {
    match m.try_lock() {
        Ok(g) => Some(g),
        Err(TryLockError::WouldBlock) => None,
        Err(TryLockError::Poisoned(p)) => Some(p.into_inner()),
    }
}

/// [`try_lock`] を `deadline` まで取り直す。worker の短い保持をまたいで待つが、
/// 長い solve の間はブロックしない。
pub fn try_lock_until<T>(m: &Mutex<T>, deadline: Instant) -> Option<MutexGuard<'_, T>> {
    loop {
        if let Some(g) = try_lock(m) {
            return Some(g);
        }
        if Instant::now() >= deadline {
            return None;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// 観測一致度 → スキャン注入の減衰段数 ([`PlannerCore::observe_scan_gated`] の
/// `shift`)。`gate` 以上は 0 (満額 2048)、下回ると注入 ∝ quality/gate の 2 冪
/// 量子化 (PenaltyOverlay が指数しか持てないため冪に丸める)。上限 11
/// (2048 → 1)。`gate <= 0` でゲート無効。External localizer は quality 1.0 を
/// 返すので常に 0 — 挙動は従来と変わらない。
pub fn quality_shift(quality: f64, gate: f64) -> u32 {
    if gate <= 0.0 || quality >= gate {
        return 0;
    }
    if quality <= 0.0 {
        return 11;
    }
    ((gate / quality).log2().ceil() as u32).min(11)
}

/// solve 済み価値関数の実体。
enum Field {
    /// 密経路: 全域の `ValueIteratorLocal`。広域も狭域もこれを直接読む。
    Dense(Box<ValueIteratorLocal>),
    /// compact 経路: 確定出力 (sink) + 幾何。狭域は [`compact::Patch`] を経由する。
    Compact(Box<CompactField>),
}

struct CachedGoal {
    goal_x: f64,
    goal_y: f64,
    goal_t_deg: i32,
    field: Field,
    /// まだ解き終わっていない場か ([`PlanConfig::early_start`])。背景の解き切りが
    /// 不動点に達すると [`PlannerCore::mark_complete`] が降ろす。
    ///
    /// **捨てて解き直す判断に要る**: 解き終わる前の場でロールアウトが通らなかった
    /// とき、この印が無いと同じキャッシュを何度でも返して同じ失敗を繰り返す
    /// (BT の 1Hz リプランはキャッシュヒットなので solve に入らない = 自然には
    /// 直らない)。
    partial: bool,
}

impl CachedGoal {
    /// 背景の解き切りが不動点に達した = 場が解き終わった。`partial` の印を降ろす。
    ///
    /// 呼ぶのは「伝播の能動集合が空になった」ときだけ (密は
    /// [`PlannerCore::sweep_global`]、compact は待ち行列が空になったタイル修復)。
    /// どちらも「どのセルも動かなかった」= Bellman 作用素の不動点 = V\* なので、
    /// この時点の場は収束まで解いたものと同一。暴走ガードで諦めた修復からは**呼ばない** (場はまだ解け切って
    /// いないので、`discard_partial` の逃げ道を残しておく必要がある)。
    fn mark_complete(&mut self) {
        if !self.partial {
            return;
        }
        self.partial = false;
        // 走り出しの短縮を使ったときだけ出る 1 行。「走り出したあとちゃんと解き
        // 切ったのか」を見る手掛かりが他に無い (掃きの done ログは狭域の伝播でも
        // 出るので区別が付かない)。
        eprintln!(
            "vi_planner: the value function is now solved to convergence \
             (early_start let the robot leave before it was)"
        );
    }
}

/// 密経路の全域伝播 (能動集合) の作業場。
///
/// **地図を丸ごと掃かない。** あるセルの値が動くのは、遷移先の値かペナルティが
/// 動いたときだけなので、動いたセルを遷移の届く距離だけ膨張させた範囲を掃けば
/// 取りこぼさない。仕事量は地図の広さではなく**変化が広がった範囲**で決まる
/// (compact 側のタイル修復 [`compact::Repair`] と同じ考えで、粒度がタイルでは
/// なくセル)。空フロンティア = どのセルも動かなかった = 不動点。
///
/// ラウンドを bounded なジョブへ切り出すため、その続きだけを核に保持する。
#[derive(Default)]
struct DenseSweep {
    nx: usize,
    /// `pending` に積み済みのセル印 (`iy * nx + ix`)。
    pending_mark: Vec<u8>,
    /// **次の**ラウンドの種。前のラウンドで値が動いたセルと、狭域が外から
    /// 積んだ矩形が入る (2D。θ 層は候補セルごとにまとめて評価するので持たない)。
    pending: Vec<(u32, u32)>,
    /// 再利用する 2D フロンティア膨張器。
    dilator: Option<Frontier2D>,
    /// いま処理中のラウンドの候補セル (種を `reach` だけ膨張させて列挙したもの)。
    cands: Vec<(u32, u32)>,
    /// `cands` の次に見る位置。
    pos: usize,
    /// いまの伝播で評価した候補セル数 (キューが空になると畳む)。
    visits: usize,
    /// 直近の伝播 1 回ぶんの評価セル数 (ログ用に `visits` を畳む前に写す)。
    last_visits: usize,
    /// 遷移が届くセル数。幾何だけで決まるので初回に測って持ち回す。
    reach: Option<u32>,
}

impl DenseSweep {
    /// グローバルセル矩形 `(x0, x1, y0, y1)` (両端含む) を次ラウンドの種に加える。
    fn seed_rect(&mut self, nx: i32, ny: i32, (x0, x1, y0, y1): (i32, i32, i32, i32)) {
        let (nxu, nyu) = (nx.max(0) as usize, ny.max(0) as usize);
        let total = nxu * nyu;
        if self.nx != nxu || self.pending_mark.len() != total {
            self.nx = nxu;
            self.pending_mark.clear();
            self.pending_mark.resize(total, 0);
            self.pending.clear();
            self.dilator = None;
        }
        for iy in y0.max(0)..=y1.min(ny - 1) {
            let row = iy as usize * nxu;
            for ix in x0.max(0)..=x1.min(nx - 1) {
                let idx = row + ix as usize;
                if self.pending_mark[idx] == 0 {
                    self.pending_mark[idx] = 1;
                    self.pending.push((ix as u32, iy as u32));
                }
            }
        }
    }

    fn pending_len(&self) -> usize {
        self.pending.len()
    }

    /// 溜まった種を `reach` だけ膨張して `cands` に詰め、`pending` を空にする。
    fn refill_cands(&mut self, nx: i32, ny: i32, reach: u32) {
        self.cands.clear();
        self.pos = 0;
        if self.pending.is_empty() {
            return;
        }
        let nxu = nx as usize;
        for &(ix, iy) in &self.pending {
            self.pending_mark[iy as usize * nxu + ix as usize] = 0;
        }
        let dil = self
            .dilator
            .get_or_insert_with(|| Frontier2D::new(nx as u32, ny as u32));
        dil.dilate_into(&self.pending, reach, reach, &mut self.cands);
        self.pending.clear();
    }

    /// 伝播 1 回ぶんの終わり。評価数はログ用に写してから畳む。
    fn settle(&mut self) {
        self.last_visits = self.visits;
        self.visits = 0;
    }

    /// 掃く仕事を捨てる (ゴールを解き直したとき)。`reach` は幾何なので残す。
    fn clear(&mut self) {
        for &(ix, iy) in &self.pending {
            self.pending_mark[iy as usize * self.nx + ix as usize] = 0;
        }
        self.pending.clear();
        self.cands.clear();
        self.pos = 0;
        self.settle();
    }
}

pub struct PlannerCore {
    build: BuildParams,
    cfg: PlanConfig,
    cached: Option<CachedGoal>,
    /// compact 経路の追従用パッチ (ゴール非依存なのでキャッシュの外に置く)。
    /// 密経路では使わない。
    patch: Option<Patch>,
    /// compact 経路で観測した `local_penalty` の全域表 (密経路では None)。
    /// 密経路の `states.local_penalty` に対応するので、寿命も同じ
    /// (ゴールを解き直すと `states` ごと作り直される = ここでは `clear`)。
    penalty: Option<PenaltyOverlay>,
    /// compact 経路の全域伝播の作業場 (密経路と `global_sweep: false` では None)。
    repair: Option<Repair>,
    /// 密経路の全域伝播の作業場 (compact では使わない)。`repair` の密版。
    sweep: DenseSweep,
    global: global::Manager,
    /// 狭域が共有場を動かしたか。`refine_for` が Δ>0 を出すと立ち、全域掃きが
    /// Δ=0 で 1 周し終えると落ちる。全域掃きを回すかの唯一の判断材料
    /// (「狭域が通れないと言っている」を別途検出する必要はない — 通れないなら
    /// 必ず値が動くので、これがその信号そのものになる)。
    dirty: bool,
    /// 次のウェイポイントの先読み ([`Prefetcher`])。`None` = 先読みなし
    /// (`waypoint_prefetch: false`) で、そのときこの核の挙動は従来と同じ。
    prefetch: Option<Prefetcher>,
    /// 追従の道具 (パッチ・penalty 表・修復タイル) を持たず solve だけをする核か。
    /// 先読みワーカーが抱える予備の核がこれ ([`PlannerCore::new_solve_only`])。
    solve_only: bool,
    /// follow 1 tick の判断器 ([`PlanConfig::follow_controller`] から組む)。
    /// solve・ロールアウトには関与しない — `decide` だけがこれを通る。
    follow: Box<dyn FollowController>,
    scan_collision: ScanCollision,
    /// greedy が実行中の行動 id と、続けた tick 数 ([`Self::decide`])。
    running_action: Mutex<Option<(usize, u32)>>,
}

/// 2 つのゴール `(x, y, θ[deg])` を「同じ」とみなせるか。キャッシュの再利用判定と
/// 先読み ([`Prefetcher`]) の照合が同じ規則を使うためにここに 1 つだけ置く
/// (別々にすると、先読みが用意した場を採用できないゴールが静かに生まれる)。
fn goal_matches(a: (f64, f64, i32), b: (f64, f64, i32)) -> bool {
    a == b
}

/// `from` からゴール圏まで、いまの場の方策だけで辿り着けるか。早期走り出し
/// ([`PlanConfig::early_start`]) の判定はこれ 1 つで、密経路と compact 経路が同じ
/// 規則を使う (どちらも [`rollout_path_on`] = `plan` が実際に返す経路の作り方そのもの
/// なので、「走り出させたのに `plan` が失敗する」がこの判定を通った場では起きない)。
fn reaches_goal(p: &dyn PolicyView, cfg: &PlanConfig, from: PoseView) -> bool {
    rollout_path_on(
        p,
        from.x,
        from.y,
        from.yaw_rad,
        cfg.max_rollout_steps,
        cfg.start_tolerance_cells,
    )
    .reached_goal()
}

/// solve の進行制御 ([`SolveObserver`] 実装)。密・compact の両経路がこれ 1 つを使う —
/// 旧実装の「密: 手書きチャンクループ / compact: stop クロージャ」の置き換え。
/// 境界 (ソルバの自然な粒度: 密系はラウンド/スイープ × `solve_chunk`、compact は
/// バンド finalize) ごとに呼ばれ、
///   1. cancel (プリエンプト) を観測し、
///   2. 途中経過を `on_progress` へ流し (probe の場は要求時にだけ具現化される)、
///   3. `from` があれば早期走り出し ([`PlanConfig::early_start`]) を判定する。
/// 判定は [`reaches_goal`] そのもの = `plan` が実際に返す経路の作り方なので、密でも
/// compact でも「走り出させたのに `plan` が失敗する」形の外し方をしない。
///
/// 3 の `Stop` は**走り出しの合図であって solve の終わりではない**。ここで
/// ソルバの呼び出しは畳むが、場は解き終わっていない印付きでキャッシュに載り、
/// 残りは背景の解き切り (`sweep_global`) が同じ Bellman 更新で詰める。
struct SolveDirector<'a> {
    cfg: &'a PlanConfig,
    cancel: &'a AtomicBool,
    /// 早期走り出しの起点 (None = 収束まで解いてから返す)。
    from: Option<PoseView>,
    on_progress: &'a mut dyn FnMut(&dyn PolicyView),
}

impl SolveObserver for SolveDirector<'_> {
    fn interval(&self) -> u32 {
        self.cfg.solve_chunk.max(1)
    }
    fn boundary(&mut self, probe: &mut dyn SolveProbe) -> SolveFlow {
        if self.cancel.load(Ordering::Relaxed) {
            return SolveFlow::Cancel;
        }
        let p = probe.policy();
        (self.on_progress)(p);
        if let Some(from) = self.from {
            if reaches_goal(p, self.cfg, from) {
                return SolveFlow::Stop;
            }
        }
        SolveFlow::Continue
    }
}

/// `(ix, iy, it)` に方策があれば Decision::Action を返す (読み取り専用)。
fn action_at(vi: &ValueIterator, ix: i32, iy: i32, it: i32) -> Option<Decision> {
    let id = optimal_action_at(vi, ix, iy, it);
    if id < 0 {
        return None;
    }
    let a = vi.actions.iter().find(|a| a.id == id)?;
    Some(Decision::Action { id: Some(id as usize), fw: a.delta_fw, rot_deg: a.delta_rot })
}

/// 範囲内で final_state か (境界チェック込み)。
fn is_final(vi: &ValueIterator, ix: i32, iy: i32, it: i32) -> bool {
    vi.in_map_area(ix, iy)
        && it >= 0
        && it < vi.cell_num_t
        && vi.states[vi.to_index(ix, iy, it) as usize].final_state
}

/// solve 済み場の θ=0 全域スライスを可視化用 OccupancyGrid に描画する (0..=100、
/// 未到達 -1)。ビューは [`PolicyView`] — 密 (`&ValueIterator`) も compact (sink ビュー)
/// も同じ実装で描く。solve 中の途中経過 (`*_with_progress` のコールバック) からも呼べる。
pub fn value_grid_on(p: &dyn PolicyView, threshold_steps: u64) -> OccupancyGrid {
    let (nx, ny, _) = p.cell_num();
    let (w, h) = (nx as usize, ny as usize);
    let mut slice = vec![0u64; w * h];
    for iy in 0..ny {
        for ix in 0..nx {
            slice[iy as usize * w + ix as usize] = p.value_at(ix, iy, 0);
        }
    }
    let (ox, oy) = p.map_origin();
    OccupancyGrid {
        width: nx,
        height: ny,
        resolution: p.xy_resolution(),
        origin_x: ox,
        origin_y: oy,
        origin_quat: p.map_origin_quat(),
        data: value_slice_to_occupancy(&slice, w, h, threshold_steps),
    }
}

impl PlannerCore {
    pub fn new(build: BuildParams, cfg: PlanConfig) -> Self {
        // penalty 表は compact だけ (密は states がそのまま持つ)。8 B/セルなので
        // 津田沼の 0.25 m/cell (1177x800) で約 7.5 MB。
        let penalty = cfg
            .use_compact()
            .then(|| PenaltyOverlay::new(build.grid.width, build.grid.height));
        // 修復タイルは遷移表の再計算が要るので、パッチと一緒に最初の solve で作る
        // (`prepare_goal_with_progress`)。掃きスレッドの中で作ると、数秒かかる
        // 再計算をロックの中でやることになる。
        let follow = follow::make_controller(&cfg, &build.actions);
        Self {
            build,
            cfg,
            follow,
            scan_collision: ScanCollision::default(),
            running_action: Mutex::new(None),
            cached: None,
            patch: None,
            penalty,
            repair: None,
            sweep: DenseSweep::default(),
            global: global::Manager::default(),
            dirty: false,
            prefetch: None,
            solve_only: false,
        }
    }

    /// 先読み ([`Prefetcher`]) を付ける。付けると `prepare_goal_*` が 2 つのことを
    /// するようになる: ゴールの場が先読みで用意できていれば solve を飛ばして
    /// 受け取り、ゴールが確定するたびに**並びの次の点**を先読みへ注文する。
    /// 付けなければ (既定) この核の挙動は従来のまま。
    pub fn with_prefetch(mut self, prefetch: Prefetcher) -> Self {
        self.prefetch = Some(prefetch);
        self
    }

    /// 先読みワーカーが抱える予備の核。追従はしないので、パッチも penalty 表も
    /// 修復タイルも作らない (作ると遷移表の再計算ぶんだけ最初の先読みが遅くなる)。
    fn new_solve_only(build: BuildParams, cfg: PlanConfig) -> Self {
        Self { solve_only: true, penalty: None, ..Self::new(build, cfg) }
    }

    // ──────────────────────────────────────────────────────────────────────
    // solve (広域・狭域で共有する唯一の価値関数)
    //
    // 密は `solve_dense` (すぐ下)、compact は `compact::` 側の `solve_compact`。
    // ──────────────────────────────────────────────────────────────────────

    // ──────────────────────────────────────────────────────────────────────
    // 広域: compute_path_to_pose
    // ──────────────────────────────────────────────────────────────────────

    // ──────────────────────────────────────────────────────────────────────
    // 狭域: follow_path
    // ──────────────────────────────────────────────────────────────────────

    /// 現在ゴールまでの XY 距離 (Feedback 用)。ゴール未設定なら None。
    pub fn goal_distance(&self, x: f64, y: f64) -> Option<f64> {
        self.cached
            .as_ref()
            .map(|c| ((c.goal_x - x).powi(2) + (c.goal_y - y).powi(2)).sqrt())
    }

    // ──────────────────────────────────────────────────────────────────────
    // 狭域 → 広域: 共有場の全域掃き
    //
    // 入口は `sweep_global` 1 つ。密はここでセル単位の能動集合、compact は
    // `compact::` 側の `repair_one_tile` (タイル単位の能動集合) へ分岐する。
    // ──────────────────────────────────────────────────────────────────────


}
