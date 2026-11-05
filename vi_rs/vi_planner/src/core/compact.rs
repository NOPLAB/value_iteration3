//! アウトオブコア (compact) 経路だけの機構。密 (dense) 経路との違いは価値関数の
//! **持ち方**だけで、「広域と狭域が 1 本の場を共有する」という [`super`] の性質は
//! どちらでも同じように成り立つ — 密の `states` にあたるものが、こちらでは
//! [`CompactField`] の sink になる。[`PlannerCore`] のうち compact でしか走らない
//! メソッド (`solve_compact` / `harvest_penalties` / `commit_window` /
//! `repair_one_tile`) もここに置いてある。
//!
//! # 追従は近傍の密パッチで回す ([`Patch`])
//!
//! 追従は `states` を必要とするので、**ロボット近傍だけを compact の場から起こした
//! 小さな密パッチ**の上で回す。パッチは「ローカルウィンドウ (±1m) + 遷移が届く
//! 距離 + 動ける余裕」の大きさで、ウィンドウの外側は compact の値のまま凍結して
//! 境界条件に使う。凍結境界が成り立つ条件は「ウィンドウ内のセルの遷移先がパッチ内に
//! 収まる」ことで、これは遷移表の実測値で起動時に検証する ([`transition_reach`])。
//!
//! # sink が共有場になる (書き戻し + penalty 表)
//!
//! パッチだけで回すと、狭域の成果はパッチの外へ出られない。sink は全域ぶんの配列
//! なので、**狭域が動かした窓を毎 tick sink へ書き戻す** ([`PlannerCore::commit_window`])
//! と、密経路の `states` と同じ意味で共有場になる。
//!
//! ただし sink は `(value, action)` の 12 B/state しか持たない。値を書き戻しても
//! **その値を正当化する `local_penalty` がどこにも残らない**ので、次にその区画を
//! 起こし直すと penalty 0 で価値反復が回り、上げた値が静かに元へ戻る。そこで
//! 観測した penalty だけを別に覚える ([`PenaltyOverlay`], 8 B/セル)。密経路の
//! `states.local_penalty` の代わりで、寿命も同じ (ゴールを解き直すと消える)。
//!
//! # 全域伝播はタイル修復で回す ([`Repair`])
//!
//! 密経路の [`PlannerCore::sweep_global`] は全域の `states` を要求するので compact
//! では使えない。代わりに sink を**タイル単位**で起こして掃く。
//! 1 タイル = 更新する interior (既定 16 セル角) + 遷移が届く距離 `reach` の halo。
//! halo を凍結境界にして interior だけを掃き、変わった列を sink へ返し、変化の
//! 外接矩形から `reach` セル以内のタイルを待ち行列へ入れ直す。キューが空になったら
//! 収束 = 全域 Gauss–Seidel と同じ不動点。更新式は `value_iteration_at` そのまま
//! なので、狭域・広域・solve・修復の 4 者が同一の更新式のままになる。仕事量は
//! 地図の大きさではなく**実際に影響が及ぶ範囲**に比例し、メモリはタイル 1 枚ぶん
//! (暴走ガードとして訪問回数に上限を置いてある)。
//!
//! 修復が追従パッチの footprint を書き換えたら、パッチを無効化して次の tick で
//! 起こし直させる ([`PlannerCore::repair_one_tile`] の「パッチの無効化」)。パッチは
//! sink の写しを凍結して持っているので、これをしないと修復とパッチが同じセルを
//! 互いに上書きし合って**収束しない** (値は正しいまま、掃きが終わらなくなる)。

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};

use vi_lib::bridge::PoseView;
use vi_lib::msg::{OccupancyGrid, Quaternion};
use vi_lib::params::{MAX_COST, PROB_BASE};
use vi_lib::planner::CompactPolicy;
use vi_lib::planner::PolicyView;
use vi_lib::solvers::frontier2d_sparse_compact::{
    default_threads, solve_compact_mapped_observed, CompactSink, MmapSink, RamSink,
};
use vi_lib::state::State;
use vi_lib::value_iterator::ValueIterator;
use vi_lib::{Action, ValueIteratorLocal};

use super::{
    BuildParams, Field, PlanError, PlannerCore, SinkDir, SinkGen, SolveDirector, SolveStats,
};

/// compact 経路の solve 結果。`sink` は orig 索引で `(total_cost, action)` を返す確定出力。
pub(super) struct CompactField {
    pub(super) sink: Box<dyn CompactSink + Send>,
    actions: Vec<Action>,
    pub(super) cell_num: (i32, i32, i32),
    pub(super) resolution: f64,
    pub(super) origin: (f64, f64),
    pub(super) origin_quat: Quaternion,
    goal: (f64, f64, i32),
}

impl CompactField {
    pub(super) fn policy(&self) -> CompactPolicy<'_> {
        CompactPolicy::new(
            self.sink.as_ref(),
            &self.actions,
            self.cell_num,
            self.resolution,
            self.origin,
            self.goal,
        )
        .with_origin_quat(self.origin_quat.clone())
    }

    /// グローバルセル `(ix,iy,it)` の sink 索引 (usize 演算; 広域地図で i32 が溢れる)。
    pub(super) fn orig(&self, ix: i32, iy: i32, it: i32) -> usize {
        it as usize
            + ix as usize * self.cell_num.2 as usize
            + iy as usize * self.cell_num.0 as usize * self.cell_num.2 as usize
    }
}

/// compact 経路の追従用パッチ: ロボット近傍だけを compact の場から起こした密な
/// `ValueIteratorLocal`。幾何 (解像度・θ 数・行動) だけで決まりゴールには依らないので、
/// ゴールが変わってもパッチ自体は作り直さず、中身 (`at` とハイドレート値) だけ入れ替える。
///
/// 寸法: `half = 2*ウィンドウ半径 + 遷移到達距離 + 2` セル。ウィンドウの外側は
/// compact の値のまま凍結して境界条件に使う。ロボットがパッチの縁に近づいたら
/// 置き直す (`needs_recenter`)。
pub(super) struct Patch {
    pub(super) vi: ValueIteratorLocal,
    /// パッチ半径 [セル] (パッチは `2*half+1` 辺の正方形)。
    pub(super) half: i32,
    /// 遷移が届く最大セル数 (遷移表の実測値)。凍結境界の成立条件に使う。
    pub(super) reach: i32,
    /// パッチ左下のグローバルセル座標。`None` = 未ハイドレート。
    pub(super) at: Option<(i32, i32)>,
}

impl Patch {
    /// ロボットのグローバルセル `(gx, gy)` に対してパッチを置き直す必要があるか。
    /// 判定は寸法ではなく凍結境界の条件そのもの: ウィンドウ (±`local_ixy_range`) と
    /// そこから遷移が届く先 (±`reach`) がパッチに収まらなくなったら置き直す。
    pub(super) fn needs_recenter(&self, gx: i32, gy: i32) -> bool {
        let Some((p0x, p0y)) = self.at else { return true };
        let need = self.vi.local_ixy_range + self.reach;
        let side_max = 2 * self.half;
        gx - need < p0x
            || gy - need < p0y
            || gx - p0x + need > side_max
            || gy - p0y + need > side_max
    }

    /// compact の場と静的地図からパッチの `states` を起こす。
    /// `p0` はパッチ左下のグローバルセル座標 (地図外へ食い込んでよい — その分は
    /// 占有セル扱いになり、`action_cost` が MAX_COST を返す = 元の地図外判定と同じ)。
    ///
    /// `overlay` からは観測済みの `local_penalty` を**パッチ全体に**戻す。窓の中
    /// だけでは足りない: `action_cost` は遷移先の `local_penalty` を読むので、
    /// 凍結境界側が 0 のままだと窓の縁の行動コストが sink の値と食い違う。
    pub(super) fn hydrate(
        &mut self,
        f: &CompactField,
        build: &BuildParams,
        overlay: Option<&PenaltyOverlay>,
        p0: (i32, i32),
    ) {
        self.at = Some(p0);
        hydrate_states(&mut self.vi.base, f, build, overlay, p0);
    }


}

/// compact 経路で観測した `local_penalty` の全域表 (密経路では作らない)。
///
/// sink は `(value, action)` しか持たないので、書き戻した値を裏付ける penalty が
/// 残らない。これが密経路の `states.local_penalty` の代わりで、パッチを起こし直す
/// たびに [`Patch::hydrate`] が窓の外の分もここから復元する。無いと、置き直した
/// 直後の価値反復が penalty 0 で回って上げた値を戻してしまう。
///
/// Exact fixed-point values support arbitrary integer scan strengths, quality
/// attenuation, and beam halving. Penalties are theta-uniform, so 8 B/cell in
/// two dimensions suffices; no full-map dense states are needed.
pub(super) struct PenaltyOverlay {
    values: Vec<u64>,
    nx: i32,
    ny: i32,
}

impl PenaltyOverlay {
    pub(super) fn new(nx: i32, ny: i32) -> Self {
        Self { values: vec![0; nx as usize * ny as usize], nx, ny }
    }

    fn idx(&self, ix: i32, iy: i32) -> Option<usize> {
        (ix >= 0 && ix < self.nx && iy >= 0 && iy < self.ny)
            .then(|| iy as usize * self.nx as usize + ix as usize)
    }

    pub(super) fn get(&self, ix: i32, iy: i32) -> u64 {
        self.idx(ix, iy).map_or(0, |i| self.values[i])
    }

    /// Preserve the complete value, including non-power-of-two decay.
    pub(super) fn set(&mut self, ix: i32, iy: i32, v: u64) {
        let Some(i) = self.idx(ix, iy) else { return };
        self.values[i] = v;
    }

    pub(super) fn clear(&mut self) {
        self.values.fill(0);
    }
}

/// compact 経路の全域伝播 (タイル修復) の作業場。密経路では作らない。
///
/// 密経路の `sweep_global` は `states` の上で能動集合を回すが、compact にあるのは
/// 12 B/state の sink だけなので、地図をタイルに切って 1 枚ずつ起こしては掃いて
/// 書き戻す。**ブロック Gauss–Seidel** なので、キューが空になったときの場は密の
/// 伝播と同じ不動点になる (更新式は `value_iteration_at` そのまま)。粒度が
/// 「セル」ではなく「タイル」なだけで、能動集合であることは密と同じ。
///
/// タイル = 更新する interior (`interior` セル角) + 凍結する halo (`halo` = 遷移が
/// 届く距離 `reach`)。halo があるので interior のセルの遷移先はすべてタイル内に
/// 収まり、外は読むだけで済む。
///
/// キューは FIFO。狭域の窓は毎 tick 自分のタイルを汚し得るので、LIFO にすると
/// 外向きの伝播が窓に食われる。
pub(super) struct Repair {
    /// タイル 1 枚ぶんの密な場。地図はハイドレートで丸ごと上書きするので、
    /// ここで確定しているのは幾何と遷移表だけ。
    pub(super) vi: ValueIterator,
    /// interior の 1 辺 [セル]。タイル格子の刻みでもある。
    pub(super) interior: i32,
    /// 凍結境界の厚み [セル] = 遷移が届く最大セル数。
    pub(super) halo: i32,
    /// タイル格子の大きさ。
    pub(super) tnx: i32,
    pub(super) tny: i32,
    /// タイルが待ち行列に入っているか (二重投入よけ)。
    pub(super) queued: Vec<bool>,
    pub(super) queue: VecDeque<u32>,
    /// いまの伝播で消費したタイル訪問数 (キューが空になるとリセット)。
    pub(super) visits: usize,
    /// 直近の伝播 1 回ぶんの訪問数 (`visits` をリセットする前に写す)。掃きに
    /// 実際どれだけ働いたかを呼び出し側がログに出すためだけの値 — compact 側の
    /// 「1 掃き」は地図の大きさでは決まらないので、経過時間だけでは読めない。
    pub(super) last_visits: usize,
}

impl Repair {
    /// タイル番号 → interior のグローバルセル範囲 `(x0, x1, y0, y1)` (両端含む)。
    pub(super) fn interior_of(&self, t: u32, gnx: i32, gny: i32) -> (i32, i32, i32, i32) {
        let (tx, ty) = ((t as i32) % self.tnx, (t as i32) / self.tnx);
        let x0 = tx * self.interior;
        let y0 = ty * self.interior;
        (x0, (x0 + self.interior - 1).min(gnx - 1), y0, (y0 + self.interior - 1).min(gny - 1))
    }

    /// グローバル矩形 `(x0, x1, y0, y1)` から `halo` セル以内にある interior を
    /// 持つタイルをすべて投入する。値が動いたセルの**上流**は必ずこの範囲にいる
    /// (上流はチェビシェフ距離 `reach` = `halo` 以内) ので、これで取りこぼさない。
    pub(super) fn enqueue_around(&mut self, (x0, x1, y0, y1): (i32, i32, i32, i32)) {
        let h = self.halo;
        let tx0 = ((x0 - h).max(0) / self.interior).min(self.tnx - 1);
        let tx1 = ((x1 + h).max(0) / self.interior).min(self.tnx - 1);
        let ty0 = ((y0 - h).max(0) / self.interior).min(self.tny - 1);
        let ty1 = ((y1 + h).max(0) / self.interior).min(self.tny - 1);
        for ty in ty0..=ty1 {
            for tx in tx0..=tx1 {
                let t = (ty * self.tnx + tx) as u32;
                if !self.queued[t as usize] {
                    self.queued[t as usize] = true;
                    self.queue.push_back(t);
                }
            }
        }
    }

    /// 地図を丸ごと待ち行列へ入れる。早期走り出し ([`super::PlanConfig::early_start`])
    /// で未確定のまま走り出した場を背景で解き切るための起点で、そこでしか呼ばない。
    ///
    /// 通常の伝播 ([`Self::enqueue_around`]) は「値が動いた範囲」から広げるが、
    /// 解き終わっていない場では未確定域が地図のどこにでもあるので起点が要る。
    /// 確定済みのタイルは 1 パスで Δ=0 になって抜けるだけなので、余分な仕事は
    /// 地図 1 周ぶんに収まる。
    pub(super) fn enqueue_all(&mut self) {
        for t in 0..(self.tnx * self.tny) as u32 {
            if !self.queued[t as usize] {
                self.queued[t as usize] = true;
                self.queue.push_back(t);
            }
        }
    }

    /// 伝播 1 回ぶんの終わり。訪問数はログ用に写してから畳む。
    pub(super) fn settle(&mut self) {
        self.last_visits = self.visits;
        self.visits = 0;
    }

    pub(super) fn clear(&mut self) {
        self.queue.clear();
        self.queued.fill(false);
        self.settle();
    }
}

/// 遷移表が届く最大セル数 (x/y のチェビシェフ距離)。遷移は解像度と θ 数だけで
/// 決まるので、パッチの凍結境界が成り立つかはこの実測値で判定できる。
pub(super) fn transition_reach(vi: &ValueIterator) -> i32 {
    vi.actions
        .iter()
        .flat_map(|a| a.state_transitions.iter().flatten())
        .map(|t| t.dix.abs().max(t.diy.abs()))
        .max()
        .unwrap_or(0)
}

/// compact 経路の追従用パッチを 1 つ作る (ゴール非依存)。
///
/// 中身はまだ空 (`at: None`)。寸法は「ローカルウィンドウ半径 + 遷移到達距離 +
/// 動ける余裕 (= ウィンドウ半径ぶん)」で、余裕を使い切ったら `hydrate` で置き直す。
pub(super) fn new_patch(build: &BuildParams) -> Result<Patch, PlanError> {
    let res = build.grid.resolution;
    if res <= 0.0 {
        return Err(PlanError::Patch(format!("planner grid resolution is {res}")));
    }
    // ウィンドウの外接半径。**`set_local_shape` 後の `local_ixy_range` と同じ式**
    // (下の不変条件検査がその値と `half` を突き合わせるので、別々に計算しては
    // いけない — 楕円/カプセルの変形ぶんだけ食い違い、凍結境界がウィンドウの
    // 内側に入る)。
    let win = build.local_shape.bbox_cells(res);
    // 遷移の x/y 変位の上界。`cell_delta` はセル内オフセット (0..res) を足してから
    // floor するので、正側は floor(|fw|/res)、負側は -floor(|fw|/res)-1 まで届く。
    let max_fw = build.actions.iter().map(|a| a.delta_fw.abs()).fold(0.0f64, f64::max);
    let reach_bound = (max_fw / res).floor() as i32 + 1;
    let half = 2 * win + reach_bound + build.patch_slack_cells;
    let side = 2 * half + 1;

    let mut vi = ValueIteratorLocal::new(build.actions.clone(), 1);
    // 中身は hydrate が全部上書きするので、地図は全 free のダミーでよい
    // (ここで確定するのは幾何と遷移表と sweep_orders)。
    let dummy = OccupancyGrid {
        width: side,
        height: side,
        resolution: res,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: build.grid.origin_quat.clone(),
        data: vec![0i8; (side as usize) * (side as usize)],
    };
    vi.set_map_with_occupancy_grid(
        &dummy,
        build.theta_cell_num,
        build.map_penalty_radius,
        build.map_penalty,
        build.goal_margin_radius,
        build.goal_margin_theta,
    );
    vi.set_local_shape(build.local_shape);

    // 凍結境界の成立条件: ウィンドウ内のセルの遷移先がパッチに収まること。
    // 解析上界ではなく遷移表の実測で確かめる (1 セル足りないと、ある方位でだけ
    // ウィンドウ端の値が MAX_COST に見えて NoAction で止まる — 気付きにくい)。
    let reach = transition_reach(&vi.base);
    let win = vi.local_ixy_range;
    if win + reach >= half {
        return Err(PlanError::Patch(format!(
            "window {win} + transition reach {reach} cells does not fit in the follow patch \
             (half = {half} cells) at {res:.3} m/cell; action_forward_m is too large"
        )));
    }
    // パッチは窓の寸法の 2 乗で効く。0.05 m/cell の地図で local_xy_range を 2 m に
    // したり ellipse/capsule で ratio_max を上げたりすると数百 MB に届くので、
    // 黙って確保しない (`win + reach >= half` の門は half を win から作る以上
    // 構造上鳴らない — 気付く手掛かりはこのログだけ)。
    let mb = (side as f64).powi(2) * vi.base.cell_num_t as f64 * 56.0 / (1024.0 * 1024.0);
    eprintln!(
        "vi_planner: follow patch {side}x{side}x{} = {mb:.1} MB          (window {win} + transition reach {reach} cells at {res:.3} m/cell)",
        vi.base.cell_num_t
    );
    Ok(Patch { vi, half, reach, at: None })
}

/// compact 経路の修復タイルを 1 枚作る (ゴール非依存)。`reach` は [`new_patch`] が
/// 遷移表から実測した値。
///
/// interior は `BuildParams::repair_interior_cells` だが `reach` を下回らせない。取りこぼしの
/// 心配は無い ([`Repair::enqueue_around`] はセル単位で `halo` ぶん広げてから
/// タイルに割るので、interior の大きさに依らず上流を覆う)。halo より薄い interior は
/// ハイドレートした `(interior + 2*halo)²` のうち更新するのが 1/9 未満になって
/// 割に合わないだけ。
pub(super) fn new_repair(build: &BuildParams, reach: i32) -> Result<Repair, PlanError> {
    let halo = reach.max(1);
    let interior = build.repair_interior_cells.max(halo);
    let side = interior + 2 * halo;

    let mut vi = ValueIterator::new(build.actions.clone(), 1);
    // 中身は hydrate_states が全部上書きするので、地図は全 free のダミーでよい
    // (ここで確定するのは幾何と遷移表)。
    let dummy = OccupancyGrid {
        width: side,
        height: side,
        resolution: build.grid.resolution,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: build.grid.origin_quat.clone(),
        data: vec![0i8; (side as usize) * (side as usize)],
    };
    vi.set_map_with_occupancy_grid(
        &dummy,
        build.theta_cell_num,
        build.map_penalty_radius,
        build.map_penalty,
        build.goal_margin_radius,
        build.goal_margin_theta,
    );
    // パッチと同じ幾何なので同じはずだが、halo が凍結境界として足りることは
    // タイル自身の遷移表でも確かめる (1 セル足りないと interior の縁だけが
    // MAX_COST を見て、値が静かにずれる)。
    let tile_reach = transition_reach(&vi);
    if tile_reach > halo {
        return Err(PlanError::Patch(format!(
            "repair tile halo {halo} cells is short of the transition reach {tile_reach}"
        )));
    }

    let ceil_div = |n: i32| ((n + interior - 1) / interior).max(1);
    let (tnx, tny) = (ceil_div(build.grid.width), ceil_div(build.grid.height));
    let tiles = tnx as usize * tny as usize;
    Ok(Repair {
        vi,
        interior,
        halo,
        tnx,
        tny,
        queued: vec![false; tiles],
        queue: VecDeque::new(),
        visits: 0,
        last_visits: 0,
    })
}

/// compact の場と静的地図から、左下がグローバルセル `p0` の密な矩形を起こす。
/// 大きさは `base` の `cell_num_x/y` で決まる (追従パッチと修復タイルで共用)。
///
/// `overlay` からは観測済みの `local_penalty` を**矩形全体に**戻す。中だけでは
/// 足りない: `action_cost` は遷移先の `local_penalty` を読むので、凍結境界側が 0 の
/// ままだと縁の行動コストが sink の値と食い違う。
pub(super) fn hydrate_states(
    base: &mut ValueIterator,
    f: &CompactField,
    build: &BuildParams,
    overlay: Option<&PenaltyOverlay>,
    p0: (i32, i32),
) {
    let (gnx, gny, nt) = f.cell_num;
    let res = f.resolution;
    let (sx, sy) = (base.cell_num_x, base.cell_num_y);
    let margin = (build.map_penalty_radius / res).ceil() as i32;

    base.map_origin_x = f.origin.0 + p0.0 as f64 * res;
    base.map_origin_y = f.origin.1 + p0.1 as f64 * res;
    base.map_origin_quat = f.origin_quat.clone();
    // ゴールは sink 側の規約 (value == 0) で判定するので final_state の再計算は
    // 要らないが、幾何の一貫性のために持たせておく。
    base.goal_x = f.goal.0;
    base.goal_y = f.goal.1;
    base.goal_t = f.goal.2;

    for py in 0..sy {
        let gy = p0.1 + py;
        for px in 0..sx {
            let gx = p0.0 + px;
            let inside = gx >= 0 && gx < gnx && gy >= 0 && gy < gny;
            // free / penalty は**グローバル座標・グローバル幅**で評価する。
            // `State::from_occupancy` の margin ループは行跨ぎバグを持つので、
            // 切り出してから評価すると compact solve が見た値とズレる。
            let proto = if inside {
                let pen = build.map_penalty;
                State::from_occupancy(gx, gy, 0, &build.grid, margin, pen, gnx)
            } else {
                // 地図外。`from_occupancy` の非 free 早期リターンと同じ形。
                State {
                    total_cost: MAX_COST,
                    penalty: PROB_BASE,
                    local_penalty: 0,
                    ix: 0,
                    iy: 0,
                    it: 0,
                    free: false,
                    final_state: false,
                    optimal_action: None,
                }
            };
            let orig0 = if inside { Some(f.orig(gx, gy, 0)) } else { None };
            let pen = overlay.map(|o| o.get(gx, gy)).unwrap_or(0);
            for it in 0..nt {
                let (v, a) = match orig0 {
                    Some(o) => f.sink.read(o + it as usize),
                    None => (MAX_COST, -1),
                };
                let idx = base.to_index(px, py, it) as usize;
                let s = &mut base.states[idx];
                s.ix = px;
                s.iy = py;
                s.it = it;
                s.free = proto.free;
                s.penalty = proto.penalty;
                s.local_penalty = pen;
                s.total_cost = v;
                s.optimal_action = if a >= 0 { Some(a as usize) } else { None };
                // sink の規約: 未到達 = MAX_COST、ゴール圏 = 0 (`CompactPolicy::is_final`
                // と同じ判定)。`value_iteration_raw` は final_state を更新しないので、
                // ゴール圏はローカル精密化でも 0 に留まる。
                s.final_state = v == 0;
            }
        }
    }
}

/// 起こした矩形の一部 (局所座標 `[x0..=x1] x [y0..=y1]`、左下が `p0`) を sink へ
/// 書き戻す。**実際に変わった列だけ**書き、変わったセルのグローバル座標の外接矩形を
/// 返す (何も変わっていなければ None)。
///
/// 変わっていない列を飛ばすのは mmap sink のためで、書けばそのままページの汚しに
/// なる。落ち着いた窓は 1 ページも汚さない。
fn commit_states(
    f: &mut CompactField,
    base: &ValueIterator,
    p0: (i32, i32),
    (x0, x1, y0, y1): (i32, i32, i32, i32),
) -> Option<(i32, i32, i32, i32)> {
    let (gnx, gny, nt) = f.cell_num;
    let mut values = vec![0u64; nt as usize];
    let mut actions = vec![-1i32; nt as usize];
    let mut bbox: Option<(i32, i32, i32, i32)> = None;
    for py in y0..=y1 {
        let gy = p0.1 + py;
        if gy < 0 || gy >= gny {
            continue;
        }
        for px in x0..=x1 {
            let gx = p0.0 + px;
            if gx < 0 || gx >= gnx {
                continue;
            }
            let orig = f.orig(gx, gy, 0);
            let mut changed = false;
            for it in 0..nt {
                let s = &base.states[base.to_index(px, py, it) as usize];
                // sink の action はインデックス (`hydrate_states` / `CompactPolicy` と
                // 同じ規約。`Action::id` ではない)。
                let a = s.optimal_action.map(|ai| ai as i32).unwrap_or(-1);
                values[it as usize] = s.total_cost;
                actions[it as usize] = a;
                changed |= f.sink.read(orig + it as usize) != (s.total_cost, a);
            }
            if changed {
                f.sink.write_column(orig, &values, &actions);
                bbox = Some(match bbox {
                    None => (gx, gx, gy, gy),
                    Some((bx0, bx1, by0, by1)) => {
                        (bx0.min(gx), bx1.max(gx), by0.min(gy), by1.max(gy))
                    }
                });
            }
        }
    }
    bbox
}

/// compact 出力 sink を作る。`dir` 指定時はディスク mmap、無指定は RAM。
///
/// `gen` があるときは `dir` 直下ではなく `dir/gen<N>` に置き、その場を捨てた
/// ときにディレクトリごと消す ([`MmapSink::new_owned`])。**先読み
/// ([`super::Prefetcher`]) を入れると場が同時に 2 つ以上生きる**ので、固定
/// ファイル名のままだと後から解くほうが `truncate` で先の場のファイルを潰す
/// (mmap したまま長さが変わるので、読めばゼロか SIGBUS)。カウンタは先読み側の
/// 核と共有していて、どの solve も自分だけのディレクトリを取る。
///
/// 先読みを使わないときは `gen` が None で、置き場は従来どおり `dir` 直下。
fn make_sink(
    nstates: usize,
    dir: &SinkDir,
    gen: &SinkGen,
) -> Result<Box<dyn CompactSink + Send>, PlanError> {
    let Some(dir) = dir else { return Ok(Box::new(RamSink::new(nstates))) };
    let boxed = |s| Box::new(s) as Box<dyn CompactSink + Send>;
    let Some(counter) = gen else {
        return MmapSink::new(dir, nstates)
            .map(boxed)
            .map_err(|e| PlanError::Sink(e.to_string()));
    };
    let n = counter.fetch_add(1, Ordering::Relaxed);
    if n == 0 {
        // 前回の実行が落ちて残した世代を片付ける。この時点で世代を持っている
        // 場はまだ 1 つも無いので、生きている sink を消す心配はない。
        remove_stale_generations(dir);
    }
    MmapSink::new_owned(&dir.join(format!("gen{n}")), nstates)
        .map(boxed)
        .map_err(|e| PlanError::Sink(e.to_string()))
}

/// `dir` の下に残っている `gen*` を消す (失敗は無視 — 消せなくても解ける)。
fn remove_stale_generations(dir: &std::path::Path) {
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    for e in entries.flatten() {
        if e.file_name().to_string_lossy().starts_with("gen") {
            let _ = std::fs::remove_dir_all(e.path());
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────
// PlannerCore のうち compact でしか走らないもの
//
// 呼ぶのは [`super`] 側の分岐 (`prepare_goal_with_progress` / `observe_scan` /
// `refine_for` / `sweep_global`)。密経路ではどれも通らない。
// ──────────────────────────────────────────────────────────────────────────

impl PlannerCore {
    /// compact (アウトオブコア) 経路: `states` を作らず地図とゴールから直接解き、
    /// 確定出力を sink に置く。進行制御は密経路と同じ [`SolveDirector`] 1 つ —
    /// 境界はバンド finalize ごとで、そこで cancel 観測・途中経過 (`on_chunk`、sink
    /// ビューの `&dyn PolicyView`)・早期走り出し ([`super::PlanConfig::early_start`])
    /// が行われる (`cancel` はソルバ内部のラウンド境界でも従来どおり観測される)。
    ///
    /// 早期走り出しで**返した時点の sink はそのまま使える** — finalize は値の昇順に
    /// 進むので、載っている列は最後まで解いたときと同じ値で、未 finalize の列が
    /// `MAX_COST` のまま残っているだけ。したがって「経路が引けた」= その経路上の
    /// 列は全部確定済み、になる。残りは走りながら [`Repair`] が同じ不動点まで
    /// 詰める (待ち行列の起点は `prepare_goal_with_progress` の [`Repair::enqueue_all`])。
    pub(super) fn solve_compact(
        &self,
        goal: &PoseView,
        goal_t_deg: i32,
        from: Option<PoseView>,
        stats: &mut SolveStats,
        cancel: &AtomicBool,
        on_chunk: &mut dyn FnMut(&dyn PolicyView),
    ) -> Result<Field, PlanError> {
        let g = &self.build.grid;
        let nt = self.build.theta_cell_num;
        let nstates = g.width as usize * g.height as usize * nt as usize;
        let mut sink =
            make_sink(nstates, &self.cfg.compact_sink_dir, &self.cfg.compact_sink_gen)?;
        let nthreads =
            if self.cfg.vi_threads > 0 { self.cfg.vi_threads } else { default_threads() };

        let mut director =
            SolveDirector { cfg: &self.cfg, cancel, from, on_progress: on_chunk };
        let s = solve_compact_mapped_observed(
            self.build.actions.clone(),
            1,
            g,
            nt,
            self.build.map_penalty_radius,
            self.build.map_penalty,
            self.build.goal_margin_radius,
            self.build.goal_margin_theta,
            goal.x,
            goal.y,
            goal_t_deg,
            self.cfg.max_solve_iter,
            None,
            sink.as_mut(),
            nthreads,
            cancel,
            &mut director,
        );
        stats.iters = s.iters;
        stats.partial = s.stopped;
        if s.cancelled {
            return Err(PlanError::Cancelled);
        }
        if !s.converged && !s.stopped {
            return Err(PlanError::NotConverged);
        }
        Ok(Field::Compact(Box::new(CompactField {
            sink,
            actions: self.build.actions.clone(),
            cell_num: (g.width, g.height, nt),
            resolution: g.resolution,
            origin: (g.origin_x, g.origin_y),
            origin_quat: g.origin_quat.clone(),
            goal: (goal.x, goal.y, goal_t_deg),
        })))
    }

    /// compact 経路: いま窓に入っている `local_penalty` を全域表へ写す
    /// (`set_local_cost` はヒット帯へ書くのも減衰させるのも窓の中だけなので、
    /// 変化はすべてこの範囲に収まる)。密経路では何もしない。
    pub(super) fn harvest_penalties(&mut self) {
        let (Some(p), Some(ov)) = (self.patch.as_ref(), self.penalty.as_mut()) else { return };
        let Some((p0x, p0y)) = p.at else { return };
        let vi = &p.vi;
        for py in vi.local_iy_min..=vi.local_iy_max {
            for px in vi.local_ix_min..=vi.local_ix_max {
                let idx = vi.base.to_index(px, py, 0) as usize;
                ov.set(p0x + px, p0y + py, vi.base.states[idx].local_penalty);
            }
        }
    }

    /// compact 経路: 狭域が動かした窓の `(value, policy)` を sink へ書き戻す。
    ///
    /// **これが compact 版の「共有場」の実体**。sink は全域ぶんの配列なので、
    /// 書き戻せば広域のロールアウト (`plan_*` は sink を読む) からも、次に同じ
    /// 区画を起こし直したパッチからも見える。書き戻すのは窓の中だけ — パッチの
    /// 外周は凍結境界で、そもそも更新していない。
    ///
    /// 実際に変わった列だけ書く。値が動かない tick のほうが多く、mmap sink では
    /// 書き込みがそのままページの汚しになるため (0.25 m/cell の窓 9x9x60 で
    /// 58 KB/tick、10 Hz なら 580 KB/s)。
    ///
    /// 書けた範囲は修復の待ち行列にも入れる。**ここが狭域 → 全域伝播の入口**で、
    /// 「窓が動いた」以外に伝播の起点は無い。
    pub(super) fn commit_window(&mut self) {
        let patch = self.patch.as_ref();
        let mut repair = self.repair.as_mut();
        let Some(c) = self.cached.as_mut() else { return };
        let Field::Compact(f) = &mut c.field else { return };
        let Some(p) = patch else { return };
        let Some(p0) = p.at else { return };
        let win = (
            p.vi.local_ix_min,
            p.vi.local_ix_max,
            p.vi.local_iy_min,
            p.vi.local_iy_max,
        );
        for y in win.2..=win.3 {
            for x in win.0..=win.1 {
                if !p.vi.in_local_area(x, y) {
                    continue;
                }
                if let Some(bbox) = commit_states(f, &p.vi.base, p0, (x, x, y, y)) {
                    if let Some(r) = repair.as_deref_mut() {
                        r.enqueue_around(bbox);
                        self.dirty = true;
                    }
                }
            }
        }
    }

}
