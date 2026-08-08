//! 解けた値関数を「連続に読む」制御層 — V̂ 三線形補間 + DWA 型軌道サンプリング。
//!
//! ソルバ・方策の意味論 (bit-exact) には一切触れない読み取り専用の層。solve 済みの
//! [`PolicyView`] から cmd_vel 相当の連続行動 `(v [m/s], ω [deg/s])` を引く。
//! 大域最適性は V̂ (終端コスト) が、滑らかさは連続候補集合が担保する二層構成で、
//! 6 離散行動の argmin (`vi_planner` の `decide`) の置き換え候補。
//!
//! - [`v_hat`] — (x,y) 双線形 × θ 周期線形の三線形補間。8 頂点に未到達/障害物
//!   (`MAX_COST`) が混ざる点は「評価不能 = `None`」(障害物際で安全側に倒す)。
//! - [`dwa_decide`] — (v, ω) 候補格子をユニサイクルモデルで `horizon_s` 前進
//!   シミュレーションし、**終端 V̂ 最小** (ゴール圏到達候補は最優先・早い到達優先)
//!   の候補を返す。占有セルに触れる候補・終端 V̂ が評価不能な候補は棄却。全滅
//!   なら `None` (呼び出し側が従来の離散 greedy へフォールバックする)。
//! - [`mppi_decide`] — MPPI 型: 前 tick の名目制御列 (warm start) の周りに
//!   ガウス摂動した制御**列**を `n_samples` 本ロールアウトし、softmax 重み付き
//!   平均で名目列を更新して先頭を実行する。DWA が定数 (v, ω) の弧しか描けない
//!   のに対し、ホライズン内で舵を切り替える軌道 (S 字・減速→旋回) を表現できる。
//!   評価規約は DWA と同一 (衝突棄却 + 終端 V̂ のみ)。乱数は決定的
//!   (xorshift64* + Box–Muller、状態は [`MppiState`])。
//! - 実行系は 1 tick (`tick_s`) ごとに再決定する前提 (follow ループと同じ)。
//!
//! 評価に走行中の penalty 積算項を**入れない**のは意図的: V̂ は margin penalty を
//! 含む将来コストを既に符号化しており (VI は action の着地セルにだけ penalty を
//! 課すので、0.3 m の 1 手で margin 帯を跳び越える経路は VI 会計では無料)、走行中
//! の滞在を tick 課金で再徴収すると V̂ が推す経路を DWA が拒否して帯の縁で膠着
//! する (壁角テストで実測)。安全 (占有セル非侵入) は衝突棄却が担う。margin 帯の
//! 一時的な通過露出は本家 greedy (方策追従) も同様に持つ性質。

use crate::action::Action;
use crate::params::MAX_COST;
use crate::planner::PolicyView;
use crate::value_iterator::ValueIterator;

/// 衝突・ペナルティ判定に使う地図コスト面。`free`/`penalty` は θ 非依存なので
/// 2 次元 (ix, iy) で読む。compact 経路ではパッチ/静的地図から実装できる。
pub trait CostView {
    /// (ix, iy) が free (非占有) か。範囲外は false。
    fn free_at(&self, ix: i32, iy: i32) -> bool;
    /// (ix, iy) の penalty + local_penalty (18bit 固定小数点)。範囲外は 0
    /// (`free_at` が先に落とす前提)。
    fn penalty_at(&self, ix: i32, iy: i32) -> u64;
}

impl CostView for ValueIterator {
    fn free_at(&self, ix: i32, iy: i32) -> bool {
        self.in_map_area(ix, iy) && self.states[self.to_index(ix, iy, 0) as usize].free
    }
    fn penalty_at(&self, ix: i32, iy: i32) -> u64 {
        if !self.in_map_area(ix, iy) {
            return 0;
        }
        // free/penalty は θ 非依存 (State::from_occupancy は (x,y) だけで決まる) ので
        // θ=0 の State を読む。
        let s = &self.states[self.to_index(ix, iy, 0) as usize];
        s.penalty + s.local_penalty
    }
}

/// V̂ — 値関数の三線形補間 (x,y 双線形 × θ 周期線形)。
///
/// セル中心 (`origin + (i+0.5)·res`、θ は `(it+0.5)·t_res` 度) を格子点として
/// 補間する。8 頂点のどれかが `MAX_COST` 以上 (未到達・障害物・地図外) なら
/// `None`。ゴール圏は値 0 なので自然に引き込む。
pub fn v_hat(p: &dyn PolicyView, x: f64, y: f64, yaw_rad: f64) -> Option<f64> {
    let (ox, oy) = p.map_origin();
    let res = p.xy_resolution();
    let nt = p.cell_num().2;
    let t_res = p.t_resolution();

    let ux = (x - ox) / res - 0.5;
    let uy = (y - oy) / res - 0.5;
    let fxf = ux.floor();
    let fyf = uy.floor();
    let (ix0, fx) = (fxf as i32, ux - fxf);
    let (iy0, fy) = (fyf as i32, uy - fyf);

    let deg = normalize_deg(yaw_rad.to_degrees());
    let ut = deg / t_res - 0.5;
    let ftf = ut.floor();
    let ft = ut - ftf;
    let it0 = (((ftf as i32) % nt) + nt) % nt;
    let it1 = (it0 + 1) % nt;

    let mut acc = 0.0f64;
    for (dix, wx) in [(0, 1.0 - fx), (1, fx)] {
        for (diy, wy) in [(0, 1.0 - fy), (1, fy)] {
            for (it, wt) in [(it0, 1.0 - ft), (it1, ft)] {
                let v = p.value_at(ix0 + dix, iy0 + diy, it);
                if v >= MAX_COST {
                    return None;
                }
                acc += (v as f64) * wx * wy * wt;
            }
        }
    }
    Some(acc)
}

/// DWA 型サンプリングの設定。
#[derive(Clone, Debug)]
pub struct DwaConfig {
    /// 並進速度の候補範囲 [m/s] (負 = 後退許可)。
    pub v_min: f64,
    pub v_max: f64,
    /// 角速度の候補範囲 ±w_max_deg [deg/s]。
    pub w_max_deg: f64,
    /// v 候補数 (`v_min..=v_max` 等間隔)。
    pub n_v: usize,
    /// ω 候補数 (`-w_max..=w_max` 等間隔、奇数なら 0 を含む)。
    pub n_w: usize,
    /// 前進シミュレーション時間 [s]。
    pub horizon_s: f64,
    /// 制御周期 [s]。penalty の「1 action」換算単位。
    pub tick_s: f64,
    /// 衝突判定の弧長サンプリング間隔 [m]。0 = auto (セル解像度の半分)。
    pub collide_step_m: f64,
}

impl DwaConfig {
    /// 行動集合から速度範囲を取る (v = delta_fw [m/s]、ω = delta_rot [deg/s] —
    /// 本家 `decision` の cmd_vel 変換と同じ読み方)。候補数とホライズンは既定値。
    pub fn from_actions(actions: &[Action], tick_s: f64) -> Self {
        let v_max = actions.iter().map(|a| a.delta_fw).fold(0.0f64, f64::max);
        let v_min = actions.iter().map(|a| a.delta_fw).fold(0.0f64, f64::min);
        let w_max_deg = actions.iter().map(|a| a.delta_rot.abs()).fold(0.0f64, f64::max);
        Self {
            v_min,
            v_max,
            w_max_deg,
            n_v: 7,
            n_w: 11,
            horizon_s: 1.0,
            tick_s,
            collide_step_m: 0.0,
        }
    }
}

/// [`dwa_decide`] の選択結果。
#[derive(Clone, Copy, Debug)]
pub struct DwaChoice {
    /// 並進速度 [m/s]。
    pub v: f64,
    /// 角速度 [deg/s]。
    pub w_deg: f64,
    /// 評価値。非ゴール候補は終端 V̂ (18bit 固定小数点、≥0)。ゴール到達候補は
    /// `到達時刻 [s] − GOAL_BONUS` (負) — 常に非ゴール候補より優先され、
    /// ゴール候補同士は早く着く方が勝つ。
    pub cost: f64,
    /// ホライズン内にゴール圏 (`final_state`) へ入る候補か。
    pub hits_goal: bool,
}

/// ゴール到達候補を非ゴール候補より必ず優先させる番兵 (V̂ ≥ 0 なのでこれで足りる)。
const GOAL_BONUS: f64 = 1e18;

/// ユニサイクルモデルの 1 ステップ (定速 (v, ω) の閉形式弧積分)。
pub fn unicycle_step(x: f64, y: f64, yaw_rad: f64, v: f64, w_rad_s: f64, dt: f64) -> (f64, f64, f64) {
    if w_rad_s.abs() < 1e-9 {
        (x + v * dt * yaw_rad.cos(), y + v * dt * yaw_rad.sin(), yaw_rad)
    } else {
        let r = v / w_rad_s;
        let yaw2 = yaw_rad + w_rad_s * dt;
        (x + r * (yaw2.sin() - yaw_rad.sin()), y - r * (yaw2.cos() - yaw_rad.cos()), yaw2)
    }
}

/// (v, ω) 候補格子を評価して最良の連続行動を返す。
///
/// 現在セルが free でない、または全候補が棄却された (衝突・終端評価不能) 場合は
/// `None` — 呼び出し側は従来の離散 greedy (`decide` 相当) へフォールバックすること。
pub fn dwa_decide(
    p: &dyn PolicyView,
    cost: &dyn CostView,
    cfg: &DwaConfig,
    x: f64,
    y: f64,
    yaw_rad: f64,
) -> Option<DwaChoice> {
    let (ox, oy) = p.map_origin();
    let res = p.xy_resolution();
    let cix = ((x - ox) / res).floor() as i32;
    let ciy = ((y - oy) / res).floor() as i32;
    if !cost.free_at(cix, ciy) {
        return None;
    }
    let step = if cfg.collide_step_m > 0.0 { cfg.collide_step_m } else { res * 0.5 };

    let mut best: Option<DwaChoice> = None;
    for i in 0..cfg.n_v.max(1) {
        let rv = if cfg.n_v > 1 { i as f64 / (cfg.n_v - 1) as f64 } else { 1.0 };
        let v = cfg.v_min + (cfg.v_max - cfg.v_min) * rv;
        for j in 0..cfg.n_w.max(1) {
            let rw = if cfg.n_w > 1 { j as f64 / (cfg.n_w - 1) as f64 } else { 0.5 };
            let w_deg = -cfg.w_max_deg + 2.0 * cfg.w_max_deg * rw;
            let Some(c) = eval_candidate(p, cost, cfg, x, y, yaw_rad, v, w_deg, step) else {
                continue;
            };
            if best.map(|b| c.cost < b.cost).unwrap_or(true) {
                best = Some(c);
            }
        }
    }
    best
}

/// 1 候補 (v, ω) の前進シミュレーション評価。棄却は `None`。
#[allow(clippy::too_many_arguments)]
fn eval_candidate(
    p: &dyn PolicyView,
    cost: &dyn CostView,
    cfg: &DwaConfig,
    x: f64,
    y: f64,
    yaw_rad: f64,
    v: f64,
    w_deg: f64,
    step_m: f64,
) -> Option<DwaChoice> {
    let (ox, oy) = p.map_origin();
    let res = p.xy_resolution();
    // 時間刻み: 弧長 step_m を超えず、かつ tick_s 以下 (回転のみでもゴール判定を刻む)。
    let dt = if v.abs() > 1e-9 { (step_m / v.abs()).min(cfg.tick_s) } else { cfg.tick_s };
    let n = (cfg.horizon_s / dt).ceil().max(1.0) as usize;
    let w_rad = w_deg.to_radians();

    let (mut px, mut py, mut pyaw) = (x, y, yaw_rad);
    for k in 0..n {
        let (nx2, ny2, nyaw) = unicycle_step(px, py, pyaw, v, w_rad, dt);
        px = nx2;
        py = ny2;
        pyaw = nyaw;
        let ix = ((px - ox) / res).floor() as i32;
        let iy = ((py - oy) / res).floor() as i32;
        if !cost.free_at(ix, iy) {
            return None; // 地図外 or 占有セル。
        }
        if p.is_final(ix, iy, theta_cell(p, pyaw)) {
            // ゴール圏に入れる候補: そこで停まれる。早い到達ほど低コスト。
            let t_hit = (k + 1) as f64 * dt;
            return Some(DwaChoice { v, w_deg, cost: t_hit - GOAL_BONUS, hits_goal: true });
        }
    }
    let vh = v_hat(p, px, py, pyaw)?;
    Some(DwaChoice { v, w_deg, cost: vh, hits_goal: false })
}

/// MPPI 型サンプリングの設定。
#[derive(Clone, Debug)]
pub struct MppiConfig {
    /// 並進速度のクランプ範囲 [m/s] (負 = 後退許可)。
    pub v_min: f64,
    pub v_max: f64,
    /// 角速度のクランプ範囲 ±w_max_deg [deg/s]。
    pub w_max_deg: f64,
    /// サンプル本数 K (名目列そのもの = 無摂動サンプルを 1 本含む)。
    pub n_samples: usize,
    /// 前進シミュレーション時間 [s]。ホライズン長 = `ceil(horizon_s / tick_s)` 手。
    pub horizon_s: f64,
    /// 制御周期 [s] (1 手の長さ)。
    pub tick_s: f64,
    /// softmax 温度 (無次元)。重みは `exp(-(dev_k + pen_k) / λ)` — `dev_k` は
    /// 地図コストの偏差を偏差平均で正規化した無次元量 (V̂ の 18bit 固定小数点
    /// スケールに依存しない)、`pen_k` は制御逸脱ペナルティ (`gamma` 参照)。
    /// 小さいほど argmin (DWA 相当)、大きいほど平均化 — 平均化はノイズを相殺
    /// するので実行指令はむしろ滑らかになる。
    pub lambda: f64,
    /// 制御ノイズの標準偏差 (1 手あたり)。
    pub sigma_v: f64,
    pub sigma_w_deg: f64,
    /// ノイズの時間相関 (0 = 白色、1 に近いほどホライズン内で滑らかな摂動)。
    /// `ε_j = α ε_{j-1} + √(1-α²) σ ξ_j` — 白色だと重み付き平均の先頭要素に
    /// ノイズが漏れて実行指令が tick ごとに震える (実測で Σ|Δω| が 1 桁悪化)。
    pub alpha: f64,
    /// 制御逸脱ペナルティの重み (無次元)。`pen_k = γ/H · Σ_j (ε_v/σ_v)² +
    /// (ε_ω/σ_ω)²`。V̂ は行動の滑らかさを符号化しないので、これは地図コストの
    /// 二重課金にはならない (走行 penalty を入れない設計はそのまま)。名目列
    /// (ε=0) が同等コストの摂動列より必ず重くなり、実行指令のドリフトを抑える。
    pub gamma: f64,
    /// 衝突判定の弧長サンプリング間隔 [m]。0 = auto (セル解像度の半分)。
    pub collide_step_m: f64,
    /// 乱数シード ([`MppiState::new`] に渡す既定値)。
    pub seed: u64,
}

impl MppiConfig {
    /// 行動集合から速度範囲を取る (読み方は [`DwaConfig::from_actions`] と同じ)。
    /// ノイズ幅は範囲比で既定する (σ_v = 幅の 1/4、σ_ω = 上限の 1/4)。
    pub fn from_actions(actions: &[Action], tick_s: f64) -> Self {
        let v_max = actions.iter().map(|a| a.delta_fw).fold(0.0f64, f64::max);
        let v_min = actions.iter().map(|a| a.delta_fw).fold(0.0f64, f64::min);
        let w_max_deg = actions.iter().map(|a| a.delta_rot.abs()).fold(0.0f64, f64::max);
        Self {
            v_min,
            v_max,
            w_max_deg,
            n_samples: 256,
            horizon_s: 1.0,
            tick_s,
            lambda: 1.0,
            sigma_v: 0.25 * (v_max - v_min),
            sigma_w_deg: 0.25 * w_max_deg,
            alpha: 0.8,
            gamma: 0.1,
            collide_step_m: 0.0,
            seed: 0x5EED_0BAD_F00D_u64,
        }
    }

    fn steps(&self) -> usize {
        (self.horizon_s / self.tick_s.max(1e-6)).ceil().max(1.0) as usize
    }
}

/// MPPI の tick 間状態 — warm start 用の名目制御列 `(v, ω_deg)` と乱数状態。
/// 呼び出し側が保持して毎 tick 同じものを渡す (共有ロック下で使うなら内部可変性で)。
/// ゴールが替わった等で列が無意味になったら [`reset`](Self::reset) — しなくても
/// 数 tick の再サンプリングで回復する (名目は事前分布でしかない)。
#[derive(Clone, Debug)]
pub struct MppiState {
    nominal: Vec<(f64, f64)>,
    rng: u64,
}

impl MppiState {
    pub fn new(seed: u64) -> Self {
        Self { nominal: Vec::new(), rng: seed | 1 }
    }

    /// 名目列を捨てる (乱数状態は進んだまま)。
    pub fn reset(&mut self) {
        self.nominal.clear();
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// (0, 1] の一様乱数 (Box–Muller の ln 用に 0 を除く)。
    fn next_unit(&mut self) -> f64 {
        ((self.next_u64() >> 11) + 1) as f64 / (1u64 << 53) as f64
    }

    /// 標準正規の対 (Box–Muller)。
    fn next_gauss_pair(&mut self) -> (f64, f64) {
        let r = (-2.0 * self.next_unit().ln()).sqrt();
        let th = 2.0 * std::f64::consts::PI * self.next_unit();
        (r * th.cos(), r * th.sin())
    }
}

/// 名目列の周りをサンプリングして最良の連続行動を返す (MPPI)。
///
/// 返り値の意味は [`dwa_decide`] と同じ ([`DwaChoice`] を共用)。`None` は
/// 現在セルが非 free、または有効サンプル全滅 (全衝突・全終端評価不能) —
/// 呼び出し側は離散 greedy へフォールバックし、`state.reset()` しておくこと
/// が望ましい (その名目列は場に合っていない)。
pub fn mppi_decide(
    p: &dyn PolicyView,
    cost: &dyn CostView,
    cfg: &MppiConfig,
    state: &mut MppiState,
    x: f64,
    y: f64,
    yaw_rad: f64,
) -> Option<DwaChoice> {
    let (ox, oy) = p.map_origin();
    let res = p.xy_resolution();
    let cix = ((x - ox) / res).floor() as i32;
    let ciy = ((y - oy) / res).floor() as i32;
    if !cost.free_at(cix, ciy) {
        return None;
    }
    let step_m = if cfg.collide_step_m > 0.0 { cfg.collide_step_m } else { res * 0.5 };
    let h = cfg.steps();
    // warm start: 前 tick の名目列を 1 手ずらして引き継ぐ (足りない分は末尾複製、
    // 初回は停止列 = 現在地の V̂ を基準に探索を始める)。
    if state.nominal.len() == h {
        state.nominal.rotate_left(1);
        let last = *state.nominal.last().unwrap();
        *state.nominal.last_mut().unwrap() = last;
    } else {
        state.nominal.resize(h, (0.0, 0.0));
    }

    let clamp = |v: f64, w: f64| {
        (v.clamp(cfg.v_min, cfg.v_max), w.clamp(-cfg.w_max_deg, cfg.w_max_deg))
    };

    // サンプル生成 + 評価。k=0 は無摂動の名目列 (前 tick の計画を必ず候補に残す)。
    // ノイズは時間相関 (α) 付き — 白色だと平均後の先頭要素にノイズが漏れて実行
    // 指令が震える。ε の逸脱量から制御ペナルティ (γ、無次元) も同時に積む。
    let base = state.nominal.clone();
    let k = cfg.n_samples.max(2);
    let a = cfg.alpha.clamp(0.0, 0.999);
    let b = (1.0 - a * a).sqrt();
    let mut seqs: Vec<Vec<(f64, f64)>> = Vec::with_capacity(k);
    let mut epss: Vec<Vec<(f64, f64)>> = Vec::with_capacity(k); // 非クランプの摂動 ε
    let mut evals: Vec<(usize, f64, bool, f64)> = Vec::new(); // (idx, 地図コスト, hits_goal, pen)
    for i in 0..k {
        let mut pen = 0.0f64;
        let mut eps: Vec<(f64, f64)> = vec![(0.0, 0.0); h];
        if i > 0 {
            let (mut ev, mut ew) = state.next_gauss_pair();
            ev *= cfg.sigma_v;
            ew *= cfg.sigma_w_deg;
            for e in &mut eps {
                *e = (ev, ew);
                pen += (ev / cfg.sigma_v.max(1e-9)).powi(2)
                    + (ew / cfg.sigma_w_deg.max(1e-9)).powi(2);
                let (gv, gw) = state.next_gauss_pair();
                ev = a * ev + b * cfg.sigma_v * gv;
                ew = a * ew + b * cfg.sigma_w_deg * gw;
            }
        }
        let seq: Vec<(f64, f64)> =
            base.iter().zip(&eps).map(|(&(v, w), &(ev, ew))| clamp(v + ev, w + ew)).collect();
        if let Some((c, hits)) = rollout_seq(p, cost, cfg, x, y, yaw_rad, &seq, step_m) {
            evals.push((i, c, hits, cfg.gamma * pen / h as f64));
        }
        seqs.push(seq);
        epss.push(eps);
    }
    if evals.is_empty() {
        return None;
    }

    // ゴール到達サンプルがあればそれだけを母集団にする (到達は離散事象なので
    // 非到達サンプルと混ぜて平均しない — GOAL_BONUS のスケール混在も避ける)。
    let any_goal = evals.iter().any(|&(_, _, hits, _)| hits);
    let pool: Vec<&(usize, f64, bool, f64)> =
        evals.iter().filter(|&&(_, _, hits, _)| hits == any_goal).collect();

    // softmax 重み: 地図コストは偏差平均で無次元化し、制御ペナルティと足して
    // 温度 λ で焼く (cfg.lambda / cfg.gamma の doc 参照)。
    let smin = pool.iter().map(|e| e.1).fold(f64::INFINITY, f64::min);
    let mean_dev: f64 = pool.iter().map(|e| e.1 - smin).sum::<f64>() / pool.len() as f64;
    let mut du = vec![(0.0f64, 0.0f64); h];
    let mut wsum = 0.0f64;
    for &&(idx, s, _, pen) in &pool {
        let ndev = if mean_dev > 0.0 { (s - smin) / mean_dev } else { 0.0 };
        let w = (-(ndev + pen) / cfg.lambda.max(1e-6)).exp();
        wsum += w;
        for (j, &(ev, ew)) in epss[idx].iter().enumerate() {
            du[j].0 += w * ev;
            du[j].1 += w * ew;
        }
    }
    // ε 空間で平均してから 1 回だけクランプする (クランプ済み制御を平均すると、
    // 最適が範囲端にあるとき — v_max 巡航など — ノイズが内側にしか動けず平均が
    // 端に張り付けない)。
    let mut nominal: Vec<(f64, f64)> = base
        .iter()
        .zip(&du)
        .map(|(&(v, w), &(dv, dw))| clamp(v + dv / wsum, w + dw / wsum))
        .collect();
    // 時間軸の 3 点移動平均 (端は 2 点)。実行されるのは先頭要素だけなので、
    // そこに残るサンプルノイズを隣接手と相殺させる (nav2 MPPI が更新列に
    // Savitzky–Golay を掛けるのと同じ目的の軽量版)。
    if h >= 2 {
        let raw = nominal.clone();
        for (j, u) in nominal.iter_mut().enumerate() {
            let (lo, hi) = (j.saturating_sub(1), (j + 1).min(h - 1));
            let n = (hi - lo + 1) as f64;
            let sv: f64 = raw[lo..=hi].iter().map(|q| q.0).sum();
            let sw: f64 = raw[lo..=hi].iter().map(|q| q.1).sum();
            *u = clamp(sv / n, sw / n);
        }
    }

    // 重み付き平均は非凸な自由空間では衝突し得るので、更新後の名目列自体を
    // 再評価してから実行する。ダメなら最良サンプルに落とす (こちらは検証済み)。
    let (chosen, cost_v, hits) = match rollout_seq(p, cost, cfg, x, y, yaw_rad, &nominal, step_m) {
        Some((c, hits)) => (nominal, c, hits),
        None => {
            let &&(bi, bc, bh, _) = pool
                .iter()
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .unwrap();
            (seqs[bi].clone(), bc, bh)
        }
    };
    let (v0, w0) = chosen[0];
    state.nominal = chosen;
    Some(DwaChoice { v: v0, w_deg: w0, cost: cost_v, hits_goal: hits })
}

/// 制御列 1 本の前進シミュレーション評価 ([`eval_candidate`] の可変制御版)。
/// 棄却 (衝突・終端評価不能) は `None`、それ以外は `(コスト, ゴール到達か)`。
fn rollout_seq(
    p: &dyn PolicyView,
    cost: &dyn CostView,
    cfg: &MppiConfig,
    x: f64,
    y: f64,
    yaw_rad: f64,
    seq: &[(f64, f64)],
    step_m: f64,
) -> Option<(f64, bool)> {
    let (ox, oy) = p.map_origin();
    let res = p.xy_resolution();
    let (mut px, mut py, mut pyaw) = (x, y, yaw_rad);
    let mut t = 0.0f64;
    for &(v, w_deg) in seq {
        let w_rad = w_deg.to_radians();
        // 1 手 (tick_s) を弧長 step_m 以下の小片に割って衝突・ゴールを判定する。
        let m = ((v.abs() * cfg.tick_s / step_m).ceil() as usize).max(1);
        let dt = cfg.tick_s / m as f64;
        for _ in 0..m {
            let (nx2, ny2, nyaw) = unicycle_step(px, py, pyaw, v, w_rad, dt);
            px = nx2;
            py = ny2;
            pyaw = nyaw;
            t += dt;
            let ix = ((px - ox) / res).floor() as i32;
            let iy = ((py - oy) / res).floor() as i32;
            if !cost.free_at(ix, iy) {
                return None;
            }
            if p.is_final(ix, iy, theta_cell(p, pyaw)) {
                return Some((t - GOAL_BONUS, true));
            }
        }
    }
    let vh = v_hat(p, px, py, pyaw)?;
    Some((vh, false))
}

/// yaw [rad] → θ セル (本家 `posToAction` の θ 変換と同じ i32 切り捨て + 正規化)。
fn theta_cell(p: &dyn PolicyView, yaw_rad: f64) -> i32 {
    let t = (180.0 * yaw_rad / std::f64::consts::PI) as i32;
    ((((t + 360 * 100) % 360) as f64) / p.t_resolution()).floor() as i32
}

/// 度を [0, 360) へ正規化。
fn normalize_deg(mut d: f64) -> f64 {
    d %= 360.0;
    if d < 0.0 {
        d += 360.0;
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action::Action;
    use crate::msg::{OccupancyGrid, Quaternion};
    use crate::planner::pose_to_cell;
    use crate::solvers::{solve, U64Solver};

    const RES: f64 = 0.05;
    const NT: i32 = 60;

    fn actions() -> Vec<Action> {
        vec![
            Action::new("forward", 0.3, 0.0, 0),
            Action::new("back", -0.2, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2),
            Action::new("rightfw", 0.2, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4),
            Action::new("leftfw", 0.2, 20.0, 5),
        ]
    }

    fn solved_vi(width: i32, height: i32, data: Vec<i8>, goal: (f64, f64, i32)) -> ValueIterator {
        let grid = OccupancyGrid {
            width,
            height,
            resolution: RES,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data,
        };
        let mut vi = ValueIterator::new(actions(), 1);
        vi.set_map_with_occupancy_grid(&grid, NT, 0.1, 30.0, 0.2, 180);
        vi.set_goal(goal.0, goal.1, goal.2);
        let stats = solve(&mut vi, U64Solver::Frontier3D, 100_000);
        assert!(stats.converged, "test map must converge");
        vi
    }

    fn empty_map(size: i32) -> Vec<i8> {
        vec![0i8; (size * size) as usize]
    }

    /// 前進 1 ステップ (0.3 m = 6 セル) で飛び越えられない厚さの縦壁 (上端だけ開口)。
    fn walled_map(size: i32) -> Vec<i8> {
        let mut data = empty_map(size);
        let wall_x = size / 2;
        for y in 0..(size - 8) {
            for dx in 0..8 {
                data[(y * size + wall_x + dx) as usize] = 100;
            }
        }
        data
    }

    /// DWA を 1 tick ごとに再決定しながらユニサイクルで実行する (テスト用 follow ループ)。
    /// 経路上の全セルが free であることも検証する。
    fn dwa_follow(
        vi: &ValueIterator,
        start: (f64, f64, f64),
        max_ticks: usize,
    ) -> (bool, Vec<(f64, f64, f64)>) {
        use crate::planner::PolicyView as _;
        let cfg = DwaConfig::from_actions(&vi.actions, 0.1);
        let (mut x, mut y, mut yaw) = start;
        let mut traj = vec![start];
        for _ in 0..max_ticks {
            let (ix, iy, it) = pose_to_cell(vi, x, y, yaw);
            assert!(
                CostView::free_at(vi, ix, iy),
                "trajectory entered a non-free cell at ({x:.3}, {y:.3})"
            );
            if vi.is_final(ix, iy, it) {
                return (true, traj);
            }
            let Some(c) = dwa_decide(vi, vi, &cfg, x, y, yaw) else {
                return (false, traj);
            };
            let (nx2, ny2, nyaw) = unicycle_step(x, y, yaw, c.v, c.w_deg.to_radians(), cfg.tick_s);
            x = nx2;
            y = ny2;
            yaw = nyaw;
            traj.push((x, y, yaw));
        }
        (false, traj)
    }

    #[test]
    fn v_hat_matches_cell_value_at_centers() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0));
        use crate::planner::PolicyView as _;
        for (ix, iy, it) in [(10, 10, 5), (30, 20, 0), (5, 40, 59)] {
            let x = (ix as f64 + 0.5) * RES;
            let y = (iy as f64 + 0.5) * RES;
            let yaw = ((it as f64 + 0.5) * vi.t_resolution).to_radians();
            let exact = vi.value_at(ix, iy, it) as f64;
            let interp = v_hat(&vi, x, y, yaw).expect("interior cell must interpolate");
            let tol = exact.abs() * 1e-9 + 1e-3;
            assert!(
                (interp - exact).abs() <= tol,
                "cell ({ix},{iy},{it}): interp {interp} != exact {exact}"
            );
        }
    }

    #[test]
    fn v_hat_is_periodic_in_theta() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0));
        let (x, y) = (1.0, 1.0);
        for yaw in [0.1, 1.0, 3.0, -2.0] {
            let a = v_hat(&vi, x, y, yaw).unwrap();
            let b = v_hat(&vi, x, y, yaw + 2.0 * std::f64::consts::PI).unwrap();
            assert!((a - b).abs() <= a.abs() * 1e-9 + 1e-6, "yaw {yaw}: {a} != {b}");
        }
        // θ ラップ (it0=59, it1=0) を跨ぐ点も評価できること。
        assert!(v_hat(&vi, x, y, 0.5f64.to_radians()).is_some());
    }

    #[test]
    fn v_hat_rejects_points_next_to_unreached_cells() {
        let size = 64;
        let vi = solved_vi(size, size, walled_map(size), (2.8, 0.6, 0));
        // 壁 (x セル 32..40) の左隣 (セル 31) で壁向きの重みが立つ点 → 頂点に
        // 障害物セル (MAX_COST) が入り None。
        let x = 31.8 * RES;
        let y = 1.0;
        assert!(v_hat(&vi, x, y, 0.0).is_none());
        // 壁から離れた点は評価できる。
        assert!(v_hat(&vi, 20.0 * RES, y, 0.0).is_some());
    }

    #[test]
    fn dwa_picks_a_candidate_and_descends_v_hat() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0));
        let cfg = DwaConfig::from_actions(&vi.actions, 0.1);
        let start = (0.6, 0.6, 0.0);
        let c = dwa_decide(&vi, &vi, &cfg, start.0, start.1, start.2).expect("open space");
        assert!(c.cost.is_finite());
        // 50 tick 実行して V̂ が下がっていること (ゴールへ向かう)。
        let (_, traj) = dwa_follow(&vi, start, 50);
        let v0 = v_hat(&vi, start.0, start.1, start.2).unwrap();
        let last = traj.last().unwrap();
        let v1 = v_hat(&vi, last.0, last.1, last.2).unwrap();
        assert!(v1 < v0, "V̂ did not decrease: {v0} -> {v1}");
    }

    #[test]
    fn dwa_follow_reaches_goal_on_empty_map() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0));
        let (reached, traj) = dwa_follow(&vi, (0.6, 0.6, 0.0), 3000);
        assert!(reached, "did not reach goal in 3000 ticks ({} poses)", traj.len());
    }

    #[test]
    fn dwa_follow_avoids_wall_and_reaches_goal() {
        let size = 64;
        let vi = solved_vi(size, size, walled_map(size), (2.8, 0.6, 0));
        // 壁の左側から出発 → 上端の開口を回ってゴールへ。free 検証は dwa_follow 内。
        let (reached, traj) = dwa_follow(&vi, (0.4, 0.4, 0.0), 6000);
        assert!(reached, "did not reach goal in 6000 ticks ({} poses)", traj.len());
    }

    #[test]
    fn dwa_on_non_free_cell_returns_none() {
        let size = 64;
        let vi = solved_vi(size, size, walled_map(size), (2.8, 0.6, 0));
        // 壁セルの中心に置く → 判断不能 (呼び出し側が greedy 救済へ)。
        let cfg = DwaConfig::from_actions(&vi.actions, 0.1);
        let x = 34.5 * RES;
        let y = 1.0;
        assert!(dwa_decide(&vi, &vi, &cfg, x, y, 0.0).is_none());
    }

    /// MPPI を 1 tick ごとに再決定しながらユニサイクルで実行する (dwa_follow の
    /// MPPI 版)。経路上の全セルが free であることも検証する。
    fn mppi_follow(
        vi: &ValueIterator,
        start: (f64, f64, f64),
        max_ticks: usize,
    ) -> (bool, Vec<(f64, f64, f64)>) {
        use crate::planner::PolicyView as _;
        let cfg = MppiConfig::from_actions(&vi.actions, 0.1);
        let mut state = MppiState::new(cfg.seed);
        let (mut x, mut y, mut yaw) = start;
        let mut traj = vec![start];
        for _ in 0..max_ticks {
            let (ix, iy, it) = pose_to_cell(vi, x, y, yaw);
            assert!(
                CostView::free_at(vi, ix, iy),
                "trajectory entered a non-free cell at ({x:.3}, {y:.3})"
            );
            if vi.is_final(ix, iy, it) {
                return (true, traj);
            }
            let Some(c) = mppi_decide(vi, vi, &cfg, &mut state, x, y, yaw) else {
                return (false, traj);
            };
            let (nx2, ny2, nyaw) = unicycle_step(x, y, yaw, c.v, c.w_deg.to_radians(), cfg.tick_s);
            x = nx2;
            y = ny2;
            yaw = nyaw;
            traj.push((x, y, yaw));
        }
        (false, traj)
    }

    #[test]
    fn mppi_follow_reaches_goal_on_empty_map() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0));
        let (reached, traj) = mppi_follow(&vi, (0.6, 0.6, 0.0), 3000);
        assert!(reached, "did not reach goal in 3000 ticks ({} poses)", traj.len());
    }

    #[test]
    fn mppi_follow_avoids_wall_and_reaches_goal() {
        let size = 64;
        let vi = solved_vi(size, size, walled_map(size), (2.8, 0.6, 0));
        let (reached, traj) = mppi_follow(&vi, (0.4, 0.4, 0.0), 6000);
        assert!(reached, "did not reach goal in 6000 ticks ({} poses)", traj.len());
    }

    #[test]
    fn mppi_on_non_free_cell_returns_none() {
        let size = 64;
        let vi = solved_vi(size, size, walled_map(size), (2.8, 0.6, 0));
        let cfg = MppiConfig::from_actions(&vi.actions, 0.1);
        let mut state = MppiState::new(cfg.seed);
        let x = 34.5 * RES;
        let y = 1.0;
        assert!(mppi_decide(&vi, &vi, &cfg, &mut state, x, y, 0.0).is_none());
    }

    #[test]
    fn mppi_is_deterministic() {
        let vi = solved_vi(64, 64, empty_map(64), (2.0, 2.0, 0));
        let cfg = MppiConfig::from_actions(&vi.actions, 0.1);
        // 同一シードの状態 2 つで 20 tick 走らせて指令列が一致すること (再現性)。
        let run = || {
            let mut state = MppiState::new(cfg.seed);
            let (mut x, mut y, mut yaw) = (0.6, 0.6, 0.0);
            let mut cmds = Vec::new();
            for _ in 0..20 {
                let c = mppi_decide(&vi, &vi, &cfg, &mut state, x, y, yaw).unwrap();
                cmds.push((c.v, c.w_deg));
                let (nx2, ny2, nyaw) = unicycle_step(x, y, yaw, c.v, c.w_deg.to_radians(), cfg.tick_s);
                x = nx2;
                y = ny2;
                yaw = nyaw;
            }
            cmds
        };
        assert_eq!(run(), run());
    }
}
