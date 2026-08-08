//! 全地図 belief 推定器 — VIOLA の「belief を VI の状態へ」の推定側。
//!
//! 旧 範囲 belief (localize.rs の GridLocalizer の窓 / AdaptiveLocalizer の
//! 多重解像度レベル) の後継。belief をプランナの [`crate::ValueIterator`] と
//! **同一の格子** (map_scale 適用後の nx×ny×θ) に全域で持つ — 窓の再センタ
//! リングもレベル遷移も無く、推定の状態空間と計画の状態空間が 1:1 に対応する
//! (QMDP・多目標 VI がセル添字のまま繋がる)。尤度場だけは生解像度地図
//! (native) から起こす — スキャン端点の評価は格子を粗くすると壊れるため。
//!
//! 旧実装からの置き換え:
//! - 窓/レベル + expand/contract カスケード → 全域 1 枚 + EMCL 風一様混合
//!   リセット ([`BeliefConfig::reset_quality`])。
//! - ロスト検出 (粗レベル滞在 = pose None) → ESS しきい値
//!   ([`BeliefConfig::lost_ess`]): belief の広がりの実測がしきい値を超えたら
//!   [`Belief::pose`] は `None`。
//! - 毎 tick の全面シフト predict → O(1) の移動量累積 + observe 冒頭の一括
//!   flush (アクティブセルだけの scatter)。全域を毎 tick 舐めない。
//! - viterbi (min-plus / MAP) はロスト時の特殊モードではなく全期間の更新則
//!   (旧 localize/viterbi.rs の doc にある VI Bellman 掃引との同型性は
//!   そのまま)。
//!
//! アクティブ集合: `b[i] > 0` のセル添字を `active` に持ち、predict の flush・
//! 尤度・平均・ESS はすべて active だけを舐める。`weight_skip_ratio` の枝刈りが
//! 集合を belief の広がりに比例した大きさに保つ。

use crate::bridge::PoseView;
use crate::msg::{LaserScan, OccupancyGrid};

/// [`Belief`] のチューニング。実機はスキャナも床も理想モデルからずれるので、
/// ノイズ幅は必ずパラメータで残す。
#[derive(Clone, Copy, Debug)]
pub struct BeliefConfig {
    /// 尤度場のガウス幅 [m] (スキャン端点と最近障害物の距離に対する σ)。
    pub sensor_sigma_m: f64,
    /// 補正に使うビームの間引き (1 = 全ビーム)。
    pub beam_step: usize,
    /// これより遠いレンジは補正に使わない [m] (無効レンジの差し替え値も弾く)。
    pub max_range_m: f64,
    /// predict 1 tick あたりの位置ノイズ σ [m]。
    pub motion_sigma_xy_m: f64,
    /// predict 1 tick あたりの方位ノイズ σ [deg]。
    pub motion_sigma_theta_deg: f64,
    /// [`Belief::seed`] (手動シード) で置く初期 belief の σ [m] / [deg]。
    pub init_sigma_xy_m: f64,
    pub init_sigma_theta_deg: f64,
    /// ビームごとの尤度の床 (完全ミスマッチでも重みを 0 にしない)。本家
    /// likelihood field モデルの z_rand/z_max 混合に相当する 1 定数。
    pub z_min: f64,
    /// 重みの相対しきい値 (max との比)。observe 後の枝刈り = 疎なアクティブ
    /// 集合のしきい値を兼ね、補正コストを belief の広がりに比例させる。
    pub weight_skip_ratio: f64,
    /// 観測一致度 EWMA がこれ未満なら observe 末尾で free 一様を
    /// [`MIX_UNIFORM`] だけ混合する (EMCL 風リセット — 旧 expand/contract
    /// カスケードの後継、レベルなし)。
    pub reset_quality: f64,
    /// ESS ([`Belief::ess`] = 1/Σb²) がこれ超で [`Belief::pose`] = None
    /// (= ロスト)。
    pub lost_ess: f64,
    /// min-plus (MAP / Viterbi) 更新則で回す。全期間 min-plus (レベル切替なし)。
    pub viterbi: bool,
}

impl Default for BeliefConfig {
    fn default() -> Self {
        Self {
            sensor_sigma_m: 0.2,
            beam_step: 10,
            max_range_m: 25.0,
            motion_sigma_xy_m: 0.03,
            motion_sigma_theta_deg: 2.0,
            init_sigma_xy_m: 0.3,
            init_sigma_theta_deg: 10.0,
            z_min: 0.05,
            weight_skip_ratio: 1e-4,
            reset_quality: 0.25,
            lost_ess: 500.0,
            viterbi: false,
        }
    }
}

/// 観測一致度 EWMA の平滑係数。
const EWMA_BETA: f64 = 0.3;
/// リセットで free 一様分布と混ぜる質量比 (EMCL の resetting 相当)。
const MIX_UNIFORM: f32 = 0.5;
/// ロスト解除 (再集中) とみなす ESS (旧 AdaptiveLocalizer の ESS_CONTRACT)。
const ESS_CONTRACT: f64 = 50.0;
/// [`Belief::b_hat`] の下端アンカー: 「十分集中」とみなす ESS。
// ponytail: 定数、必要なら BeliefConfig へ昇格。
const TIGHT_ESS: f64 = 30.0;
/// 運動ノイズの min-plus 緩和コスト [nats/セル]: 経路が 1 セル逸れるごとに
/// e^{-λ} の尤度比を払う指数型ノイズ (sum-product 側の拡散に対応)。
// ponytail: 定数 2 個。地図・センサごとの調整が要るなら BeliefConfig へ昇格。
const VIT_LAMBDA_XY: f32 = 4.0;
/// 同、θ 1 ビンあたり。
const VIT_LAMBDA_T: f32 = 4.0;

/// belief の添字。θ 面優先 (`(it*ny + iy)*nx + ix`) — 移動シフトが θ 面ごとの
/// 連続 2D 面で回るように (旧 bidx2 レイアウト維持)。
#[inline]
fn bidx2(nx: i32, ny: i32, ix: i32, iy: i32, it: i32) -> usize {
    ((it * ny + iy) * nx + ix) as usize
}

/// scatter の書き込み + アクティブ候補の記録。scratch は使用間で全ゼロ不変
/// なので、初回タッチ (== 0.0) が重複なしの候補収集を兼ねる。
#[inline]
fn deposit(scratch: &mut [f32], cand: &mut Vec<u32>, j: usize, m: f32) {
    if m > 0.0 {
        if scratch[j] == 0.0 {
            cand.push(j as u32);
        }
        scratch[j] += m;
    }
}

/// 3 点カーネル [a, 1-2a, a] の a。1 tick σ [セル] のランダムウォーク分散
/// (2a セル²) を合わせる (旧 GridLocalizer::blur_a)。累積 tick 分の a は
/// 呼び出し側が pass 分割で安定域 (中心重み非負) に収める — 旧 0.25
/// クランプの後継。
#[inline]
fn blur_a(sigma_cells: f64) -> f64 {
    sigma_cells * sigma_cells / 2.0
}

/// 静的地図から起こす 2D 尤度場: セルごとに exp(-d²/2σ²) (d = 最近障害物までの
/// 距離) を u8 量子化して持つ。チャンファー 2 パスなので構築は O(セル数)。
struct LikelihoodField {
    w: i32,
    h: i32,
    res: f64,
    ox: f64,
    oy: f64,
    lf: Vec<u8>,
    /// free (data == 0) セルの bitset。belief の物理拘束用 — 壁・未知の中の
    /// 姿勢仮説を許さない (尤度場はビームの当たり先しか見ないので、これが
    /// 無いと「壁の中に居る」仮説が観測で一切罰されない)。
    free: Vec<u64>,
}

impl LikelihoodField {
    fn from_grid(g: &OccupancyGrid, sigma_m: f64) -> Self {
        let (w, h) = (g.width, g.height);
        let n = (w as usize) * (h as usize);
        // ValueIterator と同じ規約: data == 0 が free、非 0 は障害物。
        let mut d = vec![f32::INFINITY; n];
        let mut free = vec![0u64; n.div_ceil(64)];
        for i in 0..n {
            if g.data[i] != 0 {
                d[i] = 0.0;
            } else {
                free[i >> 6] |= 1u64 << (i & 63);
            }
        }
        let idx = |x: i32, y: i32| (y * w + x) as usize;
        const DIAG: f32 = std::f32::consts::SQRT_2;
        // 前進パス (左上 → 右下)。
        for y in 0..h {
            for x in 0..w {
                let mut v = d[idx(x, y)];
                if x > 0 {
                    v = v.min(d[idx(x - 1, y)] + 1.0);
                }
                if y > 0 {
                    v = v.min(d[idx(x, y - 1)] + 1.0);
                    if x > 0 {
                        v = v.min(d[idx(x - 1, y - 1)] + DIAG);
                    }
                    if x < w - 1 {
                        v = v.min(d[idx(x + 1, y - 1)] + DIAG);
                    }
                }
                d[idx(x, y)] = v;
            }
        }
        // 後退パス (右下 → 左上)。
        for y in (0..h).rev() {
            for x in (0..w).rev() {
                let mut v = d[idx(x, y)];
                if x < w - 1 {
                    v = v.min(d[idx(x + 1, y)] + 1.0);
                }
                if y < h - 1 {
                    v = v.min(d[idx(x, y + 1)] + 1.0);
                    if x < w - 1 {
                        v = v.min(d[idx(x + 1, y + 1)] + DIAG);
                    }
                    if x > 0 {
                        v = v.min(d[idx(x - 1, y + 1)] + DIAG);
                    }
                }
                d[idx(x, y)] = v;
            }
        }
        let inv_2s2 = 1.0 / (2.0 * sigma_m * sigma_m);
        let lf = d
            .into_iter()
            .map(|dc| {
                let dm = dc as f64 * g.resolution;
                (255.0 * (-dm * dm * inv_2s2).exp()).round() as u8
            })
            .collect();
        Self { w, h, res: g.resolution, ox: g.origin_x, oy: g.origin_y, lf, free }
    }

    /// セルが free か。地図外は false。
    #[inline]
    fn free_cell(&self, x: i32, y: i32) -> bool {
        if x < 0 || y < 0 || x >= self.w || y >= self.h {
            return false;
        }
        let i = (y * self.w + x) as usize;
        self.free[i >> 6] & (1u64 << (i & 63)) != 0
    }

    /// 世界座標が free セルに乗っているか。
    #[inline]
    fn free_at(&self, wx: f64, wy: f64) -> bool {
        self.free_cell(
            ((wx - self.ox) / self.res).floor() as i32,
            ((wy - self.oy) / self.res).floor() as i32,
        )
    }

    /// 世界座標の尤度 [0,1]。地図外は 0。
    fn at(&self, wx: f64, wy: f64) -> f64 {
        let x = ((wx - self.ox) / self.res).floor() as i32;
        let y = ((wy - self.oy) / self.res).floor() as i32;
        if x < 0 || y < 0 || x >= self.w || y >= self.h {
            return 0.0;
        }
        self.lf[(y * self.w + x) as usize] as f64 / 255.0
    }
}

/// 全地図 belief ヒストグラム — 「belief を VI の状態へ」の推定側。
///
/// belief は VI と同一の格子 (vi_grid = map_scale 適用後の占有格子 × θ ビン)
/// に載る。旧 範囲 belief (GridLocalizer の 5m 窓 / AdaptiveLocalizer の
/// 多重解像度レベル) の後継で、窓オフセット (wx0/wy0) と Level 間接参照は
/// 存在しない — セル (ix, iy, it) は常に地図全域の絶対座標。
///
/// - predict は O(1) の移動量累積のみ。flush (シフト + 拡散) は observe 冒頭で
///   アクティブセルだけの scatter として一括適用する。flush 条件は
///   |pend_f| ≥ 2·res または |pend_rot_deg| ≥ t_res_deg 相当だが、実際の
///   適用点は observe 冒頭に統一 — [`Belief::pose`] は残 pend を平均へ解析的に
///   足すだけで済む (観測間の累積は上のしきい値オーダーに留まる)。
/// - ロスト = ESS > `lost_ess` **または** q_ewma < `reset_quality` (レベル
///   なし)。復帰は同じ q_ewma しきい値での free 一様混合 (EMCL 風リセット)。
/// - `viterbi: true` は同じ遷移モデルを min-plus 半環で回し、observe ごとに
///   b = exp(δmin − δ) を実体化するので ess/top_cells/quality は共通に動く。
pub struct Belief {
    cfg: BeliefConfig,
    /// native 解像度地図の 2D 尤度場 (θ なし・全域・構築 1 回)。
    field: LikelihoodField,
    /// belief 格子 = vi_grid の寸法・幾何。
    nx: i32,
    ny: i32,
    nt: i32,
    res: f64,
    t_res_deg: f64,
    ox: f64,
    oy: f64,
    /// vi_grid の free (data == 0) マスク (2D)。belief の物理拘束。
    free: Vec<bool>,
    n_free: usize,
    /// belief 本体 (レイアウトは [`bidx2`])。b[i] > 0 ⟺ i ∈ active。
    b: Vec<f32>,
    /// scatter の書き込み先。sum-product 経路では使用間で全ゼロ不変。
    scratch: Vec<f32>,
    /// アクティブセル添字 (疎な作業集合)。
    active: Vec<u32>,
    /// min-plus の累積 -ln 尤度 (viterbi 時のみ確保)。非 free・枝刈りは +INF。
    delta: Vec<f32>,
    /// 前回 flush から溜めた移動量 (predict はここに足すだけ)。
    pend_f: f64,
    pend_rot_deg: f64,
    pend_ticks: u32,
    initialized: bool,
    /// 直近 observe の belief 加重ビーム幾何平均尤度。
    quality: f64,
    q_ewma: f64,
    /// ESS = 1/Σb² のキャッシュ (b が変わる場所の末尾で更新 — pose() が
    /// 10 Hz で読むので毎回 active を舐めない)。
    ess_c: f64,
    /// ロスト状態のラッチ (旧 AdaptiveLocalizer の「レベル > 0」に相当)。
    ///
    /// 瞬時値の述語では表せない: 誘拐直後の belief は「間違った場所で自信
    /// 満々」なので ESS は小さいまま (ESS だけでは取りこぼす)、かつ観測不一致
    /// で発火する一様混合リセットが同じ observe 内で q_ewma を書き戻すので、
    /// observe から戻った時点では「合っていなかった」証拠も消えている。
    /// 立つ: 観測不一致 (q_ewma < reset_quality) か belief 拡大 (ess > lost_ess)。
    /// 降りる: 再集中 (ess < [`ESS_CONTRACT`]) かつ観測が合っている
    /// (旧 AdaptiveLocalizer の contract 条件と同じ)。
    lost: bool,
}

impl Belief {
    /// `vi_grid` = プランナと同じ (map_scale 適用後の) 格子 — 幾何
    /// (nx/ny/res/origin) と free マスクの出どころ。`native` = 尤度場用の
    /// 生解像度地図。`theta_bins` は VI と同じ `theta_cell_num`。
    pub fn new(
        vi_grid: &OccupancyGrid,
        theta_bins: i32,
        native: &OccupancyGrid,
        cfg: BeliefConfig,
    ) -> Self {
        let field = LikelihoodField::from_grid(native, cfg.sensor_sigma_m);
        let (nx, ny) = (vi_grid.width, vi_grid.height);
        let nt = theta_bins.max(1);
        let n = (nx as usize) * (ny as usize) * (nt as usize);
        let free: Vec<bool> = vi_grid.data.iter().map(|&d| d == 0).collect();
        let n_free = free.iter().filter(|&&f| f).count();
        Self {
            cfg,
            field,
            nx,
            ny,
            nt,
            res: vi_grid.resolution,
            t_res_deg: 360.0 / nt as f64,
            ox: vi_grid.origin_x,
            oy: vi_grid.origin_y,
            free,
            n_free,
            b: vec![0.0; n],
            scratch: vec![0.0; n],
            active: Vec::new(),
            delta: if cfg.viterbi { vec![f32::INFINITY; n] } else { Vec::new() },
            pend_f: 0.0,
            pend_rot_deg: 0.0,
            pend_ticks: 0,
            initialized: false,
            quality: 0.0,
            q_ewma: 0.0,
            ess_c: 0.0,
            lost: false,
        }
    }

    /// belief バッファのメモリ量 [MB] (起動ログ用)。
    pub fn belief_mb(&self) -> f64 {
        ((self.b.len() + self.scratch.len() + self.delta.len()) * 4) as f64 / 1e6
    }

    /// vi 格子の free セル数 (起動ログ / 一様混合の分母)。
    pub fn free_cells(&self) -> usize {
        self.n_free
    }

    /// 直近の補正の観測一致度 [0,1] (belief 加重のビーム幾何平均尤度)。
    pub fn quality(&self) -> f64 {
        self.quality
    }

    /// 有効セル数 ESS = 1/Σb² (正規化済み前提)。集中度の指標 (キャッシュ)。
    pub fn ess(&self) -> f64 {
        self.ess_c
    }

    /// ESS の対数ビニング: [`TIGHT_ESS`]〜`lost_ess` を log スケールで
    /// 0..nb-1 へ。ロスト ⇔ nb-1 と一貫させる (下の [`Belief::is_lost`] 参照)。
    pub fn b_hat(&self, nb: i32) -> i32 {
        if self.is_lost() {
            return (nb - 1).max(0);
        }
        let r = (self.ess_c / TIGHT_ESS).max(1e-300).ln() / (self.cfg.lost_ess / TIGHT_ESS).ln();
        ((nb as f64 * r).floor() as i32).clamp(0, nb - 1)
    }

    /// ロスト中か ([`Belief::lost`] ラッチの読み出し)。
    fn is_lost(&self) -> bool {
        self.lost
    }

    /// min-plus のリセット床の深さ λ [nats]: free 全セルへ δ ≤ δmin + λ を張る
    /// ときの λ。「一様成分の総質量 : ピーク = α : 1−α」(α = [`MIX_UNIFORM`],
    /// sum-product 側の混合比) から N·e^{−λ}/1 = α/(1−α) ⇒ λ = ln(N·α/(1−α))。
    ///
    /// 定数 λ にしてはいけない — 旧実装の 8.0 は粗レベル (~5k セル) 用の値で、
    /// 全地図 (~10⁶ セル) では一様側がピークの数百倍になり belief が永久に
    /// 平坦化する。枝刈りしきい値もこの λ を内側に含める必要がある
    /// ([`Belief::vit_observe`] 参照)。
    fn reset_floor_ln(&self) -> f64 {
        let n = (self.n_free * self.nt as usize) as f64;
        (n * MIX_UNIFORM as f64 / (1.0 - MIX_UNIFORM as f64)).max(1.0).ln()
    }

    /// θ ビン中心の方位 [rad] (ビン k は [k·res, (k+1)·res) 度 — 切り捨て規約に
    /// 合わせ、中心はその +0.5 ビン)。
    #[inline]
    fn theta_center(&self, it: i32) -> f64 {
        ((it as f64 + 0.5) * self.t_res_deg).to_radians()
    }

    /// セル中心の世界座標。
    #[inline]
    fn cell_center(&self, ix: i32, iy: i32) -> (f64, f64) {
        (
            self.ox + (ix as f64 + 0.5) * self.res,
            self.oy + (iy as f64 + 0.5) * self.res,
        )
    }

    /// 添字 → (ix, iy, it) ([`bidx2`] の逆)。
    #[inline]
    fn decode(&self, i: u32) -> (i32, i32, i32) {
        let plane = (self.nx as usize) * (self.ny as usize);
        let i = i as usize;
        let (it, r) = (i / plane, i % plane);
        ((r % self.nx as usize) as i32, (r / self.nx as usize) as i32, it as i32)
    }

    /// 手動シード (旧 set_pose 相当): belief を pose 中心のガウスで張り直す。
    /// 全地図なので窓の clamp・再センタリングは無い。
    pub fn seed(&mut self, pose: PoseView) {
        for &i in &self.active {
            self.b[i as usize] = 0.0;
        }
        self.active.clear();
        let s_xy = self.cfg.init_sigma_xy_m.max(self.res);
        let s_t = self.cfg.init_sigma_theta_deg.max(self.t_res_deg);
        // 分離ガウスの 1 次元重みを軸ごとに前計算し、無視できる行・列・ビンは
        // 飛ばす (全地図 nx·ny·nt ループを σ 窓に落とす。打ち切った成分は
        // 正規化後どのみち weight_skip_ratio の枝刈り域)。
        const CUT: f64 = 1e-9;
        let gx: Vec<(i32, f64)> = (0..self.nx)
            .filter_map(|ix| {
                let c = self.ox + (ix as f64 + 0.5) * self.res;
                let w = (-(c - pose.x).powi(2) / (2.0 * s_xy * s_xy)).exp();
                (w > CUT).then_some((ix, w))
            })
            .collect();
        let gy: Vec<(i32, f64)> = (0..self.ny)
            .filter_map(|iy| {
                let c = self.oy + (iy as f64 + 0.5) * self.res;
                let w = (-(c - pose.y).powi(2) / (2.0 * s_xy * s_xy)).exp();
                (w > CUT).then_some((iy, w))
            })
            .collect();
        let gt: Vec<(i32, f64)> = (0..self.nt)
            .filter_map(|it| {
                // 円環上の角度差。
                let d = (((it as f64 + 0.5) * self.t_res_deg) - pose.yaw_rad.to_degrees())
                    .rem_euclid(360.0);
                let dd = d.min(360.0 - d);
                let w = (-dd * dd / (2.0 * s_t * s_t)).exp();
                (w > CUT).then_some((it, w))
            })
            .collect();
        for &(it, wt) in &gt {
            for &(iy, wy) in &gy {
                let row = (iy * self.nx) as usize;
                for &(ix, wx) in &gx {
                    // 物理拘束: 非 free (壁・未知) に張らない (旧 apply_free_mask 相当)。
                    if !self.free[row + ix as usize] {
                        continue;
                    }
                    let w = (wt * wy * wx) as f32;
                    if w > 0.0 {
                        let i = bidx2(self.nx, self.ny, ix, iy, it);
                        self.b[i] = w;
                        self.active.push(i as u32);
                    }
                }
            }
        }
        self.normalize_active();
        if self.cfg.viterbi {
            self.vit_enter();
        }
        self.pend_f = 0.0;
        self.pend_rot_deg = 0.0;
        self.pend_ticks = 0;
        self.initialized = true;
        self.q_ewma = 1.0;
        // 手動シードは「ここに居る」という外部の主張 — ラッチを降ろす。
        self.lost = false;
        self.recompute_ess();
    }

    /// 大域初期化 / 全滅復旧: belief を free 一様で張る (旧 enter_uniform の
    /// 全域レベル版)。
    pub fn enter_uniform_free(&mut self) {
        for &i in &self.active {
            self.b[i as usize] = 0.0;
        }
        self.active.clear();
        for it in 0..self.nt {
            for iy in 0..self.ny {
                let row = (iy * self.nx) as usize;
                for ix in 0..self.nx {
                    if !self.free[row + ix as usize] {
                        continue;
                    }
                    let i = bidx2(self.nx, self.ny, ix, iy, it);
                    self.b[i] = 1.0;
                    self.active.push(i as u32);
                }
            }
        }
        self.normalize_active();
        if self.cfg.viterbi {
            self.vit_enter();
        }
        self.pend_f = 0.0;
        self.pend_rot_deg = 0.0;
        self.pend_ticks = 0;
        self.initialized = true;
        // リセット直後に再リセットしない (旧 enter_uniform の q_ewma 復帰と同じ)。
        self.q_ewma = self.cfg.reset_quality;
        self.lost = true;
        self.recompute_ess();
    }

    /// 予測 (実行した速度指令 1 tick ぶん): O(1) の累積のみ。実際のシフト +
    /// 拡散は observe 冒頭の flush が一括適用する。
    pub fn predict(&mut self, v: f64, w_deg: f64, dt: f64) {
        if !self.initialized {
            return;
        }
        self.pend_f += v * dt;
        self.pend_rot_deg += w_deg * dt;
        self.pend_ticks += 1;
    }

    /// 補正: flush(移動シフト + 拡散) → 尤度 → 枝刈り → 正規化 →
    /// q_ewma < reset_quality なら free 一様を混合。未シードなら大域初期化。
    pub fn observe(&mut self, scan: &LaserScan) {
        if !self.initialized {
            // 未シード: 最初のスキャンから大域一様で立ち上がる (大域初期化)。
            self.enter_uniform_free();
        }
        let lost = self.is_lost();
        self.flush();
        if self.active.is_empty() {
            // flush で全質量が壁・地図外へ抜けた — 全滅復旧。
            self.enter_uniform_free();
        }
        // ビーム収集 (`set_local_cost` と同じ世界角規約: ビーム角 = yaw +
        // angle_min + i·inc)。ロスト中は 4 倍間引き。
        // ponytail: ロスト中の observe は全域走査 — TB3 級で数十 ms/scan、
        // キャンパス級は秒単位。上限を上げるなら θ 間引き → coarse-to-fine
        // ゲートの順。
        let step = self.cfg.beam_step.max(1) * if lost { 4 } else { 1 };
        let max_r = self.cfg.max_range_m;
        let beams: Vec<(f64, f64)> = scan
            .ranges
            .iter()
            .enumerate()
            .step_by(step)
            .filter_map(|(i, &r)| {
                (r.is_finite() && r > 0.0 && r <= max_r)
                    .then(|| (scan.angle_min + scan.angle_increment * i as f64, r))
            })
            .collect();
        if beams.is_empty() {
            return;
        }
        let quality = if self.cfg.viterbi {
            self.vit_observe(&beams)
        } else {
            self.sum_observe(&beams)
        };
        self.quality = quality;
        self.q_ewma = (1.0 - EWMA_BETA) * self.q_ewma + EWMA_BETA * quality;
        if self.active.is_empty() {
            // 尤度が全セルでアンダーフローする等の全滅 — 一様から出直す。
            self.enter_uniform_free();
            return;
        }
        // ラッチ条件は mix_uniform の**前**に確定させる — リセットは q_ewma を
        // reset_quality へ書き戻すので、後から見ると不一致の証拠が消えている。
        let mismatched = self.q_ewma < self.cfg.reset_quality;
        if mismatched {
            self.mix_uniform();
        }
        self.recompute_ess();
        if mismatched || self.ess_c > self.cfg.lost_ess {
            self.lost = true;
        } else if self.lost && self.ess_c < ESS_CONTRACT {
            self.lost = false;
        }
    }

    /// 現在の推定姿勢。ロスト ([`Belief::is_lost`]) と未初期化は None。free
    /// スナップ (壁・未知に落ちた平均の実在仮説への吸着) は旧実装のまま。
    pub fn pose(&self) -> Option<PoseView> {
        if !self.initialized || self.is_lost() {
            return None;
        }
        let m = self.mean()?;
        let m = if self.field.free_at(m.x, m.y) {
            m
        } else {
            // 多峰・ドーナツ状 belief の平均は穴 (壁・未知) に落ちる — 実在する
            // 仮説 (free 上の mode) へ吸着して返す。
            self.mode_free().unwrap_or(m)
        };
        // 残 pend (未 flush の移動) の解析加算 — flush は observe 冒頭なので、
        // 観測間の姿勢は平均へ足すだけで済む (広がりの増分は次の flush が持つ)。
        Some(PoseView {
            x: m.x + self.pend_f * m.yaw_rad.cos(),
            y: m.y + self.pend_f * m.yaw_rad.sin(),
            yaw_rad: m.yaw_rad + self.pend_rot_deg.to_radians(),
        })
    }

    /// belief の上位 `k` 仮説 (セル中心の姿勢, 正規化重み)。重み降順 — QMDP
    /// の仮説集合の契約。ロスト中も多峰のまま返す。
    pub fn top_cells(&self, k: usize) -> Vec<(PoseView, f64)> {
        if !self.initialized || k == 0 {
            return Vec::new();
        }
        let mut cells: Vec<(f32, u32)> = self
            .active
            .iter()
            .filter(|&&i| self.b[i as usize] > 0.0)
            .map(|&i| (self.b[i as usize], i))
            .collect();
        if cells.len() > k {
            cells.select_nth_unstable_by(k - 1, |a, b| b.0.total_cmp(&a.0));
            cells.truncate(k);
        }
        cells.sort_by(|a, b| b.0.total_cmp(&a.0));
        cells
            .into_iter()
            .map(|(w, i)| {
                let (ix, iy, it) = self.decode(i);
                let (x, y) = self.cell_center(ix, iy);
                (PoseView { x, y, yaw_rad: self.theta_center(it) }, w as f64)
            })
            .collect()
    }

    // ═══ 内部: 共通機構 ═══

    /// 重み付き平均 (θ は円環平均)。合計 0 なら None。active だけ舐める。
    fn mean(&self) -> Option<PoseView> {
        let (mut sw, mut sx, mut sy, mut sc, mut ss) = (0.0f64, 0.0, 0.0, 0.0, 0.0);
        for &iu in &self.active {
            let w = self.b[iu as usize] as f64;
            if w <= 0.0 {
                continue;
            }
            let (ix, iy, it) = self.decode(iu);
            let th = self.theta_center(it);
            let (cx, cy) = self.cell_center(ix, iy);
            sw += w;
            sx += w * cx;
            sy += w * cy;
            sc += w * th.cos();
            ss += w * th.sin();
        }
        (sw > 0.0).then(|| PoseView { x: sx / sw, y: sy / sw, yaw_rad: ss.atan2(sc) })
    }

    /// free セル上の最大重み仮説 (mode)。free 上に質量が無ければ None。
    fn mode_free(&self) -> Option<PoseView> {
        let mut best: Option<(f32, u32)> = None;
        for &iu in &self.active {
            let w = self.b[iu as usize];
            if w > 0.0 && best.map_or(true, |(bw, _)| w > bw) {
                let (ix, iy, _) = self.decode(iu);
                if self.free[(iy * self.nx + ix) as usize] {
                    best = Some((w, iu));
                }
            }
        }
        best.map(|(_, iu)| {
            let (ix, iy, it) = self.decode(iu);
            let (x, y) = self.cell_center(ix, iy);
            PoseView { x, y, yaw_rad: self.theta_center(it) }
        })
    }

    /// active 上の正規化。
    fn normalize_active(&mut self) {
        let sum: f64 = self.active.iter().map(|&i| self.b[i as usize] as f64).sum();
        if sum > 0.0 && sum.is_finite() {
            // ミスマッチ時のビーム積は f32 subnormal 域まで沈む。そこで
            // `(1/sum) as f32` を掛けると inv が ∞ に飽和して 0×∞ = NaN が belief に
            // 混ざり、EWMA が NaN → リセットが二度と発火しなくなる。f64 で割る。
            for &i in &self.active {
                let v = &mut self.b[i as usize];
                *v = ((*v as f64) / sum) as f32;
            }
        } else if !sum.is_finite() {
            // ∞/NaN が混ざったら復旧不能 — 0 へ落とし、全滅復旧
            // (enter_uniform_free) に回復を任せる。
            for &i in &self.active {
                self.b[i as usize] = 0.0;
            }
            self.active.clear();
        }
    }

    /// ESS キャッシュの更新 (b が変わる操作の末尾で呼ぶ)。
    fn recompute_ess(&mut self) {
        let s2: f64 = self
            .active
            .iter()
            .map(|&i| {
                let v = self.b[i as usize] as f64;
                v * v
            })
            .sum();
        self.ess_c = if s2 > 0.0 { 1.0 / s2 } else { 0.0 };
    }

    /// 溜めた移動量の一括適用 (observe 冒頭)。sum-product はアクティブセルの
    /// scatter シフト + 拡散、min-plus は整数シフト + 緩和。適用後 pend は 0。
    fn flush(&mut self) {
        let (pf, pt_deg, ticks) = (self.pend_f, self.pend_rot_deg, self.pend_ticks);
        self.pend_f = 0.0;
        self.pend_rot_deg = 0.0;
        self.pend_ticks = 0;
        if self.cfg.viterbi {
            let n = self.b.len();
            let (nx, ny, nt) = (self.nx, self.ny, self.nt);
            // セル未満の移動は繰り越さず捨てる — 半セルの誤差は緩和が吸収する。
            if pf.abs() >= 0.5 * self.res || pt_deg.abs() >= 0.5 * self.t_res_deg {
                minplus_shift(
                    &mut self.delta[..n],
                    &mut self.scratch[..n],
                    nx,
                    ny,
                    nt,
                    self.res,
                    self.t_res_deg,
                    pf,
                    pt_deg,
                );
            }
            // 移動ゼロでも回す (sum-product 側が毎 tick 拡散するのと同役)。
            // scratch の全ゼロ不変は sum-product 経路専用なので汚してよい。
            minplus_relax(&mut self.delta[..n], nx, ny, nt, VIT_LAMBDA_XY, VIT_LAMBDA_T);
            return;
        }
        if ticks == 0 && pf == 0.0 && pt_deg == 0.0 {
            return;
        }
        if pf != 0.0 || pt_deg != 0.0 {
            self.shift_scatter(pf, pt_deg);
        }
        let a_xy = blur_a(self.cfg.motion_sigma_xy_m / self.res) * ticks as f64;
        let a_t = if self.nt > 2 {
            blur_a(self.cfg.motion_sigma_theta_deg / self.t_res_deg) * ticks as f64
        } else {
            0.0
        };
        if a_xy > 0.0 || a_t > 0.0 {
            // 6 近傍 scatter の中心重み 1-4a_xy-2a_t を余裕をもって非負に保つ
            // pass 分割 (旧 blur_a の 0.25 クランプの後継 — 1 pass で表せない
            // 累積拡散は複数 pass で表す)。
            let passes = ((4.0 * a_xy + 2.0 * a_t) / 0.5).ceil().max(1.0) as usize;
            let (pa_xy, pa_t) = ((a_xy / passes as f64) as f32, (a_t / passes as f64) as f32);
            for _ in 0..passes {
                self.diffuse_scatter(pa_xy, pa_t);
            }
        }
        // シフト・拡散が壁・地図外へ落とした質量を回収 (物理拘束後の再正規化)。
        self.normalize_active();
        self.recompute_ess();
    }

    /// sum-product のシフト: 各アクティブセルを、その θ の世界方向へ回した
    /// 前進 pf + θ 回転ぶんだけ trilinear scatter で動かす (旧 predict の
    /// 後方双線形サンプリングの、疎集合向け前方版)。非 free・地図外に落ちる
    /// 質量は捨てる。
    fn shift_scatter(&mut self, pf: f64, pt_deg: f64) {
        let (nx, ny, nt) = (self.nx, self.ny, self.nt);
        let ft = pt_deg / self.t_res_deg;
        let mut cand: Vec<u32> = Vec::with_capacity(self.active.len().saturating_mul(8));
        let active = std::mem::take(&mut self.active);
        for &iu in &active {
            let i = iu as usize;
            let w = self.b[i];
            // 旧側を掃除しておく (swap 後に scratch となる — 全ゼロ不変の維持)。
            self.b[i] = 0.0;
            if w <= 0.0 {
                continue;
            }
            let (ix, iy, it) = self.decode(iu);
            let th = self.theta_center(it);
            let ux = ix as f64 + pf * th.cos() / self.res;
            let uy = iy as f64 + pf * th.sin() / self.res;
            let ut = it as f64 + ft;
            let (x0, y0, t0) = (ux.floor(), uy.floor(), ut.floor());
            let (fx, fy, ftr) = (ux - x0, uy - y0, ut - t0);
            for (ot, wt) in [(0i32, 1.0 - ftr), (1, ftr)] {
                if wt <= 0.0 {
                    continue;
                }
                let jt = (t0 as i32 + ot).rem_euclid(nt);
                for (oy, wy) in [(0i32, 1.0 - fy), (1, fy)] {
                    let jy = y0 as i32 + oy;
                    if wy <= 0.0 || jy < 0 || jy >= ny {
                        continue;
                    }
                    for (ox, wx) in [(0i32, 1.0 - fx), (1, fx)] {
                        let jx = x0 as i32 + ox;
                        if wx <= 0.0 || jx < 0 || jx >= nx {
                            continue;
                        }
                        if !self.free[(jy * nx + jx) as usize] {
                            continue;
                        }
                        let j = bidx2(nx, ny, jx, jy, jt);
                        deposit(&mut self.scratch, &mut cand, j, w * (wt * wy * wx) as f32);
                    }
                }
            }
        }
        std::mem::swap(&mut self.b, &mut self.scratch);
        self.active = cand;
    }

    /// sum-product の拡散 1 pass: 6 近傍 (±x, ±y, ±θ) への scatter。旧 blur の
    /// 3 点カーネルを軸ごとに掛ける代わりに、疎集合の 1 回の scatter で近似する
    /// (pass 分割は flush 側)。非 free へ漏れた質量は捨てる (物理拘束)。
    fn diffuse_scatter(&mut self, a_xy: f32, a_t: f32) {
        let (nx, ny, nt) = (self.nx, self.ny, self.nt);
        let wc = 1.0 - 4.0 * a_xy - 2.0 * a_t; // pass 分割が非負を保証
        let mut cand: Vec<u32> = Vec::with_capacity(self.active.len().saturating_mul(7));
        let active = std::mem::take(&mut self.active);
        for &iu in &active {
            let i = iu as usize;
            let w = self.b[i];
            self.b[i] = 0.0;
            if w <= 0.0 {
                continue;
            }
            let (ix, iy, it) = self.decode(iu);
            // 中心 (発生元は free)。
            deposit(&mut self.scratch, &mut cand, i, w * wc);
            for (jx, jy) in [(ix - 1, iy), (ix + 1, iy), (ix, iy - 1), (ix, iy + 1)] {
                if jx < 0 || jx >= nx || jy < 0 || jy >= ny {
                    continue;
                }
                if !self.free[(jy * nx + jx) as usize] {
                    continue;
                }
                deposit(&mut self.scratch, &mut cand, bidx2(nx, ny, jx, jy, it), w * a_xy);
            }
            if a_t > 0.0 {
                for jt in [(it + nt - 1) % nt, (it + 1) % nt] {
                    deposit(&mut self.scratch, &mut cand, bidx2(nx, ny, ix, iy, jt), w * a_t);
                }
            }
        }
        std::mem::swap(&mut self.b, &mut self.scratch);
        self.active = cand;
    }

    /// sum-product の補正: アクティブセルの重みへビーム尤度の積を乗じ、乗算後の
    /// 相対しきい値で枝刈りして正規化する。戻り値は観測一致度。
    ///
    /// 枝刈りを乗算**後**に置くのは意図的 — 乗算前に相対しきい値で切ると、
    /// 一様混合リセットが張った床 (α/free 数 ≪ max·ratio) がビーム評価される
    /// 前に消え、リセットが機能しなくなるため。
    fn sum_observe(&mut self, beams: &[(f64, f64)]) -> f64 {
        let z_min = self.cfg.z_min;
        let m_inv = 1.0 / beams.len() as f64;
        // ビーム角は加法定理で回す (セルごとに全ビームの sin/cos を呼ばない)。
        let bt: Vec<(f64, f64, f64)> =
            beams.iter().map(|&(ba, r)| (ba.cos(), ba.sin(), r)).collect();
        let mut quality = 0.0f64;
        let mut active = std::mem::take(&mut self.active);
        active.retain(|&iu| {
            let i = iu as usize;
            let w = self.b[i];
            if w <= 0.0 {
                return false;
            }
            let (ix, iy, it) = self.decode(iu);
            // 物理拘束: 壁・未知の中の仮説はビーム評価するまでもなく棄却。
            if !self.free[(iy * self.nx + ix) as usize] {
                self.b[i] = 0.0;
                return false;
            }
            let th = self.theta_center(it);
            let (ct, st) = (th.cos(), th.sin());
            let (cx, cy) = self.cell_center(ix, iy);
            let mut prod = 1.0f64;
            for &(cb, sb, r) in &bt {
                let (ca, sa) = (ct * cb - st * sb, st * cb + ct * sb);
                let l = self.field.at(cx + r * ca, cy + r * sa);
                prod *= z_min + (1.0 - z_min) * l;
            }
            // 観測一致度はビームの**幾何**平均 (= prod^(1/M))。算術平均だと
            // ミスマッチでも「たまたま障害物帯に乗った端点」の寄与で 0.3 台に
            // 浮き、ロスト検出のしきい値と分離できない。幾何平均は外れビームに
            // 引きずられて z_min 側へ落ちるので、整合 (~0.5+) と乖離する。
            quality += w as f64 * prod.powf(m_inv);
            self.b[i] = (w as f64 * prod) as f32;
            true
        });
        // 枝刈り: 乗算後の集中をアクティブ集合へ反映 (weight_skip_ratio が
        // 疎な作業集合のしきい値を兼ねる)。
        let maxw = active.iter().map(|&i| self.b[i as usize]).fold(0.0f32, f32::max);
        let thr = maxw * self.cfg.weight_skip_ratio as f32;
        active.retain(|&iu| {
            let i = iu as usize;
            if self.b[i] <= thr {
                self.b[i] = 0.0;
                false
            } else {
                true
            }
        });
        self.active = active;
        self.normalize_active();
        quality
    }

    /// EMCL 風リセット: free 一様を [`MIX_UNIFORM`] だけ混ぜる (旧 expand の
    /// 混合部だけの salvage — 粗レベルへの射影は無い)。min-plus では free 全
    /// セルへ δ の床を与える等価操作。
    fn mix_uniform(&mut self) {
        if self.n_free == 0 {
            return;
        }
        if self.cfg.viterbi {
            let lam = self.reset_floor_ln() as f32;
            let dmin = self
                .active
                .iter()
                .map(|&i| self.delta[i as usize])
                .fold(f32::INFINITY, f32::min);
            if !dmin.is_finite() {
                // 全滅は observe / vit_observe 側の自己修復に任せる。
                return;
            }
            self.active.clear();
            for it in 0..self.nt {
                for iy in 0..self.ny {
                    let row = (iy * self.nx) as usize;
                    for ix in 0..self.nx {
                        if !self.free[row + ix as usize] {
                            continue;
                        }
                        let i = bidx2(self.nx, self.ny, ix, iy, it);
                        let d = self.delta[i].min(dmin + lam);
                        self.delta[i] = d;
                        // b = exp(δmin − δ) の再実体化 (床込み)。
                        self.b[i] = ((dmin - d) as f64).exp() as f32;
                        self.active.push(i as u32);
                    }
                }
            }
            self.normalize_active();
        } else {
            // (1−α)·belief + α·free-一様。belief は正規化済みなので総和は 1 のまま。
            let alpha = MIX_UNIFORM;
            for &i in &self.active {
                self.b[i as usize] *= 1.0 - alpha;
            }
            let u = alpha / (self.n_free * self.nt as usize) as f32;
            self.active.clear();
            for it in 0..self.nt {
                for iy in 0..self.ny {
                    let row = (iy * self.nx) as usize;
                    for ix in 0..self.nx {
                        if !self.free[row + ix as usize] {
                            continue;
                        }
                        let i = bidx2(self.nx, self.ny, ix, iy, it);
                        self.b[i] += u;
                        self.active.push(i as u32);
                    }
                }
            }
        }
        // リセット直後に再リセットしない。
        self.q_ewma = self.cfg.reset_quality;
    }

    // ═══ 内部: min-plus (viterbi) ═══

    /// δ の初期化: 今の b (正規化済み) を -ln で写す (min-plus は定数シフト
    /// 不変なので正規化定数は気にしない)。b = 0 (非 free 等) は +INF。
    fn vit_enter(&mut self) {
        self.delta.fill(f32::INFINITY);
        for &i in &self.active {
            let w = self.b[i as usize];
            if w > 0.0 {
                self.delta[i as usize] = (-(w as f64).ln()) as f32;
            }
        }
    }

    /// 補正の min-plus 版: (flush 済みの) δ へ観測コスト -ln(尤度積) を加算し、
    /// b = exp(δmin − δ) を実体化する。戻り値は quality (従来と同じ
    /// 「前回 b 加重のビーム幾何平均尤度」— しきい値系をそのまま使う)。
    fn vit_observe(&mut self, beams: &[(f64, f64)]) -> f64 {
        let (nx, ny, nt) = (self.nx, self.ny, self.nt);
        let n = (nx as usize) * (ny as usize) * (nt as usize);
        let (ox, oy, res, t_res) = (self.ox, self.oy, self.res, self.t_res_deg);
        let z_min = self.cfg.z_min;
        // Bayes 側の weight_skip_ratio と同じ意味の枝刈り (δ は -ln 重み)。
        // ただしリセット床 ([`Belief::reset_floor_ln`]) は必ず内側に含める:
        // 床を切ってから観測を足すと、一様混合が張った仮説が尤度で評価される
        // 前に消え、再定位が二度と起きない (sum_observe が枝刈りを乗算**後**に
        // 置いているのと同じ理由)。床が張られた直後の 1 回だけ全域評価になる。
        let thr_ln =
            (-(self.cfg.weight_skip_ratio.max(1e-30)).ln()).max(self.reset_floor_ln()) as f32;
        let m_inv = 1.0 / beams.len() as f64;
        let bt: Vec<(f64, f64, f64)> =
            beams.iter().map(|&(ba, r)| (ba.cos(), ba.sin(), r)).collect();
        let mut quality = 0.0f64;
        let delta = &mut self.delta;
        let dmin0 = delta[..n].iter().cloned().fold(f32::INFINITY, f32::min);
        for it in 0..nt {
            let th = ((it as f64 + 0.5) * t_res).to_radians();
            let (ct, st) = (th.cos(), th.sin());
            for iy in 0..ny {
                let cy = oy + (iy as f64 + 0.5) * res;
                for ix in 0..nx {
                    let i = bidx2(nx, ny, ix, iy, it);
                    let d = delta[i];
                    // NaN (全滅時の INF−INF) もこの否定形で落ちる。
                    if !(d - dmin0 <= thr_ln) {
                        delta[i] = f32::INFINITY;
                        continue;
                    }
                    if !self.free[(iy * nx + ix) as usize] {
                        delta[i] = f32::INFINITY;
                        continue;
                    }
                    let cx = ox + (ix as f64 + 0.5) * res;
                    let mut prod = 1.0f64;
                    for &(cb, sb, r) in &bt {
                        let (ca, sa) = (ct * cb - st * sb, st * cb + ct * sb);
                        let l = self.field.at(cx + r * ca, cy + r * sa);
                        prod *= z_min + (1.0 - z_min) * l;
                    }
                    quality += self.b[i] as f64 * prod.powf(m_inv);
                    delta[i] = d - prod.ln() as f32;
                }
            }
        }
        // b = exp(δmin − δ) の実体化 + アクティブ集合の再構成。
        let dmin = delta[..n].iter().cloned().fold(f32::INFINITY, f32::min);
        self.active.clear();
        if dmin.is_finite() {
            for (i, d) in delta[..n].iter().enumerate() {
                self.b[i] = if d.is_finite() {
                    let w = ((dmin - d) as f64).exp() as f32;
                    if w > 0.0 {
                        self.active.push(i as u32);
                    }
                    w
                } else {
                    0.0
                };
            }
        } else {
            // 全滅 (シフトで地図外へ抜けた等) — free 一様へ自己修復。
            for it in 0..nt {
                for iy in 0..ny {
                    let row = (iy * nx) as usize;
                    for ix in 0..nx {
                        let i = bidx2(nx, ny, ix, iy, it);
                        let f = self.free[row + ix as usize];
                        delta[i] = if f { 0.0 } else { f32::INFINITY };
                        self.b[i] = if f { 1.0 } else { 0.0 };
                        if f {
                            self.active.push(i as u32);
                        }
                    }
                }
            }
        }
        self.normalize_active();
        quality
    }
}

/// min-plus の決定的シフト: 各 θ 面をその方位の移動量ぶん整数シフトし、θ 面
/// 自体を回転ぶん円環シフトする (sum-product のシフトの min-plus 版 — 補間は
/// しない。半セルの誤差は直後の緩和が吸収する)。範囲外からの取り込みは +INF。
#[allow(clippy::too_many_arguments)]
fn minplus_shift(
    delta: &mut [f32],
    tmp: &mut [f32],
    nx: i32,
    ny: i32,
    nt: i32,
    res: f64,
    t_res: f64,
    pf: f64,
    pt_deg: f64,
) {
    for it in 0..nt {
        let th = ((it as f64 + 0.5) * t_res).to_radians();
        let rx = (pf * th.cos() / res).round() as i32;
        let ry = (pf * th.sin() / res).round() as i32;
        for iy in 0..ny {
            for ix in 0..nx {
                let (sx, sy) = (ix - rx, iy - ry);
                tmp[bidx2(nx, ny, ix, iy, it)] = if sx >= 0 && sx < nx && sy >= 0 && sy < ny {
                    delta[bidx2(nx, ny, sx, sy, it)]
                } else {
                    f32::INFINITY
                };
            }
        }
    }
    let rt = ((pt_deg / t_res).round() as i32).rem_euclid(nt);
    for it in 0..nt {
        let st = (it - rt).rem_euclid(nt);
        for iy in 0..ny {
            for ix in 0..nx {
                delta[bidx2(nx, ny, ix, iy, it)] = tmp[bidx2(nx, ny, ix, iy, st)];
            }
        }
    }
}

/// 軸分離の min-plus 緩和 (soft erosion): δ(s) ← min_k δ(s ± k·e) + λ·k。
/// 前進 + 後退の 2 掃引で軸ごとの距離変換になる (θ は円環なので 2 周する)。
fn minplus_relax(delta: &mut [f32], nx: i32, ny: i32, nt: i32, l_xy: f32, l_t: f32) {
    for it in 0..nt {
        for iy in 0..ny {
            for ix in 1..nx {
                let p = delta[bidx2(nx, ny, ix - 1, iy, it)] + l_xy;
                let i = bidx2(nx, ny, ix, iy, it);
                if p < delta[i] {
                    delta[i] = p;
                }
            }
            for ix in (0..nx - 1).rev() {
                let p = delta[bidx2(nx, ny, ix + 1, iy, it)] + l_xy;
                let i = bidx2(nx, ny, ix, iy, it);
                if p < delta[i] {
                    delta[i] = p;
                }
            }
        }
        for ix in 0..nx {
            for iy in 1..ny {
                let p = delta[bidx2(nx, ny, ix, iy - 1, it)] + l_xy;
                let i = bidx2(nx, ny, ix, iy, it);
                if p < delta[i] {
                    delta[i] = p;
                }
            }
            for iy in (0..ny - 1).rev() {
                let p = delta[bidx2(nx, ny, ix, iy + 1, it)] + l_xy;
                let i = bidx2(nx, ny, ix, iy, it);
                if p < delta[i] {
                    delta[i] = p;
                }
            }
        }
    }
    if nt > 1 {
        for iy in 0..ny {
            for ix in 0..nx {
                for k in 1..(2 * nt) {
                    let p = delta[bidx2(nx, ny, ix, iy, (k - 1).rem_euclid(nt))] + l_t;
                    let i = bidx2(nx, ny, ix, iy, k % nt);
                    if p < delta[i] {
                        delta[i] = p;
                    }
                }
                for k in (0..(2 * nt - 1)).rev() {
                    let p = delta[bidx2(nx, ny, ix, iy, (k + 1) % nt)] + l_t;
                    let i = bidx2(nx, ny, ix, iy, k % nt);
                    if p < delta[i] {
                        delta[i] = p;
                    }
                }
            }
        }
    }
}

/// 真値姿勢からの全周スキャンをレイマーチで合成する理想センサ (angle_min = 0)。
/// テストと `viola_bench` の閉ループシミュレーションが共用する。
pub fn cast_scan(g: &OccupancyGrid, truth: PoseView, n_beams: usize, max_r: f64) -> LaserScan {
    let inc = 2.0 * std::f64::consts::PI / n_beams as f64;
    let step = g.resolution / 2.0;
    let ranges = (0..n_beams)
        .map(|i| {
            let a = truth.yaw_rad + i as f64 * inc;
            let mut r = step;
            loop {
                if r >= max_r {
                    break max_r;
                }
                let ix = ((truth.x + r * a.cos() - g.origin_x) / g.resolution).floor() as i32;
                let iy = ((truth.y + r * a.sin() - g.origin_y) / g.resolution).floor() as i32;
                if ix < 0 || iy < 0 || ix >= g.width || iy >= g.height {
                    break max_r;
                }
                if g.data[(iy * g.width + ix) as usize] != 0 {
                    break r;
                }
                r += step;
            }
        })
        .collect();
    LaserScan { angle_min: 0.0, angle_increment: inc, ranges }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Quaternion;

    fn pose(x: f64, y: f64, yaw: f64) -> PoseView {
        PoseView { x, y, yaw_rad: yaw }
    }

    /// 外周を壁で囲い、対称性崩しの内部ブロックを 1 つ置いた占有格子 (@0.05 m)。
    fn walled_grid(size: i32) -> OccupancyGrid {
        let mut g = OccupancyGrid {
            width: size,
            height: size,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        };
        for i in 0..size {
            for (x, y) in [(i, 0), (i, size - 1), (0, i), (size - 1, i)] {
                g.data[(y * size + x) as usize] = 100;
            }
        }
        for y in 10..14 {
            for x in 40..44 {
                g.data[(y * size + x) as usize] = 100;
            }
        }
        g
    }

    /// 10m×10m、非対称な内部構造つきの占有格子 (@0.05 m)。ブロック配置が
    /// 場所ごとに違うスキャン特徴を作るので、大域再定位が一意に解ける。
    fn tenm_grid() -> OccupancyGrid {
        let size = 200;
        let mut g = OccupancyGrid {
            width: size,
            height: size,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        };
        for i in 0..size {
            for (x, y) in [(i, 0), (i, size - 1), (0, i), (size - 1, i)] {
                g.data[(y * size + x) as usize] = 100;
            }
        }
        for (x0, x1, y0, y1) in
            [(20, 30, 20, 30), (140, 170, 40, 48), (60, 66, 150, 180), (118, 126, 118, 126)]
        {
            for y in y0..y1 {
                for x in x0..x1 {
                    g.data[(y * size + x) as usize] = 100;
                }
            }
        }
        g
    }

    fn wrap_rad(d: f64) -> f64 {
        use std::f64::consts::PI;
        (d + PI).rem_euclid(2.0 * PI) - PI
    }

    /// ずらしたシードから合成スキャンで真値へ収束すること (correct の本体)。
    #[test]
    fn belief_tightens_onto_the_true_pose() {
        let g = walled_grid(60); // 3m×3m @0.05
        let truth = pose(1.2, 1.5, 0.5);
        let bc = BeliefConfig {
            beam_step: 1,
            init_sigma_xy_m: 0.3,
            init_sigma_theta_deg: 20.0,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);
        assert!(loc.pose().is_none(), "シード前は None");

        // 真値から 0.14m / 11° ずらして手動シード。
        loc.seed(pose(1.3, 1.4, 0.3));
        let scan = cast_scan(&g, truth, 36, 5.0);
        for _ in 0..6 {
            loc.observe(&scan);
            loc.predict(0.0, 0.0, 0.1); // 静止でも動作ノイズの拡散は回る
        }

        let m = loc.pose().expect("収束後の平均");
        assert!(
            (m.x - truth.x).abs() < 0.1 && (m.y - truth.y).abs() < 0.1,
            "mean ({:.3}, {:.3}) が真値 ({:.3}, {:.3}) から遠い",
            m.x, m.y, truth.x, truth.y
        );
        assert!(
            wrap_rad(m.yaw_rad - truth.yaw_rad).abs() < 0.2,
            "yaw {:.3} が真値 {:.3} から遠い",
            m.yaw_rad, truth.yaw_rad
        );
        // 観測一致度は AdaptiveLocalizer 由来の「z_min 混合後のビーム**幾何**
        // 平均」(旧 GridLocalizer の生尤度の算術平均ではない — 幾何平均でないと
        // ロストと追従がしきい値で分離できない、という理由でこちらを採った)。
        // 同じ収束状態でも値域が違うので、意味のある主張は「リセット
        // (= ロスト) しきい値から十分上」。
        let reset_q = BeliefConfig::default().reset_quality;
        assert!(
            loc.quality() > 1.5 * reset_q,
            "観測一致度が低すぎる: {} (リセットしきい値 {})",
            loc.quality(),
            reset_q
        );
    }

    /// predict が指令どおり平均を進め、回すこと (動作モデル = 自分の cmd_vel)。
    /// flush 前は残 pend の解析加算、observe (flush) 後は belief 本体のシフト。
    #[test]
    fn belief_predict_advances_the_mean_along_the_heading() {
        let g = walled_grid(60);
        // ESS ゲートに掛からない程度に締めた初期 belief (observe 無しで pose を読む)。
        let bc = BeliefConfig {
            init_sigma_xy_m: 0.1,
            init_sigma_theta_deg: 5.0,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);
        loc.seed(pose(1.5, 1.5, 0.0));

        // 前進 0.3 m/s × 1.0 s。flush 前でも pose() は解析 pend で前進を返す。
        for _ in 0..10 {
            loc.predict(0.3, 0.0, 0.1);
        }
        let m = loc.pose().expect("解析 pend の平均");
        assert!((m.x - 1.8).abs() < 0.08, "x = {:.3} (期待 1.8 付近)", m.x);
        assert!((m.y - 1.5).abs() < 0.05, "y = {:.3} (期待 1.5 のまま)", m.y);

        // observe が flush を適用 — belief 本体も前進しているはず。
        let scan = cast_scan(&g, pose(1.8, 1.5, 0.0), 36, 5.0);
        loc.observe(&scan);
        let m = loc.pose().expect("flush 後の平均");
        assert!((m.x - 1.8).abs() < 0.1, "flush 後 x = {:.3} (期待 1.8 付近)", m.x);

        // その場旋回 90 deg/s × 1.0 s (解析 pend)。
        for _ in 0..10 {
            loc.predict(0.0, 90.0, 0.1);
        }
        let m = loc.pose().expect("平均");
        assert!(
            wrap_rad(m.yaw_rad - std::f64::consts::FRAC_PI_2).abs() < 0.2,
            "yaw = {:.3} (期待 π/2 付近)",
            m.yaw_rad
        );
    }

    /// 誘拐 (瞬間移動) から一様混合リセットで復帰すること。ロスト検出
    /// (ESS ゲートで pose None) → リセット → 再定位、の全カスケード。
    #[test]
    fn belief_recovers_from_kidnap() {
        let g = tenm_grid();
        let bc = BeliefConfig { beam_step: 4, ..BeliefConfig::default() };
        let mut loc = Belief::new(&g, 36, &g, bc);

        let a = pose(2.5, 2.0, 0.4);
        loc.seed(a);
        let scan_a = cast_scan(&g, a, 180, 12.0);
        for _ in 0..5 {
            loc.observe(&scan_a);
        }
        assert!(loc.pose().is_some());

        // 誘拐: 約 8m 離れた場所へ。
        let b = pose(8.0, 8.0, 2.0);
        let scan_b = cast_scan(&g, b, 180, 12.0);
        let (mut lost, mut recovered) = (false, None);
        for i in 0..150 {
            loc.observe(&scan_b);
            match loc.pose() {
                None => lost = true,
                Some(p) => {
                    if lost
                        && (p.x - b.x).abs() < 0.4
                        && (p.y - b.y).abs() < 0.4
                        && wrap_rad(p.yaw_rad - b.yaw_rad).abs() < 0.4
                    {
                        recovered = Some(i);
                        break;
                    }
                }
            }
        }
        assert!(lost, "誘拐でロスト (pose None) を検出すること");
        assert!(
            recovered.is_some(),
            "150 スキャン以内に再定位すること (q={:.3}, ess={:.0})",
            loc.quality(),
            loc.ess()
        );
    }

    /// 未シードでも最初のスキャンから大域初期化で立ち上がること。
    #[test]
    fn belief_global_init_without_seed() {
        let g = tenm_grid();
        let bc = BeliefConfig { beam_step: 4, ..BeliefConfig::default() };
        let mut loc = Belief::new(&g, 36, &g, bc);
        assert!(loc.pose().is_none());
        let truth = pose(8.0, 8.0, 2.0);
        let scan = cast_scan(&g, truth, 180, 12.0);
        let mut ok = None;
        for i in 0..150 {
            loc.observe(&scan);
            if let Some(p) = loc.pose() {
                if (p.x - truth.x).abs() < 0.4 && (p.y - truth.y).abs() < 0.4 {
                    ok = Some(i);
                    break;
                }
            }
        }
        assert!(ok.is_some(), "未シードでも大域初期化で再定位すること (ess={:.0})", loc.ess());
    }

    /// 推定が壁・未知の中に落ちないこと (free マスク + free スナップ)。
    /// ブロックのど真ん中へシードすると、マスクが質量を周囲の free へ追い出し、
    /// リング状に残った belief の平均はブロック内 (穴) に戻る — pose() は free 上の
    /// mode へ吸着して返すはず。マスクかスナップのどちらが欠けても落ちる。
    #[test]
    fn estimate_never_lands_in_occupied_space() {
        let g = walled_grid(80);
        let free_at = |p: PoseView| {
            let ix = ((p.x - g.origin_x) / g.resolution).floor() as i32;
            let iy = ((p.y - g.origin_y) / g.resolution).floor() as i32;
            (0..g.width).contains(&ix)
                && (0..g.height).contains(&iy)
                && g.data[(iy * g.width + ix) as usize] == 0
        };
        // walled_grid の内部ブロック (x40..44, y10..14) の中心。ESS ゲートに
        // 掛からない程度に締めた初期 belief。
        let block_center = pose(42.0 * 0.05, 12.0 * 0.05, 0.0);
        let bc = BeliefConfig {
            init_sigma_xy_m: 0.1,
            init_sigma_theta_deg: 5.0,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);
        loc.seed(block_center);
        let p = loc.pose().expect("マスク後も free 側に質量が残ること");
        assert!(free_at(p), "推定 ({:.2}, {:.2}) が free でない", p.x, p.y);
    }

    /// viterbi (min-plus): 誘拐から復帰すること — sum-product をやめて全期間
    /// 再帰 MAP にしても同じ復帰性が保たれる回帰。
    #[test]
    fn viterbi_recovers_from_kidnap() {
        let g = tenm_grid();
        let bc = BeliefConfig { beam_step: 4, viterbi: true, ..BeliefConfig::default() };
        let mut loc = Belief::new(&g, 36, &g, bc);

        let a = pose(2.5, 2.0, 0.4);
        loc.seed(a);
        let scan_a = cast_scan(&g, a, 180, 12.0);
        for _ in 0..5 {
            loc.observe(&scan_a);
        }
        assert!(loc.pose().is_some());

        let b = pose(8.0, 8.0, 2.0);
        let scan_b = cast_scan(&g, b, 180, 12.0);
        let (mut lost, mut recovered) = (false, None);
        for i in 0..150 {
            loc.observe(&scan_b);
            match loc.pose() {
                None => lost = true,
                Some(p) => {
                    if lost
                        && (p.x - b.x).abs() < 0.4
                        && (p.y - b.y).abs() < 0.4
                        && wrap_rad(p.yaw_rad - b.yaw_rad).abs() < 0.4
                    {
                        recovered = Some(i);
                        break;
                    }
                }
            }
        }
        assert!(lost, "誘拐でロスト (pose None) を検出すること");
        assert!(
            recovered.is_some(),
            "150 スキャン以内に再定位すること (q={:.3}, ess={:.0})",
            loc.quality(),
            loc.ess()
        );
    }

    /// viterbi: 未シードの大域初期化 (enter_uniform_free → δ 一様) でも
    /// 立ち上がること。ロスト中の predict は移動量の記録だけ (O(1))。
    #[test]
    fn viterbi_global_init_without_seed() {
        let g = tenm_grid();
        let bc = BeliefConfig { beam_step: 4, viterbi: true, ..BeliefConfig::default() };
        let mut loc = Belief::new(&g, 36, &g, bc);
        assert!(loc.pose().is_none());
        let truth = pose(8.0, 8.0, 2.0);
        let scan = cast_scan(&g, truth, 180, 12.0);
        let mut ok = None;
        for i in 0..150 {
            loc.observe(&scan);
            // 静止でも落ちないこと (pend 累積のみ)。
            loc.predict(0.0, 0.0, 0.1);
            if let Some(p) = loc.pose() {
                if (p.x - truth.x).abs() < 0.4 && (p.y - truth.y).abs() < 0.4 {
                    ok = Some(i);
                    break;
                }
            }
        }
        assert!(ok.is_some(), "大域初期化から再定位できない (ess={:.0})", loc.ess());
    }

    /// top_cells: シード直後の最大重み仮説がシード姿勢のセルで、重みが降順な
    /// こと (QMDP の仮説集合の契約)。
    #[test]
    fn top_cells_returns_descending_hypotheses_near_the_seed() {
        let g = walled_grid(60);
        let seed = pose(1.5, 1.5, 0.0);
        let mut loc = Belief::new(&g, 36, &g, BeliefConfig::default());
        assert!(loc.top_cells(8).is_empty(), "シード前は空");
        loc.seed(seed);
        let cells = loc.top_cells(8);
        assert!(!cells.is_empty(), "仮説が空");
        assert!(cells.len() <= 8, "k を超過");
        for w in cells.windows(2) {
            assert!(w[0].1 >= w[1].1, "重みが降順でない");
        }
        let top = cells[0].0;
        assert!(
            (top.x - seed.x).abs() < 0.1 && (top.y - seed.y).abs() < 0.1,
            "最大重み仮説 ({:.2}, {:.2}) がシードから遠い",
            top.x, top.y
        );
    }

    /// ESS が pose のゲートと b_hat の広がり報告を担うこと:
    /// free 一様 ⇒ ESS 大・pose None・b_hat 上端、シード + 補正 ⇒ ESS 小・
    /// pose 有り・b_hat 下端。
    #[test]
    fn ess_gates_pose_and_reports_spread() {
        let g = walled_grid(60);
        let bc = BeliefConfig {
            beam_step: 1,
            init_sigma_xy_m: 0.1,
            init_sigma_theta_deg: 5.0,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);

        loc.enter_uniform_free();
        assert!(loc.ess() > 500.0, "一様 belief の ESS が小さすぎる: {:.0}", loc.ess());
        assert!(loc.pose().is_none(), "ロスト (ESS 超過) では pose は None");
        assert_eq!(loc.b_hat(4), 3, "ロスト ⇔ 上端ビン");

        let truth = pose(1.2, 1.5, 0.5);
        loc.seed(pose(1.3, 1.4, 0.3));
        let scan = cast_scan(&g, truth, 36, 5.0);
        for _ in 0..6 {
            loc.observe(&scan);
        }
        assert!(loc.ess() < 60.0, "収束後の ESS が大きすぎる: {:.1}", loc.ess());
        assert!(loc.pose().is_some(), "集中した belief は pose を返す");
        assert_eq!(loc.b_hat(4), 0, "集中 ⇔ 下端ビン (ess={:.1})", loc.ess());
    }
}
