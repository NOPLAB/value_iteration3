//! 自己位置の出どころ ([`Localizer`]) — 外部推定の素通しと、内蔵ヒストグラム
//! MCL (VIOLA の推定部) を trait で抽象化する。ROS 配線と切り替え (パラメータ
//! `localizer`) は vi_planner 側 (`FollowController` ↔ [`crate::ctrl`] と同じ
//! 分担: アルゴリズムは vi_lib、配線はノード)。
//!
//! - [`ExternalLocalizer`] — `pose_topic` (mcl 等) の推定をそのまま返す。既定。
//! - [`GridLocalizer`] — ロボット近傍の窓 (既定 5m×5m × θ) に belief ヒストグラムを
//!   持ち、predict (自分が出した速度指令によるシフト + ノイズぼかし) と
//!   correct (スキャン端点 × 静的地図の 2D 尤度場) で自己位置を推定する。
//!   このとき `pose_topic` は **手動シード** (initialpose 等) として扱い、
//!   メッセージが来るたびに belief を初期化し直す — 連続推定器 (mcl) を
//!   指したままにしないこと (毎メッセージでリセットされて素通しと変わらなくなる)。
//! - [`AdaptiveLocalizer`] — GridLocalizer の多重解像度版。通常は同じ細窓 (L0) で
//!   推定し、観測一致度が落ちると belief を粗い広域レベルへ広げて再定位する
//!   (EMCL の expansion resetting のヒストグラム版)。最終レベルは地図全域なので
//!   誘拐 (持ち上げ移動) からも復帰でき、未シードなら大域初期化で立ち上がる。
//!   ロスト中 (粗レベル滞在中) は `pose()` が `None` — 呼び出し側は既存の
//!   「pose なし」停止経路に乗る。
//! - `viterbi` ([`BeliefConfig::viterbi`] = AdaptiveLocalizer の min-plus モード) —
//!   全域レベルの推定だけを sum-product から max-product (Viterbi / MAP) に替える
//!   (localize/viterbi.rs の doc)。窓レベルの追跡・レベル遷移機構は共通。
//!
//! # 設計上の要点
//!
//! - 動作モデルはオドメトリではなく**自分が配信した cmd_vel そのもの**
//!   ([`Localizer::predict`] に follow ループが渡す)。プランナの行動と推定器の
//!   予測が原理的に一致する。実行誤差は `motion_sigma_*` のぼかしが吸収する。
//! - 尤度場は θ なしの 2D・全域・起動時 1 回 (スキャン端点は窓の外に着地するため)。
//!   1 B/セル量子化なのでキャンパス地図 (14000×800) でも 11 MB。
//! - belief 窓は VI と同じ地図座標系の **native 解像度** (map_scale をかける前) に
//!   載せる。窓は平均が中心から離れると整数セルシフトで再センタリングする。
//! - ROS 配線 (購読・predict の呼び出し) は vi_planner の main.rs 側。

use crate::bridge::PoseView;
use crate::msg::{LaserScan, OccupancyGrid};

mod viterbi;
use viterbi::VitState;

/// 自己位置の出どころ。follow ループ・plan サーバは [`Localizer::pose`] の結果
/// (`latest_pose`) だけを見るので、実装を替えても走行側のコードは変わらない。
///
/// 呼び出し規約 (main.rs 側):
/// - `set_pose` — `pose_topic` の受信ごと。
/// - `observe` — `scan_topic` の受信ごと。
/// - `predict` — follow ループが速度指令を配信した tick ごと (v [m/s],
///   w [deg/s], dt [s] = 制御周期)。停止指令は動きゼロなので呼ばなくてよい。
pub trait Localizer: Send {
    fn name(&self) -> &'static str;
    /// 外部姿勢の取り込み。External = そのまま採用。Grid = belief を初期化。
    fn set_pose(&mut self, pose: PoseView);
    /// 実行した速度指令 1 tick ぶんの予測ステップ。
    fn predict(&mut self, v: f64, w_deg: f64, dt: f64);
    /// スキャンによる補正ステップ。
    fn observe(&mut self, scan: &LaserScan);
    /// 現在の推定姿勢 (未初期化なら None)。
    fn pose(&self) -> Option<PoseView>;
    /// 直近の補正の観測一致度 [0,1] (belief 加重のビーム平均尤度)。External は
    /// 常に 1.0。lost 検出や settle 遷移の材料として node 側が読める。
    fn quality(&self) -> f64 {
        1.0
    }
    /// belief の上位 `k` 仮説 (セル中心の姿勢, 非正規化重み)。重み降順。QMDP
    /// ([`crate::planner::qmdp_decide`]) の仮説集合用。単一仮説の実装 (External)
    /// は空を返す — 呼び出し側は `pose()` の点推定にフォールバックすること。
    fn top_cells(&self, _k: usize) -> Vec<(PoseView, f64)> {
        Vec::new()
    }
    /// ロスト中の能動的再定位の行き先候補 (世界座標、仮説ごとに 1 点)。
    /// 空 = 提案なし (非ロスト / 単峰 / どこへ行っても判別不能)。呼び出し側は
    /// この集合を多目標 VI (`ValueIterator::set_goal_region`) のゴールにして
    /// QMDP で走る — どの仮説が真でも、その仮説にとっての判別点へ向かう。
    /// 多峰 belief を持つ推定器 ([`AdaptiveLocalizer`]) だけが実装する。
    fn reloc_targets(&self) -> Vec<(f64, f64)> {
        Vec::new()
    }
}

/// 現行動作: `pose_topic` の推定を素通しする (既定)。
#[derive(Default)]
pub struct ExternalLocalizer {
    pose: Option<PoseView>,
}

impl Localizer for ExternalLocalizer {
    fn name(&self) -> &'static str {
        "external"
    }
    fn set_pose(&mut self, pose: PoseView) {
        self.pose = Some(pose);
    }
    fn predict(&mut self, _v: f64, _w_deg: f64, _dt: f64) {}
    fn observe(&mut self, _scan: &LaserScan) {}
    fn pose(&self) -> Option<PoseView> {
        self.pose
    }
}

/// [`GridLocalizer`] のチューニング。実機はスキャナも床も理想モデルから
/// ずれるので、ノイズ幅は必ずパラメータで残す。
#[derive(Clone, Copy, Debug)]
pub struct BeliefConfig {
    /// belief 窓の半径 [m] (窓は 2×half_m 四方)。
    pub half_m: f64,
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
    /// `set_pose` (手動シード) で置く初期 belief の σ [m] / [deg]。
    pub init_sigma_xy_m: f64,
    pub init_sigma_theta_deg: f64,
    /// ビームごとの尤度の床 (完全ミスマッチでも重みを 0 にしない)。本家 likelihood
    /// field モデルの z_rand/z_max 混合に相当する 1 定数。
    pub z_min: f64,
    /// 補正で読む重みの相対しきい値 (max との比)。収束後の補正コストを belief の
    /// 広がりに比例させる (窓サイズには比例させない) ための枝刈り。
    pub weight_skip_ratio: f32,
    /// [`AdaptiveLocalizer`] 用: 観測一致度 EWMA がこれを割ると belief を 1 段
    /// 粗いレベルへ広げる (EMCL の expansion resetting 相当)。0 以下で多重解像度を
    /// 無効化 (L0 のみ = GridLocalizer と同じ固定窓)。GridLocalizer は無視する。
    pub expand_quality: f64,
    /// [`AdaptiveLocalizer`] 用: 全域レベルを min-plus (Viterbi / MAP) で回す
    /// (localize/viterbi.rs の doc)。窓レベルの追跡は sum-product のままで、
    /// ロスト中の全域推定だけが max-product に替わる。GridLocalizer は無視する。
    pub viterbi: bool,
}

impl Default for BeliefConfig {
    fn default() -> Self {
        Self {
            half_m: 2.5,
            sensor_sigma_m: 0.2,
            beam_step: 10,
            max_range_m: 25.0,
            motion_sigma_xy_m: 0.03,
            motion_sigma_theta_deg: 2.0,
            init_sigma_xy_m: 0.3,
            init_sigma_theta_deg: 10.0,
            z_min: 0.05,
            weight_skip_ratio: 1e-4,
            expand_quality: 0.25,
            viterbi: false,
        }
    }
}

/// belief の添字。θ 面優先 (`(it*nw + iy)*nw + ix`) — predict の xy シフトが
/// θ 面ごとの連続 2D 面で回るように。自由関数なのは、可変インデックスの中で
/// `&self` メソッドを呼べない (E0502) ため。
#[inline]
fn bidx(nw: i32, ix: i32, iy: i32, it: i32) -> usize {
    ((it * nw + iy) * nw + ix) as usize
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

/// 内蔵ヒストグラム MCL: native 地図格子上の 5m×5m×θ belief。
pub struct GridLocalizer {
    cfg: BeliefConfig,
    field: LikelihoodField,
    /// 窓の一辺 [セル] と θ ビン数・分解能。
    nw: i32,
    nt: i32,
    t_res_deg: f64,
    /// 窓左下の native セル座標 (地図原点基準)。
    wx0: i32,
    wy0: i32,
    /// belief 本体 (レイアウトは [`bidx`])。
    b: Vec<f32>,
    /// predict のシフト先スクラッチ (毎 tick の再確保を避ける)。
    tmp: Vec<f32>,
    initialized: bool,
    quality: f64,
}

impl GridLocalizer {
    /// `grid` は **native 解像度** (map_scale をかける前) の占有格子。
    /// `theta_bins` は VI と同じ `theta_cell_num` (360 を割り切る値)。
    pub fn new(grid: &OccupancyGrid, theta_bins: i32, cfg: BeliefConfig) -> Self {
        let field = LikelihoodField::from_grid(grid, cfg.sensor_sigma_m);
        let nw = ((2.0 * cfg.half_m / grid.resolution).ceil() as i32).max(3);
        let nt = theta_bins.max(1);
        let n = (nw as usize) * (nw as usize) * (nt as usize);
        Self {
            cfg,
            field,
            nw,
            nt,
            t_res_deg: 360.0 / nt as f64,
            wx0: 0,
            wy0: 0,
            b: vec![0.0; n],
            tmp: vec![0.0; n],
            initialized: false,
            quality: 0.0,
        }
    }

    /// belief のメモリ量 [MB] (起動ログ用)。
    pub fn belief_mb(&self) -> f64 {
        self.b.len() as f64 * 4.0 * 2.0 / 1e6
    }

    /// θ ビン中心の方位 [rad] (ビン k は [k·res, (k+1)·res) 度 — `pose_to_cell`
    /// の切り捨て規約に合わせ、中心はその +0.5 ビン)。
    #[inline]
    fn theta_center(&self, it: i32) -> f64 {
        ((it as f64 + 0.5) * self.t_res_deg).to_radians()
    }

    /// セル中心の世界座標。
    #[inline]
    fn cell_center(&self, ix: i32, iy: i32) -> (f64, f64) {
        let res = self.field.res;
        (
            self.field.ox + (self.wx0 + ix) as f64 * res + res * 0.5,
            self.field.oy + (self.wy0 + iy) as f64 * res + res * 0.5,
        )
    }

    fn normalize(&mut self) {
        let sum: f64 = self.b.iter().map(|&v| v as f64).sum();
        if sum > 0.0 {
            let inv = (1.0 / sum) as f32;
            for v in &mut self.b {
                *v *= inv;
            }
        }
    }

    /// 重み付き平均 (θ は円環平均)。合計 0 なら None。
    fn mean(&self) -> Option<PoseView> {
        let nw = self.nw;
        let (mut sw, mut sx, mut sy, mut sc, mut ss) = (0.0f64, 0.0, 0.0, 0.0, 0.0);
        for it in 0..self.nt {
            let th = self.theta_center(it);
            let (cos_t, sin_t) = (th.cos(), th.sin());
            for iy in 0..nw {
                for ix in 0..nw {
                    let w = self.b[bidx(nw, ix, iy, it)] as f64;
                    if w <= 0.0 {
                        continue;
                    }
                    let (cx, cy) = self.cell_center(ix, iy);
                    sw += w;
                    sx += w * cx;
                    sy += w * cy;
                    sc += w * cos_t;
                    ss += w * sin_t;
                }
            }
        }
        (sw > 0.0).then(|| PoseView { x: sx / sw, y: sy / sw, yaw_rad: ss.atan2(sc) })
    }

    /// 平均が窓中心から 2 セル以上ずれたら窓を整数シフトで置き直す。
    /// はみ出た質量は捨てる (窓を超えて広がった belief はどのみち lost 扱い)。
    fn recenter(&mut self) {
        let Some(m) = self.mean() else { return };
        let res = self.field.res;
        let cx = ((m.x - self.field.ox) / res).floor() as i32 - self.wx0;
        let cy = ((m.y - self.field.oy) / res).floor() as i32 - self.wy0;
        let (dx, dy) = (cx - self.nw / 2, cy - self.nw / 2);
        if dx.abs() < 2 && dy.abs() < 2 {
            return;
        }
        let nw = self.nw;
        self.tmp.iter_mut().for_each(|v| *v = 0.0);
        for it in 0..self.nt {
            for iy in 0..nw {
                let sy = iy + dy;
                if sy < 0 || sy >= nw {
                    continue;
                }
                for ix in 0..nw {
                    let sx = ix + dx;
                    if sx < 0 || sx >= nw {
                        continue;
                    }
                    self.tmp[bidx(nw, ix, iy, it)] = self.b[bidx(nw, sx, sy, it)];
                }
            }
        }
        std::mem::swap(&mut self.b, &mut self.tmp);
        self.wx0 += dx;
        self.wy0 += dy;
        self.normalize();
    }

    /// 物理拘束: free でないセル (壁・未知) の質量を落とす。全質量が消えるとき
    /// だけ何もしない — 非 free 地帯に取り残された belief の復帰は observe と
    /// 呼び出し側の再シードに任せる (0 除算・全滅 NaN をここで作らない)。
    fn apply_free_mask(&mut self) {
        let nw = self.nw;
        let mut kept = 0.0f64;
        for iy in 0..nw {
            for ix in 0..nw {
                if self.field.free_cell(self.wx0 + ix, self.wy0 + iy) {
                    for it in 0..self.nt {
                        kept += self.b[bidx(nw, ix, iy, it)] as f64;
                    }
                }
            }
        }
        if kept <= 0.0 {
            return;
        }
        for iy in 0..nw {
            for ix in 0..nw {
                if !self.field.free_cell(self.wx0 + ix, self.wy0 + iy) {
                    for it in 0..self.nt {
                        self.b[bidx(nw, ix, iy, it)] = 0.0;
                    }
                }
            }
        }
    }

    /// free セル上の最大重み仮説 (mode)。free 上に質量が無ければ None。
    fn mode_free(&self) -> Option<PoseView> {
        let nw = self.nw;
        let mut best: Option<(f32, i32, i32, i32)> = None;
        for it in 0..self.nt {
            for iy in 0..nw {
                for ix in 0..nw {
                    let w = self.b[bidx(nw, ix, iy, it)];
                    if w > 0.0
                        && best.map_or(true, |(bw, ..)| w > bw)
                        && self.field.free_cell(self.wx0 + ix, self.wy0 + iy)
                    {
                        best = Some((w, ix, iy, it));
                    }
                }
            }
        }
        best.map(|(_, ix, iy, it)| {
            let (x, y) = self.cell_center(ix, iy);
            PoseView { x, y, yaw_rad: self.theta_center(it) }
        })
    }

    /// 3 点カーネル [a, 1-2a, a] の a。σ [セル] 1 tick のランダムウォーク分散
    /// (2a セル²) を合わせる。0.25 で頭打ち (それ以上は 1 tick で表せない)。
    fn blur_a(sigma_cells: f64) -> f32 {
        ((sigma_cells * sigma_cells) / 2.0).clamp(0.0, 0.25) as f32
    }

    /// xy 両軸 + θ 軸の 3 点ぼかし。窓外は吸収境界 (最後に normalize で戻す)。
    fn blur(&mut self, a_xy: f32, a_t: f32) {
        let (nw, nt) = (self.nw, self.nt);
        if a_xy > 0.0 {
            for it in 0..nt {
                // x 軸。
                for iy in 0..nw {
                    let mut prev = 0.0f32;
                    for ix in 0..nw {
                        let cur = self.b[bidx(nw, ix, iy, it)];
                        let next =
                            if ix + 1 < nw { self.b[bidx(nw, ix + 1, iy, it)] } else { 0.0 };
                        self.b[bidx(nw, ix, iy, it)] =
                            cur * (1.0 - 2.0 * a_xy) + (prev + next) * a_xy;
                        prev = cur;
                    }
                }
                // y 軸。
                for ix in 0..nw {
                    let mut prev = 0.0f32;
                    for iy in 0..nw {
                        let cur = self.b[bidx(nw, ix, iy, it)];
                        let next =
                            if iy + 1 < nw { self.b[bidx(nw, ix, iy + 1, it)] } else { 0.0 };
                        self.b[bidx(nw, ix, iy, it)] =
                            cur * (1.0 - 2.0 * a_xy) + (prev + next) * a_xy;
                        prev = cur;
                    }
                }
            }
        }
        if a_t > 0.0 && nt > 2 {
            // θ 軸 (円環)。列単位で一時列を取って回す。
            for iy in 0..nw {
                for ix in 0..nw {
                    let col: Vec<f32> =
                        (0..nt).map(|it| self.b[bidx(nw, ix, iy, it)]).collect();
                    for it in 0..nt {
                        let prev = col[((it + nt - 1) % nt) as usize];
                        let next = col[((it + 1) % nt) as usize];
                        self.b[bidx(nw, ix, iy, it)] =
                            col[it as usize] * (1.0 - 2.0 * a_t) + (prev + next) * a_t;
                    }
                }
            }
        }
    }
}

impl Localizer for GridLocalizer {
    fn name(&self) -> &'static str {
        "grid"
    }

    /// 手動シード: belief を pose 中心のガウスで置き直す (窓も pose 中心へ)。
    fn set_pose(&mut self, pose: PoseView) {
        let res = self.field.res;
        let nw = self.nw;
        self.wx0 = ((pose.x - self.field.ox) / res).floor() as i32 - nw / 2;
        self.wy0 = ((pose.y - self.field.oy) / res).floor() as i32 - nw / 2;
        let s_xy = self.cfg.init_sigma_xy_m.max(res);
        let s_t = self.cfg.init_sigma_theta_deg.max(self.t_res_deg);
        for it in 0..self.nt {
            // 円環上の角度差。
            let dt_deg = {
                let d = (self.theta_center(it).to_degrees() - pose.yaw_rad.to_degrees())
                    .rem_euclid(360.0);
                d.min(360.0 - d)
            };
            let wt = (-dt_deg * dt_deg / (2.0 * s_t * s_t)).exp();
            for iy in 0..nw {
                for ix in 0..nw {
                    let (cx, cy) = self.cell_center(ix, iy);
                    let d2 = (cx - pose.x).powi(2) + (cy - pose.y).powi(2);
                    self.b[bidx(nw, ix, iy, it)] =
                        (wt * (-d2 / (2.0 * s_xy * s_xy)).exp()) as f32;
                }
            }
        }
        self.apply_free_mask();
        self.initialized = true;
        self.normalize();
    }

    /// 予測: 各 θ 面をその方位の移動量ぶん双線形シフト → θ 円環シフト → ぼかし。
    fn predict(&mut self, v: f64, w_deg: f64, dt: f64) {
        if !self.initialized {
            return;
        }
        let (nw, nt) = (self.nw, self.nt);
        let res = self.field.res;

        // xy シフト (θ 面ごとに移動ベクトルが違う)。後方サンプリングの双線形。
        self.tmp.iter_mut().for_each(|x| *x = 0.0);
        for it in 0..nt {
            let th = self.theta_center(it);
            let fx = (v * dt * th.cos() / res) as f32;
            let fy = (v * dt * th.sin() / res) as f32;
            for iy in 0..nw {
                for ix in 0..nw {
                    let (ux, uy) = (ix as f32 - fx, iy as f32 - fy);
                    let (x0, y0) = (ux.floor() as i32, uy.floor() as i32);
                    let (ax, ay) = (ux - x0 as f32, uy - y0 as f32);
                    let mut acc = 0.0f32;
                    for (oy, wy) in [(0, 1.0 - ay), (1, ay)] {
                        for (ox, wx) in [(0, 1.0 - ax), (1, ax)] {
                            let (sx, sy) = (x0 + ox, y0 + oy);
                            if sx >= 0 && sx < nw && sy >= 0 && sy < nw {
                                acc += self.b[bidx(nw, sx, sy, it)] * wx * wy;
                            }
                        }
                    }
                    self.tmp[bidx(nw, ix, iy, it)] = acc;
                }
            }
        }
        std::mem::swap(&mut self.b, &mut self.tmp);

        // θ シフト (円環、隣接ビン線形補間)。
        let ft = w_deg * dt / self.t_res_deg;
        if ft != 0.0 {
            self.tmp.iter_mut().for_each(|x| *x = 0.0);
            for it in 0..nt {
                let u = it as f64 - ft;
                let i0 = u.floor();
                let f = (u - i0) as f32;
                let s0 = (i0 as i32).rem_euclid(nt);
                let s1 = (s0 + 1) % nt;
                for iy in 0..nw {
                    for ix in 0..nw {
                        self.tmp[bidx(nw, ix, iy, it)] = self.b[bidx(nw, ix, iy, s0)]
                            * (1.0 - f)
                            + self.b[bidx(nw, ix, iy, s1)] * f;
                    }
                }
            }
            std::mem::swap(&mut self.b, &mut self.tmp);
        }

        self.blur(
            Self::blur_a(self.cfg.motion_sigma_xy_m / res),
            Self::blur_a(self.cfg.motion_sigma_theta_deg / self.t_res_deg),
        );
        // 拡散が壁・未知へ漏らした質量を毎 tick 回収する (物理拘束)。
        self.apply_free_mask();
        self.normalize();
    }

    /// 補正: 間引いたビームの端点を尤度場で評価し、重みへ乗じる。
    /// `set_local_cost` と同じ世界角規約 (ビーム角 = yaw + angle_min + i·inc)。
    fn observe(&mut self, scan: &LaserScan) {
        if !self.initialized {
            return;
        }
        let step = self.cfg.beam_step.max(1);
        let beams: Vec<(f64, f64)> = scan
            .ranges
            .iter()
            .enumerate()
            .step_by(step)
            .filter_map(|(i, &r)| {
                (r.is_finite() && r > 0.0 && r <= self.cfg.max_range_m)
                    .then(|| (scan.angle_min + scan.angle_increment * i as f64, r))
            })
            .collect();
        if beams.is_empty() {
            return;
        }

        let nw = self.nw;
        let maxw = self.b.iter().cloned().fold(0.0f32, f32::max);
        let thr = maxw * self.cfg.weight_skip_ratio;
        let z_min = self.cfg.z_min;
        let mut quality = 0.0f64;
        for it in 0..self.nt {
            let th = self.theta_center(it);
            for iy in 0..nw {
                for ix in 0..nw {
                    let i = bidx(nw, ix, iy, it);
                    let w = self.b[i];
                    if w <= thr {
                        self.b[i] = 0.0;
                        continue;
                    }
                    // 物理拘束: 壁・未知の中の仮説はビーム評価するまでもなく棄却。
                    if !self.field.free_cell(self.wx0 + ix, self.wy0 + iy) {
                        self.b[i] = 0.0;
                        continue;
                    }
                    let (cx, cy) = self.cell_center(ix, iy);
                    let mut prod = 1.0f64;
                    let mut lsum = 0.0f64;
                    for &(ba, r) in &beams {
                        let a = th + ba;
                        let l = self.field.at(cx + r * a.cos(), cy + r * a.sin());
                        lsum += l;
                        prod *= z_min + (1.0 - z_min) * l;
                    }
                    // belief (正規化済み) 加重のビーム平均尤度 = 観測一致度。
                    quality += w as f64 * (lsum / beams.len() as f64);
                    self.b[i] = (w as f64 * prod) as f32;
                }
            }
        }
        self.quality = quality;
        self.normalize();
        self.recenter();
    }

    fn pose(&self) -> Option<PoseView> {
        let m = self.initialized.then(|| self.mean()).flatten()?;
        if self.field.free_at(m.x, m.y) {
            return Some(m);
        }
        // 多峰・ドーナツ状 belief の平均は穴 (壁・未知) に落ちる — 実在する
        // 仮説 (free 上の mode) へ吸着して返す。
        self.mode_free().or(Some(m))
    }

    fn quality(&self) -> f64 {
        self.quality
    }

    fn top_cells(&self, k: usize) -> Vec<(PoseView, f64)> {
        if !self.initialized || k == 0 {
            return Vec::new();
        }
        let nw = self.nw;
        let plane = (nw * nw) as usize;
        top_k_weights(&self.b, k)
            .into_iter()
            .map(|(w, i)| {
                let (it, rem) = ((i / plane) as i32, i % plane);
                let (iy, ix) = ((rem / nw as usize) as i32, (rem % nw as usize) as i32);
                let (x, y) = self.cell_center(ix, iy);
                (PoseView { x, y, yaw_rad: self.theta_center(it) }, w as f64)
            })
            .collect()
    }
}

// ═══ AdaptiveLocalizer — 多重解像度 belief + expansion resetting ═══

/// L0 (= [`GridLocalizer`] 相当の窓) より上の粗レベルの
/// (xy 解像度倍率, θ ビン除数, ビーム間引き倍率)。最終レベルは常に地図全域。
/// 窓のセル数は全レベル共通なので、物理的な窓幅は解像度倍率ぶん広がる
/// (既定 half_m 2.5 なら L0 5m → L1 20m → L2 全域)。
// ponytail: レベル構成は固定 2 段 — 地図/センサごとの調整が要るなら BeliefConfig へ昇格。
const COARSE: [(i32, i32, usize); 2] = [(4, 2, 2), (10, 3, 4)];
/// 観測一致度 EWMA の平滑係数。
const EWMA_BETA: f64 = 0.3;
/// レベル遷移後、次の遷移までに最低限挟む observe 回数 (振動防止)。
const MIN_DWELL: u32 = 3;
/// ESS (有効セル数 = 1/Σb²) がこれを下回ったら 1 段細かいレベルへ降りる。
const ESS_CONTRACT: f64 = 50.0;
/// expansion で free 一様分布と混ぜる質量比 (EMCL の resetting 相当)。
const MIX_UNIFORM: f32 = 0.5;
/// 能動的再定位 ([`Localizer::reloc_targets`]): 判別に使う上位モード数、候補変位の
/// 半径 [m]、ロボット系方位の分割数、署名リングの半径 [m]。
// ponytail: 定数 4 個 — 実地図でスケール調整が要るなら BeliefConfig へ昇格。
const RELOC_MODES: usize = 4;
const RELOC_RADII: [f64; 2] = [1.5, 3.0];
const RELOC_HEADINGS: usize = 12;
const RELOC_SIG_R: f64 = 1.0;

/// 1 レベルぶんの幾何。belief 本体は持たない (バッファは最大レベル分を共用し、
/// 一度にアクティブなレベルは 1 つ — メモリと CPU を最大レベルで頭打ちにする)。
struct Level {
    res: f64,
    nt: i32,
    t_res_deg: f64,
    /// belief 窓の大きさ [このレベルのセル]。`whole` なら地図全域と一致。
    nx: i32,
    ny: i32,
    whole: bool,
    beam_step: usize,
    /// θ 粗視化の接線誤差 r·sin(τ/2) が σ を大きく超える遠距離ビームを弾く
    /// レンジ上限 [m] (L0 は無制限)。
    range_cap: f64,
    /// 地図全域のこのレベルでのセル数。
    map_w: i32,
    map_h: i32,
    /// 全域 free マスク (uniform シード用)。L0 は使わないので空。
    free: Vec<bool>,
}

/// 汎用添字 (θ 面優先、長方形窓)。[`bidx`] の nx≠ny 版。
#[inline]
fn bidx2(nx: i32, ny: i32, ix: i32, iy: i32, it: i32) -> usize {
    ((it * ny + iy) * nx + ix) as usize
}

/// belief バッファ `b[..n]` の上位 `k` セルを (重み, 添字) の降順で返す。
/// [`Localizer::top_cells`] の共通部 (Grid / Adaptive で幾何だけが違う)。
fn top_k_weights(b: &[f32], k: usize) -> Vec<(f32, usize)> {
    let mut cells: Vec<(f32, usize)> =
        b.iter().enumerate().filter(|&(_, &w)| w > 0.0).map(|(i, &w)| (w, i)).collect();
    if cells.len() > k {
        cells.select_nth_unstable_by(k - 1, |a, b| b.0.total_cmp(&a.0));
        cells.truncate(k);
    }
    cells.sort_by(|a, b| b.0.total_cmp(&a.0));
    cells
}

/// レベルのセル (窓座標) が free か。L0 は native の bitset、粗レベルは構築時の
/// 集約マスク (native free を 1 つでも含めば free)。自由関数なのは borrow 分割
/// (`self.b` の可変ループ内から呼ぶ) のため。
#[inline]
fn level_free(field: &LikelihoodField, l: &Level, wx0: i32, wy0: i32, ix: i32, iy: i32) -> bool {
    if l.free.is_empty() {
        return field.free_cell(wx0 + ix, wy0 + iy);
    }
    let (x, y) = (wx0 + ix, wy0 + iy);
    if x < 0 || y < 0 || x >= l.map_w || y >= l.map_h {
        return false;
    }
    l.free[(y * l.map_w + x) as usize]
}

/// 多重解像度ヒストグラム MCL。通常走行は [`GridLocalizer`] と同じ細窓 (L0)。
/// 観測一致度の EWMA が [`BeliefConfig::expand_quality`] を割ると belief を
/// 1 段粗いレベルへ射影して free 一様と混ぜ (expansion resetting)、ESS が
/// 十分小さくなったら mode を中心に 1 段細かいレベルへ降りる。最終レベルは
/// 地図全域なので、誘拐 (持ち上げ移動) からも数十スキャンで再定位する。
///
/// - ロスト中 (レベル > 0) は [`Localizer::pose`] が `None` — 多峰 belief の
///   平均は台のどこでもない点に落ちるため。呼び出し側 (vi_planner の follow
///   ループ / viola_bench) は既存の「pose なし」停止経路に乗り、スキャンの
///   購読は回り続けるので復帰は走行系と独立に進む。
/// - 未シードで最初のスキャンが来たら全域一様で開始する (大域初期化)。
/// - contract の mode が外れ仮説なら quality が再び落ちて expansion に戻る
///   (仮説を 1 つずつ検証する EMCL 流の挙動)。
// ponytail: VI 遷移テーブル駆動の predict (計画と同一 P) は未接続 — 全レベル
// cmd_vel の連続シフト。粗レベルは行動 1 ステップ粒度でテーブルを回すのが本来形。
pub struct AdaptiveLocalizer {
    cfg: BeliefConfig,
    field: LikelihoodField,
    levels: Vec<Level>,
    cur: usize,
    /// 現レベルの窓左下 [そのレベルのセル、地図原点基準]。whole レベルでは 0。
    wx0: i32,
    wy0: i32,
    /// belief と作業バッファ。最大レベルの寸法で確保し、先頭 n_active だけ使う。
    b: Vec<f32>,
    tmp: Vec<f32>,
    initialized: bool,
    quality: f64,
    q_ewma: f64,
    /// 直近のレベル遷移からの observe 回数。
    dwell: u32,
    /// min-plus (Viterbi) モードの全域レベル状態 ([`BeliefConfig::viterbi`] の
    /// ときだけ Some)。実装は localize/viterbi.rs。
    vit: Option<VitState>,
}

impl AdaptiveLocalizer {
    /// `grid` は **native 解像度** (map_scale をかける前) の占有格子
    /// (data == 0 が free)。`theta_bins` は VI と同じ `theta_cell_num`。
    pub fn new(grid: &OccupancyGrid, theta_bins: i32, cfg: BeliefConfig) -> Self {
        let field = LikelihoodField::from_grid(grid, cfg.sensor_sigma_m);
        let nt0 = theta_bins.max(1);
        let win = ((2.0 * cfg.half_m / grid.resolution).ceil() as i32).max(3);
        let (w, h) = (grid.width, grid.height);
        let mut levels = vec![Level {
            res: grid.resolution,
            nt: nt0,
            t_res_deg: 360.0 / nt0 as f64,
            nx: win.min(w),
            ny: win.min(h),
            whole: win >= w && win >= h,
            beam_step: cfg.beam_step.max(1),
            range_cap: f64::INFINITY,
            map_w: w,
            map_h: h,
            free: Vec::new(),
        }];
        if cfg.expand_quality > 0.0 {
            for (i, &(m, tdiv, bmult)) in COARSE.iter().enumerate() {
                let map_w = (w + m - 1) / m;
                let map_h = (h + m - 1) / m;
                let nt = (nt0 / tdiv).max(4);
                let t_res_deg = 360.0 / nt as f64;
                let whole = i == COARSE.len() - 1 || (win >= map_w && win >= map_h);
                let (nx, ny) =
                    if whole { (map_w, map_h) } else { (win.min(map_w), win.min(map_h)) };
                let mut free = vec![false; (map_w * map_h) as usize];
                for y in 0..h {
                    for x in 0..w {
                        if grid.data[(y * w + x) as usize] == 0 {
                            free[((y / m) * map_w + x / m) as usize] = true;
                        }
                    }
                }
                let half_rad = (t_res_deg / 2.0).to_radians();
                let range_cap = (2.0 * cfg.sensor_sigma_m / half_rad.sin()).max(3.0);
                levels.push(Level {
                    res: grid.resolution * m as f64,
                    nt,
                    t_res_deg,
                    nx,
                    ny,
                    whole,
                    beam_step: cfg.beam_step.max(1) * bmult,
                    range_cap,
                    map_w,
                    map_h,
                    free,
                });
            }
        }
        let nmax = levels.iter().map(|l| (l.nx * l.ny * l.nt) as usize).max().unwrap();
        let vit = (cfg.viterbi && levels.len() > 1).then(|| VitState::new(&levels));
        Self {
            cfg,
            field,
            levels,
            cur: 0,
            wx0: 0,
            wy0: 0,
            b: vec![0.0; nmax],
            tmp: vec![0.0; nmax],
            initialized: false,
            quality: 0.0,
            q_ewma: 0.0,
            dwell: 0,
            vit,
        }
    }

    /// belief バッファのメモリ量 [MB] (起動ログ用)。
    pub fn belief_mb(&self) -> f64 {
        self.b.len() as f64 * 4.0 * 2.0 / 1e6
    }

    /// レベル数 (起動ログ用)。
    pub fn num_levels(&self) -> usize {
        self.levels.len()
    }

    /// 現在のレベル (0 = 通常走行の細窓)。観測・テスト用。
    pub fn level(&self) -> usize {
        self.cur
    }

    fn n_active(&self) -> usize {
        let l = &self.levels[self.cur];
        (l.nx * l.ny * l.nt) as usize
    }

    fn normalize(&mut self) {
        let n = self.n_active();
        let sum: f64 = self.b[..n].iter().map(|&v| v as f64).sum();
        if sum > 0.0 && sum.is_finite() {
            // ミスマッチ時のビーム積は f32 subnormal 域まで沈む。そこで
            // `(1/sum) as f32` を掛けると inv が ∞ に飽和して 0×∞ = NaN が belief に
            // 混ざり、EWMA が NaN → レベル遷移が二度と発火しなくなる。f64 で割る。
            for v in &mut self.b[..n] {
                *v = ((*v as f64) / sum) as f32;
            }
        } else if !sum.is_finite() {
            // ∞/NaN が混ざったら復旧不能 — 0 へ落とし、q 低下 → expansion の
            // 一様リセットに回復を任せる。
            self.b[..n].fill(0.0);
        }
    }

    /// 重み付き平均 (θ は円環平均)。合計 0 なら None。
    fn mean(&self) -> Option<PoseView> {
        let l = &self.levels[self.cur];
        let (nx, ny, nt, res, t_res) = (l.nx, l.ny, l.nt, l.res, l.t_res_deg);
        let (ox, oy) = (self.field.ox, self.field.oy);
        let (mut sw, mut sx, mut sy, mut sc, mut ss) = (0.0f64, 0.0, 0.0, 0.0, 0.0);
        for it in 0..nt {
            let th = ((it as f64 + 0.5) * t_res).to_radians();
            let (cos_t, sin_t) = (th.cos(), th.sin());
            for iy in 0..ny {
                let cy = oy + (self.wy0 + iy) as f64 * res + res * 0.5;
                for ix in 0..nx {
                    let wgt = self.b[bidx2(nx, ny, ix, iy, it)] as f64;
                    if wgt <= 0.0 {
                        continue;
                    }
                    sw += wgt;
                    sx += wgt * (ox + (self.wx0 + ix) as f64 * res + res * 0.5);
                    sy += wgt * cy;
                    sc += wgt * cos_t;
                    ss += wgt * sin_t;
                }
            }
        }
        (sw > 0.0).then(|| PoseView { x: sx / sw, y: sy / sw, yaw_rad: ss.atan2(sc) })
    }

    /// 物理拘束: free でないセルの質量を落とす ([`GridLocalizer::apply_free_mask`]
    /// と同じ規約 — 全質量が消えるときだけ何もしない)。
    fn apply_free_mask(&mut self) {
        let l = &self.levels[self.cur];
        let (nx, ny, nt) = (l.nx, l.ny, l.nt);
        let (wx0, wy0) = (self.wx0, self.wy0);
        let mut kept = 0.0f64;
        for iy in 0..ny {
            for ix in 0..nx {
                if level_free(&self.field, l, wx0, wy0, ix, iy) {
                    for it in 0..nt {
                        kept += self.b[bidx2(nx, ny, ix, iy, it)] as f64;
                    }
                }
            }
        }
        if kept <= 0.0 {
            return;
        }
        for iy in 0..ny {
            for ix in 0..nx {
                if !level_free(&self.field, l, wx0, wy0, ix, iy) {
                    for it in 0..nt {
                        self.b[bidx2(nx, ny, ix, iy, it)] = 0.0;
                    }
                }
            }
        }
    }

    /// 現レベルの free セル上の最大重み仮説 (mode)。free 上に質量が無ければ None。
    fn mode_free(&self) -> Option<PoseView> {
        let l = &self.levels[self.cur];
        let (nx, ny, nt, res, t_res) = (l.nx, l.ny, l.nt, l.res, l.t_res_deg);
        let mut best: Option<(f32, i32, i32, i32)> = None;
        for it in 0..nt {
            for iy in 0..ny {
                for ix in 0..nx {
                    let w = self.b[bidx2(nx, ny, ix, iy, it)];
                    if w > 0.0
                        && best.map_or(true, |(bw, ..)| w > bw)
                        && level_free(&self.field, l, self.wx0, self.wy0, ix, iy)
                    {
                        best = Some((w, ix, iy, it));
                    }
                }
            }
        }
        best.map(|(_, ix, iy, it)| PoseView {
            x: self.field.ox + (self.wx0 + ix) as f64 * res + res * 0.5,
            y: self.field.oy + (self.wy0 + iy) as f64 * res + res * 0.5,
            yaw_rad: ((it as f64 + 0.5) * t_res).to_radians(),
        })
    }

    /// 有効セル数 ESS = 1/Σb² (正規化済み前提)。集中度の指標。
    fn ess(&self) -> f64 {
        let n = self.n_active();
        let s2: f64 = self.b[..n].iter().map(|&v| (v as f64) * (v as f64)).sum();
        if s2 > 0.0 {
            1.0 / s2
        } else {
            0.0
        }
    }

    /// 現レベルの窓を pose 中心に置き (whole は不動)、ガウスで belief を張り直す。
    fn seed_gaussian(&mut self, pose: PoseView, s_xy_m: f64, s_t_deg: f64) {
        let (nx, ny, nt, res, t_res, whole, map_w, map_h) = {
            let l = &self.levels[self.cur];
            (l.nx, l.ny, l.nt, l.res, l.t_res_deg, l.whole, l.map_w, l.map_h)
        };
        if whole {
            self.wx0 = 0;
            self.wy0 = 0;
        } else {
            let cx = ((pose.x - self.field.ox) / res).floor() as i32;
            let cy = ((pose.y - self.field.oy) / res).floor() as i32;
            self.wx0 = (cx - nx / 2).clamp(0, (map_w - nx).max(0));
            self.wy0 = (cy - ny / 2).clamp(0, (map_h - ny).max(0));
        }
        let s_xy = s_xy_m.max(res);
        let s_t = s_t_deg.max(t_res);
        let (ox, oy) = (self.field.ox, self.field.oy);
        let (wx0, wy0) = (self.wx0, self.wy0);
        for it in 0..nt {
            let dt_deg = {
                let d = (((it as f64 + 0.5) * t_res) - pose.yaw_rad.to_degrees())
                    .rem_euclid(360.0);
                d.min(360.0 - d)
            };
            let wt = (-dt_deg * dt_deg / (2.0 * s_t * s_t)).exp();
            for iy in 0..ny {
                let cy = oy + (wy0 + iy) as f64 * res + res * 0.5;
                for ix in 0..nx {
                    let cx = ox + (wx0 + ix) as f64 * res + res * 0.5;
                    let d2 = (cx - pose.x).powi(2) + (cy - pose.y).powi(2);
                    self.b[bidx2(nx, ny, ix, iy, it)] =
                        (wt * (-d2 / (2.0 * s_xy * s_xy)).exp()) as f32;
                }
            }
        }
        self.apply_free_mask();
        self.normalize();
    }

    /// 全域レベル `to` を free 一様で張る (大域初期化 / 射影が空のときの復帰)。
    fn enter_uniform(&mut self, to: usize) {
        self.cur = to;
        self.wx0 = 0;
        self.wy0 = 0;
        let (nx, ny, nt) = {
            let l = &self.levels[to];
            (l.nx, l.ny, l.nt)
        };
        let n = (nx * ny * nt) as usize;
        self.b[..n].fill(0.0);
        let free = &self.levels[to].free;
        for iy in 0..ny {
            for ix in 0..nx {
                if !free.is_empty() && !free[(iy * nx + ix) as usize] {
                    continue;
                }
                for it in 0..nt {
                    self.b[bidx2(nx, ny, ix, iy, it)] = 1.0;
                }
            }
        }
        self.normalize();
        if self.vit_on() {
            self.vit_enter();
        }
        self.q_ewma = self.cfg.expand_quality;
        self.dwell = 0;
    }

    /// 1 段粗いレベルへ広げる: 現 belief を射影し、free 一様と [`MIX_UNIFORM`] で
    /// 混ぜる (EMCL の expansion resetting)。窓レベルへは現平均を中心に置く。
    fn expand(&mut self, to: usize) {
        let center = self.mean();
        let (onx, ony, ont, ores, otres) = {
            let l = &self.levels[self.cur];
            (l.nx, l.ny, l.nt, l.res, l.t_res_deg)
        };
        let (owx0, owy0) = (self.wx0, self.wy0);
        let (nnx, nny, nnt, nres, ntres, nwhole, nmap_w, nmap_h) = {
            let l = &self.levels[to];
            (l.nx, l.ny, l.nt, l.res, l.t_res_deg, l.whole, l.map_w, l.map_h)
        };
        let (ox, oy) = (self.field.ox, self.field.oy);
        let (nwx0, nwy0) = if nwhole {
            (0, 0)
        } else {
            let c = center.unwrap_or(PoseView {
                x: ox + (owx0 + onx / 2) as f64 * ores,
                y: oy + (owy0 + ony / 2) as f64 * ores,
                yaw_rad: 0.0,
            });
            let cx = ((c.x - ox) / nres).floor() as i32;
            let cy = ((c.y - oy) / nres).floor() as i32;
            (
                (cx - nnx / 2).clamp(0, (nmap_w - nnx).max(0)),
                (cy - nny / 2).clamp(0, (nmap_h - nny).max(0)),
            )
        };
        let nn = (nnx * nny * nnt) as usize;
        self.tmp[..nn].fill(0.0);
        // 射影: 旧セル中心の世界座標 → 新レベルのセル、θ はビン中心 → 新ビン。
        {
            let (b, tmp) = (&self.b, &mut self.tmp);
            for it in 0..ont {
                let nit = ((((it as f64 + 0.5) * otres) / ntres).floor() as i32)
                    .rem_euclid(nnt);
                for iy in 0..ony {
                    let wy = oy + (owy0 + iy) as f64 * ores + ores * 0.5;
                    let jy = ((wy - oy) / nres).floor() as i32 - nwy0;
                    if jy < 0 || jy >= nny {
                        continue;
                    }
                    for ix in 0..onx {
                        let w = b[bidx2(onx, ony, ix, iy, it)];
                        if w <= 0.0 {
                            continue;
                        }
                        let wx = ox + (owx0 + ix) as f64 * ores + ores * 0.5;
                        let jx = ((wx - ox) / nres).floor() as i32 - nwx0;
                        if jx < 0 || jx >= nnx {
                            continue;
                        }
                        tmp[bidx2(nnx, nny, jx, jy, nit)] += w;
                    }
                }
            }
        }
        // (1-α)·射影 + α·free 一様。射影が空なら全部一様。
        let s: f64 = self.tmp[..nn].iter().map(|&v| v as f64).sum();
        let alpha = if s > 0.0 { MIX_UNIFORM } else { 1.0 };
        if s > 0.0 {
            let k = ((1.0 - alpha as f64) / s) as f32;
            for v in &mut self.tmp[..nn] {
                *v *= k;
            }
        }
        let free_cnt = {
            let free = &self.levels[to].free;
            let mut c = 0usize;
            for jy in 0..nny {
                for jx in 0..nnx {
                    if free.is_empty()
                        || free[((nwy0 + jy) * nmap_w + (nwx0 + jx)) as usize]
                    {
                        c += 1;
                    }
                }
            }
            c * nnt as usize
        };
        if free_cnt == 0 {
            // 窓内に free が 1 つも無い — 窓全体に撒く (自己修復)。
            let u = alpha / nn as f32;
            for v in &mut self.tmp[..nn] {
                *v += u;
            }
        } else {
            let u = alpha / free_cnt as f32;
            let free = &self.levels[to].free;
            for jy in 0..nny {
                for jx in 0..nnx {
                    if free.is_empty()
                        || free[((nwy0 + jy) * nmap_w + (nwx0 + jx)) as usize]
                    {
                        for it in 0..nnt {
                            self.tmp[bidx2(nnx, nny, jx, jy, it)] += u;
                        }
                    }
                }
            }
        }
        std::mem::swap(&mut self.b, &mut self.tmp);
        self.cur = to;
        self.wx0 = nwx0;
        self.wy0 = nwy0;
        self.normalize();
        if self.vit_on() {
            self.vit_enter();
        }
        self.q_ewma = self.cfg.expand_quality;
        self.dwell = 0;
    }

    /// 姿勢 1 点のビーム幾何平均尤度 (native 尤度場、レンジ制限なし)。contract の
    /// 仮説検証に使う — 粗い格子のスコアより桁違いに鋭い。
    fn pose_score(&self, p: PoseView, beams: &[(f64, f64)]) -> f64 {
        if beams.is_empty() {
            return 0.0;
        }
        let z = self.cfg.z_min;
        let mut ln = 0.0f64;
        for &(ba, r) in beams {
            let a = p.yaw_rad + ba;
            let l = self.field.at(p.x + r * a.cos(), p.y + r * a.sin());
            ln += (z + (1.0 - z) * l).ln();
        }
        (ln / beams.len() as f64).exp()
    }

    /// 上位モード最大 `k` 個 (xy で `min_sep` セル以上離す)。contract の候補列挙。
    fn top_modes(&self, k: usize, min_sep: i32) -> Vec<PoseView> {
        let l = &self.levels[self.cur];
        let (nx, ny, res, t_res) = (l.nx, l.ny, l.res, l.t_res_deg);
        let n = self.n_active();
        let maxw = self.b[..n].iter().cloned().fold(0.0f32, f32::max);
        if maxw <= 0.0 {
            return Vec::new();
        }
        let thr = maxw * 0.05;
        let mut cells: Vec<(f32, i32)> = self.b[..n]
            .iter()
            .enumerate()
            .filter(|&(_, &w)| w >= thr)
            .map(|(i, &w)| (w, i as i32))
            .collect();
        cells.sort_by(|a, b| b.0.total_cmp(&a.0));
        let plane = nx * ny;
        let mut picked: Vec<(i32, i32)> = Vec::new();
        let mut out = Vec::new();
        for (_, i) in cells {
            let (it, rem) = (i / plane, i % plane);
            let (iy, ix) = (rem / nx, rem % nx);
            if picked
                .iter()
                .any(|&(px, py)| (px - ix).abs() < min_sep && (py - iy).abs() < min_sep)
            {
                continue;
            }
            picked.push((ix, iy));
            out.push(PoseView {
                x: self.field.ox + (self.wx0 + ix) as f64 * res + res * 0.5,
                y: self.field.oy + (self.wy0 + iy) as f64 * res + res * 0.5,
                yaw_rad: ((it as f64 + 0.5) * t_res).to_radians(),
            });
            if out.len() >= k {
                break;
            }
        }
        out
    }

    /// 1 段細かいレベルへ降りる。粗い argmax をそのまま信じると、粗い格子で
    /// たまたま勝った外れ仮説へ決定論的にロックインする (同じスキャン → 同じ勝者 →
    /// 同じ失敗、の無限循環)。そこで上位モードを θ 微調整つきの 1 点スコアで
    /// 検証し、勝った仮説を中心に粗レベルのセル幅を σ にしてガウス再シードする。
    /// それでも外れなら quality が再び落ちて expansion に戻る。
    fn contract(&mut self, to: usize, scan: &LaserScan) {
        let cands = self.top_modes(8, 2);
        if cands.is_empty() {
            return;
        }
        let (ores, otres) = {
            let l = &self.levels[self.cur];
            (l.res, l.t_res_deg)
        };
        // 検証用ビームは最細レベルの間引きで、レンジ制限なし。
        let step = self.levels[0].beam_step;
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
        let mut best = (-1.0f64, cands[0]);
        for c in &cands {
            // θ はビン中心 ±τ/3 の 3 点で量子化誤差を補正する。
            for kof in [-1.0f64, 0.0, 1.0] {
                let p = PoseView {
                    yaw_rad: c.yaw_rad + (kof * otres / 3.0).to_radians(),
                    ..*c
                };
                let s = self.pose_score(p, &beams);
                if s > best.0 {
                    best = (s, p);
                }
            }
        }
        self.cur = to;
        self.seed_gaussian(best.1, ores, otres);
        self.q_ewma = self.q_ewma.max(self.cfg.expand_quality);
        self.dwell = 0;
    }

    fn maybe_transition(&mut self, scan: &LaserScan) {
        if self.dwell < MIN_DWELL {
            return;
        }
        if self.q_ewma < self.cfg.expand_quality && self.cur + 1 < self.levels.len() {
            self.expand(self.cur + 1);
        } else if self.cur > 0 && self.ess() < ESS_CONTRACT {
            self.contract(self.cur - 1, scan);
        }
    }

    /// 平均が窓中心から 2 セル以上ずれたら窓を整数シフトで置き直す (地図内に
    /// clamp)。はみ出た質量は捨てる。whole レベルでは呼ばれない。
    fn recenter(&mut self) {
        let (nx, ny, nt, res, map_w, map_h) = {
            let l = &self.levels[self.cur];
            (l.nx, l.ny, l.nt, l.res, l.map_w, l.map_h)
        };
        let Some(m) = self.mean() else { return };
        let cx = ((m.x - self.field.ox) / res).floor() as i32;
        let cy = ((m.y - self.field.oy) / res).floor() as i32;
        let dx = (cx - nx / 2).clamp(0, (map_w - nx).max(0)) - self.wx0;
        let dy = (cy - ny / 2).clamp(0, (map_h - ny).max(0)) - self.wy0;
        if dx.abs() < 2 && dy.abs() < 2 {
            return;
        }
        let n = (nx * ny * nt) as usize;
        self.tmp[..n].fill(0.0);
        {
            let (b, tmp) = (&self.b, &mut self.tmp);
            for it in 0..nt {
                for iy in 0..ny {
                    let sy = iy + dy;
                    if sy < 0 || sy >= ny {
                        continue;
                    }
                    for ix in 0..nx {
                        let sx = ix + dx;
                        if sx < 0 || sx >= nx {
                            continue;
                        }
                        tmp[bidx2(nx, ny, ix, iy, it)] = b[bidx2(nx, ny, sx, sy, it)];
                    }
                }
            }
        }
        std::mem::swap(&mut self.b, &mut self.tmp);
        self.wx0 += dx;
        self.wy0 += dy;
        self.normalize();
    }

    /// xy 両軸 + θ 軸の 3 点ぼかし ([`GridLocalizer::blur`] の nx≠ny 版)。
    fn blur(&mut self, a_xy: f32, a_t: f32) {
        let (nx, ny, nt) = {
            let l = &self.levels[self.cur];
            (l.nx, l.ny, l.nt)
        };
        if a_xy > 0.0 {
            for it in 0..nt {
                for iy in 0..ny {
                    let mut prev = 0.0f32;
                    for ix in 0..nx {
                        let cur = self.b[bidx2(nx, ny, ix, iy, it)];
                        let next =
                            if ix + 1 < nx { self.b[bidx2(nx, ny, ix + 1, iy, it)] } else { 0.0 };
                        self.b[bidx2(nx, ny, ix, iy, it)] =
                            cur * (1.0 - 2.0 * a_xy) + (prev + next) * a_xy;
                        prev = cur;
                    }
                }
                for ix in 0..nx {
                    let mut prev = 0.0f32;
                    for iy in 0..ny {
                        let cur = self.b[bidx2(nx, ny, ix, iy, it)];
                        let next =
                            if iy + 1 < ny { self.b[bidx2(nx, ny, ix, iy + 1, it)] } else { 0.0 };
                        self.b[bidx2(nx, ny, ix, iy, it)] =
                            cur * (1.0 - 2.0 * a_xy) + (prev + next) * a_xy;
                        prev = cur;
                    }
                }
            }
        }
        if a_t > 0.0 && nt > 2 {
            for iy in 0..ny {
                for ix in 0..nx {
                    let col: Vec<f32> =
                        (0..nt).map(|it| self.b[bidx2(nx, ny, ix, iy, it)]).collect();
                    for it in 0..nt {
                        let prev = col[((it + nt - 1) % nt) as usize];
                        let next = col[((it + 1) % nt) as usize];
                        self.b[bidx2(nx, ny, ix, iy, it)] =
                            col[it as usize] * (1.0 - 2.0 * a_t) + (prev + next) * a_t;
                    }
                }
            }
        }
    }
}

impl Localizer for AdaptiveLocalizer {
    fn name(&self) -> &'static str {
        if self.vit.is_some() {
            "viterbi"
        } else {
            "adaptive"
        }
    }

    /// 手動シード: L0 に降りて pose 中心のガウスで張り直す。
    fn set_pose(&mut self, pose: PoseView) {
        self.cur = 0;
        let (res0, tres0) = {
            let l = &self.levels[0];
            (l.res, l.t_res_deg)
        };
        self.seed_gaussian(
            pose,
            self.cfg.init_sigma_xy_m.max(res0),
            self.cfg.init_sigma_theta_deg.max(tres0),
        );
        self.initialized = true;
        self.q_ewma = 1.0;
        self.dwell = 0;
    }

    /// 予測: 現レベル上での連続シフト + ぼかし (GridLocalizer と同じ演算)。
    /// min-plus モードの全域レベルでは移動量を溜めるだけ O(1) (シフト・緩和は
    /// 次の observe がまとめて適用する)。
    fn predict(&mut self, v: f64, w_deg: f64, dt: f64) {
        if !self.initialized {
            return;
        }
        if self.vit_on() {
            let s = self.vit.as_mut().unwrap();
            s.pend_f += v * dt;
            s.pend_dt_deg += w_deg * dt;
            return;
        }
        let (nx, ny, nt, res, t_res) = {
            let l = &self.levels[self.cur];
            (l.nx, l.ny, l.nt, l.res, l.t_res_deg)
        };
        let n = (nx * ny * nt) as usize;

        // xy シフト (θ 面ごとに移動ベクトルが違う)。後方サンプリングの双線形。
        self.tmp[..n].fill(0.0);
        {
            let (b, tmp) = (&self.b, &mut self.tmp);
            for it in 0..nt {
                let th = ((it as f64 + 0.5) * t_res).to_radians();
                let fx = (v * dt * th.cos() / res) as f32;
                let fy = (v * dt * th.sin() / res) as f32;
                for iy in 0..ny {
                    for ix in 0..nx {
                        let (ux, uy) = (ix as f32 - fx, iy as f32 - fy);
                        let (x0, y0) = (ux.floor() as i32, uy.floor() as i32);
                        let (ax, ay) = (ux - x0 as f32, uy - y0 as f32);
                        let mut acc = 0.0f32;
                        for (oy2, wy) in [(0, 1.0 - ay), (1, ay)] {
                            for (ox2, wx) in [(0, 1.0 - ax), (1, ax)] {
                                let (sx, sy) = (x0 + ox2, y0 + oy2);
                                if sx >= 0 && sx < nx && sy >= 0 && sy < ny {
                                    acc += b[bidx2(nx, ny, sx, sy, it)] * wx * wy;
                                }
                            }
                        }
                        tmp[bidx2(nx, ny, ix, iy, it)] = acc;
                    }
                }
            }
        }
        std::mem::swap(&mut self.b, &mut self.tmp);

        // θ シフト (円環、隣接ビン線形補間)。
        let ft = w_deg * dt / t_res;
        if ft != 0.0 {
            self.tmp[..n].fill(0.0);
            {
                let (b, tmp) = (&self.b, &mut self.tmp);
                for it in 0..nt {
                    let u = it as f64 - ft;
                    let i0 = u.floor();
                    let f = (u - i0) as f32;
                    let s0 = (i0 as i32).rem_euclid(nt);
                    let s1 = (s0 + 1) % nt;
                    for iy in 0..ny {
                        for ix in 0..nx {
                            tmp[bidx2(nx, ny, ix, iy, it)] = b[bidx2(nx, ny, ix, iy, s0)]
                                * (1.0 - f)
                                + b[bidx2(nx, ny, ix, iy, s1)] * f;
                        }
                    }
                }
            }
            std::mem::swap(&mut self.b, &mut self.tmp);
        }

        self.blur(
            GridLocalizer::blur_a(self.cfg.motion_sigma_xy_m / res),
            GridLocalizer::blur_a(self.cfg.motion_sigma_theta_deg / t_res),
        );
        // 拡散が壁・未知へ漏らした質量を毎 tick 回収する (物理拘束)。
        self.apply_free_mask();
        self.normalize();
    }

    /// 補正 + レベル遷移。未シードなら全域一様で開始する (大域初期化)。
    fn observe(&mut self, scan: &LaserScan) {
        if !self.initialized {
            if self.levels.len() > 1 {
                let last = self.levels.len() - 1;
                self.enter_uniform(last);
                self.initialized = true;
            } else {
                return;
            }
        }
        let (nx, ny, nt, res, t_res, step, cap, whole) = {
            let l = &self.levels[self.cur];
            (l.nx, l.ny, l.nt, l.res, l.t_res_deg, l.beam_step, l.range_cap, l.whole)
        };
        let max_r = self.cfg.max_range_m;
        let collect = |cap: f64| -> Vec<(f64, f64)> {
            scan.ranges
                .iter()
                .enumerate()
                .step_by(step)
                .filter_map(|(i, &r)| {
                    (r.is_finite() && r > 0.0 && r <= max_r && r <= cap)
                        .then(|| (scan.angle_min + scan.angle_increment * i as f64, r))
                })
                .collect()
        };
        // 粗レベルは θ 粗視化の接線誤差 (r·sin(τ/2)) が σ を壊す遠距離ビームを
        // 弾く。近距離ビームが 1 本も無いときだけ全ビームへフォールバック
        // (開けた場所で補正が完全に止まるのを防ぐ)。
        let mut beams = collect(cap);
        if beams.is_empty() && cap.is_finite() {
            beams = collect(f64::INFINITY);
        }
        if beams.is_empty() {
            return;
        }
        let quality = if self.vit_on() {
            // min-plus (Viterbi) モード: δ の更新 + b の実体化 (localize/viterbi.rs)。
            self.vit_observe(&beams)
        } else {
            let n = (nx * ny * nt) as usize;
            let maxw = self.b[..n].iter().cloned().fold(0.0f32, f32::max);
            let thr = maxw * self.cfg.weight_skip_ratio;
            let z_min = self.cfg.z_min;
            let (ox, oy) = (self.field.ox, self.field.oy);
            let (wx0, wy0) = (self.wx0, self.wy0);
            let lvl = &self.levels[self.cur];
            let mut quality = 0.0f64;
            for it in 0..nt {
                let th = ((it as f64 + 0.5) * t_res).to_radians();
                for iy in 0..ny {
                    let cy = oy + (wy0 + iy) as f64 * res + res * 0.5;
                    for ix in 0..nx {
                        let i = bidx2(nx, ny, ix, iy, it);
                        let w = self.b[i];
                        if w <= thr {
                            self.b[i] = 0.0;
                            continue;
                        }
                        // 物理拘束: 壁・未知の中の仮説はビーム評価するまでもなく棄却。
                        if !level_free(&self.field, lvl, wx0, wy0, ix, iy) {
                            self.b[i] = 0.0;
                            continue;
                        }
                        let cx = ox + (wx0 + ix) as f64 * res + res * 0.5;
                        let mut prod = 1.0f64;
                        for &(ba, r) in &beams {
                            let a = th + ba;
                            let l = self.field.at(cx + r * a.cos(), cy + r * a.sin());
                            prod *= z_min + (1.0 - z_min) * l;
                        }
                        // 観測一致度はビームの**幾何**平均 (= prod^(1/M))。算術平均だと
                        // ミスマッチでも「たまたま障害物帯に乗った端点」の寄与で 0.3 台に
                        // 浮き、ロスト検出のしきい値と分離できない。幾何平均は外れビームに
                        // 引きずられて z_min 側へ落ちるので、整合 (~0.5+) と乖離する。
                        quality += w as f64 * prod.powf(1.0 / beams.len() as f64);
                        self.b[i] = (w as f64 * prod) as f32;
                    }
                }
            }
            quality
        };
        self.quality = quality;
        self.q_ewma = (1.0 - EWMA_BETA) * self.q_ewma + EWMA_BETA * quality;
        self.normalize();
        if !whole {
            self.recenter();
        }
        self.dwell = self.dwell.saturating_add(1);
        self.maybe_transition(scan);
    }

    /// L0 に居るときだけ返す — ロスト中 (粗レベル) の多峰 belief の平均は台の
    /// どこでもない点に落ちるので None にし、呼び出し側の「pose なし」停止経路に
    /// 乗せる。
    fn pose(&self) -> Option<PoseView> {
        let m = (self.initialized && self.cur == 0).then(|| self.mean()).flatten()?;
        if self.field.free_at(m.x, m.y) {
            return Some(m);
        }
        // 多峰・ドーナツ状 belief の平均は穴 (壁・未知) に落ちる — 実在する
        // 仮説 (free 上の mode) へ吸着して返す。
        self.mode_free().or(Some(m))
    }

    fn quality(&self) -> f64 {
        self.quality
    }

    /// ロスト中の判別変位の選択。上位モード仮説 {pᵢ} はオドメトリ共有で
    /// 「同じロボット系変位 δ で一緒に動く」ので、δ 先の地図が仮説間で最も
    /// 違う δ* を選べば、そこへ走るだけで観測が仮説を判別する:
    ///
    ///   δ* = argmax_δ Σ_{i<j} ‖sig_i(δ) − sig_j(δ)‖₁
    ///
    /// sig は δ 先周りの尤度場リング標本 (仮説の向きに合わせて回す = 擬似的な
    /// 期待スキャン)。全仮説の δ 先が free な δ だけ許す (どの仮説が真でも
    /// 行ける行き先)。スコア 0 (完全対称) は空 — 受動復帰に任せる。
    fn reloc_targets(&self) -> Vec<(f64, f64)> {
        use std::f64::consts::PI;
        if !self.initialized || self.cur == 0 {
            return Vec::new();
        }
        let modes = self.top_modes(RELOC_MODES, 2);
        if modes.len() < 2 {
            return Vec::new();
        }
        let displaced = |p: &PoseView, dr: f64, dphi: f64| {
            let a = p.yaw_rad + dphi;
            (p.x + dr * a.cos(), p.y + dr * a.sin())
        };
        let mut best: Option<(f64, f64, f64)> = None; // (score, dr, dphi)
        for &dr in &RELOC_RADII {
            for k in 0..RELOC_HEADINGS {
                let dphi = k as f64 * (2.0 * PI / RELOC_HEADINGS as f64);
                if !modes.iter().all(|p| {
                    let (x, y) = displaced(p, dr, dphi);
                    self.field.free_at(x, y)
                }) {
                    continue;
                }
                let sigs: Vec<[f64; 8]> = modes
                    .iter()
                    .map(|p| {
                        let (x, y) = displaced(p, dr, dphi);
                        let mut s = [0.0; 8];
                        for (j, sv) in s.iter_mut().enumerate() {
                            let a = p.yaw_rad + j as f64 * (2.0 * PI / 8.0);
                            *sv = self.field.at(x + RELOC_SIG_R * a.cos(), y + RELOC_SIG_R * a.sin());
                        }
                        s
                    })
                    .collect();
                let mut score = 0.0;
                for i in 0..sigs.len() {
                    for j in (i + 1)..sigs.len() {
                        for m in 0..8 {
                            score += (sigs[i][m] - sigs[j][m]).abs();
                        }
                    }
                }
                if best.map_or(true, |(bs, ..)| score > bs) {
                    best = Some((score, dr, dphi));
                }
            }
        }
        match best {
            Some((score, dr, dphi)) if score > 0.0 => {
                modes.iter().map(|p| displaced(p, dr, dphi)).collect()
            }
            _ => Vec::new(),
        }
    }

    /// 現レベルの上位仮説。ロスト中 (粗レベル) も返す — pose() と違い多峰の
    /// まま渡せるのが QMDP の意義なので、レベルでは絞らない。
    fn top_cells(&self, k: usize) -> Vec<(PoseView, f64)> {
        if !self.initialized || k == 0 {
            return Vec::new();
        }
        let l = &self.levels[self.cur];
        let (nx, ny, res, t_res) = (l.nx, l.ny, l.res, l.t_res_deg);
        let plane = (nx * ny) as usize;
        let n = self.n_active();
        top_k_weights(&self.b[..n], k)
            .into_iter()
            .map(|(w, i)| {
                let (it, rem) = ((i / plane) as i32, i % plane);
                let (iy, ix) = ((rem / nx as usize) as i32, (rem % nx as usize) as i32);
                (
                    PoseView {
                        x: self.field.ox + (self.wx0 + ix) as f64 * res + res * 0.5,
                        y: self.field.oy + (self.wy0 + iy) as f64 * res + res * 0.5,
                        yaw_rad: ((it as f64 + 0.5) * t_res).to_radians(),
                    },
                    w as f64,
                )
            })
            .collect()
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

    fn wrap_rad(d: f64) -> f64 {
        use std::f64::consts::PI;
        (d + PI).rem_euclid(2.0 * PI) - PI
    }

    #[test]
    fn external_localizer_passes_pose_through() {
        let mut l = ExternalLocalizer::default();
        assert!(l.pose().is_none());
        // predict/observe は no-op のまま落ちないこと。
        l.predict(0.3, 20.0, 0.1);
        l.observe(&LaserScan::default());
        l.set_pose(pose(1.0, 2.0, 0.5));
        let p = l.pose().expect("set_pose 後は返す");
        assert_eq!((p.x, p.y, p.yaw_rad), (1.0, 2.0, 0.5));
        assert_eq!(l.quality(), 1.0);
    }

    /// grid: ずらしたシードから合成スキャンで真値へ収束すること (correct の本体)。
    #[test]
    fn grid_localizer_tightens_onto_the_true_pose() {
        let g = walled_grid(60); // 3m×3m @0.05
        let truth = pose(1.2, 1.5, 0.5);
        let bc = BeliefConfig {
            half_m: 1.0,
            beam_step: 1,
            init_sigma_xy_m: 0.3,
            init_sigma_theta_deg: 20.0,
            ..BeliefConfig::default()
        };
        let mut loc = GridLocalizer::new(&g, 36, bc);
        assert!(loc.pose().is_none(), "シード前は None");

        // 真値から 0.14m / 11° ずらして手動シード。
        loc.set_pose(pose(1.3, 1.4, 0.3));
        let scan = cast_scan(&g, truth, 36, 5.0);
        for _ in 0..6 {
            loc.observe(&scan);
            loc.predict(0.0, 0.0, 0.1); // 静止でも動作ノイズのぼかしは回る
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
        assert!(loc.quality() > 0.5, "観測一致度が低すぎる: {}", loc.quality());
    }

    /// grid: predict が指令どおり平均を進め、回すこと (動作モデル = 自分の cmd_vel)。
    #[test]
    fn grid_localizer_predict_advances_the_mean_along_the_heading() {
        let g = walled_grid(60);
        let mut loc =
            GridLocalizer::new(&g, 36, BeliefConfig { half_m: 1.0, ..BeliefConfig::default() });
        loc.set_pose(pose(1.5, 1.5, 0.0));

        // 前進 0.3 m/s × 1.0 s。
        for _ in 0..10 {
            loc.predict(0.3, 0.0, 0.1);
        }
        let m = loc.pose().expect("平均");
        assert!((m.x - 1.8).abs() < 0.08, "x = {:.3} (期待 1.8 付近)", m.x);
        assert!((m.y - 1.5).abs() < 0.05, "y = {:.3} (期待 1.5 のまま)", m.y);

        // その場旋回 90 deg/s × 1.0 s。
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

    /// adaptive: 整合した観測なら L0 に留まり、grid と同じ追跡をすること。
    #[test]
    fn adaptive_localizer_tracks_at_level0_like_grid() {
        let g = walled_grid(60);
        let truth = pose(1.2, 1.5, 0.5);
        let bc = BeliefConfig {
            half_m: 1.0,
            beam_step: 1,
            init_sigma_xy_m: 0.3,
            init_sigma_theta_deg: 20.0,
            ..BeliefConfig::default()
        };
        let mut loc = AdaptiveLocalizer::new(&g, 36, bc);
        assert!(loc.pose().is_none(), "シード前は None");
        loc.set_pose(pose(1.3, 1.4, 0.3));
        let scan = cast_scan(&g, truth, 36, 5.0);
        for _ in 0..6 {
            loc.observe(&scan);
            loc.predict(0.0, 0.0, 0.1);
        }
        assert_eq!(loc.level(), 0, "整合した観測で拡張しないこと");
        let m = loc.pose().expect("収束後の平均");
        assert!(
            (m.x - truth.x).abs() < 0.1 && (m.y - truth.y).abs() < 0.1,
            "mean ({:.3}, {:.3}) が真値 ({:.3}, {:.3}) から遠い",
            m.x, m.y, truth.x, truth.y
        );
        assert!(wrap_rad(m.yaw_rad - truth.yaw_rad).abs() < 0.2);
    }

    /// adaptive: 誘拐 (L1 窓の外へ瞬間移動) から expansion resetting で復帰する
    /// こと。ロスト検出 (pose None) → 全域レベルまで拡張 → 再定位、の全カスケード。
    #[test]
    fn adaptive_localizer_recovers_from_kidnap() {
        let g = tenm_grid();
        let bc = BeliefConfig { half_m: 1.0, beam_step: 4, ..BeliefConfig::default() };
        let mut loc = AdaptiveLocalizer::new(&g, 36, bc);
        assert!(loc.num_levels() >= 3);

        let a = pose(2.5, 2.0, 0.4);
        loc.set_pose(a);
        let scan_a = cast_scan(&g, a, 180, 12.0);
        for _ in 0..5 {
            loc.observe(&scan_a);
        }
        assert_eq!(loc.level(), 0);
        assert!(loc.pose().is_some());

        // 誘拐: 約 8m 離れた場所へ (L1 窓 8m の外 → L2 全域まで上がるはず)。
        let b = pose(8.0, 8.0, 2.0);
        let scan_b = cast_scan(&g, b, 180, 12.0);
        let mut max_level = 0;
        let mut lost = false;
        let mut recovered = None;
        for i in 0..150 {
            loc.observe(&scan_b);
            max_level = max_level.max(loc.level());
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
        assert!(max_level >= 2, "全域レベルまで拡張すること (max_level={max_level})");
        assert!(
            recovered.is_some(),
            "150 スキャン以内に再定位すること (max_level={max_level}, q={:.3})",
            loc.quality()
        );
    }

    /// adaptive: 未シードでも最初のスキャンから大域初期化で立ち上がること。
    #[test]
    fn adaptive_localizer_global_init_without_seed() {
        let g = tenm_grid();
        let bc = BeliefConfig { half_m: 1.0, beam_step: 4, ..BeliefConfig::default() };
        let mut loc = AdaptiveLocalizer::new(&g, 36, bc);
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
        assert!(ok.is_some(), "未シードでも大域初期化で再定位すること");
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
        // walled_grid の内部ブロック (x40..44, y10..14) の中心。
        let block_center = pose(42.0 * 0.05, 12.0 * 0.05, 0.0);

        let mut gl = GridLocalizer::new(&g, 36, BeliefConfig::default());
        gl.set_pose(block_center);
        let p = gl.pose().expect("マスク後も free 側に質量が残ること");
        assert!(free_at(p), "grid: 推定 ({:.2}, {:.2}) が free でない", p.x, p.y);

        let mut al = AdaptiveLocalizer::new(&g, 36, BeliefConfig::default());
        al.set_pose(block_center);
        let p = al.pose().expect("マスク後も free 側に質量が残ること");
        assert!(free_at(p), "adaptive: 推定 ({:.2}, {:.2}) が free でない", p.x, p.y);
    }

    /// viterbi (min-plus 全域レベル): 誘拐から復帰すること — adaptive の
    /// sum-product L2 を再帰 MAP に替えても同じ復帰性が保たれる回帰。
    #[test]
    fn viterbi_localizer_recovers_from_kidnap() {
        let g = tenm_grid();
        let bc =
            BeliefConfig { half_m: 1.0, beam_step: 4, viterbi: true, ..BeliefConfig::default() };
        let mut loc = AdaptiveLocalizer::new(&g, 36, bc);
        assert_eq!(loc.name(), "viterbi");

        let a = pose(2.5, 2.0, 0.4);
        loc.set_pose(a);
        let scan_a = cast_scan(&g, a, 180, 12.0);
        for _ in 0..5 {
            loc.observe(&scan_a);
        }
        assert_eq!(loc.level(), 0);

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
            "150 スキャン以内に再定位すること (level={}, q={:.3})",
            loc.level(),
            loc.quality()
        );
    }

    /// viterbi: 未シードの大域初期化 (enter_uniform → δ 一様) でも立ち上がること。
    #[test]
    fn viterbi_localizer_global_init_without_seed() {
        let g = tenm_grid();
        let bc =
            BeliefConfig { half_m: 1.0, beam_step: 4, viterbi: true, ..BeliefConfig::default() };
        let mut loc = AdaptiveLocalizer::new(&g, 36, bc);
        assert!(loc.pose().is_none());
        let truth = pose(8.0, 8.0, 2.0);
        let scan = cast_scan(&g, truth, 180, 12.0);
        let mut ok = None;
        for i in 0..150 {
            loc.observe(&scan);
            // ロスト中の predict は移動量の記録だけ (O(1)) — 静止でも落ちないこと。
            loc.predict(0.0, 0.0, 0.1);
            if let Some(p) = loc.pose() {
                if (p.x - truth.x).abs() < 0.4 && (p.y - truth.y).abs() < 0.4 {
                    ok = Some(i);
                    break;
                }
            }
        }
        assert!(ok.is_some(), "大域初期化から再定位できない (level={})", loc.level());
    }

    /// reloc_targets: 対称な 2 仮説から「地図が仮説間で違って見える方向」への
    /// 変位を選ぶこと (能動的再定位の行き先)。北側の一方にだけ障害物クラスタが
    /// ある地図で、両仮説とも北向きの行き先が返るはず。
    #[test]
    fn reloc_targets_point_toward_disambiguating_terrain() {
        // 20m×10m @0.1、開けた空間 + 仮説 A の北にだけ障害物クラスタ。
        let (w, h) = (200, 100);
        let mut g = OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.1,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (w * h) as usize],
        };
        for y in 90..96 {
            for x in 40..70 {
                g.data[(y * w + x) as usize] = 100;
            }
        }
        let mut loc = AdaptiveLocalizer::new(&g, 36, BeliefConfig::default());
        assert!(loc.reloc_targets().is_empty(), "未初期化は空");

        // ロスト状態を直接組む: 全域レベルで (5.2, 5.2) と (15.2, 5.2) の 2 仮説。
        let last = loc.levels.len() - 1;
        loc.cur = last;
        loc.initialized = true;
        loc.wx0 = 0;
        loc.wy0 = 0;
        let (nx, ny) = (loc.levels[last].nx, loc.levels[last].ny);
        let n = loc.n_active();
        loc.b[..n].fill(0.0);
        let res = loc.levels[last].res;
        let cell = |wx: f64, wy: f64| ((wx / res) as i32, (wy / res) as i32);
        let (ax, ay) = cell(5.2, 5.2);
        let (bx, by) = cell(15.2, 5.2);
        loc.b[bidx2(nx, ny, ax, ay, 0)] = 0.6;
        loc.b[bidx2(nx, ny, bx, by, 0)] = 0.4;

        let t = loc.reloc_targets();
        assert_eq!(t.len(), 2, "仮説ごとに 1 点");
        for &(x, y) in &t {
            assert!(y > 6.0, "行き先 ({x:.1}, {y:.1}) が判別地形 (北) を向いていない");
        }
        // 同じロボット系変位 δ (両仮説とも yaw ビン 0) — 世界系でも同じずれ。
        assert!(
            ((t[1].0 - t[0].0) - 10.0).abs() < 0.5,
            "2 つの行き先は同じ δ で結ばれるはず: {t:?}"
        );
    }

    /// top_cells: シード直後の最大重み仮説がシード姿勢のセルで、重みが降順なこと
    /// (QMDP の仮説集合の契約)。grid / adaptive 両方。
    #[test]
    fn top_cells_returns_descending_hypotheses_near_the_seed() {
        let g = walled_grid(60);
        let seed = pose(1.5, 1.5, 0.0);
        let check = |cells: Vec<(PoseView, f64)>, name: &str| {
            assert!(!cells.is_empty(), "{name}: 仮説が空");
            assert!(cells.len() <= 8, "{name}: k を超過");
            for w in cells.windows(2) {
                assert!(w[0].1 >= w[1].1, "{name}: 重みが降順でない");
            }
            let top = cells[0].0;
            assert!(
                (top.x - seed.x).abs() < 0.1 && (top.y - seed.y).abs() < 0.1,
                "{name}: 最大重み仮説 ({:.2}, {:.2}) がシードから遠い",
                top.x, top.y
            );
        };
        let mut gl =
            GridLocalizer::new(&g, 36, BeliefConfig { half_m: 1.0, ..BeliefConfig::default() });
        assert!(gl.top_cells(8).is_empty(), "シード前は空");
        gl.set_pose(seed);
        check(gl.top_cells(8), "grid");

        let mut al = AdaptiveLocalizer::new(&g, 36, BeliefConfig::default());
        al.set_pose(seed);
        check(al.top_cells(8), "adaptive");
    }
}
