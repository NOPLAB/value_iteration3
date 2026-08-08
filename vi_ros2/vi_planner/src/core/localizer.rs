//! 自己位置の出どころ ([`Localizer`]) — 外部推定の素通し (現行動作) と、内蔵
//! ヒストグラム MCL を trait で抽象化し、ROS パラメータ `localizer` で切り替える
//! ([`FollowController`](super::follow) と同じ切り替え方式)。
//!
//! - [`ExternalLocalizer`] — `pose_topic` (mcl 等) の推定をそのまま返す。既定。
//! - [`GridLocalizer`] — ロボット近傍の窓 (既定 5m×5m × θ) に belief ヒストグラムを
//!   持ち、predict (自分が出した速度指令によるシフト + ノイズぼかし) と
//!   correct (スキャン端点 × 静的地図の 2D 尤度場) で自己位置を推定する。
//!   このとき `pose_topic` は **手動シード** (initialpose 等) として扱い、
//!   メッセージが来るたびに belief を初期化し直す — 連続推定器 (mcl) を
//!   指したままにしないこと (毎メッセージでリセットされて素通しと変わらなくなる)。
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
//! - ここは rclrs 非依存 (core の分離クレート方式でホストテスト可能)。ROS 配線
//!   (購読・predict の呼び出し) は main.rs 側。

use vi_lib::bridge::PoseView;
use vi_lib::msg::{LaserScan, OccupancyGrid};

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
}

impl LikelihoodField {
    fn from_grid(g: &OccupancyGrid, sigma_m: f64) -> Self {
        let (w, h) = (g.width, g.height);
        let n = (w as usize) * (h as usize);
        // ValueIterator と同じ規約: data == 0 が free、非 0 は障害物。
        let mut d = vec![f32::INFINITY; n];
        for i in 0..n {
            if g.data[i] != 0 {
                d[i] = 0.0;
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
        Self { w, h, res: g.resolution, ox: g.origin_x, oy: g.origin_y, lf }
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
        self.initialized.then(|| self.mean()).flatten()
    }

    fn quality(&self) -> f64 {
        self.quality
    }
}
