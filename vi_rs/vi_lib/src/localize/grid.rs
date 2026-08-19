//! 内蔵ヒストグラム MCL の窓 1 段版 ([`GridLocalizer`])。多重解像度版は
//! [`super::adaptive`]、共有の尤度場・添字ヘルパは親モジュール。

use super::*;

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
            blur_a(self.cfg.motion_sigma_xy_m / res),
            blur_a(self.cfg.motion_sigma_theta_deg / self.t_res_deg),
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

    fn belief_grid(&self) -> Option<OccupancyGrid> {
        if !self.initialized {
            return None;
        }
        let nw = self.nw;
        let res = self.field.res;
        let m = marginal(&self.b, (nw * nw) as usize);
        Some(mass_to_grid(
            &m,
            nw,
            nw,
            res,
            self.field.ox + self.wx0 as f64 * res,
            self.field.oy + self.wy0 as f64 * res,
            self.field.oq.clone(),
        ))
    }
}
