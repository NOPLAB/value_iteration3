//! 多重解像度ヒストグラム MCL ([`AdaptiveLocalizer`]) — 窓レベルの追跡と
//! expansion resetting。min-plus (Viterbi) モードは子モジュール `viterbi`。

use super::*;

mod viterbi;
use viterbi::VitState;

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
/// 能動的再定位 ([`Localizer::reloc_targets`]) の判別に使う上位モード数
/// (変位探索側の定数は [`crate::belief::reloc_targets`])。
const RELOC_MODES: usize = 4;

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
    /// ときだけ Some)。実装は localize/adaptive/viterbi.rs。
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

    /// テスト専用: 全域レベルのロスト状態 (世界座標の 2 仮説、yaw ビン 0) を直接組む。
    /// localize/tests.rs の reloc_targets_point_toward_disambiguating_terrain 用 —
    /// これが無いと窓・レベルの私有フィールドを 6 個公開することになる。
    #[cfg(test)]
    pub(super) fn force_bimodal_global(&mut self, a: (f64, f64, f32), b: (f64, f64, f32)) {
        let last = self.levels.len() - 1;
        self.cur = last;
        self.initialized = true;
        self.wx0 = 0;
        self.wy0 = 0;
        let (nx, ny, res) = (self.levels[last].nx, self.levels[last].ny, self.levels[last].res);
        let n = self.n_active();
        self.b[..n].fill(0.0);
        for (wx, wy, w) in [a, b] {
            self.b[bidx2(nx, ny, (wx / res) as i32, (wy / res) as i32, 0)] = w;
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
            blur_a(self.cfg.motion_sigma_xy_m / res),
            blur_a(self.cfg.motion_sigma_theta_deg / t_res),
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
            // min-plus (Viterbi) モード: δ の更新 + b の実体化 (localize/adaptive/viterbi.rs)。
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

    /// ロスト中 (粗レベル滞在中) の判別変位。探索そのものは全地図 belief と共有
    /// ([`crate::belief::reloc_targets`] の doc に式)。
    fn reloc_targets(&self) -> Vec<(f64, f64)> {
        if !self.initialized || self.cur == 0 {
            return Vec::new();
        }
        crate::belief::reloc_targets(
            &self.top_modes(RELOC_MODES, 2),
            |x, y| self.field.free_at(x, y),
            |x, y| self.field.at(x, y),
        )
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

    /// 現レベルの窓の belief。レベルが上がると寸法も解像度も変わるが、
    /// メッセージが幾何を全部持つので RViz 側は追従する。
    fn belief_grid(&self) -> Option<OccupancyGrid> {
        if !self.initialized {
            return None;
        }
        let l = &self.levels[self.cur];
        let (nx, ny, res) = (l.nx, l.ny, l.res);
        let n = self.n_active();
        let m = marginal(&self.b[..n], (nx * ny) as usize);
        Some(mass_to_grid(
            &m,
            nx,
            ny,
            res,
            self.field.ox + self.wx0 as f64 * res,
            self.field.oy + self.wy0 as f64 * res,
            self.field.oq.clone(),
        ))
    }
}
