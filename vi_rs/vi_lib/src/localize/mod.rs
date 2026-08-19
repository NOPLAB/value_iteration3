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
//!   (localize/adaptive/viterbi.rs の doc)。窓レベルの追跡・レベル遷移機構は共通。
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

use crate::belief::mass_to_grid;
use crate::bridge::PoseView;
use crate::msg::{LaserScan, OccupancyGrid, Quaternion};

mod adaptive;
mod grid;

pub use adaptive::AdaptiveLocalizer;
pub use grid::GridLocalizer;

#[cfg(test)]
mod tests;

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
    /// 多峰 belief を持つ推定器 ([`AdaptiveLocalizer`] と全地図の
    /// [`crate::belief::Belief`]) が実装する — 探索は
    /// [`crate::belief::reloc_targets`] の共有版。
    fn reloc_targets(&self) -> Vec<(f64, f64)> {
        Vec::new()
    }
    /// 現在の belief の θ 周辺分布を可視化用 OccupancyGrid に描いたもの
    /// ([`crate::belief::mass_to_grid`] のスケール)。窓つきの実装では**窓の
    /// 範囲だけ**の格子で、地図全域とは寸法も原点も違う (RViz は Map 表示が
    /// メッセージごとに原点を見るのでそのまま重なる)。belief を持たない
    /// [`ExternalLocalizer`] と未シードは None。
    fn belief_grid(&self) -> Option<OccupancyGrid> {
        None
    }
}

/// belief バッファ (θ 面優先) を θ で周辺化する。窓つき 2 実装の共有部。
fn marginal(b: &[f32], plane: usize) -> Vec<f32> {
    let mut m = vec![0f32; plane];
    for (i, &w) in b.iter().enumerate() {
        m[i % plane] += w;
    }
    m
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
    /// (localize/adaptive/viterbi.rs の doc)。窓レベルの追跡は sum-product のままで、
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

/// 汎用添字 (θ 面優先、長方形窓)。[`bidx`] の nx≠ny 版。
#[inline]
fn bidx2(nx: i32, ny: i32, ix: i32, iy: i32, it: i32) -> usize {
    ((it * ny + iy) * nx + ix) as usize
}

/// 3 点カーネル [a, 1-2a, a] の a。σ [セル] 1 tick のランダムウォーク分散
/// (2a セル²) を合わせる。0.25 で頭打ち (それ以上は 1 tick で表せない)。
fn blur_a(sigma_cells: f64) -> f32 {
    ((sigma_cells * sigma_cells) / 2.0).clamp(0.0, 0.25) as f32
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

/// 静的地図から起こす 2D 尤度場: セルごとに exp(-d²/2σ²) (d = 最近障害物までの
/// 距離) を u8 量子化して持つ。チャンファー 2 パスなので構築は O(セル数)。
struct LikelihoodField {
    w: i32,
    h: i32,
    res: f64,
    ox: f64,
    oy: f64,
    /// 地図原点の回転 (可視化グリッドが echo するだけ)。
    oq: Quaternion,
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
        Self {
            w,
            h,
            res: g.resolution,
            ox: g.origin_x,
            oy: g.origin_y,
            oq: g.origin_quat.clone(),
            lf,
            free,
        }
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
