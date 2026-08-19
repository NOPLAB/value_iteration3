//! 本家 `ValueIteratorLocal` 忠実移植。`ValueIterator` を内包 (合成) し override を再定義。
//! local の `actionCostLocal` は本家 `actionCost` と完全同一なので base 経由で計算する。

use crate::action::Action;
use crate::msg::{LaserScan, OccupancyGrid};
use crate::params::{PROB_BASE, PROB_BASE_BIT};
use crate::value_iterator::ValueIterator;

/// 各ビームを始点から「ヒット点の `stop_back` [m] 手前」まで `step` [m] 刻みで
/// 歩き、通過点の世界座標を `f` に渡す。`f` が false を返したらそのビームは
/// 打ち切る (格子外に出た等 — ビームは直進するので戻ってこない。これが
/// `range` が inf のときの無限ループも止める)。
///
/// VI 側 (`clear_map_from_scan`) と belief 側 (`Belief::clear_free_from_scan`)
/// の共有ルーチン。両者は解像度も原点も違う格子を持つので、セル化は呼び側。
pub fn walk_beams(
    msg: &LaserScan,
    x: f64,
    y: f64,
    t: f64,
    step: f64,
    stop_back: f64,
    mut f: impl FnMut(f64, f64) -> bool,
) {
    for i in 0..msg.ranges.len() {
        let a = t + msg.angle_increment * i as f64 + msg.angle_min;
        let (ca, sa) = (a.cos(), a.sin());
        let stop = msg.ranges[i] - stop_back; // 実物の障害物は開けない
        let mut s = step;
        while s < stop {
            if !f(x + s * ca, y + s * sa) {
                break;
            }
            s += step;
        }
    }
}

/// ローカルウィンドウの形状 (本家に無い、既定 [`LocalShape::Square`] = 本家挙動)。
///
/// `Square` だけ**地図軸に固定**で、残りはロボット座標系に載る (前後 = 進行方向)。
/// どれも**凸**であること — `clear_map_from_scan` の `walk_beams` は「窓を出た
/// ビームは戻らない」として打ち切るので、凹形状 (扇形・前方コーン) を足すと
/// 再入するビームを黙って切る。
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum LocalShape {
    /// 地図軸の矩形 (前後 `range_m` × 左右 `lat_m`)。本家 `inLocalArea` そのもの。
    #[default]
    Square,
    /// ロボット座標系の楕円。半軸は前後 `range_m`・横 `lat_m` で固定。
    Circle,
    /// `Circle` の前後半軸を走行方向で伸縮させたもの (前後で別々の半軸 = 卵形)。
    Ellipse,
    /// 半径 `lat_m` の円を進行方向の線分に沿って掃いた形 (スタジアム形)。
    /// 楕円と違い前縁でも幅が `lat_m` のままなので、伸ばした先の横被覆が落ちない。
    Capsule,
}

impl LocalShape {
    /// パラメータ名からの変換 (未知の名前は None)。
    pub fn from_name(s: &str) -> Option<Self> {
        match s {
            "square" => Some(Self::Square),
            "circle" => Some(Self::Circle),
            "ellipse" => Some(Self::Ellipse),
            "capsule" => Some(Self::Capsule),
            _ => None,
        }
    }

    /// ロボット方位に追従するか (`Square` だけ地図軸固定)。
    fn oriented(self) -> bool {
        !matches!(self, Self::Square)
    }

    /// 走行方向で前後の半軸が伸縮するか。
    fn dynamic(self) -> bool {
        matches!(self, Self::Ellipse | Self::Capsule)
    }
}

/// [`LocalShape`] とその寸法・変形量の制限。既定は本家 (1 m 四方の地図軸矩形)。
#[derive(Clone, Copy, Debug)]
pub struct LocalShapeConfig {
    pub shape: LocalShape,
    /// 前後の基準半径 r [m]。
    pub range_m: f64,
    /// 横の半径 b [m]。`<= 0` なら `range_m` と同じ (等方)。
    pub lat_m: f64,
    /// 前後半軸の倍率上限。`1.0` で変形なし。下限は `r / ratio_max`。
    /// **和 `a_fwd + a_bwd = 2r` を保つ** (面積 ∝ `b·(a_fwd+a_bwd)` を一定にして
    /// 掃きコストが速度で暴れないようにする) ので、伸ばせる上限は後方の下限に
    /// 引かれて `2r - r/ratio_max` に落ちる。
    pub ratio_max: f64,
    /// 半軸の 1 呼び出しあたりの変化上限 [m]。`<= 0` で無制限。
    pub slew_m: f64,
    /// 1 呼び出しあたりの前進変位の基準 [m]。この量だけ進んだ tick を「全速」と
    /// して正規化する。`<= 0` で変形しない (= 静的形状)。
    pub fw_ref_m: f64,
}

impl Default for LocalShapeConfig {
    fn default() -> Self {
        Self {
            shape: LocalShape::Square,
            range_m: 1.0,
            lat_m: 0.0,
            ratio_max: 1.0,
            slew_m: 0.0,
            fw_ref_m: 0.0,
        }
    }
}

impl LocalShapeConfig {
    /// 横半径 [m] (0 指定は前後と同じ)。
    fn lat(&self) -> f64 {
        if self.lat_m > 0.0 {
            self.lat_m
        } else {
            self.range_m
        }
    }

    /// 外接矩形の半径 [セル] `(x, y)`。`Square` は地図軸なので前後/左右がそのまま
    /// x/y、向きを持つ形状は回転するので両軸とも「変形の上限を含めた」最大半軸。
    pub fn radii_cells(&self, res: f64) -> (i32, i32) {
        if self.shape.oriented() {
            let m = ((self.range_m * self.ratio_max.max(1.0)).max(self.lat()) / res) as i32;
            (m, m)
        } else {
            ((self.range_m / res) as i32, (self.lat() / res) as i32)
        }
    }

    /// **最小**半軸 [m]。σ マージン膨張の上限に使う — ここを超えて膨らませると
    /// 一番細い方向でウィンドウが帯で埋まり、ロボットが出口を失う。変形する形状は
    /// 前後が `range_m / ratio_max` まで縮み得るのでそれも見る。
    pub fn min_radius_m(&self) -> f64 {
        let fore = if self.shape.dynamic() {
            self.range_m / self.ratio_max.max(1.0)
        } else {
            self.range_m
        };
        fore.min(self.lat())
    }

    /// 外接矩形の半径 [セル] (両軸の最大)。`set_local_shape` 後の
    /// `local_ixy_range` と**必ず一致する** — compact の追従パッチはこの値で
    /// 寸法と再配置を決めるので、パッチ側は VI を作る前にここから取ること。
    pub fn bbox_cells(&self, res: f64) -> i32 {
        let (rx, ry) = self.radii_cells(res);
        rx.max(ry)
    }
}

/// 前進変位がこれ未満 [セル] の tick は符号がノイズなので半軸を据え置く。
const DEADBAND_CELLS: f64 = 0.3;

pub struct ValueIteratorLocal {
    pub base: ValueIterator,
    pub local_ix_min: i32,
    pub local_ix_max: i32,
    pub local_iy_min: i32,
    pub local_iy_max: i32,
    /// 外接矩形の半径 [セル] = 全方位・全時刻での半軸の最大値。compact の追従
    /// パッチはこれで寸法と再配置を決めるので、走行中に増えてはいけない。
    pub local_ixy_range: i32,
    pub local_xy_range: f64,
    /// ウィンドウ形状 (本家に無い、既定 [`LocalShape::Square`] = 本家挙動)。
    shape: LocalShapeConfig,
    /// 現在の半軸 [セル]: 前方・後方・横。`Square` では前後 = x、横 = y の
    /// 地図軸半径そのもの。
    a_fwd: f64,
    a_bwd: f64,
    lat: f64,
    /// `set_local_window` に渡された最新の姿勢 (中心セル座標と方位)。マスク判定用。
    center: (f64, f64),
    cos_t: f64,
    sin_t: f64,
    /// 前回の `set_local_window` の位置 [m]。前進変位の測定に使う。
    prev_xy: Option<(f64, f64)>,
    /// レーザーが貫通したセルの**地図由来**コスト (`free` / `penalty`) も反証するか
    /// (本家に無い、既定 false) — ビームが通り抜けた以上そこは空いている。地図が
    /// 壁と言っているなら地図が古いか自己位置がずれているかで、どちらにせよ通れる。
    /// スキャン由来の証拠がある所しか開けないので「地図を無視して突っ切る」には
    /// ならない。寿命は `local_penalty` と同じ (ゴールを解き直すと戻る)。
    pub clear_map_from_scan: bool,
    /// `set_map_with_occupancy_grid` に渡された静的マージン [m] とその penalty。
    /// [`Self::inflate_by_sigma`] が「静的帯の外側」と「同じ強さ」を知るために保つ。
    safety_radius: f64,
    safety_radius_penalty: f64,
    /// 障害物までの L∞ 距離 [セル] (`iy*nx+ix`)。[`Self::inflate_by_sigma`] の初回
    /// 呼び出しで `states` から起こす — 使わない設定では確保しない。
    obstacle_dist: Vec<u16>,
}

impl ValueIteratorLocal {
    /// 本家 `ValueIteratorLocal(actions, thread_num)`。
    pub fn new(actions: Vec<Action>, thread_num: i32) -> Self {
        Self {
            base: ValueIterator::new(actions, thread_num),
            local_ix_min: 0,
            local_ix_max: 0,
            local_iy_min: 0,
            local_iy_max: 0,
            local_ixy_range: 0,
            local_xy_range: 0.0,
            shape: LocalShapeConfig::default(),
            a_fwd: 0.0,
            a_bwd: 0.0,
            lat: 0.0,
            center: (0.0, 0.0),
            cos_t: 1.0,
            sin_t: 0.0,
            prev_xy: None,
            clear_map_from_scan: false,
            safety_radius: 0.0,
            safety_radius_penalty: 0.0,
            obstacle_dist: Vec::new(),
        }
    }

    /// 本家 `ValueIteratorLocal::setMapWithOccupancyGrid`。base を呼んでから local window 初期化。
    pub fn set_map_with_occupancy_grid(
        &mut self,
        map: &OccupancyGrid,
        theta_cell_num: i32,
        safety_radius: f64,
        safety_radius_penalty: f64,
        goal_margin_radius: f64,
        goal_margin_theta: i32,
    ) {
        self.base.set_map_with_occupancy_grid(
            map,
            theta_cell_num,
            safety_radius,
            safety_radius_penalty,
            goal_margin_radius,
            goal_margin_theta,
        );
        // 形状も寸法も既定 (本家の 1 m 等方矩形) へ戻す。
        self.set_local_shape(LocalShapeConfig::default());
        self.safety_radius = safety_radius;
        self.safety_radius_penalty = safety_radius_penalty;
        self.obstacle_dist = Vec::new(); // 地図が変わったので距離場は捨てる
    }

    /// ローカルウィンドウ半径 [m] を変更する (本家は 1.0 固定)。
    /// `set_map_with_occupancy_grid` が半径を 1.0 に戻すので、その後に呼ぶこと。
    pub fn set_local_xy_range(&mut self, range_m: f64) {
        self.set_local_shape(LocalShapeConfig { range_m, ..Default::default() });
    }

    /// ウィンドウの形状と寸法を設定する (本家に無い)。`set_local_xy_range` は
    /// これの「本家と同じ等方矩形」版。`set_map_with_occupancy_grid` が形状を
    /// 既定へ戻すので、その後に呼ぶこと。
    pub fn set_local_shape(&mut self, cfg: LocalShapeConfig) {
        let res = self.base.xy_resolution;
        self.shape = cfg;
        self.local_xy_range = cfg.range_m;
        self.a_fwd = cfg.range_m / res;
        self.a_bwd = self.a_fwd;
        self.lat = cfg.lat() / res;
        self.prev_xy = None;
        let (rx, ry) = self.rect_radii();
        // compact の追従パッチはこれで寸法を決める。走行中に増えてはいけないので
        // **変形の上限**を含めた最大値にする (`rect_radii` が形状ごとに出す)。
        self.local_ixy_range = rx.max(ry);
        self.local_ix_min = 0;
        self.local_iy_min = 0;
        self.local_ix_max = rx * 2;
        self.local_iy_max = ry * 2;
    }

    /// 外接矩形の半径 [セル] `(x, y)` ([`LocalShapeConfig::radii_cells`])。
    fn rect_radii(&self) -> (i32, i32) {
        self.shape.radii_cells(self.base.xy_resolution)
    }

    /// 本家 `inLocalArea` + 形状マスク。**ウィンドウの定義はここ 1 箇所**で、
    /// スキャン注入・狭域の掃き・σ マージン膨張は全部これを通る。
    pub fn in_local_area(&self, ix: i32, iy: i32) -> bool {
        if ix < self.local_ix_min
            || ix > self.local_ix_max
            || iy < self.local_iy_min
            || iy > self.local_iy_max
        {
            return false;
        }
        if !self.shape.shape.oriented() {
            return true; // Square = 本家の矩形そのもの
        }
        // ロボット座標系でのセル中心のオフセット [セル] (fx = 前方, fy = 左)。
        let dx = ix as f64 + 0.5 - self.center.0;
        let dy = iy as f64 + 0.5 - self.center.1;
        let fx = dx * self.cos_t + dy * self.sin_t;
        let fy = -dx * self.sin_t + dy * self.cos_t;
        let b = self.lat.max(f64::EPSILON);
        match self.shape.shape {
            LocalShape::Square => true,
            LocalShape::Circle | LocalShape::Ellipse => {
                let a = (if fx >= 0.0 { self.a_fwd } else { self.a_bwd }).max(f64::EPSILON);
                (fx / a).powi(2) + (fy / b).powi(2) <= 1.0
            }
            // 幅 b の円を [-(a_bwd-b), (a_fwd-b)] の線分に沿って掃いた形。
            // 前後の到達距離は楕円と同じ a_fwd / a_bwd になる。
            LocalShape::Capsule => {
                let c = fx.clamp(-(self.a_bwd - b).max(0.0), (self.a_fwd - b).max(0.0));
                (fx - c).powi(2) + fy * fy <= b * b
            }
        }
    }

    /// 走行方向に応じて前後の半軸を更新する ([`LocalShape::dynamic`] のみ)。
    ///
    /// 変化量の制限は 3 段: 前進変位を `fw_ref_m` で正規化して ±1 にクランプ →
    /// 半軸を `[r/ratio_max, 2r - r/ratio_max]` にクランプ → 1 呼び出しあたり
    /// `slew_m` までのスルーレート。自己位置推定の再収束で姿勢が m 単位で飛んでも
    /// 1 tick で形が変わり切らないようにするため、3 段とも要る。
    fn update_axes(&mut self, x: f64, y: f64, yaw: f64) {
        let cfg = self.shape;
        let prev = self.prev_xy.replace((x, y));
        if !cfg.shape.dynamic() || cfg.ratio_max <= 1.0 || cfg.fw_ref_m <= 0.0 {
            return;
        }
        let Some((px, py)) = prev else { return };
        let res = self.base.xy_resolution;
        let disp = (x - px) * yaw.cos() + (y - py) * yaw.sin();
        if disp.abs() < DEADBAND_CELLS * res {
            return; // 停止中は符号が暴れるので据え置く
        }
        let v = (disp / cfg.fw_ref_m).clamp(-1.0, 1.0);
        let r = cfg.range_m;
        let lo = r / cfg.ratio_max;
        let hi = (2.0 * r - lo).min(r * cfg.ratio_max);
        let target = (r * (1.0 + (cfg.ratio_max - 1.0) * v)).clamp(lo, hi);
        let cur = self.a_fwd * res;
        let slew = if cfg.slew_m > 0.0 { cfg.slew_m } else { f64::INFINITY };
        let next = target.clamp(cur - slew, cur + slew);
        self.a_fwd = next / res;
        self.a_bwd = (2.0 * r - next) / res;
    }

    /// 本家 `valueIterationLocal` = `valueIteration` (actionCostLocal は actionCost と同一)。
    pub fn value_iteration_local(&mut self, idx: usize) -> u64 {
        self.base.value_iteration_at(idx)
    }

    /// 本家 `localValueIterationLoop`。local window 内を走査。
    pub fn local_value_iteration_loop(&mut self) {
        let nt = self.base.cell_num_t;
        for iix in self.local_ix_min..=self.local_ix_max {
            for iiy in self.local_iy_min..=self.local_iy_max {
                if !self.in_local_area(iix, iiy) {
                    continue;
                }
                for iit in 0..nt {
                    let i = self.base.to_index(iix, iiy, iit) as usize;
                    self.value_iteration_local(i);
                }
            }
        }
    }

    /// 本家 `setLocalCost`。レーザヒット点周辺に local_penalty を設定/半減。
    pub fn set_local_cost(&mut self, msg: &LaserScan, x: f64, y: f64, t: f64) {
        self.set_local_cost_attenuated(msg, x, y, t, 0);
    }

    /// `set_local_cost` の注入値を `2048 >> shift` に減衰させる拡張 (本家に無い)。
    /// 自己位置の観測一致度が低い tick のスキャン投影に満額の壁を建てさせない
    /// 品質ゲート用。減衰後も 2 の冪 (vi_planner の PenaltyOverlay は指数しか
    /// 持たない)。shift は 11 でクランプ — 最低 `1 << PROB_BASE_BIT` を書き、
    /// 0 で既存の壁を消し飛ばすことはしない。
    pub fn set_local_cost_attenuated(
        &mut self,
        msg: &LaserScan,
        x: f64,
        y: f64,
        t: f64,
        shift: u32,
    ) {
        let inject = (2048u64 >> shift.min(11)) << PROB_BASE_BIT;
        let start_angle = msg.angle_min;
        let nt = self.base.cell_num_t;
        let (ox, oy, res) = (self.base.map_origin_x, self.base.map_origin_y, self.base.xy_resolution);

        for i in 0..msg.ranges.len() {
            let a = t + msg.angle_increment * i as f64 + start_angle;
            let r = msg.ranges[i];
            let lx = x + r * a.cos();
            let ly = y + r * a.sin();
            let ix = ((lx - ox) / res).floor() as i32;
            let iy = ((ly - oy) / res).floor() as i32;

            // d = 0.1..=0.9 (本家 f64 刻みを忠実再現)
            let mut d = 0.1;
            while d <= 0.9 {
                let half_lx = x + r * a.cos() * d;
                let half_ly = y + r * a.sin() * d;
                let half_ix = ((half_lx - ox) / res).floor() as i32;
                let half_iy = ((half_ly - oy) / res).floor() as i32;
                if self.in_local_area(half_ix, half_iy) {
                    for it in 0..nt {
                        let index = self.base.to_index(half_ix, half_iy, it) as usize;
                        self.base.states[index].local_penalty /= 2;
                    }
                }
                d += 0.1;
            }

            for iix in (ix - 2)..=(ix + 2) {
                for iiy in (iy - 2)..=(iy + 2) {
                    if !self.in_local_area(iix, iiy) {
                        continue;
                    }
                    for it in 0..nt {
                        let index = self.base.to_index(iix, iiy, it) as usize;
                        self.base.states[index].local_penalty = inject;
                    }
                }
            }
        }

        // 地図の壁/膨張帯の反証は上の d ループに相乗りできない: 0.1r 刻みの
        // 9 点では r が大きいほど間隔が空き (r=3m・res=0.05 で 6 セル飛ぶ)、
        // 開いた穴が繋がらないので通路にならない。ビーム線分をセル刻みで歩く。
        if self.clear_map_from_scan {
            walk_beams(msg, x, y, t, res * 0.5, res, |wx, wy| {
                let bx = ((wx - ox) / res).floor() as i32;
                let by = ((wy - oy) / res).floor() as i32;
                // ビームは直進するので、窓を出たら戻らない = 打ち切ってよい。
                if !self.in_local_area(bx, by) {
                    return false;
                }
                for it in 0..nt {
                    let index = self.base.to_index(bx, by, it) as usize;
                    self.base.states[index].free = true;
                    self.base.states[index].penalty = PROB_BASE;
                }
                true
            });
        }
    }

    /// 本家 `setLocalWindow`。ロボット位置中心に local window をクランプ。
    ///
    /// 本家に無い引数 `t` [rad] は形状マスクの向き — 向きを持つ形状
    /// ([`LocalShape`]) はここで方位を取り込み、走行方向に応じて半軸も更新する
    /// (`Square` では無視され、本家と完全に同じ矩形になる)。
    pub fn set_local_window(&mut self, x: f64, y: f64, t: f64) {
        self.update_axes(x, y, t);
        let res = self.base.xy_resolution;
        self.center = ((x - self.base.map_origin_x) / res, (y - self.base.map_origin_y) / res);
        self.cos_t = t.cos();
        self.sin_t = t.sin();
        let ix = self.center.0.floor() as i32;
        let iy = self.center.1.floor() as i32;
        let (rx, ry) = self.rect_radii();
        self.local_ix_min = if ix - rx >= 0 { ix - rx } else { 0 };
        self.local_iy_min = if iy - ry >= 0 { iy - ry } else { 0 };
        self.local_ix_max = if ix + rx < self.base.cell_num_x {
            ix + rx
        } else {
            self.base.cell_num_x - 1
        };
        self.local_iy_max = if iy + ry < self.base.cell_num_y {
            iy + ry
        } else {
            self.base.cell_num_y - 1
        };
    }

    /// 自己位置の広がり `extra_m` [m] のぶん、静的マージンの**外側**へ同じ強さの
    /// ペナルティ帯を足す (上田ら 2023 の 4·2·2 — マージン `m` に σ を足す操作)。
    ///
    /// 文献は状態空間に σ 軸を足して層ごとに `m` を変えるが、そこで効いているのは
    /// 「今の σ で壁からどれだけ離れるか」なので、3 次元のままウィンドウの
    /// `local_penalty` に同じ帯を書けば同じ形の場になる (状態数は増えない)。
    /// 膨らませる起点は**地図由来の障害物**であること — 文献の問題設定は
    /// センシングできない障害物なので、スキャンのヒット点からでは届かない。
    ///
    /// `set_local_window` の後に呼ぶ。既存の `local_penalty` は max で残す
    /// (スキャン注入の壁を消さない)。帯の寿命も `local_penalty` と同じ — 本家同様
    /// ウィンドウの外では消えず、ゴールを解き直すと戻る。
    // ponytail: 帯は「今の σ」への反応で、文献の 4D のように 2 手先の σ までは
    // 見ない。そこまで要るなら状態空間の拡張に戻すしかない。
    pub fn inflate_by_sigma(&mut self, extra_m: f64) {
        if extra_m <= 0.0 {
            return;
        }
        self.ensure_obstacle_dist();
        let res = self.base.xy_resolution;
        // 静的帯 (from_occupancy の正方形マージン) の縁と、σ ぶん外へ出した縁。
        let m0 = (self.safety_radius / res).ceil() as u16;
        let m1 = m0.saturating_add((extra_m / res).ceil() as u16);
        let pen = (self.safety_radius_penalty * PROB_BASE as f64) as u64;
        let (nx, nt) = (self.base.cell_num_x, self.base.cell_num_t);
        for ix in self.local_ix_min..=self.local_ix_max {
            for iy in self.local_iy_min..=self.local_iy_max {
                if !self.in_local_area(ix, iy) {
                    continue;
                }
                let d = self.obstacle_dist[(iy * nx + ix) as usize];
                // d <= m0 は障害物自身と静的帯の中 (既に penalty がある)、d > m1 は帯の外。
                if d <= m0 || d > m1 {
                    continue;
                }
                for it in 0..nt {
                    let i = self.base.to_index(ix, iy, it) as usize;
                    let s = &mut self.base.states[i];
                    s.local_penalty = s.local_penalty.max(pen);
                }
            }
        }
    }

    /// 障害物までの L∞ 距離を `states` の `free` から起こす (初回のみ)。
    fn ensure_obstacle_dist(&mut self) {
        if !self.obstacle_dist.is_empty() {
            return;
        }
        let (nx, ny, nt) = (self.base.cell_num_x, self.base.cell_num_y, self.base.cell_num_t);
        let states = &self.base.states;
        // to_index(ix, iy, 0) = ix*nt + iy*(nt*nx) (本家 toIndex の it=0)。
        let d = dist_linf(nx, ny, |ix, iy| !states[(ix * nt + iy * nt * nx) as usize].free);
        self.obstacle_dist = d;
    }
}

/// 障害物までの **L∞** 距離場 [セル] (`iy*nx+ix`)。チャンファー 2 パス、8 近傍。
///
/// L∞ なのは `State::from_occupancy` の静的マージンが正方形範囲だから — これで
/// `d <= margin` がそのまま「静的ペナルティ帯の中」と一致する。障害物セル自身は 0、
/// 障害物が 1 つも無ければ全セル `u16::MAX`。
fn dist_linf(nx: i32, ny: i32, is_obstacle: impl Fn(i32, i32) -> bool) -> Vec<u16> {
    let (nx, ny) = (nx.max(0) as usize, ny.max(0) as usize);
    if nx == 0 || ny == 0 {
        return Vec::new();
    }
    let mut d = vec![u16::MAX; nx * ny];
    for iy in 0..ny {
        for ix in 0..nx {
            if is_obstacle(ix as i32, iy as i32) {
                d[iy * nx + ix] = 0;
            }
        }
    }
    for iy in 0..ny {
        for ix in 0..nx {
            let i = iy * nx + ix;
            let mut v = d[i];
            if ix > 0 {
                v = v.min(d[i - 1].saturating_add(1));
            }
            if iy > 0 {
                v = v.min(d[i - nx].saturating_add(1));
                if ix > 0 {
                    v = v.min(d[i - nx - 1].saturating_add(1));
                }
                if ix + 1 < nx {
                    v = v.min(d[i - nx + 1].saturating_add(1));
                }
            }
            d[i] = v;
        }
    }
    for iy in (0..ny).rev() {
        for ix in (0..nx).rev() {
            let i = iy * nx + ix;
            let mut v = d[i];
            if ix + 1 < nx {
                v = v.min(d[i + 1].saturating_add(1));
            }
            if iy + 1 < ny {
                v = v.min(d[i + nx].saturating_add(1));
                if ix + 1 < nx {
                    v = v.min(d[i + nx + 1].saturating_add(1));
                }
                if ix > 0 {
                    v = v.min(d[i + nx - 1].saturating_add(1));
                }
            }
            d[i] = v;
        }
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;

    fn free_grid(w: i32, h: i32) -> OccupancyGrid {
        OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: vec![0; (w * h) as usize],
        }
    }

    #[test]
    fn set_map_initializes_local_window() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(60, 60); // res=0.05 → local_ixy_range = 1.0/0.05 = 20
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        assert_eq!(vi.local_ixy_range, 20);
        assert_eq!(vi.local_ix_max, 40);
        assert_eq!(vi.local_iy_max, 40);
    }

    #[test]
    fn set_local_xy_range_resizes_window() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(120, 120);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        vi.set_local_xy_range(2.0); // res=0.05 → range = 40 セル
        assert_eq!(vi.local_ixy_range, 40);
        vi.set_local_window(2.5, 2.5, 0.0); // ix=iy=50 → 10..=90
        assert_eq!(vi.local_ix_min, 10);
        assert_eq!(vi.local_ix_max, 90);
    }

    #[test]
    fn set_local_window_clamps() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(60, 60);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        // ロボットを原点に → ix=iy=0、range=20 → min は 0 にクランプ、max は 20。
        vi.set_local_window(0.0, 0.0, 0.0);
        assert_eq!(vi.local_ix_min, 0);
        assert_eq!(vi.local_iy_min, 0);
        assert_eq!(vi.local_ix_max, 20);
        assert_eq!(vi.local_iy_max, 20);
    }

    #[test]
    fn set_local_cost_sets_penalty_band() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(60, 60);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        // 1 ビーム、正面 (angle_min=0, increment=0), range=0.5m → ヒット点 (10,0) 付近。
        let scan = LaserScan {
            angle_min: 0.0,
            angle_increment: 0.0,
            ranges: vec![0.5],
        };
        vi.set_local_cost(&scan, 0.0, 0.0, 0.0);
        // ヒット点±2 セルのどこかに 2048<<bit が立っていること。
        let hit = vi.base.to_index(10, 0, 0) as usize;
        assert_eq!(vi.base.states[hit].local_penalty, 2048u64 << PROB_BASE_BIT);
    }

    #[test]
    fn set_local_cost_attenuated_scales_and_clamps() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(60, 60);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        let scan = LaserScan {
            angle_min: 0.0,
            angle_increment: 0.0,
            ranges: vec![0.5],
        };
        let hit = vi.base.to_index(10, 0, 0) as usize;
        vi.set_local_cost_attenuated(&scan, 0.0, 0.0, 0.0, 2);
        assert_eq!(vi.base.states[hit].local_penalty, 512u64 << PROB_BASE_BIT);
        // クランプ: shift が大きくても 0 にはならず 1<<bit で止まる。
        vi.set_local_cost_attenuated(&scan, 0.0, 0.0, 0.0, 99);
        assert_eq!(vi.base.states[hit].local_penalty, 1u64 << PROB_BASE_BIT);
    }

    /// ビームが貫通したセルは地図が壁でも通れるようになる (`clear_map_from_scan`)。
    /// ヒット点そのものは開けない — そこは障害物が現に在る。
    #[test]
    fn clear_map_from_scan_opens_only_what_the_beam_passed_through() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let mut map = free_grid(60, 60); // res=0.05
        // 地図の壁 2 枚: ix=4..=12 (0.2〜0.6m 先の幽霊壁) と ix=20 (1.0m 先、
        // レーザが実際に当たる本物)。ロボットは (0,0) 正面、range=1.0m。
        for ix in 4..=12 {
            map.data[ix] = 100;
        }
        map.data[20] = 100;
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![1.0] };

        let at: Vec<usize> = (0..=20).map(|ix| vi.base.to_index(ix, 0, 0) as usize).collect();
        let margin = at[13]; // free だが膨張帯 (壁 12 の隣)
        assert!(!vi.base.states[at[6]].free, "地図では壁");
        assert!(vi.base.states[margin].penalty > PROB_BASE, "地図では膨張帯");

        vi.clear_map_from_scan = true;
        vi.set_local_cost(&scan, 0.0, 0.0, 0.0);

        // **連続していること** — 通路になるかどうかはここで決まる。0.1r 刻みの
        // 9 点サンプリングだと 2 セルおきにしか開かず、この assert が落ちる。
        for ix in 0..=18 {
            assert!(vi.base.states[at[ix]].free, "ビームが通った ix={ix} が開くこと");
        }
        assert_eq!(vi.base.states[margin].penalty, PROB_BASE, "膨張帯も落ちること");
        assert!(!vi.base.states[at[20]].free, "ヒット点は貫通していない — 開けないこと");
    }

    /// inflate_by_sigma: 静的マージンの**外側**にだけ帯が立ち、静的帯の中と
    /// 帯の外は触らないこと (上田ら 2023 のマージン膨張の 3D 版)。
    #[test]
    fn inflate_by_sigma_bands_outside_static_margin() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let mut map = free_grid(60, 60);
        map.data[30 * 60 + 30] = 100; // 中央 1 セルだけ障害物 (世界座標 1.5, 1.5)
        // res=0.05, safety_radius=0.1 → 静的マージン m0 = 2 セル。
        vi.set_map_with_occupancy_grid(&map, 4, 0.1, 30.0, 0.2, 10);
        vi.set_local_window(1.5, 1.5, 0.0);
        let pen_at = |vi: &ValueIteratorLocal, ix, iy| {
            vi.base.states[vi.base.to_index(ix, iy, 0) as usize].local_penalty
        };

        vi.inflate_by_sigma(0.15); // σ 3 セルぶん → 帯は L∞ 距離 3..=5
        assert_eq!(pen_at(&vi, 32, 30), 0, "静的マージンの中 (d=2) は据え置き");
        assert_eq!(
            pen_at(&vi, 34, 30),
            (30.0 * PROB_BASE as f64) as u64,
            "帯 (d=4) は静的帯と同じ強さ"
        );
        assert_eq!(pen_at(&vi, 36, 30), 0, "帯の外 (d=6)");

        // 既存の local_penalty は消さない (スキャン注入の壁を残す)。
        let idx = vi.base.to_index(36, 30, 0) as usize;
        vi.base.states[idx].local_penalty = u64::MAX;
        vi.inflate_by_sigma(0.15);
        assert_eq!(vi.base.states[idx].local_penalty, u64::MAX);
    }

    /// 形状マスクの基本: `square` は本家の矩形そのまま (マスク素通り)、
    /// `circle` は角が落ちる、`square` に横半径を与えると異方の矩形になる。
    #[test]
    fn shapes_mask_the_window() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(200, 200); // res=0.05
        vi.set_map_with_occupancy_grid(&map, 4, 0.1, 30.0, 0.2, 10);

        // square (等方 1.0m): 従来と bit 一致 — 矩形の四隅も窓の中。
        vi.set_local_shape(LocalShapeConfig { range_m: 1.0, ..Default::default() });
        assert_eq!(vi.local_ixy_range, 20);
        vi.set_local_window(5.0, 5.0, 0.0); // ix=iy=100
        assert_eq!((vi.local_ix_min, vi.local_ix_max), (80, 120));
        assert!(vi.in_local_area(120, 120), "square は四隅も窓の中");

        // square + 横半径: 前後 2m x 左右 0.5m の地図軸矩形。
        vi.set_local_shape(LocalShapeConfig { range_m: 2.0, lat_m: 0.5, ..Default::default() });
        vi.set_local_window(5.0, 5.0, 0.0);
        assert_eq!((vi.local_ix_min, vi.local_ix_max), (60, 140));
        assert_eq!((vi.local_iy_min, vi.local_iy_max), (90, 110));
        assert_eq!(vi.local_ixy_range, 40, "パッチ寸法は両軸の最大");

        // circle: 外接矩形は同じでも四隅は落ちる。
        vi.set_local_shape(LocalShapeConfig {
            shape: LocalShape::Circle,
            range_m: 1.0,
            ..Default::default()
        });
        vi.set_local_window(5.0, 5.0, 0.0);
        assert_eq!((vi.local_ix_min, vi.local_ix_max), (80, 120));
        assert!(!vi.in_local_area(120, 120), "circle は四隅が落ちる");
        assert!(vi.in_local_area(119, 100), "正面 1m 弱は窓の中");
        assert!(vi.in_local_area(100, 119), "真横 1m 弱も窓の中 (等方)");
    }

    /// ellipse / capsule が走行方向へ伸び、後ろへ同量縮み (面積一定)、
    /// 制限 3 段 (デッドバンド・上下限・スルーレート) が効くこと。
    #[test]
    fn dynamic_shapes_stretch_toward_travel_and_are_rate_limited() {
        for shape in [LocalShape::Ellipse, LocalShape::Capsule] {
            let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
            let map = free_grid(400, 400);
            vi.set_map_with_occupancy_grid(&map, 4, 0.1, 30.0, 0.2, 10);
            let cfg = LocalShapeConfig {
                shape,
                range_m: 1.0,
                lat_m: 0.5,
                ratio_max: 2.0,
                slew_m: 0.05, // 1 呼び出しあたり 0.05 m
                fw_ref_m: 0.03,
                ..Default::default()
            };
            let (r, res) = (cfg.range_m, 0.05);
            vi.set_local_shape(cfg);
            // 外接半径は変形の上限込み: max(1.0*2.0, 0.5)/0.05 = 40 セル。
            assert_eq!(vi.local_ixy_range, 40, "{shape:?}");

            // 停止中 (デッドバンド以下) は据え置き。
            vi.set_local_window(5.0, 5.0, 0.0);
            vi.set_local_window(5.0 + res * 0.2, 5.0, 0.0);
            assert_eq!(vi.a_fwd, vi.a_bwd, "{shape:?}: 停止中は変形しない");

            // 前進を続ける: 前へ伸び、和は 2r のまま、1 tick の変化は slew 以下。
            let (mut x, mut prev) = (5.0, vi.a_fwd * res);
            for _ in 0..40 {
                x += 0.03;
                vi.set_local_window(x, 5.0, 0.0);
                let now = vi.a_fwd * res;
                assert!((now - prev).abs() <= cfg.slew_m + 1e-9, "{shape:?}: スルーレート違反");
                assert!(
                    ((vi.a_fwd + vi.a_bwd) * res - 2.0 * r).abs() < 1e-9,
                    "{shape:?}: 半軸の和 (= 面積) が一定でない"
                );
                prev = now;
            }
            let fwd = vi.a_fwd * res;
            assert!(fwd > r, "{shape:?}: 前進で前へ伸びること");
            // 上限は和一定に引かれて 2r - r/ratio_max = 1.5 m。
            assert!(fwd <= 2.0 * r - r / cfg.ratio_max + 1e-9, "{shape:?}: 上限を超えた");
            // マスクも実際に前後非対称になっていること (ix=100 が中心、+x が前方)。
            vi.set_local_window(5.0, 5.0, 0.0); // ix=iy=100 に置き直す (半軸は保つ)
            let ahead = (0..40).filter(|d| vi.in_local_area(100 + d, 100)).count();
            let behind = (0..40).filter(|d| vi.in_local_area(100 - d, 100)).count();
            assert!(ahead > behind, "{shape:?}: 前方の到達が後方より長いこと");

            // 後進に転じると非対称が反転する。
            for _ in 0..80 {
                x -= 0.03;
                vi.set_local_window(x, 5.0, 0.0);
            }
            assert!(vi.a_bwd > vi.a_fwd, "{shape:?}: 後進で後ろへ伸びること");
        }
    }

    /// capsule は前縁でも幅が `lat_m` のまま (楕円は尖って横被覆を失う)。
    #[test]
    fn capsule_keeps_its_width_at_the_leading_edge() {
        let map = free_grid(400, 400);
        let width_of = |shape| {
            let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
            vi.set_map_with_occupancy_grid(&map, 4, 0.1, 30.0, 0.2, 10);
            vi.set_local_shape(LocalShapeConfig {
                shape,
                range_m: 1.0,
                lat_m: 0.5,
                ..Default::default()
            });
            vi.set_local_window(5.0, 5.0, 0.0); // ix=iy=100
            // 前方 0.75m (15 セル) での横幅 [セル]
            (0..20).filter(|d| vi.in_local_area(115, 100 + d)).count()
        };
        assert!(
            width_of(LocalShape::Capsule) > width_of(LocalShape::Ellipse),
            "capsule は前方でも幅を保つこと"
        );
    }

    #[test]
    fn local_loop_runs_value_iteration_in_window() {
        let mut vi = ValueIteratorLocal::new(
            vec![
                Action::new("forward", 0.3, 0.0, 0),
                Action::new("left", 0.0, 20.0, 4),
            ],
            1,
        );
        let map = free_grid(60, 60);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        vi.base.set_goal(0.5, 0.5, 0); // window 内にゴール
        vi.set_local_window(0.5, 0.5, 0.0);
        // local ループを数回回すと window 内の到達可能セルが伝播する。
        for _ in 0..50 {
            vi.local_value_iteration_loop();
        }
        let reachable = (vi.local_ix_min..=vi.local_ix_max).any(|xx| {
            (vi.local_iy_min..=vi.local_iy_max).any(|yy| {
                let idx = vi.base.to_index(xx, yy, 0) as usize;
                let s = &vi.base.states[idx];
                !s.final_state && s.total_cost < crate::params::MAX_COST
            })
        });
        assert!(reachable, "local VI should propagate value within window");
    }
}
