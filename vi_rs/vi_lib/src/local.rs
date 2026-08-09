//! 本家 `ValueIteratorLocal` 忠実移植。`ValueIterator` を内包 (合成) し override を再定義。
//! local の `actionCostLocal` は本家 `actionCost` と完全同一なので base 経由で計算する。

use crate::action::Action;
use crate::msg::{LaserScan, OccupancyGrid};
use crate::params::{PROB_BASE, PROB_BASE_BIT};
use crate::value_iterator::ValueIterator;

pub struct ValueIteratorLocal {
    pub base: ValueIterator,
    pub local_ix_min: i32,
    pub local_ix_max: i32,
    pub local_iy_min: i32,
    pub local_iy_max: i32,
    pub local_ixy_range: i32,
    pub local_xy_range: f64,
    /// レーザーが貫通したセルの**地図由来**コスト (`free` / `penalty`) も反証するか
    /// (本家に無い、既定 false)。`clear_local_penalty_around` と同じ理屈を地図側へ
    /// 当てたもの — ビームが通り抜けた以上そこは空いている。地図が壁と言っている
    /// なら地図が古いか自己位置がずれているかで、どちらにせよ通れる。
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
        self.local_xy_range = 1.0;
        self.local_ixy_range = (self.local_xy_range / self.base.xy_resolution) as i32;
        self.local_ix_min = 0;
        self.local_iy_min = 0;
        self.local_ix_max = self.local_ixy_range * 2;
        self.local_iy_max = self.local_ixy_range * 2;
        self.safety_radius = safety_radius;
        self.safety_radius_penalty = safety_radius_penalty;
        self.obstacle_dist = Vec::new(); // 地図が変わったので距離場は捨てる
    }

    /// ローカルウィンドウ半径 [m] を変更する (本家は 1.0 固定)。
    /// `set_map_with_occupancy_grid` が半径を 1.0 に戻すので、その後に呼ぶこと。
    pub fn set_local_xy_range(&mut self, range_m: f64) {
        self.local_xy_range = range_m;
        self.local_ixy_range = (range_m / self.base.xy_resolution) as i32;
        self.local_ix_min = 0;
        self.local_iy_min = 0;
        self.local_ix_max = self.local_ixy_range * 2;
        self.local_iy_max = self.local_ixy_range * 2;
    }

    /// (x,y) を中心とする半径 `radius_m` の正方形の local_penalty を 0 にする
    /// (footprint クリア、本家に無い)。ロボットが現にいる場所は free という
    /// 反証不能な証拠 — スキャンのゴースト壁が機体の真上で閉じるのを防ぐ。
    pub fn clear_local_penalty_around(&mut self, x: f64, y: f64, radius_m: f64) {
        let res = self.base.xy_resolution;
        let cx = ((x - self.base.map_origin_x) / res).floor() as i32;
        let cy = ((y - self.base.map_origin_y) / res).floor() as i32;
        let r = (radius_m / res).ceil() as i32;
        let nt = self.base.cell_num_t;
        for ix in (cx - r).max(0)..=(cx + r).min(self.base.cell_num_x - 1) {
            for iy in (cy - r).max(0)..=(cy + r).min(self.base.cell_num_y - 1) {
                for it in 0..nt {
                    let idx = self.base.to_index(ix, iy, it) as usize;
                    self.base.states[idx].local_penalty = 0;
                }
            }
        }
    }

    /// 本家 `inLocalArea`。
    fn in_local_area(&self, ix: i32, iy: i32) -> bool {
        ix >= self.local_ix_min
            && ix <= self.local_ix_max
            && iy >= self.local_iy_min
            && iy <= self.local_iy_max
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

            // 地図の壁/膨張帯の反証は上の d ループに相乗りできない: 0.1r 刻みの
            // 9 点では r が大きいほど間隔が空き (r=3m・res=0.05 で 6 セル飛ぶ)、
            // 開いた穴が繋がらないので通路にならない。ビーム線分をセル刻みで歩く。
            if self.clear_map_from_scan {
                let (ca, sa) = (a.cos(), a.sin());
                let step = res * 0.5;
                let stop = r - res; // ヒット点の手前まで — 実物の障害物は開けない
                let mut s = step;
                while s < stop {
                    let bx = ((x + s * ca - ox) / res).floor() as i32;
                    let by = ((y + s * sa - oy) / res).floor() as i32;
                    // ビームは直進するので、窓を出たら戻らない = 打ち切ってよい。
                    // range が inf のときの無限ループもこれで止まる。
                    if !self.in_local_area(bx, by) {
                        break;
                    }
                    for it in 0..nt {
                        let index = self.base.to_index(bx, by, it) as usize;
                        self.base.states[index].free = true;
                        self.base.states[index].penalty = PROB_BASE;
                    }
                    s += step;
                }
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
    }

    /// 本家 `setLocalWindow`。ロボット位置中心に local window をクランプ。
    pub fn set_local_window(&mut self, x: f64, y: f64) {
        let ix = ((x - self.base.map_origin_x) / self.base.xy_resolution).floor() as i32;
        let iy = ((y - self.base.map_origin_y) / self.base.xy_resolution).floor() as i32;
        let rng = self.local_ixy_range;
        self.local_ix_min = if ix - rng >= 0 { ix - rng } else { 0 };
        self.local_iy_min = if iy - rng >= 0 { iy - rng } else { 0 };
        self.local_ix_max = if ix + rng < self.base.cell_num_x {
            ix + rng
        } else {
            self.base.cell_num_x - 1
        };
        self.local_iy_max = if iy + rng < self.base.cell_num_y {
            iy + rng
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
        vi.set_local_window(2.5, 2.5); // ix=iy=50 → 10..=90
        assert_eq!(vi.local_ix_min, 10);
        assert_eq!(vi.local_ix_max, 90);
    }

    #[test]
    fn set_local_window_clamps() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(60, 60);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        // ロボットを原点に → ix=iy=0、range=20 → min は 0 にクランプ、max は 20。
        vi.set_local_window(0.0, 0.0);
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

    #[test]
    fn clear_local_penalty_around_erases_footprint() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(60, 60);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        let scan = LaserScan {
            angle_min: 0.0,
            angle_increment: 0.0,
            ranges: vec![0.1], // ヒット点 (2, 0) — ±2 ブロックがロボットのセルを覆う
        };
        vi.set_local_cost(&scan, 0.0, 0.0, 0.0);
        let robot = vi.base.to_index(0, 0, 0) as usize;
        assert_eq!(vi.base.states[robot].local_penalty, 2048u64 << PROB_BASE_BIT);
        vi.clear_local_penalty_around(0.0, 0.0, 0.1);
        assert_eq!(vi.base.states[robot].local_penalty, 0);
        // クリア半径の外 (ヒット点の右端) は残る。
        let outside = vi.base.to_index(4, 0, 0) as usize;
        assert_eq!(vi.base.states[outside].local_penalty, 2048u64 << PROB_BASE_BIT);
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
        vi.set_local_window(1.5, 1.5);
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
        vi.set_local_window(0.5, 0.5);
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
