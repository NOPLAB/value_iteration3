//! 本家 `ValueIteratorLocal` 忠実移植。`ValueIterator` を内包 (合成) し override を再定義。
//! local の `actionCostLocal` は本家 `actionCost` と完全同一なので base 経由で計算する。

use crate::action::Action;
use crate::msg::{LaserScan, OccupancyGrid};
use crate::params::PROB_BASE_BIT;
use crate::value_iterator::ValueIterator;

pub struct ValueIteratorLocal {
    pub base: ValueIterator,
    pub local_ix_min: i32,
    pub local_ix_max: i32,
    pub local_iy_min: i32,
    pub local_iy_max: i32,
    pub local_ixy_range: i32,
    pub local_xy_range: f64,
    /// 地図帰属サプレッションの半径 [セル] (0 = 無効 = 本家挙動)。ヒット点から
    /// この距離以内に静的な非 free セル (障害物・未知) があるとき、そのヒットは
    /// 「地図の壁の再投影 (pose 誤差ぶんずれたゴースト)」とみなして注入しない。
    pub scan_attribution_cells: i32,
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
            scan_attribution_cells: 0,
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

    /// 地図帰属サプレッションの半径 [m] を設定 (0 以下で無効 = 本家挙動)。
    /// `set_map_with_occupancy_grid` の後に呼ぶこと (解像度を読む)。
    pub fn set_scan_attribution_range(&mut self, range_m: f64) {
        self.scan_attribution_cells = if range_m > 0.0 {
            (range_m / self.base.xy_resolution).ceil() as i32
        } else {
            0
        };
    }

    /// (ix,iy) の周囲 `scan_attribution_cells` セル以内に静的な非 free セル
    /// (障害物・未知) があるか。free は xy にのみ依存するので θ=0 だけ見る。
    fn near_static_obstacle(&self, ix: i32, iy: i32) -> bool {
        let a = self.scan_attribution_cells;
        let (nx, ny) = (self.base.cell_num_x, self.base.cell_num_y);
        for iix in (ix - a).max(0)..=(ix + a).min(nx - 1) {
            for iiy in (iy - a).max(0)..=(iy + a).min(ny - 1) {
                if !self.base.states[self.base.to_index(iix, iiy, 0) as usize].free {
                    return true;
                }
            }
        }
        false
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

            // 地図帰属: ヒットが既知障害物の近傍なら、それは地図の壁の再投影
            // (pose 誤差ぶんずれたゴースト) とみなして注入しない — 静的 penalty が
            // 既にその壁を守っている。オープンスペースのヒット (人・箱 = 地図に
            // 無い障害物) だけが注入に残る。通過セルの半減 (上) はそのまま —
            // ビームが通った事実は姿勢がずれていても free の証拠になる。
            if self.scan_attribution_cells > 0 && self.near_static_obstacle(ix, iy) {
                continue;
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

    #[test]
    fn map_attributed_hits_are_not_injected() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let mut map = free_grid(60, 60);
        // (20, 0) 付近に地図障害物の列。ヒット (10, 0) はそこから 10 セル =
        // 0.5m 離れている。
        for iy in 0..5 {
            map.data[(iy * 60 + 20) as usize] = 100;
        }
        vi.set_map_with_occupancy_grid(&map, 60, 0.05, 30.0, 0.2, 10);
        let scan = LaserScan {
            angle_min: 0.0,
            angle_increment: 0.0,
            ranges: vec![0.5], // ヒット点 (10, 0)
        };
        let hit = vi.base.to_index(10, 0, 0) as usize;

        // 帰属半径 0.6m (12 セル) — 障害物列が圏内なので注入しない。
        vi.set_scan_attribution_range(0.6);
        vi.set_local_cost(&scan, 0.0, 0.0, 0.0);
        assert_eq!(vi.base.states[hit].local_penalty, 0);

        // 帰属半径 0.3m — 圏外なのでオープンスペースの新規障害物として注入する。
        vi.set_scan_attribution_range(0.3);
        vi.set_local_cost(&scan, 0.0, 0.0, 0.0);
        assert_eq!(vi.base.states[hit].local_penalty, 2048u64 << PROB_BASE_BIT);
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
