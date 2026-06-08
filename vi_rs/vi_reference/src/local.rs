//! 本家 `ValueIteratorLocal` 忠実移植。`ValueIterator` を内包 (合成) し override を再定義。
//! local の `actionCostLocal` は本家 `actionCost` と完全同一なので base 経由で計算する。

use std::f64::consts::PI;

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

    /// 本家 `localValueIterationWorker`。status が canceled/goal の間 executing に書き換え、
    /// その後 status が canceled/goal になるまで local ループを回す (背景スレッド前提)。
    /// 注: 決定的テストでは `local_value_iteration_loop` を直接呼ぶこと。
    pub fn local_value_iteration_worker(&mut self, _id: i32) {
        while self.base.status == "canceled" || self.base.status == "goal" {
            self.base.status = "executing".to_string();
        }
        while self.base.status != "canceled" && self.base.status != "goal" {
            self.local_value_iteration_loop();
        }
    }

    /// 本家 `setLocalCost`。レーザヒット点周辺に local_penalty を設定/半減。
    pub fn set_local_cost(&mut self, msg: &LaserScan, x: f64, y: f64, t: f64) {
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
                        self.base.states[index].local_penalty = 2048u64 << PROB_BASE_BIT;
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

    /// 本家 `ValueIteratorLocal::posToAction` (override)。
    pub fn pos_to_action(&mut self, x: f64, y: f64, t_rad: f64) -> Option<usize> {
        let ix = ((x - self.base.map_origin_x) / self.base.xy_resolution).floor() as i32;
        let iy = ((y - self.base.map_origin_y) / self.base.xy_resolution).floor() as i32;
        let t = (180.0 * t_rad / PI) as i32;
        let it = (((t + 360 * 100) % 360) as f64 / self.base.t_resolution).floor() as i32;
        let index = self.base.to_index(ix, iy, it) as usize;
        if self.base.states[index].final_state {
            self.base.status = "goal".to_string();
            None
        } else if self.base.states[index].optimal_action.is_some() {
            self.base.states[index].optimal_action
        } else {
            None
        }
    }

    /// 本家 `makeLocalValueFunctionMap`。
    pub fn make_local_value_function_map(
        &self,
        threshold: i32,
        x: f64,
        y: f64,
        yaw_rad: f64,
    ) -> OccupancyGrid {
        let nx_local = self.local_ixy_range * 2 + 1;
        let ny_local = self.local_ixy_range * 2 + 1;
        let it = ((((yaw_rad / PI * 180.0) as i32 + 360 * 100) % 360) as f64
            / self.base.t_resolution)
            .floor() as i32;
        let mut data: Vec<i8> = Vec::new();
        for yy in self.local_iy_min..=self.local_iy_max {
            for xx in self.local_ix_min..=self.local_ix_max {
                let index = self.base.to_index(xx, yy, it) as usize;
                let cost = self.base.states[index].total_cost as f64 / PROB_BASE as f64;
                let val: i32 = if cost < threshold as f64 {
                    (cost / threshold as f64 * 250.0) as i32
                } else if self.base.states[index].free {
                    250
                } else {
                    255
                };
                data.push(val as u8 as i8);
            }
        }
        OccupancyGrid {
            width: nx_local,
            height: ny_local,
            resolution: self.base.xy_resolution,
            origin_x: x - self.local_xy_range,
            origin_y: y - self.local_xy_range,
            origin_quat: self.base.map_origin_quat.clone(),
            data,
        }
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
