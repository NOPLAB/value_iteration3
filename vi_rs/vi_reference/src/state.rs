//! 本家 `State` 忠実移植。コンストラクタ 2 種。
//! occupancy 版の margin penalty ループは線形 `pos` 境界のみで列範囲を見ない
//! **固有バグ**を保持する。

use crate::msg::OccupancyGrid;
use crate::params::{MAX_COST, PROB_BASE, PROB_BASE_BIT};

#[derive(Clone, Debug)]
pub struct State {
    pub total_cost: u64,
    pub penalty: u64,
    pub local_penalty: u64,
    pub ix: i32,
    pub iy: i32,
    pub it: i32,
    pub free: bool,
    pub final_state: bool,
    /// 本家 `Action *optimal_action_` → `actions` ベクタへの索引。
    pub optimal_action: Option<usize>,
}

impl State {
    /// 本家 `State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map,
    ///            int margin, double margin_penalty, int x_num)`。
    pub fn from_occupancy(
        x: i32,
        y: i32,
        theta: i32,
        map: &OccupancyGrid,
        margin: i32,
        margin_penalty: f64,
        x_num: i32,
    ) -> Self {
        // 本家: margin_penalty>1e10 で ROS_ERROR を出すだけ (計算続行)。ここでは省略。
        let mut s = State {
            ix: x,
            iy: y,
            it: theta,
            total_cost: MAX_COST,
            penalty: PROB_BASE,
            local_penalty: 0,
            final_state: false,
            optimal_action: None,
            free: false,
        };

        // free_ = (map.data[y*x_num + x] == 0)
        let idx0 = (y * x_num + x) as usize;
        s.free = map.data[idx0] == 0;
        if !s.free {
            return s;
        }

        // ★固有バグ: 境界判定が線形 pos のみ。ix2 が負/列外でも pos が [0,len) なら
        //   隣接行のセルを読む。本家 `map.data[iy*x_num + ix]` は `data[pos]` と同値。
        for ix2 in (-margin + x)..=(margin + x) {
            for iy2 in (-margin + y)..=(margin + y) {
                let pos: i64 = iy2 as i64 * x_num as i64 + ix2 as i64;
                if 0 <= pos && (pos as usize) < map.data.len() && map.data[pos as usize] != 0 {
                    s.penalty = (margin_penalty * PROB_BASE as f64) as u64 + PROB_BASE;
                }
            }
        }
        s
    }

    /// 本家 `State(int x, int y, int theta, unsigned int cost)`。
    pub fn from_cost(x: i32, y: i32, theta: i32, cost: u32) -> Self {
        let free = cost != 255;
        State {
            ix: x,
            iy: y,
            it: theta,
            total_cost: MAX_COST,
            penalty: if free { (cost as u64) << PROB_BASE_BIT } else { 0 },
            local_penalty: 0,
            final_state: false,
            optimal_action: None,
            free,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(width: i32, height: i32, data: Vec<i8>) -> OccupancyGrid {
        OccupancyGrid {
            width,
            height,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data,
        }
    }

    #[test]
    fn occupied_cell_is_not_free_and_returns_early() {
        // data[0] != 0 → not free, penalty 残置 (PROB_BASE)。
        let map = grid(2, 2, vec![100, 0, 0, 0]);
        let s = State::from_occupancy(0, 0, 0, &map, 0, 30.0, 2);
        assert!(!s.free);
        assert_eq!(s.penalty, PROB_BASE);
        assert_eq!(s.total_cost, MAX_COST);
    }

    #[test]
    fn free_cell_with_no_obstacle_in_margin_keeps_base_penalty() {
        let map = grid(3, 3, vec![0; 9]);
        let s = State::from_occupancy(1, 1, 0, &map, 1, 30.0, 3);
        assert!(s.free);
        assert_eq!(s.penalty, PROB_BASE);
    }

    #[test]
    fn free_cell_near_obstacle_gets_margin_penalty() {
        // 中央 free、隣接に障害物 → penalty = 30*PROB_BASE + PROB_BASE。
        let mut data = vec![0; 9];
        data[0] = 100; // (x=0,y=0) 障害物
        let map = grid(3, 3, data);
        let s = State::from_occupancy(1, 1, 0, &map, 1, 30.0, 3);
        assert!(s.free);
        assert_eq!(s.penalty, (30.0 * PROB_BASE as f64) as u64 + PROB_BASE);
    }

    #[test]
    fn margin_loop_row_crossing_bug_is_reproduced() {
        // ★バグ再現: x=0 の free セルで margin=1 とすると ix2=-1 が現れる。
        // iy2=1, ix2=-1 → pos = 1*width + (-1) = width-1 = 前の行(行0)の右端セル。
        // そこに障害物を置くと、列(x=-1)は本来マップ外なのに penalty が立つ。
        // width=3: pos=2 → data[2] (行0,x=2)。
        let mut data = vec![0; 9];
        data[2] = 100; // 行0,x=2 に障害物
        let map = grid(3, 3, data);
        // 対象セル (x=0, y=1)。margin=1 → ix2 ∈ {-1,0,1}, iy2 ∈ {0,1,2}。
        // iy2=0,ix2=2 は範囲外だが、iy2=1,ix2=-1 → pos=2 → data[2]!=0 でヒット。
        let s = State::from_occupancy(0, 1, 0, &map, 1, 30.0, 3);
        assert!(s.free);
        assert_eq!(
            s.penalty,
            (30.0 * PROB_BASE as f64) as u64 + PROB_BASE,
            "行跨ぎバグにより penalty が立つこと"
        );
    }

    #[test]
    fn from_cost_free_and_obstacle() {
        let free = State::from_cost(2, 3, 5, 100);
        assert!(free.free);
        assert_eq!(free.penalty, 100u64 << PROB_BASE_BIT);
        assert_eq!(free.total_cost, MAX_COST);

        let occ = State::from_cost(2, 3, 5, 255);
        assert!(!occ.free);
        assert_eq!(occ.penalty, 0);
    }
}
