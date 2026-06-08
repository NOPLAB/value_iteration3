//! 本家 `ValueIterator` 忠実移植 (フルパイプライン)。

use std::collections::BTreeMap;
use std::f64::consts::PI;

use crate::action::Action;
use crate::msg::{OccupancyGrid, Quaternion};
use crate::params::{MAX_COST, PROB_BASE, PROB_BASE_BIT, RESOLUTION_T_BIT, RESOLUTION_XY_BIT};
use crate::state::State;
use crate::state_transition::StateTransition;
use crate::sweep_status::SweepWorkerStatus;

pub struct ValueIterator {
    pub states: Vec<State>,
    pub actions: Vec<Action>,
    pub sweep_orders: Vec<Vec<i32>>,
    pub thread_status: BTreeMap<i32, SweepWorkerStatus>,
    pub status: String,

    pub goal_x: f64,
    pub goal_y: f64,
    pub goal_margin_radius: f64,
    pub goal_t: i32,
    pub goal_margin_theta: i32,
    pub thread_num: i32,

    pub xy_resolution: f64,
    pub t_resolution: f64,
    pub cell_num_x: i32,
    pub cell_num_y: i32,
    pub cell_num_t: i32,
    pub map_origin_x: f64,
    pub map_origin_y: f64,
    pub map_origin_quat: Quaternion,
}

impl ValueIterator {
    /// 本家 `ValueIterator(std::vector<Action> &actions, int thread_num)`。
    pub fn new(actions: Vec<Action>, thread_num: i32) -> Self {
        Self {
            states: Vec::new(),
            actions,
            sweep_orders: Vec::new(),
            thread_status: BTreeMap::new(),
            status: "init".to_string(),
            goal_x: 0.0,
            goal_y: 0.0,
            goal_margin_radius: 0.0,
            goal_t: 0,
            goal_margin_theta: 0,
            thread_num,
            xy_resolution: 0.0,
            t_resolution: 0.0,
            cell_num_x: 0,
            cell_num_y: 0,
            cell_num_t: 0,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            map_origin_quat: Quaternion::default(),
        }
    }

    /// 本家 `toIndex(ix,iy,it) = it + ix*cell_num_t_ + iy*(cell_num_t_*cell_num_x_)`。
    pub fn to_index(&self, ix: i32, iy: i32, it: i32) -> i32 {
        to_index_raw(ix, iy, it, self.cell_num_x, self.cell_num_t)
    }

    /// 本家 `inMapArea`。
    pub fn in_map_area(&self, ix: i32, iy: i32) -> bool {
        ix >= 0 && ix < self.cell_num_x && iy >= 0 && iy < self.cell_num_y
    }
}

// ── コア free 関数 (単スレッド経路とマルチスレッド経路で共有) ──

#[inline]
pub(crate) fn to_index_raw(ix: i32, iy: i32, it: i32, cell_num_x: i32, cell_num_t: i32) -> i32 {
    it + ix * cell_num_t + iy * (cell_num_t * cell_num_x)
}

/// 本家 `cellDelta`。`it` は絶対インデックス (負正規化しない)。
pub(crate) fn cell_delta(
    x: f64,
    y: f64,
    t: f64,
    xy_resolution: f64,
    t_resolution: f64,
) -> (i32, i32, i32) {
    let mut ix = (x.abs() / xy_resolution).floor() as i32;
    if x < 0.0 {
        ix = -ix - 1;
    }
    let mut iy = (y.abs() / xy_resolution).floor() as i32;
    if y < 0.0 {
        iy = -iy - 1;
    }
    let it = (t / t_resolution).floor() as i32;
    (ix, iy, it)
}

/// 本家 `noNoiseStateTransition`。`to_t` は負方向しか正規化しない (>=360 は残す)。
pub(crate) fn no_noise_state_transition(
    delta_fw: f64,
    delta_rot: f64,
    from_x: f64,
    from_y: f64,
    from_t: f64,
) -> (f64, f64, f64) {
    let ang = from_t / 180.0 * PI;
    let to_x = from_x + delta_fw * ang.cos();
    let to_y = from_y + delta_fw * ang.sin();
    let mut to_t = from_t + delta_rot;
    while to_t < 0.0 {
        to_t += 360.0;
    }
    (to_x, to_y, to_t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn to_index_layout() {
        // cell_num_x=4, cell_num_t=60。
        assert_eq!(to_index_raw(0, 0, 0, 4, 60), 0);
        assert_eq!(to_index_raw(0, 0, 5, 4, 60), 5);
        assert_eq!(to_index_raw(1, 0, 0, 4, 60), 60);
        assert_eq!(to_index_raw(0, 1, 0, 4, 60), 240);
    }

    #[test]
    fn cell_delta_negative_correction() {
        // xy_res=0.05。x=-0.01 → |x|/res=0.2 → floor 0 → x<0 → -0-1 = -1。
        let (ix, _, _) = cell_delta(-0.01, 0.0, 0.0, 0.05, 6.0);
        assert_eq!(ix, -1);
        // x=0.06 → 1.2 → floor 1。
        let (ix2, _, _) = cell_delta(0.06, 0.0, 0.0, 0.05, 6.0);
        assert_eq!(ix2, 1);
    }

    #[test]
    fn cell_delta_theta_absolute_not_normalized() {
        // t=366, t_res=6 → floor(61) = 61 (絶対、wrap しない)。
        let (_, _, it) = cell_delta(0.0, 0.0, 366.0, 0.05, 6.0);
        assert_eq!(it, 61);
    }

    #[test]
    fn no_noise_negative_theta_normalized_once() {
        // from_t=10, rot=-20 → to_t=-10 → +360 = 350。
        let (_, _, to_t) = no_noise_state_transition(0.0, -20.0, 0.0, 0.0, 10.0);
        assert!((to_t - 350.0).abs() < 1e-9);
    }

    #[test]
    fn no_noise_over_360_not_normalized() {
        // from_t=350, rot=20 → to_t=370 (>=360 は残す)。
        let (_, _, to_t) = no_noise_state_transition(0.0, 20.0, 0.0, 0.0, 350.0);
        assert!((to_t - 370.0).abs() < 1e-9);
    }

    #[test]
    fn no_noise_forward_uses_cos_sin() {
        // fw=0.3, from_t=0 → to_x=0.3, to_y=0。
        let (to_x, to_y, _) = no_noise_state_transition(0.3, 0.0, 0.0, 0.0, 0.0);
        assert!((to_x - 0.3).abs() < 1e-9);
        assert!(to_y.abs() < 1e-9);
    }
}
