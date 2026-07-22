//! ROS-free conversion layer between ROS message views and vi_reference types.
//!
//! Bridge functions take "view" structs (plain borrowed POD) rather than ROS
//! message types. ROS nodes (vi_ros2/vi_node, vi_ros2/vi_global_planner) pull fields
//! out of `nav_msgs::msg::OccupancyGrid` / `geometry_msgs::msg::PoseStamped`
//! and construct these views, so this module stays pure and host-testable —
//! and is shared by every embedding (ROS2 nodes, future FFI).
//!
//! (旧 vi_ros2/vi_node/src/bridge.rs から移設。vi_global_planner と共有するため
//! vi_reference 本体に置く。)
//!
//! In the u64 (本家忠実) port the penalty field and goal mask are not built
//! here — `ValueIterator::set_map_with_occupancy_grid` + `set_goal` compute
//! them internally (in 18-bit fixed point). This module only (a) turns an
//! occupancy view into a `vi_reference::OccupancyGrid` the iterator can ingest,
//! and (b) renders a value slice to an `OccupancyGrid` `data[]` for publishing.

use ndarray::Array2;

use crate::msg::{OccupancyGrid, Quaternion};
use crate::params::{MAX_COST, PROB_BASE};

#[derive(Debug, Clone, Copy)]
pub struct OccupancyGridView<'a> {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: &'a [i8],
}

#[derive(Debug, Clone, Copy)]
pub struct PoseView {
    pub x: f64,
    pub y: f64,
    pub yaw_rad: f64,
}

/// `yaw_rad` → goal heading in degrees, wrapped into `[0, 360)`, truncated to an
/// `i32` for `ValueIterator::set_goal` (本家 `executeVi`: `int t = yaw*180/π`).
pub fn yaw_to_goal_theta_deg(yaw_rad: f64) -> i32 {
    let mut deg = yaw_rad.to_degrees();
    deg = ((deg % 360.0) + 360.0) % 360.0;
    deg as i32
}

/// Build a `vi_reference::OccupancyGrid` from an occupancy view.
///
/// `ValueIterator` treats `data == 0` as free and any non-zero as blocked, and
/// applies the safety-radius inflation itself, so this only needs to produce a
/// binary obstacle grid: free cells → `0`, blocked cells → `100`.
///
/// A nav `OccupancyGrid` cell is `0` free, `100` occupied, `-1` unknown. A cell
/// is blocked iff `v >= 100` or (`v < 0` and `unknown_as_obstacle`).
pub fn occupancy_view_to_vi_grid(
    grid: &OccupancyGridView,
    unknown_as_obstacle: bool,
) -> OccupancyGrid {
    let w = grid.width as usize;
    let h = grid.height as usize;
    assert_eq!(grid.data.len(), w * h, "OccupancyGrid data length mismatch");
    let data: Vec<i8> = grid
        .data
        .iter()
        .map(|&v| {
            let blocked = v >= 100 || (v < 0 && unknown_as_obstacle);
            if blocked {
                100
            } else {
                0
            }
        })
        .collect();
    OccupancyGrid {
        width: grid.width as i32,
        height: grid.height as i32,
        resolution: grid.resolution,
        origin_x: grid.origin_x,
        origin_y: grid.origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data,
    }
}

/// total_cost slice → `OccupancyGrid` `data[]` (length `width*height`).
///
/// - `total_cost == MAX_COST` (never reached) → `-1` (unknown).
/// - cost `0` → `0` (free / goal).
/// - otherwise `display = total_cost / PROB_BASE` (本家 int-division), linearly
///   mapped `0..=threshold_steps` → `0..=100`, clamped.
///
/// `threshold_steps` is `cost_drawing_threshold` in step (≈second) units, the
/// same unit 本家 `valueFunctionWriter` uses after dividing by PROB_BASE.
pub fn value_slice_to_occupancy(value: &Array2<u64>, threshold_steps: u64) -> Vec<i8> {
    let h = value.shape()[0];
    let w = value.shape()[1];
    let mut out = vec![0i8; w * h];
    for iy in 0..h {
        for ix in 0..w {
            let c = value[[iy, ix]];
            out[iy * w + ix] = if c >= MAX_COST {
                -1
            } else {
                let display = c / PROB_BASE;
                if threshold_steps == 0 {
                    if display == 0 {
                        0
                    } else {
                        100
                    }
                } else {
                    let scaled = display.saturating_mul(100) / threshold_steps;
                    if scaled >= 100 {
                        100
                    } else {
                        scaled as i8
                    }
                }
            };
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn yaw_wraps_into_zero_to_360() {
        assert_eq!(yaw_to_goal_theta_deg(-std::f64::consts::FRAC_PI_2), 270);
        assert_eq!(yaw_to_goal_theta_deg(std::f64::consts::FRAC_PI_2), 90);
        assert_eq!(yaw_to_goal_theta_deg(0.0), 0);
    }

    fn view(w: u32, h: u32, data: &[i8]) -> OccupancyGridView<'_> {
        OccupancyGridView { width: w, height: h, resolution: 0.05, origin_x: 0.0, origin_y: 0.0, data }
    }

    #[test]
    fn occupied_and_unknown_become_blocked() {
        let data = [0i8, 100, -1, 0];
        let g = occupancy_view_to_vi_grid(&view(2, 2, &data), true);
        assert_eq!(g.data, vec![0, 100, 100, 0]); // unknown -> blocked
        assert_eq!((g.width, g.height), (2, 2));
    }

    #[test]
    fn unknown_free_when_flag_unset() {
        let data = [-1i8, 0];
        let g = occupancy_view_to_vi_grid(&view(2, 1, &data), false);
        assert_eq!(g.data, vec![0, 0]); // unknown -> free
    }

    #[test]
    fn value_max_cost_renders_as_minus_one() {
        let mut v = Array2::<u64>::zeros((1, 1));
        v[[0, 0]] = MAX_COST;
        let d = value_slice_to_occupancy(&v, 60);
        assert_eq!(d[0], -1);
    }

    #[test]
    fn value_zero_renders_zero() {
        let v = Array2::<u64>::zeros((2, 3));
        let d = value_slice_to_occupancy(&v, 60);
        assert!(d.iter().all(|&x| x == 0));
    }

    #[test]
    fn value_above_threshold_clamps_to_100() {
        let mut v = Array2::<u64>::zeros((1, 1));
        // display = 100 steps, threshold 60 -> scaled 166 -> clamp 100.
        v[[0, 0]] = 100 * PROB_BASE;
        let d = value_slice_to_occupancy(&v, 60);
        assert_eq!(d[0], 100);
    }

    #[test]
    fn value_mid_scales_linearly() {
        let mut v = Array2::<u64>::zeros((1, 1));
        // display = 30 steps, threshold 60 -> 30*100/60 = 50.
        v[[0, 0]] = 30 * PROB_BASE;
        let d = value_slice_to_occupancy(&v, 60);
        assert_eq!(d[0], 50);
    }
}
