//! ROS-free conversion layer between ROS message views and vi_rs types.
//!
//! Bridge functions take "view" structs (plain borrowed POD) rather than
//! ROS message types. `main.rs` is responsible for pulling fields out of
//! `nav_msgs::msg::OccupancyGrid` / `geometry_msgs::msg::PoseStamped`
//! and constructing these views. Keeping this module ROS-free means
//! `cargo test -p vi_node --lib` runs without ROS installed.

use ndarray::{Array2, ArrayView3};
use vi_core::{
    ActionIdx, GoalSpec, Penalty, Value,
    MAX_VALUE, N_ACTIONS, PENALTY_OBSTACLE,
};

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

#[derive(Debug, Clone, Copy)]
pub struct PenaltyParams {
    pub safety_radius_m: f64,
    pub safety_radius_penalty: u16,
    /// Behavior for OccupancyGrid cells with value `-1` (unknown).
    /// vi_node uses `Obstacle` by default — matches the conservative
    /// reading of value_iteration when no cost map is provided.
    pub unknown_as_obstacle: bool,
}

/// `OccupancyGrid` (data values in `-1` or `0..=100`) → `Array2<Penalty>`
/// indexed as `[iy, ix]`.
///
/// - `data[iy * width + ix] == 100` (or `-1` when `unknown_as_obstacle`)
///   → `PENALTY_OBSTACLE`.
/// - free cells start at `0`, then any free cell within
///   `safety_radius_m` (chessboard distance in cells) of an obstacle
///   is set to `safety_radius_penalty` unless it is already obstacle.
pub fn occupancy_to_penalty(
    grid: &OccupancyGridView,
    params: &PenaltyParams,
) -> Array2<Penalty> {
    let w = grid.width as usize;
    let h = grid.height as usize;
    assert_eq!(grid.data.len(), w * h, "OccupancyGrid data length mismatch");
    let mut p = Array2::<Penalty>::zeros((h, w));
    let radius_cells = (params.safety_radius_m / grid.resolution).ceil() as i32;

    // First pass: obstacles.
    for iy in 0..h {
        for ix in 0..w {
            let v = grid.data[iy * w + ix];
            let obs = v >= 100 || (v < 0 && params.unknown_as_obstacle);
            if obs {
                p[[iy, ix]] = PENALTY_OBSTACLE;
            }
        }
    }

    // Second pass: dilation.
    if radius_cells > 0 && params.safety_radius_penalty > 0 {
        let r = radius_cells;
        for iy in 0..h {
            for ix in 0..w {
                if p[[iy, ix]] == PENALTY_OBSTACLE { continue; }
                let mut near_obs = false;
                'scan: for dy in -r..=r {
                    for dx in -r..=r {
                        let ny = iy as i32 + dy;
                        let nx = ix as i32 + dx;
                        if nx < 0 || ny < 0 || nx >= w as i32 || ny >= h as i32 { continue; }
                        if p[[ny as usize, nx as usize]] == PENALTY_OBSTACLE {
                            near_obs = true;
                            break 'scan;
                        }
                    }
                }
                if near_obs {
                    p[[iy, ix]] = params.safety_radius_penalty;
                }
            }
        }
    }

    p
}

/// `PoseStamped` → `GoalSpec`. `yaw_rad` is wrapped into `[0, 360)` deg.
/// `goal_x` / `goal_y` come from the pose directly (not cell-indexed —
/// `make_goal_mask` does the cell math).
pub fn pose_to_goal_spec(
    pose: &PoseView,
    grid: &OccupancyGridView,
    goal_radius_m: f64,
    goal_margin_theta_deg: f64,
) -> GoalSpec {
    let mut deg = pose.yaw_rad.to_degrees();
    deg = ((deg % 360.0) + 360.0) % 360.0;
    GoalSpec {
        xy_resolution: grid.resolution,
        map_origin_x: grid.origin_x,
        map_origin_y: grid.origin_y,
        goal_x: pose.x,
        goal_y: pose.y,
        goal_theta_deg: deg,
        goal_radius_m,
        goal_margin_theta_deg,
    }
}

/// Value slice → OccupancyGrid `data[]` (length `width*height`).
///
/// - `MAX_VALUE` (unreachable) → `-1` (unknown).
/// - 0 → 0 (free).
/// - Otherwise linearly mapped 0..=`threshold_value` → 0..=100, clamped.
///
/// `threshold_value` should be the value at which "cost is too high to draw
/// a meaningful gradient" — value_iteration uses `cost_drawing_threshold`
/// in u64-PROB_BASE space; here it is already in `Value` (u16) space.
pub fn value_slice_to_occupancy(
    value: ArrayView3<Value>,
    theta_idx: usize,
    threshold_value: Value,
) -> Vec<i8> {
    let h = value.shape()[0];
    let w = value.shape()[1];
    let mut out = vec![0i8; w * h];
    let denom = threshold_value as u32;
    for iy in 0..h {
        for ix in 0..w {
            let v = value[[iy, ix, theta_idx]];
            out[iy * w + ix] = if v == MAX_VALUE {
                -1
            } else if denom == 0 {
                if v == 0 { 0 } else { 100 }
            } else {
                let scaled = (v as u32 * 100) / denom;
                if scaled >= 100 { 100 } else { scaled as i8 }
            };
        }
    }
    out
}

/// Optimal-action slice (theta_idx) → OccupancyGrid `data[]`.
/// Action ids `0..N_ACTIONS` map to evenly spaced 0..100 buckets.
pub fn action_table_to_occupancy(
    actions: ArrayView3<ActionIdx>,
    theta_idx: usize,
) -> Vec<i8> {
    let h = actions.shape()[0];
    let w = actions.shape()[1];
    let mut out = vec![0i8; w * h];
    let step = 100 / N_ACTIONS as i32;
    for iy in 0..h {
        for ix in 0..w {
            let a = actions[[iy, ix, theta_idx]] as i32;
            out[iy * w + ix] = (a * step).min(100) as i8;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(w: u32, h: u32, res: f64, data: Vec<i8>) -> (Vec<i8>, OccupancyGridView<'static>) {
        // Leak the data to obtain a 'static slice for ergonomic test setup.
        let leaked: &'static [i8] = Box::leak(data.clone().into_boxed_slice());
        (data, OccupancyGridView {
            width: w, height: h, resolution: res,
            origin_x: 0.0, origin_y: 0.0,
            data: leaked,
        })
    }

    fn params(rad: f64, pen: u16, unk_as_obs: bool) -> PenaltyParams {
        PenaltyParams {
            safety_radius_m: rad,
            safety_radius_penalty: pen,
            unknown_as_obstacle: unk_as_obs,
        }
    }

    #[test]
    fn all_free_yields_all_zero() {
        let (_, g) = grid(4, 3, 0.05, vec![0; 12]);
        let p = occupancy_to_penalty(&g, &params(0.0, 0, true));
        assert!(p.iter().all(|&v| v == 0));
    }

    #[test]
    fn obstacle_value_100_is_marked() {
        let mut data = vec![0i8; 9];
        data[4] = 100;
        let (_, g) = grid(3, 3, 0.05, data);
        let p = occupancy_to_penalty(&g, &params(0.0, 0, true));
        assert_eq!(p[[1, 1]], PENALTY_OBSTACLE);
    }

    #[test]
    fn unknown_treated_as_obstacle_when_flag_set() {
        let (_, g) = grid(2, 1, 0.05, vec![-1, 0]);
        let p = occupancy_to_penalty(&g, &params(0.0, 0, true));
        assert_eq!(p[[0, 0]], PENALTY_OBSTACLE);
        assert_eq!(p[[0, 1]], 0);
    }

    #[test]
    fn unknown_treated_as_free_when_flag_unset() {
        let (_, g) = grid(2, 1, 0.05, vec![-1, 0]);
        let p = occupancy_to_penalty(&g, &params(0.0, 0, false));
        assert_eq!(p[[0, 0]], 0);
    }

    #[test]
    fn safety_radius_one_cell_dilation() {
        // 5x5, single obstacle in the center. radius=0.05m, res=0.05m → 1 cell.
        let mut data = vec![0i8; 25];
        data[12] = 100;
        let (_, g) = grid(5, 5, 0.05, data);
        let p = occupancy_to_penalty(&g, &params(0.05, 42, true));
        assert_eq!(p[[2, 2]], PENALTY_OBSTACLE);
        // Immediate neighbours (chessboard 1) get the dilation value.
        for (iy, ix) in [(1,1),(1,2),(1,3),(2,1),(2,3),(3,1),(3,2),(3,3)] {
            assert_eq!(p[[iy, ix]], 42, "dilated cell ({iy},{ix}) must be 42");
        }
        // Distance-2 cells stay 0.
        assert_eq!(p[[0, 0]], 0);
        assert_eq!(p[[4, 4]], 0);
    }

    use ndarray::Array3;
    use vi_core::params::N_THETA;

    #[test]
    fn yaw_wraps_into_zero_to_360() {
        let g = OccupancyGridView { width:1,height:1,resolution:0.05,origin_x:0.0,origin_y:0.0, data: &[0i8] };
        let p = PoseView { x:0.0, y:0.0, yaw_rad: -std::f64::consts::FRAC_PI_2 };
        let spec = pose_to_goal_spec(&p, &g, 0.1, 15.0);
        assert!((spec.goal_theta_deg - 270.0).abs() < 1e-9, "got {}", spec.goal_theta_deg);
    }

    #[test]
    fn yaw_positive_quarter() {
        let g = OccupancyGridView { width:1,height:1,resolution:0.05,origin_x:0.0,origin_y:0.0, data: &[0i8] };
        let p = PoseView { x:0.0, y:0.0, yaw_rad: std::f64::consts::FRAC_PI_2 };
        let spec = pose_to_goal_spec(&p, &g, 0.1, 15.0);
        assert!((spec.goal_theta_deg - 90.0).abs() < 1e-9);
    }

    #[test]
    fn value_max_renders_as_minus_one() {
        let mut v = Array3::<Value>::zeros((1, 1, N_THETA));
        v[[0, 0, 0]] = MAX_VALUE;
        let d = value_slice_to_occupancy(v.view(), 0, 1000);
        assert_eq!(d[0], -1);
    }

    #[test]
    fn value_zero_renders_zero() {
        let v = Array3::<Value>::zeros((2, 3, N_THETA));
        let d = value_slice_to_occupancy(v.view(), 0, 1000);
        assert!(d.iter().all(|&x| x == 0));
    }

    #[test]
    fn value_above_threshold_clamps_to_100() {
        let mut v = Array3::<Value>::zeros((1, 1, N_THETA));
        v[[0, 0, 0]] = 5000;
        let d = value_slice_to_occupancy(v.view(), 0, 1000);
        assert_eq!(d[0], 100);
    }

    #[test]
    fn action_table_maps_to_evenly_spaced_buckets() {
        let mut a = Array3::<ActionIdx>::zeros((1, N_ACTIONS, N_THETA));
        for i in 0..N_ACTIONS { a[[0, i, 0]] = i as ActionIdx; }
        let d = action_table_to_occupancy(a.view(), 0);
        assert_eq!(d[0], 0);
        assert_eq!(d[5], (5 * (100 / N_ACTIONS as i32)) as i8);
    }
}
