//! Re-export of the ROS-free conversion layer.
//!
//! The implementation moved to `vi_reference::bridge` so vi_node and
//! vi_planner share it (and future FFI embeddings can reuse it). This
//! module remains so `vi_node::bridge::...` import paths keep working.

pub use vi_reference::bridge::{
    occupancy_view_to_vi_grid, value_slice_to_occupancy, yaw_to_goal_theta_deg,
    OccupancyGridView, PoseView,
};
