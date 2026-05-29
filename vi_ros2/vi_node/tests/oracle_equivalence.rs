//! Proves that the ROS-free bridge layer does not perturb vi_rs.
//!
//! Runs the bridge-constructed VIContext through Reference and compares to
//! a direct vi_fixtures-constructed VIContext (same goal, same transitions).
//! Bit-exact equality required.
//!
//! Run with the serial feature only so the comparison is Gauss-Seidel
//! against Gauss-Seidel:
//!
//!   cargo test -p vi_node --test oracle_equivalence --no-default-features

use ndarray::Array3;
use vi_algorithm::{Budget, Reference, Solver};
use vi_algorithm::context::{MapDims, VIContext};
use vi_core::{
    GoalSpec, MAX_VALUE, N_THETA, make_goal_mask,
};
use vi_fixtures::{generate_transitions, TransitionMode};
use vi_node::bridge::{
    occupancy_to_penalty, pose_to_goal_spec,
    OccupancyGridView, PenaltyParams, PoseView,
};

fn empty_grid(w: u32, h: u32, res: f64) -> OccupancyGridView<'static> {
    let data = vec![0i8; (w * h) as usize];
    let leaked: &'static [i8] = Box::leak(data.into_boxed_slice());
    OccupancyGridView {
        width: w, height: h, resolution: res,
        origin_x: 0.0, origin_y: 0.0,
        data: leaked,
    }
}

#[test]
fn bridge_constructs_same_context_as_direct_fixtures() {
    // 16x16 empty map, goal at the center, no obstacles.
    let w = 16u32;
    let h = 16u32;
    let res = 0.05;
    let grid = empty_grid(w, h, res);
    let params = PenaltyParams {
        safety_radius_m: 0.0, safety_radius_penalty: 0, unknown_as_obstacle: true,
    };
    let penalty_bridge = occupancy_to_penalty(&grid, &params);

    let pose = PoseView { x: (w as f64 / 2.0) * res, y: (h as f64 / 2.0) * res, yaw_rad: 0.0 };
    let spec = pose_to_goal_spec(&pose, &grid, 0.05, 15.0);
    let goal_mask = make_goal_mask(w, h, &spec);

    let trans = generate_transitions(TransitionMode::Full { xy_resolution: res });

    let mut value = Array3::<u16>::from_elem((h as usize, w as usize, N_THETA), MAX_VALUE);
    for ((iy, ix, it), &g) in goal_mask.indexed_iter() {
        if g { value[[iy, ix, it]] = 0; }
    }

    let ctx_bridge = VIContext {
        dims: MapDims { map_x: w, map_y: h },
        value, penalty: penalty_bridge.clone(), goal_mask: goal_mask.clone(),
        transitions: trans.unpack(),
    };

    // Direct construction: identical inputs, just bypasses the view wrappers.
    let spec_direct = GoalSpec {
        xy_resolution: res, map_origin_x: 0.0, map_origin_y: 0.0,
        goal_x: pose.x, goal_y: pose.y, goal_theta_deg: 0.0,
        goal_radius_m: 0.05, goal_margin_theta_deg: 15.0,
    };
    let goal_mask_direct = make_goal_mask(w, h, &spec_direct);
    let penalty_direct = ndarray::Array2::<u16>::zeros((h as usize, w as usize));
    let mut value_direct = Array3::<u16>::from_elem((h as usize, w as usize, N_THETA), MAX_VALUE);
    for ((iy, ix, it), &g) in goal_mask_direct.indexed_iter() {
        if g { value_direct[[iy, ix, it]] = 0; }
    }
    let trans_direct = generate_transitions(TransitionMode::Full { xy_resolution: res });

    let ctx_direct = VIContext {
        dims: MapDims { map_x: w, map_y: h },
        value: value_direct, penalty: penalty_direct, goal_mask: goal_mask_direct,
        transitions: trans_direct.unpack(),
    };

    // Sanity check inputs match before solving.
    assert_eq!(ctx_bridge.penalty, ctx_direct.penalty);
    assert_eq!(ctx_bridge.goal_mask, ctx_direct.goal_mask);
    assert_eq!(ctx_bridge.value, ctx_direct.value);

    let mut a = ctx_bridge.clone_value();
    let mut b = ctx_direct.clone_value();
    Reference { threshold: 0 }.run(&mut a, Budget::Sweeps(200));
    Reference { threshold: 0 }.run(&mut b, Budget::Sweeps(200));
    assert_eq!(a.value, b.value, "bridge-constructed context must match direct construction bit-exactly");
}
