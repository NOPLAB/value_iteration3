//! 本家 6 アクションでの小マップ end-to-end。

use vi_reference::params::MAX_COST;
use vi_reference::{Action, OccupancyGrid, ValueIterator};

fn default_actions() -> Vec<Action> {
    vec![
        Action::new("forward", 0.3, 0.0, 0),
        Action::new("back", -0.2, 0.0, 1),
        Action::new("right", 0.0, -20.0, 2),
        Action::new("rightfw", 0.2, -20.0, 3),
        Action::new("left", 0.0, 20.0, 4),
        Action::new("leftfw", 0.2, 20.0, 5),
    ]
}

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
fn small_map_value_iteration_end_to_end() {
    let mut vi = ValueIterator::new(default_actions(), 1);
    let map = free_grid(8, 8);
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
    vi.set_goal(0.2, 0.2, 0); // セル (4,4) 付近

    vi.run_value_iteration(1000);

    // ゴール近傍に到達可能なセルが存在し、価値関数が抽出できる。
    let vf = vi.value_function_writer();
    assert_eq!(vf.layers.len(), 60);
    let threshold = MAX_COST as f64 / vi_reference::params::PROB_BASE as f64;
    let any_reachable = vf
        .layers
        .iter()
        .any(|layer| layer.iter().any(|&v| v < threshold));
    assert!(any_reachable, "value function should contain reachable cells");

    // policy も抽出できる。
    let pol = vi.policy_writer();
    assert_eq!(pol.layers.len(), 60);
}

#[test]
fn obstacle_cell_stays_max_cost() {
    // 障害物セルは not free → value_iteration がスキップ → total_cost は MAX_COST のまま。
    let mut data = vec![0i8; 5 * 5];
    data[(2 + 5 * 2) as usize] = 100; // 中央 (2,2) に障害物
    let map = OccupancyGrid {
        width: 5,
        height: 5,
        resolution: 0.05,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Default::default(),
        data,
    };
    let mut vi = ValueIterator::new(default_actions(), 1);
    vi.set_map_with_occupancy_grid(&map, 60, 0.0, 30.0, 0.2, 10); // safety_radius=0 → margin 0
    vi.set_goal(0.0, 0.0, 0);
    vi.run_value_iteration(500);

    let obs = vi.to_index(2, 2, 0) as usize;
    assert!(!vi.states[obs].free, "obstacle cell must be not free");
    assert_eq!(
        vi.states[obs].total_cost,
        MAX_COST,
        "obstacle cell is skipped and stays at MAX_COST"
    );
}
