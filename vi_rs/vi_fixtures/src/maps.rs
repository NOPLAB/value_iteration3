//! Map fixture generation. Mirrors `vi_matlab/workflows/validation/tests/gen_test_map.m`.

use ndarray::{Array2, Array3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use vi_core::{GoalSpec, Value, Penalty, make_goal_mask,
              MAX_VALUE, PENALTY_OBSTACLE, N_THETA};

#[derive(Clone, Copy, Debug)]
pub enum MapType {
    Empty,
    Obstacle,
    Sentinel,
    Random { density: f64, seed: u64 },
}

/// 0-indexed map fixture. `goal_x`/`goal_y` are cell indices into `penalty`/`value`.
pub struct GeneratedMap {
    pub value: Array3<Value>,
    pub penalty: Array2<Penalty>,
    pub goal_mask: Array3<bool>,
    pub goal_x: u32,
    pub goal_y: u32,
    pub spec: GoalSpec,
}

pub fn generate_map(map_x: u32, map_y: u32, ty: MapType) -> GeneratedMap {
    let mx = map_x as usize;
    let my = map_y as usize;

    let mut value = Array3::from_elem((my, mx, N_THETA), MAX_VALUE);
    let mut penalty = Array2::<Penalty>::zeros((my, mx));

    let goal_x = (map_x - 1) / 2;
    let goal_y = (map_y - 1) / 2;
    let gx = goal_x as usize;
    let gy = goal_y as usize;

    match ty {
        MapType::Empty => {}
        MapType::Obstacle => {
            let wall_y = gy.saturating_sub(3);
            for wy in wall_y..=(wall_y + 1).min(my - 1) {
                for wx in gx.saturating_sub(3)..=(gx + 3).min(mx - 1) {
                    penalty[[wy, wx]] = PENALTY_OBSTACLE;
                }
            }
        }
        MapType::Sentinel => {
            if gy > 0 {
                penalty[[gy - 1, gx]] = PENALTY_OBSTACLE;
            }
            if gy + 1 < my {
                penalty[[gy + 1, gx]] = PENALTY_OBSTACLE;
            }
            if gx > 0 {
                penalty[[gy, gx - 1]] = PENALTY_OBSTACLE;
            }
        }
        MapType::Random { density, seed } => {
            let mut rng = StdRng::seed_from_u64(seed);
            for iy in 0..my {
                for ix in 0..mx {
                    if rng.gen::<f64>() < density {
                        penalty[[iy, ix]] = PENALTY_OBSTACLE;
                    }
                }
            }
            let keep_y_lo = gy.saturating_sub(1);
            let keep_y_hi = (gy + 1).min(my - 1);
            let keep_x_lo = gx.saturating_sub(1);
            let keep_x_hi = (gx + 1).min(mx - 1);
            for iy in keep_y_lo..=keep_y_hi {
                for ix in keep_x_lo..=keep_x_hi {
                    penalty[[iy, ix]] = 0;
                }
            }
        }
    }

    let spec = GoalSpec {
        xy_resolution: 0.05,
        map_origin_x: 0.0,
        map_origin_y: 0.0,
        goal_x: (goal_x as f64 + 0.5) * 0.05,
        goal_y: (goal_y as f64 + 0.5) * 0.05,
        goal_theta_deg: 90.0,
        goal_radius_m: 0.30,
        goal_margin_theta_deg: 15.0,
    };

    let goal_mask = make_goal_mask(map_x, map_y, &spec);

    for ((iy, ix, it), g) in goal_mask.indexed_iter() {
        if *g {
            value[[iy, ix, it]] = 0;
        }
    }

    GeneratedMap { value, penalty, goal_mask, goal_x, goal_y, spec }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Empty map tests ---

    #[test]
    fn empty_5x5_value_initialized_to_max() {
        let m = generate_map(5, 5, MapType::Empty);
        let non_goal_max = m.value.iter()
            .zip(m.goal_mask.iter())
            .filter(|(_, &g)| !g)
            .all(|(&v, _)| v == MAX_VALUE);
        assert!(non_goal_max, "non-goal cells must be MAX_VALUE");
    }

    #[test]
    fn empty_5x5_goal_cells_are_zero() {
        let m = generate_map(5, 5, MapType::Empty);
        let goals_zero = m.value.iter()
            .zip(m.goal_mask.iter())
            .filter(|(_, &g)| g)
            .all(|(&v, _)| v == 0);
        assert!(goals_zero, "goal cells must be 0");
    }

    #[test]
    fn empty_5x5_no_obstacles() {
        let m = generate_map(5, 5, MapType::Empty);
        assert!(m.penalty.iter().all(|&p| p != PENALTY_OBSTACLE));
    }

    #[test]
    fn empty_5x5_goal_at_center() {
        let m = generate_map(5, 5, MapType::Empty);
        assert_eq!(m.goal_x, 2);
        assert_eq!(m.goal_y, 2);
    }

    #[test]
    fn empty_8x8_goal_at_center() {
        let m = generate_map(8, 8, MapType::Empty);
        assert_eq!(m.goal_x, 3);
        assert_eq!(m.goal_y, 3);
    }

    #[test]
    fn empty_goal_mask_has_true_cells() {
        let m = generate_map(8, 8, MapType::Empty);
        assert!(m.goal_mask.iter().any(|&v| v));
    }

    // --- Obstacle map tests ---

    #[test]
    fn obstacle_8x8_exact_wall_layout() {
        // goal=(3,3), wall_y=0, wy∈{0,1}, wx∈{0..6} → 2×7=14 cells
        let m = generate_map(8, 8, MapType::Obstacle);
        let n_obs = m.penalty.iter().filter(|&&p| p == PENALTY_OBSTACLE).count();
        assert_eq!(n_obs, 14);
        for wy in 0..=1 {
            for wx in 0..=6 {
                assert_eq!(m.penalty[[wy, wx]], PENALTY_OBSTACLE,
                    "obstacle expected at ({wx},{wy})");
            }
        }
    }

    #[test]
    fn obstacle_5x5_exact_wall_layout() {
        // goal=(2,2), wall_y=0, wy∈{0,1}, wx∈{0..4} → 2×5=10 cells
        let m = generate_map(5, 5, MapType::Obstacle);
        let n_obs = m.penalty.iter().filter(|&&p| p == PENALTY_OBSTACLE).count();
        assert_eq!(n_obs, 10);
        for wy in 0..=1 {
            for wx in 0..=4 {
                assert_eq!(m.penalty[[wy, wx]], PENALTY_OBSTACLE,
                    "obstacle expected at ({wx},{wy})");
            }
        }
    }

    #[test]
    fn obstacle_8x8_goal_not_blocked() {
        let m = generate_map(8, 8, MapType::Obstacle);
        assert_ne!(m.penalty[[m.goal_y as usize, m.goal_x as usize]], PENALTY_OBSTACLE);
    }

    // --- Sentinel map tests ---

    #[test]
    fn sentinel_8x8_has_three_obstacles() {
        let m = generate_map(8, 8, MapType::Sentinel);
        let n_obs = m.penalty.iter().filter(|&&p| p == PENALTY_OBSTACLE).count();
        assert_eq!(n_obs, 3, "sentinel places obstacles in 3 cardinal directions");
    }

    #[test]
    fn sentinel_8x8_obstacles_adjacent_to_goal() {
        let m = generate_map(8, 8, MapType::Sentinel);
        let gx = m.goal_x as usize;
        let gy = m.goal_y as usize;
        assert_eq!(m.penalty[[gy - 1, gx]], PENALTY_OBSTACLE);
        assert_eq!(m.penalty[[gy + 1, gx]], PENALTY_OBSTACLE);
        assert_eq!(m.penalty[[gy, gx - 1]], PENALTY_OBSTACLE);
    }

    // --- Random map tests ---

    #[test]
    fn random_8x8_has_obstacles() {
        let m = generate_map(8, 8, MapType::Random { density: 0.15, seed: 42 });
        let n_obs = m.penalty.iter().filter(|&&p| p == PENALTY_OBSTACLE).count();
        assert!(n_obs > 0, "random map with density=0.15 should have obstacles");
    }

    #[test]
    fn random_8x8_goal_region_clear() {
        let m = generate_map(8, 8, MapType::Random { density: 0.5, seed: 42 });
        let gx = m.goal_x as usize;
        let gy = m.goal_y as usize;
        for dy in gy.saturating_sub(1)..=(gy + 1).min(7) {
            for dx in gx.saturating_sub(1)..=(gx + 1).min(7) {
                assert_ne!(m.penalty[[dy, dx]], PENALTY_OBSTACLE,
                    "3x3 around goal must be free at ({dx},{dy})");
            }
        }
    }

    #[test]
    fn random_reproducible_same_seed() {
        let m1 = generate_map(16, 16, MapType::Random { density: 0.15, seed: 99 });
        let m2 = generate_map(16, 16, MapType::Random { density: 0.15, seed: 99 });
        assert_eq!(m1.penalty, m2.penalty);
    }

    #[test]
    fn random_different_seed_differs() {
        let m1 = generate_map(16, 16, MapType::Random { density: 0.15, seed: 1 });
        let m2 = generate_map(16, 16, MapType::Random { density: 0.15, seed: 2 });
        assert_ne!(m1.penalty, m2.penalty);
    }

    // --- GoalSpec consistency ---

    #[test]
    fn spec_goal_position_matches_cell_center() {
        let m = generate_map(8, 8, MapType::Empty);
        let expected_x = (m.goal_x as f64 + 0.5) * 0.05;
        let expected_y = (m.goal_y as f64 + 0.5) * 0.05;
        assert!((m.spec.goal_x - expected_x).abs() < 1e-12);
        assert!((m.spec.goal_y - expected_y).abs() < 1e-12);
    }

    #[test]
    fn empty_5x5_penalty_is_zero_everywhere() {
        let m = generate_map(5, 5, MapType::Empty);
        assert!(m.penalty.iter().all(|&p| p == 0),
            "empty map penalty is 0 everywhere (MATLAB convention)");
    }
}
