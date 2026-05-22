//! Goal-area mask generation. Mirrors `vi_matlab/src/common/make_goal_mask.m`.

use crate::params::N_THETA;
use ndarray::Array3;

pub struct GoalSpec {
    pub xy_resolution: f64,
    pub map_origin_x: f64,
    pub map_origin_y: f64,
    pub goal_x: f64,
    pub goal_y: f64,
    pub goal_theta_deg: f64,
    pub goal_radius_m: f64,
    pub goal_margin_theta_deg: f64,
}

/// Build a boolean mask `[map_y, map_x, N_THETA]` marking cells inside the goal disk and theta window.
/// Mirrors `make_goal_mask.m`.
pub fn make_goal_mask(map_x: u32, map_y: u32, spec: &GoalSpec) -> Array3<bool> {
    let mx = map_x as usize;
    let my = map_y as usize;
    let mut mask = Array3::from_elem((my, mx, N_THETA), false);
    let t_resolution = 360.0 / N_THETA as f64;
    let r2_thresh = spec.goal_radius_m * spec.goal_radius_m;

    for iy in 0..my {
        for ix in 0..mx {
            let x0 = ix as f64 * spec.xy_resolution + spec.map_origin_x;
            let y0 = iy as f64 * spec.xy_resolution + spec.map_origin_y;
            let x1 = x0 + spec.xy_resolution;
            let y1 = y0 + spec.xy_resolution;

            let r0 = (x0 - spec.goal_x).powi(2) + (y0 - spec.goal_y).powi(2);
            let r1 = (x1 - spec.goal_x).powi(2) + (y1 - spec.goal_y).powi(2);
            if r0 >= r2_thresh || r1 >= r2_thresh {
                continue;
            }

            // WHY: wrapped_goal is always the 360-offset counterpart, per MATLAB lines 35-40.
            // It never equals goal_theta_deg itself — it's the wrap-around alias.
            let wrapped_goal = if spec.goal_theta_deg > 180.0 {
                spec.goal_theta_deg - 360.0
            } else {
                spec.goal_theta_deg + 360.0
            };
            let margin = spec.goal_margin_theta_deg;

            for it in 0..N_THETA {
                let t0 = it as f64 * t_resolution;
                let t1 = (it + 1) as f64 * t_resolution;
                // WHY: lower bound is checked against t0 (bin start), upper against t1 (bin end).
                // Bin [t0, t1) is "inside the margin window" only if both endpoints are inside.
                // Matches MATLAB make_goal_mask.m:41-44.
                let in_theta = (spec.goal_theta_deg - margin <= t0 && t1 <= spec.goal_theta_deg + margin)
                    || (wrapped_goal - margin <= t0 && t1 <= wrapped_goal + margin);
                if in_theta {
                    mask[[iy, ix, it]] = true;
                }
            }
        }
    }
    mask
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_mask_for_distant_goal() {
        let spec = GoalSpec {
            xy_resolution: 0.05,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            goal_x: 10.0,
            goal_y: 10.0,
            goal_theta_deg: 90.0,
            goal_radius_m: 0.30,
            goal_margin_theta_deg: 15.0,
        };
        let mask = make_goal_mask(4, 4, &spec);
        assert!(mask.iter().all(|&v| !v));
    }

    #[test]
    fn goal_at_center_marks_some_cells() {
        let spec = GoalSpec {
            xy_resolution: 0.05,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            goal_x: 0.20,
            goal_y: 0.20,
            goal_theta_deg: 90.0,
            goal_radius_m: 0.30,
            goal_margin_theta_deg: 15.0,
        };
        let mask = make_goal_mask(8, 8, &spec);
        assert!(mask.iter().any(|&v| v));
    }

    #[test]
    fn mask_cells_outside_disk_are_false() {
        // Converse test: cells where at least one corner falls outside the disk must be false.
        let spec = GoalSpec {
            xy_resolution: 0.05,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            goal_x: 0.20,
            goal_y: 0.20,
            goal_theta_deg: 90.0,
            goal_radius_m: 0.10,
            goal_margin_theta_deg: 15.0,
        };
        let map_x: u32 = 8;
        let map_y: u32 = 8;
        let mask = make_goal_mask(map_x, map_y, &spec);
        let r2_thresh = spec.goal_radius_m * spec.goal_radius_m;

        for iy in 0..map_y as usize {
            for ix in 0..map_x as usize {
                let x0 = ix as f64 * spec.xy_resolution + spec.map_origin_x;
                let y0 = iy as f64 * spec.xy_resolution + spec.map_origin_y;
                let x1 = x0 + spec.xy_resolution;
                let y1 = y0 + spec.xy_resolution;
                let r0 = (x0 - spec.goal_x).powi(2) + (y0 - spec.goal_y).powi(2);
                let r1 = (x1 - spec.goal_x).powi(2) + (y1 - spec.goal_y).powi(2);
                // If either corner is outside (or on) the disk, no theta should be true.
                if r0 >= r2_thresh || r1 >= r2_thresh {
                    for it in 0..N_THETA {
                        assert!(
                            !mask[[iy, ix, it]],
                            "mask[[{iy},{ix},{it}]] should be false when corner is outside disk"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn wrap_around_theta_at_350_deg_covers_near_zero() {
        // Golden output from MATLAB make_goal_mask.
        // goal_theta_deg=350, margin=20 → primary window [330,370] covers bins 55-59,
        // wrapped window centered at -10 → [-30,10] covers bin 0 only.
        let spec = GoalSpec {
            xy_resolution: 0.05,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            goal_x: 0.025,
            goal_y: 0.025,
            goal_theta_deg: 350.0,
            goal_radius_m: 0.04,
            goal_margin_theta_deg: 20.0,
        };
        let mask = make_goal_mask(1, 1, &spec);

        let expected: std::collections::HashSet<usize> =
            [0usize, 55, 56, 57, 58, 59].into_iter().collect();
        let mut actual = std::collections::HashSet::new();
        for it in 0..N_THETA {
            if mask[[0, 0, it]] {
                actual.insert(it);
            }
        }
        assert_eq!(actual, expected);
    }

    #[test]
    fn matches_matlab_make_goal_mask_5x5_center() {
        // Golden output from MATLAB make_goal_mask.
        // All 25 cells fit inside r=0.30 disk; theta window [75,105] covers bins 13-16.
        let spec = GoalSpec {
            xy_resolution: 0.05,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            goal_x: 0.125,
            goal_y: 0.125,
            goal_theta_deg: 90.0,
            goal_radius_m: 0.30,
            goal_margin_theta_deg: 15.0,
        };
        let mask = make_goal_mask(5, 5, &spec);

        assert_eq!(mask.iter().filter(|b| **b).count(), 100);
        let expected_thetas: std::collections::HashSet<usize> =
            [13usize, 14, 15, 16].into_iter().collect();
        for it in 0..N_THETA {
            let n_true_in_layer = (0..5usize)
                .flat_map(|iy| (0..5usize).map(move |ix| (iy, ix)))
                .filter(|&(iy, ix)| mask[[iy, ix, it]])
                .count();
            if expected_thetas.contains(&it) {
                assert_eq!(n_true_in_layer, 25, "theta {it} should have all 25 true");
            } else {
                assert_eq!(n_true_in_layer, 0, "theta {it} should have 0 true");
            }
        }
    }
}
