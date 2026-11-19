use std::f64::consts::TAU;

use vi_lib::bridge::PoseView;
use vi_lib::ctrl::unicycle_step;
use vi_lib::msg::LaserScan;

use super::INVALID_RANGE_M;

/// Latest scan hits in map coordinates, separate from the cost overlay.
#[derive(Default)]
pub(super) struct ScanCollision {
    hits: Vec<(f64, f64)>,
}

impl ScanCollision {
    pub(super) fn observe(&mut self, scan: &LaserScan, pose: PoseView) {
        self.hits.clear();
        let mut angle = pose.yaw_rad + scan.angle_min;
        for &range in &scan.ranges {
            if range > 0.0 && range < INVALID_RANGE_M && angle.is_finite() {
                let (sin, cos) = angle.sin_cos();
                self.hits.push((pose.x + range * cos, pose.y + range * sin));
            }
            angle += scan.angle_increment;
        }
    }

    pub(super) fn blocks(&self, pose: PoseView, fw: f64, rot_deg: f64, radius: f64) -> bool {
        if fw == 0.0 || radius <= 0.0 || self.hits.is_empty() {
            return false;
        }
        // Follow publishes the action deltas as constant velocities. One second
        // covers one action's travel; action costs do not change this horizon.
        let omega = rot_deg.to_radians();
        let (end_x, end_y, _) = unicycle_step(0.0, 0.0, 0.0, fw, omega, 1.0);
        let (sin, cos) = pose.yaw_rad.sin_cos();
        let reach = fw.abs() + radius;
        self.hits.iter().any(|&(hx, hy)| {
            let dx = hx - pose.x;
            let dy = hy - pose.y;
            let start = dx.hypot(dy);
            if start > reach {
                return false;
            }
            let x = cos * dx + sin * dy;
            let y = cos * dy - sin * dx;
            let closest = if omega.abs() < 1e-9 {
                (x - x.clamp(fw.min(0.0), fw.max(0.0))).hypot(y)
            } else {
                let turn_radius = fw / omega;
                let side = turn_radius.signum();
                // Angle from the arc's start radius to the hit, in the direction of travel.
                let angle =
                    (omega.signum() * (side * x).atan2(side * (turn_radius - y))).rem_euclid(TAU);
                if angle <= omega.abs() {
                    let (cx, cy, _) = unicycle_step(0.0, 0.0, 0.0, fw, omega, angle / omega.abs());
                    (x - cx).hypot(y - cy)
                } else {
                    start.min((x - end_x).hypot(y - end_y))
                }
            };
            if start <= radius {
                // Already overlapping: permit retreat or a tangent, not motion closer.
                closest + 1e-9 < start
            } else {
                closest <= radius
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_turning_motion_is_blocked_by_a_hit_on_the_arc_but_not_the_chord() {
        let pose = PoseView { x: 0.0, y: 0.0, yaw_rad: 0.0 };
        // The midpoint of a 0.5 m/s left semicircle is (r, r).
        let r = 0.5 / std::f64::consts::PI;
        let scan = LaserScan {
            angle_min: std::f64::consts::FRAC_PI_4,
            angle_increment: 0.0,
            ranges: vec![r * 2.0_f64.sqrt()],
        };
        let mut collision = ScanCollision::default();
        collision.observe(&scan, pose);

        assert!(collision.blocks(pose, 0.5, 180.0, 0.05));
        assert!(!collision.blocks(pose, 0.5, -180.0, 0.05));
    }

    #[test]
    fn overlapping_front_hit_allows_retreat_and_is_replaced_by_a_new_scan() {
        let pose = PoseView { x: 1.0, y: 2.0, yaw_rad: std::f64::consts::FRAC_PI_2 };
        let mut collision = ScanCollision::default();
        collision.observe(&LaserScan { ranges: vec![0.05], ..LaserScan::default() }, pose);

        assert!(!collision.blocks(pose, -0.2, 0.0, 0.2));
        assert!(collision.blocks(pose, 0.5, 0.0, 0.2));

        collision
            .observe(&LaserScan { ranges: vec![INVALID_RANGE_M], ..LaserScan::default() }, pose);
        assert!(!collision.blocks(pose, 0.5, 0.0, 0.2));
    }
}
