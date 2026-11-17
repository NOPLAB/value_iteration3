//! Ordinary localizer operations for the whole-map estimator.
use super::Belief;
use crate::bridge::PoseView;
use crate::localize::Localizer;
use crate::msg::{LaserScan, OccupancyGrid};

impl Localizer for Belief {
    fn name(&self) -> &'static str {
        if self.cfg.viterbi { "viterbi" } else { "belief" }
    }

    fn set_pose(&mut self, pose: PoseView) {
        self.seed(pose);
    }

    fn predict(&mut self, v: f64, w_deg: f64, dt: f64) {
        Belief::predict(self, v, w_deg, dt);
    }

    fn observe(&mut self, scan: &LaserScan) {
        Belief::observe(self, scan);
    }

    fn pose(&self) -> Option<PoseView> {
        Belief::pose(self)
    }

    fn quality(&self) -> f64 {
        Belief::quality(self)
    }

    fn top_cells(&self, k: usize) -> Vec<(PoseView, f64)> {
        Belief::top_cells(self, k)
    }

    fn reloc_targets(&self) -> Vec<(f64, f64)> {
        Belief::reloc_targets(self)
    }

    fn belief_grid(&self) -> Option<OccupancyGrid> {
        self.grid()
    }
}
