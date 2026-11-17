//! Shared node/benchmark wiring; estimator algorithms remain separate.
use super::{ExternalLocalizer, Localizer};
use crate::belief::Belief;
use crate::bridge::PoseView;
use crate::msg::{LaserScan, OccupancyGrid};

/// The ordinary interface is shared; whole-map-only operations stay explicit.
/// Keep this outside the planner mutex so scans can correct while a solve runs.
pub enum Estimator {
    External(ExternalLocalizer),
    Windowed(Box<dyn Localizer>),
    Belief(Box<Belief>),
}

impl Estimator {
    fn localizer(&self) -> &dyn Localizer {
        match self {
            Self::External(l) => l,
            Self::Windowed(l) => l.as_ref(),
            Self::Belief(b) => b.as_ref(),
        }
    }

    fn localizer_mut(&mut self) -> &mut dyn Localizer {
        match self {
            Self::External(l) => l,
            Self::Windowed(l) => l.as_mut(),
            Self::Belief(b) => b.as_mut(),
        }
    }

    pub fn set_pose(&mut self, pose: PoseView) {
        self.localizer_mut().set_pose(pose);
    }

    pub fn predict(&mut self, v: f64, w_deg: f64, dt: f64) {
        self.localizer_mut().predict(v, w_deg, dt);
    }

    pub fn observe(&mut self, scan: &LaserScan) {
        self.localizer_mut().observe(scan);
    }

    /// Apply the whole-map free-space rebuttal before correcting the estimate.
    /// Windowed estimators do not support this operation.
    pub fn observe_with_map_clear(&mut self, scan: &LaserScan, map_clear: bool) {
        if map_clear {
            if let Self::Belief(b) = self {
                b.clear_free_from_scan(scan);
            }
        }
        self.observe(scan);
    }

    pub fn pose(&self) -> Option<PoseView> {
        self.localizer().pose()
    }

    pub fn quality(&self) -> f64 {
        self.localizer().quality()
    }

    pub fn top_cells(&self, k: usize) -> Vec<(PoseView, f64)> {
        self.localizer().top_cells(k)
    }

    pub fn reloc_targets(&self) -> Vec<(f64, f64)> {
        self.localizer().reloc_targets()
    }

    pub fn belief_grid(&self) -> Option<OccupancyGrid> {
        self.localizer().belief_grid()
    }

    /// Benchmark diagnostics: windowed/external estimators have no whole-map ESS.
    pub fn ess(&self) -> f64 {
        match self {
            Self::External(_) | Self::Windowed(_) => f64::NAN,
            Self::Belief(b) => b.ess(),
        }
    }

    pub fn probe_weight(&self, pose: PoseView) -> (f64, f64, Option<PoseView>) {
        match self {
            Self::External(_) | Self::Windowed(_) => (f64::NAN, f64::NAN, None),
            Self::Belief(b) => b.probe_weight(pose),
        }
    }

    pub fn in_probation(&self) -> bool {
        match self {
            Self::External(_) | Self::Windowed(_) => false,
            Self::Belief(b) => b.in_probation(),
        }
    }

    pub fn fail_probation(&mut self) {
        if let Self::Belief(b) = self {
            b.fail_probation();
        }
    }
}
