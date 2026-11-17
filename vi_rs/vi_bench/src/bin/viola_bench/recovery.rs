//! Recovery episode state; simulation, random draws and measurements stay in the caller.
use super::wrap_rad;
use vi_lib::bridge::PoseView;
use vi_lib::msg::LaserScan;

const RELEASE_GUARD_TICKS: usize = 600;

pub(super) struct Recovery {
    pub(super) field_is_reloc: bool,
    pub(super) ticks: u32,
    pub(super) gave_up: bool,
    frozen: Option<Vec<(f64, f64)>>,
    delta: Option<(f64, f64)>,
    creep_rot: f64,
    post_release: usize,
    was_none: bool,
}

impl Recovery {
    pub(super) fn new() -> Self {
        Self {
            field_is_reloc: false,
            ticks: 0,
            gave_up: false,
            frozen: None,
            delta: None,
            creep_rot: 0.0,
            post_release: usize::MAX,
            was_none: false,
        }
    }

    pub(super) fn note_pose(&mut self, available: bool) {
        if !available {
            self.was_none = true;
        } else if self.was_none {
            self.was_none = false;
            self.post_release = 0;
        } else if self.post_release != usize::MAX {
            self.post_release = self.post_release.saturating_add(1);
        }
    }

    pub(super) fn is_active(&self, enabled: bool, limit: u32) -> bool {
        enabled && !self.gave_up && self.ticks < limit
    }

    pub(super) fn resume(&mut self) {
        self.ticks = 0;
        self.gave_up = false;
        self.frozen = None;
        self.delta = None;
        self.creep_rot = 0.0;
    }

    pub(super) fn verifying(&self, probation: bool, recovering: bool, detected: bool) -> bool {
        probation || (recovering && detected) || self.post_release < RELEASE_GUARD_TICKS
    }

    pub(super) fn reject_starvation(&self, probation: bool, recovering: bool, whole: bool) -> bool {
        probation || ((recovering || self.post_release < RELEASE_GUARD_TICKS) && whole)
    }

    pub(super) fn has_target(&self) -> bool {
        self.frozen.is_some()
    }

    /// Freeze one target relative to the current leading hypothesis. A target
    /// closer than one metre is excluded so arrival cannot refreeze the same point.
    pub(super) fn freeze_target(&mut self, hyps: &[(PoseView, f64)], targets: Vec<(f64, f64)>) {
        if targets.is_empty() {
            return;
        }
        if let Some(&(mp, _)) = hyps.first() {
            if let Some((tx, ty)) = targets.iter().copied()
                .filter(|&(tx, ty)| (tx - mp.x).hypot(ty - mp.y) > 1.0)
                .min_by(|a, b| {
                    (a.0 - mp.x).hypot(a.1 - mp.y)
                        .total_cmp(&(b.0 - mp.x).hypot(b.1 - mp.y))
                })
            {
                self.delta = Some((
                    (tx - mp.x).hypot(ty - mp.y),
                    wrap_rad((ty - mp.y).atan2(tx - mp.x) - mp.yaw_rad),
                ));
            }
        }
        self.frozen = Some(targets);
    }

    pub(super) fn preferred_heading(&self) -> Option<f64> {
        self.delta.map(|(_, b)| b)
    }

    pub(super) fn creep(&mut self, scan: Option<&LaserScan>, max_range: f64) -> (f64, f64) {
        let cmd = match scan {
            Some(s) => vi_lib::ctrl::lost_creep(s, self.preferred_heading(), max_range, self.creep_rot),
            None => (0.0, 0.0),
        };
        self.creep_rot = cmd.1;
        cmd
    }

    /// Advance a frozen target using commanded odometry, never reselected peaks.
    pub(super) fn advance_target(&mut self, moved: Option<(f64, f64)>, tick_s: f64) {
        if let (Some((d, b)), Some((v, w_deg))) = (&mut self.delta, moved) {
            let (bx, by) = (*d * b.cos() - v * tick_s, *d * b.sin());
            *b = wrap_rad(by.atan2(bx) - (w_deg.to_radians()) * tick_s);
            *d = bx.hypot(by);
            if *d < 0.5 {
                self.delta = None;
                self.frozen = None;
            }
        }
    }
}
