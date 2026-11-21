//! Shared pose input. Stamped external poses expire in both ROS and steady time.

use std::sync::Mutex;
use std::time::{Duration, Instant};

use rclrs::Clock;
use vi_lib::bridge::PoseView;
use vi_planner::core::lock;

#[derive(Clone, Copy)]
struct Stamp {
    nanos: i64,
    received: Instant,
}

pub struct PoseSlot {
    pose: Mutex<Option<(PoseView, Option<Stamp>)>>,
    max_age: Option<Duration>,
    clock: Clock,
}

impl PoseSlot {
    pub fn new(clock: Clock, max_age: Option<Duration>) -> Self {
        Self {
            pose: Mutex::new(None),
            max_age,
            clock,
        }
    }

    pub fn requires_fresh_stamp(&self) -> bool {
        self.max_age.is_some()
    }

    pub fn get(&self) -> Option<PoseView> {
        let (pose, stamp) = (*lock(&self.pose))?;
        match self.max_age {
            None => Some(pose),
            Some(max_age) => self.is_fresh(stamp?, max_age).then_some(pose),
        }
    }

    /// Localizer predictions and scans cannot renew the lifetime of an external sample.
    pub fn set(&self, pose: Option<PoseView>) {
        if !self.requires_fresh_stamp() {
            *lock(&self.pose) = pose.map(|p| (p, None));
        }
    }

    pub fn set_stamped(&self, pose: PoseView, sec: i32, nanosec: u32) -> bool {
        let Some(max_age) = self.max_age else {
            return false;
        };
        let stamp = Stamp {
            nanos: i64::from(sec) * 1_000_000_000 + i64::from(nanosec),
            received: Instant::now(),
        };
        if !self.is_fresh(stamp, max_age) {
            return false;
        }
        let mut current = lock(&self.pose);
        // A pose can change via odometry without a new map correction. Accept the
        // coordinates without renewing their freshness, including when /clock pauses.
        if let Some((_, Some(old))) = *current {
            if old.nanos == stamp.nanos {
                if !self.is_fresh(old, max_age) {
                    return false;
                }
                *current = Some((pose, Some(old)));
                return true;
            }
        }
        *current = Some((pose, Some(stamp)));
        true
    }

    fn is_fresh(&self, stamp: Stamp, max_age: Duration) -> bool {
        let Ok((sec, nanosec)) = self.clock.now().to_sec_nanosec() else {
            return false;
        };
        let now = i64::from(sec) * 1_000_000_000 + i64::from(nanosec);
        fresh(stamp.nanos, now, stamp.received.elapsed(), max_age)
    }
}

fn fresh(stamp: i64, now: i64, elapsed: Duration, max_age: Duration) -> bool {
    stamp > 0
        && elapsed < max_age
        && u64::try_from(now - stamp).is_ok_and(|age| Duration::from_nanos(age) < max_age)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_expires_on_either_clock_and_rejects_unset_or_future_stamps() {
        let timeout = Duration::from_millis(500);
        let stamp = 2_000_000_000;
        assert!(fresh(stamp, stamp + 200_000_000, Duration::ZERO, timeout));
        assert!(!fresh(stamp, stamp + 500_000_000, Duration::ZERO, timeout));
        assert!(!fresh(stamp, stamp, timeout, timeout));
        assert!(!fresh(stamp, stamp - 1, Duration::ZERO, timeout));
        assert!(!fresh(0, 0, Duration::ZERO, timeout));
    }
}
