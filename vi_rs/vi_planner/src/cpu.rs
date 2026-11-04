//! Per-worker CPU allocation, measured against one core rather than lock wait time.

use std::io;
use std::time::{Duration, Instant};

use cpu_time::ThreadTime;

/// CPU seconds per wall second for one worker. Independent workers may sum above 1.
#[derive(Clone, Copy, Debug)]
pub struct CpuShare(f64);

impl CpuShare {
    pub fn new(value: f64) -> Option<Self> {
        (value.is_finite() && value > 0.0 && value <= 1.0).then_some(Self(value))
    }

    pub fn get(self) -> f64 {
        self.0
    }

    fn idle(self, cpu: Duration, wall: Duration) -> Duration {
        cpu.div_f64(self.0).saturating_sub(wall)
    }
}

/// Pace bounded jobs after about 10 ms of actual CPU work.
/// Waiting for a mutex or being descheduled does not consume the CPU allocation.
pub struct CpuPacer {
    share: CpuShare,
    wall: Instant,
    cpu: ThreadTime,
}

impl CpuPacer {
    pub fn new(share: CpuShare) -> io::Result<Self> {
        Ok(Self {
            share,
            wall: Instant::now(),
            cpu: ThreadTime::try_now()?,
        })
    }

    pub fn pace(&mut self) -> io::Result<()> {
        let cpu = self.cpu.try_elapsed()?;
        if cpu >= Duration::from_millis(10) {
            let idle = self.share.idle(cpu, self.wall.elapsed());
            if !idle.is_zero() {
                std::thread::sleep(idle);
            }
            self.reset()?;
        }
        Ok(())
    }

    /// No goal or local owner: do not accumulate credits to spend in a later burst.
    pub fn reset(&mut self) -> io::Result<()> {
        self.cpu = ThreadTime::try_now()?;
        self.wall = Instant::now();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shares_reject_values_outside_one_core() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -0.1, 0.0, 1.01] {
            assert!(CpuShare::new(value).is_none(), "{value}");
        }
        for value in [0.01, 0.2, 0.6, 1.0] {
            assert!(CpuShare::new(value).is_some(), "{value}");
        }
    }

    #[test]
    fn allocation_counts_cpu_time_and_subtracts_waits() {
        for (share, cpu_ms, wall_ms, expected_ms) in [
            (0.6, 60, 60, 40),
            (0.2, 20, 20, 80),
            (0.2, 20, 70, 30),
            (0.6, 60, 120, 0),
            (1.0, 10, 10, 0),
        ] {
            let idle = CpuShare::new(share).unwrap().idle(
                Duration::from_millis(cpu_ms),
                Duration::from_millis(wall_ms),
            );
            assert_eq!(idle, Duration::from_millis(expected_ms));
        }
    }
}
