//! Per-worker CPU thread-equivalent allocation, excluding lock wait time.

use std::io;
use std::time::{Duration, Instant};

use cpu_time::ThreadTime;

/// CPU seconds per wall second for one single-threaded worker, in (0, 1].
/// Independent workers may sum above 1.
#[derive(Clone, Copy, Debug)]
pub struct CpuShare(f64);

impl CpuShare {
    pub fn new(value: f64) -> Option<Self> {
        (value.is_finite() && value > 0.0 && value <= 1.0).then_some(Self(value))
    }

    pub fn get(self) -> f64 {
        self.0
    }
}

/// Total local CPU thread-equivalent budget: 4.0 means four threads, 0.6 means
/// 60% of one thread, independent of the host's available parallelism.
#[derive(Clone, Copy, Debug)]
pub struct LocalCpuShare(f64);

impl LocalCpuShare {
    pub fn new(value: f64) -> Option<Self> {
        (value.is_finite() && value > 0.0).then_some(Self(value))
    }

    pub fn get(self) -> f64 {
        self.0
    }

    pub fn threads(self) -> usize {
        // Float-to-integer saturation is intentional for budgets beyond usize::MAX.
        self.0.ceil() as usize
    }

    pub fn pacer(self) -> io::Result<CpuPacer> {
        CpuPacer::with_budget(self)
    }

    fn idle(self, cpu: Duration, wall: Duration) -> Duration {
        cpu.div_f64(self.0).saturating_sub(wall)
    }
}

/// Pace bounded jobs after about 10 ms of actual CPU work.
/// Waiting for a mutex or being descheduled does not consume the CPU allocation.
pub struct CpuPacer {
    share: LocalCpuShare,
    wall: Instant,
    cpu: ThreadTime,
    external_cpu: Duration,
}

impl CpuPacer {
    pub fn new(share: CpuShare) -> io::Result<Self> {
        Self::with_budget(LocalCpuShare(share.get()))
    }

    fn with_budget(share: LocalCpuShare) -> io::Result<Self> {
        Ok(Self {
            share,
            wall: Instant::now(),
            cpu: ThreadTime::try_now()?,
            external_cpu: Duration::ZERO,
        })
    }

    pub fn pace(&mut self) -> io::Result<()> {
        self.pace_with_cpu(Duration::ZERO)
    }

    /// Include CPU used by joined helper threads; lock waits remain excluded.
    pub fn pace_with_cpu(&mut self, external: Duration) -> io::Result<()> {
        self.external_cpu += external;
        let cpu = self.cpu.try_elapsed()? + self.external_cpu;
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
        self.external_cpu = Duration::ZERO;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shares_reject_values_outside_one_thread() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -0.1, 0.0, 1.01] {
            assert!(CpuShare::new(value).is_none(), "{value}");
        }
        for value in [0.01, 0.2, 0.6, 1.0] {
            assert!(CpuShare::new(value).is_some(), "{value}");
        }
    }

    #[test]
    fn local_budgets_request_threads_independently_of_host_parallelism() {
        let available = std::thread::available_parallelism().map_or(1, usize::from);
        let above_host = available + 1;
        for (value, threads) in [
            (0.6, 1),
            (1.0, 1),
            (1.5, 2),
            (4.0, 4),
            (above_host as f64, above_host),
        ] {
            let share = LocalCpuShare::new(value).unwrap();
            assert_eq!(share.get(), value);
            assert_eq!(share.threads(), threads);
        }
        for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(LocalCpuShare::new(value).is_none());
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
            (4.0, 40, 10, 0),
            (2.0, 40, 10, 10),
        ] {
            let idle = LocalCpuShare::new(share).unwrap().idle(
                Duration::from_millis(cpu_ms),
                Duration::from_millis(wall_ms),
            );
            assert_eq!(idle, Duration::from_millis(expected_ms));
        }
    }
}
