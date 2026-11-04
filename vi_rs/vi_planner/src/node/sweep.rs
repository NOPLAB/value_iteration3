//! 制御周期から独立した狭域・広域 worker。計算は共有ロック外で並行する。

use std::io;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use vi_planner::core::{lock, PlannerCore};
use vi_planner::cpu::{CpuPacer, CpuShare};

use super::handles::Handles;
use super::msg::ros_grid_from;
use super::params::Params;

const STATES_PER_JOB: usize = 5_000;
const REPORT_EVERY: Duration = Duration::from_secs(2);
const IDLE: Duration = Duration::from_millis(20);

/// 追従終了時に狭域の仕事を広域へ返す。古い追従の終了で新しい担当を外さない。
pub struct LocalRegion<'a> {
    core: &'a Mutex<PlannerCore>,
    owner: u64,
}

impl<'a> LocalRegion<'a> {
    pub fn claim(core: &'a Mutex<PlannerCore>) -> Self {
        let owner = lock(core).claim_local();
        Self { core, owner }
    }
}

impl Drop for LocalRegion<'_> {
    fn drop(&mut self) {
        lock(self.core).release_local(self.owner);
    }
}

fn value_grid(core: &Mutex<PlannerCore>, threshold: u64) -> Option<vi_lib::msg::OccupancyGrid> {
    let (epoch, mut grid, height) = lock(core).value_grid_rows(0..1, threshold)?;
    grid.data.reserve(grid.width as usize * (height - 1));
    for row in 1..height {
        let (next_epoch, part, _) = lock(core).value_grid_rows(row..row + 1, threshold)?;
        if epoch != next_epoch {
            return None;
        }
        grid.data.extend(part.data);
    }
    grid.height = height as i32;
    Some(grid)
}

/// 各 worker が自分のジョブだけを管理する。相手の計算終了は待たない。
pub fn spawn(handles: Arc<Handles>, params: &Params) -> Result<()> {
    if params.follow {
        let h = Arc::clone(&handles);
        let share = params.local_cpu_share;
        std::thread::Builder::new()
            .name("vi-local".into())
            .spawn(move || {
                if let Err(error) = local_worker(h, share) {
                    eprintln!("ERROR: vi_planner: local worker stopped: {error}");
                }
            })
            .context("starting local worker")?;
    }
    if params.global_sweep || params.early_start {
        let share = params.global_cpu_share;
        std::thread::Builder::new()
            .name("vi-global".into())
            .spawn(move || {
                if let Err(error) = global_worker(handles, share) {
                    eprintln!("ERROR: vi_planner: global worker stopped: {error}");
                }
            })
            .context("starting global worker")?;
    }
    eprintln!(
        "vi_planner: worker CPU shares local={} global={} (one core = 1.0)",
        params.local_cpu_share.get(),
        params.global_cpu_share.get(),
    );
    Ok(())
}

fn local_worker(handles: Arc<Handles>, share: CpuShare) -> io::Result<()> {
    let mut pacer = CpuPacer::new(share)?;
    loop {
        let job = lock(&handles.core).prepare_local_job();
        if let Some(mut job) = job {
            job.run();
            let _delta = lock(&handles.core).commit_local_job(job);
            pacer.pace()?;
        } else {
            std::thread::sleep(IDLE);
            pacer.reset()?;
        }
    }
}

fn global_worker(handles: Arc<Handles>, share: CpuShare) -> io::Result<()> {
    let mut pacer = CpuPacer::new(share)?;
    let mut last_report = Instant::now();
    let mut last_draw = Instant::now();
    loop {
        let job = {
            let mut core = lock(&handles.core);
            core.restart_global_check();
            core.prepare_global_job(STATES_PER_JOB)
        };
        if let Some(mut job) = job {
            job.run();
            let (_, done) = lock(&handles.core).commit_global_job(job);
            if last_report.elapsed() >= REPORT_EVERY {
                last_report = Instant::now();
                let core = lock(&handles.core);
                let (work, unit) = core.sweep_work();
                eprintln!("vi_planner: global check ({work} {unit}, settled={done})");
            }
            if let Some(v) = handles.viz.as_ref() {
                let draw = if v.interval.is_zero() {
                    done
                } else {
                    last_draw.elapsed() >= v.interval
                };
                if draw {
                    last_draw = Instant::now();
                    if let Some(g) = value_grid(&handles.core, v.threshold_steps) {
                        let _ = v.vf_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
                    }
                }
            }
            pacer.pace()?;
        } else {
            std::thread::sleep(IDLE);
            pacer.reset()?;
        }
    }
}
