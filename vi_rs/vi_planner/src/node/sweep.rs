//! 狭域の担当割り当てと、広域ジョブの配送・結果反映を行う manager。

use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, Instant};

use vi_planner::core::{lock, GlobalJob, PlannerCore};

use super::handles::Handles;
use super::msg::ros_grid_from;
use super::params::Params;

/// ジョブを処理してから CPU を譲るまでの目安。ロック保持時間ではない。
const BUDGET: Duration = Duration::from_millis(20);
const STATES_PER_JOB: usize = 5_000;
const REPORT_EVERY: Duration = Duration::from_secs(2);

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

/// manager は取り出し・反映だけをロック内で行い、Bellman 更新は worker に渡す。
/// 配送と返却は各 1 件。広域の計算を待つのは manager だけで、追従は待たない。
pub fn spawn(handles: Arc<Handles>, params: &Params) {
    if !params.global_sweep && !params.early_start {
        return;
    }
    let duty = params.global_sweep_duty.clamp(1, 100) as f64 / 100.0;
    std::thread::spawn(move || {
        std::thread::scope(|scope| {
            let (jobs, incoming) = mpsc::sync_channel::<GlobalJob>(1);
            let (finished, results) = mpsc::sync_channel::<GlobalJob>(1);
            scope.spawn(move || {
                while let Ok(mut job) = incoming.recv() {
                    job.run();
                    if finished.send(job).is_err() {
                        return;
                    }
                }
            });

            let mut sweep_start = None;
            let mut last_report = Instant::now();
            loop {
                let batch_start = Instant::now();
                let mut worked = false;
                let mut done = false;
                loop {
                    let job = lock(&handles.core).prepare_global_job(STATES_PER_JOB);
                    let Some(job) = job else { break };
                    sweep_start.get_or_insert_with(Instant::now);
                    worked = true;
                    if jobs.send(job).is_err() {
                        eprintln!("ERROR: vi_planner: global worker disconnected");
                        return;
                    }
                    let Ok(job) = results.recv() else {
                        eprintln!(
                            "ERROR: vi_planner: global worker stopped before returning a job"
                        );
                        return;
                    };
                    done = lock(&handles.core).commit_global_job(job).1;
                    if done || batch_start.elapsed() >= BUDGET {
                        break;
                    }
                }

                // 局所側が最後の仕事を終えた場合も収束を報告する。
                done |= !lock(&handles.core).is_dirty();
                let mut draw = false;
                if sweep_start.is_some() && (done || last_report.elapsed() >= REPORT_EVERY) {
                    last_report = Instant::now();
                    {
                        let c = lock(&handles.core);
                        if let Some(started) = sweep_start {
                            if done {
                                let (work, unit) = c.sweep_work();
                                eprintln!(
                                    "vi_planner: global propagation settled in {:.1}s ({work} {unit})",
                                    started.elapsed().as_secs_f64(),
                                );
                            } else if let Some((processed, queued)) = c.sweep_progress() {
                                let (_, unit) = c.sweep_work();
                                eprintln!(
                                    "vi_planner: global propagation running for {:.1}s \
                                     ({processed} {unit} done, {queued} queued)",
                                    started.elapsed().as_secs_f64(),
                                );
                            }
                        }
                        if let Some(v) = handles.viz.as_ref() {
                            if done || !v.interval.is_zero() {
                                draw = true;
                            }
                        }
                    }
                    if done {
                        sweep_start = None;
                    }
                }
                if draw {
                    if let Some(v) = handles.viz.as_ref() {
                        if let Some(g) = value_grid(&handles.core, v.threshold_steps) {
                            let _ = v.vf_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
                        }
                    }
                }

                let idle = if worked {
                    batch_start.elapsed().mul_f64((1.0 - duty) / duty)
                } else {
                    Duration::from_millis(200)
                };
                std::thread::sleep(idle);
            }
        });
    });
}
