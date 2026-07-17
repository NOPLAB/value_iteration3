//! Worker thread that drives the u64 (本家忠実) solver on a `ValueIterator` and
//! answers read requests from the publisher / cmd_vel timers.
//!
//! The solver runs in bounded chunks (`solve(.., CHUNK)`) so the worker can
//! interleave reader requests, cancellation, and per-chunk feedback — mirroring
//! the legacy u16 worker's per-sweep loop. For frontier-family solvers each
//! chunk re-seeds the frontier from the current reached set; by VI monotonicity
//! this only adds redundant interior passes (no effect on the fixed point), so
//! the converged result stays bit-exact with a single monolithic solve.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};

use crossbeam_channel::{unbounded, Receiver, Sender};
use ndarray::{Array2, Array3};

use vi_reference::params::PROB_BASE;
use vi_reference::planner::optimal_action_at;
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::ValueIterator;

/// Per-tick solver budget (sweeps for Reference, iterations for frontier). The
/// worker re-enters `solve` each tick to answer reads, observe cancel, and emit
/// feedback. Smaller = finer feedback / faster cancel; larger = less frontier
/// re-seed overhead.
const CHUNK: u32 = 4;

/// Terminal stats reported by the worker on join.
#[derive(Clone, Copy, Debug)]
pub struct WorkerStats {
    pub sweeps: u32,
    pub updates: u64,
    pub converged: bool,
}

pub struct FeedbackTick {
    pub sweep_count: u32,
    /// Always 0 for u64 solvers — convergence is signalled by the worker
    /// breaking its loop (and dropping the feedback sender), NOT by a delta.
    pub final_delta: u16,
}

/// Final value/policy snapshot for the benchmark dump. Both are `f64` so the
/// vi_compare pipeline reads the `ros2` side like the other u64 sides:
/// value = `total_cost / PROB_BASE` (本家 valueFunctionWriter int-division),
/// policy = action id (-1 where none).
pub struct DumpData {
    pub value: Array3<f64>,
    pub policy: Array3<f64>,
}

/// Build the full optimal-policy table as action ids; -1 where obstacle / goal /
/// unreachable (mirrors the legacy node's `optimal_action_ == NULL ? -1`).
pub fn compute_policy(vi: &ValueIterator) -> Array3<f64> {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let mut pol = Array3::<f64>::from_elem((ny as usize, nx as usize, nt as usize), -1.0);
    for iy in 0..ny {
        for ix in 0..nx {
            for it in 0..nt {
                let s = &vi.states[vi.to_index(ix, iy, it) as usize];
                if !s.free || s.final_state {
                    continue;
                }
                if let Some(ai) = s.optimal_action {
                    pol[[iy as usize, ix as usize, it as usize]] = vi.actions[ai].id as f64;
                }
            }
        }
    }
    pol
}

/// Value/policy snapshot for the bench dump (both `f64`, see [`DumpData`]).
fn dump_from(vi: &ValueIterator) -> DumpData {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let mut value = Array3::<f64>::zeros((ny as usize, nx as usize, nt as usize));
    for iy in 0..ny {
        for ix in 0..nx {
            for it in 0..nt {
                let s = &vi.states[vi.to_index(ix, iy, it) as usize];
                value[[iy as usize, ix as usize, it as usize]] = (s.total_cost / PROB_BASE) as f64;
            }
        }
    }
    DumpData { value, policy: compute_policy(vi) }
}

pub enum WorkerRequest {
    /// total_cost slice at a fixed theta, shape `[cell_num_y, cell_num_x]`.
    ValueSlice { theta_idx: usize, resp: Sender<Array2<u64>> },
    /// optimal action id at `(ix, iy, it)`, or -1 if none / obstacle / goal.
    OptimalAction { ix: i32, iy: i32, it: usize, resp: Sender<i32> },
}

pub struct SweepHandle {
    pub cancel: Arc<AtomicBool>,
    pub feedback_rx: Receiver<FeedbackTick>,
    pub request_tx: Sender<WorkerRequest>,
    pub join: JoinHandle<WorkerStats>,
}

/// total_cost slice at `theta_idx`.
fn value_slice(vi: &ValueIterator, theta_idx: usize) -> Array2<u64> {
    let (nx, ny) = (vi.cell_num_x, vi.cell_num_y);
    let mut out = Array2::<u64>::zeros((ny as usize, nx as usize));
    for iy in 0..ny {
        for ix in 0..nx {
            out[[iy as usize, ix as usize]] =
                vi.states[vi.to_index(ix, iy, theta_idx as i32) as usize].total_cost;
        }
    }
    out
}

/// Spawn the sweep worker. `max_budget` caps total solver units (sweeps /
/// iterations) as a safety bound; the worker runs to convergence or until
/// cancelled within that cap.
pub fn spawn_sweep(
    mut vi: ValueIterator,
    solver: U64Solver,
    max_budget: u32,
    cancel: Arc<AtomicBool>,
    dump_slot: Option<Arc<Mutex<Option<DumpData>>>>,
) -> SweepHandle {
    let (feedback_tx, feedback_rx) = unbounded::<FeedbackTick>();
    let (request_tx, request_rx) = unbounded::<WorkerRequest>();
    let cancel_inner = Arc::clone(&cancel);

    let join = thread::spawn(move || {
        let mut total_sweeps: u32 = 0;
        let mut total_updates: u64 = 0;
        let mut converged = false;
        let mut remaining = max_budget;

        loop {
            // Drain reader requests.
            while let Ok(req) = request_rx.try_recv() {
                match req {
                    WorkerRequest::ValueSlice { theta_idx, resp } => {
                        let _ = resp.send(value_slice(&vi, theta_idx));
                    }
                    WorkerRequest::OptimalAction { ix, iy, it, resp } => {
                        let _ = resp.send(optimal_action_at(&vi, ix, iy, it as i32));
                    }
                }
            }
            if cancel_inner.load(Ordering::Relaxed) {
                break;
            }
            if remaining == 0 {
                break;
            }
            let chunk = remaining.min(CHUNK);
            let stats = solve(&mut vi, solver, chunk);
            total_sweeps = total_sweeps.saturating_add(stats.iters);
            total_updates = total_updates.saturating_add(stats.updates);
            remaining = remaining.saturating_sub(chunk);
            let _ = feedback_tx.send(FeedbackTick { sweep_count: total_sweeps, final_delta: 0 });
            if stats.converged {
                converged = true;
                break;
            }
        }

        if let Some(slot) = dump_slot {
            *slot.lock().unwrap() = Some(dump_from(&vi));
        }
        WorkerStats { sweeps: total_sweeps, updates: total_updates, converged }
    });

    SweepHandle { cancel, feedback_rx, request_tx, join }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    use crossbeam_channel::bounded;
    use vi_reference::{Action, OccupancyGrid, Quaternion};

    const BIG_BUDGET: u32 = 1_000_000;

    fn actions() -> Vec<Action> {
        vec![
            Action::new("forward", 0.3, 0.0, 0),
            Action::new("back", -0.2, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2),
            Action::new("rightfw", 0.2, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4),
            Action::new("leftfw", 0.2, 20.0, 5),
        ]
    }

    fn vi_empty(size: i32) -> ValueIterator {
        let grid = OccupancyGrid {
            width: size,
            height: size,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        };
        let mut vi = ValueIterator::new(actions(), 1);
        vi.set_map_with_occupancy_grid(&grid, 60, 0.2, 30.0, 0.1, 15);
        let g = size as f64 * 0.05 / 2.0;
        vi.set_goal(g, g, 0);
        vi
    }

    #[test]
    fn converges_and_joins() {
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(vi_empty(8), U64Solver::Reference, BIG_BUDGET, cancel, None);
        let stats = h.join.join().expect("worker panicked");
        assert!(stats.converged, "small empty map must converge with Reference");
    }

    #[test]
    fn frontier_converges_and_joins() {
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(vi_empty(8), U64Solver::Frontier3D, BIG_BUDGET, cancel, None);
        let stats = h.join.join().expect("worker panicked");
        assert!(stats.converged, "frontier3d must converge and signal via flag");
    }

    #[test]
    fn cancel_stops_worker() {
        use std::time::Instant;
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(vi_empty(64), U64Solver::Reference, BIG_BUDGET, Arc::clone(&cancel), None);
        std::thread::sleep(Duration::from_millis(50));
        let start = Instant::now();
        cancel.store(true, Ordering::Relaxed);
        let _stats = h.join.join().expect("worker panicked");
        assert!(
            start.elapsed() < Duration::from_secs(3),
            "worker must observe cancel and exit within a few chunk boundaries"
        );
    }

    #[test]
    fn value_slice_request_returns_slice() {
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(vi_empty(8), U64Solver::Reference, BIG_BUDGET, cancel, None);
        let (tx, rx) = bounded::<Array2<u64>>(1);
        h.request_tx.send(WorkerRequest::ValueSlice { theta_idx: 0, resp: tx }).unwrap();
        let slice = rx.recv_timeout(Duration::from_secs(2)).expect("slice");
        assert_eq!(slice.shape(), &[8, 8]);
        h.join.join().expect("worker panicked");
    }

    #[test]
    fn optimal_action_request_returns_action_id() {
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(vi_empty(8), U64Solver::Reference, BIG_BUDGET, cancel, None);
        let (tx, rx) = bounded::<i32>(1);
        h.request_tx
            .send(WorkerRequest::OptimalAction { ix: 0, iy: 0, it: 0, resp: tx })
            .unwrap();
        let a = rx.recv_timeout(Duration::from_secs(2)).expect("action");
        assert!(a == -1 || (0..6).contains(&a));
        h.join.join().expect("worker panicked");
    }

    #[test]
    fn dump_slot_is_filled_on_exit() {
        let cancel = Arc::new(AtomicBool::new(false));
        let slot: Arc<Mutex<Option<DumpData>>> = Arc::new(Mutex::new(None));
        let h = spawn_sweep(
            vi_empty(8),
            U64Solver::Reference,
            BIG_BUDGET,
            cancel,
            Some(Arc::clone(&slot)),
        );
        h.join.join().expect("worker panicked");
        let guard = slot.lock().unwrap();
        let dump = guard.as_ref().expect("dump slot must be filled");
        assert_eq!(dump.value.shape(), &[8, 8, 60]);
        assert_eq!(dump.policy.shape(), &[8, 8, 60]);
        for &v in dump.policy.iter() {
            assert!(v == -1.0 || (0.0..6.0).contains(&v));
        }
    }
}
