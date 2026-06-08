//! Worker thread that drives `Solver::run(Budget::Sweeps(1))` and answers
//! read requests from publisher / cmd_vel timers.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};

use crossbeam_channel::{unbounded, Receiver, Sender};
use ndarray::{s, Array2, Array3};
use vi_algorithm::{Budget, SolveStats, Solver, VIContext};
use vi_core::{ActionIdx, Value, MAX_VALUE, N_THETA, PENALTY_OBSTACLE};

pub struct FeedbackTick {
    pub sweep_count: u32,
    pub final_delta: u16,
}

pub struct DumpData {
    pub value: Array3<Value>,
    pub policy: Array3<i16>,
}

/// Build the full optimal-policy table; -1 where obstacle / goal / unreachable
/// (mirrors the legacy node's `optimal_action_ == NULL ? -1`).
pub fn compute_policy(ctx: &VIContext) -> Array3<i16> {
    let h = ctx.dims.map_y as usize;
    let w = ctx.dims.map_x as usize;
    let mut pol = Array3::<i16>::from_elem((h, w, N_THETA), -1);
    for iy in 0..h {
        for ix in 0..w {
            if ctx.penalty[[iy, ix]] == PENALTY_OBSTACLE {
                continue;
            }
            for it in 0..N_THETA {
                if ctx.goal_mask[[iy, ix, it]] || ctx.value[[iy, ix, it]] == MAX_VALUE {
                    continue;
                }
                pol[[iy, ix, it]] =
                    vi_algorithm::optimal_action_at(ctx, ix as i32, iy as i32, it) as i16;
            }
        }
    }
    pol
}

pub enum WorkerRequest {
    ValueSlice    { theta_idx: usize,                resp: Sender<Array2<Value>> },
    OptimalAction { ix: i32, iy: i32, it: usize,     resp: Sender<ActionIdx> },
}

pub struct SweepHandle {
    pub cancel: Arc<AtomicBool>,
    pub feedback_rx: Receiver<FeedbackTick>,
    pub request_tx: Sender<WorkerRequest>,
    pub join: JoinHandle<SolveStats>,
}

pub fn spawn_sweep(
    mut ctx: VIContext,
    solver: Box<dyn Solver>,
    cancel: Arc<AtomicBool>,
    dump_slot: Option<Arc<Mutex<Option<DumpData>>>>,
) -> SweepHandle {
    let (feedback_tx, feedback_rx) = unbounded::<FeedbackTick>();
    let (request_tx, request_rx) = unbounded::<WorkerRequest>();
    let cancel_inner = Arc::clone(&cancel);

    let join = thread::spawn(move || {
        let mut total: u32 = 0;
        let mut last_stats = SolveStats {
            iters_or_sweeps: 0,
            updates: 0,
            final_delta: vi_core::MAX_VALUE,
            converged: false,
            extra: None,
        };
        loop {
            // Drain reader requests.
            while let Ok(req) = request_rx.try_recv() {
                match req {
                    WorkerRequest::ValueSlice { theta_idx, resp } => {
                        let slice = ctx.value.slice(s![.., .., theta_idx]).to_owned();
                        let _ = resp.send(slice);
                    }
                    WorkerRequest::OptimalAction { ix, iy, it, resp } => {
                        let a = vi_algorithm::optimal_action_at(&ctx, ix, iy, it);
                        let _ = resp.send(a);
                    }
                }
            }
            if cancel_inner.load(Ordering::Relaxed) { break; }
            let stats = solver.run(&mut ctx, Budget::Sweeps(1));
            total = total.saturating_add(stats.iters_or_sweeps);
            let _ = feedback_tx.send(FeedbackTick {
                sweep_count: total,
                final_delta: stats.final_delta,
            });
            let done = stats.converged;
            last_stats = stats;
            if done { break; }
        }
        if let Some(slot) = dump_slot {
            let policy = compute_policy(&ctx);
            *slot.lock().unwrap() = Some(DumpData { value: ctx.value.clone(), policy });
        }
        last_stats
    });

    SweepHandle { cancel, feedback_rx, request_tx, join }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use crossbeam_channel::bounded;
    use vi_algorithm::context::MapDims;
    use vi_algorithm::Reference;
    use vi_fixtures::{generate_map, generate_transitions, MapType, TransitionMode};

    fn ctx_with_goal() -> VIContext {
        let m = generate_map(8, 8, MapType::Empty);
        let trans = generate_transitions(TransitionMode::Full { xy_resolution: 0.05 });
        VIContext {
            dims: MapDims { map_x: 8, map_y: 8 },
            value: m.value, penalty: m.penalty, goal_mask: m.goal_mask,
            transitions: trans.unpack(),
        }
    }

    #[test]
    fn converges_and_joins() {
        let ctx = ctx_with_goal();
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(ctx, Box::new(Reference { threshold: 0 }), cancel, None);
        let stats = h.join.join().expect("worker panicked");
        assert!(stats.converged, "small empty map must converge with Reference");
    }

    #[test]
    fn cancel_stops_worker() {
        // 64x64 empty map with goal at center. Reference on this size + N_THETA=60
        // takes well over 1s to converge with threshold=0; we cancel after 50ms
        // and assert the worker joins quickly. The point of this test is to
        // verify the cancel signal IS observed by the worker, not that the map
        // happens to be slow.
        use std::time::Instant;
        let m = generate_map(64, 64, MapType::Empty);
        let trans = generate_transitions(TransitionMode::Full { xy_resolution: 0.05 });
        let big = VIContext {
            dims: MapDims { map_x: 64, map_y: 64 },
            value: m.value, penalty: m.penalty, goal_mask: m.goal_mask,
            transitions: trans.unpack(),
        };
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(big, Box::new(Reference { threshold: 0 }), Arc::clone(&cancel), None);

        std::thread::sleep(Duration::from_millis(50));
        let start = Instant::now();
        cancel.store(true, Ordering::Relaxed);
        let stats = h.join.join().expect("worker panicked");
        let elapsed = start.elapsed();

        // The worker must observe the cancel and exit within one sweep boundary.
        // A single Reference sweep on 64x64x60 takes ~50-200ms on typical hardware;
        // 2 seconds is a generous upper bound that will fail only if the cancel is
        // not honored at all (or the machine is pathologically slow — in which case
        // the rest of the test suite would also be unusable).
        assert!(
            elapsed < Duration::from_secs(2),
            "worker must observe cancel and exit within 2s of cancel signal; took {:?}",
            elapsed
        );
        // We don't assert on `stats.converged` because whether convergence happens
        // BEFORE the cancel observation is a race the test doesn't need to win.
        // What matters is the worker doesn't keep running indefinitely.
        let _ = stats;
    }

    #[test]
    fn value_slice_request_returns_slice() {
        let ctx = ctx_with_goal();
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(ctx, Box::new(Reference { threshold: 0 }), cancel, None);
        let (tx, rx) = bounded::<Array2<Value>>(1);
        h.request_tx.send(WorkerRequest::ValueSlice { theta_idx: 0, resp: tx }).unwrap();
        let slice = rx.recv_timeout(Duration::from_secs(2)).expect("slice");
        assert_eq!(slice.shape(), &[8, 8]);
        h.join.join().expect("worker panicked");
    }

    #[test]
    fn optimal_action_request_returns_action_id() {
        let ctx = ctx_with_goal();
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(ctx, Box::new(Reference { threshold: 0 }), cancel, None);
        let (tx, rx) = bounded::<ActionIdx>(1);
        h.request_tx.send(WorkerRequest::OptimalAction { ix: 0, iy: 0, it: 0, resp: tx }).unwrap();
        let a = rx.recv_timeout(Duration::from_secs(2)).expect("action");
        assert!((a as usize) < vi_core::N_ACTIONS);
        h.join.join().expect("worker panicked");
    }

    #[test]
    fn dump_slot_is_filled_on_exit() {
        use std::sync::Mutex;
        let ctx = ctx_with_goal();
        let cancel = Arc::new(AtomicBool::new(false));
        let slot: Arc<Mutex<Option<DumpData>>> = Arc::new(Mutex::new(None));
        let h = spawn_sweep(ctx, Box::new(Reference { threshold: 0 }), cancel, Some(Arc::clone(&slot)));
        h.join.join().expect("worker panicked");
        let guard = slot.lock().unwrap();
        let dump = guard.as_ref().expect("dump slot must be filled");
        assert_eq!(dump.value.shape(), &[8, 8, vi_core::N_THETA]);
        assert_eq!(dump.policy.shape(), &[8, 8, vi_core::N_THETA]);
        for &v in dump.policy.iter() {
            assert!(v == -1 || (0..vi_core::N_ACTIONS as i16).contains(&v));
        }
    }
}
