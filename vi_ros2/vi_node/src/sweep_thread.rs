//! Worker thread that drives `Solver::run(Budget::Sweeps(1))` and answers
//! read requests from publisher / cmd_vel timers.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};

use crossbeam_channel::{unbounded, Receiver, Sender};
use ndarray::{s, Array2};
use vi_algorithm::{Budget, SolveStats, Solver, VIContext};
use vi_core::{ActionIdx, Value};

pub struct FeedbackTick {
    pub sweep_count: u32,
    pub final_delta: u16,
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
    use vi_core::{params::N_THETA, GoalSpec, make_goal_mask};
    use vi_fixtures::{generate_map, generate_transitions, MapType, TransitionMode};

    fn ctx_with_goal() -> VIContext {
        let m = generate_map(8, 8, MapType::Empty);
        let mut goal_mask = ndarray::Array3::from_elem((8usize, 8usize, N_THETA), false);
        let spec = GoalSpec {
            xy_resolution: 0.05, map_origin_x: 0.0, map_origin_y: 0.0,
            goal_x: 0.20, goal_y: 0.20, goal_theta_deg: 0.0,
            goal_radius_m: 0.10, goal_margin_theta_deg: 90.0,
        };
        let m_goal = make_goal_mask(8, 8, &spec);
        for ((iy, ix, it), &v) in m_goal.indexed_iter() {
            goal_mask[[iy, ix, it]] = v;
        }
        let mut value = m.value;
        for ((iy, ix, it), &g) in goal_mask.indexed_iter() {
            if g { value[[iy, ix, it]] = 0; }
        }
        let trans = generate_transitions(TransitionMode::Full { xy_resolution: 0.05 });
        VIContext {
            dims: MapDims { map_x: 8, map_y: 8 },
            value, penalty: m.penalty, goal_mask, transitions: trans.unpack(),
        }
    }

    #[test]
    fn converges_and_joins() {
        let ctx = ctx_with_goal();
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(ctx, Box::new(Reference { threshold: 0 }), cancel);
        let stats = h.join.join().expect("worker panicked");
        assert!(stats.converged, "small empty map must converge with Reference");
    }

    #[test]
    fn cancel_stops_worker() {
        // Use a context with no goal pinned so Reference would otherwise iterate
        // a long time.
        let ctx = generate_map(64, 64, MapType::Empty);
        let trans = generate_transitions(TransitionMode::Full { xy_resolution: 0.05 });
        let big = VIContext {
            dims: MapDims { map_x: 64, map_y: 64 },
            value: ctx.value, penalty: ctx.penalty, goal_mask: ctx.goal_mask,
            transitions: trans.unpack(),
        };
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(big, Box::new(Reference { threshold: 0 }), Arc::clone(&cancel));
        std::thread::sleep(Duration::from_millis(50));
        cancel.store(true, Ordering::Relaxed);
        let stats = h.join.join().expect("worker panicked");
        // Reference without a goal will not "converge"; cancel must have fired.
        assert!(!stats.converged);
    }

    #[test]
    fn value_slice_request_returns_slice() {
        let ctx = ctx_with_goal();
        let cancel = Arc::new(AtomicBool::new(false));
        let h = spawn_sweep(ctx, Box::new(Reference { threshold: 0 }), cancel);
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
        let h = spawn_sweep(ctx, Box::new(Reference { threshold: 0 }), cancel);
        let (tx, rx) = bounded::<ActionIdx>(1);
        h.request_tx.send(WorkerRequest::OptimalAction { ix: 0, iy: 0, it: 0, resp: tx }).unwrap();
        let a = rx.recv_timeout(Duration::from_secs(2)).expect("action");
        assert!((a as usize) < vi_core::N_ACTIONS);
        h.join.join().expect("worker panicked");
    }
}
