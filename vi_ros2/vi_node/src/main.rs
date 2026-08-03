//! vi_node entry point.
//!
//! Boot order (see spec §4.1):
//!   1. `Context::default_from_env` + basic executor + node creation
//!   2. Parameters declared and validated (fail-fast on mismatch)
//!   3. Rayon thread-pool init (parallel feature only)
//!   4. /map received (transient_local, blocks until first message)
//!   5. OccupancyGrid + actions captured for per-goal ValueIterator builds
//!   6. Action server, publishers, timers wired
//!   7. executor.spin()
//!
//! u64 (本家忠実) port: the solver runs on a `vi_reference::ValueIterator` whose
//! penalty field and goal mask are computed inside the iterator (18-bit fixed
//! point). A fresh `ValueIterator` is built per action goal from the captured
//! `OccupancyGrid` + actions, then `set_goal` pins the goal cells.
//!
//! NOTE: This file uses the rclrs API as found on ros2-rust/ros2_rust @ commit
//! 2c6b926 (rclrs 0.7.0), which is what the Docker image builds. The
//! action-server callback returns a future that rclrs polls on its own
//! `futures`-based executor — there is NO tokio runtime, so the feedback pump /
//! worker-join are run on a dedicated std thread and bridged back to the async
//! callback via a oneshot channel.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{anyhow, Context as ACtx, Result};

use vi_reference::{Action, OccupancyGrid, ValueIterator};

use vi_node::bridge::{
    occupancy_view_to_vi_grid, value_slice_to_occupancy, yaw_to_goal_theta_deg, OccupancyGridView,
};
use vi_node::solver_factory::make_solver;
use vi_node::sweep_thread::{spawn_sweep, SweepHandle, WorkerRequest};

// rclrs API — matches upstream main-branch executor/node pattern.
use rclrs::*;

/// Safety cap on total solver units (sweeps / iterations). The worker runs to
/// convergence or cancellation well before this; it only guards a never-
/// converging solver from spinning forever.
const MAX_SOLVER_BUDGET: u32 = u32::MAX;

// ──────────────────────────────────────────────────────────────────────────────
// Parameter struct
// ──────────────────────────────────────────────────────────────────────────────

/// All declared ROS parameters collected into one struct for convenience.
struct Params {
    solver: String,
    theta_cell_num: i64,
    safety_radius: f64,
    safety_radius_penalty: i64,
    goal_margin_radius: f64,
    goal_margin_theta_deg: f64,
    online: bool,
    cost_drawing_threshold: i64,
    // Declared for ROS-interface parity with the legacy node; not yet threaded
    // into the solver factory (which currently fixes the convergence threshold
    // at 0). Kept so the parameter is still declared/validated on the node.
    #[allow(dead_code)]
    delta_threshold: i64,
    thread_num: i64,
    map_wait_sec: i64,
    // Benchmark hook: when non-empty, the action callback dumps the full
    // value/policy arrays as `.npy` into this directory after the solve
    // finishes. Empty (default) = off = unchanged production behavior.
    bench_dump_path: String,
    // Flattened from three parallel arrays (names, fw, rot).
    action_list: Vec<(String, f64, f64)>,
}

/// Declare all node parameters and collect their values.
fn read_params(node: &Node) -> Result<Params> {
    let solver = node
        .declare_parameter::<Arc<str>>("solver")
        .default("frontier3d".into())
        .mandatory()
        .map_err(|e| anyhow!("declare solver: {e}"))?
        .get()
        .to_string();

    let theta_cell_num = node
        .declare_parameter::<i64>("theta_cell_num")
        .default(60)
        .mandatory()
        .map_err(|e| anyhow!("declare theta_cell_num: {e}"))?
        .get();

    let safety_radius = node
        .declare_parameter::<f64>("safety_radius")
        .default(0.2)
        .mandatory()
        .map_err(|e| anyhow!("declare safety_radius: {e}"))?
        .get();

    let safety_radius_penalty = node
        .declare_parameter::<i64>("safety_radius_penalty")
        .default(30)
        .mandatory()
        .map_err(|e| anyhow!("declare safety_radius_penalty: {e}"))?
        .get();

    let goal_margin_radius = node
        .declare_parameter::<f64>("goal_margin_radius")
        .default(0.3)
        .mandatory()
        .map_err(|e| anyhow!("declare goal_margin_radius: {e}"))?
        .get();

    let goal_margin_theta_deg = node
        .declare_parameter::<f64>("goal_margin_theta")
        .default(15.0)
        .mandatory()
        .map_err(|e| anyhow!("declare goal_margin_theta: {e}"))?
        .get();

    let online = node
        .declare_parameter::<bool>("online")
        .default(false)
        .mandatory()
        .map_err(|e| anyhow!("declare online: {e}"))?
        .get();

    let cost_drawing_threshold = node
        .declare_parameter::<i64>("cost_drawing_threshold")
        .default(60)
        .mandatory()
        .map_err(|e| anyhow!("declare cost_drawing_threshold: {e}"))?
        .get();

    let delta_threshold = node
        .declare_parameter::<i64>("delta_threshold")
        .default(0)
        .mandatory()
        .map_err(|e| anyhow!("declare delta_threshold: {e}"))?
        .get();

    let thread_num = node
        .declare_parameter::<i64>("thread_num")
        .default(0)
        .mandatory()
        .map_err(|e| anyhow!("declare thread_num: {e}"))?
        .get();

    let map_wait_sec = node
        .declare_parameter::<i64>("map_wait_sec")
        .default(30)
        .mandatory()
        .map_err(|e| anyhow!("declare map_wait_sec: {e}"))?
        .get();

    let bench_dump_path = node
        .declare_parameter::<Arc<str>>("bench_dump_path")
        .default("".into())
        .mandatory()
        .map_err(|e| anyhow!("declare bench_dump_path: {e}"))?
        .get()
        .to_string();

    let names: Vec<String> = node
        .declare_parameter::<Arc<[Arc<str>]>>("action_names")
        .default_string_array([
            "forward", "back", "right", "rightfw", "left", "leftfw",
        ])
        .mandatory()
        .map_err(|e| anyhow!("declare action_names: {e}"))?
        .get()
        .iter()
        .map(|s| s.to_string())
        .collect();

    let fws: Vec<f64> = node
        .declare_parameter::<Arc<[f64]>>("action_forward_m")
        .default_from_iter([0.3, -0.2, 0.0, 0.2, 0.0, 0.2])
        .mandatory()
        .map_err(|e| anyhow!("declare action_forward_m: {e}"))?
        .get()
        .to_vec();

    let rots: Vec<f64> = node
        .declare_parameter::<Arc<[f64]>>("action_rotation_deg")
        .default_from_iter([0.0, 0.0, -20.0, -20.0, 20.0, 20.0])
        .mandatory()
        .map_err(|e| anyhow!("declare action_rotation_deg: {e}"))?
        .get()
        .to_vec();

    if names.len() != fws.len() || fws.len() != rots.len() {
        return Err(anyhow!(
            "action_names/action_forward_m/action_rotation_deg length mismatch: \
             names={}, fws={}, rots={}",
            names.len(),
            fws.len(),
            rots.len()
        ));
    }

    let action_list: Vec<(String, f64, f64)> = names
        .into_iter()
        .zip(fws)
        .zip(rots)
        .map(|((n, f), r)| (n, f, r))
        .collect();

    Ok(Params {
        solver,
        theta_cell_num,
        safety_radius,
        safety_radius_penalty,
        goal_margin_radius,
        goal_margin_theta_deg,
        online,
        cost_drawing_threshold,
        delta_threshold,
        thread_num,
        map_wait_sec,
        bench_dump_path,
        action_list,
    })
}

// ──────────────────────────────────────────────────────────────────────────────
// Parameter validation
// ──────────────────────────────────────────────────────────────────────────────

/// Validate that the parameters are self-consistent (fail-fast).
///
/// `ValueIterator` takes the action set and the theta count at run time, so the
/// values themselves are the launch file's business — this used to compare them
/// against vi_core's compile-time constants, which only got in the way of
/// changing them. What is left are the constraints the code downstream actually
/// imposes.
fn validate(p: &Params) -> Result<()> {
    // 本家 `t_resolution_ = 360/cell_num_t_` は整数除算。割り切れないと
    // `it = (t % 360) / t_resolution` が cell_num_t 以上を返し、states の範囲外を指す。
    if p.theta_cell_num <= 0 || p.theta_cell_num > 360 || 360 % p.theta_cell_num != 0 {
        return Err(anyhow!(
            "theta_cell_num must divide 360 (t_resolution = 360/theta_cell_num is an \
             integer division), got {}",
            p.theta_cell_num
        ));
    }
    if p.action_list.is_empty() {
        return Err(anyhow!("action_names/action_forward_m/action_rotation_deg are empty"));
    }
    // Verify solver string is known; make_solver is pure-Rust and checkable here.
    make_solver(&p.solver)?;
    Ok(())
}

/// Build the `vi_reference::Action` list from the validated action params.
fn build_actions(action_list: &[(String, f64, f64)]) -> Vec<Action> {
    action_list
        .iter()
        .enumerate()
        .map(|(i, (name, fw, rot))| Action::new(name, *fw, *rot, i as i32))
        .collect()
}

// ──────────────────────────────────────────────────────────────────────────────
// Rayon init (parallel feature only)
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(feature = "parallel")]
fn init_rayon(thread_num: i64) {
    if thread_num > 0 {
        let _ = rayon::ThreadPoolBuilder::new()
            .num_threads(thread_num as usize)
            .build_global();
    }
}

#[cfg(not(feature = "parallel"))]
fn init_rayon(thread_num: i64) {
    if thread_num > 0 {
        eprintln!("WARN: thread_num={thread_num} ignored (built without --features parallel)");
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// GridMeta helper — small owned copy of grid geometry for timers / publishers
// ──────────────────────────────────────────────────────────────────────────────

/// Owned copy of map geometry. Used by publishers and cmd_vel timer.
#[derive(Clone)]
pub(crate) struct GridMeta {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
}

impl GridMeta {
    pub(crate) fn to_map_meta_data(&self) -> nav_msgs::msg::MapMetaData {
        let mut m = nav_msgs::msg::MapMetaData::default();
        m.resolution = self.resolution as f32;
        m.width = self.width;
        m.height = self.height;
        m.origin.position.x = self.origin_x;
        m.origin.position.y = self.origin_y;
        m.origin.orientation.w = 1.0;
        m
    }
}

/// Per-goal ValueIterator build inputs, captured once from `/map`.
#[derive(Clone)]
struct MapBuild {
    grid: OccupancyGrid,
    actions: Vec<Action>,
    theta_cell_num: i32,
    safety_radius: f64,
    safety_radius_penalty: f64,
    goal_margin_radius: f64,
    goal_margin_theta: i32,
}

impl MapBuild {
    /// Build a fresh ValueIterator with the map applied (no goal yet).
    fn build(&self) -> ValueIterator {
        let mut vi = ValueIterator::new(self.actions.clone(), 1);
        vi.set_map_with_occupancy_grid(
            &self.grid,
            self.theta_cell_num,
            self.safety_radius,
            self.safety_radius_penalty,
            self.goal_margin_radius,
            self.goal_margin_theta,
        );
        vi
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Quaternion → yaw helper
// ──────────────────────────────────────────────────────────────────────────────

/// Extract yaw (Z-rotation in radians) from a ROS quaternion. Standard formula.
fn yaw_from_quat(q: &geometry_msgs::msg::Quaternion) -> f64 {
    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    siny_cosp.atan2(cosy_cosp)
}

// ──────────────────────────────────────────────────────────────────────────────
// main
// ──────────────────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    // 1. ROS context + executor + node.
    let context = Context::default_from_env().context("rclrs context init")?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("vi_node").context("create vi_node")?;

    // 2. Parameters.
    let params = read_params(&node).context("reading parameters")?;
    validate(&params).context("validating parameters")?;

    // 3. Rayon thread-pool (vestigial for u64 serial solvers; kept for parity).
    init_rayon(params.thread_num);

    // 4. Wait for /map (transient_local, blocks until first message).
    let map_msg = wait_for_map(&node, &mut executor, params.map_wait_sec)
        .context("waiting for /map")?;

    // 5. Capture OccupancyGrid + actions for per-goal ValueIterator builds.
    let grid_meta = GridMeta {
        width: map_msg.info.width,
        height: map_msg.info.height,
        resolution: map_msg.info.resolution as f64,
        origin_x: map_msg.info.origin.position.x,
        origin_y: map_msg.info.origin.position.y,
    };
    let grid_view = OccupancyGridView {
        width: grid_meta.width,
        height: grid_meta.height,
        resolution: grid_meta.resolution,
        origin_x: grid_meta.origin_x,
        origin_y: grid_meta.origin_y,
        data: &map_msg.data[..],
    };
    // unknown -> obstacle (conservative; matches the legacy node default).
    let vi_grid = occupancy_view_to_vi_grid(&grid_view, true);

    let map_build = MapBuild {
        grid: vi_grid,
        actions: build_actions(&params.action_list),
        theta_cell_num: params.theta_cell_num as i32,
        safety_radius: params.safety_radius,
        safety_radius_penalty: params.safety_radius_penalty as f64,
        goal_margin_radius: params.goal_margin_radius,
        goal_margin_theta: params.goal_margin_theta_deg as i32,
    };

    // Shared sweep handle — replaced on every new action goal.
    let sweep_handle: Arc<Mutex<Option<SweepHandle>>> = Arc::new(Mutex::new(None));

    // 6. Wire action server + publishers + timers.
    let _action_server = spawn_action_server(&node, &params, &sweep_handle, &map_build)?;

    let _vf_timer = spawn_value_function_publisher(
        &node,
        &sweep_handle,
        &grid_meta,
        params.cost_drawing_threshold.max(0) as u64,
    )?;
    let _cmd_vel_timer = if params.online {
        Some(spawn_cmd_vel_timer(&node, &sweep_handle, &grid_meta, &params.action_list)?)
    } else {
        None
    };

    // 7. Spin (blocks until shutdown).
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

// ──────────────────────────────────────────────────────────────────────────────
// wait_for_map — transient_local subscriber, blocks until first message
// ──────────────────────────────────────────────────────────────────────────────

fn wait_for_map(
    node: &Node,
    executor: &mut Executor,
    wait_sec: i64,
) -> Result<nav_msgs::msg::OccupancyGrid> {
    use std::sync::mpsc::sync_channel;

    let (tx, rx) = sync_channel::<nav_msgs::msg::OccupancyGrid>(1);
    let tx_c = tx.clone();

    let _sub = node.create_subscription::<nav_msgs::msg::OccupancyGrid, _>(
        "map".transient_local().reliable().keep_last(1),
        move |msg: nav_msgs::msg::OccupancyGrid| {
            let _ = tx_c.try_send(msg);
        },
    )?;

    let deadline = std::time::Instant::now() + Duration::from_secs(wait_sec as u64);
    loop {
        if let Ok(msg) = rx.try_recv() {
            return Ok(msg);
        }
        if std::time::Instant::now() > deadline {
            return Err(anyhow!("map not received within {} seconds", wait_sec));
        }
        executor.spin(SpinOptions::default().timeout(Duration::from_millis(100)));
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// spawn_action_server — vi_controller Vi.action server
// ──────────────────────────────────────────────────────────────────────────────

/// Wire the `vi_controller` action server (spec §4.2).
///
/// The callback receives a `RequestedGoal<Vi>`, accepts it, then:
///   1. Cancel any in-flight sweep.
///   2. Build a fresh ValueIterator from the captured map + set_goal.
///   3. Spawn new sweep worker.
///   4. Pump FeedbackTick at 10 Hz, publish Vi_Feedback.
///   5. Join worker → publish Vi_Result (`finished = converged`).
fn spawn_action_server(
    node: &Node,
    params: &Params,
    sweep_handle: &Arc<Mutex<Option<SweepHandle>>>,
    map_build: &MapBuild,
) -> Result<ActionServer<vi_interfaces::action::Vi>> {
    let sweep_handle = Arc::clone(sweep_handle);
    let map_build = map_build.clone();
    let solver_name = params.solver.clone();
    let bench_dump_path = params.bench_dump_path.clone();

    let server = node.create_action_server::<vi_interfaces::action::Vi, _>(
        "vi_controller",
        move |requested_goal: RequestedGoal<vi_interfaces::action::Vi>| {
            let sweep_handle = Arc::clone(&sweep_handle);
            let map_build = map_build.clone();
            let solver_name = solver_name.clone();
            let bench_dump_path = bench_dump_path.clone();

            async move {
                // ── Step 1: cancel any prior in-flight sweep ──────────────────
                {
                    let old_handle = sweep_handle.lock().unwrap().take();
                    if let Some(old) = old_handle {
                        old.cancel.store(true, Ordering::SeqCst);
                        let _ = old.join.join();
                    }
                }

                // ── Step 2: accept the goal ────────────────────────────────────
                let accepted = requested_goal.accept();

                // ── Step 3: extract goal pose from Vi.action Goal message ──────
                let goal_pose = &accepted.goal().goal.pose;
                let yaw = yaw_from_quat(&goal_pose.orientation);
                let goal_theta_deg = yaw_to_goal_theta_deg(yaw);

                // ── Step 4: build a fresh ValueIterator and pin the goal ──────
                let mut vi = map_build.build();
                vi.set_goal(goal_pose.position.x, goal_pose.position.y, goal_theta_deg);

                // ── Step 5: create solver and spawn sweep worker ──────────────
                let solver = match make_solver(&solver_name) {
                    Ok(s) => s,
                    Err(e) => {
                        eprintln!("ERROR: make_solver failed: {e}");
                        let executing = accepted.execute();
                        return executing.aborted_with(vi_interfaces::action::Vi_Result {
                            finished: false,
                        });
                    }
                };
                let cancel = Arc::new(AtomicBool::new(false));
                let dump_slot: Option<Arc<Mutex<Option<vi_node::sweep_thread::DumpData>>>> =
                    if bench_dump_path.is_empty() {
                        None
                    } else {
                        Some(Arc::new(Mutex::new(None)))
                    };
                let handle = spawn_sweep(
                    vi,
                    solver,
                    MAX_SOLVER_BUDGET,
                    Arc::clone(&cancel),
                    dump_slot.clone(),
                );
                let feedback_rx = handle.feedback_rx.clone();

                *sweep_handle.lock().unwrap() = Some(handle);

                // ── Step 6: begin executing, pump feedback ────────────────────
                let executing = accepted.execute();
                let feedback_publisher = executing.feedback_publisher();

                let (done_tx, done_rx) = futures::channel::oneshot::channel::<bool>();
                let sweep_handle_thread = Arc::clone(&sweep_handle);
                std::thread::spawn(move || {
                    // The worker thread is the SOLE authority on convergence: it
                    // breaks its loop on `WorkerStats.converged` and drops
                    // `feedback_tx`, surfacing here as a `Disconnected` recv. We
                    // must NOT infer convergence from `final_delta` (always 0 for
                    // u64 solvers — they signal only via the converged flag).
                    loop {
                        std::thread::sleep(Duration::from_millis(100));

                        if cancel.load(Ordering::Relaxed) {
                            break;
                        }

                        let mut last_tick = None;
                        while let Ok(tick) = feedback_rx.try_recv() {
                            last_tick = Some(tick);
                        }

                        if let Some(tick) = last_tick {
                            let feedback = vi_interfaces::action::Vi_Feedback {
                                current_sweep_times: std_msgs::msg::UInt32MultiArray {
                                    data: vec![tick.sweep_count],
                                    ..Default::default()
                                },
                                deltas: std_msgs::msg::Float32MultiArray {
                                    data: vec![tick.final_delta as f32],
                                    ..Default::default()
                                },
                            };
                            let _ = feedback_publisher.publish(feedback);
                        }

                        if matches!(
                            feedback_rx.try_recv(),
                            Err(crossbeam_channel::TryRecvError::Disconnected)
                        ) {
                            break;
                        }
                    }

                    // ── Step 7: join worker and report result ─────────────────
                    let stats = {
                        let handle = sweep_handle_thread.lock().unwrap().take();
                        if let Some(hnd) = handle {
                            hnd.cancel.store(true, Ordering::SeqCst);
                            hnd.join.join().ok()
                        } else {
                            None
                        }
                    };

                    let finished = stats.map(|s| s.converged).unwrap_or(false);
                    let _ = done_tx.send(finished);
                });

                match done_rx.await {
                    Ok(finished) => {
                        if let Some(slot) = dump_slot {
                            if let Some(dump) = slot.lock().unwrap().take() {
                                let vpath = format!("{}/value_ros2.npy", bench_dump_path);
                                let ppath = format!("{}/policy_ros2.npy", bench_dump_path);
                                if let Err(e) = vi_node::npy::write_f64(&vpath, &dump.value) {
                                    eprintln!("ERROR: write {vpath}: {e}");
                                }
                                if let Err(e) = vi_node::npy::write_f64(&ppath, &dump.policy) {
                                    eprintln!("ERROR: write {ppath}: {e}");
                                }
                                eprintln!("bench dump written to {bench_dump_path}");
                            } else {
                                eprintln!(
                                    "WARN: bench_dump_path set but dump slot was empty \
                                     (sweep cancelled before completion?)"
                                );
                            }
                        }
                        executing.succeeded_with(vi_interfaces::action::Vi_Result { finished })
                    }
                    Err(_) => {
                        executing.aborted_with(vi_interfaces::action::Vi_Result { finished: false })
                    }
                }
            }
        },
    )?;

    Ok(server)
}

// ──────────────────────────────────────────────────────────────────────────────
// spawn_value_function_publisher — 1 Hz OccupancyGrid publisher
// ──────────────────────────────────────────────────────────────────────────────

/// Publish `value_function` and `policy` OccupancyGrids at 1 Hz.
///
/// `value_function` publishes a theta=0 slice of the current total_cost array as
/// a signed-byte OccupancyGrid (0–100 scaled to `threshold_steps`, -1 = unreached).
fn spawn_value_function_publisher(
    node: &Node,
    handle: &Arc<Mutex<Option<SweepHandle>>>,
    grid_meta: &GridMeta,
    threshold_steps: u64,
) -> Result<Timer> {
    use nav_msgs::msg::OccupancyGrid as RosOccupancyGrid;
    use std_msgs::msg::Header;

    let pub_value = node.create_publisher::<RosOccupancyGrid>(
        "value_function".reliable().transient_local().keep_last(1),
    )?;
    let pub_policy = node.create_publisher::<RosOccupancyGrid>(
        "policy".reliable().transient_local().keep_last(1),
    )?;

    let handle_c = Arc::clone(handle);
    let grid_meta = grid_meta.clone();
    let node_clock = node.get_clock();

    let timer = node.create_timer_repeating(std::time::Duration::from_secs(1), move || {
        let h_guard = handle_c.lock().unwrap();
        let Some(h) = h_guard.as_ref() else { return; };

        let theta_idx = 0usize;
        let (tx, rx) = crossbeam_channel::bounded(1);
        if h.request_tx
            .send(WorkerRequest::ValueSlice { theta_idx, resp: tx })
            .is_err()
        {
            return;
        }
        let slice = match rx.recv_timeout(std::time::Duration::from_secs(2)) {
            Ok(s) => s,
            Err(_) => return,
        };

        let data = value_slice_to_occupancy(&slice, threshold_steps);

        let (sec, nanosec) = node_clock.now().to_sec_nanosec().unwrap_or((0, 0));
        let header = || {
            let mut h = Header::default();
            h.stamp.sec = sec;
            h.stamp.nanosec = nanosec;
            h.frame_id = "map".into();
            h
        };

        let msg = RosOccupancyGrid {
            header: header(),
            info: grid_meta.to_map_meta_data(),
            data,
        };
        let _ = pub_value.publish(msg);

        // Policy publisher placeholder — all cells -1 (unknown) until the
        // worker exposes the latest policy table. See spec §8 open items.
        let _ = pub_policy.publish(RosOccupancyGrid {
            header: header(),
            info: grid_meta.to_map_meta_data(),
            data: vec![-1i8; (grid_meta.width * grid_meta.height) as usize],
        });
    })?;

    Ok(timer)
}

// ──────────────────────────────────────────────────────────────────────────────
// spawn_cmd_vel_timer — 10 Hz cmd_vel publisher (online mode only)
// ──────────────────────────────────────────────────────────────────────────────

/// Publish `cmd_vel` Twist at 10 Hz when `params.online == true`.
///
/// # tf2 deferral
/// TODO(tf2_rs): the robot pose `(ix, iy, it)` is currently hardcoded to
/// `(0, 0, 0)`. When tf2_rs is integrated, replace with a `map → base_link`
/// transform lookup. Until then this is only useful for smoke-testing topic flow.
fn spawn_cmd_vel_timer(
    node: &Node,
    handle: &Arc<Mutex<Option<SweepHandle>>>,
    _grid_meta: &GridMeta,
    action_list: &[(String, f64, f64)],
) -> Result<Timer> {
    use geometry_msgs::msg::Twist;

    let pub_cmd = node.create_publisher::<Twist>("cmd_vel".keep_last(2))?;

    let actions: Vec<(f64, f64)> = action_list.iter().map(|(_, fw, rot)| (*fw, *rot)).collect();

    let handle_c = Arc::clone(handle);
    let period = std::time::Duration::from_millis(100);

    let timer = node.create_timer_repeating(period, move || {
        let h_guard = handle_c.lock().unwrap();
        let Some(h) = h_guard.as_ref() else {
            let _ = pub_cmd.publish(Twist::default());
            return;
        };

        // TODO(tf2_rs): replace (0, 0, 0) with map → base_link lookup.
        let (ix, iy, it) = (0i32, 0i32, 0usize);

        let (tx, rx) = crossbeam_channel::bounded(1);
        if h.request_tx
            .send(WorkerRequest::OptimalAction { ix, iy, it, resp: tx })
            .is_err()
        {
            let _ = pub_cmd.publish(Twist::default());
            return;
        }
        let aid = match rx.recv_timeout(std::time::Duration::from_secs(2)) {
            Ok(a) => a,
            Err(_) => {
                let _ = pub_cmd.publish(Twist::default());
                return;
            }
        };

        // -1 (no action / obstacle / goal) → zero velocity.
        let (fw, rot_deg) = if aid >= 0 {
            actions.get(aid as usize).copied().unwrap_or((0.0, 0.0))
        } else {
            (0.0, 0.0)
        };
        let mut tw = Twist::default();
        tw.linear.x = fw / period.as_secs_f64();
        tw.angular.z = rot_deg.to_radians() / period.as_secs_f64();
        let _ = pub_cmd.publish(tw);
    })?;

    Ok(timer)
}
