//! vi_node entry point.
//!
//! Boot order (see spec §4.1):
//!   1. `Context::default_from_env` + basic executor + node creation
//!   2. Parameters declared and validated (fail-fast on mismatch)
//!   3. Rayon thread-pool init (parallel feature only)
//!   4. /map received (transient_local, blocks until first message)
//!   5. Penalty + transitions + initial VIContext built
//!   6. Action server, publishers, timers wired
//!   7. executor.spin()
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
use ndarray::Array3;

use vi_algorithm::context::{MapDims, VIContext};
use vi_core::{
    make_goal_mask, MAX_VALUE, N_ACTIONS, N_THETA,
    ACTION_FW, ACTION_ROT,
};
use vi_fixtures::{generate_transitions, TransitionMode};
use vi_node::bridge::{
    occupancy_to_penalty, pose_to_goal_spec, OccupancyGridView, PenaltyParams, PoseView,
};
use vi_node::solver_factory::make_solver;
use vi_node::sweep_thread::{spawn_sweep, SweepHandle, WorkerRequest};

// rclrs API — matches upstream main-branch executor/node pattern.
// `use rclrs::*` brings in: Context, Executor, Node, CreateBasicExecutor,
//   Publisher, Subscription, QoSProfile, SpinOptions, RclrsError,
//   IntoActionServerOptions, RequestedGoal, TerminatedGoal, …
use rclrs::*;

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
    allow_action_mismatch: bool,
    // Benchmark hook: when non-empty, the action callback dumps the full
    // value/policy arrays as `.npy` into this directory after the solve
    // finishes. Empty (default) = off = unchanged production behavior.
    bench_dump_path: String,
    // Flattened from three parallel arrays (names, fw, rot).
    action_list: Vec<(String, f64, f64)>,
}

/// Declare all node parameters and collect their values.
///
/// rclrs ParameterBuilder API (rclrs 0.7 parameter.rs):
///   `node.declare_parameter(name).default(value).mandatory()?`
///   `.get()` → returns a clone of the stored value.
///
/// `ParameterVariant` is implemented for `bool`, `i64`, `f64`, `Arc<str>`,
/// `Arc<[i64]>`, `Arc<[f64]>`, and `Arc<[Arc<str>]>` — there is no `String` /
/// `Vec<T>` impl, so string params use `Arc<str>` and array params use the
/// `Arc<[..]>` forms (built via `.default_string_array` / `.default_from_iter`).
fn read_params(node: &Node) -> Result<Params> {
    // Scalar parameters.
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

    let allow_action_mismatch = node
        .declare_parameter::<bool>("allow_action_mismatch")
        .default(false)
        .mandatory()
        .map_err(|e| anyhow!("declare allow_action_mismatch: {e}"))?
        .get();

    // Benchmark dump directory. String params use `Arc<str>` (no `String`
    // ParameterVariant impl — see the module docstring); empty default = off.
    let bench_dump_path = node
        .declare_parameter::<Arc<str>>("bench_dump_path")
        .default("".into())
        .mandatory()
        .map_err(|e| anyhow!("declare bench_dump_path: {e}"))?
        .get()
        .to_string();

    // Action list — three parallel arrays instead of list-of-dicts (rclrs
    // does not support nested dict parameters). rclrs array params are
    // `Arc<[Arc<str>]>` / `Arc<[f64]>`; the `default_string_array` and
    // `default_from_iter` builder helpers populate them from iterables.
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
        allow_action_mismatch,
        bench_dump_path,
        action_list,
    })
}

// ──────────────────────────────────────────────────────────────────────────────
// Parameter validation
// ──────────────────────────────────────────────────────────────────────────────

/// Validate parameters against compiled-in vi_rs constants (fail-fast).
fn validate(p: &Params) -> Result<()> {
    if p.theta_cell_num != N_THETA as i64 {
        return Err(anyhow!(
            "vi_rs is compiled with N_THETA={}, got theta_cell_num={}",
            N_THETA,
            p.theta_cell_num
        ));
    }
    if p.action_list.len() != N_ACTIONS {
        return Err(anyhow!(
            "vi_rs requires exactly {} actions, got {}",
            N_ACTIONS,
            p.action_list.len()
        ));
    }
    for (i, (_, fw, rot)) in p.action_list.iter().enumerate() {
        if (fw - ACTION_FW[i]).abs() > 1e-6 || (rot - ACTION_ROT[i]).abs() > 1e-6 {
            let msg = format!(
                "action[{i}] differs from vi_rs constants: got (fw={fw}, rot={rot}), \
                 expected (fw={}, rot={})",
                ACTION_FW[i], ACTION_ROT[i]
            );
            if p.allow_action_mismatch {
                eprintln!("WARN: {msg}");
            } else {
                return Err(anyhow!(msg));
            }
        }
    }
    // Verify solver string is known; make_solver is pure-Rust and checkable here.
    make_solver(&p.solver)?;
    Ok(())
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
    // thread_num == 0 → let rayon choose (#CPUs), no explicit setup needed.
}

#[cfg(not(feature = "parallel"))]
fn init_rayon(thread_num: i64) {
    if thread_num > 0 {
        eprintln!(
            "WARN: thread_num={thread_num} ignored (built without --features parallel)"
        );
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// GridMeta helper — small owned copy of grid geometry for timers / publishers
// (they outlive the OccupancyGrid message from wait_for_map).
// ──────────────────────────────────────────────────────────────────────────────

/// Owned copy of map geometry. Used by publishers and cmd_vel timer (Task 10).
#[derive(Clone)]
pub(crate) struct GridMeta {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
}

impl GridMeta {
    /// Convert to the `MapMetaData` sub-message used inside `OccupancyGrid`.
    ///
    /// NOTE: `nav_msgs::msg::MapMetaData` field shapes follow REP-103; these
    /// are standard and not expected to change between rclrs versions.
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
    //    Upstream pattern (rclrs lib.rs doc example):
    //      Context::default_from_env()? → context.create_basic_executor()
    //      → executor.create_node("name")?
    let context = Context::default_from_env().context("rclrs context init")?;
    let mut executor = context.create_basic_executor();
    let node = executor
        .create_node("vi_node")
        .context("create vi_node")?;

    // 2. Parameters.
    let params = read_params(&node).context("reading parameters")?;
    validate(&params).context("validating parameters")?;

    // 3. Rayon thread-pool.
    init_rayon(params.thread_num);

    // 4. Wait for /map (transient_local, blocks until first message).
    let map_msg = wait_for_map(&node, &mut executor, params.map_wait_sec)
        .context("waiting for /map")?;

    // 5. Build MapResources (penalty + transitions + base VIContext).
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
    let pen_params = PenaltyParams {
        safety_radius_m: params.safety_radius,
        safety_radius_penalty: params.safety_radius_penalty as u16,
        unknown_as_obstacle: true,
    };
    let penalty = occupancy_to_penalty(&grid_view, &pen_params);

    // Build transitions from map resolution (PaperMonteCarlo matches the
    // legacy `value_iteration` node default).
    let trans = generate_transitions(TransitionMode::PaperMonteCarlo {
        xy_resolution: grid_meta.resolution,
    });

    // Blank value array; goal cells will be pinned per-action-goal.
    let blank_value = Array3::<u16>::from_elem(
        (
            grid_meta.height as usize,
            grid_meta.width as usize,
            N_THETA,
        ),
        MAX_VALUE,
    );
    let blank_goal_mask =
        Array3::from_elem((grid_meta.height as usize, grid_meta.width as usize, N_THETA), false);

    let base_ctx = VIContext {
        dims: MapDims {
            map_x: grid_meta.width,
            map_y: grid_meta.height,
        },
        value: blank_value,
        penalty,
        goal_mask: blank_goal_mask,
        transitions: trans.unpack(),
    };

    // Shared sweep handle — replaced on every new action goal.
    let sweep_handle: Arc<Mutex<Option<SweepHandle>>> = Arc::new(Mutex::new(None));

    // 6. Wire action server.
    // The action server and timer guards must stay alive for the spin's
    // lifetime; dropping a timer guard stops the timer from firing.
    let _action_server = spawn_action_server(&node, &params, &sweep_handle, &base_ctx, &grid_meta)?;

    // Wire publishers + timers (Task 10 stubs — do not panic before spin).
    let _vf_timer = spawn_value_function_publisher(
        &node,
        &sweep_handle,
        &grid_meta,
        params.cost_drawing_threshold as u16,
    )?;
    let _cmd_vel_timer = if params.online {
        Some(spawn_cmd_vel_timer(&node, &sweep_handle, &grid_meta, &params.action_list)?)
    } else {
        None
    };

    // 7. Spin (blocks until shutdown).
    // `first_error()` converts the Vec<RclrsError> into a single Result.
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

// ──────────────────────────────────────────────────────────────────────────────
// wait_for_map — transient_local subscriber, blocks until first message
// ──────────────────────────────────────────────────────────────────────────────

/// Subscribe to `/map` with transient_local QoS and block until one message
/// arrives (or the deadline is reached).
///
/// Pattern: create subscription with a `sync_channel(1)` callback, then spin
/// the executor in short bursts while polling the channel.
///
/// QoS is built with the `IntoPrimitiveOptions` builder on the topic `&str`
/// (`.transient_local()`, `.reliable()`, `.keep_last(n)`), which yields the
/// `impl Into<SubscriptionOptions>` that `create_subscription` accepts. The
/// callback takes the message by value.
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
        // Spin for up to 100 ms to let pending subscriptions deliver, then poll
        // the channel again. `SpinOptions::default().timeout(d)` bounds each burst.
        executor.spin(SpinOptions::default().timeout(Duration::from_millis(100)));
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// spawn_action_server — vi_controller Vi.action server
// ──────────────────────────────────────────────────────────────────────────────

/// Wire the `vi_controller` action server (spec §4.2).
///
/// The returned `ActionServer<vi_interfaces::action::Vi>` Arc MUST be kept
/// alive for the spin's lifetime — the caller stores it in `_action_server`.
///
/// The callback receives a `RequestedGoal<Vi>`, accepts it, executes it:
///   1. Cancel any in-flight sweep.
///   2. Rebuild goal_mask, reinitialise value.
///   3. Spawn new sweep worker.
///   4. Pump FeedbackTick at 10 Hz, publish Vi_Feedback.
///   5. Join worker → publish Vi_Result.
///
/// # Async runtime
/// rclrs polls the returned future on its own `futures`-based executor (no
/// tokio runtime). The blocking feedback-pump + worker-join therefore runs on
/// a dedicated `std::thread`, communicating the terminal `finished` flag back
/// to this async callback through a `futures::channel::oneshot`. Feedback is
/// published from that thread via a cloned `FeedbackPublisher`, which is
/// `Clone + Send + Sync` and keeps working until the goal reaches a terminal
/// state.
///
/// The rosidl-generated type paths follow the `Fibonacci` / `Fibonacci_Goal` /
/// `Fibonacci_Result` / `Fibonacci_Feedback` convention, so for this action
/// they are `vi_interfaces::action::{Vi, Vi_Goal, Vi_Result, Vi_Feedback}` and
/// the action's associated types are `Vi::Goal` / `Vi::Result` / `Vi::Feedback`.
fn spawn_action_server(
    node: &Node,
    params: &Params,
    sweep_handle: &Arc<Mutex<Option<SweepHandle>>>,
    base_ctx: &VIContext,
    grid_meta: &GridMeta,
) -> Result<ActionServer<vi_interfaces::action::Vi>> {
    let sweep_handle = Arc::clone(sweep_handle);
    let base_ctx = base_ctx.clone_value(); // owned clone; vi_algorithm::context::VIContext::clone_value
    let grid_meta = grid_meta.clone();
    let solver_name = params.solver.clone();
    let goal_margin_radius = params.goal_margin_radius;
    let goal_margin_theta_deg = params.goal_margin_theta_deg;
    let bench_dump_path = params.bench_dump_path.clone();

    // node.create_action_server signature (rclrs 0.7 node.rs):
    //   pub fn create_action_server<'a, A: Action, Task>(
    //       self: &Arc<Self>,
    //       options: impl IntoActionServerOptions<'a>,
    //       callback: impl FnMut(RequestedGoal<A>) -> Task + Send + Sync + 'static,
    //   ) -> Result<ActionServer<A>, RclrsError>
    //   where Task: Future<Output = TerminatedGoal> + Send + Sync + 'static
    //
    // A bare &str satisfies `IntoActionServerOptions`.
    let server = node.create_action_server::<vi_interfaces::action::Vi, _>(
        "vi_controller",
        move |requested_goal: RequestedGoal<vi_interfaces::action::Vi>| {
            // Clone shared state for the async closure.
            let sweep_handle = Arc::clone(&sweep_handle);
            let mut base_ctx = base_ctx.clone_value();
            let grid_meta = grid_meta.clone();
            let solver_name = solver_name.clone();
            let bench_dump_path = bench_dump_path.clone();

            async move {
                // ── Step 1: cancel any prior in-flight sweep ──────────────────
                // The worker checks `cancel` at each sweep boundary, so this
                // join returns quickly. We are in an async task with no blocking
                // budget concern (the sweep is already winding down), so join
                // directly rather than via a runtime-specific spawn_blocking.
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
                // Vi.action Goal field: `geometry_msgs/PoseStamped goal`
                // (see vi_ros2/vi_interfaces/action/Vi.action). `accepted.goal()`
                // returns `&Arc<Vi_Goal>`, so `.goal.pose` reaches the PoseStamped.
                let goal_pose = &accepted.goal().goal.pose;
                let yaw = yaw_from_quat(&goal_pose.orientation);
                let pose_view = PoseView {
                    x: goal_pose.position.x,
                    y: goal_pose.position.y,
                    yaw_rad: yaw,
                };

                // Build a temporary OccupancyGridView with a dummy empty slice
                // (we only need the grid geometry, not the actual data).
                let tmp_grid = OccupancyGridView {
                    width: grid_meta.width,
                    height: grid_meta.height,
                    resolution: grid_meta.resolution,
                    origin_x: grid_meta.origin_x,
                    origin_y: grid_meta.origin_y,
                    data: &[], // not used by pose_to_goal_spec
                };

                let goal_spec = pose_to_goal_spec(
                    &pose_view,
                    &tmp_grid,
                    goal_margin_radius,
                    goal_margin_theta_deg,
                );

                // ── Step 4: rebuild goal_mask and reinitialise value ──────────
                let goal_mask = make_goal_mask(grid_meta.width, grid_meta.height, &goal_spec);

                // Pin goal cells to 0; all others reset to MAX_VALUE.
                let h = grid_meta.height as usize;
                let w = grid_meta.width as usize;
                let mut new_value = Array3::<u16>::from_elem((h, w, N_THETA), MAX_VALUE);
                for ((iy, ix, it), &is_goal) in goal_mask.indexed_iter() {
                    if is_goal {
                        new_value[[iy, ix, it]] = 0;
                    }
                }
                base_ctx.value = new_value;
                base_ctx.goal_mask = goal_mask;

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
                // Benchmark hook: allocate a dump slot when bench_dump_path is
                // set. The worker fills it with the final value/policy arrays
                // right before it exits; we read it after the worker is joined.
                let dump_slot: Option<Arc<Mutex<Option<vi_node::sweep_thread::DumpData>>>> =
                    if bench_dump_path.is_empty() {
                        None
                    } else {
                        Some(Arc::new(Mutex::new(None)))
                    };
                let handle = spawn_sweep(base_ctx, solver, Arc::clone(&cancel), dump_slot.clone());
                let feedback_rx = handle.feedback_rx.clone();

                // Store handle so publishers can access it and action can cancel.
                *sweep_handle.lock().unwrap() = Some(handle);

                // ── Step 6: begin executing, pump feedback ────────────────────
                let executing = accepted.execute();
                let feedback_publisher = executing.feedback_publisher();

                // Run the 10 Hz feedback pump + final worker-join on a std thread
                // (rclrs has no tokio runtime, so we cannot use tokio timers).
                // The thread reports the terminal `finished` flag back through a
                // oneshot the async callback awaits.
                let (done_tx, done_rx) = futures::channel::oneshot::channel::<bool>();
                let sweep_handle_thread = Arc::clone(&sweep_handle);
                std::thread::spawn(move || {
                    let mut converged = false;
                    loop {
                        std::thread::sleep(Duration::from_millis(100));

                        // The cancel flag on the SweepHandle doubles as both the
                        // ROS-cancel signal and the vi_rs cancel; checking it here
                        // covers both.
                        if cancel.load(Ordering::Relaxed) {
                            break;
                        }

                        // Drain all pending feedback ticks.
                        let mut last_tick = None;
                        while let Ok(tick) = feedback_rx.try_recv() {
                            last_tick = Some(tick);
                        }

                        if let Some(tick) = last_tick {
                            converged = tick.final_delta == 0;

                            // Vi.action feedback:
                            //   std_msgs/UInt32MultiArray current_sweep_times
                            //   std_msgs/Float32MultiArray deltas
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

                            if converged {
                                break;
                            }
                        }

                        // The worker drops feedback_tx on exit; a Disconnected
                        // result means the sweep has finished.
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

                    let finished = stats.map(|s| s.converged).unwrap_or(converged);
                    let _ = done_tx.send(finished);
                });

                // Await the pump thread's terminal result. If the sender is
                // dropped without sending (thread panicked), treat as not
                // finished and abort the goal rather than block forever.
                //
                // The pump thread joins the sweep worker (`hnd.join.join()`)
                // BEFORE sending `done_tx`, and the worker fills `dump_slot`
                // right before it returns — so once `done_rx.await` completes
                // the slot is guaranteed populated. Read/write it here.
                match done_rx.await {
                    Ok(finished) => {
                        if let Some(slot) = dump_slot {
                            if let Some(dump) = slot.lock().unwrap().take() {
                                let vpath = format!("{}/value_ros2.npy", bench_dump_path);
                                let ppath = format!("{}/policy_ros2.npy", bench_dump_path);
                                if let Err(e) = vi_node::npy::write_u16(&vpath, &dump.value) {
                                    eprintln!("ERROR: write {vpath}: {e}");
                                }
                                if let Err(e) = vi_node::npy::write_i16(&ppath, &dump.policy) {
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
// spawn_value_function_publisher — 1 Hz OccupancyGrid publisher (Task 10)
// ──────────────────────────────────────────────────────────────────────────────

/// Publish `value_function` and `policy` OccupancyGrids at 1 Hz.
///
/// `value_function` publishes a theta=0 slice of the current value array as
/// a signed-byte OccupancyGrid (0–100 scaled to `threshold`, -1 = unreachable).
///
/// `policy` publishes a placeholder grid of all -1 (unknown) until the worker
/// exposes the latest ActionTable. See spec §8 open items — wiring it to
/// `WorkerRequest::ActionTableSlice` is left as a follow-up.
///
/// # API notes
/// - `node.create_publisher::<T>("topic".reliable()...)` takes a single options
///   argument built via `IntoPrimitiveOptions`; QoS is folded into it.
/// - `node.create_timer_repeating(duration, callback)` returns a `Timer` guard;
///   we keep it alive by returning it to main() (dropping the guard would stop
///   the timer). The callback is `FnMut() + Send`.
/// - `node.get_clock().now().to_ros_msg()` yields the `builtin_interfaces` Time
///   used in `Header.stamp` (fallible on i64→i32 overflow; default on error).
/// - `Publisher::publish(msg)` takes the message by value.
fn spawn_value_function_publisher(
    node: &Node,
    handle: &Arc<Mutex<Option<SweepHandle>>>,
    grid_meta: &GridMeta,
    threshold: u16,
) -> Result<Timer> {
    use nav_msgs::msg::OccupancyGrid;
    use std_msgs::msg::Header;

    let pub_value = node.create_publisher::<OccupancyGrid>(
        "value_function".reliable().transient_local().keep_last(1),
    )?;
    let pub_policy = node.create_publisher::<OccupancyGrid>(
        "policy".reliable().transient_local().keep_last(1),
    )?;

    let handle_c = Arc::clone(handle);
    let grid_meta = grid_meta.clone();
    let node_clock = node.get_clock();

    let timer = node.create_timer_repeating(std::time::Duration::from_secs(1), move || {
        let h_guard = handle_c.lock().unwrap();
        let Some(h) = h_guard.as_ref() else { return; };

        // Request theta=0 slice from the worker (online mode uses current yaw;
        // offline always uses yaw=0 as a representative slice for visualisation).
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

        // Worker returns Array2<Value> [h, w]; insert a dummy theta axis so
        // value_slice_to_occupancy sees ArrayView3 [h, w, 1] and theta_idx=0.
        let data = vi_node::bridge::value_slice_to_occupancy(
            slice.view().insert_axis(ndarray::Axis(2)),
            0,
            threshold,
        );

        // Build the Header stamp from the rclrs clock. The clock yields rclrs's
        // internal `ros_env::builtin_interfaces` Time, which is a *distinct* type
        // from the message-side `builtin_interfaces::msg::Time` in Header.stamp;
        // rather than depend on builtin_interfaces directly just to name that
        // type, set the stamp fields on a default Header (no type name needed).
        let (sec, nanosec) = node_clock.now().to_sec_nanosec().unwrap_or((0, 0));
        let header = || {
            let mut h = Header::default();
            h.stamp.sec = sec;
            h.stamp.nanosec = nanosec;
            h.frame_id = "map".into();
            h
        };

        let msg = OccupancyGrid {
            header: header(),
            info: grid_meta.to_map_meta_data(),
            data,
        };
        let _ = pub_value.publish(msg);

        // Policy publisher placeholder — all cells -1 (unknown) until the
        // worker exposes the latest ActionTable. See spec §8 open items.
        let _ = pub_policy.publish(OccupancyGrid {
            header: header(),
            info: grid_meta.to_map_meta_data(),
            data: vec![-1i8; (grid_meta.width * grid_meta.height) as usize],
        });
    })?;

    Ok(timer)
}

// ──────────────────────────────────────────────────────────────────────────────
// spawn_cmd_vel_timer — 10 Hz cmd_vel publisher (Task 10, online mode only)
// ──────────────────────────────────────────────────────────────────────────────

/// Publish `cmd_vel` Twist at 10 Hz when `params.online == true`.
///
/// Each tick requests the optimal action for the current robot cell via
/// `WorkerRequest::OptimalAction` and converts it to forward/angular velocity.
/// Velocities are computed as motion-per-period / period so they represent
/// instantaneous body-frame rates (m/s and rad/s).
///
/// # tf2 deferral
/// TODO(tf2_rs): the robot pose `(ix, iy, it)` is currently hardcoded to
/// `(0, 0, 0)`. When tf2_rs is integrated, replace with a `map → base_link`
/// transform lookup and convert the translation/yaw to grid cell indices via
/// a helper analogous to `pose_to_goal_spec` (inline or extracted to bridge.rs
/// once it grows — see spec §4.5).
///
/// Until tf2_rs is available this function is only useful for smoke-testing
/// the topic flow (confirming cmd_vel appears and responds to value changes).
///
/// # API notes
/// - See notes in `spawn_value_function_publisher`: `create_publisher` takes a
///   single options arg (QoS via `IntoPrimitiveOptions`), `create_timer_repeating`
///   returns a `Timer` guard we must keep alive, and `publish` is by-value.
fn spawn_cmd_vel_timer(
    node: &Node,
    handle: &Arc<Mutex<Option<SweepHandle>>>,
    _grid_meta: &GridMeta,
    action_list: &[(String, f64, f64)],
) -> Result<Timer> {
    use geometry_msgs::msg::Twist;

    let pub_cmd = node.create_publisher::<Twist>("cmd_vel".keep_last(2))?;

    // Collect (forward_m, rotation_deg) pairs indexed by action id.
    let actions: Vec<(f64, f64)> = action_list
        .iter()
        .map(|(_, fw, rot)| (*fw, *rot))
        .collect();

    let handle_c = Arc::clone(handle);
    let period = std::time::Duration::from_millis(100);

    let timer = node.create_timer_repeating(period, move || {
        let h_guard = handle_c.lock().unwrap();
        let Some(h) = h_guard.as_ref() else {
            // No active sweep — publish zero velocity.
            let _ = pub_cmd.publish(Twist::default());
            return;
        };

        // TODO(tf2_rs): replace (0, 0, 0) with map → base_link lookup when
        // tf2_rs is available. Until then cmd_vel always queries the same cell,
        // which is only useful for smoke-testing the topic flow.
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
            Ok(a) => a as usize,
            Err(_) => {
                let _ = pub_cmd.publish(Twist::default());
                return;
            }
        };

        let (fw, rot_deg) = actions.get(aid).copied().unwrap_or((0.0, 0.0));
        let mut tw = Twist::default();
        // Convert per-period motion to instantaneous body-frame rates.
        tw.linear.x = fw / period.as_secs_f64();
        tw.angular.z = rot_deg.to_radians() / period.as_secs_f64();
        let _ = pub_cmd.publish(tw);
    })?;

    Ok(timer)
}
