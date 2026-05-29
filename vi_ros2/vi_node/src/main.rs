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
//! NOTE: This file uses the rclrs API as found on ros2-rust/ros2_rust main branch
//! (commit 2c6b926, Jan 2026). The action-server callback is async (tokio-based).
//! Compile verification is deferred to Task 11 (make ros2-build inside Docker).

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
    delta_threshold: i64,
    thread_num: i64,
    map_wait_sec: i64,
    allow_action_mismatch: bool,
    // Flattened from three parallel arrays (names, fw, rot).
    action_list: Vec<(String, f64, f64)>,
}

/// Declare all node parameters and collect their values.
///
/// rclrs ParameterBuilder API (upstream node.rs line 1416):
///   `node.declare_parameter::<T>(name).default(value).mandatory()?`
///   `.get()` → returns a clone of the stored value.
///
/// NOTE: The exact `ParameterVariant` for `String` may be `Arc<str>` rather
/// than `String` depending on the rclrs version in Docker. If the compiler
/// complains, change `String` to `Arc<str>` and call `.to_string()` on `.get()`.
///
/// TODO(Task 11): adjust types and `.get()` coercions to match the exact
/// ParameterVariant impls that colcon exposes.
fn read_params(node: &Node) -> Result<Params> {
    // Scalar parameters.
    let solver = node
        .declare_parameter::<String>("solver")
        .default("frontier3d".to_string())
        .mandatory()
        .map_err(|e| anyhow!("declare solver: {e}"))?
        .get();

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

    // Action list — three parallel arrays instead of list-of-dicts (rclrs
    // Humble does not support nested dict parameters).
    //
    // TODO(Task 11): rclrs ParameterVariant for Vec<String> may need
    // `Arc<[Arc<str>]>` or similar; adjust after `make ros2-build`.
    let names: Vec<String> = node
        .declare_parameter::<Vec<String>>("action_names")
        .default(vec![
            "forward".into(),
            "back".into(),
            "right".into(),
            "rightfw".into(),
            "left".into(),
            "leftfw".into(),
        ])
        .mandatory()
        .map_err(|e| anyhow!("declare action_names: {e}"))?
        .get();

    let fws: Vec<f64> = node
        .declare_parameter::<Vec<f64>>("action_forward_m")
        .default(vec![0.3, -0.2, 0.0, 0.2, 0.0, 0.2])
        .mandatory()
        .map_err(|e| anyhow!("declare action_forward_m: {e}"))?
        .get();

    let rots: Vec<f64> = node
        .declare_parameter::<Vec<f64>>("action_rotation_deg")
        .default(vec![0.0, 0.0, -20.0, -20.0, 20.0, 20.0])
        .mandatory()
        .map_err(|e| anyhow!("declare action_rotation_deg: {e}"))?
        .get();

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
    let _action_server = spawn_action_server(&node, &params, &sweep_handle, &base_ctx, &grid_meta)?;

    // Wire publishers + timers (Task 10 stubs — do not panic before spin).
    spawn_value_function_publisher(&node, &sweep_handle, &grid_meta, params.cost_drawing_threshold as u16)?;
    if params.online {
        spawn_cmd_vel_timer(&node, &sweep_handle, &grid_meta, &params.action_list)?;
    }

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
/// NOTE: `rclrs::QoSProfile` methods `reliable()`, `transient_local()`,
/// `keep_last(n)` are confirmed in upstream qos.rs lines 258-291.
/// `SubscriptionOptions` accepts `impl Into<SubscriptionOptions>` which is
/// satisfied by `(topic_str, qos_profile)` via `IntoPrimitiveOptions` —
/// see upstream subscription.rs for the exact conversion impl.
///
/// TODO(Task 11): if the subscription API rejects the `(topic, qos)` tuple,
/// construct `SubscriptionOptions::new("map").qos(profile)` explicitly.
fn wait_for_map(
    node: &Node,
    executor: &mut Executor,
    wait_sec: i64,
) -> Result<nav_msgs::msg::OccupancyGrid> {
    use std::sync::mpsc::sync_channel;

    let (tx, rx) = sync_channel::<nav_msgs::msg::OccupancyGrid>(1);
    let tx_c = tx.clone();

    // TODO(Task 11): verify exact subscription creation signature.
    // The upstream `node.create_subscription` takes:
    //   (options: impl Into<SubscriptionOptions>, callback: impl Callback)
    // where a bare `&str` satisfies `Into<SubscriptionOptions>` with default QoS.
    // IntoPrimitiveOptions on `&str` adds `.transient_local()`, `.reliable()`,
    // `.keep_last(n)` builder methods — upstream subscription.rs line 916 shows
    // `"my_topic".transient_local()`.
    // If that doesn't compile, construct QoSProfile explicitly and pass via
    // SubscriptionOptions: `SubscriptionOptions { topic: "map", qos: <profile> }`.
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
        // Spin for 100 ms to allow pending subscriptions to deliver.
        // `SpinOptions::default().timeout(d)` — upstream executor.rs line 416-430
        // shows SpinOptions builder with `.timeout()`.
        //
        // TODO(Task 11): if `SpinOptions::spin_once()` exists (upstream line 416)
        // prefer that; otherwise this timeout-based spin works.
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
/// NOTE: The exact rosidl-generated type paths for vi_interfaces depend on
/// how rosidl_generator_rs names them. Two common conventions:
///   a) `vi_interfaces::action::Vi`   (module path)
///   b) `vi_interfaces::action::Vi_Goal`, `Vi_Result`, `Vi_Feedback`
/// The typical ros2_rust convention at the time of writing is (a) for the
/// action type and the associated types are `Vi::Goal`, `Vi::Result`, `Vi::Feedback`.
///
/// TODO(Task 11): Verify the rosidl type paths after `make ros2-build`.
/// If `vi_interfaces::action::Vi` does not exist, try `vi_interfaces::action::vi::Vi`.
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

    // node.create_action_server signature (upstream node.rs line 401-410):
    //   pub fn create_action_server<'a, A: Action, Task>(
    //       self: &Arc<Self>,
    //       options: impl IntoActionServerOptions<'a>,
    //       callback: impl FnMut(RequestedGoal<A>) -> Task + Send + Sync + 'static,
    //   ) -> Result<ActionServer<A>, RclrsError>
    //   where Task: Future<Output = TerminatedGoal> + Send + Sync + 'static
    //
    // A bare &str satisfies `IntoActionServerOptions` (upstream action_server.rs line 135).
    let server = node.create_action_server::<vi_interfaces::action::Vi, _>(
        "vi_controller",
        move |requested_goal: RequestedGoal<vi_interfaces::action::Vi>| {
            // Clone shared state for the async closure.
            let sweep_handle = Arc::clone(&sweep_handle);
            let mut base_ctx = base_ctx.clone_value();
            let grid_meta = grid_meta.clone();
            let solver_name = solver_name.clone();

            async move {
                // ── Step 1: cancel any prior in-flight sweep ──────────────────
                {
                    let old_handle = sweep_handle.lock().unwrap().take();
                    if let Some(old) = old_handle {
                        old.cancel.store(true, Ordering::SeqCst);
                        // Blocking join — wrap in spawn_blocking so we don't
                        // stall the tokio async runtime.
                        // TODO(Task 11): confirm tokio is available as a dep in
                        // the rclrs Docker build; if not, join synchronously
                        // (acceptable since the worker checks cancel at each sweep).
                        let _ = tokio::task::spawn_blocking(move || {
                            let _ = old.join.join();
                        })
                        .await;
                    }
                }

                // ── Step 2: accept the goal ────────────────────────────────────
                let accepted = requested_goal.accept();

                // ── Step 3: extract goal pose from Vi.action Goal message ──────
                // Vi.action Goal field: `geometry_msgs/PoseStamped goal`
                // (see vi_ros2/vi_interfaces/action/Vi.action)
                //
                // TODO(Task 11): verify rosidl field access:
                //   requested_goal.goal().goal.pose.position.{x,y}
                //   requested_goal.goal().goal.pose.orientation.{w,x,y,z}
                // The accepted goal still has access to the goal data.
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
                        // TODO(Task 11): Vi_Result { finished: bool }
                        // rosidl type: vi_interfaces::action::Vi_Result or Vi::Result
                        return executing.aborted_with(vi_interfaces::action::Vi_Result {
                            finished: false,
                        });
                    }
                };
                let cancel = Arc::new(AtomicBool::new(false));
                let handle = spawn_sweep(base_ctx, solver, Arc::clone(&cancel));
                let feedback_rx = handle.feedback_rx.clone();

                // Store handle so publishers can access it and action can cancel.
                *sweep_handle.lock().unwrap() = Some(handle);

                // ── Step 6: begin executing, pump feedback ────────────────────
                let executing = accepted.execute();
                let feedback_publisher = executing.feedback_publisher();

                // 10 Hz feedback loop using tokio interval.
                // TODO(Task 11): confirm tokio::time is available.
                let mut interval = tokio::time::interval(Duration::from_millis(100));
                let mut converged = false;

                loop {
                    interval.tick().await;

                    // Check for ROS cancellation request.
                    // TODO(Task 11): `executing.until_cancel_requested(future)` is the
                    // rclrs-idiomatic way; for simplicity we check cancel_requested via
                    // the live goal state. If `ExecutingGoal` exposes `cancel_requested()`
                    // or a flag, use that instead.
                    // For now, the cancel flag on the SweepHandle doubles as both
                    // ROS-cancel and vi_rs cancel — checking `cancel` AtomicBool suffices.
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

                        // TODO(Task 11): verify Vi_Feedback field paths.
                        // Vi.action feedback: `std_msgs/UInt32MultiArray current_sweep_times`
                        //                    `std_msgs/Float32MultiArray deltas`
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
                        // FeedbackPublisher::publish (upstream feedback_publisher.rs line 14)
                        let _ = feedback_publisher.publish(feedback);

                        if converged {
                            break;
                        }
                    }

                    // Check if the worker thread has finished (join handle is_finished).
                    // TODO(Task 11): JoinHandle::is_finished() is stable Rust 1.61+
                    // — available on our MSRV 1.75. However, the handle is inside
                    // the Mutex<Option<SweepHandle>>. We can't call is_finished() without
                    // temporarily removing it; instead we rely on the feedback channel
                    // closing (try_recv returning Err(Disconnected)) to detect completion.
                    // The worker sends on every sweep and drops feedback_tx on exit.
                    if matches!(feedback_rx.try_recv(), Err(crossbeam_channel::TryRecvError::Disconnected)) {
                        break;
                    }
                }

                // ── Step 7: join worker and send result ───────────────────────
                let stats = {
                    let handle = sweep_handle.lock().unwrap().take();
                    if let Some(h) = handle {
                        h.cancel.store(true, Ordering::SeqCst);
                        // Blocking join again via spawn_blocking.
                        tokio::task::spawn_blocking(move || h.join.join().ok())
                            .await
                            .ok()
                            .flatten()
                    } else {
                        None
                    }
                };

                let finished = stats.map(|s| s.converged).unwrap_or(converged);

                // TODO(Task 11): verify Vi_Result type path and field name.
                executing.succeeded_with(vi_interfaces::action::Vi_Result { finished })
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
/// # API notes (TODO(Task 11))
/// - `node.create_publisher::<T>(topic, qos)` — signature matches upstream
///   node.rs but may vary; adjust if `CreatePublisher` trait or options struct
///   is required.
/// - `node.create_wall_timer(duration, callback)` — returns a guard handle.
///   The handle is discarded here (dropped at function exit). If the timer
///   requires the guard to be live, return it from this function and bind it
///   with a leading `_` in main(). See TODO(Task 11) comment inside.
/// - `node.get_clock().now().to_msg()` — unverified. Fallback: build
///   `builtin_interfaces::msg::Time` directly from `std::time::SystemTime`.
/// - `pub.publish(&msg)` — rclrs may require by-value; flip to `pub.publish(msg)`
///   if `&msg` fails to compile.
fn spawn_value_function_publisher(
    node: &Node,
    handle: &Arc<Mutex<Option<SweepHandle>>>,
    grid_meta: &GridMeta,
    threshold: u16,
) -> Result<()> {
    use nav_msgs::msg::OccupancyGrid;
    use std_msgs::msg::Header;

    let qos = QoSProfile::default().reliable().transient_local().keep_last(1);
    let pub_value = node.create_publisher::<OccupancyGrid>("value_function", qos.clone())?;
    let pub_policy = node.create_publisher::<OccupancyGrid>("policy", qos)?;

    let handle_c = Arc::clone(handle);
    let grid_meta = grid_meta.clone();
    let node_clock = node.get_clock();

    // TODO(Task 11): verify that the returned timer guard does not need to be
    // kept alive for the timer to fire. If the timer silently stops when the
    // guard is dropped, return it from this function and bind it in main() with
    // `let _vf_timer = spawn_value_function_publisher(...)`.
    node.create_wall_timer(std::time::Duration::from_secs(1), move || {
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

        let msg = OccupancyGrid {
            header: Header {
                stamp: node_clock.now().to_msg(),
                frame_id: "map".into(),
            },
            info: grid_meta.to_map_meta_data(),
            data,
        };
        // TODO(Task 11): if rclrs requires publish by value, change to
        // `pub_value.publish(msg)`.
        let _ = pub_value.publish(&msg);

        // Policy publisher placeholder — all cells -1 (unknown) until the
        // worker exposes the latest ActionTable. See spec §8 open items.
        let _ = pub_policy.publish(&OccupancyGrid {
            header: Header {
                stamp: node_clock.now().to_msg(),
                frame_id: "map".into(),
            },
            info: grid_meta.to_map_meta_data(),
            data: vec![-1i8; (grid_meta.width * grid_meta.height) as usize],
        });
    })?;

    Ok(())
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
/// # API notes (TODO(Task 11))
/// - See notes in `spawn_value_function_publisher` regarding timer guard
///   lifetime, `create_publisher` signature, and `publish` by-ref vs by-value.
fn spawn_cmd_vel_timer(
    node: &Node,
    handle: &Arc<Mutex<Option<SweepHandle>>>,
    _grid_meta: &GridMeta,
    action_list: &[(String, f64, f64)],
) -> Result<()> {
    use geometry_msgs::msg::Twist;

    let pub_cmd = node.create_publisher::<Twist>("cmd_vel", QoSProfile::default().keep_last(2))?;

    // Collect (forward_m, rotation_deg) pairs indexed by action id.
    let actions: Vec<(f64, f64)> = action_list
        .iter()
        .map(|(_, fw, rot)| (*fw, *rot))
        .collect();

    let handle_c = Arc::clone(handle);
    let period = std::time::Duration::from_millis(100);

    // TODO(Task 11): see timer guard lifetime note in spawn_value_function_publisher.
    node.create_wall_timer(period, move || {
        let h_guard = handle_c.lock().unwrap();
        let Some(h) = h_guard.as_ref() else {
            // No active sweep — publish zero velocity.
            let _ = pub_cmd.publish(&Twist::default());
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
            let _ = pub_cmd.publish(&Twist::default());
            return;
        }
        let aid = match rx.recv_timeout(std::time::Duration::from_secs(2)) {
            Ok(a) => a as usize,
            Err(_) => {
                let _ = pub_cmd.publish(&Twist::default());
                return;
            }
        };

        let (fw, rot_deg) = actions.get(aid).copied().unwrap_or((0.0, 0.0));
        let mut tw = Twist::default();
        // Convert per-period motion to instantaneous body-frame rates.
        tw.linear.x = fw / period.as_secs_f64();
        tw.angular.z = rot_deg.to_radians() / period.as_secs_f64();
        let _ = pub_cmd.publish(&tw);
    })?;

    Ok(())
}
