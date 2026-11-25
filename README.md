# Development

## Nix shell

Run `nix-shell` from the repository root for the C compiler, Make, Cargo, Rust,
rustfmt, Clippy, rust-analyzer, pkg-config, Git, and the Docker client. The Rust
tools come from the same `<nixpkgs>` as the shell; no rustup setup is required.
Tool versions follow your configured Nixpkgs channel.

```sh
nix-shell
make test-host
make rs-test
```

ROS2 CI builds and tests the planner on Humble, Jazzy, and Lyrical. To run the
same Docker targets locally, run `make ros2-docker` followed by `make ros2-test`
(Humble by default). For Jazzy or Lyrical, pass `VI_ROS_DISTRO=jazzy` or
`VI_ROS_DISTRO=lyrical` to both commands in a separate checkout.
A running Docker daemon is required. This shell does not install native ROS2,
MATLAB, Vivado, or Vitis.

If a generated `.cargo/config.toml` from a ROS2 build is present, run host Cargo
commands from outside the repository with `--manifest-path`, as described below.

## vi_planner runtime parameters

`vi_rs/vi_planner` serves the ROS2 planning and following actions from one value
field. Parameters are declared in `src/node/params.rs`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `map_penalty_radius` | `0.2` | Map obstacle cost band radius in metres, finite and nonnegative. |
| `map_penalty` | `10000` | Map obstacle margin cost, integer from 0 to 1000000000. |
| `scan_penalty_radius` | `0.1` | Scan hit band radius in metres. |
| `scan_penalty` | `10000` | Integer scan hit cost, from 0 to 1000000000 inclusive; 0 disables hit injection. |
| `scan_collision_radius` | `0.2` | Circular footprint radius in metres for greedy following. Set it to match the robot; 0 disables the check. |
| `local_cpu_share` | `0.6` | Local CPU thread-equivalent share (CPU seconds per wall second); any finite positive number. |
| `global_cpu_share` | `0.2` | Global single-thread CPU share (CPU seconds per wall second), in (0, 1]. |
| `action_costs_s` | `1.0` for each action | Traversal cost in seconds, parallel to `action_names`. |

The complete default action table is:

```yaml
vi_planner:
  ros__parameters:
    action_names:        ["forward", "back", "right", "rightfw", "left", "leftfw"]
    action_forward_m:    [0.3, -0.2, 0.0, 0.2, 0.0, 0.2]
    action_rotation_deg: [0.0, 0.0, -20.0, -20.0, 20.0, 20.0]
    action_costs_s:      [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

All four arrays must have the same nonzero length. When `action_costs_s` is
omitted, it defaults to one second for each configured action name, including
custom action counts. Each cost must be finite, positive, at most 1,000,000
seconds, and large enough to survive quantization. Costs round down to fixed-point
units of `1 / PROB_BASE` seconds (`PROB_BASE = 2^18 = 262144`), so the minimum
accepted cost is `1 / 262144` seconds, approximately 3.815 microseconds.
The upper bound keeps an individual cost within the solvers' reachable-value
range; accumulated path costs still have that range limit.

These values describe the planning objective in seconds. They do not change
motion duration, control frequency, forward distance, rotation, or the velocities
derived from the motion table. The default equal costs preserve the existing
objective of minimizing action count.

For example, `local_cpu_share: 4.0` budgets the equivalent of four CPU threads,
and `0.6` budgets 60% of one thread. It requests `ceil(share)` helper threads
independently of the host's available parallelism; fewer work chunks may use
fewer helpers. The share is not a fraction of all host threads.
The global worker remains single-threaded: `global_cpu_share: 0.2` budgets 20%
of one thread. Local helpers calculate a synchronous Jacobi pass on
one immutable local snapshot outside the core mutex. One checked commit publishes
the pass; changes to the goal, owner, window, or scan inputs reject stale results.
The default single-thread path retains its existing in-place sweep order.
CPU pacing includes the helpers' thread CPU times and excludes lock waits.
Thread creation, snapshot copying, and small windows limit practical speedup;
the share is a CPU budget, not a guaranteed utilization or speedup.

Migrate `safety_radius` to `map_penalty_radius` and `safety_radius_penalty` to
`map_penalty`; the old ROS parameter names have no aliases. The map radius applies
to the cost band around occupied map cells, including the base band used by
localization uncertainty inflation. It does not specify a collision footprint.

Scan cost is represented exactly as `scan_penalty * 2^18`, with no power-of-two
rounding. Localization quality attenuates the configured strength by right shifts,
with a minimum of one cost unit when enabled; beam traversal still halves existing
fixed-point penalties. Compact planning stores exact theta-uniform penalties in
an 8-byte-per-map-cell overlay, including decayed values after patch recentering.

Greedy following checks the circular footprint along one second of the selected
motion against the latest scan hits projected at observation time. If it would hit an obstacle between
VI cells, the follower tries the lowest-cost unblocked action, or stops if
none is available. If a hit is already inside the footprint, an action that
moves away from it is still allowed. The check uses scan hits directly and is
independent of `scan_penalty` and `scan_quality_gate`. It does not change the
global plan, and does not apply to DWA, MPPI, or QMDP.

A changed observed traversal cost, raised or lowered, is repaired by the same
Bellman sweeps as any other change: local refinement first, then the global worker.
Policies are never discarded, so the controller keeps following the current field
while it is repaired; the scan collision check above guards the motion meanwhile.

Run `scripts/ros2_test.sh` in the existing ROS2 environment to build the release
planner through colcon and run its core tests. Run vi_lib tests from outside the
repository using `cargo test --manifest-path <repo>/vi_rs/Cargo.toml -p vi_lib`.
