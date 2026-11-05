# vi_planner runtime parameters

`vi_rs/vi_planner` serves the ROS2 planning and following actions from one value
field. Parameters are declared in `src/node/params.rs`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `map_penalty_radius` | `0.2` | Map obstacle cost band radius in metres, finite and nonnegative. |
| `map_penalty` | `10000` | Map obstacle margin cost, integer from 0 to 1000000000. |
| `scan_penalty_radius` | `0.1` | Scan hit band radius in metres. |
| `scan_penalty` | `10000` | Integer scan hit cost, from 0 to 1000000000 inclusive; 0 disables hit injection. |
| `local_cpu_share` | `0.6` | Local CPU seconds per wall second; any finite positive number. |
| `global_cpu_share` | `0.2` | Global CPU seconds per wall second, in (0, 1]. |

For example, `local_cpu_share: 4.0` enables up to four helper threads, bounded by
the host's available parallelism. They calculate a synchronous Jacobi pass on
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

With global repair enabled, an increased observed traversal cost first invalidates
the stored policies that depend on it, tracing every positive-probability successor
across positions and headings. Affected actions become unavailable; calculation
and controller reads treat their values as `MAX_COST`, while the last committed
values remain in storage until repair publishes a result. Observations, goals, and unrelated
policies remain intact.
Bounded batches complete that dependency closure before local or global Bellman
work resumes, and older snapshots cannot publish across the input revision.
Decreased costs use ordinary repair. With `global_sweep: false`, local refinement
retains the existing frozen global field instead of discarding policies that no
global worker could restore.

Run `scripts/ros2_test.sh` in the existing ROS2 environment to build the release
planner through colcon and run its core tests. Run vi_lib tests from outside the
repository using `cargo test --manifest-path <repo>/vi_rs/Cargo.toml -p vi_lib`.
