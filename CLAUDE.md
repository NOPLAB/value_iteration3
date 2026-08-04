# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

FPGA accelerator for 3D (x, y, theta) Value Iteration path planning, targeting Ultra96-V2 (Zynq UltraScale+ ZU3EG). Goal: solve a 14000×800×60 campus map in <60 s. The same VI algorithm is realized in several coordinated implementations that share a single 16-bit data contract, plus a faithful u64 reference model (`vi_rs/vi_reference`) mirroring the ROS1 original:

- **`vi_fpga/`** — the Vitis HLS kernel (`vi_sweep`), its Linux user-space driver (UIO + u-dma-buf), a C CLI, a host-side reference solver, and the Petalinux/board bring-up. This is the hardware vertical.
- **`vi_matlab/`** — a MATLAB HDL Coder variant of the streaming kernel plus algorithm experiments/benchmarks; mirrors the streaming HLS kernel.
- **`vi_rs/`** — a Rust Cargo workspace with two coordinated VI models: the 16-bit HLS data contract (`vi_core`, types only) and a faithful u64 port of the ROS1 `value_iteration` node (`vi_reference`, which also carries the word-parallel `bitboard` primitives its frontier solvers use). The fast CPU solvers (reference, frontier, block, pyramid, stream-mimic) live in `vi_reference::solvers` — u64, bit-exact with the ROS1 original — and `vi_bench` is the benchmark harness.
- **`vi_ros2/`** — ROS2 (Humble) Rust nodes: `vi_node` (Vi.action server, interface-equivalent to the ROS1 node), `vi_planner` (**one** node serving both `compute_path_to_pose` and `follow_path` from a **single** value function — replaces planner_server and controller_server together) and `vi_global_planner` (`compute_path_to_pose` only; pair it with nav2_controller's controller_server). Both carry `map_scale` and the out-of-core `frontier2d_sparse_compact` path for maps too large to solve densely. All call `vi_rs` (the u64 `vi_reference` solvers) and build via colcon in the Docker image.

Phase plan and design specs live in `docs/superpowers/specs/` and `docs/superpowers/plans/` — read them before making non-trivial changes to algorithms, datatypes, or memory layout. Specs are written in Japanese.

## Repository layout (important)

The C/HLS/driver/Petalinux code lives **under `vi_fpga/`**, not at the repo root. The root `Makefile` is a thin wrapper that delegates the software/FPGA/EDF targets into `vi_fpga/...` (and `matlab-*` / `rs-*` / `ros2-*` into their own trees), so the commands below are run from the repo root. Note: recursive `$(MAKE)` invocation fails under Windows GnuWin32 (`e=87`); run these on Linux/WSL (which is also where the Ultra96-targeted builds belong), or invoke the sub-Makefile directly (`make -C vi_fpga/host test-host`).

## Build & Test

### Software (driver + host CLI), under `vi_fpga/`

- `make driver` — build `libvi_sweep.a` / `.so` (UIO + u-dma-buf Linux ops + mock) via `vi_fpga/driver/uio/`.
- `make host` — build `vi_fpga/host/vi_cli` linked against the Linux libvi_sweep (depends on `driver`).
- `make test-host` — build the mock-only lib and run all host unit tests (`vi_fpga/host/test/test_*.c`). No FPGA needed.
- `make test-hw` — HW integration via SSH. Requires `VI_TARGET_HOST=<ultra96-hostname>`; runs `vi_fpga/host/test/hw/run_smoke.sh` then `run_big.sh`, which scp the CLI + generated maps to the target and execute `vi_cli --verify` there.
- Run a single host test: `make -C vi_fpga/host test/test_penalty.run` (pattern: `test/<name>.run`).
- Host-only CLI with the mock backend (no UIO needed, useful for local debugging): `make -C vi_fpga/host cli-mock` → `vi_fpga/host/vi_cli_mock`.

### FPGA build (`vi_fpga/Makefile`)

Tools must be on `PATH` — invoke bare `vitis-run` / `vivado` (Vitis 2025.2). Do **not** prefix with `source settings.sh`. Tile and streaming kernels have fully separate build paths. All TCL scripts live in `vi_fpga/tcl/`; build artifacts go to `vi_fpga/build/`. From the root wrapper, select the kernel with `KERNEL=tile` / `KERNEL=stream`; invoking `vi_fpga/Makefile` directly instead selects it via a `tile`/`stream` goal (`make -C vi_fpga csim stream`).

- `make csim KERNEL=stream` — HLS C-simulation of streaming kernel (`vi_fpga/hls/stream/`); `KERNEL=tile` for the tile kernel (`vi_fpga/hls/tile/`).
- `make hls KERNEL=tile` — HLS synth + IP export (tile) into `vi_fpga/build/hls_build_tile/`, IP to `ip_repo_tile/`; `KERNEL=stream` for streaming.
- `make bitstream KERNEL=tile` — HLS + Vivado synthesis + bitstream for tile kernel, project `vi_fpga/build/vi_tile/`; `KERNEL=stream` → `vi_fpga/build/vi_stream/`.
- `make clean-fpga` — clean both tile and stream build artifacts (`make -C vi_fpga clean`; append `tile`/`stream` to clean one).
- After regenerating HLS IP, sync the register header into the driver: `make sync-hw-header KERNEL=tile` or `KERNEL=stream` (copies `xvi_sweep_hw.h` / `xvi_sweep_stream_hw.h` into `vi_fpga/driver/uio/generated/`; review the diff).

### Rust workspace (`vi_rs/`)

A 3-crate Cargo workspace (`vi_core`, `vi_reference`, `vi_bench`) plus a standalone `vi_ml/` crate that is **deliberately not a workspace member** (use undefined; left untouched). The former `vi_algorithm` (its `bitboard` primitives are now `vi_reference::bitboard`) and `vi_fixtures` (orphaned synthetic u16 maps) crates were removed in the dependency-trimming pass. Driven from the repo-root Makefile (these targets are current):

- `make rs-test` — `cd vi_rs && cargo test --workspace`.
- `make rs-bench` — criterion microbenchmarks (`cargo bench -p vi_bench`) over the u64 `vi_reference` solvers (the `bitboard` microbench exercises `vi_reference::bitboard`).
- `make rs-bench-summary` — `bench_summary` CLI: a `benchmark_vi.m`-compatible macro comparison table across sizes/map-types over every u64 solver, emits CSV/markdown. Each solver is bit-exact with the ROS1 original (mismatch=0 vs the Reference oracle).
- `make rs-bench-parallel` — same; a no-op for `bench_summary` (its solver set is serial). Multithreaded u64 solvers *do* exist (`frontier2d_par`/`_par_unsafe`/`_fused`/`_sparse`, via `std::thread` — **not** rayon) but are selected directly through `bench_map`, not this flag.
- Run a single crate's tests: `cd vi_rs && cargo test -p vi_reference`.

The u64 solvers in `vi_reference` are the bit-exact regression oracle. The default solvers (Reference/Frontier/Block/Pyramid/Stream, exercised by `rs-bench`/`rs-bench-summary`) are serial; a deterministic multithreaded CPU path also exists in the `frontier2d_par*` family — `std::thread`-based, **not** rayon (the old u16 `vi_algorithm` rayon `parallel` feature was dropped with the u16 solvers, and that crate has since been folded into `vi_reference`) — and is exercised via `bench_map`.

### MATLAB kernel (`vi_matlab/`)

Requires MATLAB R2024b+ with HDL Coder, HDL Verifier, Fixed-Point Designer, SoC Blockset. Driven from the repo root:

- `make matlab-sim` — run the MATLAB `matlab.unittest` suite (`run_matlab_tests.m`).
- `make matlab-hdl` — export packaged HDL IP for the repo Vivado flow (`export_repo_ip`).
- `make matlab-cosim` — HDL Verifier cosimulation via Xsim.
- `make matlab-bitstream` — Vivado bitstream from the exported MATLAB HDL IP.
- `make matlab-bench` / `matlab-bench-codegen` — MATLAB-native and MATLAB Coder C-generation benchmarks (`benchmark_vi`). Pass `REBUILD=1` to force a clean MEX rebuild.

The MATLAB kernel is a variant alongside tile and stream HLS kernels. Algorithm functions in `vi_matlab/src/` mirror the streaming HLS kernel (`vi_fpga/hls/stream/src/`). Constants in `vi_params.m` must stay synchronized with `vi_stream_types.h`.

### ROS2 node (`vi_ros2/`)

ROS2 Humble Rust node, built via `colcon` + `cargo-ament-build` inside a Docker image. Runs the u64 `vi_reference` solvers; interface-equivalent to the ROS1 node and builds/links via colcon (the tf2-based robot-pose lookup for `cmd_vel` is still a `(0,0,0)` stub). Driven from the repo root:

- `make ros2-docker` — build the dev image (`vi_ros2/docker/Dockerfile`), tag `vi_ros2_dev:humble` (override `VI_ROS2_DOCKER_IMG`).
- `make ros2-shell` — interactive shell in the image with the repo mounted at `/workspace`.
- `make ros2-build` / `make ros2-test` — run `scripts/ros2_build.sh` / `scripts/ros2_test.sh` in the container.

Four packages (`vi_planner` and `vi_global_planner` are alternatives, never launched together):

- `vi_interfaces/` — ament_cmake package defining `action/Vi.action` only; `rosidl_generator_rs` emits Rust types for rclrs.
- `vi_node/` — the rclrs node, built on the u64 `vi_reference::ValueIterator` + `solvers::solve`. **`vi_node` is deliberately outside the `vi_rs` Cargo workspace** (its `Cargo.toml` has an explicit empty `[workspace]`) so its `path = "../../vi_rs/*"` deps don't pull it into that workspace. `rclrs`/`nav_msgs`/`vi_interfaces` are wired as `*` deps and `[patch.crates-io]`-redirected (repo `.cargo/config.toml`) to colcon-built crates; the **binary links only via colcon** (`make ros2-build`), not plain `cargo build`. Its `solver_factory` delegates to `U64Solver::from_name` (plus the legacy `"pyramid"` alias), so every u64 solver — including the sparse family — is selectable via the `solver` ROS param.
- `vi_global_planner/` — the Nav2 global-planner replacement (same crate isolation pattern as vi_node). Serves `compute_path_to_pose` (`nav2_msgs/action/ComputePathToPose`), so nav2_bt_navigator uses it transparently **instead of nav2_planner's planner_server** (launch nav2 without planner_server and drop it from the lifecycle manager list; vi_global_planner is not a lifecycle node — rclrs has none). Per goal it builds a `ValueIterator` from `/map`, solves with the `solver` param (default `frontier2d_sparse`; `vi_threads` sets `VI_THREADS`), caches the solved value function keyed by goal (`goal_tolerance_xy`/`goal_tolerance_deg`), and answers replans to the same goal with a rollout only (`vi_reference::planner::rollout_path` + `densify`). Robot pose comes from a `PoseWithCovarianceStamped` topic (`pose_topic`, default `mcl_pose` for emcl2; AMCL: `amcl_pose`) because rclrs has no tf2. A new goal preempts an in-flight solve via a cancel flag observed every `solve_chunk` iterations (fused/sparse solvers write back partial state on early exit, so chunked re-entry keeps progress). Publishes `value_function` (θ=0 slice) after each fresh solve **and during the solve** every `value_publish_interval_ms` (default 500 ms, 0 = final only) — the chunked write_back means RViz shows the wavefront expanding from the goal. The rclrs-free core (`src/core.rs`: `PlannerCore` — cache decision, cancelable chunked solve, rollout) is host-testable. Ships two launch files: `launch/vi_global_planner.launch.py` (node only) and `launch/navigation_launch.py` — a robot-agnostic derivative of nav2_bringup's `navigation_launch.py` that brings up the full Nav2 navigation stack with vi_global_planner in place of planner_server (drop-in include for any Nav2 robot; daifuku_autonomous includes it when `planner:=vi`).
- `vi_planner/` — **the unified planner: one node, one value function, both Nav2 actions.** Serves `compute_path_to_pose` *and* `follow_path`, replacing planner_server and controller_server simultaneously (same crate isolation pattern). This is the successor to the old `vi_global_planner` + `vi_local_planner` **pair**, which ran in two processes and therefore solved the *same* value function for the *same* goal twice — doubling both time-to-first-motion and resident memory — while the `nav_msgs/Path` the global side produced was discarded by the local side except for its final pose. Here a goal is solved once (`PlannerCore::prepare_goal_with_progress`), `compute_path_to_pose` is a greedy rollout of that solved field (`rollout_path_on` + `densify` — exactly the old cache-hit path), and `follow_path` refines the *same* field inside the ±1m window: move window → inject `local_penalty` from `scan_topic` LaserScans → re-iterate under `refine_budget_ms` → greedy action to `cmd_vel` (delta_fw [m] → linear.x, delta_rot [deg] → angular.z, 本家 `ViNode::decision` 準拠; remap to `cmd_vel_nav` in a Nav2 bringup). Goal detection is the VI `final_state` itself; a policy-less pose borrows the nearest neighbor's action within `action_tolerance`. **Lock discipline matters**: the two actions share one `Mutex<PlannerCore>`, so the follow loop takes the lock *per tick* (10 Hz, 40 ms budget) rather than for the whole follow — otherwise the BT's 1 Hz replan would block until the goal is reached. The follow loop also re-checks `is_cached_goal` every tick, so if a plan request for a different goal replaces the cache it preempts instead of driving on a stale policy. RViz: `value_function` (θ=0 slice, solve progress every `value_publish_interval_ms` + once on convergence) and `local_window_value` (the clamped ±1m window at the robot's current-heading θ slice). There is no `local_value_function` topic — there is only one value function now. Supports `map_scale`/`downsample_policy` and the out-of-core `frontier2d_sparse_compact` solver (`compact_sink_dir`, plus `compact_ram_limit_mb` which spills the sink to `/tmp/vi_planner_sink` when it would not fit in RAM — costlier here than in vi_global_planner, since the follow loop re-reads the sink on every patch recenter; `src/sink.rs` — a copy of vi_global_planner's `MmapSink`). The follow loop needs a dense `ValueIterator::states`, which a compact solve never allocates, so with the compact solver it hydrates a **patch** around the robot instead of the whole map: `±1m window + transition reach + slack` cells (27×27×60 ≈ 2.5 MB at 0.25 m/cell), filled from the sink (values/policy) and the static grid (`free`/`penalty`, evaluated in *global* coordinates so `State::from_occupancy`'s row-crossing bug matches what the solve saw). Everything outside the window stays frozen at the compact values and serves as the boundary condition; the invariant "the window's transitions land inside the patch" is checked at startup against the measured transition table (`transition_reach`), and the patch is re-centered when the robot nears its edge. Two deliberate differences from the dense path: `compute_path_to_pose` rolls out over the pristine sink (so scan penalties do not reach the published path — following still avoids obstacles via the patch), and re-centering discards that tick's local refinement (the next scan restores it). **Local → global feedback (`global_sweep`, on by default, dense only):** because both actions read the same `states`, what the follow loop learns from the laser *is* in the global field — but `refine_pass_until` only sweeps the ±1m window, so the raised values never propagate and a rollout descending from 20 m away keeps aiming into a blocked corridor (this is the entry point for `LoopDetected` and the BT's recovery loop). `PlannerCore::sweep_global` closes it by Gauss–Seidel sweeping the whole shared field, reusing `value_iteration_at` so local/global/solve stay on one update rule (no new Bellman code; nothing changed in vi_rs). It is chunked through a caller-held `SweepCursor` and must **not** run a whole sweep under the lock — the 10 Hz follow loop `try_lock`s the same mutex and stops the robot after 3 consecutive misses; `global_sweep_budget_ms`:`global_sweep_idle_ms` (default 20:60) is the CPU share. The only trigger is a `dirty` flag set by `refine_for`; it clears when a full sweep yields Δ=0. Measured: the far field moves on sweep **1**, ~converged by 30, Δ=0 at ~80; sweep throughput is 5.23 M cells/s on the host (1.5 s per sweep for 19F at map_scale 2). Under compact it is a no-op (no `states`), so the node warns and the daifuku launch refuses compact + `global_sweep`. `local_penalty` outside the window is never cleared (本家 `ViNode` behaviour, deliberately kept) — a penalty injected while passing an obstacle distorts the global field for the rest of that goal. Dense costs 80 B/state (`states` 56 + `sweep_orders` 24; measured 654.8 MB for 19F at map_scale 2) and `dense_limit_mb` refuses to start above the limit rather than being OOM-killed. The rclrs-free core (`src/core/`: `mod.rs` = the public types, `PlannerCore` and the dense path; `compact.rs` = the out-of-core machinery only — sink, follow patch, `PenaltyOverlay`, tile `Repair`, and the `PlannerCore` methods that only run there; `tests.rs` = both paths, one module) is host-testable. Launch: `launch/vi_planner.launch.py` (node only).
- Which of the two to run is `local_planner` in `vi_global_planner/launch/navigation_launch.py`, and they are **mutually exclusive** — launching both would put two servers on `compute_path_to_pose`. `local_planner:=vi` (default) = `vi_planner` alone; `local_planner:=nav2` = `vi_global_planner` + nav2_controller's controller_server. Both handle `map_scale`/compact, so wide maps (map_tsudanuma) can use either.
- The rclrs-free libraries (vi_node: `bridge`/`npy`/`solver_factory`/`sweep_thread` + the `oracle` equivalence tests; vi_global_planner: `core`; vi_planner: `core`) run via `cargo test --lib` **inside the Docker image**; on the host they are checkable via a scratch isolation crate that `#[path]`-includes those modules (the repo `.cargo/config.toml` ROS patches block a plain host build). A plain `cargo test --test ...` does NOT work — it forces cargo to build the rclrs binary, which only links under colcon (so those tests live in the libraries, not `tests/`). The shared ROS-view conversions live in `vi_reference::bridge` (moved from vi_node); `vi_node::bridge` is now a re-export.
- `nav2_msgs` Rust bindings for vi_global_planner are produced in the **Docker image's** `/ros2_rust_ws`: the Dockerfile clones `navigation2` (humble) and colcon-builds `nav2_msgs` there after rclrs, so `rosidl_generator_rs` emits the Rust crate that the generated cargo config patches in (requires `ros-humble-nav2-common`, installed in the image).

The external ROS interface is **interface-equivalent** to the ROS1 `value_iteration` catkin package (action name `vi_controller`, `/map` in, `value_function`/`policy`/`cmd_vel` out) but uses ROS2-native message types. See `docs/superpowers/specs/2026-05-29-vi-ros2-design.md`.

### EDF / Petalinux (`vi_fpga/petalinux/`)

Docker-based Yocto/EDF build for the Ultra96-V2 Linux image (delegates to `vi_fpga/petalinux/`):

- `make edf-docker` — build the Docker container for the EDF environment.
- `make edf-shell` — open an interactive shell in the container.
- `make edf-setup XSA=<path>` — initialize the EDF project from an XSA hardware description.
- `make edf-build MACHINE=<machine>` — run the full Yocto/EDF build.
- `make clean-edf` — clean EDF build artifacts.

## Architecture

The HLS hardware vertical (`vi_fpga/`) has four integrated layers sharing the same 16-bit data contract defined in `vi_fpga/hls/tile/src/vi_types.h` (tile) and `vi_fpga/hls/stream/src/vi_stream_types.h` (streaming). The MATLAB (`vi_matlab/`) port replicates that same contract; on the Rust side only the type widths remain (`vi_rs/vi_core/src/types.rs`), the constants having moved to `vi_bench::params` as benchmark values. Keep them all in sync.

Datatypes: `value_t`/`penalty_t` are `ap_uint<16>`; offsets `ap_int<8>`. Sentinels: `PENALTY_OBSTACLE = 0xFFFF` (impassable); `PENALTY_GOAL = 0xFFFE` — **when read as a neighbor's penalty it must be treated as 0** so the goal cell's value stays pinned at 0 (this convention is load-bearing; see the testbench and `vi_fpga/host/src/penalty.c`. The Rust u64 model in `vi_reference` pins goal cells its own way in `value_iterator.rs` (`set_goal` / `set_state_values`); it no longer mirrors the 16-bit sentinel since `vi_core/src/goal.rs` was removed). Transition table is a packed `(dix, diy, dit)` word per `(action, theta)` — 6×60 = 360 entries, precomputed on ARM and DMA'd into the kernel.

### 1. HLS kernel (`vi_fpga/hls/tile/` and `vi_fpga/hls/stream/`)
Two kernel architectures share the data contract but differ in how they sweep the grid:

- **Tile kernel** (`vi_fpga/hls/tile/`): Dataflow pipeline `vi_sweep_top` = `load_tiles` → `compute_bellman` → `store_tiles`, processing 32×32 tiles with a 6-cell halo (TILE_W_H = 44). Two CUs are instantiated in the Vivado BD for red/black tile sweeping.
- **Streaming kernel** (`vi_fpga/hls/stream/`): Strip-based row streaming via `vi_sweep_stream`. Processes horizontal strips using 13-row line buffers (`WINDOW_ROWS = 2*HALO_MAX+1`). Pipeline: `load_store_row` feeds rows → `stream_strip` manages the line buffer → `compute_row` does per-cell Bellman updates. Two CUs split the map vertically.

### 2. Device layer (`vi_fpga/driver/uio/`)
`vi_device.h` defines a `vi_device_ops_t` vtable (init/shutdown/read_reg/write_reg/wait_irq/map_buf) with two implementations:
- `vi_device_linux.c` — real UIO + u-dma-buf (requires the device-tree overlay in `vi_fpga/driver/dts/vi_sweep.dtsi` applied via Petalinux on the target).
- `vi_device_mock.c` — in-memory software model used for host unit tests and `cli-mock`.

`libvi_sweep.c` sits on top of the vtable and exposes the public API (`libvi_sweep.h`). Build flavors:
- `libvi_sweep.a` / `.so` — full build, both backends.
- `libvi_sweep_mock.a` — built with `-DVI_MOCK_ONLY`, no Linux ops; used by `test-host` and `cli-mock`. Any code touching `vi_linux_ops` must be guarded by `#ifndef VI_MOCK_ONLY`.

Register offsets come from the HLS-generated `xvi_sweep_hw.h`; never hand-edit `vi_fpga/driver/uio/generated/xvi_sweep_hw.h` — regenerate via `sync-hw-header` after an HLS rebuild.

### 3. Host CLI + reference (`vi_fpga/host/`)
`vi_cli.c` loads a PGM map + YAML metadata (`map_pgm.c`), builds the penalty field (`penalty.c`), computes the transition table (`transitions.c`), opens the vi_sweep device, runs sweeps, and optionally verifies against `vi_reference_c.c` (pure-C value iteration matching the HLS testbench reference). `--verify` asserts bit-exact equality vs the reference; this is the oracle for HW correctness. Unit tests in `vi_fpga/host/test/` exercise each module and a full mock-backed run (`test_vi_run_mock.c`, `test_reference_eq.c`).

### 4. FPGA/board bring-up (`vi_fpga/tcl/`, `vi_fpga/vivado/`, `vi_fpga/pynq/`)
`create_bd_*.tcl` / `create_project_*.tcl` (in `vi_fpga/tcl/`) build the Vivado block design wrapping two `vi_sweep` CUs with AXI and interrupts. `vi_fpga/pynq/` holds bitstream + hwh + a PYNQ-side overlay helper for pre-Linux-driver experimentation on Ultra96-V2.

### 5. Rust algorithm port (`vi_rs/`)
- `vi_core` — the immutable 16-bit data contract, now down to `types` alone (`Value`/`Penalty`/`Offset`/`ThetaIdx`/`ActionIdx`). **Nothing in Rust references it any more**; it is kept as the record of the HLS type widths. The algorithm constants (`params`: `N_THETA`/`ACTION_FW`/`ACTION_ROT`/the `PENALTY_*` sentinels) were removed: their only consumer was the three `vi_ros2` nodes checking that incoming ROS params equalled them, which just blocked launch files from changing values the solver takes at run time anyway. The same numbers now live in `vi_bench::params` as the benchmark baseline. The old u16 contract *logic* (`cost_of`, packed↔unpacked `transitions`, `make_goal_mask`/`goal`) was removed earlier with its only consumer, `vi_fixtures` — the u64 model reimplements all of it in `vi_reference`.
- `vi_reference` — a faithful u64 port of the ROS1 `value_iteration` node (`ValueIterator` / `State` / `Action`, PROB_BASE = 2^18 fixed point), reproducing its quirks (including the original's int-division and margin-penalty bugs). `solvers::solve(&mut ValueIterator, U64Solver, max_iter)` dispatches 18 `U64Solver` variants. The 10 canonical solvers — `Reference`, `Frontier2D/3D{,Tau,TopK,CoarseTheta}`, `FrontierStack`, `BlockRefine`, `PyramidSweep`, `StreamMimic` — are the set `bench_summary` gates as bit-exact; the other 8 are performance-experiment variants (`Frontier2D` SoA/Pad/Par/ParUnsafe/Fused/Sparse — the Par/ParUnsafe/Fused/Sparse ones are `std::thread`-parallel) plus two priority-queue solvers (`PriorityLabelSetting`/`Correcting`), exercised only via `bench_map` / `vi_prio_measure` / external `vi_compare` scripts. Each applies the original per-cell Bellman update over an active set, so the converged value of reachable cells is bit-exact with the ROS1 original (proven by the `solvers::test_support` parity tests). This is the active CPU model. Design: `docs/superpowers/specs/2026-06-08-vi-reference-faithful-port-design.md`, `2026-06-09-vi-u64-fast-solvers-design.md`.
- `vi_reference::bitboard` — the value-type-agnostic `bitboard` primitives (3-D θ-periodic dilation, 2-D AND/OR, enumerate, ndarray conv) that `vi_reference`'s frontier solvers reuse and the `vi_bench` `bitboard` microbench exercises. Formerly the standalone `vi_algorithm` crate; folded in during the dependency-trimming pass (the u16 `Solver`/`VIContext` family that used to live alongside it was ported to `vi_reference` and removed earlier).
- `vi_bench` — criterion benches + the `bench_summary` / `bench_map` CLIs, all over the u64 `vi_reference` solvers; the `bitboard` microbench exercises `vi_reference::bitboard`.

## Conventions

- C code: `-std=c11 -Wall -Wextra -Werror`. Keep new code warning-clean or the build breaks.
- When changing the HLS data contract (types, tile size, sentinels, transition packing), update **in lockstep**: `vi_fpga/hls/tile/src/vi_types.h`, `vi_fpga/hls/stream/src/vi_stream_types.h`, `vi_fpga/host/src/vi_reference_c.c`, `vi_fpga/host/src/penalty.c`/`transitions.c`, the mock device, `vi_matlab/.../vi_params.m`, and `vi_rs/vi_core` (`types.rs`; the constants moved to `vi_bench/src/params.rs`, and the u64 model's matching logic lives in `vi_reference`). Then re-run `make -C vi_fpga/host test-host` and `make rs-test`.
- Goal-cell handling: the `PENALTY_GOAL` → 0 substitution when read as a neighbor's penalty is required across all implementations — do not "simplify" it away.
- HW tests are SSH-driven. They assume the target already has the bitstream loaded and the `vi_sweep` overlay applied; they do not program the FPGA themselves.
