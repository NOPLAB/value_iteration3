# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

FPGA accelerator for 3D (x, y, theta) Value Iteration path planning, targeting Ultra96-V2 (Zynq UltraScale+ ZU3EG). Goal: solve a 14000×800×60 campus map in <60 s. A Vitis HLS kernel (`vi_sweep`) is driven from Linux user-space through a UIO + u-dma-buf device layer and exercised by a C CLI plus host-side reference solver. Phase plan and design specs live in `docs/superpowers/specs/` and `docs/superpowers/plans/` — read them before making non-trivial changes to algorithms, datatypes, or memory layout.

## Build & Test

Top-level `Makefile` delegates to `driver/uio/` and `host/`. Run from the repo root unless noted.

- `make driver` — build `libvi_sweep.a` / `.so` (UIO + u-dma-buf Linux ops + mock).
- `make host` — build `host/vi_cli` linked against the Linux libvi_sweep.
- `make test-host` — build mock-only lib and run all host unit tests (`host/test/test_*.c`). No FPGA needed.
- `make test-hw` — HW integration via SSH. Requires `VI_TARGET_HOST=<ultra96-hostname>`; runs `host/test/hw/run_smoke.sh` then `run_big.sh`, which scp the CLI + generated maps to the target and execute `vi_cli --verify` there.
- Run a single host test: `make -C host test/test_penalty.run` (pattern: `test/<name>.run`).
- Host-only CLI with the mock backend (no UIO needed, useful for local debugging): `make -C host cli-mock` → `host/vi_cli_mock`.

### FPGA build (`fpga/Makefile`)

Tools must be on `PATH` — invoke bare `vitis-run` / `vivado` (Vitis 2025.2). Do **not** prefix with `source settings.sh`. Tile and streaming kernels have fully separate build paths. All TCL scripts live in `fpga/tcl/`; build artifacts go to `fpga/build/`.

- `make -C fpga csim tile` — HLS C-simulation of tile-based kernel (`fpga/hls/tile/`).
- `make -C fpga csim stream` — HLS C-simulation of streaming kernel (`fpga/hls/stream/`).
- `make -C fpga hls tile` — HLS synth + IP export (tile) into `fpga/build/hls_build_tile/`, IP to `ip_repo_tile/`.
- `make -C fpga hls stream` — HLS synth + IP export (streaming) into `fpga/build/hls_build_stream/`, IP to `ip_repo_stream/`.
- `make -C fpga bitstream tile` — HLS + Vivado synthesis + bitstream for tile kernel, project `fpga/build/vi_tile/`.
- `make -C fpga bitstream stream` — HLS + Vivado synthesis + bitstream for streaming kernel, project `fpga/build/vi_stream/`.
- `make -C fpga clean` — clean both tile and stream build artifacts. Append `tile` or `stream` to clean one.
- After regenerating HLS IP, sync the register header into the driver: `make -C driver/uio sync-hw-header KERNEL=tile` or `KERNEL=stream` (copies `xvi_sweep_hw.h` / `xvi_sweep_stream_hw.h` into `driver/uio/generated/`; review the diff).

### MATLAB kernel (`matlab/`)

Requires MATLAB R2024b+ with HDL Coder, HDL Verifier, Fixed-Point Designer, SoC Blockset.

- `make matlab-sim` — run MATLAB algorithm tests (`tb_full_sweep`).
- `make matlab-hdl` — generate/update Simulink model.
- `make matlab-cosim` — HDL Verifier cosimulation via Xsim.
- `make matlab-bitstream` — SoC Builder bitstream generation.

The MATLAB kernel is a third variant alongside tile and stream HLS kernels. Algorithm functions in `matlab/src/` mirror the streaming HLS kernel (`fpga/hls/stream/src/`). Constants in `vi_params.m` must stay synchronized with `vi_stream_types.h`.

### EDF / Petalinux (`petalinux/`)

Docker-based Yocto/EDF build for the Ultra96-V2 Linux image. Driven from the repo root:

- `make edf-docker` — build the Docker container for the EDF environment.
- `make edf-shell` — open an interactive shell in the container.
- `make edf-setup XSA=<path>` — initialize the EDF project from an XSA hardware description.
- `make edf-build MACHINE=<machine>` — run the full Yocto/EDF build.
- `make clean-edf` — clean EDF build artifacts.

## Architecture

Four vertically integrated layers share the same 16-bit data contract defined in `fpga/hls/tile/src/vi_types.h` (tile-based) and `fpga/hls/stream/src/vi_stream_types.h` (streaming). Keep them in sync.

### 1. HLS kernel (`fpga/hls/tile/` and `fpga/hls/stream/`)
Two kernel architectures share the same data contract but differ in how they sweep the grid:

- **Tile kernel** (`fpga/hls/tile/`): Dataflow pipeline `vi_sweep_top` = `load_tiles` → `compute_bellman` → `store_tiles`, processing 32×32 tiles with a 6-cell halo (TILE_W_H = 44). Two CUs are instantiated in the Vivado BD for red/black tile sweeping.
- **Streaming kernel** (`fpga/hls/stream/`): Strip-based row streaming via `vi_sweep_stream`. Processes horizontal strips using 13-row line buffers (`WINDOW_ROWS = 2*HALO_MAX+1`). Pipeline: `load_store_row` feeds rows → `stream_strip` manages the line buffer → `compute_row` does per-cell Bellman updates. Two CUs split the map vertically.

Datatypes: `value_t`/`penalty_t` are `ap_uint<16>`; offsets `ap_int<8>`. Sentinels: `PENALTY_OBSTACLE = 0xFFFF` (impassable); `PENALTY_GOAL = 0xFFFE` — **when read as a neighbor's penalty it must be treated as 0** so the goal cell's value stays pinned at 0 (this convention is load-bearing; see the testbench and `host/src/penalty.c`). Transition table is a packed `(dix, diy, dit)` word per `(action, theta)` — 6×60 = 360 entries, precomputed on ARM and DMA'd into the kernel.

### 2. Device layer (`driver/uio/`)
`vi_device.h` defines a `vi_device_ops_t` vtable (init/shutdown/read_reg/write_reg/wait_irq/map_buf) with two implementations:
- `vi_device_linux.c` — real UIO + u-dma-buf (requires the device-tree overlay in `driver/dts/vi_sweep.dtsi` applied via Petalinux on the target).
- `vi_device_mock.c` — in-memory software model used for host unit tests and `cli-mock`.

`libvi_sweep.c` sits on top of the vtable and exposes the public API (`libvi_sweep.h`). Build flavors:
- `libvi_sweep.a` / `.so` — full build, both backends.
- `libvi_sweep_mock.a` — built with `-DVI_MOCK_ONLY`, no Linux ops; used by `test-host` and `cli-mock`. Any code touching `vi_linux_ops` must be guarded by `#ifndef VI_MOCK_ONLY`.

Register offsets come from the HLS-generated `xvi_sweep_hw.h`; never hand-edit `driver/uio/generated/xvi_sweep_hw.h` — regenerate via `sync-hw-header` after an HLS rebuild.

### 3. Host CLI + reference (`host/`)
`vi_cli.c` loads a PGM map + YAML metadata (`map_pgm.c`), builds the penalty field (`penalty.c`), computes the transition table (`transitions.c`), opens the vi_sweep device, runs sweeps, and optionally verifies against `vi_reference_c.c` (pure-C value iteration matching the HLS testbench reference). `--verify` asserts bit-exact equality vs the reference; this is the oracle for HW correctness. Unit tests in `host/test/` exercise each module and a full mock-backed run (`test_vi_run_mock.c`, `test_reference_eq.c`).

### 4. FPGA/board bring-up (`fpga/vivado/`, `fpga/pynq/`)
`create_bd.tcl` / `create_project.tcl` build the Vivado block design wrapping two `vi_sweep` CUs with AXI and interrupts. `fpga/pynq/` holds bitstream + hwh + a PYNQ-side overlay helper for pre-Linux-driver experimentation on Ultra96-V2.

## Conventions

- C code: `-std=c11 -Wall -Wextra -Werror`. Keep new code warning-clean or the build breaks.
- When changing the HLS data contract (types, tile size, sentinels, transition packing), update `vi_types.h`, `host/src/vi_reference_c.c`, `host/src/penalty.c`/`transitions.c`, and the mock device in lockstep, and re-run `make test-host`.
- Goal-cell handling: the `PENALTY_GOAL` → 0 substitution when read as a neighbor's penalty is required — do not "simplify" it away.
- HW tests are SSH-driven. They assume the target already has the bitstream loaded and the `vi_sweep` overlay applied; they do not program the FPGA themselves.
