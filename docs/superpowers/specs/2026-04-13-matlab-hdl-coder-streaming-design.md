# MATLAB HDL Coder Streaming Kernel Design

**Date:** 2026-04-13
**Status:** Approved
**Target:** Ultra96-V2 (Zynq UltraScale+ ZU3EG)

## Overview

Third kernel variant for the Value Iteration FPGA accelerator, implemented using MATLAB HDL Coder + SoC Blockset. Sits alongside existing Vitis HLS tile-based and streaming kernels as `fpga/matlab/`. Based on the streaming (line-buffer) kernel architecture.

### Goals

- Reproduce the streaming kernel's full functionality in Simulink/HDL Coder
- Leverage Fixed-Point Tool for optimal bit-width exploration (float-first, then convert)
- Use SoC Blockset's SoC Builder for end-to-end IP generation and bitstream flow
- Validate via HDL Verifier cosimulation before synthesis
- Integrate with existing UIO driver layer via new `vi_matlab_ops` vtable entry

### Non-Goals

- Replacing the existing HLS kernels (tile/stream remain as-is)
- Changing the host CLI or reference solver
- New algorithmic approaches (systolic array, etc.)

## Architecture

### Simulink Model Hierarchy

```
vi_sweep_stream_matlab (SoC Blockset Top)
├── AXI4-Lite Register Interface (SoC Blockset auto-generated)
│   ├── map_x, map_y, cu_id (control registers)
│   └── max_delta (status register)
├── AXI4-Master DDR Interface (SoC Blockset auto-generated)
│   ├── gmem0: value_table (R/W)
│   ├── gmem1: penalty_table + trans_table (R)
│   └── gmem2: value_table_rd (R)
└── Algorithm Subsystem (HDL Coder target)
    ├── load_transitions  — DDR → register expansion (360 entries)
    ├── stream_strip      — per-strip sliding window control
    │   ├── load_row      — DDR → line buffer
    │   ├── compute_row   — Bellman update (6-action parallel min-reduction)
    │   └── store_row     — line buffer → DDR
    └── strip_loop_ctrl   — strip partitioning & CU direction control (Stateflow)
```

### Mapping to Existing HLS Sources

| HLS (C++) | Simulink Component |
|-----------|-------------------|
| `vi_sweep_stream_top.cpp` | SoC Blockset top + strip_loop_ctrl |
| `stream_strip.cpp` | stream_strip subsystem |
| `compute_row.cpp` | compute_row subsystem |
| `load_store_row.cpp` | load_row / store_row subsystems |
| AXI pragma annotations | SoC Blockset AXI interface blocks |

### Data Flow

Same sequential processing flow as the HLS streaming kernel:

1. Load transition table from DDR into registers (360 entries)
2. Loop over X-strips (CU0: left half L→R, CU1: right half R→L)
3. Per strip: initialize 13-row window → stream all rows in Y direction
4. Per row: compute_row → store_row → load_row (next row)
5. After all strips: write max_delta to status register

## Data Type Strategy

### Phase A: Floating-Point Model (Functional Verification)

All signals use `double`. Compare MATLAB simulation output against the existing C reference (`vi_reference_c.c`) for algorithm correctness.

### Phase B: Fixed-Point Optimization

Use Fixed-Point Advisor to analyze dynamic range from testbench data and explore optimal bit widths:

| Signal | HLS Type | Exploration Range | Notes |
|--------|----------|-------------------|-------|
| value | `uint16` | 12-16 bits | Cost value, 0-65534 |
| penalty | `uint16` | 12-16 bits | OBSTACLE=0xFFFF, GOAL=0xFFFE |
| offset (dix/diy/dit) | `int8` | 6-8 bits | Max ±6 |
| cost_of intermediate | `uint17` | 17-18 bits | value + penalty addition |
| nx/ny index | `int32` | Minimum required | Buffer address computation |

### Sentinel Value Handling

`PENALTY_OBSTACLE` (0xFFFF) and `PENALTY_GOAL` (0xFFFE) must maintain their bit patterns after fixed-point conversion. The `cost_of` function's `PENALTY_GOAL → 0` substitution when read as a neighbor's penalty is implemented as an explicit if-branch in both float and fixed-point models. This logic is excluded from automatic bit-width exploration and remains hardcoded.

### Acceptance Criteria

- Phase A: floating-point output exactly matches C reference
- Phase B: fixed-point output matches floating-point with zero error (integer arithmetic, no rounding), or is bit-exact with HLS version

## Simulink Model Details

### Line Buffer Configuration

| Buffer | Shape | Simulink Block | Purpose |
|--------|-------|----------------|---------|
| `val_buf` | [13][157][60] | HDL RAM | Value storage, theta dimension fully unrolled |
| `pen_buf_0/1/2` | [13][157] | HDL RAM | Penalty 3-bank (port conflict avoidance) |

`BUF_W = STRIP_W_MAX + 2*HALO_MAX = 145 + 12 = 157`

### compute_row Subsystem

```
Input: val_buf reads (6 actions), pen_buf reads, delta_table
       ↓
┌─────────────────────────────────────────┐
│  6 parallel cost_of computations        │
│  ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐     │
│  │act0 │ │act1 │ │act2 │ ... │act5│    │
│  │fwd  │ │bwd  │ │left │    │fwdR│    │
│  └──┬──┘ └──┬──┘ └──┬──┘    └─┬──┘    │
│     └───┬───┘       └────┬────┘        │
│    min(0,1)          min(4,5)           │
│       └──────┬──────┘                   │
│         min(01,23)    min(45)           │
│            └────┬────┘                  │
│            min_cost                     │
│              ↓                          │
│   skip ? old_val : min_cost → new_val  │
│              ↓                          │
│   |new_val - old_val| → delta          │
└─────────────────────────────────────────┘
Output: new_val (write back to val_buf), row_max_delta
```

Pipelining: inner loop (theta=60) targets 1 cycle/theta using Simulink Pipeline attributes, equivalent to HLS `II=1`.

### Control FSM (Stateflow)

```
IDLE → LOAD_TRANS → STRIP_LOOP → DONE
                      ↓
              INIT_WINDOW → ROW_LOOP
                              ↓
                    COMPUTE → STORE → LOAD_NEXT → (next row or next strip)
```

### AXI Burst Configuration

Using SoC Blockset AXI4 Master Read/Write blocks:
- **Read**: `max_read_burst_length=256`, 128-bit width (8 x 16-bit values packed)
- **Write**: `max_write_burst_length=256`, same width
- Equivalent to HLS `max_widen_bitwidth=128`

## SoC Blockset Integration

### SoC Builder Workflow

```
1. Simulink model (Algorithm Subsystem)
      ↓  HDL Coder
2. Verilog/VHDL generation
      ↓  SoC Builder (IP Core Generation)
3. Vivado IP (.xci) + block design auto-generated
      ↓  SoC Builder (Build Bitstream)
4. .bit + .hwh → Ultra96-V2
```

### AXI Interface Mapping

| Port | AXI Type | Direction | Connection |
|------|----------|-----------|------------|
| `value_table` | AXI4-Master (gmem0) | R/W | DDR HP0 |
| `value_table_rd` | AXI4-Master (gmem2) | R | DDR HP2 |
| `penalty_table` | AXI4-Master (gmem1) | R | DDR HP1 |
| `trans_table` | AXI4-Master (gmem1) | R | DDR HP1 (shared) |
| Control registers | AXI4-Lite Slave | R/W | PS GP0 |

### CU Configuration

Same 2-CU configuration as HLS version:
- Model instantiated twice in SoC Builder (CU0: cu_id=0, CU1: cu_id=1)
- Each CU has independent AXI4-Master ports
- Interrupt signals generated from each CU's DONE output

### Block Design

SoC Blockset generates its own independent block design (separate from existing `create_bd.tcl`). This avoids interference with existing HLS kernel builds and aligns with the parallel development strategy.

## Verification Strategy

### Stage 1: MATLAB Simulation

| Test | Input | Expected Output | Criterion |
|------|-------|-----------------|-----------|
| Small grid (8x8x60) | Hand-crafted map (obstacle+goal) | C reference output | Exact match |
| Medium grid (32x32x60) | Random obstacle map | C reference output | Exact match |
| Strip boundary (width=300) | 2-strip split map | C reference output | Exact match |
| Sentinel test | GOAL-adjacent + OBSTACLE-surrounded | C reference output | GOAL=0 maintained |

C reference output obtained by MEX-wrapping `vi_reference_c.c` or pre-computing to file.

### Stage 2: HDL Verifier Cosimulation

- Backend: Vivado Xsim (no ModelSim license required)
- Simulink testbench drives generated HDL at cycle-accurate level
- Same test cases as Stage 1
- Verify bit-exact match after fixed-point conversion
- Measure latency and throughput in cycles

### Stage 3: Hardware Validation

- SoC Builder generates bitstream → deploy to Ultra96-V2
- Reuse existing `vi_cli --verify` flow via MATLAB ops
- Run smoke/big tests equivalent to `make test-hw`

### Test Data Sharing

- Create MATLAB script (`gen_test_map.m`) to generate test maps matching `host/test/` patterns
- Or convert existing test maps to MAT files for MATLAB consumption

## Driver Integration

### New Files

```
driver/uio/
├── vi_device_matlab.c      — MATLAB IP register map implementation
└── generated/
    └── xvi_sweep_matlab_hw.h   — Extracted from SoC Builder output
```

### Existing File Changes

- `driver/uio/vi_device.h` — Add `extern const vi_device_ops_t vi_matlab_ops;`
- `fpga/Makefile` — Add `matlab` targets

No changes to `host/`, existing HLS builds, or `create_bd.tcl`.

## Directory Structure

```
fpga/matlab/
├── model/
│   ├── vi_sweep_stream_matlab.slx      — Simulink top model (SoC Blockset)
│   ├── compute_row.slx                 — compute_row subsystem
│   ├── stream_strip.slx                — stream_strip subsystem
│   ├── load_store_row.slx              — load_row / store_row subsystems
│   └── strip_loop_ctrl.sfx             — Stateflow control FSM
├── src/
│   ├── cost_of.m                       — cost_of function (HDL Coder target)
│   ├── compute_row_algo.m              — compute_row algorithm (reference)
│   └── vi_params.m                     — Constants (N_ACTIONS, N_THETA, HALO_MAX, etc.)
├── testbench/
│   ├── tb_compute_row.m                — compute_row unit test
│   ├── tb_stream_strip.m               — stream_strip test
│   ├── tb_full_sweep.m                 — Full kernel test
│   ├── gen_test_map.m                  — Test map generator
│   └── compare_c_reference.m           — C reference comparison script
├── fixedpoint/
│   ├── fp_config.m                     — Fixed-Point Advisor configuration
│   └── fp_report/                      — Conversion report output
├── cosim/
│   ├── cosim_config.m                  — HDL Verifier cosimulation config
│   └── cosim_tb.m                      — Cosimulation testbench
├── soc/
│   ├── soc_config.m                    — SoC Builder board/interface config
│   └── build_bitstream.m               — Bitstream generation script
└── README.md                           — Setup instructions & toolbox dependencies
```

### Makefile Integration

Added to `fpga/Makefile`:

```makefile
matlab-sim:       matlab -batch "cd matlab/testbench; tb_full_sweep"
matlab-hdl:       matlab -batch "cd matlab/soc; hdl_generate"
matlab-cosim:     matlab -batch "cd matlab/cosim; cosim_tb"
matlab-bitstream: matlab -batch "cd matlab/soc; build_bitstream"
```

## Required MATLAB Toolboxes

- MATLAB (R2024b+)
- Simulink
- HDL Coder
- HDL Verifier
- Fixed-Point Designer
- SoC Blockset
- Zynq UltraScale+ support package (or custom Ultra96-V2 BSP)

## Constants Reference

Shared with HLS streaming kernel (`vi_stream_types.h`):

| Constant | Value | Description |
|----------|-------|-------------|
| `N_ACTIONS` | 6 | Forward, backward, left, right, fwd-left, fwd-right |
| `N_THETA` | 60 | Heading discretization (6 deg) |
| `HALO_MAX` | 6 | Max transition offset |
| `WINDOW_ROWS` | 13 | 2*HALO_MAX + 1 |
| `STRIP_W_MAX` | 145 | Max strip width per CU |
| `BUF_W` | 157 | STRIP_W_MAX + 2*HALO_MAX |
| `TRANS_TABLE_SIZE` | 360 | N_ACTIONS * N_THETA |
| `MAX_VALUE` | 0xFFFF | Uninitialized state sentinel |
| `PENALTY_OBSTACLE` | 0xFFFF | Impassable cell |
| `PENALTY_GOAL` | 0xFFFE | Goal cell (value pinned at 0) |
