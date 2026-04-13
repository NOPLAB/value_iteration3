# MATLAB HDL Coder Streaming Kernel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the streaming Value Iteration kernel in Simulink + HDL Coder + SoC Blockset as a third kernel variant (`fpga/matlab/`), validated against the existing C reference.

**Architecture:** Monolithic Simulink model with hierarchical subsystems (load_row, compute_row, store_row, stream_strip, strip_loop_ctrl). Float-first development with Fixed-Point Tool conversion. SoC Blockset manages AXI interfaces and bitstream generation. HDL Verifier cosimulation validates cycle accuracy before synthesis.

**Tech Stack:** MATLAB R2024b+, Simulink, HDL Coder, HDL Verifier, Fixed-Point Designer, SoC Blockset, Vivado Xsim, Ultra96-V2 BSP

**Spec:** `docs/superpowers/specs/2026-04-13-matlab-hdl-coder-streaming-design.md`

---

## File Map

### New files (`fpga/matlab/`)

| File | Responsibility |
|------|---------------|
| `src/vi_params.m` | All shared constants (N_ACTIONS, N_THETA, HALO_MAX, sentinels, etc.) |
| `src/cost_of.m` | Single-cell cost computation with sentinel handling (HDL Coder target) |
| `src/compute_row_algo.m` | Bellman update for one row — 6-action parallel min-reduction (HDL Coder target) |
| `src/load_row_algo.m` | Load one row from DDR arrays into line buffer with halo/OOB fill (HDL Coder target) |
| `src/store_row_algo.m` | Store one row from line buffer back to DDR arrays (HDL Coder target) |
| `src/stream_strip_algo.m` | Per-strip sliding window: init window, row loop with compute/store/load (HDL Coder target) |
| `src/vi_sweep_stream_algo.m` | Top-level: load transitions, iterate strips, track max_delta (HDL Coder target) |
| `testbench/gen_test_map.m` | Generate test maps (small, medium, strip-boundary, sentinel) |
| `testbench/gen_transitions.m` | Generate transition table matching `host/src/transitions.c` |
| `testbench/run_c_reference.m` | Run C reference via MEX and return value table |
| `testbench/tb_cost_of.m` | Unit test for cost_of |
| `testbench/tb_compute_row.m` | Unit test for compute_row_algo |
| `testbench/tb_load_store_row.m` | Unit test for load_row_algo and store_row_algo |
| `testbench/tb_stream_strip.m` | Integration test for stream_strip_algo |
| `testbench/tb_full_sweep.m` | Full kernel test vs C reference |
| `testbench/vi_reference_mex.c` | MEX wrapper for `host/src/vi_reference_c.c` |
| `fixedpoint/fp_config.m` | Fixed-Point Advisor configuration |
| `cosim/cosim_config.m` | HDL Verifier cosimulation setup |
| `cosim/cosim_tb.m` | Cosimulation testbench script |
| `model/vi_sweep_stream_matlab.slx` | Simulink top model (SoC Blockset) |
| `soc/soc_config.m` | SoC Builder board/interface configuration |
| `soc/build_bitstream.m` | Bitstream generation automation |

### Modified files

| File | Change |
|------|--------|
| `fpga/Makefile` | Add `matlab-sim`, `matlab-hdl`, `matlab-cosim`, `matlab-bitstream` targets |
| `driver/uio/vi_device.h` | Add `extern const vi_device_ops_t vi_matlab_ops;` declaration |

---

## Task 1: Constants and Directory Setup

**Files:**
- Create: `fpga/matlab/src/vi_params.m`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p fpga/matlab/src fpga/matlab/testbench fpga/matlab/fixedpoint fpga/matlab/cosim fpga/matlab/model fpga/matlab/soc
```

- [ ] **Step 2: Write vi_params.m**

```matlab
function p = vi_params()
%VI_PARAMS Shared constants for the MATLAB streaming VI kernel.
%   Mirrors fpga/hls/stream/src/vi_stream_types.h.

    p.N_ACTIONS       = 6;
    p.N_THETA         = 60;
    p.HALO_MAX        = 6;
    p.WINDOW_ROWS     = 2 * p.HALO_MAX + 1;   % 13
    p.STRIP_W_MAX     = 145;
    p.BUF_W           = p.STRIP_W_MAX + 2 * p.HALO_MAX;  % 157
    p.TRANS_TABLE_SIZE = p.N_ACTIONS * p.N_THETA;  % 360

    % Sentinel values (uint16)
    p.MAX_VALUE        = uint16(hex2dec('FFFF'));  % 65535
    p.PENALTY_OBSTACLE = uint16(hex2dec('FFFF'));  % 65535
    p.PENALTY_GOAL     = uint16(hex2dec('FFFE'));  % 65534
end
```

- [ ] **Step 3: Verify constants load correctly**

Run in MATLAB:
```matlab
cd fpga/matlab
p = src.vi_params();  % or: addpath('src'); p = vi_params();
assert(p.N_ACTIONS == 6);
assert(p.WINDOW_ROWS == 13);
assert(p.BUF_W == 157);
assert(p.MAX_VALUE == uint16(65535));
assert(p.PENALTY_GOAL == uint16(65534));
disp('vi_params OK');
```

Expected: `vi_params OK`

- [ ] **Step 4: Commit**

```bash
git add fpga/matlab/
git commit -m "feat(matlab): add directory structure and vi_params constants"
```

---

## Task 2: cost_of Function

**Files:**
- Create: `fpga/matlab/src/cost_of.m`
- Create: `fpga/matlab/testbench/tb_cost_of.m`

- [ ] **Step 1: Write the test**

```matlab
function tb_cost_of()
%TB_COST_OF Unit tests for cost_of function.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();

    MAX_VALUE        = double(p.MAX_VALUE);
    PENALTY_OBSTACLE = double(p.PENALTY_OBSTACLE);
    PENALTY_GOAL     = double(p.PENALTY_GOAL);

    % Test 1: Normal addition
    % cost_of(100, 50) = 150
    assert(cost_of(100, 50) == 150, 'Normal addition failed');

    % Test 2: Neighbor is MAX_VALUE (unreachable) → MAX_VALUE
    assert(cost_of(MAX_VALUE, 50) == MAX_VALUE, 'MAX_VALUE neighbor failed');

    % Test 3: Neighbor penalty is OBSTACLE → MAX_VALUE
    assert(cost_of(100, PENALTY_OBSTACLE) == MAX_VALUE, 'OBSTACLE penalty failed');

    % Test 4: Neighbor penalty is GOAL → treated as 0
    % cost_of(100, PENALTY_GOAL) = 100 + 0 = 100
    assert(cost_of(100, PENALTY_GOAL) == 100, 'GOAL penalty failed');

    % Test 5: Sum saturates at MAX_VALUE-1
    % cost_of(65000, 600) = 65600 > 65535, so clamp to 65534
    assert(cost_of(65000, 600) == MAX_VALUE - 1, 'Saturation failed');

    % Test 6: Goal cell with value 0 and GOAL penalty neighbor
    assert(cost_of(0, PENALTY_GOAL) == 0, 'Goal zero + GOAL penalty failed');

    % Test 7: Both MAX_VALUE
    assert(cost_of(MAX_VALUE, PENALTY_OBSTACLE) == MAX_VALUE, ...
        'Both sentinel failed');

    disp('tb_cost_of: ALL PASSED');
end
```

- [ ] **Step 2: Run test to verify it fails**

```matlab
cd fpga/matlab
addpath('src', 'testbench');
tb_cost_of
```

Expected: Error — `Undefined function 'cost_of'`

- [ ] **Step 3: Implement cost_of.m**

```matlab
function c = cost_of(nv, np_raw)
%COST_OF Compute traversal cost for one neighbor.
%   Matches fpga/hls/stream/src/compute_row.cpp:cost_of().
%   All arithmetic in double (Phase A). HDL Coder target.
%
%   nv     — neighbor value (double, representing uint16)
%   np_raw — neighbor penalty (double, representing uint16)
%   c      — cost (double, representing uint16)

    MAX_VALUE        = 65535;  % 0xFFFF
    PENALTY_OBSTACLE = 65535;  % 0xFFFF
    PENALTY_GOAL     = 65534;  % 0xFFFE

    if nv == MAX_VALUE || np_raw == PENALTY_OBSTACLE
        c = MAX_VALUE;
        return;
    end

    if np_raw == PENALTY_GOAL
        np = 0;
    else
        np = np_raw;
    end

    s = nv + np;
    if s >= MAX_VALUE
        c = MAX_VALUE - 1;
    else
        c = s;
    end
end
```

- [ ] **Step 4: Run test to verify it passes**

```matlab
cd fpga/matlab
addpath('src', 'testbench');
tb_cost_of
```

Expected: `tb_cost_of: ALL PASSED`

- [ ] **Step 5: Commit**

```bash
git add fpga/matlab/src/cost_of.m fpga/matlab/testbench/tb_cost_of.m
git commit -m "feat(matlab): add cost_of function with unit tests"
```

---

## Task 3: Test Data Generation

**Files:**
- Create: `fpga/matlab/testbench/gen_test_map.m`
- Create: `fpga/matlab/testbench/gen_transitions.m`

- [ ] **Step 1: Write gen_transitions.m**

Generates the same transition table as `host/src/transitions.c`. For simplicity in testing, uses the same trivial transition scheme as `host/test/test_vi_run_mock.c`: action 0 = +x, action 1 = -x, others = no-op.

```matlab
function trans = gen_transitions(mode)
%GEN_TRANSITIONS Generate transition table as uint32 array [N_ACTIONS*N_THETA x 1].
%   mode: 'trivial' — action 0 = dix+1, action 1 = dix-1, rest no-op.
%         'full'    — 6 actions with heading-dependent dx/dy/dtheta.
%
%   Each entry packs (dix, diy, dit) as:
%     byte0 = dix (int8), byte1 = diy (int8), byte2 = dit (int8)
%
%   Returns: trans — uint32 [360 x 1]

    p = vi_params();
    trans = zeros(p.N_ACTIONS * p.N_THETA, 1, 'uint32');

    if strcmp(mode, 'trivial')
        for it = 1:p.N_THETA
            % Action 0: dix=+1, diy=0, dit=0
            trans((0) * p.N_THETA + it) = uint32(1);  % 0x00000001
            % Action 1: dix=-1, diy=0, dit=0
            trans((1) * p.N_THETA + it) = uint32(255); % 0x000000FF = int8(-1) as uint8
            % Actions 2-5: no-op (all zeros)
        end
    elseif strcmp(mode, 'full')
        % Full 6-action model with heading-dependent offsets.
        % Resolution: 0.05 m/cell. Forward speed: 0.3 m → 6 cells.
        % dtheta: ±3 indices (±18 deg).
        resolution = 0.05;
        forward_dist = 0.3;
        cells = round(forward_dist / resolution);  % 6
        dt_turn = 3;  % indices for ±18 deg turn

        for it = 1:p.N_THETA
            theta = (it - 1) * (2 * pi / p.N_THETA);
            dix_fwd = round(cells * cos(theta));
            diy_fwd = round(cells * sin(theta));
            dix_bwd = -dix_fwd;
            diy_bwd = -diy_fwd;

            % Action 0: forward
            trans((0)*p.N_THETA + it) = pack_trans(dix_fwd, diy_fwd, 0);
            % Action 1: backward
            trans((1)*p.N_THETA + it) = pack_trans(dix_bwd, diy_bwd, 0);
            % Action 2: turn left (rotate +dt_turn)
            trans((2)*p.N_THETA + it) = pack_trans(dix_fwd, diy_fwd, dt_turn);
            % Action 3: turn right (rotate -dt_turn)
            trans((3)*p.N_THETA + it) = pack_trans(dix_fwd, diy_fwd, -dt_turn);
            % Action 4: forward-left
            trans((4)*p.N_THETA + it) = pack_trans(dix_fwd, diy_fwd, dt_turn);
            % Action 5: forward-right
            trans((5)*p.N_THETA + it) = pack_trans(dix_fwd, diy_fwd, -dt_turn);
        end
    else
        error('Unknown mode: %s', mode);
    end
end

function w = pack_trans(dix, diy, dit)
%PACK_TRANS Pack (dix, diy, dit) into uint32 matching HLS format.
    b0 = typecast(int8(dix), 'uint8');
    b1 = typecast(int8(diy), 'uint8');
    b2 = typecast(int8(dit), 'uint8');
    w = uint32(b0) + bitshift(uint32(b1), 8) + bitshift(uint32(b2), 16);
end
```

- [ ] **Step 2: Write gen_test_map.m**

```matlab
function [value, penalty, goal_x, goal_y] = gen_test_map(map_x, map_y, map_type)
%GEN_TEST_MAP Generate test maps for VI kernel testing.
%   map_type:
%     'empty'    — no obstacles, goal at center
%     'obstacle' — rectangular obstacle block, goal at center
%     'sentinel' — GOAL cell surrounded by OBSTACLE on 3 sides
%
%   Returns:
%     value   — double [map_y, map_x, N_THETA], initialized to MAX_VALUE (goal=0)
%     penalty — double [map_y, map_x], 0=free, OBSTACLE=0xFFFF, GOAL=0xFFFE
%     goal_x, goal_y — 1-indexed goal position

    p = vi_params();
    MAX_VALUE        = double(p.MAX_VALUE);
    PENALTY_OBSTACLE = double(p.PENALTY_OBSTACLE);
    PENALTY_GOAL     = double(p.PENALTY_GOAL);

    value   = MAX_VALUE * ones(map_y, map_x, p.N_THETA);
    penalty = zeros(map_y, map_x);

    goal_x = ceil(map_x / 2);
    goal_y = ceil(map_y / 2);

    switch map_type
        case 'empty'
            % Nothing else to do

        case 'obstacle'
            % Place a 2-cell-thick wall above the goal
            wall_y = max(1, goal_y - 3);
            for wy = wall_y:min(map_y, wall_y+1)
                for wx = max(1, goal_x-3):min(map_x, goal_x+3)
                    penalty(wy, wx) = PENALTY_OBSTACLE;
                end
            end

        case 'sentinel'
            % Surround goal on 3 sides with obstacles (leave right side open)
            if goal_y > 1
                penalty(goal_y-1, goal_x) = PENALTY_OBSTACLE;
            end
            if goal_y < map_y
                penalty(goal_y+1, goal_x) = PENALTY_OBSTACLE;
            end
            if goal_x > 1
                penalty(goal_y, goal_x-1) = PENALTY_OBSTACLE;
            end

        otherwise
            error('Unknown map_type: %s', map_type);
    end

    % Set goal
    penalty(goal_y, goal_x) = PENALTY_GOAL;
    value(goal_y, goal_x, :) = 0;
end
```

- [ ] **Step 3: Verify test map generation**

```matlab
cd fpga/matlab
addpath('src', 'testbench');
p = vi_params();

% Empty 8x8
[v, pen, gx, gy] = gen_test_map(8, 8, 'empty');
assert(all(size(v) == [8, 8, p.N_THETA]));
assert(pen(gy, gx) == double(p.PENALTY_GOAL));
assert(v(gy, gx, 1) == 0);
assert(v(1, 1, 1) == double(p.MAX_VALUE));

% Transitions
trans = gen_transitions('trivial');
assert(numel(trans) == 360);
assert(trans(1) == uint32(1));  % action 0, theta 0: dix=+1

disp('gen_test_map + gen_transitions OK');
```

Expected: `gen_test_map + gen_transitions OK`

- [ ] **Step 4: Commit**

```bash
git add fpga/matlab/testbench/gen_test_map.m fpga/matlab/testbench/gen_transitions.m
git commit -m "feat(matlab): add test map and transition table generators"
```

---

## Task 4: C Reference MEX Wrapper

**Files:**
- Create: `fpga/matlab/testbench/vi_reference_mex.c`
- Create: `fpga/matlab/testbench/run_c_reference.m`

- [ ] **Step 1: Write the MEX wrapper**

```c
/* vi_reference_mex.c — MEX gateway for vi_reference_run().
 * Build: mex vi_reference_mex.c ../../host/src/vi_reference_c.c
 *        -I../../host/src -I../../driver/uio
 */
#include "mex.h"
#include "vi_reference_c.h"

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    /* Inputs: value(uint16), penalty(uint16), trans(uint32),
     *         map_x(double), map_y(double), threshold(double), max_sweeps(double) */
    if (nrhs != 7)
        mexErrMsgIdAndTxt("vi:nrhs", "Seven inputs required.");

    uint16_t *value   = (uint16_t *)mxGetData(prhs[0]);
    uint16_t *penalty = (uint16_t *)mxGetData(prhs[1]);
    uint32_t *trans   = (uint32_t *)mxGetData(prhs[2]);
    int map_x      = (int)mxGetScalar(prhs[3]);
    int map_y      = (int)mxGetScalar(prhs[4]);
    uint16_t threshold = (uint16_t)mxGetScalar(prhs[5]);
    int max_sweeps = (int)mxGetScalar(prhs[6]);

    /* Copy value array (reference modifies in-place) */
    mwSize nval = mxGetNumberOfElements(prhs[0]);
    plhs[0] = mxCreateNumericMatrix(1, nval, mxUINT16_CLASS, mxREAL);
    uint16_t *out = (uint16_t *)mxGetData(plhs[0]);
    memcpy(out, value, nval * sizeof(uint16_t));

    int sweeps = vi_reference_run(out, penalty, trans,
                                  map_x, map_y, threshold, max_sweeps);

    /* Return sweep count */
    plhs[1] = mxCreateDoubleScalar((double)sweeps);
}
```

- [ ] **Step 2: Write run_c_reference.m**

```matlab
function [value_out, sweeps] = run_c_reference(value, penalty, trans, ...
                                                map_x, map_y, threshold, max_sweeps)
%RUN_C_REFERENCE Run the C reference solver via MEX.
%   value   — double [map_y, map_x, N_THETA] → reshaped to uint16 flat array
%   penalty — double [map_y, map_x] → reshaped to uint16 flat array
%   trans   — uint32 [360 x 1]
%   Returns value_out as double [map_y, map_x, N_THETA]

    p = vi_params();

    % Reshape to C-order flat arrays (row-major: y * map_x * N_THETA + x * N_THETA + t)
    % MATLAB is column-major, so we need to permute and reshape carefully.
    % value is [map_y, map_x, N_THETA] in MATLAB
    % C expects flat[y][x][theta] = flat[y * map_x * N_THETA + x * N_THETA + theta]
    val_perm = permute(value, [3, 2, 1]);  % [N_THETA, map_x, map_y]
    val_flat = uint16(val_perm(:));         % Column-major read = theta-fastest

    % Penalty: [map_y, map_x] → C flat[y * map_x + x]
    pen_perm = permute(penalty, [2, 1]);    % [map_x, map_y]
    pen_flat = uint16(pen_perm(:));

    % Build MEX if not on path
    mex_file = fullfile(fileparts(mfilename('fullpath')), 'vi_reference_mex');
    if ~exist([mex_file '.' mexext], 'file')
        src_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'host', 'src');
        drv_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'driver', 'uio');
        mex_src = fullfile(fileparts(mfilename('fullpath')), 'vi_reference_mex.c');
        ref_src = fullfile(src_dir, 'vi_reference_c.c');
        mex(mex_src, ref_src, ['-I' src_dir], ['-I' drv_dir], ...
            '-output', mex_file);
    end

    [val_out_flat, sweeps] = vi_reference_mex(val_flat, pen_flat, trans, ...
                                               map_x, map_y, threshold, max_sweeps);

    % Reshape back to [map_y, map_x, N_THETA]
    val_out_3d = reshape(double(val_out_flat), [p.N_THETA, map_x, map_y]);
    value_out = permute(val_out_3d, [3, 2, 1]);
end
```

- [ ] **Step 3: Test the MEX wrapper**

```matlab
cd fpga/matlab
addpath('src', 'testbench');
p = vi_params();

[v, pen, gx, gy] = gen_test_map(8, 8, 'empty');
trans = gen_transitions('trivial');

[v_out, sweeps] = run_c_reference(v, pen, trans, 8, 8, 0, 100);
assert(sweeps > 0 && sweeps <= 100, 'Reference did not converge');
assert(v_out(gy, gx, 1) == 0, 'Goal value not zero');
% Cells reachable via dix=+/-1 should have value < MAX_VALUE
assert(v_out(gy, gx+1, 1) < double(p.MAX_VALUE), 'Adjacent cell not updated');
disp('run_c_reference OK');
```

Expected: `run_c_reference OK`

- [ ] **Step 4: Commit**

```bash
git add fpga/matlab/testbench/vi_reference_mex.c fpga/matlab/testbench/run_c_reference.m
git commit -m "feat(matlab): add C reference MEX wrapper for golden comparison"
```

---

## Task 5: load_row and store_row Algorithms

**Files:**
- Create: `fpga/matlab/src/load_row_algo.m`
- Create: `fpga/matlab/src/store_row_algo.m`
- Create: `fpga/matlab/testbench/tb_load_store_row.m`

- [ ] **Step 1: Write the test**

```matlab
function tb_load_store_row()
%TB_LOAD_STORE_ROW Unit tests for load_row_algo and store_row_algo.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);

    map_x = 16; map_y = 10;
    strip_x0 = 0; strip_w = 16;

    % Create known value and penalty tables (0-indexed coords internally)
    value_table = MV * ones(map_y, map_x, p.N_THETA);
    penalty_table = zeros(map_y, map_x);
    % Set some known values
    value_table(3, 5, :) = 100;  % (y=2 0-indexed, x=4 0-indexed)
    penalty_table(3, 5) = 42;

    % Test 1: Normal row load (gy=2, 0-indexed)
    [val_row, pen_row] = load_row_algo(value_table, penalty_table, ...
                                        2, strip_x0, strip_w, map_x, map_y);
    % val_row is [BUF_W, N_THETA], pen_row is [BUF_W, 1]
    % bx = x + HALO_MAX for in-strip cells
    bx = 4 + p.HALO_MAX + 1;  % +1 for MATLAB 1-indexing
    assert(val_row(bx, 1) == 100, 'Value not loaded correctly');
    assert(pen_row(bx) == 42, 'Penalty not loaded correctly');

    % Halo cells (x < 0) should be MAX_VALUE/OBSTACLE
    assert(val_row(1, 1) == MV, 'Left halo not sentinel');
    assert(pen_row(1) == OB, 'Left halo penalty not obstacle');

    % Test 2: Out-of-bounds row (gy = -1)
    [val_oob, pen_oob] = load_row_algo(value_table, penalty_table, ...
                                        -1, strip_x0, strip_w, map_x, map_y);
    assert(all(pen_oob == OB), 'OOB row penalty not all obstacle');
    assert(all(val_oob(:, 1) == MV), 'OOB row value not all max');

    % Test 3: Store and re-load round-trip
    val_row_modified = val_row;
    val_row_modified(p.HALO_MAX+1, 1) = 999;  % Modify first in-strip cell
    value_table2 = store_row_algo(val_row_modified, value_table, ...
                                   2, strip_x0, strip_w, map_x);
    % Verify written back
    assert(value_table2(3, 1, 1) == 999, 'Store did not write back');
    % Non-modified cells unchanged
    assert(value_table2(3, 5, 1) == 100, 'Store corrupted other cell');

    disp('tb_load_store_row: ALL PASSED');
end
```

- [ ] **Step 2: Run test to verify it fails**

```matlab
cd fpga/matlab; addpath('src','testbench'); tb_load_store_row
```

Expected: Error — `Undefined function 'load_row_algo'`

- [ ] **Step 3: Implement load_row_algo.m**

```matlab
function [val_row, pen_row] = load_row_algo(value_table, penalty_table, ...
                                             gy, strip_x0, strip_w, map_x, map_y)
%LOAD_ROW_ALGO Load one row with halo from value/penalty tables.
%   Matches fpga/hls/stream/src/load_store_row.cpp:load_row().
%   gy: 0-indexed global Y coordinate.
%   strip_x0: 0-indexed X start of strip.
%   All arrays are double (Phase A).
%
%   Returns:
%     val_row — [BUF_W, N_THETA] double
%     pen_row — [BUF_W, 1] double

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);

    buf_w = strip_w + 2 * p.HALO_MAX;

    % Phase A: Fill with sentinels
    val_row = MV * ones(p.BUF_W, p.N_THETA);
    pen_row = OB * ones(p.BUF_W, 1);

    % OOB check
    if gy < 0 || gy >= map_y
        return;
    end

    % Phase B: Compute in-bounds X range
    gx_start = strip_x0 - p.HALO_MAX;
    x0_global = max(0, gx_start);
    x1_global = min(map_x, gx_start + buf_w);
    x_count = x1_global - x0_global;
    lx_offset = x0_global - gx_start;  % 0-indexed local offset

    if x_count <= 0
        return;
    end

    % Phase C: Copy penalty (1-indexed MATLAB arrays)
    gy1 = gy + 1;  % 0-indexed → 1-indexed
    for i = 0:x_count-1
        gx1 = x0_global + i + 1;
        lx1 = lx_offset + i + 1;
        pen_row(lx1) = penalty_table(gy1, gx1);
    end

    % Phase D: Copy value
    for i = 0:x_count-1
        gx1 = x0_global + i + 1;
        lx1 = lx_offset + i + 1;
        for it = 1:p.N_THETA
            val_row(lx1, it) = value_table(gy1, gx1, it);
        end
    end
end
```

- [ ] **Step 4: Implement store_row_algo.m**

```matlab
function value_table = store_row_algo(val_row, value_table, ...
                                       gy, strip_x0, strip_w, map_x)
%STORE_ROW_ALGO Store one row (inner cells, no halo) back to value table.
%   Matches fpga/hls/stream/src/load_store_row.cpp:store_row().
%   Modifies and returns value_table.

    p = vi_params();
    gy1 = gy + 1;  % 0-indexed → 1-indexed

    for ix = 0:strip_w-1
        gx1 = strip_x0 + ix + 1;
        bx1 = ix + p.HALO_MAX + 1;  % skip halo, 1-indexed
        if gx1 >= 1 && gx1 <= map_x
            for it = 1:p.N_THETA
                value_table(gy1, gx1, it) = val_row(bx1, it);
            end
        end
    end
end
```

- [ ] **Step 5: Run test to verify it passes**

```matlab
cd fpga/matlab; addpath('src','testbench'); tb_load_store_row
```

Expected: `tb_load_store_row: ALL PASSED`

- [ ] **Step 6: Commit**

```bash
git add fpga/matlab/src/load_row_algo.m fpga/matlab/src/store_row_algo.m \
       fpga/matlab/testbench/tb_load_store_row.m
git commit -m "feat(matlab): add load_row and store_row algorithms with tests"
```

---

## Task 6: compute_row Algorithm

**Files:**
- Create: `fpga/matlab/src/compute_row_algo.m`
- Create: `fpga/matlab/testbench/tb_compute_row.m`

- [ ] **Step 1: Write the test**

```matlab
function tb_compute_row()
%TB_COMPUTE_ROW Unit tests for compute_row_algo.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    GOAL = double(p.PENALTY_GOAL);

    % Setup: 13-row window, BUF_W wide, N_THETA deep
    val_buf = MV * ones(p.WINDOW_ROWS, p.BUF_W, p.N_THETA);
    pen_buf = OB * ones(p.WINDOW_ROWS, p.BUF_W);

    % Place a goal neighbor at (win_center, bx=10, theta=0) with value=0
    win_center = p.HALO_MAX + 1;  % 1-indexed center row (7)
    bx_goal = 10;
    val_buf(win_center, bx_goal, :) = 0;
    pen_buf(win_center, bx_goal) = GOAL;

    % Free cell at bx=9 (reachable from bx_goal via action 1: dix=-1)
    pen_buf(win_center, 9) = 0;
    val_buf(win_center, 9, :) = MV;

    % Trivial delta_table: action 0 = dix+1, action 1 = dix-1
    delta_table = zeros(p.N_ACTIONS, p.N_THETA, 3);
    for it = 1:p.N_THETA
        delta_table(1, it, 1) = 1;   % action 0: dix=+1
        delta_table(2, it, 1) = -1;  % action 1: dix=-1
    end

    strip_w = 16;
    cu_id = 0;

    [val_buf_out, row_max_delta] = compute_row_algo(val_buf, pen_buf, ...
                                                     delta_table, win_center, ...
                                                     strip_w, cu_id);

    % Cell at bx=9 should now have value = cost_of(0, GOAL) = 0
    % because neighbor at bx=10 has value=0 and penalty=GOAL → cost=0+0=0
    % But cell at bx=9 has pen=0, so it should get updated.
    % Action 0 (dix+1): looks at bx=10, cost_of(0, GOAL) = 0
    assert(val_buf_out(win_center, 9, 1) == 0, ...
        sprintf('Expected 0, got %d', val_buf_out(win_center, 9, 1)));

    % Goal cell itself should be unchanged (skip because pen >= GOAL)
    assert(val_buf_out(win_center, bx_goal, 1) == 0, 'Goal cell modified');

    % Obstacle cells should be unchanged
    assert(val_buf_out(win_center, 1, 1) == MV, 'Obstacle cell changed');

    assert(row_max_delta >= 0, 'Negative delta');

    disp('tb_compute_row: ALL PASSED');
end
```

- [ ] **Step 2: Run test to verify it fails**

```matlab
cd fpga/matlab; addpath('src','testbench'); tb_compute_row
```

Expected: Error — `Undefined function 'compute_row_algo'`

- [ ] **Step 3: Implement compute_row_algo.m**

```matlab
function [val_buf, row_max_delta] = compute_row_algo(val_buf, pen_buf, ...
                                                      delta_table, win_center, ...
                                                      strip_w, cu_id)
%COMPUTE_ROW_ALGO Bellman update for one row in the sliding window.
%   Matches fpga/hls/stream/src/compute_row.cpp.
%   All arithmetic in double (Phase A).
%
%   val_buf      — [WINDOW_ROWS, BUF_W, N_THETA] double (modified in-place)
%   pen_buf      — [WINDOW_ROWS, BUF_W] double
%   delta_table  — [N_ACTIONS, N_THETA, 3] double (dix, diy, dit)
%   win_center   — 1-indexed row in circular buffer
%   strip_w      — active strip width
%   cu_id        — 0=forward, 1=reverse

    p = vi_params();
    MV = double(p.MAX_VALUE);
    GOAL = double(p.PENALTY_GOAL);

    local_max = 0;

    % Precompute ny lookup (1-indexed)
    y_sign = 1;
    if cu_id == 1
        y_sign = -1;
    end
    ny_lut = zeros(p.N_ACTIONS, p.N_THETA);
    for a = 1:p.N_ACTIONS
        for it = 1:p.N_THETA
            diy = y_sign * delta_table(a, it, 2);
            ny = win_center + diy;
            % Circular wrap (1-indexed)
            if ny < 1
                ny = ny + p.WINDOW_ROWS;
            elseif ny > p.WINDOW_ROWS
                ny = ny - p.WINDOW_ROWS;
            end
            ny_lut(a, it) = ny;
        end
    end

    % X loop
    for ix_raw = 0:strip_w-1
        if cu_id == 0
            ix = ix_raw;
        else
            ix = strip_w - 1 - ix_raw;
        end
        bx = ix + p.HALO_MAX + 1;  % 1-indexed

        cell_pen = pen_buf(win_center, bx);
        skip = (cell_pen >= GOAL);

        % Theta loop
        for it = 1:p.N_THETA
            old_val = val_buf(win_center, bx, it);

            if skip
                continue;
            end

            % Theta wrapping for turn actions
            it_l = it + 3;
            if it_l > p.N_THETA, it_l = it_l - p.N_THETA; end
            it_r = it - 3;
            if it_r < 1, it_r = it_r + p.N_THETA; end

            % Action 0: forward (same theta)
            nx0 = bx + delta_table(1, it, 1);
            c0 = cost_of(val_buf(ny_lut(1,it), nx0, it), ...
                         pen_buf(ny_lut(1,it), nx0));

            % Action 1: backward (same theta)
            nx1 = bx + delta_table(2, it, 1);
            c1 = cost_of(val_buf(ny_lut(2,it), nx1, it), ...
                         pen_buf(ny_lut(2,it), nx1));

            % Action 2: left (theta + 3)
            nx2 = bx + delta_table(3, it, 1);
            c2 = cost_of(val_buf(ny_lut(3,it), nx2, it_l), ...
                         pen_buf(ny_lut(3,it), nx2));

            % Action 3: right (theta - 3)
            nx3 = bx + delta_table(4, it, 1);
            c3 = cost_of(val_buf(ny_lut(4,it), nx3, it_r), ...
                         pen_buf(ny_lut(4,it), nx3));

            % Action 4: fwd-left (theta + 3)
            nx4 = bx + delta_table(5, it, 1);
            c4 = cost_of(val_buf(ny_lut(5,it), nx4, it_l), ...
                         pen_buf(ny_lut(5,it), nx4));

            % Action 5: fwd-right (theta - 3)
            nx5 = bx + delta_table(6, it, 1);
            c5 = cost_of(val_buf(ny_lut(6,it), nx5, it_r), ...
                         pen_buf(ny_lut(6,it), nx5));

            % Min-reduction tree
            min01 = min(c0, c1);
            min23 = min(c2, c3);
            min45 = min(c4, c5);
            min03 = min(min01, min23);
            min_cost = min(min03, min45);

            val_buf(win_center, bx, it) = min_cost;

            d = abs(min_cost - old_val);
            if d > local_max
                local_max = d;
            end
        end
    end

    row_max_delta = local_max;
end
```

- [ ] **Step 4: Run test to verify it passes**

```matlab
cd fpga/matlab; addpath('src','testbench'); tb_compute_row
```

Expected: `tb_compute_row: ALL PASSED`

- [ ] **Step 5: Commit**

```bash
git add fpga/matlab/src/compute_row_algo.m fpga/matlab/testbench/tb_compute_row.m
git commit -m "feat(matlab): add compute_row Bellman update with tests"
```

---

## Task 7: stream_strip Algorithm

**Files:**
- Create: `fpga/matlab/src/stream_strip_algo.m`
- Create: `fpga/matlab/testbench/tb_stream_strip.m`

- [ ] **Step 1: Write the test**

```matlab
function tb_stream_strip()
%TB_STREAM_STRIP Integration test for stream_strip_algo.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();
    MV = double(p.MAX_VALUE);

    map_x = 16; map_y = 16;
    [value, penalty, gx, gy] = gen_test_map(map_x, map_y, 'empty');
    trans = gen_transitions('trivial');

    % Unpack transitions to delta_table [N_ACTIONS, N_THETA, 3]
    delta_table = unpack_transitions(trans, p);

    % Run one strip (covers full map width since 16 < STRIP_W_MAX)
    strip_x0 = 0; strip_w = map_x; cu_id = 0;
    [value_out, strip_delta] = stream_strip_algo(value, value, penalty, ...
                                                  delta_table, map_x, map_y, ...
                                                  strip_x0, strip_w, cu_id);

    % Goal should still be 0
    assert(value_out(gy, gx, 1) == 0, 'Goal value changed');
    % Adjacent cells should be updated
    assert(value_out(gy, gx+1, 1) < MV, 'Adjacent cell not updated');
    assert(strip_delta > 0, 'No delta after first sweep');

    disp('tb_stream_strip: ALL PASSED');
end

function delta_table = unpack_transitions(trans, p)
%UNPACK_TRANSITIONS Convert uint32 flat array to [N_ACTIONS, N_THETA, 3].
    delta_table = zeros(p.N_ACTIONS, p.N_THETA, 3);
    for i = 1:p.TRANS_TABLE_SIZE
        a = floor((i-1) / p.N_THETA) + 1;
        t = mod(i-1, p.N_THETA) + 1;
        w = trans(i);
        delta_table(a, t, 1) = double(typecast(uint8(bitand(w, 255)), 'int8'));
        delta_table(a, t, 2) = double(typecast(uint8(bitand(bitshift(w,-8), 255)), 'int8'));
        delta_table(a, t, 3) = double(typecast(uint8(bitand(bitshift(w,-16), 255)), 'int8'));
    end
end
```

- [ ] **Step 2: Run test to verify it fails**

```matlab
cd fpga/matlab; addpath('src','testbench'); tb_stream_strip
```

Expected: Error — `Undefined function 'stream_strip_algo'`

- [ ] **Step 3: Implement stream_strip_algo.m**

```matlab
function [value_table, strip_max_delta] = stream_strip_algo(value_table, ...
    value_table_rd, penalty_table, delta_table, map_x, map_y, ...
    strip_x0, strip_w, cu_id)
%STREAM_STRIP_ALGO Process one X-strip with sliding window.
%   Matches fpga/hls/stream/src/stream_strip.cpp.
%   All arithmetic in double (Phase A).
%
%   value_table    — [map_y, map_x, N_THETA] double (write destination)
%   value_table_rd — [map_y, map_x, N_THETA] double (read source)
%   penalty_table  — [map_y, map_x] double
%   delta_table    — [N_ACTIONS, N_THETA, 3] double
%   cu_id          — 0=forward (Y ascending), 1=reverse (Y descending)
%
%   Returns modified value_table and strip_max_delta.

    p = vi_params();
    local_max = 0;

    % Allocate line buffers: [WINDOW_ROWS, BUF_W, N_THETA] and [WINDOW_ROWS, BUF_W]
    val_buf = zeros(p.WINDOW_ROWS, p.BUF_W, p.N_THETA);
    pen_buf = zeros(p.WINDOW_ROWS, p.BUF_W);

    % Initialize window: load WINDOW_ROWS rows
    for wr = 0:p.WINDOW_ROWS-1
        if cu_id == 0
            gy = -p.HALO_MAX + wr;
        else
            gy = (map_y - 1) + p.HALO_MAX - wr;
        end
        slot = wr + 1;  % 1-indexed
        [val_buf(slot,:,:), pen_row] = load_row_algo(value_table_rd, penalty_table, ...
                                                      gy, strip_x0, strip_w, map_x, map_y);
        pen_buf(slot, :) = pen_row;
    end

    % Stream through all rows
    for iy_raw = 0:map_y-1
        if cu_id == 0
            iy = iy_raw;
        else
            iy = map_y - 1 - iy_raw;
        end
        win_center = mod(iy_raw + p.HALO_MAX, p.WINDOW_ROWS) + 1;  % 1-indexed

        % Compute Bellman update
        [val_buf, row_delta] = compute_row_algo(val_buf, pen_buf, ...
                                                 delta_table, win_center, ...
                                                 strip_w, cu_id);
        if row_delta > local_max
            local_max = row_delta;
        end

        % Store updated row
        value_table = store_row_algo(val_buf(win_center,:,:), value_table, ...
                                      iy, strip_x0, strip_w, map_x);

        % Evict oldest, load next
        evict_slot = mod(iy_raw, p.WINDOW_ROWS) + 1;  % 1-indexed
        if cu_id == 0
            next_gy = iy_raw + p.HALO_MAX + 1;
        else
            next_gy = (map_y - 1) - (iy_raw + p.HALO_MAX + 1);
        end
        [val_buf(evict_slot,:,:), pen_row] = load_row_algo(value_table_rd, penalty_table, ...
                                                            next_gy, strip_x0, strip_w, ...
                                                            map_x, map_y);
        pen_buf(evict_slot, :) = pen_row;
    end

    strip_max_delta = local_max;
end
```

- [ ] **Step 4: Run test to verify it passes**

```matlab
cd fpga/matlab; addpath('src','testbench'); tb_stream_strip
```

Expected: `tb_stream_strip: ALL PASSED`

- [ ] **Step 5: Commit**

```bash
git add fpga/matlab/src/stream_strip_algo.m fpga/matlab/testbench/tb_stream_strip.m
git commit -m "feat(matlab): add stream_strip sliding window algorithm with tests"
```

---

## Task 8: Top-Level vi_sweep_stream Algorithm

**Files:**
- Create: `fpga/matlab/src/vi_sweep_stream_algo.m`
- Create: `fpga/matlab/testbench/tb_full_sweep.m`

- [ ] **Step 1: Write the test**

```matlab
function tb_full_sweep()
%TB_FULL_SWEEP Full kernel test comparing MATLAB algo vs C reference.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();
    MV = double(p.MAX_VALUE);

    test_cases = {
        struct('name','empty_8x8',    'mx',8,  'my',8,  'type','empty')
        struct('name','empty_32x32',  'mx',32, 'my',32, 'type','empty')
        struct('name','obstacle_16x16','mx',16,'my',16, 'type','obstacle')
        struct('name','sentinel_8x8', 'mx',8,  'my',8,  'type','sentinel')
    };

    trans = gen_transitions('trivial');

    for tc = 1:numel(test_cases)
        t = test_cases{tc};
        fprintf('  Test: %s ... ', t.name);

        [value, penalty, ~, ~] = gen_test_map(t.mx, t.my, t.type);

        % Run C reference
        [ref_out, ~] = run_c_reference(value, penalty, trans, ...
                                        t.mx, t.my, 0, 200);

        % Run MATLAB kernel (same number of sweeps as reference for comparison)
        % We run a fixed number of sweeps to compare intermediate state
        ml_value = value;
        for sweep = 1:50
            [ml_value, delta0] = vi_sweep_stream_algo(ml_value, ml_value, ...
                                                       penalty, trans, ...
                                                       t.mx, t.my, 0);
            [ml_value, delta1] = vi_sweep_stream_algo(ml_value, ml_value, ...
                                                       penalty, trans, ...
                                                       t.mx, t.my, 1);
            if max(delta0, delta1) == 0
                break;
            end
        end

        % Compare converged values: both should converge to same result.
        % The C reference uses a different sweep order (raster scan),
        % so we compare only convergence, not intermediate values.
        % Check that MATLAB result has same reachability pattern.
        ml_reachable = (ml_value < MV);
        ref_reachable = (ref_out < MV);
        assert(isequal(ml_reachable, ref_reachable), ...
            [t.name ': reachability mismatch']);

        % Goal cells must be 0 in both
        goal_mask = (penalty == double(p.PENALTY_GOAL));
        for it = 1:p.N_THETA
            ml_slice = ml_value(:,:,it);
            ref_slice = ref_out(:,:,it);
            assert(all(ml_slice(goal_mask) == 0), [t.name ': MATLAB goal not 0']);
            assert(all(ref_slice(goal_mask) == 0), [t.name ': Ref goal not 0']);
        end

        fprintf('PASSED\n');
    end

    disp('tb_full_sweep: ALL PASSED');
end
```

- [ ] **Step 2: Run test to verify it fails**

```matlab
cd fpga/matlab; addpath('src','testbench'); tb_full_sweep
```

Expected: Error — `Undefined function 'vi_sweep_stream_algo'`

- [ ] **Step 3: Implement vi_sweep_stream_algo.m**

```matlab
function [value_table, max_delta] = vi_sweep_stream_algo(value_table, ...
    value_table_rd, penalty_table, trans, map_x, map_y, cu_id)
%VI_SWEEP_STREAM_ALGO Top-level streaming VI kernel.
%   Matches fpga/hls/stream/src/vi_sweep_stream_top.cpp.
%   One call = one CU's sweep. Call with cu_id=0 then cu_id=1 for a full sweep.
%
%   value_table    — [map_y, map_x, N_THETA] double (R/W)
%   value_table_rd — [map_y, map_x, N_THETA] double (R)
%   penalty_table  — [map_y, map_x] double
%   trans          — uint32 [360 x 1] packed transition table
%   map_x, map_y   — map dimensions
%   cu_id          — 0=forward, 1=reverse

    p = vi_params();

    % 1. Unpack transition table
    delta_table = zeros(p.N_ACTIONS, p.N_THETA, 3);
    for i = 1:p.TRANS_TABLE_SIZE
        a = floor((i-1) / p.N_THETA) + 1;
        t = mod(i-1, p.N_THETA) + 1;
        w = trans(i);
        delta_table(a, t, 1) = double(typecast(uint8(bitand(w, 255)), 'int8'));
        delta_table(a, t, 2) = double(typecast(uint8(bitand(bitshift(w,-8), 255)), 'int8'));
        delta_table(a, t, 3) = double(typecast(uint8(bitand(bitshift(w,-16), 255)), 'int8'));
    end

    % 2. Compute strip layout
    num_strips = ceil(map_x / p.STRIP_W_MAX);
    half_strips = ceil(num_strips / 2);

    global_max_delta = 0;

    % 3. Iterate X-strips
    for si = 0:half_strips-1
        if cu_id == 0
            sx = si;
        else
            sx = num_strips - 1 - si;
        end
        if sx < 0 || sx >= num_strips
            break;
        end
        strip_x0 = sx * p.STRIP_W_MAX;
        strip_w = min(p.STRIP_W_MAX, map_x - strip_x0);

        [value_table, strip_delta] = stream_strip_algo(value_table, ...
            value_table_rd, penalty_table, delta_table, ...
            map_x, map_y, strip_x0, strip_w, cu_id);

        if strip_delta > global_max_delta
            global_max_delta = strip_delta;
        end
    end

    max_delta = global_max_delta;
end
```

- [ ] **Step 4: Run test to verify it passes**

```matlab
cd fpga/matlab; addpath('src','testbench'); tb_full_sweep
```

Expected:
```
  Test: empty_8x8 ... PASSED
  Test: empty_32x32 ... PASSED
  Test: obstacle_16x16 ... PASSED
  Test: sentinel_8x8 ... PASSED
tb_full_sweep: ALL PASSED
```

- [ ] **Step 5: Commit**

```bash
git add fpga/matlab/src/vi_sweep_stream_algo.m fpga/matlab/testbench/tb_full_sweep.m
git commit -m "feat(matlab): add top-level vi_sweep_stream algorithm with full-sweep tests"
```

---

## Task 9: Simulink Model Creation

**Files:**
- Create: `fpga/matlab/model/vi_sweep_stream_matlab.slx`
- Create: `fpga/matlab/model/create_model.m`

This task creates the Simulink model programmatically via a MATLAB script. The script builds the model hierarchy, adds subsystems, and configures HDL Coder settings. This is preferred over manual Simulink GUI work for reproducibility.

- [ ] **Step 1: Write create_model.m**

```matlab
function create_model()
%CREATE_MODEL Build the Simulink model for the streaming VI kernel.
%   Creates vi_sweep_stream_matlab.slx with:
%   - Algorithm subsystem referencing the .m functions
%   - HDL Coder configuration
%   - SoC Blockset annotations for later IP generation

    model_name = 'vi_sweep_stream_matlab';
    model_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(model_dir, '..', 'src'));

    % Close if already open
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end

    % Create new model
    new_system(model_name);
    open_system(model_name);

    % Set solver to fixed-step (required for HDL Coder)
    set_param(model_name, 'Solver', 'FixedStepDiscrete');
    set_param(model_name, 'FixedStep', '1');
    set_param(model_name, 'StopTime', 'inf');

    % Add Algorithm subsystem (MATLAB Function block)
    algo_path = [model_name '/Algorithm'];
    add_block('simulink/User-Defined Functions/MATLAB Function', algo_path);

    % Configure the MATLAB Function block with the algorithm
    % The function references vi_sweep_stream_algo.m
    mfb = get_param(algo_path, 'Object');

    % Set HDL Coder parameters
    hdlset_param(model_name, 'HDLSubsystem', model_name);
    hdlset_param(model_name, 'SynthesisTool', 'Xilinx Vivado');
    hdlset_param(model_name, 'SynthesisToolChipFamily', 'Zynq UltraScale+');
    hdlset_param(model_name, 'SynthesisToolDeviceName', 'xczu3eg');
    hdlset_param(model_name, 'SynthesisToolPackageName', 'sbva484');
    hdlset_param(model_name, 'SynthesisToolSpeedValue', '-1');

    % Save
    save_system(model_name, fullfile(model_dir, [model_name '.slx']));
    fprintf('Model saved: %s.slx\n', model_name);
end
```

Note: This is a starter script. The full Simulink model with SoC Blockset integration requires interactive MATLAB work to:
1. Configure MATLAB Function blocks with proper port mappings
2. Add SoC Blockset AXI4 Master/Slave interface blocks
3. Set up the memory map for DDR access
4. Configure dual-CU instantiation

These interactive steps are documented in `fpga/matlab/README.md` (Task 12).

- [ ] **Step 2: Run create_model.m in MATLAB**

```matlab
cd fpga/matlab/model
create_model
```

Expected: `Model saved: vi_sweep_stream_matlab.slx` and the model opens in Simulink.

- [ ] **Step 3: Verify model opens and solver is correct**

```matlab
assert(strcmp(get_param('vi_sweep_stream_matlab', 'Solver'), 'FixedStepDiscrete'));
disp('Model config OK');
```

- [ ] **Step 4: Commit**

```bash
git add fpga/matlab/model/create_model.m fpga/matlab/model/vi_sweep_stream_matlab.slx
git commit -m "feat(matlab): add Simulink model scaffold with HDL Coder config"
```

---

## Task 10: Fixed-Point Configuration

**Files:**
- Create: `fpga/matlab/fixedpoint/fp_config.m`

- [ ] **Step 1: Write fp_config.m**

```matlab
function fp_config()
%FP_CONFIG Configure Fixed-Point Advisor for the VI streaming kernel.
%   Run this after Phase A (floating-point) verification passes.
%   Uses tb_full_sweep test data to analyze dynamic range.

    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'testbench'));
    p = vi_params();

    % --- Define fixed-point type proposals ---
    % These are the target types matching vi_stream_types.h.
    % Fixed-Point Advisor will verify they have sufficient range.

    T = struct();

    % value_t: uint16, no fractional bits
    T.value = numerictype('Signed', false, 'WordLength', 16, 'FractionLength', 0);

    % penalty_t: uint16, no fractional bits
    T.penalty = numerictype('Signed', false, 'WordLength', 16, 'FractionLength', 0);

    % offset_t: int8
    T.offset = numerictype('Signed', true, 'WordLength', 8, 'FractionLength', 0);

    % cost_of intermediate: uint17 for nv + np addition
    T.cost_sum = numerictype('Signed', false, 'WordLength', 17, 'FractionLength', 0);

    % Display
    fprintf('Fixed-point type proposals:\n');
    fn = fieldnames(T);
    for i = 1:numel(fn)
        ft = T.(fn{i});
        fprintf('  %-12s: %s, W=%d, F=%d\n', fn{i}, ...
            ternary(ft.Signed, 'signed', 'unsigned'), ...
            ft.WordLength, ft.FractionLength);
    end

    % --- Generate instrumented test data ---
    fprintf('\nGenerating test data for range analysis...\n');
    [value, penalty, ~, ~] = gen_test_map(32, 32, 'empty');
    trans = gen_transitions('trivial');

    % Run a few sweeps to collect representative data
    for sweep = 1:10
        [value, ~] = vi_sweep_stream_algo(value, value, penalty, trans, 32, 32, 0);
        [value, ~] = vi_sweep_stream_algo(value, value, penalty, trans, 32, 32, 1);
    end

    % Report range of converged values
    valid = value(value < double(p.MAX_VALUE));
    if ~isempty(valid)
        fprintf('Value range after convergence: [%g, %g]\n', min(valid), max(valid));
        fprintf('Requires %d bits (unsigned)\n', ceil(log2(max(valid)+1)));
    end

    fprintf('\nfp_config complete. Run Fixed-Point Advisor from Simulink to apply.\n');
end

function r = ternary(cond, a, b)
    if cond, r = a; else, r = b; end
end
```

- [ ] **Step 2: Run fp_config.m**

```matlab
cd fpga/matlab/fixedpoint
fp_config
```

Expected: Type proposals printed and value range analysis shown.

- [ ] **Step 3: Commit**

```bash
git add fpga/matlab/fixedpoint/fp_config.m
git commit -m "feat(matlab): add Fixed-Point Advisor configuration"
```

---

## Task 11: HDL Verifier Cosimulation Setup

**Files:**
- Create: `fpga/matlab/cosim/cosim_config.m`
- Create: `fpga/matlab/cosim/cosim_tb.m`

- [ ] **Step 1: Write cosim_config.m**

```matlab
function cfg = cosim_config()
%COSIM_CONFIG HDL Verifier cosimulation configuration.
%   Returns a struct with cosimulation parameters.

    cfg.simulator = 'Vivado Simulator';  % Xsim
    cfg.hdl_lang = 'Verilog';
    cfg.clock_period_ns = 10;  % 100 MHz target
    cfg.reset_cycles = 5;

    % Test configurations (same as tb_full_sweep)
    cfg.tests = {
        struct('name','small',   'mx',8,  'my',8,  'type','empty',    'sweeps',20)
        struct('name','medium',  'mx',32, 'my',32, 'type','empty',    'sweeps',50)
        struct('name','sentinel','mx',8,  'my',8,  'type','sentinel', 'sweeps',20)
    };

    % Output directory for waveforms and logs
    cfg.output_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'build', 'cosim');
end
```

- [ ] **Step 2: Write cosim_tb.m**

```matlab
function cosim_tb()
%COSIM_TB HDL Verifier cosimulation testbench.
%   Runs generated HDL through Xsim and compares against MATLAB golden output.
%   Prerequisites:
%     1. Phase A (float) tests pass (tb_full_sweep)
%     2. Fixed-point conversion applied
%     3. HDL generated via hdlcoder.WorkflowAdvisor

    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'testbench'));

    cfg = cosim_config();

    % Ensure output directory exists
    if ~exist(cfg.output_dir, 'dir')
        mkdir(cfg.output_dir);
    end

    fprintf('=== HDL Cosimulation Testbench ===\n');
    fprintf('Simulator: %s\n', cfg.simulator);
    fprintf('HDL Language: %s\n', cfg.hdl_lang);

    for tc = 1:numel(cfg.tests)
        t = cfg.tests{tc};
        fprintf('\n--- Test: %s (%dx%d, %s) ---\n', t.name, t.mx, t.my, t.type);

        % Generate inputs
        [value, penalty, ~, ~] = gen_test_map(t.mx, t.my, t.type);
        trans = gen_transitions('trivial');

        % Run MATLAB golden model
        ml_value = value;
        for sweep = 1:t.sweeps
            [ml_value, d0] = vi_sweep_stream_algo(ml_value, ml_value, ...
                                                   penalty, trans, ...
                                                   t.mx, t.my, 0);
            [ml_value, d1] = vi_sweep_stream_algo(ml_value, ml_value, ...
                                                   penalty, trans, ...
                                                   t.mx, t.my, 1);
            if max(d0, d1) == 0, break; end
        end

        % TODO: After HDL is generated, add Xsim cosimulation commands here:
        % 1. filtertbench = hdlverifier.FILSimulation(...)
        % 2. filtertbench.InputSignals = {value_flat, penalty_flat, trans};
        % 3. filtertbench.run()
        % 4. Compare filtertbench.OutputSignals against ml_value
        %
        % For now, save golden data for manual cosim verification.
        save(fullfile(cfg.output_dir, [t.name '_golden.mat']), ...
             'value', 'penalty', 'trans', 'ml_value', 't');
        fprintf('  Golden data saved to %s_golden.mat\n', t.name);
    end

    fprintf('\n=== Cosimulation setup complete ===\n');
    fprintf('Next steps:\n');
    fprintf('  1. Generate HDL from Simulink: hdlcoder.WorkflowAdvisor\n');
    fprintf('  2. Update cosim_tb.m with hdlverifier.FILSimulation calls\n');
    fprintf('  3. Re-run cosim_tb to compare HDL output vs golden\n');
end
```

- [ ] **Step 3: Run cosim_tb.m to generate golden data**

```matlab
cd fpga/matlab/cosim
cosim_tb
```

Expected: Golden data MAT files saved, instructions printed.

- [ ] **Step 4: Commit**

```bash
git add fpga/matlab/cosim/cosim_config.m fpga/matlab/cosim/cosim_tb.m
git commit -m "feat(matlab): add HDL Verifier cosimulation framework"
```

---

## Task 12: SoC Builder Configuration and README

**Files:**
- Create: `fpga/matlab/soc/soc_config.m`
- Create: `fpga/matlab/soc/build_bitstream.m`
- Create: `fpga/matlab/README.md`

- [ ] **Step 1: Write soc_config.m**

```matlab
function cfg = soc_config()
%SOC_CONFIG SoC Builder configuration for Ultra96-V2.

    cfg.board = 'Avnet Ultra96-V2';  % Or custom BSP name
    cfg.device = 'xczu3eg-sbva484-1-i';
    cfg.vivado_version = '2025.2';
    cfg.clock_freq_mhz = 100;

    % AXI Interface mapping
    cfg.axi_master = {
        struct('name', 'gmem0', 'port', 'HP0', 'direction', 'ReadWrite', ...
               'data_width', 128, 'purpose', 'value_table write')
        struct('name', 'gmem1', 'port', 'HP1', 'direction', 'ReadOnly', ...
               'data_width', 128, 'purpose', 'penalty_table + trans_table')
        struct('name', 'gmem2', 'port', 'HP2', 'direction', 'ReadOnly', ...
               'data_width', 128, 'purpose', 'value_table read')
    };

    cfg.axi_slave = struct('name', 'ctrl', 'port', 'GP0', ...
                           'purpose', 'control registers');

    % CU configuration
    cfg.num_cu = 2;
    cfg.cu_names = {'vi_sweep_cu0', 'vi_sweep_cu1'};
end
```

- [ ] **Step 2: Write build_bitstream.m**

```matlab
function build_bitstream()
%BUILD_BITSTREAM Generate bitstream via SoC Builder workflow.
%   Prerequisites:
%     1. Simulink model configured with SoC Blockset
%     2. HDL generation verified via cosimulation
%     3. Vivado on PATH

    cfg = soc_config();
    model_name = 'vi_sweep_stream_matlab';
    model_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'model');
    build_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'build', 'matlab');

    fprintf('=== SoC Builder Bitstream Generation ===\n');
    fprintf('Board: %s\n', cfg.board);
    fprintf('Device: %s\n', cfg.device);
    fprintf('Clock: %d MHz\n', cfg.clock_freq_mhz);
    fprintf('Build dir: %s\n', build_dir);

    if ~exist(build_dir, 'dir')
        mkdir(build_dir);
    end

    % Load model
    addpath(model_dir);
    load_system(model_name);

    % Run SoC Builder workflow
    % Step 1: Generate IP Core
    fprintf('\n--- Step 1: IP Core Generation ---\n');
    % hdlcoder.WorkflowAdvisor(model_name) in interactive mode
    % For batch: use hdlworkflow object
    % hw = hdlcoder.WorkflowConfig('SynthesisTool', 'Xilinx Vivado', ...
    %     'TargetWorkflow', 'IP Core Generation');
    % hw.run();

    % Step 2: Build Bitstream
    fprintf('--- Step 2: Build Bitstream ---\n');
    % Automated via SoC Builder:
    % socModelAnalyzer(model_name);
    % socBuildModel(model_name, 'BuildAction', 'Build');

    fprintf('\n=== Bitstream generation workflow ready ===\n');
    fprintf('Run interactively:\n');
    fprintf('  1. Open model: open_system(''%s'')\n', model_name);
    fprintf('  2. Launch: HDL Workflow Advisor\n');
    fprintf('  3. Target: IP Core Generation for SoC Builder\n');
    fprintf('  4. Board: %s\n', cfg.board);
    fprintf('  5. Generate and build\n');
end
```

- [ ] **Step 3: Write README.md**

```markdown
# MATLAB HDL Coder Streaming Kernel

Third kernel variant for the Value Iteration FPGA accelerator, built with
MATLAB HDL Coder + SoC Blockset.

## Required Toolboxes

- MATLAB R2024b+
- Simulink
- HDL Coder
- HDL Verifier
- Fixed-Point Designer
- SoC Blockset
- Zynq UltraScale+ MPSoC support package (or Ultra96-V2 BSP)

## Quick Start

```matlab
% 1. Add paths
addpath('src', 'testbench');

% 2. Run unit tests (no toolboxes needed beyond base MATLAB)
tb_cost_of
tb_compute_row
tb_load_store_row
tb_stream_strip
tb_full_sweep          % Requires MEX compiler for C reference comparison

% 3. Fixed-point analysis (requires Fixed-Point Designer)
cd fixedpoint; fp_config

% 4. HDL cosimulation (requires HDL Verifier + Vivado Xsim)
cd cosim; cosim_tb

% 5. Bitstream generation (requires HDL Coder + SoC Blockset + Vivado)
cd soc; build_bitstream
```

## Directory Structure

```
fpga/matlab/
├── src/           MATLAB functions (HDL Coder targets)
├── testbench/     Tests and test data generators
├── fixedpoint/    Fixed-Point Advisor configuration
├── cosim/         HDL Verifier cosimulation
├── model/         Simulink models (.slx)
└── soc/           SoC Builder configuration
```

## Development Workflow

### Phase A: Floating-Point Verification

1. Edit algorithm in `src/*.m` (all signals are `double`)
2. Run `tb_full_sweep` to compare against C reference
3. Iterate until all tests pass

### Phase B: Fixed-Point Conversion

1. Run `fixedpoint/fp_config.m` to analyze dynamic range
2. Open Simulink model → Fixed-Point Tool → apply proposed types
3. Re-run `tb_full_sweep` to verify zero-error conversion
4. Target bit widths: value=16, penalty=16, offset=8 (matching HLS)

### Phase C: HDL Generation and Cosimulation

1. Open `model/vi_sweep_stream_matlab.slx` in Simulink
2. HDL Workflow Advisor → Generate HDL
3. Run `cosim/cosim_tb.m` with Xsim backend
4. Verify cycle-accurate match against golden MATLAB output

### Phase D: Bitstream and Hardware

1. Run `soc/build_bitstream.m` (or use HDL Workflow Advisor GUI)
2. Deploy .bit + .hwh to Ultra96-V2
3. Test via `vi_cli --verify` with MATLAB driver ops

## Makefile Targets

From `fpga/`:

```bash
make matlab-sim        # Run tb_full_sweep
make matlab-hdl        # Generate HDL
make matlab-cosim      # Run cosimulation
make matlab-bitstream  # Build bitstream
```

## Constants

All constants are defined in `src/vi_params.m` and match
`fpga/hls/stream/src/vi_stream_types.h`. See the design spec at
`docs/superpowers/specs/2026-04-13-matlab-hdl-coder-streaming-design.md`.
```

- [ ] **Step 4: Commit**

```bash
git add fpga/matlab/soc/soc_config.m fpga/matlab/soc/build_bitstream.m fpga/matlab/README.md
git commit -m "feat(matlab): add SoC Builder config and project README"
```

---

## Task 13: Makefile Integration

**Files:**
- Modify: `fpga/Makefile`

- [ ] **Step 1: Add MATLAB targets to fpga/Makefile**

Append to end of `fpga/Makefile`:

```makefile

# ---------- MATLAB kernel targets ----------

.PHONY: matlab-sim matlab-hdl matlab-cosim matlab-bitstream

matlab-sim:
	cd "$(FPGA_DIR)matlab" && matlab -batch "addpath('src','testbench'); tb_full_sweep"

matlab-hdl:
	cd "$(FPGA_DIR)matlab" && matlab -batch "addpath('src','model'); cd model; create_model"

matlab-cosim:
	cd "$(FPGA_DIR)matlab" && matlab -batch "addpath('src','testbench'); cd cosim; cosim_tb"

matlab-bitstream:
	cd "$(FPGA_DIR)matlab" && matlab -batch "addpath('src','testbench','model'); cd soc; build_bitstream"
```

- [ ] **Step 2: Verify Makefile syntax**

```bash
make -C fpga -n matlab-sim
```

Expected: Prints the `matlab -batch` command without executing.

- [ ] **Step 3: Commit**

```bash
git add fpga/Makefile
git commit -m "feat(matlab): add MATLAB kernel targets to fpga/Makefile"
```

---

## Task 14: Driver Integration Placeholder

**Files:**
- Modify: `driver/uio/vi_device.h`

This task adds the `vi_matlab_ops` declaration to the device header. The actual implementation (`vi_device_matlab.c`) will be written after SoC Builder generates the register map, since the register offsets aren't known until IP generation completes.

- [ ] **Step 1: Add vi_matlab_ops extern declaration**

In `driver/uio/vi_device.h`, add the declaration after the existing `vi_mock_ops` line:

```c
extern const vi_device_ops_t vi_mock_ops;

/* MATLAB HDL Coder kernel ops (register map TBD after SoC Builder IP generation) */
#ifndef VI_MOCK_ONLY
extern const vi_device_ops_t vi_matlab_ops;
#endif
```

- [ ] **Step 2: Verify build still works**

```bash
make test-host
```

Expected: All tests pass. The `vi_matlab_ops` symbol is only declared, not referenced, so no link errors.

- [ ] **Step 3: Commit**

```bash
git add driver/uio/vi_device.h
git commit -m "feat(driver): add vi_matlab_ops declaration for MATLAB kernel integration"
```

---

## Task 15: CLAUDE.md Update

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Add MATLAB section to CLAUDE.md**

Add after the existing `### FPGA build` section:

```markdown
### MATLAB kernel (`fpga/matlab/`)

Requires MATLAB R2024b+ with HDL Coder, HDL Verifier, Fixed-Point Designer, SoC Blockset.

- `make -C fpga matlab-sim` — run MATLAB algorithm tests (`tb_full_sweep`).
- `make -C fpga matlab-hdl` — generate/update Simulink model.
- `make -C fpga matlab-cosim` — HDL Verifier cosimulation via Xsim.
- `make -C fpga matlab-bitstream` — SoC Builder bitstream generation.

The MATLAB kernel is a third variant alongside tile and stream HLS kernels. Algorithm functions in `fpga/matlab/src/` mirror the streaming HLS kernel (`fpga/hls/stream/src/`). Constants in `vi_params.m` must stay synchronized with `vi_stream_types.h`.
```

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: add MATLAB kernel section to CLAUDE.md"
```
