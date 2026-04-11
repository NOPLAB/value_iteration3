# Value Iteration FPGA Accelerator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a Value Iteration FPGA accelerator for Ultra96-V2 that processes a 700m campus course map (14,000 x 800 cells, theta=60) within 60 seconds.

**Architecture:** Vitis HLS C++ kernel (`vi_sweep`) performs tiled Bellman updates on DDR-resident Value/Penalty tables. Two Compute Units process tiles in a checkerboard pattern. 6 actions are fully unrolled for II=1 throughput. ARM PS manages sweep loop and convergence detection.

**Tech Stack:** Vitis HLS 2023.2+, Vivado 2023.2+, C++14, Ultra96-V2 (ZU3EG), PYNQ

**Spec:** `docs/superpowers/specs/2026-04-10-value-iteration-fpga-design.md`

**Reference project:** `../sindy/fpga/` (same board, same build flow)

---

### Task 1: Project Setup & Type Definitions

**Files:**
- Create: `fpga/hls/vi_sweep/src/vi_types.h`
- Create: `.gitignore`

- [ ] **Step 1: Initialize git repository**

```bash
cd /c/Users/nop/dev/mywork/value_iteration_fpga
git init
```

- [ ] **Step 2: Create .gitignore**

Create `.gitignore`:

```
# HLS build artifacts
fpga/scripts/hls_build/
fpga/hls/vi_sweep/solution*/

# Vivado project artifacts
fpga/vivado/ultra96v2/vi_ultra96v2/
fpga/vivado/ultra96v2/ip_repo/

# OS
.Xil/
*.jou
*.log
*.str
```

- [ ] **Step 3: Create directory structure**

```bash
mkdir -p fpga/hls/vi_sweep/src
mkdir -p fpga/hls/vi_sweep/tb
mkdir -p fpga/vivado/ultra96v2
mkdir -p fpga/pynq
mkdir -p fpga/scripts
mkdir -p host/src
mkdir -p host/test
```

- [ ] **Step 4: Write vi_types.h**

Create `fpga/hls/vi_sweep/src/vi_types.h`:

```cpp
#pragma once

#include <ap_int.h>
#include <hls_stream.h>

// ---------------------------------------------------------------------------
// Data types — 16-bit for DDR efficiency (see spec section 2.4)
// ---------------------------------------------------------------------------
typedef ap_uint<16> value_t;
typedef ap_uint<16> penalty_t;
typedef ap_int<8>   offset_t;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
constexpr int N_ACTIONS = 6;
constexpr int N_THETA   = 60;

// Tile geometry
constexpr int TILE_W    = 32;
constexpr int TILE_H    = 32;
constexpr int HALO      = 6;   // max forward 0.3m / 0.05m resolution = 6 cells
constexpr int TILE_W_H  = TILE_W + 2 * HALO;  // 44
constexpr int TILE_H_H  = TILE_H + 2 * HALO;  // 44

// Sentinel values
constexpr value_t   MAX_VALUE       = 0xFFFF;
constexpr penalty_t PENALTY_OBSTACLE = 0xFFFF;  // impassable cell
constexpr penalty_t PENALTY_GOAL     = 0xFFFE;  // goal cell — value stays 0

// Transition table: packed as (dix, diy, dit) in one 32-bit word
// Layout: bits [7:0]=dix, [15:8]=diy, [23:16]=dit, [31:24]=reserved
// Total entries: N_ACTIONS * N_THETA = 360
constexpr int TRANS_TABLE_SIZE = N_ACTIONS * N_THETA;
```

- [ ] **Step 5: Commit**

```bash
git add .gitignore fpga/hls/vi_sweep/src/vi_types.h
git commit -m "feat: project scaffolding and HLS type definitions"
```

---

### Task 2: CPU Reference Implementation

A standalone C++ implementation of Value Iteration used by the testbench to produce golden reference output. No HLS dependencies — plain C++14.

**Files:**
- Create: `fpga/hls/vi_sweep/tb/vi_reference.h`
- Create: `fpga/hls/vi_sweep/tb/vi_reference.cpp`

- [ ] **Step 1: Write vi_reference.h**

Create `fpga/hls/vi_sweep/tb/vi_reference.h`:

```cpp
#pragma once

#include <cstdint>
#include <vector>
#include <cmath>

// CPU reference implementation of deterministic Value Iteration.
// Uses 16-bit unsigned values to match FPGA precision.

namespace vi_ref {

constexpr int N_ACTIONS = 6;
constexpr int N_THETA   = 60;
constexpr uint16_t MAX_VALUE       = 0xFFFF;
constexpr uint16_t PENALTY_OBSTACLE = 0xFFFF;
constexpr uint16_t PENALTY_GOAL     = 0xFFFE;

struct TransitionEntry {
    int8_t dix, diy, dit;
};

// Compute deterministic transition table for 6 fixed actions.
// Actions (matching spec section 2.3):
//   0: forward      (0.3m,   0 deg)
//   1: backward     (-0.2m,  0 deg)
//   2: left          (0.0m, +20 deg)
//   3: right         (0.0m, -20 deg)
//   4: forward-left  (0.3m, +20 deg)
//   5: forward-right (0.3m, -20 deg)
void compute_transitions(
    double xy_resolution,
    TransitionEntry trans[N_ACTIONS][N_THETA]);

// Run value iteration sweeps on the entire map until convergence.
// Returns the number of sweeps executed.
//
// value_table: [map_y][map_x][N_THETA], row-major.
//              Initialized by caller: goal cells = 0, others = MAX_VALUE.
// penalty_table: [map_y][map_x].
//              PENALTY_OBSTACLE for obstacles, PENALTY_GOAL for goal cells,
//              0..PENALTY_GOAL-1 for traversable cells.
int run_vi(
    uint16_t *value_table,
    const uint16_t *penalty_table,
    const TransitionEntry trans[N_ACTIONS][N_THETA],
    int map_x, int map_y,
    uint16_t threshold,
    int max_sweeps);

} // namespace vi_ref
```

- [ ] **Step 2: Write vi_reference.cpp**

Create `fpga/hls/vi_sweep/tb/vi_reference.cpp`:

```cpp
#include "vi_reference.h"
#include <algorithm>
#include <cstdio>

namespace vi_ref {

// Action definitions (spec section 2.3)
static const double ACTION_FW[]  = { 0.3, -0.2, 0.0,  0.0, 0.3,  0.3};
static const double ACTION_ROT[] = { 0.0,  0.0, 20.0,-20.0, 20.0,-20.0};

void compute_transitions(
    double xy_resolution,
    TransitionEntry trans[N_ACTIONS][N_THETA])
{
    double t_resolution = 360.0 / N_THETA;  // 6.0 degrees

    for (int a = 0; a < N_ACTIONS; a++) {
        for (int it = 0; it < N_THETA; it++) {
            double theta_deg = it * t_resolution + t_resolution * 0.5; // cell center
            double theta_rad = theta_deg * M_PI / 180.0;

            double dx_m = ACTION_FW[a] * cos(theta_rad);
            double dy_m = ACTION_FW[a] * sin(theta_rad);
            double dt_deg = ACTION_ROT[a];

            // Convert to cell offsets
            int dix = (int)floor(dx_m / xy_resolution);
            int diy = (int)floor(dy_m / xy_resolution);

            double new_theta = theta_deg + dt_deg;
            while (new_theta < 0.0) new_theta += 360.0;
            while (new_theta >= 360.0) new_theta -= 360.0;
            int new_it = (int)floor(new_theta / t_resolution);
            int dit = new_it - it;
            // Normalize dit to smallest absolute value
            if (dit > N_THETA / 2) dit -= N_THETA;
            if (dit < -N_THETA / 2) dit += N_THETA;

            trans[a][it].dix = (int8_t)dix;
            trans[a][it].diy = (int8_t)diy;
            trans[a][it].dit = (int8_t)dit;
        }
    }
}

static inline int to_index(int ix, int iy, int it, int map_x) {
    return (iy * map_x + ix) * N_THETA + it;
}

int run_vi(
    uint16_t *value_table,
    const uint16_t *penalty_table,
    const TransitionEntry trans[N_ACTIONS][N_THETA],
    int map_x, int map_y,
    uint16_t threshold,
    int max_sweeps)
{
    int sweep;
    for (sweep = 0; sweep < max_sweeps; sweep++) {
        uint16_t max_delta = 0;

        for (int iy = 0; iy < map_y; iy++) {
            for (int ix = 0; ix < map_x; ix++) {
                uint16_t pen = penalty_table[iy * map_x + ix];

                // Skip obstacles and goals
                if (pen >= PENALTY_GOAL) continue;

                for (int it = 0; it < N_THETA; it++) {
                    int idx = to_index(ix, iy, it, map_x);
                    uint16_t old_val = value_table[idx];

                    uint16_t min_cost = MAX_VALUE;

                    for (int a = 0; a < N_ACTIONS; a++) {
                        int nx = ix + trans[a][it].dix;
                        int ny = iy + trans[a][it].diy;
                        int nt_raw = it + trans[a][it].dit;
                        int nt = (nt_raw < 0) ? nt_raw + N_THETA
                               : (nt_raw >= N_THETA) ? nt_raw - N_THETA
                               : nt_raw;

                        // Boundary check
                        if (nx < 0 || nx >= map_x || ny < 0 || ny >= map_y) continue;

                        uint16_t nv = value_table[to_index(nx, ny, nt, map_x)];
                        uint16_t np = penalty_table[ny * map_x + nx];

                        if (nv == MAX_VALUE || np == PENALTY_OBSTACLE) continue;

                        // Saturating add
                        uint32_t sum = (uint32_t)nv + (uint32_t)np;
                        uint16_t cost = (sum > MAX_VALUE) ? MAX_VALUE : (uint16_t)sum;

                        if (cost < min_cost) min_cost = cost;
                    }

                    // Gauss-Seidel update (in-place)
                    value_table[idx] = min_cost;

                    uint16_t d = (min_cost > old_val) ? (min_cost - old_val)
                                                      : (old_val - min_cost);
                    if (d > max_delta) max_delta = d;
                }
            }
        }

        if (max_delta <= threshold) {
            sweep++;
            break;
        }
    }

    return sweep;
}

} // namespace vi_ref
```

- [ ] **Step 3: Commit**

```bash
git add fpga/hls/vi_sweep/tb/vi_reference.h fpga/hls/vi_sweep/tb/vi_reference.cpp
git commit -m "feat: CPU reference implementation for Value Iteration"
```

---

### Task 3: HLS Testbench

The testbench creates a small map, runs the CPU reference, then runs the HLS kernel, and compares results.

**Files:**
- Create: `fpga/hls/vi_sweep/tb/vi_sweep_tb.cpp`

- [ ] **Step 1: Write vi_sweep_tb.cpp**

Create `fpga/hls/vi_sweep/tb/vi_sweep_tb.cpp`:

```cpp
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include "../src/vi_sweep_top.h"
#include "vi_reference.h"

// Test map dimensions (small enough for full BRAM residence)
constexpr int MAP_X = 20;
constexpr int MAP_Y = 20;
constexpr int MAP_SIZE = MAP_X * MAP_Y;
constexpr int STATE_SIZE = MAP_SIZE * vi_ref::N_THETA;
constexpr double XY_RESOLUTION = 0.05;  // meters per cell

// Build a simple test map with obstacles and a goal
static void build_test_map(
    uint16_t *penalty_table,
    uint16_t *value_table,
    int goal_x, int goal_y)
{
    // Initialize all cells as free (penalty = 0)
    for (int i = 0; i < MAP_SIZE; i++)
        penalty_table[i] = 0;

    // Add border obstacles
    for (int x = 0; x < MAP_X; x++) {
        penalty_table[0 * MAP_X + x] = vi_ref::PENALTY_OBSTACLE;
        penalty_table[(MAP_Y - 1) * MAP_X + x] = vi_ref::PENALTY_OBSTACLE;
    }
    for (int y = 0; y < MAP_Y; y++) {
        penalty_table[y * MAP_X + 0] = vi_ref::PENALTY_OBSTACLE;
        penalty_table[y * MAP_X + (MAP_X - 1)] = vi_ref::PENALTY_OBSTACLE;
    }

    // Add an L-shaped obstacle in the middle
    for (int x = 5; x <= 12; x++)
        penalty_table[10 * MAP_X + x] = vi_ref::PENALTY_OBSTACLE;
    for (int y = 6; y <= 10; y++)
        penalty_table[y * MAP_X + 12] = vi_ref::PENALTY_OBSTACLE;

    // Add safety penalty near obstacles (penalty = 100)
    for (int y = 1; y < MAP_Y - 1; y++) {
        for (int x = 1; x < MAP_X - 1; x++) {
            if (penalty_table[y * MAP_X + x] == vi_ref::PENALTY_OBSTACLE) continue;
            // Check 4-neighbors for obstacle adjacency
            bool near_obs = false;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                    if (penalty_table[(y+dy) * MAP_X + (x+dx)] == vi_ref::PENALTY_OBSTACLE)
                        near_obs = true;
            if (near_obs)
                penalty_table[y * MAP_X + x] = 100;
        }
    }

    // Set goal cells
    penalty_table[goal_y * MAP_X + goal_x] = vi_ref::PENALTY_GOAL;

    // Initialize value table: goal = 0, others = MAX_VALUE
    for (int i = 0; i < STATE_SIZE; i++)
        value_table[i] = vi_ref::MAX_VALUE;

    for (int it = 0; it < vi_ref::N_THETA; it++) {
        int idx = (goal_y * MAP_X + goal_x) * vi_ref::N_THETA + it;
        value_table[idx] = 0;
    }
}

// Pack transition table for HLS (3 int8 -> 1 uint32)
static void pack_transitions(
    const vi_ref::TransitionEntry trans[vi_ref::N_ACTIONS][vi_ref::N_THETA],
    uint32_t *packed)
{
    for (int a = 0; a < vi_ref::N_ACTIONS; a++) {
        for (int it = 0; it < vi_ref::N_THETA; it++) {
            uint32_t w = 0;
            w |= ((uint32_t)(uint8_t)trans[a][it].dix) << 0;
            w |= ((uint32_t)(uint8_t)trans[a][it].diy) << 8;
            w |= ((uint32_t)(uint8_t)trans[a][it].dit) << 16;
            packed[a * vi_ref::N_THETA + it] = w;
        }
    }
}

int main()
{
    printf("=== Value Iteration HLS C-Simulation Testbench ===\n");
    printf("Map: %d x %d, theta cells: %d, resolution: %.3f m\n",
           MAP_X, MAP_Y, vi_ref::N_THETA, XY_RESOLUTION);

    // Compute transition table
    vi_ref::TransitionEntry trans[vi_ref::N_ACTIONS][vi_ref::N_THETA];
    vi_ref::compute_transitions(XY_RESOLUTION, trans);

    printf("\nTransition table (sample, action=0 forward):\n");
    for (int it = 0; it < 5; it++)
        printf("  theta=%d: dix=%d diy=%d dit=%d\n",
               it, trans[0][it].dix, trans[0][it].diy, trans[0][it].dit);

    // Build test map
    int goal_x = 15, goal_y = 15;
    uint16_t penalty_ref[MAP_SIZE];
    uint16_t value_ref[STATE_SIZE];
    build_test_map(penalty_ref, value_ref, goal_x, goal_y);

    // Make a copy for HLS
    uint16_t penalty_hls[MAP_SIZE];
    uint16_t value_hls[STATE_SIZE];
    memcpy(penalty_hls, penalty_ref, sizeof(penalty_ref));
    memcpy(value_hls, value_ref, sizeof(value_ref));

    // Run CPU reference
    printf("\nRunning CPU reference...\n");
    int ref_sweeps = vi_ref::run_vi(value_ref, penalty_ref, trans,
                                     MAP_X, MAP_Y, 0, 200);
    printf("  Converged in %d sweeps\n", ref_sweeps);

    // Pack transition table for HLS
    uint32_t trans_packed[vi_ref::N_ACTIONS * vi_ref::N_THETA];
    pack_transitions(trans, trans_packed);

    // Run HLS kernel (multiple sweeps to converge)
    printf("\nRunning HLS kernel...\n");
    int num_tiles_x = (MAP_X + TILE_W - 1) / TILE_W;
    int num_tiles_y = (MAP_Y + TILE_H - 1) / TILE_H;
    printf("  Tiles: %d x %d\n", num_tiles_x, num_tiles_y);

    value_t hls_max_delta;
    int hls_sweeps = 0;
    for (int s = 0; s < 200; s++) {
        // Single CU (cu_id=0), process ALL tiles (no checkerboard for small map)
        vi_sweep(
            (value_t *)value_hls,
            (const penalty_t *)penalty_hls,
            (const ap_uint<32> *)trans_packed,
            MAP_X, MAP_Y,
            num_tiles_x, num_tiles_y,
            0,  // cu_id
            &hls_max_delta);

        hls_sweeps++;
        if ((uint16_t)hls_max_delta == 0) break;
    }
    printf("  Converged in %d sweeps, final max_delta=%d\n",
           hls_sweeps, (int)(uint16_t)hls_max_delta);

    // Compare results
    printf("\n=== Verification ===\n");
    int mismatch_count = 0;
    int checked = 0;
    for (int iy = 0; iy < MAP_Y; iy++) {
        for (int ix = 0; ix < MAP_X; ix++) {
            if (penalty_ref[iy * MAP_X + ix] >= vi_ref::PENALTY_GOAL) continue;
            for (int it = 0; it < vi_ref::N_THETA; it++) {
                int idx = (iy * MAP_X + ix) * vi_ref::N_THETA + it;
                uint16_t ref_v = value_ref[idx];
                uint16_t hls_v = value_hls[idx];
                checked++;

                // Allow small tolerance (tile boundary Gauss-Seidel ordering differs)
                int diff = (int)ref_v - (int)hls_v;
                if (diff < 0) diff = -diff;
                if (diff > 1) {
                    if (mismatch_count < 10) {
                        printf("  MISMATCH at (%d,%d,t=%d): ref=%u hls=%u diff=%d\n",
                               ix, iy, it, ref_v, hls_v, diff);
                    }
                    mismatch_count++;
                }
            }
        }
    }

    printf("\nChecked %d states, %d mismatches\n", checked, mismatch_count);

    // Verify goal state unchanged
    for (int it = 0; it < vi_ref::N_THETA; it++) {
        int idx = (goal_y * MAP_X + goal_x) * vi_ref::N_THETA + it;
        if (value_hls[idx] != 0) {
            printf("  FAIL: goal state (%d,%d,t=%d) value=%d (expected 0)\n",
                   goal_x, goal_y, it, (int)value_hls[idx]);
            mismatch_count++;
        }
    }

    // Verify obstacle states unchanged
    for (int iy = 0; iy < MAP_Y; iy++) {
        for (int ix = 0; ix < MAP_X; ix++) {
            if (penalty_hls[iy * MAP_X + ix] != vi_ref::PENALTY_OBSTACLE) continue;
            for (int it = 0; it < vi_ref::N_THETA; it++) {
                int idx = (iy * MAP_X + ix) * vi_ref::N_THETA + it;
                if (value_hls[idx] != vi_ref::MAX_VALUE) {
                    printf("  FAIL: obstacle (%d,%d,t=%d) value=%d (expected MAX)\n",
                           ix, iy, it, (int)value_hls[idx]);
                    mismatch_count++;
                }
            }
        }
    }

    if (mismatch_count > 0) {
        printf("\nTESTBENCH FAILED (%d errors)\n", mismatch_count);
        return 1;
    }

    printf("\nTESTBENCH PASSED\n");
    return 0;
}
```

- [ ] **Step 2: Commit**

```bash
git add fpga/hls/vi_sweep/tb/vi_sweep_tb.cpp
git commit -m "feat: HLS testbench with CPU reference comparison"
```

---

### Task 4: HLS Kernel — compute_bellman

The innermost computation: Bellman update on a BRAM-resident tile.

**Files:**
- Create: `fpga/hls/vi_sweep/src/compute_bellman.h`
- Create: `fpga/hls/vi_sweep/src/compute_bellman.cpp`

- [ ] **Step 1: Write compute_bellman.h**

Create `fpga/hls/vi_sweep/src/compute_bellman.h`:

```cpp
#pragma once

#include "vi_types.h"

// Bellman update on a tile stored in BRAM.
//
// val_buf: [TILE_H_H][TILE_W_H][N_THETA] — value table tile including halo.
//          Updated in-place (Gauss-Seidel). Only the inner TILE_W x TILE_H
//          region (offset by HALO) is written; halo region is read-only.
// pen_buf: [TILE_H_H][TILE_W_H] — penalty for each cell (theta-independent).
//          PENALTY_OBSTACLE = impassable, PENALTY_GOAL = goal (skip update).
// delta_table: [N_ACTIONS][N_THETA][3] — (dix, diy, dit) offsets.
// tile_w, tile_h: actual tile dimensions (may be < TILE_W at map edge).
// max_delta: output — maximum |V_new - V_old| across all states in this tile.
void compute_bellman(
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    const penalty_t pen_buf_0[TILE_H_H][TILE_W_H],
    const penalty_t pen_buf_1[TILE_H_H][TILE_W_H],
    const penalty_t pen_buf_2[TILE_H_H][TILE_W_H],
    const offset_t delta_table[N_ACTIONS][N_THETA][3],
    int tile_w, int tile_h,
    value_t &max_delta);
```

- [ ] **Step 2: Write compute_bellman.cpp**

Create `fpga/hls/vi_sweep/src/compute_bellman.cpp`:

```cpp
#include "compute_bellman.h"

void compute_bellman(
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    const penalty_t pen_buf_0[TILE_H_H][TILE_W_H],
    const penalty_t pen_buf_1[TILE_H_H][TILE_W_H],
    const penalty_t pen_buf_2[TILE_H_H][TILE_W_H],
    const offset_t delta_table[N_ACTIONS][N_THETA][3],
    int tile_w, int tile_h,
    value_t &max_delta)
{
    // val_buf: complete partition on theta dim for 6 parallel reads.
    // With N_THETA=60 banks, each dual-port, 6 reads are served by 3 bank pairs.
    // (forward/backward share theta bank, left/fwd-left share, right/fwd-right share)
    #pragma HLS ARRAY_PARTITION variable=val_buf complete dim=3
    #pragma HLS BIND_STORAGE variable=val_buf type=ram_2p impl=bram

    // 3 copies of penalty buffer for 6 parallel reads (2 reads per copy)
    #pragma HLS BIND_STORAGE variable=pen_buf_0 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_1 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_2 type=ram_2p impl=bram

    // Transition table fully in registers
    #pragma HLS ARRAY_PARTITION variable=delta_table complete dim=0

    value_t local_max_delta = 0;

    LOOP_Y: for (int iy = 0; iy < tile_h; iy++) {
        LOOP_X: for (int ix = 0; ix < tile_w; ix++) {
            // Penalty for this cell (used to check skip)
            int by = iy + HALO;
            int bx = ix + HALO;
            penalty_t cell_pen = pen_buf_0[by][bx];
            bool skip = (cell_pen >= PENALTY_GOAL);

            LOOP_T: for (int it = 0; it < N_THETA; it++) {
                #pragma HLS PIPELINE II=1

                value_t old_val = val_buf[by][bx][it];

                // Compute neighbor coordinates for all 6 actions
                // Actions 0,1 -> pen_buf_0; 2,3 -> pen_buf_1; 4,5 -> pen_buf_2
                value_t costs[N_ACTIONS];
                #pragma HLS ARRAY_PARTITION variable=costs complete

                for (int a = 0; a < N_ACTIONS; a++) {
                    #pragma HLS UNROLL

                    int ny = by + (int)delta_table[a][it][1];
                    int nx = bx + (int)delta_table[a][it][0];
                    int nt_raw = it + (int)delta_table[a][it][2];
                    int nt = (nt_raw < 0) ? (nt_raw + N_THETA)
                           : (nt_raw >= N_THETA) ? (nt_raw - N_THETA)
                           : nt_raw;

                    value_t nv = val_buf[ny][nx][nt];

                    penalty_t np;
                    if (a < 2)      np = pen_buf_0[ny][nx];
                    else if (a < 4) np = pen_buf_1[ny][nx];
                    else            np = pen_buf_2[ny][nx];

                    if (nv == MAX_VALUE || np >= PENALTY_OBSTACLE) {
                        costs[a] = MAX_VALUE;
                    } else {
                        ap_uint<17> sum = (ap_uint<17>)nv + (ap_uint<17>)np;
                        costs[a] = (sum >= (ap_uint<17>)MAX_VALUE)
                                 ? (value_t)(MAX_VALUE - 1) : (value_t)sum;
                    }
                }

                // Find minimum cost across 6 actions (reduction tree)
                value_t min01 = (costs[0] < costs[1]) ? costs[0] : costs[1];
                value_t min23 = (costs[2] < costs[3]) ? costs[2] : costs[3];
                value_t min45 = (costs[4] < costs[5]) ? costs[4] : costs[5];
                value_t min03 = (min01 < min23) ? min01 : min23;
                value_t min_cost = (min03 < min45) ? min03 : min45;

                // Conditional update (skip obstacles and goals)
                value_t new_val = skip ? old_val : min_cost;
                val_buf[by][bx][it] = new_val;

                // Delta tracking
                value_t d = (new_val > old_val) ? (value_t)(new_val - old_val)
                                                : (value_t)(old_val - new_val);
                value_t masked_d = skip ? (value_t)0 : d;
                if (masked_d > local_max_delta) {
                    local_max_delta = masked_d;
                }
            }
        }
    }

    max_delta = local_max_delta;
}
```

- [ ] **Step 3: Commit**

```bash
git add fpga/hls/vi_sweep/src/compute_bellman.h fpga/hls/vi_sweep/src/compute_bellman.cpp
git commit -m "feat: Bellman update kernel with 6-action UNROLL and II=1 pipeline"
```

---

### Task 5: HLS Kernel — load_tiles & store_tiles

Functions to burst-transfer tile data between DDR and BRAM.

**Files:**
- Create: `fpga/hls/vi_sweep/src/load_tiles.h`
- Create: `fpga/hls/vi_sweep/src/load_tiles.cpp`
- Create: `fpga/hls/vi_sweep/src/store_tiles.h`
- Create: `fpga/hls/vi_sweep/src/store_tiles.cpp`

- [ ] **Step 1: Write load_tiles.h**

Create `fpga/hls/vi_sweep/src/load_tiles.h`:

```cpp
#pragma once

#include "vi_types.h"

// Load transition table from DDR into register array (once per kernel invocation).
void load_transitions(
    const ap_uint<32> *trans_table,
    offset_t delta_table[N_ACTIONS][N_THETA][3]);

// Load a tile (with halo) from DDR into BRAM buffers.
// tile_ox, tile_oy: origin of tile in map coordinates (cells, not including halo).
// Cells outside map boundaries are filled with MAX_VALUE / PENALTY_OBSTACLE.
void load_tile(
    const value_t *value_table,
    const penalty_t *penalty_table,
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    penalty_t pen_buf_0[TILE_H_H][TILE_W_H],
    penalty_t pen_buf_1[TILE_H_H][TILE_W_H],
    penalty_t pen_buf_2[TILE_H_H][TILE_W_H],
    int tile_ox, int tile_oy,
    int map_x, int map_y);
```

- [ ] **Step 2: Write load_tiles.cpp**

Create `fpga/hls/vi_sweep/src/load_tiles.cpp`:

```cpp
#include "load_tiles.h"

void load_transitions(
    const ap_uint<32> *trans_table,
    offset_t delta_table[N_ACTIONS][N_THETA][3])
{
    LOAD_TRANS: for (int i = 0; i < TRANS_TABLE_SIZE; i++) {
        #pragma HLS PIPELINE II=1
        ap_uint<32> w = trans_table[i];
        int a = i / N_THETA;
        int t = i % N_THETA;
        delta_table[a][t][0] = (offset_t)(w(7, 0));    // dix
        delta_table[a][t][1] = (offset_t)(w(15, 8));   // diy
        delta_table[a][t][2] = (offset_t)(w(23, 16));  // dit
    }
}

void load_tile(
    const value_t *value_table,
    const penalty_t *penalty_table,
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    penalty_t pen_buf_0[TILE_H_H][TILE_W_H],
    penalty_t pen_buf_1[TILE_H_H][TILE_W_H],
    penalty_t pen_buf_2[TILE_H_H][TILE_W_H],
    int tile_ox, int tile_oy,
    int map_x, int map_y)
{
    // Load value table tile + halo
    LOAD_V_Y: for (int ly = 0; ly < TILE_H_H; ly++) {
        int gy = tile_oy - HALO + ly;  // global y coordinate

        LOAD_V_X: for (int lx = 0; lx < TILE_W_H; lx++) {
            int gx = tile_ox - HALO + lx;  // global x coordinate

            bool out_of_bounds = (gx < 0 || gx >= map_x || gy < 0 || gy >= map_y);

            // Load penalty (same for all 3 copies)
            penalty_t pen;
            if (out_of_bounds) {
                pen = PENALTY_OBSTACLE;
            } else {
                pen = penalty_table[gy * map_x + gx];
            }
            pen_buf_0[ly][lx] = pen;
            pen_buf_1[ly][lx] = pen;
            pen_buf_2[ly][lx] = pen;

            // Load value for all theta
            LOAD_V_T: for (int it = 0; it < N_THETA; it++) {
                #pragma HLS PIPELINE II=1
                if (out_of_bounds) {
                    val_buf[ly][lx][it] = MAX_VALUE;
                } else {
                    int addr = (gy * map_x + gx) * N_THETA + it;
                    val_buf[ly][lx][it] = value_table[addr];
                }
            }
        }
    }
}
```

- [ ] **Step 3: Write store_tiles.h**

Create `fpga/hls/vi_sweep/src/store_tiles.h`:

```cpp
#pragma once

#include "vi_types.h"

// Store the inner tile region (excluding halo) back to DDR.
void store_tile(
    value_t *value_table,
    const value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    int tile_ox, int tile_oy,
    int tile_w, int tile_h,
    int map_x);
```

- [ ] **Step 4: Write store_tiles.cpp**

Create `fpga/hls/vi_sweep/src/store_tiles.cpp`:

```cpp
#include "store_tiles.h"

void store_tile(
    value_t *value_table,
    const value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    int tile_ox, int tile_oy,
    int tile_w, int tile_h,
    int map_x)
{
    STORE_Y: for (int iy = 0; iy < tile_h; iy++) {
        int gy = tile_oy + iy;

        STORE_X: for (int ix = 0; ix < tile_w; ix++) {
            int gx = tile_ox + ix;
            int by = iy + HALO;
            int bx = ix + HALO;

            STORE_T: for (int it = 0; it < N_THETA; it++) {
                #pragma HLS PIPELINE II=1
                int addr = (gy * map_x + gx) * N_THETA + it;
                value_table[addr] = val_buf[by][bx][it];
            }
        }
    }
}
```

- [ ] **Step 5: Commit**

```bash
git add fpga/hls/vi_sweep/src/load_tiles.h fpga/hls/vi_sweep/src/load_tiles.cpp \
        fpga/hls/vi_sweep/src/store_tiles.h fpga/hls/vi_sweep/src/store_tiles.cpp
git commit -m "feat: tile load/store functions for DDR-BRAM transfer"
```

---

### Task 6: HLS Kernel — vi_sweep_top

Top-level function integrating load, compute, and store in a sequential tile loop.

**Files:**
- Create: `fpga/hls/vi_sweep/src/vi_sweep_top.h`
- Create: `fpga/hls/vi_sweep/src/vi_sweep_top.cpp`

- [ ] **Step 1: Write vi_sweep_top.h**

Create `fpga/hls/vi_sweep/src/vi_sweep_top.h`:

```cpp
#pragma once

#include "vi_types.h"

// Top-level HLS kernel: one sweep of Value Iteration over assigned tiles.
//
// value_table:   DDR, [map_y][map_x][N_THETA], ap_uint<16>. Read/write.
// penalty_table: DDR, [map_y][map_x], ap_uint<16>. Read-only.
// trans_table:   DDR, [N_ACTIONS * N_THETA], packed (dix,diy,dit). Read once.
// map_x, map_y:  map dimensions in cells.
// num_tiles_x/y: number of tiles in each direction.
// cu_id:         0 or 1 — selects checkerboard phase.
// max_delta:     output — maximum value change in this sweep.
extern "C" void vi_sweep(
    value_t *value_table,
    const penalty_t *penalty_table,
    const ap_uint<32> *trans_table,
    int map_x,
    int map_y,
    int num_tiles_x,
    int num_tiles_y,
    int cu_id,
    value_t *max_delta);
```

- [ ] **Step 2: Write vi_sweep_top.cpp**

Create `fpga/hls/vi_sweep/src/vi_sweep_top.cpp`:

```cpp
#include "vi_sweep_top.h"
#include "load_tiles.h"
#include "compute_bellman.h"
#include "store_tiles.h"

extern "C" void vi_sweep(
    value_t *value_table,
    const penalty_t *penalty_table,
    const ap_uint<32> *trans_table,
    int map_x,
    int map_y,
    int num_tiles_x,
    int num_tiles_y,
    int cu_id,
    value_t *max_delta)
{
    // -----------------------------------------------------------------------
    // AXI interface pragmas
    // -----------------------------------------------------------------------
    #pragma HLS INTERFACE m_axi port=value_table   bundle=gmem0 offset=slave depth=672000000
    #pragma HLS INTERFACE m_axi port=penalty_table bundle=gmem1 offset=slave depth=11200000
    #pragma HLS INTERFACE m_axi port=trans_table   bundle=gmem1 offset=slave depth=360

    #pragma HLS INTERFACE s_axilite port=map_x
    #pragma HLS INTERFACE s_axilite port=map_y
    #pragma HLS INTERFACE s_axilite port=num_tiles_x
    #pragma HLS INTERFACE s_axilite port=num_tiles_y
    #pragma HLS INTERFACE s_axilite port=cu_id
    #pragma HLS INTERFACE s_axilite port=max_delta
    #pragma HLS INTERFACE s_axilite port=return

    // -----------------------------------------------------------------------
    // Load transition table (once per invocation)
    // -----------------------------------------------------------------------
    offset_t delta_table[N_ACTIONS][N_THETA][3];
    #pragma HLS ARRAY_PARTITION variable=delta_table complete dim=0

    load_transitions(trans_table, delta_table);

    // -----------------------------------------------------------------------
    // BRAM tile buffers
    // -----------------------------------------------------------------------
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA];
    #pragma HLS ARRAY_PARTITION variable=val_buf complete dim=3
    #pragma HLS BIND_STORAGE variable=val_buf type=ram_2p impl=bram

    // 3 copies of penalty for parallel read (2 reads per copy via ram_2p)
    penalty_t pen_buf_0[TILE_H_H][TILE_W_H];
    penalty_t pen_buf_1[TILE_H_H][TILE_W_H];
    penalty_t pen_buf_2[TILE_H_H][TILE_W_H];
    #pragma HLS BIND_STORAGE variable=pen_buf_0 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_1 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_2 type=ram_2p impl=bram

    // -----------------------------------------------------------------------
    // Tile loop: sequential load -> compute -> store
    // -----------------------------------------------------------------------
    value_t global_max_delta = 0;

    TILE_Y: for (int ty = 0; ty < num_tiles_y; ty++) {
        TILE_X: for (int tx = 0; tx < num_tiles_x; tx++) {
            // Checkerboard assignment: process only tiles where (tx+ty)%2 == cu_id
            if ((tx + ty) % 2 != cu_id) continue;

            int tile_ox = tx * TILE_W;
            int tile_oy = ty * TILE_H;

            // Actual tile dimensions (handle map boundary)
            int tile_w = TILE_W;
            if (tile_ox + TILE_W > map_x) tile_w = map_x - tile_ox;
            int tile_h = TILE_H;
            if (tile_oy + TILE_H > map_y) tile_h = map_y - tile_oy;

            // Load tile + halo from DDR
            load_tile(value_table, penalty_table,
                      val_buf, pen_buf_0, pen_buf_1, pen_buf_2,
                      tile_ox, tile_oy, map_x, map_y);

            // Bellman update
            value_t tile_delta;
            compute_bellman(val_buf, pen_buf_0, pen_buf_1, pen_buf_2,
                           delta_table, tile_w, tile_h, tile_delta);

            if (tile_delta > global_max_delta)
                global_max_delta = tile_delta;

            // Store updated tile back to DDR
            store_tile(value_table, val_buf,
                       tile_ox, tile_oy, tile_w, tile_h, map_x);
        }
    }

    *max_delta = global_max_delta;
}
```

- [ ] **Step 3: Commit**

```bash
git add fpga/hls/vi_sweep/src/vi_sweep_top.h fpga/hls/vi_sweep/src/vi_sweep_top.cpp
git commit -m "feat: vi_sweep top-level kernel with tiled processing and checkerboard CU assignment"
```

---

### Task 7: HLS Build Infrastructure

Build scripts following the sindy project pattern.

**Files:**
- Create: `fpga/hls/vi_sweep/hls_config.cfg`
- Create: `fpga/hls/vi_sweep/vitis-comp.json`
- Create: `fpga/scripts/Makefile`
- Create: `fpga/scripts/export_hls_ip.tcl`
- Create: `fpga/scripts/build_vivado.tcl`

- [ ] **Step 1: Write hls_config.cfg**

Create `fpga/hls/vi_sweep/hls_config.cfg`:

```
part=xczu3eg-sbva484-1-i

[hls]
flow_target=vivado
package.output.format=ip_catalog
package.output.syn=false
sim.O=1
syn.top=vi_sweep
syn.file=src/vi_sweep_top.cpp
syn.file=src/compute_bellman.cpp
syn.file=src/load_tiles.cpp
syn.file=src/store_tiles.cpp
tb.file=tb/vi_sweep_tb.cpp
tb.file=tb/vi_reference.cpp
```

- [ ] **Step 2: Write vitis-comp.json**

Create `fpga/hls/vi_sweep/vitis-comp.json`:

```json
{
  "name": "vi_sweep",
  "type": "HLS",
  "configuration": {
    "componentType": "HLS",
    "configFiles": ["hls_config.cfg"],
    "work_dir": "."
  }
}
```

- [ ] **Step 3: Write Makefile**

Create `fpga/scripts/Makefile`:

```makefile
SCRIPTS_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

.PHONY: hls vivado bitstream csim clean_hls clean_vivado clean all

all: bitstream

# C-simulation (testbench)
csim:
	cd $(SCRIPTS_DIR) && vitis_hls -f run_csim.tcl

# HLS synthesis + IP export
hls:
	cd $(SCRIPTS_DIR) && vitis_hls -f export_hls_ip.tcl

# Vivado project creation + synthesis + bitstream
vivado: hls
	vivado -mode batch -source $(SCRIPTS_DIR)/build_vivado.tcl

bitstream: vivado

clean_hls:
	rm -rf $(SCRIPTS_DIR)/hls_build

clean_vivado:
	rm -rf $(SCRIPTS_DIR)/../vivado/ultra96v2/vi_ultra96v2

clean: clean_hls clean_vivado
```

- [ ] **Step 4: Write export_hls_ip.tcl**

Create `fpga/scripts/export_hls_ip.tcl`:

```tcl
# ===========================================================================
# export_hls_ip.tcl — Run Vitis HLS synthesis and export IP
# Usage: vitis_hls -f export_hls_ip.tcl
# ===========================================================================

set script_dir [file normalize [file dirname [info script]]]
set hls_dir    [file normalize "$script_dir/../hls/vi_sweep"]
set ip_dst     [file normalize "$script_dir/../vivado/ultra96v2/ip_repo"]
set part       "xczu3eg-sbva484-1-i"

open_project -reset hls_build
set_top vi_sweep
add_files "$hls_dir/src/vi_sweep_top.cpp"
add_files "$hls_dir/src/compute_bellman.cpp"
add_files "$hls_dir/src/load_tiles.cpp"
add_files "$hls_dir/src/store_tiles.cpp"
add_files -tb "$hls_dir/tb/vi_sweep_tb.cpp"
add_files -tb "$hls_dir/tb/vi_reference.cpp"

open_solution -reset "solution1" -flow_target vivado
set_part $part
create_clock -period 6.67 -name default

# Synthesize
csynth_design

# Export IP
export_design -format ip_catalog -output $ip_dst

close_project
puts "INFO: HLS IP exported to $ip_dst"
```

- [ ] **Step 5: Write run_csim.tcl**

Create `fpga/scripts/run_csim.tcl`:

```tcl
# ===========================================================================
# run_csim.tcl — Run C-simulation only
# Usage: vitis_hls -f run_csim.tcl
# ===========================================================================

set script_dir [file normalize [file dirname [info script]]]
set hls_dir    [file normalize "$script_dir/../hls/vi_sweep"]
set part       "xczu3eg-sbva484-1-i"

open_project -reset hls_build
set_top vi_sweep
add_files "$hls_dir/src/vi_sweep_top.cpp"
add_files "$hls_dir/src/compute_bellman.cpp"
add_files "$hls_dir/src/load_tiles.cpp"
add_files "$hls_dir/src/store_tiles.cpp"
add_files -tb "$hls_dir/tb/vi_sweep_tb.cpp"
add_files -tb "$hls_dir/tb/vi_reference.cpp"

open_solution -reset "solution1" -flow_target vivado
set_part $part
create_clock -period 6.67 -name default

csim_design

close_project
```

- [ ] **Step 6: Write build_vivado.tcl**

Create `fpga/scripts/build_vivado.tcl`:

```tcl
# ===========================================================================
# build_vivado.tcl — Vivado synthesis, implementation, bitstream
# Usage: vivado -mode batch -source build_vivado.tcl
# ===========================================================================

set script_dir   [file normalize [file dirname [info script]]]
set project_dir  [file normalize "$script_dir/../vivado/ultra96v2"]
set project_name "vi_ultra96v2"
set xpr_file     "$project_dir/$project_name/$project_name.xpr"

if {![file exists $xpr_file]} {
    puts "INFO: Project not found, creating..."
    source "$project_dir/create_project.tcl"
} else {
    open_project $xpr_file
}

# Synthesis
reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property STATUS [get_runs synth_1]] != "synth_design Complete!"} {
    error "Synthesis failed"
}

# Implementation + bitstream
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property STATUS [get_runs impl_1]] != "write_bitstream Complete!"} {
    error "Implementation/bitstream failed"
}

puts "INFO: Bitstream generated successfully"
```

- [ ] **Step 7: Commit**

```bash
git add fpga/hls/vi_sweep/hls_config.cfg fpga/hls/vi_sweep/vitis-comp.json \
        fpga/scripts/Makefile fpga/scripts/export_hls_ip.tcl \
        fpga/scripts/run_csim.tcl fpga/scripts/build_vivado.tcl
git commit -m "feat: HLS build infrastructure (Makefile, TCL scripts, config)"
```

---

### Task 8: C-Simulation & Verification

Run the testbench and verify correctness.

**Files:** None (verification only)

- [ ] **Step 1: Run C-simulation**

```bash
cd /c/Users/nop/dev/mywork/value_iteration_fpga/fpga/scripts
make csim
```

Expected output should end with:
```
TESTBENCH PASSED
```

- [ ] **Step 2: Debug if needed**

If testbench fails, common issues to check:
- Boundary handling in `load_tile` (cells outside map must be MAX_VALUE/PENALTY_OBSTACLE)
- Theta wrapping in `compute_bellman` (modular arithmetic for nt)
- Transition table packing/unpacking (bit ordering in pack_transitions vs load_transitions)
- Gauss-Seidel ordering difference between CPU reference and HLS (small value differences are acceptable)

- [ ] **Step 3: Commit any fixes**

```bash
git add -u
git commit -m "fix: C-simulation verification corrections"
```

---

### Task 9: HLS Synthesis & Resource Check

Run HLS synthesis and verify timing/resource targets.

**Files:** None (synthesis only)

- [ ] **Step 1: Run HLS synthesis**

```bash
cd /c/Users/nop/dev/mywork/value_iteration_fpga/fpga/scripts
make hls
```

- [ ] **Step 2: Check synthesis report**

After synthesis, check `hls_build/solution1/syn/report/vi_sweep_csynth.rpt`:

Target checks:
- `compute_bellman` inner loop (LOOP_T) achieves II=1
- Estimated clock period < 6.67ns (150MHz target)
- BRAM usage per CU: < 108 (half of 216 available)
- DSP usage: < 180 (half of 360)

If II > 1 on LOOP_T:
- Check for BRAM port conflicts in `val_buf` reads
- Verify `delta_table` is fully partitioned to registers
- Check that the modulo operation for theta wrapping is resolved

- [ ] **Step 3: Optimize if needed**

If timing fails at 150MHz, try `create_clock -period 10 -name default` (100MHz) in `export_hls_ip.tcl`. Update performance estimates accordingly.

If BRAM exceeds budget, consider reducing `TILE_W`/`TILE_H` to 24 or 16.

- [ ] **Step 4: Commit optimization changes**

```bash
git add -u
git commit -m "fix: HLS synthesis optimizations"
```

---

### Task 10: Vivado Project & Block Design

TCL scripts for Vivado project with 2 Compute Units.

**Files:**
- Create: `fpga/vivado/ultra96v2/create_project.tcl`
- Create: `fpga/vivado/ultra96v2/create_bd.tcl`

- [ ] **Step 1: Write create_project.tcl**

Create `fpga/vivado/ultra96v2/create_project.tcl`:

```tcl
# ===========================================================================
# create_project.tcl — Ultra96-V2 Vivado project with vi_sweep IP
# ===========================================================================

set project_name "vi_ultra96v2"
set project_dir  [file normalize [file dirname [info script]]]
set ip_repo_dir  [file normalize "$project_dir/ip_repo"]
set part         "xczu3eg-sbva484-1-i"

create_project $project_name "$project_dir/$project_name" -part $part -force

set_property board_part Avnet-tria:Ultra96v2:part0:1.3 [current_project]

# Add HLS IP repo (contains vi_sweep IP)
set_property ip_repo_paths $ip_repo_dir [current_project]
update_ip_catalog

# Source block design
source "$project_dir/create_bd.tcl"

# Generate output products
generate_target all [get_files vi_bd.bd]

# Create HDL wrapper
make_wrapper -files [get_files vi_bd.bd] -top
add_files -norecurse [glob "$project_dir/$project_name/$project_name.gen/sources_1/bd/vi_bd/hdl/vi_bd_wrapper.v"]
update_compile_order -fileset sources_1

puts "INFO: Project created at $project_dir/$project_name"
```

- [ ] **Step 2: Write create_bd.tcl (2 CU block design)**

Create `fpga/vivado/ultra96v2/create_bd.tcl`:

```tcl
# ===========================================================================
# create_bd.tcl — Block Design: Zynq PS + 2x vi_sweep HLS IP
# ===========================================================================

create_bd_design "vi_bd"

# --- Zynq UltraScale+ PS ---
set zynq [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 zynq_ps]

apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
    -config {apply_board_preset "1"} $zynq

# Enable HP0 for data, disable unused HPM1
set_property -dict [list \
    CONFIG.PSU__USE__S_AXI_GP2 {1} \
    CONFIG.PSU__SAXIGP2__DATA_WIDTH {128} \
    CONFIG.PSU__USE__M_AXI_GP1 {0} \
] $zynq

# --- 2x vi_sweep HLS IPs ---
set cu0 [create_bd_cell -type ip -vlnv xilinx.com:hls:vi_sweep:1.0 vi_sweep_cu0]
set cu1 [create_bd_cell -type ip -vlnv xilinx.com:hls:vi_sweep:1.0 vi_sweep_cu1]

# --- Data SmartConnect (4 AXI masters -> 1 HP slave) ---
set data_smc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 data_smc]
set_property CONFIG.NUM_SI {4} $data_smc

# --- Control SmartConnect (1 GP master -> 2 control slaves) ---
set ctrl_smc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 ctrl_smc]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {2}] $ctrl_smc

# --- Reset ---
set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0]

# --- Clock and reset wiring ---
set clk [get_bd_pins zynq_ps/pl_clk0]
set rstn [get_bd_pins proc_sys_reset_0/peripheral_aresetn]

connect_bd_net $clk \
    [get_bd_pins data_smc/aclk] \
    [get_bd_pins ctrl_smc/aclk] \
    [get_bd_pins vi_sweep_cu0/ap_clk] \
    [get_bd_pins vi_sweep_cu1/ap_clk] \
    [get_bd_pins proc_sys_reset_0/slowest_sync_clk] \
    [get_bd_pins zynq_ps/saxihp0_fpd_aclk] \
    [get_bd_pins zynq_ps/maxihpm0_fpd_aclk]

connect_bd_net [get_bd_pins zynq_ps/pl_resetn0] [get_bd_pins proc_sys_reset_0/ext_reset_in]

connect_bd_net $rstn \
    [get_bd_pins data_smc/aresetn] \
    [get_bd_pins ctrl_smc/aresetn] \
    [get_bd_pins vi_sweep_cu0/ap_rst_n] \
    [get_bd_pins vi_sweep_cu1/ap_rst_n]

# --- Control path: GP0 -> ctrl_smc -> CU0/CU1 control ---
connect_bd_intf_net [get_bd_intf_pins zynq_ps/M_AXI_HPM0_FPD] [get_bd_intf_pins ctrl_smc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins ctrl_smc/M00_AXI] [get_bd_intf_pins vi_sweep_cu0/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins ctrl_smc/M01_AXI] [get_bd_intf_pins vi_sweep_cu1/s_axi_control]

# --- Data path: CU0 gmem0/gmem1 + CU1 gmem0/gmem1 -> data_smc -> HP0 ---
connect_bd_intf_net [get_bd_intf_pins vi_sweep_cu0/m_axi_gmem0] [get_bd_intf_pins data_smc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins vi_sweep_cu0/m_axi_gmem1] [get_bd_intf_pins data_smc/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins vi_sweep_cu1/m_axi_gmem0] [get_bd_intf_pins data_smc/S02_AXI]
connect_bd_intf_net [get_bd_intf_pins vi_sweep_cu1/m_axi_gmem1] [get_bd_intf_pins data_smc/S03_AXI]
connect_bd_intf_net [get_bd_intf_pins data_smc/M00_AXI] [get_bd_intf_pins zynq_ps/S_AXI_HP0_FPD]

# --- Address assignment ---
assign_bd_address [get_bd_addr_segs zynq_ps/SAXIGP2/HP0_DDR_LOW]
assign_bd_address [get_bd_addr_segs vi_sweep_cu0/s_axi_control/Reg]
assign_bd_address [get_bd_addr_segs vi_sweep_cu1/s_axi_control/Reg]

validate_bd_design
save_bd_design

puts "INFO: Block design 'vi_bd' created (2 CU configuration)"
```

- [ ] **Step 3: Commit**

```bash
git add fpga/vivado/ultra96v2/create_project.tcl fpga/vivado/ultra96v2/create_bd.tcl
git commit -m "feat: Vivado project and block design with 2 Compute Units"
```

---

### Task 11: Bitstream Generation

Build the complete FPGA design.

**Files:** None (build only)

- [ ] **Step 1: Run full build**

```bash
cd /c/Users/nop/dev/mywork/value_iteration_fpga/fpga/scripts
make all
```

This runs: HLS synthesis -> IP export -> Vivado synthesis -> implementation -> bitstream.
Expected time: 30-60 minutes.

- [ ] **Step 2: Check implementation report**

Verify in Vivado:
- Timing: WNS (Worst Negative Slack) >= 0 at target frequency
- Resource utilization: BRAM < 90%, LUT < 80%, DSP < 80%
- If timing fails, reduce clock (change `create_clock -period 10` in export_hls_ip.tcl for 100MHz)

- [ ] **Step 3: Copy bitstream artifacts for PYNQ**

```bash
# Find the generated files
find fpga/vivado/ultra96v2/vi_ultra96v2 -name "*.bit" -o -name "*.hwh" | head -5
# Copy to pynq directory
cp fpga/vivado/ultra96v2/vi_ultra96v2/vi_ultra96v2.runs/impl_1/vi_bd_wrapper.bit fpga/pynq/
cp fpga/vivado/ultra96v2/vi_ultra96v2/vi_ultra96v2.gen/sources_1/bd/vi_bd/hw_handoff/vi_bd.hwh fpga/pynq/vi_bd_wrapper.hwh
```

---

### Task 12: PYNQ Overlay & Demo

Python overlay for hardware validation on Ultra96-V2.

**Files:**
- Create: `fpga/pynq/vi_overlay.py`
- Create: `fpga/pynq/demo_vi.py`

- [ ] **Step 1: Write vi_overlay.py**

Create `fpga/pynq/vi_overlay.py`:

```python
"""Value Iteration FPGA overlay for PYNQ on Ultra96-V2.

Usage:
    Copy vi_bd_wrapper.bit, vi_bd_wrapper.hwh, and this file to Ultra96-V2.

    from vi_overlay import VIOverlay
    vi = VIOverlay("vi_bd_wrapper.bit")
    sweeps = vi.run(value_table, penalty_table, trans_table, map_x, map_y)
"""

import struct
import numpy as np
from pynq import Overlay, allocate

# AXI-Lite register offsets (from HLS-generated driver header)
# These will be confirmed after HLS synthesis from the *_hw.h file.
AP_CTRL          = 0x00
ADDR_VALUE_TABLE = 0x10  # 64-bit address
ADDR_PENALTY     = 0x1C  # 64-bit address
ADDR_TRANS       = 0x28  # 64-bit address
ADDR_MAP_X       = 0x34
ADDR_MAP_Y       = 0x3C
ADDR_NUM_TILES_X = 0x44
ADDR_NUM_TILES_Y = 0x4C
ADDR_CU_ID       = 0x54
ADDR_MAX_DELTA   = 0x5C

TILE_W = 32
TILE_H = 32
N_THETA = 60


def _write_addr64(ip, offset, addr):
    ip.write(offset, addr & 0xFFFFFFFF)
    ip.write(offset + 4, (addr >> 32) & 0xFFFFFFFF)


class VIOverlay:
    def __init__(self, bitstream_path: str):
        self.ol = Overlay(bitstream_path)
        self.cu0 = self.ol.vi_sweep_cu0
        self.cu1 = self.ol.vi_sweep_cu1

    def run(
        self,
        value_np: np.ndarray,
        penalty_np: np.ndarray,
        trans_np: np.ndarray,
        map_x: int,
        map_y: int,
        threshold: int = 0,
        max_sweeps: int = 200,
    ) -> int:
        """Run Value Iteration on FPGA until convergence.

        Args:
            value_np: shape (map_y, map_x, N_THETA), uint16. Modified in-place.
            penalty_np: shape (map_y, map_x), uint16.
            trans_np: shape (360,), uint32. Packed transitions.
            map_x, map_y: map dimensions.
            threshold: convergence threshold for max_delta.
            max_sweeps: maximum sweep iterations.

        Returns:
            Number of sweeps executed.
        """
        num_tiles_x = (map_x + TILE_W - 1) // TILE_W
        num_tiles_y = (map_y + TILE_H - 1) // TILE_H

        # Allocate contiguous DMA buffers
        val_buf = allocate(shape=value_np.shape, dtype=np.uint16)
        pen_buf = allocate(shape=penalty_np.shape, dtype=np.uint16)
        trans_buf = allocate(shape=trans_np.shape, dtype=np.uint32)

        np.copyto(val_buf, value_np)
        np.copyto(pen_buf, penalty_np)
        np.copyto(trans_buf, trans_np)
        val_buf.sync_to_device()
        pen_buf.sync_to_device()
        trans_buf.sync_to_device()

        for cu in [self.cu0, self.cu1]:
            _write_addr64(cu, ADDR_VALUE_TABLE, val_buf.device_address)
            _write_addr64(cu, ADDR_PENALTY, pen_buf.device_address)
            _write_addr64(cu, ADDR_TRANS, trans_buf.device_address)
            cu.write(ADDR_MAP_X, map_x)
            cu.write(ADDR_MAP_Y, map_y)
            cu.write(ADDR_NUM_TILES_X, num_tiles_x)
            cu.write(ADDR_NUM_TILES_Y, num_tiles_y)

        sweep = 0
        for sweep in range(max_sweeps):
            # Start both CUs (checkerboard)
            self.cu0.write(ADDR_CU_ID, 0)
            self.cu1.write(ADDR_CU_ID, 1)
            self.cu0.write(AP_CTRL, 0x01)
            self.cu1.write(AP_CTRL, 0x01)

            # Wait for both to finish
            while not (self.cu0.read(AP_CTRL) & 0x02):
                pass
            while not (self.cu1.read(AP_CTRL) & 0x02):
                pass

            d0 = self.cu0.read(ADDR_MAX_DELTA)
            d1 = self.cu1.read(ADDR_MAX_DELTA)
            max_delta = max(d0, d1)

            if max_delta <= threshold:
                break

        # Copy results back
        val_buf.sync_from_device()
        np.copyto(value_np, val_buf)

        val_buf.freebuffer()
        pen_buf.freebuffer()
        trans_buf.freebuffer()

        return sweep + 1
```

- [ ] **Step 2: Write demo_vi.py**

Create `fpga/pynq/demo_vi.py`:

```python
"""Demo: run Value Iteration on a small test map via FPGA."""

import numpy as np
import math
import time
from vi_overlay import VIOverlay

N_ACTIONS = 6
N_THETA = 60
MAX_VALUE = 0xFFFF
PENALTY_OBSTACLE = 0xFFFF
PENALTY_GOAL = 0xFFFE

ACTION_FW = [0.3, -0.2, 0.0, 0.0, 0.3, 0.3]
ACTION_ROT = [0.0, 0.0, 20.0, -20.0, 20.0, -20.0]


def compute_transitions(xy_res: float) -> np.ndarray:
    t_res = 360.0 / N_THETA
    packed = np.zeros(N_ACTIONS * N_THETA, dtype=np.uint32)

    for a in range(N_ACTIONS):
        for it in range(N_THETA):
            theta = (it * t_res + t_res * 0.5) * math.pi / 180.0
            dx = ACTION_FW[a] * math.cos(theta)
            dy = ACTION_FW[a] * math.sin(theta)
            dix = int(math.floor(dx / xy_res))
            diy = int(math.floor(dy / xy_res))

            new_theta = it * t_res + t_res * 0.5 + ACTION_ROT[a]
            while new_theta < 0: new_theta += 360
            while new_theta >= 360: new_theta -= 360
            new_it = int(math.floor(new_theta / t_res))
            dit = new_it - it
            if dit > N_THETA // 2: dit -= N_THETA
            if dit < -N_THETA // 2: dit += N_THETA

            w = (dix & 0xFF) | ((diy & 0xFF) << 8) | ((dit & 0xFF) << 16)
            packed[a * N_THETA + it] = w

    return packed


def main():
    MAP_X, MAP_Y = 40, 40
    XY_RES = 0.05

    print(f"Map: {MAP_X}x{MAP_Y}, resolution={XY_RES}m")

    # Build penalty table
    penalty = np.zeros((MAP_Y, MAP_X), dtype=np.uint16)
    # Border obstacles
    penalty[0, :] = PENALTY_OBSTACLE
    penalty[-1, :] = PENALTY_OBSTACLE
    penalty[:, 0] = PENALTY_OBSTACLE
    penalty[:, -1] = PENALTY_OBSTACLE
    # Goal at (30, 30)
    penalty[30, 30] = PENALTY_GOAL

    # Value table
    value = np.full((MAP_Y, MAP_X, N_THETA), MAX_VALUE, dtype=np.uint16)
    value[30, 30, :] = 0

    # Transitions
    trans = compute_transitions(XY_RES)

    print("Loading overlay...")
    vi = VIOverlay("vi_bd_wrapper.bit")

    print("Running VI on FPGA...")
    t0 = time.time()
    sweeps = vi.run(value, penalty, trans, MAP_X, MAP_Y, threshold=0)
    elapsed = time.time() - t0

    print(f"Converged in {sweeps} sweeps, {elapsed:.3f}s")
    print(f"Value at (5,5,0): {value[5, 5, 0]}")
    print(f"Value at (20,20,0): {value[20, 20, 0]}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Commit**

```bash
git add fpga/pynq/vi_overlay.py fpga/pynq/demo_vi.py
git commit -m "feat: PYNQ overlay and demo for hardware validation"
```

---

## Optimization Notes (Post Phase 1-2)

After the baseline implementation is verified, consider these optimizations:

1. **DATAFLOW pipeline**: Add ping-pong BRAM buffers and `#pragma HLS DATAFLOW` to overlap load/compute/store across tiles. Saves ~29% per sweep but doubles BRAM usage. Only if BRAM budget allows after synthesis.

2. **DDR burst width**: Widen AXI master data width to 128 or 256 bits for higher DDR throughput. Requires packing multiple `value_t` entries per beat.

3. **Sweep direction alternation**: Alternate tile processing order between sweeps (forward/reverse in X and Y) to accelerate convergence. Modify the `TILE_X`/`TILE_Y` loop direction based on a sweep counter passed via AXI-Lite.

4. **Warm start**: ARM passes previous value table as initial state when goal changes. Reduces convergence sweeps significantly.

5. **Bit-width tuning**: If 16-bit causes precision issues, switch to `ap_uint<24>`. DDR usage grows from 1.34GB to 2.0GB. BRAM per tile grows from 232KB to 348KB — tight for 2 CUs. May need to reduce to 1 CU.

---

## Register Offset Verification

After HLS synthesis, verify the actual AXI-Lite register offsets from the generated header file:

```
fpga/scripts/hls_build/solution1/impl/ip/drivers/vi_sweep_v1_0/src/xvi_sweep_hw.h
```

Update `fpga/pynq/vi_overlay.py` `ADDR_*` constants to match.
