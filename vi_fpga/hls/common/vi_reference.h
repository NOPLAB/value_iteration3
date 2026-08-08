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
