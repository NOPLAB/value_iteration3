#pragma once

#include <cstdint>

// CPU reference for the HLS testbenches. The per-cell Bellman update and the
// transition table are the *shared* C implementations (driver/uio/
// vi_bellman_cell.h, host/src/transitions.c), included verbatim by
// vi_reference.cpp so the 16-bit contract cannot drift between host,
// mock device and HLS reference.

namespace vi_ref {

constexpr int N_ACTIONS = 6;
constexpr int N_THETA   = 60;
constexpr uint16_t MAX_VALUE        = 0xFFFF;
constexpr uint16_t PENALTY_OBSTACLE = 0xFFFF;
constexpr uint16_t PENALTY_GOAL     = 0xFFFE;

// Deterministic transition table for the 6 fixed actions (spec §2.3),
// packed per (action, theta) as uint32: byte0=dix, byte1=diy, byte2=dit.
// out must have N_ACTIONS * N_THETA entries.
void compute_transitions(double xy_resolution, uint32_t *out);

// Gauss-Seidel value iteration sweeps until max delta <= threshold.
// Returns the number of sweeps executed.
// value_table: [map_y][map_x][N_THETA] row-major; goal cells = 0, others MAX.
// penalty_table: [map_y][map_x]; PENALTY_OBSTACLE / PENALTY_GOAL / 0..0xFFFD.
int run_vi(uint16_t *value_table, const uint16_t *penalty_table,
           const uint32_t *trans, int map_x, int map_y,
           uint16_t threshold, int max_sweeps);

// Shared testbench map: border + L-shaped obstacles, safety penalty 100 near
// obstacles, one goal cell; value init MAX everywhere except goal = 0.
void build_test_map(uint16_t *penalty, uint16_t *value,
                    int map_x, int map_y, int goal_x, int goal_y);

} // namespace vi_ref
