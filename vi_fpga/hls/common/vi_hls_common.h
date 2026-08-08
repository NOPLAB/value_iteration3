#pragma once

#include <ap_int.h>

// ---------------------------------------------------------------------------
// Shared 16-bit data contract for both HLS kernels (tile and stream).
// Keep in lockstep with driver/uio/vi_bellman_cell.h, host/src/penalty.c,
// vi_matlab vi_params.m and vi_bench params.rs (see CLAUDE.md).
// ---------------------------------------------------------------------------
typedef ap_uint<16> value_t;
typedef ap_uint<16> penalty_t;
typedef ap_int<8>   offset_t;

constexpr int N_ACTIONS = 6;
constexpr int N_THETA   = 60;

// Halo width shared by both kernels: max |dix|, |diy| of any transition
// (0.3 m max forward / 0.05 m resolution = 6 cells).
constexpr int HALO_MAX  = 6;

// Transition table: packed as (dix, diy, dit) in one 32-bit word
// Layout: bits [7:0]=dix, [15:8]=diy, [23:16]=dit, [31:24]=reserved
constexpr int TRANS_TABLE_SIZE = N_ACTIONS * N_THETA;  // 360

// Sentinel values (ap_uint is not literal type, so use const not constexpr)
const value_t   MAX_VALUE         = 0xFFFF;
const penalty_t PENALTY_OBSTACLE  = 0xFFFF;  // impassable cell
const penalty_t PENALTY_GOAL      = 0xFFFE;  // goal cell — value stays 0

// Saturating add for cost computation.
// If either input is a sentinel (MAX_VALUE / PENALTY_OBSTACLE),
// returns MAX_VALUE. PENALTY_GOAL read as a neighbor's penalty is 0
// (load-bearing: keeps the goal cell's value pinned at 0 — do not simplify).
static inline value_t cost_of(value_t nv, penalty_t np_raw) {
    if (nv == MAX_VALUE || np_raw == PENALTY_OBSTACLE) return MAX_VALUE;
    penalty_t np = (np_raw == PENALTY_GOAL) ? (penalty_t)0 : np_raw;
    ap_uint<17> sum = (ap_uint<17>)nv + (ap_uint<17>)np;
    return (sum >= (ap_uint<17>)MAX_VALUE)
           ? (value_t)(MAX_VALUE - 1) : (value_t)sum;
}

// Load transition table from DDR into register array (once per invocation).
static inline void load_transitions(
    const ap_uint<32> *trans_table,
    offset_t delta_table[N_ACTIONS][N_THETA][3])
{
    #pragma HLS INLINE off
    LOAD_TRANS: for (int i = 0; i < TRANS_TABLE_SIZE; i++) {
        #pragma HLS PIPELINE II=1
        ap_uint<32> w = trans_table[i];
        int a = i / N_THETA;
        int t = i % N_THETA;
        delta_table[a][t][0] = (offset_t)(w(7,  0));   // dix
        delta_table[a][t][1] = (offset_t)(w(15, 8));   // diy
        delta_table[a][t][2] = (offset_t)(w(23, 16));  // dit
    }
}
