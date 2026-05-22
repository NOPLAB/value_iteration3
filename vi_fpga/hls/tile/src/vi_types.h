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

// Sentinel values (ap_uint is not literal type, so use const not constexpr)
const value_t   MAX_VALUE        = 0xFFFF;
const penalty_t PENALTY_OBSTACLE  = 0xFFFF;  // impassable cell
const penalty_t PENALTY_GOAL      = 0xFFFE;  // goal cell — value stays 0

// Transition table: packed as (dix, diy, dit) in one 32-bit word
// Layout: bits [7:0]=dix, [15:8]=diy, [23:16]=dit, [31:24]=reserved
// Total entries: N_ACTIONS * N_THETA = 360
constexpr int TRANS_TABLE_SIZE = N_ACTIONS * N_THETA;
