#pragma once

#include <ap_int.h>

// --- Data types (same as tile-based kernel) ---
typedef ap_uint<16> value_t;
typedef ap_uint<16> penalty_t;
typedef ap_int<8>   offset_t;

// --- Constants ---
constexpr int N_ACTIONS   = 6;
constexpr int N_THETA     = 60;
constexpr int HALO_MAX    = 6;     // max |dix|, |diy| at 0.05m
constexpr int WINDOW_ROWS = 2 * HALO_MAX + 1;  // 13
constexpr int STRIP_W_MAX = 145;   // max for 2 CUs: 13*(145+12)=2041 ≤ 2048 BRAM36
constexpr int BUF_W       = STRIP_W_MAX + 2 * HALO_MAX;  // 268

constexpr int TRANS_TABLE_SIZE = N_ACTIONS * N_THETA;  // 360

// Sentinel values
const value_t   MAX_VALUE         = 0xFFFF;
const penalty_t PENALTY_OBSTACLE  = 0xFFFF;
const penalty_t PENALTY_GOAL      = 0xFFFE;
