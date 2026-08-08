#pragma once

#include "../../common/vi_hls_common.h"

// ---------------------------------------------------------------------------
// Tile geometry
// ---------------------------------------------------------------------------
constexpr int TILE_W    = 32;
constexpr int TILE_H    = 32;
constexpr int HALO      = 6;   // max forward 0.3m / 0.05m resolution = 6 cells
constexpr int TILE_W_H  = TILE_W + 2 * HALO;  // 44
constexpr int TILE_H_H  = TILE_H + 2 * HALO;  // 44
