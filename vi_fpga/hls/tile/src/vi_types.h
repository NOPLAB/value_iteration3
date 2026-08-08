#pragma once

#include "../../common/vi_hls_common.h"

// ---------------------------------------------------------------------------
// Tile geometry
// ---------------------------------------------------------------------------
constexpr int TILE_W    = 32;
constexpr int TILE_H    = 32;
constexpr int HALO      = HALO_MAX;  // shared halo width (vi_hls_common.h)
constexpr int TILE_W_H  = TILE_W + 2 * HALO;  // 44
constexpr int TILE_H_H  = TILE_H + 2 * HALO;  // 44
