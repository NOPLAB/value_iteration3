#pragma once

#include "../../common/vi_hls_common.h"

// --- Streaming-kernel constants (HALO_MAX comes from vi_hls_common.h) ---
constexpr int WINDOW_ROWS = 2 * HALO_MAX + 1;  // 13
constexpr int STRIP_W_MAX = 145;   // max for 2 CUs: 13*(145+12)=2041 ≤ 2048 BRAM36
constexpr int BUF_W       = STRIP_W_MAX + 2 * HALO_MAX;  // 157
