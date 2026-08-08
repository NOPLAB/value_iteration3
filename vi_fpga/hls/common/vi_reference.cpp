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
                        uint16_t np_raw = penalty_table[ny * map_x + nx];

                        if (nv == MAX_VALUE || np_raw == PENALTY_OBSTACLE) continue;

                        // PENALTY_GOAL marks the goal cell — penalty to enter is 0
                        uint16_t np = (np_raw == PENALTY_GOAL) ? 0 : np_raw;

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
