#include "transitions.h"

#define _USE_MATH_DEFINES
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Spec §2.3 */
static const double ACTION_FW[VI_N_ACTIONS]  = { 0.3, -0.2, 0.0, 0.0, 0.3, 0.3 };
static const double ACTION_ROT[VI_N_ACTIONS] = { 0.0, 0.0, 20.0, -20.0, 20.0, -20.0 };

void transitions_compute(double xy_resolution, uint32_t *out) {
    double t_res = 360.0 / VI_N_THETA;
    for (int a = 0; a < VI_N_ACTIONS; a++) {
        for (int it = 0; it < VI_N_THETA; it++) {
            double theta_deg = it * t_res + t_res * 0.5;
            double theta_rad = theta_deg * M_PI / 180.0;
            double dx = ACTION_FW[a] * cos(theta_rad);
            double dy = ACTION_FW[a] * sin(theta_rad);
            int dix = (int)floor(dx / xy_resolution);
            int diy = (int)floor(dy / xy_resolution);

            double nt = theta_deg + ACTION_ROT[a];
            while (nt < 0) nt += 360.0;
            while (nt >= 360.0) nt -= 360.0;
            int new_it = (int)floor(nt / t_res);
            int dit = new_it - it;
            if (dit >  VI_N_THETA / 2) dit -= VI_N_THETA;
            if (dit < -VI_N_THETA / 2) dit += VI_N_THETA;

            uint32_t packed = ((uint32_t)(dix & 0xFF))
                            | ((uint32_t)(diy & 0xFF) << 8)
                            | ((uint32_t)(dit & 0xFF) << 16);
            out[a * VI_N_THETA + it] = packed;
        }
    }
}
