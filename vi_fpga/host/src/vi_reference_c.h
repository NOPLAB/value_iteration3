#ifndef VI_REFERENCE_C_H
#define VI_REFERENCE_C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Run VI on the CPU reference solver until convergence.
   value, penalty, trans follow the same layout libvi_sweep uses. */
int vi_reference_run(uint16_t *value, const uint16_t *penalty,
                     const uint32_t *trans,
                     int map_x, int map_y,
                     uint16_t threshold, int max_sweeps);

#ifdef __cplusplus
}
#endif
#endif
