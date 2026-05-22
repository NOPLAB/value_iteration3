#ifndef TRANSITIONS_H
#define TRANSITIONS_H

#include <stdint.h>
#include "libvi_sweep.h"

/* Compute deterministic (dix, diy, dit) for each (action, theta) pair
   and pack as uint32: byte0=dix, byte1=diy, byte2=dit.
   out must have VI_N_ACTIONS*VI_N_THETA entries. */
void transitions_compute(double xy_resolution, uint32_t *out);

#endif
