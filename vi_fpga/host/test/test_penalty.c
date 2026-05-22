#include "vi_assert.h"
#include "penalty.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int main(void) {
    pgm_map_t m = {0};
    m.w = 5; m.h = 5;
    m.occupied_thresh = 0.65;
    m.pixels = calloc(25, 1);
    /* center obstacle */
    m.pixels[2*5 + 2] = 255;

    uint16_t pen[25] = {0};
    penalty_build(&m, 2, 0, 0, pen);

    VI_ASSERT_EQ(pen[2*5 + 2], 0xFFFF);  /* center = obstacle */
    VI_ASSERT(pen[2*5 + 1] > 0);          /* adjacent cell has some penalty */
    VI_ASSERT_EQ(pen[0*5 + 0], 0xFFFE);   /* goal */
    VI_ASSERT_EQ(pen[4*5 + 4], 0);        /* far corner = free */

    free(m.pixels);
    VI_TEST_MAIN_END();
}
