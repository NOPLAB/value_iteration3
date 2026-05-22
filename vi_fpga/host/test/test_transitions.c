#include "vi_assert.h"
#include "transitions.h"

#include <stdlib.h>

int main(void) {
    uint32_t tr[VI_N_ACTIONS * VI_N_THETA];
    transitions_compute(0.05, tr);

    /* Action 0 (forward 0.3m) at theta=0 cell-center (~3 deg) should produce
       dix = 5 or 6 (0.3 * cos(3deg) / 0.05 ≈ 5.99). */
    uint32_t t = tr[0 * VI_N_THETA + 0];
    int8_t dix = (int8_t)(t & 0xFF);
    VI_ASSERT(dix == 5 || dix == 6);

    /* Action 2 (left, +20deg) at theta=0 should have dix=0, diy=0 but dit!=0. */
    t = tr[2 * VI_N_THETA + 0];
    dix = (int8_t)(t & 0xFF);
    int8_t diy = (int8_t)((t >> 8) & 0xFF);
    int8_t dit = (int8_t)((t >> 16) & 0xFF);
    VI_ASSERT_EQ(dix, 0);
    VI_ASSERT_EQ(diy, 0);
    VI_ASSERT(dit != 0);

    VI_TEST_MAIN_END();
}
