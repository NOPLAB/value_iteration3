#include "vi_assert.h"
#include "libvi_sweep.h"
#include "vi_device.h"

#include <stdlib.h>
#include <string.h>

int main(void) {
    void *ctx = vi_mock_ctx_new();
    vi_device_t *dev = vi_open(&vi_mock_ops, ctx);
    VI_ASSERT(dev != NULL);

    int W = 4, H = 4;
    size_t nv, np, nt;
    uint16_t *val = vi_value_buffer(dev, &nv);
    uint16_t *pen = vi_penalty_buffer(dev, &np);
    uint32_t *tr  = vi_trans_buffer(dev, &nt);

    /* All cells traversable */
    for (size_t i = 0; i < nv; i++) val[i] = 0xFFFF;
    for (size_t i = 0; i < np; i++) pen[i] = 0;

    /* Gradient: val[y][x][theta=0] = (W-1-x) * 10, so x=W-1 has the lowest
       value (0) and x=0 has the highest (30). Best action should move +x. */
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            val[((size_t)y * W + x) * VI_N_THETA + 0] = (uint16_t)((W - 1 - x) * 10);

    /* Transitions: action 3 = dix=+1, others = no-op (stay at self). */
    for (size_t i = 0; i < nt; i++) tr[i] = 0;
    for (int it = 0; it < VI_N_THETA; it++)
        tr[3 * VI_N_THETA + it] = 0x000001;   /* dix=+1, diy=0, dit=0 */

    uint8_t *act = calloc((size_t)W * H * VI_N_THETA, 1);
    int rc = vi_compute_action_table(dev, W, H, act);
    VI_ASSERT_EQ(rc, VI_OK);

    /* Cell (x=0, y=0, theta=0): action 3 (+x, 20) beats action 0 (self, 30). */
    VI_ASSERT_EQ(act[((size_t)0 * W + 0) * VI_N_THETA + 0], 3);
    /* Cell (x=W-1, y=0, theta=0): action 3 out-of-bounds -> action 0 wins. */
    VI_ASSERT_EQ(act[((size_t)0 * W + (W-1)) * VI_N_THETA + 0], 0);

    free(act);
    vi_close(dev);
    vi_mock_ctx_free(ctx);
    VI_TEST_MAIN_END();
}
