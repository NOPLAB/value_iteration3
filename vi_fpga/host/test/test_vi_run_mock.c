/* test_vi_run_mock.c — exercises libvi_sweep against mock ops. */

#include "vi_assert.h"
#include "libvi_sweep.h"
#include "vi_device.h"

#include <stdlib.h>
#include <string.h>

static void init_tiny_map(vi_device_t *dev, int w, int h, int gx, int gy) {
    (void)h;
    size_t nv, np, nt;
    uint16_t *val = vi_value_buffer(dev,   &nv);
    uint16_t *pen = vi_penalty_buffer(dev, &np);
    uint32_t *tr  = vi_trans_buffer(dev,   &nt);

    /* value: all MAX except goal */
    for (size_t i = 0; i < nv; i++) val[i] = 0xFFFF;
    /* pen: 0 everywhere except goal */
    for (size_t i = 0; i < np; i++) pen[i] = 0;

    /* goal */
    pen[gy * w + gx] = 0xFFFE;
    for (int it = 0; it < VI_N_THETA; it++)
        val[(gy * w + gx) * VI_N_THETA + it] = 0;

    /* trivial transitions: action 0 = +x, action 1 = -x, others = no-op.
       Packed uint32: byte0=dix, byte1=diy, byte2=dit */
    for (size_t i = 0; i < nt; i++) tr[i] = 0;
    for (int it = 0; it < VI_N_THETA; it++) {
        tr[0 * VI_N_THETA + it] = ((uint32_t)0x01);         /* dix=+1 */
        tr[1 * VI_N_THETA + it] = ((uint32_t)0xFF);         /* dix=-1 (int8_t sign) */
        /* actions 2-5: no-op */
    }
}

int main(void) {
    void *ctx = vi_mock_ctx_new();
    VI_ASSERT(ctx != NULL);

    vi_device_t *dev = vi_open(&vi_mock_ops, ctx);
    VI_ASSERT(dev != NULL);

    int W = 16, H = 16;
    init_tiny_map(dev, W, H, 8, 8);

    vi_run_config_t cfg = { .map_x = W, .map_y = H,
                            .threshold = 0, .max_sweeps = 100 };
    vi_run_stats_t stats = {0};
    int rc = vi_run_until_converged(dev, &cfg, &stats);
    VI_ASSERT_EQ(rc, VI_OK);
    VI_ASSERT(stats.converged == 1);
    VI_ASSERT(stats.sweeps > 0);
    VI_ASSERT(stats.sweeps <= 100);

    /* Goal should still be 0 */
    size_t nv;
    uint16_t *val = vi_value_buffer(dev, &nv);
    VI_ASSERT_EQ(val[(8 * W + 8) * VI_N_THETA + 0], 0);

    /* Non-goal cell on the same row as goal (reachable via dix=+/-1) should
       be strictly less than MAX after convergence. */
    VI_ASSERT(val[(8 * W + 4) * VI_N_THETA + 0] < 0xFFFF);

    vi_close(dev);
    vi_mock_ctx_free(ctx);
    VI_TEST_MAIN_END();
}
