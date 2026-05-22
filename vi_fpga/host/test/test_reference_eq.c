#include "vi_assert.h"
#include "libvi_sweep.h"
#include "vi_device.h"
#include "vi_reference_c.h"
#include "transitions.h"

#include <stdlib.h>
#include <string.h>

int main(void) {
    int W = 12, H = 12;

    void *ctx = vi_mock_ctx_new();
    vi_device_t *dev = vi_open(&vi_mock_ops, ctx);

    size_t nv, np, nt;
    uint16_t *val = vi_value_buffer(dev, &nv);
    uint16_t *pen = vi_penalty_buffer(dev, &np);
    uint32_t *tr  = vi_trans_buffer(dev, &nt);

    /* Clear only the part we care about (first W*H stride entries). */
    for (size_t i = 0; i < (size_t)W * H * VI_N_THETA; i++) val[i] = 0xFFFF;
    for (size_t i = 0; i < (size_t)W * H; i++) pen[i] = 0;

    /* Goal at (6, 6) using stride W */
    pen[6 * W + 6] = 0xFFFE;
    for (int it = 0; it < VI_N_THETA; it++)
        val[(6 * W + 6) * VI_N_THETA + it] = 0;

    transitions_compute(0.05, tr);

    size_t n = (size_t)W * H * VI_N_THETA;
    uint16_t *ref_val = malloc(n * sizeof(uint16_t));
    memcpy(ref_val, val, n * sizeof(uint16_t));
    vi_reference_run(ref_val, pen, tr, W, H, 0, 200);

    vi_run_config_t cfg = { W, H, 0, 200 };
    vi_run_stats_t stats = {0};
    int rc = vi_run_until_converged(dev, &cfg, &stats);
    VI_ASSERT_EQ(rc, VI_OK);

    int mismatches = 0;
    for (size_t i = 0; i < n; i++)
        if (ref_val[i] != val[i]) mismatches++;
    VI_ASSERT_EQ(mismatches, 0);

    free(ref_val);
    vi_close(dev);
    vi_mock_ctx_free(ctx);
    VI_TEST_MAIN_END();
}
