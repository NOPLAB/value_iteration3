/* vi_device_mock.c — simulates vi_sweep FPGA IP in software.
   Used for host unit testing of libvi_sweep. */

#include "vi_device.h"
#include "libvi_sweep.h"
#include "vi_bellman_cell.h"
#include "vi_regs.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define MOCK_REG_WORDS  64  /* 0x100 bytes of control space */

/* Shared physical backing (same for both CUs) */
typedef struct {
    uint32_t  regs[VI_NUM_CU][MOCK_REG_WORDS];

    /* Simulated DDR buffers */
    uint16_t *value_buf;   size_t value_size;   uint64_t value_phys;
    uint16_t *pen_buf;     size_t pen_size;     uint64_t pen_phys;
    uint32_t *trans_buf;   size_t trans_size;   uint64_t trans_phys;
} mock_ctx_t;

/* --- One simulated sweep for the checkerboard tiles of cu_id --- */
static void mock_run_sweep(mock_ctx_t *mc, int cu) {
    uint32_t *regs = mc->regs[cu];
    int map_x = (int)regs[VI_OFF_MAP_X / 4];
    int map_y = (int)regs[VI_OFF_MAP_Y / 4];
    int ntx   = (int)regs[VI_OFF_NUM_TILES_X / 4];
    int nty   = (int)regs[VI_OFF_NUM_TILES_Y / 4];
    int cu_id = (int)regs[VI_OFF_CU_ID / 4];

    if (map_x <= 0 || map_y <= 0 || !mc->value_buf) {
        regs[VI_OFF_MAX_DELTA / 4] = 0;
        return;
    }

    uint16_t local_max = 0;

    for (int ty = 0; ty < nty; ty++) {
        for (int tx = 0; tx < ntx; tx++) {
            if (((tx + ty) & 1) != cu_id) continue;

            int y0 = ty * VI_TILE_H, y1 = y0 + VI_TILE_H; if (y1 > map_y) y1 = map_y;
            int x0 = tx * VI_TILE_W, x1 = x0 + VI_TILE_W; if (x1 > map_x) x1 = map_x;

            for (int iy = y0; iy < y1; iy++) {
                for (int ix = x0; ix < x1; ix++) {
                    uint16_t d = vi_bellman_cell(mc->value_buf, mc->pen_buf,
                                                 mc->trans_buf,
                                                 map_x, map_y, ix, iy);
                    if (d > local_max) local_max = d;
                }
            }
        }
    }

    regs[VI_OFF_MAX_DELTA / 4] = local_max;
}

/* --- ops implementation --- */

static int mock_init(void *vctx) {
    mock_ctx_t *mc = (mock_ctx_t*)vctx;

    /* Small allocation for tests (full worst-case buffer is too big on host). */
    mc->value_size = 256 * 256 * VI_N_THETA * sizeof(uint16_t);
    mc->pen_size   = 256 * 256 * sizeof(uint16_t);
    mc->trans_size = VI_N_ACTIONS * VI_N_THETA * sizeof(uint32_t);

    mc->value_buf = calloc(1, mc->value_size);
    mc->pen_buf   = calloc(1, mc->pen_size);
    mc->trans_buf = calloc(1, mc->trans_size);
    if (!mc->value_buf || !mc->pen_buf || !mc->trans_buf) return VI_ERR_MMAP;

    mc->value_phys = 0x1000000;
    mc->pen_phys   = 0x2000000;
    mc->trans_phys = 0x3000000;
    memset(mc->regs, 0, sizeof mc->regs);
    return 0;
}

static void mock_shutdown(void *vctx) {
    mock_ctx_t *mc = (mock_ctx_t*)vctx;
    free(mc->value_buf); mc->value_buf = NULL;
    free(mc->pen_buf);   mc->pen_buf   = NULL;
    free(mc->trans_buf); mc->trans_buf = NULL;
}

static uint32_t mock_read_reg(void *vctx, int cu, uint32_t off) {
    mock_ctx_t *mc = (mock_ctx_t*)vctx;
    if (cu < 0 || cu >= VI_NUM_CU || off / 4 >= MOCK_REG_WORDS) return 0;
    return mc->regs[cu][off / 4];
}

static void mock_write_reg(void *vctx, int cu, uint32_t off, uint32_t v) {
    mock_ctx_t *mc = (mock_ctx_t*)vctx;
    if (cu < 0 || cu >= VI_NUM_CU || off / 4 >= MOCK_REG_WORDS) return;

    mc->regs[cu][off / 4] = v;

    /* ap_start: run one sweep synchronously. */
    if (off == VI_OFF_AP_CTRL && (v & 0x1)) {
        mock_run_sweep(mc, cu);
        /* Clear ap_start, set ap_done and ap_idle. */
        uint32_t *ctrl = &mc->regs[cu][VI_OFF_AP_CTRL / 4];
        *ctrl = (*ctrl & ~0x1u) | 0x6u;  /* done | idle */
    }
}

static int mock_wait_irq(void *vctx, int cu, int timeout_ms) {
    (void)timeout_ms;
    mock_ctx_t *mc = (mock_ctx_t*)vctx;
    if (cu < 0 || cu >= VI_NUM_CU) return VI_ERR_IRQ;
    /* Sweep already ran synchronously during write_reg(AP_CTRL).
       Just verify ap_done is set. */
    return (mc->regs[cu][VI_OFF_AP_CTRL / 4] & 0x2) ? 0 : VI_ERR_IRQ;
}

static void* mock_map_buf(void *vctx, int buf_id, size_t *size, uint64_t *phys) {
    mock_ctx_t *mc = (mock_ctx_t*)vctx;
    switch (buf_id) {
    case VI_BUF_VALUE:   *size = mc->value_size; *phys = mc->value_phys; return mc->value_buf;
    case VI_BUF_PENALTY: *size = mc->pen_size;   *phys = mc->pen_phys;   return mc->pen_buf;
    case VI_BUF_TRANS:   *size = mc->trans_size; *phys = mc->trans_phys; return mc->trans_buf;
    }
    return NULL;
}

const vi_device_ops_t vi_mock_ops = {
    .init      = mock_init,
    .shutdown  = mock_shutdown,
    .read_reg  = mock_read_reg,
    .write_reg = mock_write_reg,
    .wait_irq  = mock_wait_irq,
    .map_buf   = mock_map_buf,
};

void* vi_mock_ctx_new(void) {
    return calloc(1, sizeof(mock_ctx_t));
}

void vi_mock_ctx_free(void *ctx) {
    free(ctx);
}
