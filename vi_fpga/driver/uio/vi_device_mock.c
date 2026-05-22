/* vi_device_mock.c — simulates vi_sweep FPGA IP in software.
   Used for host unit testing of libvi_sweep. */

#include "vi_device.h"
#include "libvi_sweep.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Register offsets (must match the layout libvi_sweep.c uses). */
#define MOCK_AP_CTRL        0x00
#define MOCK_GIE            0x04
#define MOCK_IER            0x08
#define MOCK_ISR            0x0C
#define MOCK_ADDR_VALUE     0x10  /* 64-bit */
#define MOCK_ADDR_PENALTY   0x1C
#define MOCK_ADDR_TRANS     0x28
#define MOCK_MAP_X          0x34
#define MOCK_MAP_Y          0x3C
#define MOCK_NUM_TILES_X    0x44
#define MOCK_NUM_TILES_Y    0x4C
#define MOCK_CU_ID          0x54
#define MOCK_MAX_DELTA      0x5C

#define MOCK_REG_BYTES      0x100

/* Shared physical backing (same for both CUs) */
typedef struct {
    uint8_t   regs[VI_NUM_CU][MOCK_REG_BYTES];

    /* Simulated DDR buffers */
    uint16_t *value_buf;   size_t value_size;   uint64_t value_phys;
    uint16_t *pen_buf;     size_t pen_size;     uint64_t pen_phys;
    uint32_t *trans_buf;   size_t trans_size;   uint64_t trans_phys;
} mock_ctx_t;

static uint32_t rd32(const uint8_t *base, uint32_t off) {
    uint32_t v;
    memcpy(&v, base + off, 4);
    return v;
}
static void wr32(uint8_t *base, uint32_t off, uint32_t v) {
    memcpy(base + off, &v, 4);
}

/* --- One simulated sweep for the checkerboard tiles of cu_id --- */
static void mock_run_sweep(mock_ctx_t *mc, int cu) {
    uint8_t *regs = mc->regs[cu];
    int map_x = (int)rd32(regs, MOCK_MAP_X);
    int map_y = (int)rd32(regs, MOCK_MAP_Y);
    int ntx   = (int)rd32(regs, MOCK_NUM_TILES_X);
    int nty   = (int)rd32(regs, MOCK_NUM_TILES_Y);
    int cu_id = (int)rd32(regs, MOCK_CU_ID);

    if (map_x <= 0 || map_y <= 0 || !mc->value_buf) {
        wr32(regs, MOCK_MAX_DELTA, 0);
        return;
    }

    uint16_t *val = mc->value_buf;
    const uint16_t *pen = mc->pen_buf;
    const uint32_t *trans = mc->trans_buf;

    uint16_t local_max = 0;

    for (int ty = 0; ty < nty; ty++) {
        for (int tx = 0; tx < ntx; tx++) {
            if (((tx + ty) & 1) != cu_id) continue;

            int y0 = ty * VI_TILE_H, y1 = y0 + VI_TILE_H; if (y1 > map_y) y1 = map_y;
            int x0 = tx * VI_TILE_W, x1 = x0 + VI_TILE_W; if (x1 > map_x) x1 = map_x;

            for (int iy = y0; iy < y1; iy++) {
                for (int ix = x0; ix < x1; ix++) {
                    uint16_t cell_pen = pen[iy * map_x + ix];
                    if (cell_pen >= 0xFFFE) continue;  /* obstacle or goal */

                    for (int it = 0; it < VI_N_THETA; it++) {
                        size_t idx = ((size_t)iy * map_x + ix) * VI_N_THETA + it;
                        uint16_t old = val[idx];
                        uint16_t best = 0xFFFF;

                        for (int a = 0; a < VI_N_ACTIONS; a++) {
                            uint32_t t = trans[a * VI_N_THETA + it];
                            int8_t dix = (int8_t)(t & 0xFF);
                            int8_t diy = (int8_t)((t >> 8) & 0xFF);
                            int8_t dit = (int8_t)((t >> 16) & 0xFF);

                            int nx = ix + dix;
                            int ny = iy + diy;
                            int nt = it + dit;
                            if (nt < 0) nt += VI_N_THETA;
                            if (nt >= VI_N_THETA) nt -= VI_N_THETA;
                            if (nx < 0 || nx >= map_x || ny < 0 || ny >= map_y) continue;

                            size_t nidx = ((size_t)ny * map_x + nx) * VI_N_THETA + nt;
                            uint16_t nv = val[nidx];
                            uint16_t np_raw = pen[ny * map_x + nx];
                            if (nv == 0xFFFF || np_raw == 0xFFFF) continue;
                            uint16_t np = (np_raw == 0xFFFE) ? 0 : np_raw;

                            uint32_t sum = (uint32_t)nv + (uint32_t)np;
                            uint16_t c = (sum >= 0xFFFF) ? 0xFFFE : (uint16_t)sum;
                            if (c < best) best = c;
                        }

                        val[idx] = best;
                        uint16_t d = (best > old) ? (best - old) : (old - best);
                        if (d > local_max) local_max = d;
                    }
                }
            }
        }
    }

    wr32(regs, MOCK_MAX_DELTA, local_max);
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
    if (cu < 0 || cu >= VI_NUM_CU || off + 4 > MOCK_REG_BYTES) return 0;
    return rd32(mc->regs[cu], off);
}

static void mock_write_reg(void *vctx, int cu, uint32_t off, uint32_t v) {
    mock_ctx_t *mc = (mock_ctx_t*)vctx;
    if (cu < 0 || cu >= VI_NUM_CU || off + 4 > MOCK_REG_BYTES) return;

    /* ISR is W1C — read-modify-write based on the previous value. */
    if (off == MOCK_ISR) {
        uint32_t cur = rd32(mc->regs[cu], MOCK_ISR);
        wr32(mc->regs[cu], MOCK_ISR, cur & ~v);
        return;
    }

    wr32(mc->regs[cu], off, v);

    /* ap_start: run one sweep synchronously. */
    if (off == MOCK_AP_CTRL && (v & 0x1)) {
        mock_run_sweep(mc, cu);
        /* Clear ap_start, set ap_done and ap_idle. */
        uint32_t ctrl = rd32(mc->regs[cu], MOCK_AP_CTRL);
        ctrl &= ~0x1u;
        ctrl |= 0x6u;  /* done | idle */
        wr32(mc->regs[cu], MOCK_AP_CTRL, ctrl);
        /* OR in ap_done bit on ISR. */
        uint32_t isr = rd32(mc->regs[cu], MOCK_ISR);
        wr32(mc->regs[cu], MOCK_ISR, isr | 0x1u);
    }
}

static int mock_wait_irq(void *vctx, int cu, int timeout_ms) {
    (void)timeout_ms;
    mock_ctx_t *mc = (mock_ctx_t*)vctx;
    if (cu < 0 || cu >= VI_NUM_CU) return VI_ERR_IRQ;
    /* Sweep already ran synchronously during write_reg(AP_CTRL).
       Just verify ap_done is set. */
    uint32_t ctrl = rd32(mc->regs[cu], MOCK_AP_CTRL);
    return (ctrl & 0x2) ? 0 : VI_ERR_IRQ;
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
