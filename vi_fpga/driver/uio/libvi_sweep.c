/* libvi_sweep.c — core sweep loop using vi_device_ops abstraction. */

#define _POSIX_C_SOURCE 200809L

#include "libvi_sweep.h"
#include "vi_device.h"

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Register offsets — kept in sync with xvi_sweep_hw.h. If HLS regenerates,
   run `make sync-hw-header` in driver/uio/ and update these from the
   generated file. */
#define VI_OFF_AP_CTRL       0x00
#define VI_OFF_GIE           0x04
#define VI_OFF_IER           0x08
#define VI_OFF_ISR           0x0C
#define VI_OFF_VALUE_TABLE   0x10  /* 64-bit, 0x10 lo / 0x14 hi */
#define VI_OFF_PENALTY_TABLE 0x1C
#define VI_OFF_TRANS_TABLE   0x28
#define VI_OFF_MAP_X         0x34
#define VI_OFF_MAP_Y         0x3C
#define VI_OFF_NUM_TILES_X   0x44
#define VI_OFF_NUM_TILES_Y   0x4C
#define VI_OFF_CU_ID         0x54
#define VI_OFF_MAX_DELTA     0x5C

#define AP_START  0x1
#define AP_DONE   0x2
#define AP_IDLE   0x4
#define AP_READY  0x8

struct vi_device {
    const vi_device_ops_t *ops;
    void *ctx;

    uint16_t *value_buf;   size_t value_size;   uint64_t value_phys;
    uint16_t *pen_buf;     size_t pen_size;     uint64_t pen_phys;
    uint32_t *trans_buf;   size_t trans_size;   uint64_t trans_phys;
};

static void write_addr64(vi_device_t *dev, int cu, uint32_t off, uint64_t addr) {
    dev->ops->write_reg(dev->ctx, cu, off,     (uint32_t)(addr & 0xFFFFFFFFu));
    dev->ops->write_reg(dev->ctx, cu, off + 4, (uint32_t)(addr >> 32));
}

vi_device_t* vi_open(const vi_device_ops_t *ops, void *ctx) {
    if (!ops) return NULL;
    vi_device_t *dev = calloc(1, sizeof *dev);
    if (!dev) return NULL;
    dev->ops = ops;
    dev->ctx = ctx;

    if (ops->init(ctx) != 0) {
        free(dev);
        return NULL;
    }

    size_t sz; uint64_t phys;
    dev->value_buf = ops->map_buf(ctx, VI_BUF_VALUE,   &sz, &phys);
    if (!dev->value_buf) goto fail;
    dev->value_size = sz; dev->value_phys = phys;

    dev->pen_buf = ops->map_buf(ctx, VI_BUF_PENALTY, &sz, &phys);
    if (!dev->pen_buf) goto fail;
    dev->pen_size = sz; dev->pen_phys = phys;

    dev->trans_buf = ops->map_buf(ctx, VI_BUF_TRANS, &sz, &phys);
    if (!dev->trans_buf) goto fail;
    dev->trans_size = sz; dev->trans_phys = phys;

    return dev;

fail:
    ops->shutdown(ctx);
    free(dev);
    return NULL;
}

void vi_close(vi_device_t *dev) {
    if (!dev) return;
    if (dev->ops && dev->ops->shutdown) dev->ops->shutdown(dev->ctx);
    free(dev);
}

uint16_t* vi_value_buffer(vi_device_t *dev, size_t *n_u16) {
    if (n_u16) *n_u16 = dev->value_size / sizeof(uint16_t);
    return dev->value_buf;
}
uint16_t* vi_penalty_buffer(vi_device_t *dev, size_t *n_u16) {
    if (n_u16) *n_u16 = dev->pen_size / sizeof(uint16_t);
    return dev->pen_buf;
}
uint32_t* vi_trans_buffer(vi_device_t *dev, size_t *n_u32) {
    if (n_u32) *n_u32 = dev->trans_size / sizeof(uint32_t);
    return dev->trans_buf;
}

static void program_cu(vi_device_t *dev, int cu,
                       const vi_run_config_t *cfg,
                       int num_tiles_x, int num_tiles_y) {
    write_addr64(dev, cu, VI_OFF_VALUE_TABLE,   dev->value_phys);
    write_addr64(dev, cu, VI_OFF_PENALTY_TABLE, dev->pen_phys);
    write_addr64(dev, cu, VI_OFF_TRANS_TABLE,   dev->trans_phys);
    dev->ops->write_reg(dev->ctx, cu, VI_OFF_MAP_X,       (uint32_t)cfg->map_x);
    dev->ops->write_reg(dev->ctx, cu, VI_OFF_MAP_Y,       (uint32_t)cfg->map_y);
    dev->ops->write_reg(dev->ctx, cu, VI_OFF_NUM_TILES_X, (uint32_t)num_tiles_x);
    dev->ops->write_reg(dev->ctx, cu, VI_OFF_NUM_TILES_Y, (uint32_t)num_tiles_y);
    dev->ops->write_reg(dev->ctx, cu, VI_OFF_CU_ID,       (uint32_t)cu);
    dev->ops->write_reg(dev->ctx, cu, VI_OFF_IER,         0x1);
    dev->ops->write_reg(dev->ctx, cu, VI_OFF_GIE,         0x1);
}

int vi_run_until_converged(vi_device_t *dev,
                           const vi_run_config_t *cfg,
                           vi_run_stats_t *stats) {
    if (!dev || !cfg) return VI_ERR_BAD_ARG;
    if (cfg->map_x <= 0 || cfg->map_y <= 0) return VI_ERR_BAD_ARG;
    if ((size_t)cfg->map_x * cfg->map_y * VI_N_THETA * 2 > dev->value_size)
        return VI_ERR_BUF_SIZE;
    if ((size_t)cfg->map_x * cfg->map_y * 2 > dev->pen_size)
        return VI_ERR_BUF_SIZE;

    int ntx = (cfg->map_x + VI_TILE_W - 1) / VI_TILE_W;
    int nty = (cfg->map_y + VI_TILE_H - 1) / VI_TILE_H;

    if (stats) memset(stats, 0, sizeof *stats);

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    for (int cu = 0; cu < VI_NUM_CU; cu++)
        program_cu(dev, cu, cfg, ntx, nty);

    int sweep;
    uint16_t last_delta = 0xFFFF;
    int converged = 0;
    for (sweep = 0; sweep < cfg->max_sweeps; sweep++) {
        /* Start both CUs */
        for (int cu = 0; cu < VI_NUM_CU; cu++)
            dev->ops->write_reg(dev->ctx, cu, VI_OFF_AP_CTRL, AP_START);

        /* Wait for both */
        for (int cu = 0; cu < VI_NUM_CU; cu++) {
            int rc = dev->ops->wait_irq(dev->ctx, cu, 60000);
            if (rc != 0) return VI_ERR_IRQ;
        }

        /* Read deltas */
        uint16_t d0 = (uint16_t)dev->ops->read_reg(dev->ctx, 0, VI_OFF_MAX_DELTA);
        uint16_t d1 = (uint16_t)dev->ops->read_reg(dev->ctx, 1, VI_OFF_MAX_DELTA);
        last_delta = (d0 > d1) ? d0 : d1;

        if (last_delta <= cfg->threshold) { converged = 1; sweep++; break; }
    }

    clock_gettime(CLOCK_MONOTONIC, &t1);
    if (stats) {
        stats->sweeps      = sweep;
        stats->final_delta = last_delta;
        stats->converged   = converged;
        stats->elapsed_sec = (t1.tv_sec - t0.tv_sec) +
                             (t1.tv_nsec - t0.tv_nsec) * 1e-9;
    }
    return converged ? VI_OK : VI_ERR_NOT_CONV;
}

int vi_compute_action_table(vi_device_t *dev, int map_x, int map_y,
                            uint8_t *action_out) {
    if (!dev || !action_out || map_x <= 0 || map_y <= 0) return VI_ERR_BAD_ARG;
    if ((size_t)map_x * map_y * VI_N_THETA * 2 > dev->value_size)
        return VI_ERR_BUF_SIZE;

    const uint16_t *val   = dev->value_buf;
    const uint16_t *pen   = dev->pen_buf;
    const uint32_t *trans = dev->trans_buf;

    for (int y = 0; y < map_y; y++) {
        for (int x = 0; x < map_x; x++) {
            uint16_t cell_pen = pen[y * map_x + x];
            for (int it = 0; it < VI_N_THETA; it++) {
                size_t out_idx = ((size_t)y * map_x + x) * VI_N_THETA + it;

                /* Obstacle or goal: fallback to action 0 (caller ignores). */
                if (cell_pen >= 0xFFFE) {
                    action_out[out_idx] = 0;
                    continue;
                }

                uint16_t best_cost = 0xFFFF;
                uint8_t  best_act  = 0;
                for (int a = 0; a < VI_N_ACTIONS; a++) {
                    uint32_t t = trans[a * VI_N_THETA + it];
                    int8_t dix = (int8_t)(t & 0xFF);
                    int8_t diy = (int8_t)((t >> 8) & 0xFF);
                    int8_t dit = (int8_t)((t >> 16) & 0xFF);
                    int nx = x + dix, ny = y + diy, nt = it + dit;
                    if (nt < 0) nt += VI_N_THETA;
                    if (nt >= VI_N_THETA) nt -= VI_N_THETA;
                    if (nx < 0 || nx >= map_x || ny < 0 || ny >= map_y) continue;

                    size_t nidx = ((size_t)ny * map_x + nx) * VI_N_THETA + nt;
                    uint16_t nv = val[nidx];
                    if (nv >= best_cost) continue;
                    best_cost = nv;
                    best_act  = (uint8_t)a;
                }
                action_out[out_idx] = best_act;
            }
        }
    }
    return VI_OK;
}

const char* vi_strerror(int code) {
    switch (code) {
    case VI_OK:           return "OK";
    case VI_ERR_OPEN:     return "open failed";
    case VI_ERR_MMAP:     return "mmap failed";
    case VI_ERR_IRQ:      return "irq wait failed";
    case VI_ERR_BUF_SIZE: return "buffer too small for map";
    case VI_ERR_NOT_CONV: return "did not converge within max_sweeps";
    case VI_ERR_BAD_ARG:  return "bad argument";
    }
    return "unknown";
}
