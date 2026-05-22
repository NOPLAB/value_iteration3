#ifndef LIBVI_SWEEP_H
#define LIBVI_SWEEP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VI_N_THETA      60
#define VI_N_ACTIONS     6
#define VI_TILE_W       32
#define VI_TILE_H       32
#define VI_NUM_CU        2

/* Worst-case map size (spec §3, 700m x 40m at 0.05m resolution). */
#define VI_MAX_MAP_X    14000
#define VI_MAX_MAP_Y      800

/* Opaque device handle. */
typedef struct vi_device vi_device_t;

/* Forward decl of ops (see vi_device.h). */
struct vi_device_ops;

typedef struct {
    int      map_x;
    int      map_y;
    uint16_t threshold;
    int      max_sweeps;
} vi_run_config_t;

typedef struct {
    int      sweeps;
    uint16_t final_delta;
    double   elapsed_sec;
    int      converged;
} vi_run_stats_t;

/* --- Lifecycle --- */
vi_device_t* vi_open (const struct vi_device_ops *ops, void *ctx);
void         vi_close(vi_device_t *dev);

/* --- Direct buffer access (zero-copy) --- */
uint16_t* vi_value_buffer  (vi_device_t *dev, size_t *n_u16);
uint16_t* vi_penalty_buffer(vi_device_t *dev, size_t *n_u16);
uint32_t* vi_trans_buffer  (vi_device_t *dev, size_t *n_u32);

/* --- Execution --- */
int vi_run_until_converged(vi_device_t *dev,
                           const vi_run_config_t *cfg,
                           vi_run_stats_t *stats);

/* --- Post-convergence action table (argmin per state) --- */
int vi_compute_action_table(vi_device_t *dev,
                            int map_x, int map_y,
                            uint8_t *action_out);

/* --- Error helpers --- */
const char* vi_strerror(int code);

enum {
    VI_OK           =  0,
    VI_ERR_OPEN     = -1,
    VI_ERR_MMAP     = -2,
    VI_ERR_IRQ      = -3,
    VI_ERR_BUF_SIZE = -4,
    VI_ERR_NOT_CONV = -5,
    VI_ERR_BAD_ARG  = -6,
};

#ifdef __cplusplus
}
#endif
#endif
