#ifndef VI_DEVICE_H
#define VI_DEVICE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VI_BUF_VALUE    0
#define VI_BUF_PENALTY  1
#define VI_BUF_TRANS    2

typedef struct vi_device_ops {
    /* Called once from vi_open. Returns 0 on success, negative on failure. */
    int      (*init)    (void *ctx);

    /* Release all resources. Safe to call on a partially-initialized ctx. */
    void     (*shutdown)(void *ctx);

    /* AXI-Lite control register read/write for CU 0 or 1. off is byte offset. */
    uint32_t (*read_reg) (void *ctx, int cu, uint32_t off);
    void     (*write_reg)(void *ctx, int cu, uint32_t off, uint32_t v);

    /* Block until CU[cu] raises its interrupt (or timeout).
       Returns 0 on success, negative on timeout/error. */
    int      (*wait_irq)(void *ctx, int cu, int timeout_ms);

    /* Return a mmapped buffer. buf_id is one of VI_BUF_*.
       *size: byte size the buffer provides.
       *phys: physical address to program into the CU registers.
       Returns NULL on failure. */
    void*    (*map_buf)(void *ctx, int buf_id,
                        size_t *size, uint64_t *phys);
} vi_device_ops_t;

/* Exported op tables (defined in vi_device_linux.c / vi_device_mock.c) */
#ifndef VI_MOCK_ONLY
extern const vi_device_ops_t vi_linux_ops;
#endif
extern const vi_device_ops_t vi_mock_ops;

/* Mock context constructor (returns opaque ctx to pass to vi_open). */
void* vi_mock_ctx_new(void);
void  vi_mock_ctx_free(void *ctx);

#ifdef __cplusplus
}
#endif
#endif
