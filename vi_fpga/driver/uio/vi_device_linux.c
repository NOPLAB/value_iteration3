/* vi_device_linux.c — Linux implementation of vi_device_ops using UIO + u-dma-buf. */

#ifndef VI_MOCK_ONLY

/* feature test macros for clock_gettime, snprintf, etc. */
#define _POSIX_C_SOURCE 200809L

#include "vi_device.h"
#include "libvi_sweep.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#define VI_CTRL_SIZE 0x10000
#define UDMA_VALUE_DEV    "/dev/udmabuf_value"
#define UDMA_PENDATA_DEV  "/dev/udmabuf_pendata"
#define UDMA_VALUE_SYS    "/sys/class/u-dma-buf/udmabuf_value"
#define UDMA_PENDATA_SYS  "/sys/class/u-dma-buf/udmabuf_pendata"
#define UIO_CU0_NAME      "vi_sweep_cu0"
#define UIO_CU1_NAME      "vi_sweep_cu1"

typedef struct {
    int       uio_fd[VI_NUM_CU];
    volatile uint32_t *ctrl[VI_NUM_CU];

    int       udma_value_fd;
    int       udma_pendata_fd;

    void     *value_mmap;    size_t value_size;   uint64_t value_phys;
    void     *pendata_mmap;  size_t pendata_size; uint64_t pendata_phys;

    /* Penalty and trans share pendata; penalty occupies first chunk, trans follows. */
    void     *pen_ptr;       size_t pen_size;     uint64_t pen_phys;
    void     *trans_ptr;     size_t trans_size;   uint64_t trans_phys;
} linux_ctx_t;

static linux_ctx_t g_linux_ctx;

static int read_line(const char *path, char *buf, size_t sz) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    if (!fgets(buf, (int)sz, f)) { fclose(f); return -1; }
    fclose(f);
    size_t n = strlen(buf);
    while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r')) buf[--n] = 0;
    return 0;
}

static int find_uio_by_name(const char *name) {
    DIR *d = opendir("/sys/class/uio");
    if (!d) return -1;
    struct dirent *de;
    int found = -1;
    while ((de = readdir(d))) {
        if (strncmp(de->d_name, "uio", 3) != 0) continue;
        char npath[512], nbuf[128];
        snprintf(npath, sizeof npath, "/sys/class/uio/%s/name", de->d_name);
        if (read_line(npath, nbuf, sizeof nbuf) < 0) continue;
        if (strcmp(nbuf, name) == 0) { found = atoi(de->d_name + 3); break; }
    }
    closedir(d);
    return found;
}

static uint64_t read_udma_phys(const char *sys_path) {
    char p[512], buf[64];
    snprintf(p, sizeof p, "%s/phys_addr", sys_path);
    if (read_line(p, buf, sizeof buf) < 0) return 0;
    return strtoull(buf, NULL, 0);
}

static size_t read_udma_size(const char *sys_path) {
    char p[512], buf[64];
    snprintf(p, sizeof p, "%s/size", sys_path);
    if (read_line(p, buf, sizeof buf) < 0) return 0;
    return (size_t)strtoull(buf, NULL, 0);
}

static void linux_shutdown(void *vctx);

static int linux_init(void *vctx) {
    (void)vctx;
    linux_ctx_t *c = &g_linux_ctx;
    memset(c, 0, sizeof *c);
    for (int i = 0; i < VI_NUM_CU; i++) c->uio_fd[i] = -1;
    c->udma_value_fd = c->udma_pendata_fd = -1;

    /* --- UIO nodes --- */
    const char *names[VI_NUM_CU] = { UIO_CU0_NAME, UIO_CU1_NAME };
    for (int i = 0; i < VI_NUM_CU; i++) {
        int num = find_uio_by_name(names[i]);
        if (num < 0) { linux_shutdown(vctx); return VI_ERR_OPEN; }
        char path[64];
        snprintf(path, sizeof path, "/dev/uio%d", num);
        c->uio_fd[i] = open(path, O_RDWR);
        if (c->uio_fd[i] < 0) { linux_shutdown(vctx); return VI_ERR_OPEN; }
        c->ctrl[i] = mmap(NULL, VI_CTRL_SIZE, PROT_READ | PROT_WRITE,
                          MAP_SHARED, c->uio_fd[i], 0);
        if (c->ctrl[i] == MAP_FAILED) { c->ctrl[i] = NULL; linux_shutdown(vctx); return VI_ERR_MMAP; }
    }

    /* --- udmabuf nodes --- */
    c->udma_value_fd   = open(UDMA_VALUE_DEV, O_RDWR);
    c->udma_pendata_fd = open(UDMA_PENDATA_DEV, O_RDWR);
    if (c->udma_value_fd < 0 || c->udma_pendata_fd < 0) { linux_shutdown(vctx); return VI_ERR_OPEN; }

    c->value_size   = read_udma_size(UDMA_VALUE_SYS);
    c->pendata_size = read_udma_size(UDMA_PENDATA_SYS);
    if (c->value_size == 0 || c->pendata_size == 0) { linux_shutdown(vctx); return VI_ERR_OPEN; }
    c->value_phys   = read_udma_phys(UDMA_VALUE_SYS);
    c->pendata_phys = read_udma_phys(UDMA_PENDATA_SYS);

    c->value_mmap   = mmap(NULL, c->value_size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, c->udma_value_fd, 0);
    c->pendata_mmap = mmap(NULL, c->pendata_size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, c->udma_pendata_fd, 0);
    if (c->value_mmap == MAP_FAILED)   { c->value_mmap = NULL;   linux_shutdown(vctx); return VI_ERR_MMAP; }
    if (c->pendata_mmap == MAP_FAILED) { c->pendata_mmap = NULL; linux_shutdown(vctx); return VI_ERR_MMAP; }

    c->pen_size   = (size_t)VI_MAX_MAP_X * VI_MAX_MAP_Y * sizeof(uint16_t);
    if (c->pen_size > c->pendata_size) c->pen_size = c->pendata_size;
    c->pen_ptr    = c->pendata_mmap;
    c->pen_phys   = c->pendata_phys;

    c->trans_size = VI_N_ACTIONS * VI_N_THETA * sizeof(uint32_t);
    c->trans_ptr  = (char*)c->pendata_mmap + c->pen_size;
    c->trans_phys = c->pendata_phys + c->pen_size;

    return 0;
}

static void linux_shutdown(void *vctx) {
    (void)vctx;
    linux_ctx_t *c = &g_linux_ctx;
    if (c->value_mmap)   munmap(c->value_mmap,   c->value_size);
    if (c->pendata_mmap) munmap(c->pendata_mmap, c->pendata_size);
    if (c->udma_value_fd   >= 0) close(c->udma_value_fd);
    if (c->udma_pendata_fd >= 0) close(c->udma_pendata_fd);
    for (int i = 0; i < VI_NUM_CU; i++) {
        if (c->ctrl[i])    munmap((void*)c->ctrl[i], VI_CTRL_SIZE);
        if (c->uio_fd[i] >= 0) close(c->uio_fd[i]);
    }
    memset(c, 0, sizeof *c);
    for (int i = 0; i < VI_NUM_CU; i++) c->uio_fd[i] = -1;
    c->udma_value_fd = c->udma_pendata_fd = -1;
}

static uint32_t linux_read_reg(void *vctx, int cu, uint32_t off) {
    (void)vctx;
    linux_ctx_t *c = &g_linux_ctx;
    return c->ctrl[cu][off / 4];
}

static void linux_write_reg(void *vctx, int cu, uint32_t off, uint32_t v) {
    (void)vctx;
    linux_ctx_t *c = &g_linux_ctx;
    c->ctrl[cu][off / 4] = v;
}

static int linux_wait_irq(void *vctx, int cu, int timeout_ms) {
    (void)vctx;
    linux_ctx_t *c = &g_linux_ctx;
    struct pollfd pfd = { .fd = c->uio_fd[cu], .events = POLLIN, .revents = 0 };
    int rc = poll(&pfd, 1, timeout_ms);
    if (rc <= 0) return VI_ERR_IRQ;
    uint32_t count;
    if (read(c->uio_fd[cu], &count, 4) != 4) return VI_ERR_IRQ;
    /* ack ISR bit 0 (W1C) */
    c->ctrl[cu][0x0C / 4] = 0x1;
    /* re-arm UIO interrupt for next event */
    uint32_t one = 1;
    ssize_t w = write(c->uio_fd[cu], &one, 4);
    (void)w;
    return 0;
}

static void* linux_map_buf(void *vctx, int buf_id, size_t *size, uint64_t *phys) {
    (void)vctx;
    linux_ctx_t *c = &g_linux_ctx;
    switch (buf_id) {
    case VI_BUF_VALUE:   *size = c->value_size;  *phys = c->value_phys; return c->value_mmap;
    case VI_BUF_PENALTY: *size = c->pen_size;    *phys = c->pen_phys;   return c->pen_ptr;
    case VI_BUF_TRANS:   *size = c->trans_size;  *phys = c->trans_phys; return c->trans_ptr;
    }
    return NULL;
}

const vi_device_ops_t vi_linux_ops = {
    .init      = linux_init,
    .shutdown  = linux_shutdown,
    .read_reg  = linux_read_reg,
    .write_reg = linux_write_reg,
    .wait_irq  = linux_wait_irq,
    .map_buf   = linux_map_buf,
};

#endif /* VI_MOCK_ONLY */
