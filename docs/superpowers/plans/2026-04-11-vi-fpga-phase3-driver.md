# Value Iteration FPGA — Phase 3 Linux UIO Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Phase 2 で焼いたビットストリームを Petalinux 上の UIO + u-dma-buf で叩き、収束まで Sweep を回す C ライブラリ (`libvi_sweep`) と CLI ツール (`vi_cli`) を提供する。

**Architecture:** `libvi_sweep` は `vi_device_ops_t` インターフェース経由で HW を触るため、ホスト PC では mock ops でユニットテストが可能。本番 Linux 実装は `vi_device_linux.c` で UIO + u-dma-buf + HLS 生成 `xvi_sweep_hw.h` を統合する。CLI は ROS map_server 互換 PGM を読み、verify モードで参照実装と比較する。

**Tech Stack:** C11, libc + pthread のみ、GNU Make、Petalinux 2025.2、ikwzm u-dma-buf、Vivado/Vitis 2025.2。

**Spec reference:** `docs/superpowers/specs/2026-04-11-vi-fpga-phase3-driver-design.md`

---

## Conventions

- ビルドは GNU Make。`host/` と `driver/uio/` にそれぞれ Makefile を置き、トップレベル `Makefile` から再帰呼び出し
- テストは自前アサーションマクロ (`host/test/vi_assert.h`) で gtest/catch2 依存を避ける
- 全 C ソースは C11 (`-std=c11 -Wall -Wextra -Werror`) を想定
- パスはトップレベル `value_iteration_fpga/` からの相対で記述
- コミット時のユーザ確認は自動化しない。コマンドをそのまま実行する想定

---

## Task 1: Phase 2 Addendum — BD Interrupt Wiring

Phase 2 の BD に `vi_sweep_cu0/interrupt` + `vi_sweep_cu1/interrupt` を Concat IP 経由で `zynq_ps/pl_ps_irq0[1:0]` に結線する。Phase 3 本体に入る前の前提条件。

**Files:**
- Modify: `fpga/vivado/ultra96v2/create_bd.tcl`

- [ ] **Step 1: Read current create_bd.tcl to locate the clock/reset wiring section**

Run: `cat fpga/vivado/ultra96v2/create_bd.tcl`

Locate the lines that connect `vi_sweep_cu0/ap_rst_n` etc. — the interrupt wiring will be inserted after those.

- [ ] **Step 2: Add xlconcat + IRQ wiring to create_bd.tcl**

Modify `fpga/vivado/ultra96v2/create_bd.tcl`. Insert after the existing `connect_bd_net $rstn ...` block and before the `# --- Control path ---` comment:

```tcl
# --- Interrupt: CU0/CU1 ap_done -> pl_ps_irq0[1:0] ---
set irq_concat [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 irq_concat]
set_property -dict [list \
    CONFIG.NUM_PORTS {2} \
    CONFIG.IN0_WIDTH {1} \
    CONFIG.IN1_WIDTH {1} \
] $irq_concat

connect_bd_net [get_bd_pins vi_sweep_cu0/interrupt] [get_bd_pins irq_concat/In0]
connect_bd_net [get_bd_pins vi_sweep_cu1/interrupt] [get_bd_pins irq_concat/In1]
connect_bd_net [get_bd_pins irq_concat/dout]        [get_bd_pins zynq_ps/pl_ps_irq0]

# Enable pl_ps_irq0 on the PS block
set_property -dict [list \
    CONFIG.PSU__USE__IRQ0 {1} \
    CONFIG.PSU__IRQ_P2F_IRQ0_SELECT {1} \
] [get_bd_cells zynq_ps]
```

- [ ] **Step 3: Re-run create_project.tcl and verify BD validates**

Run from `fpga/vivado/ultra96v2/`:

```bash
vivado -mode batch -source create_project.tcl
```

Expected: `validate_bd_design` returns success. If it complains about `pl_ps_irq0` width, remove the `PSU__IRQ_P2F_IRQ0_SELECT` line (default is already enabled on recent Vivado versions).

- [ ] **Step 4: Note the SPI interrupt number from the BD Address Editor**

After the project opens (or from the report), identify the GIC SPI number assigned to `pl_ps_irq0[0]` and `pl_ps_irq0[1]`. Save these numbers for Task 12 (device tree). Default on Zynq UltraScale+ is SPI 89/90 but confirm per the actual build.

Record the numbers in `fpga/vivado/ultra96v2/irq_notes.txt`:

```
pl_ps_irq0[0] = SPI <NUMBER>  # for vi_sweep_cu0
pl_ps_irq0[1] = SPI <NUMBER>  # for vi_sweep_cu1
```

- [ ] **Step 5: Rebuild bitstream**

Run: `cd fpga/scripts && make all`

Expected: `.bit` and `.hwh` regenerated with interrupt wiring. Implementation should still close timing.

- [ ] **Step 6: Commit**

```bash
git add fpga/vivado/ultra96v2/create_bd.tcl fpga/vivado/ultra96v2/irq_notes.txt
git commit -m "feat: Phase 2 addendum — wire CU interrupts to pl_ps_irq0[1:0]"
```

---

## Task 2: Project Scaffolding & Build System

Create `driver/` and `host/` directory structures and Makefiles. No code yet.

**Files:**
- Create: `driver/uio/Makefile`
- Create: `host/Makefile`
- Create: `Makefile` (top-level — only if missing)
- Create: `driver/uio/generated/.gitkeep`
- Create: `host/test/hw/.gitkeep`

- [ ] **Step 1: Create directory skeleton**

```bash
mkdir -p driver/uio/generated driver/dts host/src host/test/hw
touch driver/uio/generated/.gitkeep host/test/hw/.gitkeep
```

- [ ] **Step 2: Write driver/uio/Makefile**

Create `driver/uio/Makefile`:

```make
# libvi_sweep — C library for vi_sweep FPGA IP (UIO + u-dma-buf)

CC      ?= gcc
AR      ?= ar
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O2 -fPIC -Iinclude -I. -Igenerated
LDFLAGS := -lpthread

SRC_COMMON := libvi_sweep.c vi_device_mock.c
SRC_LINUX  := vi_device_linux.c
OBJ_COMMON := $(SRC_COMMON:.c=.o)
OBJ_LINUX  := $(SRC_LINUX:.c=.o)

HEADERS := libvi_sweep.h vi_device.h

LIB_STATIC := libvi_sweep.a
LIB_SHARED := libvi_sweep.so

.PHONY: all clean sync-hw-header mock-only

all: $(LIB_STATIC) $(LIB_SHARED)

# Host-only build (no Linux UIO, for unit tests)
mock-only: CFLAGS += -DVI_MOCK_ONLY
mock-only: libvi_sweep_mock.a

libvi_sweep_mock.a: $(OBJ_COMMON)
	$(AR) rcs $@ $^

$(LIB_STATIC): $(OBJ_COMMON) $(OBJ_LINUX)
	$(AR) rcs $@ $^

$(LIB_SHARED): $(OBJ_COMMON) $(OBJ_LINUX)
	$(CC) -shared -o $@ $^ $(LDFLAGS)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

# Copy HLS-generated register header into generated/
HLS_HW_HEADER := ../../fpga/hls/vi_sweep/hls_build/hls/impl/ip/drivers/vi_sweep_v1_0/src/xvi_sweep_hw.h

sync-hw-header:
	@test -f $(HLS_HW_HEADER) || { echo "HLS header not found: $(HLS_HW_HEADER)"; exit 1; }
	install -D $(HLS_HW_HEADER) generated/xvi_sweep_hw.h
	@echo "Synced xvi_sweep_hw.h. Review diff with: git diff generated/xvi_sweep_hw.h"

clean:
	rm -f *.o *.a *.so
```

- [ ] **Step 3: Write host/Makefile**

Create `host/Makefile`:

```make
# host: vi_cli + unit tests

CC      ?= gcc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O2 \
           -Isrc -I../driver/uio -I../fpga/hls/vi_sweep/src
LDFLAGS :=

# --- vi_cli ---
CLI_SRC := src/vi_cli.c src/map_pgm.c src/penalty.c src/transitions.c src/vi_reference_c.c
CLI_OBJ := $(CLI_SRC:.c=.o)
CLI_BIN := vi_cli

# Link against the Linux libvi_sweep (static)
LIBVI := ../driver/uio/libvi_sweep.a
LIBVI_MOCK := ../driver/uio/libvi_sweep_mock.a

# --- Unit tests ---
TEST_SRC := test/test_map_pgm.c \
            test/test_penalty.c \
            test/test_transitions.c \
            test/test_vi_run_mock.c \
            test/test_action_table.c \
            test/test_reference_eq.c
TEST_BINS := $(TEST_SRC:.c=)

.PHONY: all clean test-host test-hw cli cli-mock

all: cli

cli: $(CLI_BIN)

$(CLI_BIN): $(CLI_OBJ) $(LIBVI)
	$(CC) -o $@ $^ $(LDFLAGS) -lpthread

# Mock-only CLI build (no Linux ops, for host smoke test)
cli-mock: CFLAGS += -DVI_MOCK_ONLY
cli-mock: $(LIBVI_MOCK)
	$(CC) $(CFLAGS) -o vi_cli_mock $(CLI_SRC) $(LIBVI_MOCK)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# --- Host tests (mock only, no libpthread UIO code) ---
test-host: $(LIBVI_MOCK) $(TEST_BINS:=.run)
	@echo "=== All host tests PASSED ==="

$(LIBVI_MOCK):
	$(MAKE) -C ../driver/uio mock-only

test/%: test/%.c src/map_pgm.c src/penalty.c src/transitions.c src/vi_reference_c.c $(LIBVI_MOCK)
	$(CC) $(CFLAGS) -DVI_TEST -o $@ $^

%.run: %
	@echo "--- Running $< ---"
	@./$<

# --- HW integration tests ---
test-hw:
	@test -n "$$VI_TARGET_HOST" || { echo "set VI_TARGET_HOST"; exit 1; }
	bash test/hw/run_smoke.sh
	bash test/hw/run_big.sh

clean:
	rm -f src/*.o test/*.o $(CLI_BIN) $(TEST_BINS)
```

- [ ] **Step 4: Write top-level Makefile**

Check whether `Makefile` exists at the repo root: `ls Makefile 2>&1`.

If missing, create `Makefile`:

```make
.PHONY: driver host test-host test-hw clean

driver:
	$(MAKE) -C driver/uio all

host: driver
	$(MAKE) -C host all

test-host:
	$(MAKE) -C host test-host

test-hw:
	$(MAKE) -C host test-hw

clean:
	$(MAKE) -C driver/uio clean
	$(MAKE) -C host clean
```

If one already exists, add the above targets to it (do not overwrite).

- [ ] **Step 5: Commit**

```bash
git add driver/uio/Makefile driver/uio/generated/.gitkeep \
        host/Makefile host/test/hw/.gitkeep Makefile
git commit -m "feat: Phase 3 scaffolding — Makefiles and directory layout"
```

---

## Task 3: Device Ops Abstraction + libvi_sweep Header

Define the pure-header interfaces that everything else builds on.

**Files:**
- Create: `driver/uio/vi_device.h`
- Create: `driver/uio/libvi_sweep.h`

- [ ] **Step 1: Write driver/uio/vi_device.h**

```c
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
```

- [ ] **Step 2: Write driver/uio/libvi_sweep.h**

```c
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
```

- [ ] **Step 3: Verify headers compile standalone**

Run:

```bash
cd driver/uio
gcc -std=c11 -Wall -Wextra -Werror -fsyntax-only -x c vi_device.h
gcc -std=c11 -Wall -Wextra -Werror -fsyntax-only -x c libvi_sweep.h
```

Expected: no errors.

- [ ] **Step 4: Commit**

```bash
git add driver/uio/vi_device.h driver/uio/libvi_sweep.h
git commit -m "feat: libvi_sweep public API and device ops abstraction"
```

---

## Task 4: Mock Device Implementation

Mock `vi_device_ops_t` that simulates the HW by running the reference VI algorithm internally. This is what enables host-side unit testing of `libvi_sweep` core logic.

**Files:**
- Create: `driver/uio/vi_device_mock.c`

- [ ] **Step 1: Write driver/uio/vi_device_mock.c**

The mock implements the ops such that when `write_reg(AP_CTRL, 0x1)` is called on a CU, it runs one sweep of a simple Jacobi VI iteration over **only** the tiles assigned to that CU (checkerboard by `cu_id`), stores the resulting `max_delta` into the shadow register, and makes `wait_irq` return immediately. `read_reg` returns shadow register contents.

```c
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
#define MOCK_MAX_DELTA      0x60

#define MOCK_REG_BYTES      0x100

#define MOCK_MAP_X_MAX  VI_MAX_MAP_X
#define MOCK_MAP_Y_MAX  VI_MAX_MAP_Y

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

    mc->value_size = (size_t)MOCK_MAP_X_MAX * MOCK_MAP_Y_MAX * VI_N_THETA * sizeof(uint16_t);
    mc->pen_size   = (size_t)MOCK_MAP_X_MAX * MOCK_MAP_Y_MAX * sizeof(uint16_t);
    mc->trans_size = VI_N_ACTIONS * VI_N_THETA * sizeof(uint32_t);

    /* For tests we do not need the full worst-case buffer; allocate small. */
    mc->value_size = 256 * 256 * VI_N_THETA * sizeof(uint16_t);
    mc->pen_size   = 256 * 256 * sizeof(uint16_t);

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
    wr32(mc->regs[cu], off, v);

    /* ap_start: run one sweep synchronously. */
    if (off == MOCK_AP_CTRL && (v & 0x1)) {
        mock_run_sweep(mc, cu);
        /* Clear ap_start, set ap_done and ap_idle. */
        uint32_t ctrl = rd32(mc->regs[cu], MOCK_AP_CTRL);
        ctrl &= ~0x1;
        ctrl |= 0x6;  /* done | idle */
        wr32(mc->regs[cu], MOCK_AP_CTRL, ctrl);
        /* Set ISR bit 0. */
        wr32(mc->regs[cu], MOCK_ISR, 0x1);
    }

    /* ISR W1C */
    if (off == MOCK_ISR) {
        uint32_t cur = rd32(mc->regs[cu], MOCK_ISR);
        wr32(mc->regs[cu], MOCK_ISR, cur & ~v);
    }
}

static int mock_wait_irq(void *vctx, int cu, int timeout_ms) {
    (void)timeout_ms;
    mock_ctx_t *mc = (mock_ctx_t*)vctx;
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
```

- [ ] **Step 2: Verify compiles standalone**

```bash
cd driver/uio
gcc -std=c11 -Wall -Wextra -Werror -I. -c vi_device_mock.c -o vi_device_mock.o
```

Expected: no errors, `vi_device_mock.o` produced.

- [ ] **Step 3: Clean the object**

```bash
rm -f driver/uio/vi_device_mock.o
```

- [ ] **Step 4: Commit**

```bash
git add driver/uio/vi_device_mock.c
git commit -m "feat: mock device ops with in-process reference sweep"
```

---

## Task 5: Test Harness + libvi_sweep Core (open/close, buffers, sweep loop)

Build the main library code alongside its first unit test, using the mock from Task 4.

**Files:**
- Create: `host/test/vi_assert.h`
- Create: `host/test/test_vi_run_mock.c`
- Create: `driver/uio/libvi_sweep.c`

- [ ] **Step 1: Write host/test/vi_assert.h**

```c
#ifndef VI_ASSERT_H
#define VI_ASSERT_H

#include <stdio.h>
#include <stdlib.h>

static int vi_test_failures = 0;

#define VI_ASSERT(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        vi_test_failures++; \
    } \
} while (0)

#define VI_ASSERT_EQ(a, b) do { \
    long long _a = (long long)(a), _b = (long long)(b); \
    if (_a != _b) { \
        fprintf(stderr, "FAIL %s:%d: %s (%lld) == %s (%lld)\n", \
                __FILE__, __LINE__, #a, _a, #b, _b); \
        vi_test_failures++; \
    } \
} while (0)

#define VI_TEST_MAIN_END() do { \
    if (vi_test_failures) { \
        fprintf(stderr, "=== %d FAILURES ===\n", vi_test_failures); \
        return 1; \
    } \
    printf("PASS\n"); \
    return 0; \
} while (0)

#endif
```

- [ ] **Step 2: Write host/test/test_vi_run_mock.c (failing test first)**

```c
/* test_vi_run_mock.c — exercises libvi_sweep against mock ops. */

#include "vi_assert.h"
#include "libvi_sweep.h"
#include "vi_device.h"

#include <stdlib.h>
#include <string.h>

static void init_tiny_map(vi_device_t *dev, int w, int h, int gx, int gy) {
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

    /* trivial transitions: action 0 = +x, others = no-op.
       Packed uint32: byte0=dix, byte1=diy, byte2=dit */
    for (size_t i = 0; i < nt; i++) tr[i] = 0;
    for (int it = 0; it < VI_N_THETA; it++) {
        tr[0 * VI_N_THETA + it] = ((uint32_t)0x01);         /* dix=+1 */
        tr[1 * VI_N_THETA + it] = ((uint32_t)0xFF);         /* dix=-1 */
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

    /* Non-goal cell should be strictly less than MAX */
    VI_ASSERT(val[(4 * W + 4) * VI_N_THETA + 0] < 0xFFFF);

    vi_close(dev);
    vi_mock_ctx_free(ctx);
    VI_TEST_MAIN_END();
}
```

- [ ] **Step 3: Try to build the test — expect failure (libvi_sweep.c missing)**

```bash
cd host
make test/test_vi_run_mock
```

Expected: link error — `vi_open`, `vi_run_until_converged` etc. undefined. This confirms the test is wired correctly.

- [ ] **Step 4: Write driver/uio/libvi_sweep.c**

```c
/* libvi_sweep.c — core sweep loop using vi_device_ops abstraction. */

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
#define VI_OFF_MAX_DELTA     0x60

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

/* Stub — implemented in Task 6. */
int vi_compute_action_table(vi_device_t *dev, int map_x, int map_y,
                            uint8_t *action_out) {
    (void)dev; (void)map_x; (void)map_y; (void)action_out;
    return VI_ERR_BAD_ARG;
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
```

- [ ] **Step 5: Build the mock library and the test**

```bash
cd driver/uio && make mock-only && cd ../..
cd host && make test/test_vi_run_mock && cd ..
```

Expected: successful build of `driver/uio/libvi_sweep_mock.a` and `host/test/test_vi_run_mock`.

- [ ] **Step 6: Run the test**

```bash
./host/test/test_vi_run_mock
```

Expected output: `PASS`.

- [ ] **Step 7: Commit**

```bash
git add host/test/vi_assert.h host/test/test_vi_run_mock.c driver/uio/libvi_sweep.c
git commit -m "feat: libvi_sweep sweep loop with mock-driven unit test"
```

---

## Task 6: Action Table Computation

Implement `vi_compute_action_table` (argmin per state) and unit test it.

**Files:**
- Modify: `driver/uio/libvi_sweep.c`
- Create: `host/test/test_action_table.c`

- [ ] **Step 1: Write host/test/test_action_table.c**

```c
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

    /* All cells traversable, value gradient in x direction at theta=0. */
    for (size_t i = 0; i < nv; i++) val[i] = 0xFFFF;
    for (size_t i = 0; i < np; i++) pen[i] = 0;

    /* val[y][x][theta=0] = x * 10 → best action should move +x (decreasing cost). */
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            val[((size_t)y * W + x) * VI_N_THETA + 0] = (uint16_t)(x * 10);

    /* transitions: action 3 = +x no-op theta, others = no-op (stay). */
    for (size_t i = 0; i < nt; i++) tr[i] = 0;
    for (int it = 0; it < VI_N_THETA; it++)
        tr[3 * VI_N_THETA + it] = 0x000001;   /* dix=+1, diy=0, dit=0 */

    uint8_t *act = calloc((size_t)W * H * VI_N_THETA, 1);
    int rc = vi_compute_action_table(dev, W, H, act);
    VI_ASSERT_EQ(rc, VI_OK);

    /* Cell (x=0, y=0, theta=0) should prefer action 3 (move to x=1 which has lower cost). */
    VI_ASSERT_EQ(act[((size_t)0 * W + 0) * VI_N_THETA + 0], 3);
    /* Cell (x=W-1, y=0, theta=0): action 3 moves out of bounds → fallback action 0. */
    VI_ASSERT_EQ(act[((size_t)0 * W + (W-1)) * VI_N_THETA + 0], 0);

    free(act);
    vi_close(dev);
    vi_mock_ctx_free(ctx);
    VI_TEST_MAIN_END();
}
```

- [ ] **Step 2: Verify the test fails**

```bash
cd host && make test/test_action_table && ./test/test_action_table
```

Expected: FAIL (current stub returns `VI_ERR_BAD_ARG`).

- [ ] **Step 3: Implement vi_compute_action_table in libvi_sweep.c**

Replace the stub in `driver/uio/libvi_sweep.c`:

```c
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
```

- [ ] **Step 4: Rebuild and rerun test**

```bash
cd driver/uio && make mock-only && cd ../..
cd host && make test/test_action_table && ./test/test_action_table
```

Expected: `PASS`.

- [ ] **Step 5: Commit**

```bash
git add host/test/test_action_table.c driver/uio/libvi_sweep.c
git commit -m "feat: action table argmin with unit test"
```

---

## Task 7: PGM + YAML Map Parser

Minimal ROS map_server YAML + PGM P5 loader. No libyaml/libpng dependency.

**Files:**
- Create: `host/src/map_pgm.h`
- Create: `host/src/map_pgm.c`
- Create: `host/test/test_map_pgm.c`
- Create: `host/test/data/tiny.pgm`
- Create: `host/test/data/tiny.yaml`

- [ ] **Step 1: Write host/src/map_pgm.h**

```c
#ifndef MAP_PGM_H
#define MAP_PGM_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    int      w;
    int      h;
    double   resolution;   /* meters per cell */
    double   origin_x;     /* meters */
    double   origin_y;
    double   occupied_thresh;
    double   free_thresh;
    int      negate;       /* 0 = white is free (default), 1 = black is free */
    uint8_t *pixels;       /* w*h bytes, 0=free..255=occupied (post-negate) */
} pgm_map_t;

int  map_pgm_load(const char *yaml_path, pgm_map_t *out);
void map_pgm_free(pgm_map_t *m);

#endif
```

- [ ] **Step 2: Write host/src/map_pgm.c**

```c
#include "map_pgm.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char* read_all(const char *path, size_t *out_sz) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = malloc(sz + 1);
    if (!buf) { fclose(f); return NULL; }
    if (fread(buf, 1, sz, f) != (size_t)sz) { free(buf); fclose(f); return NULL; }
    buf[sz] = 0;
    fclose(f);
    if (out_sz) *out_sz = sz;
    return buf;
}

static int find_key(const char *text, const char *key, char *out, size_t out_sz) {
    const char *p = text;
    size_t klen = strlen(key);
    while ((p = strstr(p, key))) {
        if ((p == text || p[-1] == '\n') &&
            p[klen] == ':' ) {
            p += klen + 1;
            while (*p == ' ' || *p == '\t') p++;
            size_t n = 0;
            while (*p && *p != '\n' && *p != '#' && n + 1 < out_sz) out[n++] = *p++;
            while (n > 0 && (out[n-1] == ' ' || out[n-1] == '\t' || out[n-1] == '\r')) n--;
            out[n] = 0;
            return 0;
        }
        p++;
    }
    return -1;
}

static int parse_yaml(const char *path, pgm_map_t *m, char *image_path, size_t image_sz) {
    size_t sz;
    char *yaml = read_all(path, &sz);
    if (!yaml) return -1;

    char buf[256];
    if (find_key(yaml, "image",       buf, sizeof buf) < 0) goto bad;
    strncpy(image_path, buf, image_sz - 1); image_path[image_sz - 1] = 0;

    if (find_key(yaml, "resolution",  buf, sizeof buf) < 0) goto bad;
    m->resolution = atof(buf);

    if (find_key(yaml, "origin",      buf, sizeof buf) == 0) {
        /* "[x, y, yaw]" */
        char *p = buf;
        while (*p && (*p == '[' || *p == ' ')) p++;
        m->origin_x = atof(p);
        while (*p && *p != ',') p++;
        if (*p == ',') p++;
        m->origin_y = atof(p);
    }

    if (find_key(yaml, "occupied_thresh", buf, sizeof buf) == 0)
        m->occupied_thresh = atof(buf);
    else m->occupied_thresh = 0.65;

    if (find_key(yaml, "free_thresh", buf, sizeof buf) == 0)
        m->free_thresh = atof(buf);
    else m->free_thresh = 0.196;

    if (find_key(yaml, "negate", buf, sizeof buf) == 0)
        m->negate = atoi(buf);
    else m->negate = 0;

    free(yaml);
    return 0;
bad:
    free(yaml);
    return -1;
}

static int parse_pgm(const char *path, pgm_map_t *m) {
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    char magic[3] = {0};
    if (fread(magic, 1, 2, f) != 2) { fclose(f); return -1; }
    if (magic[0] != 'P' || magic[1] != '5') { fclose(f); return -1; }

    int w = 0, h = 0, maxv = 0;
    int got = 0;
    int c = fgetc(f);
    while (got < 3) {
        while (c == ' ' || c == '\t' || c == '\n' || c == '\r') c = fgetc(f);
        if (c == '#') { while (c != '\n' && c != EOF) c = fgetc(f); continue; }
        int val = 0;
        while (isdigit(c)) { val = val * 10 + (c - '0'); c = fgetc(f); }
        if (got == 0) w = val;
        else if (got == 1) h = val;
        else maxv = val;
        got++;
    }
    (void)maxv;

    uint8_t *pixels = malloc((size_t)w * h);
    if (!pixels) { fclose(f); return -1; }
    if (fread(pixels, 1, (size_t)w * h, f) != (size_t)w * h) {
        free(pixels); fclose(f); return -1;
    }
    fclose(f);

    m->w = w;
    m->h = h;
    m->pixels = pixels;
    return 0;
}

int map_pgm_load(const char *yaml_path, pgm_map_t *out) {
    memset(out, 0, sizeof *out);

    char image[512] = {0};
    if (parse_yaml(yaml_path, out, image, sizeof image) < 0) return -1;

    /* Resolve image path relative to yaml dir */
    char full[1024];
    const char *slash = strrchr(yaml_path, '/');
    if (!slash) slash = strrchr(yaml_path, '\\');
    if (slash && image[0] != '/' && image[1] != ':') {
        size_t dirlen = (size_t)(slash - yaml_path + 1);
        if (dirlen + strlen(image) + 1 > sizeof full) return -1;
        memcpy(full, yaml_path, dirlen);
        strcpy(full + dirlen, image);
    } else {
        strncpy(full, image, sizeof full - 1);
        full[sizeof full - 1] = 0;
    }

    if (parse_pgm(full, out) < 0) return -1;

    if (out->negate) {
        size_t n = (size_t)out->w * out->h;
        for (size_t i = 0; i < n; i++) out->pixels[i] = 255 - out->pixels[i];
    }
    return 0;
}

void map_pgm_free(pgm_map_t *m) {
    if (!m) return;
    free(m->pixels);
    m->pixels = NULL;
}
```

- [ ] **Step 3: Create test fixture tiny.pgm (4x4 binary PGM, P5)**

Use a small Python one-liner to generate the file (this fits in a single bash command):

```bash
mkdir -p host/test/data
python3 -c "
import struct, sys
w, h = 4, 4
hdr = f'P5\n{w} {h}\n255\n'.encode()
# 4x4: top row black (obstacles), rest white
pixels = bytes([0]*w + [255]*(w*3))
open('host/test/data/tiny.pgm','wb').write(hdr+pixels)
"
```

Expected: `host/test/data/tiny.pgm` is 23 bytes.

- [ ] **Step 4: Create host/test/data/tiny.yaml**

```yaml
image: tiny.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

- [ ] **Step 5: Write host/test/test_map_pgm.c**

```c
#include "vi_assert.h"
#include "map_pgm.h"

int main(void) {
    pgm_map_t m = {0};
    int rc = map_pgm_load("test/data/tiny.yaml", &m);
    VI_ASSERT_EQ(rc, 0);
    VI_ASSERT_EQ(m.w, 4);
    VI_ASSERT_EQ(m.h, 4);
    VI_ASSERT(m.resolution > 0.049 && m.resolution < 0.051);
    VI_ASSERT_EQ(m.pixels[0], 0);        /* top-left black */
    VI_ASSERT_EQ(m.pixels[4], 255);      /* second row white */
    map_pgm_free(&m);
    VI_TEST_MAIN_END();
}
```

- [ ] **Step 6: Build and run**

```bash
cd host && make test/test_map_pgm && ./test/test_map_pgm
```

Expected: `PASS`. If the test cannot find `test/data/tiny.yaml`, run from the `host/` directory (that is where the relative path resolves).

- [ ] **Step 7: Commit**

```bash
git add host/src/map_pgm.h host/src/map_pgm.c \
        host/test/test_map_pgm.c \
        host/test/data/tiny.pgm host/test/data/tiny.yaml
git commit -m "feat: PGM P5 + ROS map_server YAML parser"
```

---

## Task 8: Penalty + Transitions + Reference Bridge

Implement `penalty.c`, `transitions.c`, and a C wrapper around the existing C++ `vi_reference.cpp`.

**Files:**
- Create: `host/src/penalty.h`
- Create: `host/src/penalty.c`
- Create: `host/test/test_penalty.c`
- Create: `host/src/transitions.h`
- Create: `host/src/transitions.c`
- Create: `host/test/test_transitions.c`
- Create: `host/src/vi_reference_c.h`
- Create: `host/src/vi_reference_c.c`
- Create: `host/test/test_reference_eq.c`

- [ ] **Step 1: Write host/src/penalty.h**

```c
#ifndef PENALTY_H
#define PENALTY_H

#include <stdint.h>
#include "map_pgm.h"

/* Build a 16-bit penalty table from a PGM occupancy map.
   - Obstacle cells (pixel >= occupied_thresh*255) → 0xFFFF
   - Within safety_radius of an obstacle → scaled penalty (nearer = higher)
   - Free cells far from obstacles → 0
   - Goal cells (gx, gy) marked 0xFFFE
   pen_out must have map.w*map.h entries. */
void penalty_build(const pgm_map_t *map,
                   int safety_radius,
                   int gx, int gy,
                   uint16_t *pen_out);

#endif
```

- [ ] **Step 2: Write host/src/penalty.c**

```c
#include "penalty.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

void penalty_build(const pgm_map_t *map, int safety_radius,
                   int gx, int gy, uint16_t *pen_out) {
    int w = map->w, h = map->h;
    uint8_t occ_threshold =
        (uint8_t)(map->occupied_thresh > 0 ? map->occupied_thresh * 255 : 205);

    /* Initialise: obstacles = 0xFFFF, rest = 0. */
    for (int i = 0; i < w * h; i++) {
        pen_out[i] = (map->pixels[i] >= occ_threshold) ? 0xFFFF : 0;
    }

    /* Inflate obstacles within safety_radius with scaled penalty. */
    int r = safety_radius;
    if (r <= 0) goto goal;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (pen_out[y * w + x] != 0xFFFF) continue;
            /* Stamp a circular kernel of penalties around (x, y). */
            for (int dy = -r; dy <= r; dy++) {
                int ny = y + dy; if (ny < 0 || ny >= h) continue;
                for (int dx = -r; dx <= r; dx++) {
                    int nx = x + dx; if (nx < 0 || nx >= w) continue;
                    int d2 = dx*dx + dy*dy;
                    if (d2 > r*r) continue;
                    if (pen_out[ny * w + nx] == 0xFFFF) continue;
                    /* Higher penalty for cells closer to obstacles. */
                    double ratio = 1.0 - sqrt((double)d2) / (double)r;
                    uint16_t p = (uint16_t)(ratio * 1000.0);
                    if (p > pen_out[ny * w + nx]) pen_out[ny * w + nx] = p;
                }
            }
        }
    }

goal:
    if (gx >= 0 && gx < w && gy >= 0 && gy < h) {
        pen_out[gy * w + gx] = 0xFFFE;
    }
}
```

- [ ] **Step 3: Write host/test/test_penalty.c**

```c
#include "vi_assert.h"
#include "penalty.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int main(void) {
    pgm_map_t m = {0};
    m.w = 5; m.h = 5;
    m.occupied_thresh = 0.65;
    m.pixels = calloc(25, 1);
    /* center obstacle */
    m.pixels[2*5 + 2] = 255;

    uint16_t pen[25] = {0};
    penalty_build(&m, 1, 0, 0, pen);

    VI_ASSERT_EQ(pen[2*5 + 2], 0xFFFF);  /* center = obstacle */
    VI_ASSERT(pen[2*5 + 1] > 0);          /* adjacent cell has some penalty */
    VI_ASSERT_EQ(pen[0*5 + 0], 0xFFFE);   /* goal */
    VI_ASSERT_EQ(pen[4*5 + 4], 0);        /* far corner = free */

    free(m.pixels);
    VI_TEST_MAIN_END();
}
```

- [ ] **Step 4: Write host/src/transitions.h + transitions.c**

`host/src/transitions.h`:

```c
#ifndef TRANSITIONS_H
#define TRANSITIONS_H

#include <stdint.h>
#include "libvi_sweep.h"

/* Compute deterministic (dix, diy, dit) for each (action, theta) pair
   and pack as uint32: byte0=dix, byte1=diy, byte2=dit.
   out must have VI_N_ACTIONS*VI_N_THETA entries. */
void transitions_compute(double xy_resolution, uint32_t *out);

#endif
```

`host/src/transitions.c`:

```c
#include "transitions.h"

#include <math.h>

/* Spec §2.3 */
static const double ACTION_FW[VI_N_ACTIONS]  = { 0.3, -0.2, 0.0, 0.0, 0.3, 0.3 };
static const double ACTION_ROT[VI_N_ACTIONS] = { 0.0, 0.0, 20.0, -20.0, 20.0, -20.0 };

void transitions_compute(double xy_resolution, uint32_t *out) {
    double t_res = 360.0 / VI_N_THETA;
    for (int a = 0; a < VI_N_ACTIONS; a++) {
        for (int it = 0; it < VI_N_THETA; it++) {
            double theta_deg = it * t_res + t_res * 0.5;
            double theta_rad = theta_deg * M_PI / 180.0;
            double dx = ACTION_FW[a] * cos(theta_rad);
            double dy = ACTION_FW[a] * sin(theta_rad);
            int dix = (int)floor(dx / xy_resolution);
            int diy = (int)floor(dy / xy_resolution);

            double nt = theta_deg + ACTION_ROT[a];
            while (nt < 0) nt += 360.0;
            while (nt >= 360.0) nt -= 360.0;
            int new_it = (int)floor(nt / t_res);
            int dit = new_it - it;
            if (dit >  VI_N_THETA / 2) dit -= VI_N_THETA;
            if (dit < -VI_N_THETA / 2) dit += VI_N_THETA;

            uint32_t packed = ((uint32_t)(dix & 0xFF))
                            | ((uint32_t)(diy & 0xFF) << 8)
                            | ((uint32_t)(dit & 0xFF) << 16);
            out[a * VI_N_THETA + it] = packed;
        }
    }
}
```

- [ ] **Step 5: Write host/test/test_transitions.c**

```c
#include "vi_assert.h"
#include "transitions.h"

#include <stdlib.h>

int main(void) {
    uint32_t tr[VI_N_ACTIONS * VI_N_THETA];
    transitions_compute(0.05, tr);

    /* Action 0 (forward 0.3m) at theta = 0 should produce dix = 5 or 6 (0.3 / 0.05 = 6). */
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
```

- [ ] **Step 6: Write host/src/vi_reference_c.h**

Thin C wrapper around the existing C++ `vi_ref::run_vi` so the CLI can call it from C:

```c
#ifndef VI_REFERENCE_C_H
#define VI_REFERENCE_C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Run VI on the CPU reference solver until convergence.
   value, penalty, trans follow the same layout libvi_sweep uses. */
int vi_reference_run(uint16_t *value, const uint16_t *penalty,
                     const uint32_t *trans,
                     int map_x, int map_y,
                     uint16_t threshold, int max_sweeps);

#ifdef __cplusplus
}
#endif
#endif
```

- [ ] **Step 7: Write host/src/vi_reference_c.c**

This file is C (not C++) but calls into a C shim that delegates to `vi_ref::run_vi`. The simplest path: reimplement a minimal Jacobi VI here in C (identical algorithm to the mock), so we do not need a C++ compiler in the host build.

```c
#include "vi_reference_c.h"
#include "libvi_sweep.h"

#include <string.h>

int vi_reference_run(uint16_t *value, const uint16_t *penalty,
                     const uint32_t *trans,
                     int map_x, int map_y,
                     uint16_t threshold, int max_sweeps) {
    for (int sweep = 0; sweep < max_sweeps; sweep++) {
        uint16_t max_delta = 0;
        for (int y = 0; y < map_y; y++) {
            for (int x = 0; x < map_x; x++) {
                uint16_t cell_pen = penalty[y * map_x + x];
                if (cell_pen >= 0xFFFE) continue;

                for (int it = 0; it < VI_N_THETA; it++) {
                    size_t idx = ((size_t)y * map_x + x) * VI_N_THETA + it;
                    uint16_t old = value[idx];
                    uint16_t best = 0xFFFF;

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
                        uint16_t nv = penalty[ny * map_x + nx];
                        uint16_t nvv = value[nidx];
                        if (nvv == 0xFFFF || nv == 0xFFFF) continue;
                        uint16_t np = (nv == 0xFFFE) ? 0 : nv;
                        uint32_t s = (uint32_t)nvv + np;
                        uint16_t c = (s >= 0xFFFF) ? 0xFFFE : (uint16_t)s;
                        if (c < best) best = c;
                    }
                    value[idx] = best;
                    uint16_t d = (best > old) ? (best - old) : (old - best);
                    if (d > max_delta) max_delta = d;
                }
            }
        }
        if (max_delta <= threshold) return sweep + 1;
    }
    return max_sweeps;
}
```

- [ ] **Step 8: Write host/test/test_reference_eq.c**

```c
/* Mock-based libvi_sweep run must match reference solver on a small map. */

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

    for (size_t i = 0; i < nv; i++) val[i] = 0xFFFF;
    for (size_t i = 0; i < np; i++) pen[i] = 0;

    /* Goal at (6, 6) */
    pen[6 * W + 6] = 0xFFFE;
    for (int it = 0; it < VI_N_THETA; it++)
        val[(6 * W + 6) * VI_N_THETA + it] = 0;

    transitions_compute(0.05, tr);

    /* Capture penalty/val baseline, run reference separately. */
    uint16_t *ref_val = malloc(nv * sizeof(uint16_t));
    memcpy(ref_val, val, nv * sizeof(uint16_t));
    vi_reference_run(ref_val, pen, tr, W, H, 0, 200);

    vi_run_config_t cfg = { W, H, 0, 200 };
    vi_run_stats_t stats = {0};
    int rc = vi_run_until_converged(dev, &cfg, &stats);
    VI_ASSERT_EQ(rc, VI_OK);

    /* Compare full value table over the W*H*N_THETA area. */
    int mismatches = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            for (int it = 0; it < VI_N_THETA; it++) {
                size_t idx = ((size_t)y * W + x) * VI_N_THETA + it;
                if (ref_val[idx] != val[idx]) mismatches++;
            }
    VI_ASSERT_EQ(mismatches, 0);

    free(ref_val);
    vi_close(dev);
    vi_mock_ctx_free(ctx);
    VI_TEST_MAIN_END();
}
```

- [ ] **Step 9: Build and run all new tests**

```bash
cd host && make test-host
```

Expected: each test prints `PASS`, final line `=== All host tests PASSED ===`.

- [ ] **Step 10: Commit**

```bash
git add host/src/penalty.h host/src/penalty.c host/test/test_penalty.c \
        host/src/transitions.h host/src/transitions.c host/test/test_transitions.c \
        host/src/vi_reference_c.h host/src/vi_reference_c.c host/test/test_reference_eq.c
git commit -m "feat: penalty, transitions, reference solver + unit tests"
```

---

## Task 9: CLI Tool `vi_cli`

**Files:**
- Create: `host/src/vi_cli.c`

- [ ] **Step 1: Write host/src/vi_cli.c**

```c
/* vi_cli — command-line driver for libvi_sweep. */

#include "libvi_sweep.h"
#include "vi_device.h"
#include "map_pgm.h"
#include "penalty.h"
#include "transitions.h"
#include "vi_reference_c.h"

#include <getopt.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct {
    const char *map_path;
    int         gx, gy, gt;
    int         safety_radius;
    int         threshold;
    int         max_sweeps;
    const char *out_value;
    const char *out_action;
    int         verify;
    int         mock;
    int         verbose;
} cli_args_t;

static volatile sig_atomic_t g_interrupt = 0;
static void on_sigint(int sig) { (void)sig; g_interrupt = 1; }

static void usage(const char *prog) {
    fprintf(stderr,
        "usage: %s --map PATH.yaml --goal GX,GY[,GT] [options]\n"
        "  --safety-radius N   default 6 cells\n"
        "  --threshold N       default 0\n"
        "  --max-sweeps N      default 200\n"
        "  --out-value PATH    raw uint16 value table\n"
        "  --out-action PATH   raw uint8 action table\n"
        "  --verify            compare with CPU reference\n"
        "  --mock              use mock ops (no FPGA)\n"
        "  -v                  verbose\n", prog);
}

static int parse_goal(const char *arg, int *gx, int *gy, int *gt) {
    *gt = -1;
    return sscanf(arg, "%d,%d,%d", gx, gy, gt) >= 2 ? 0 : -1;
}

static int write_bin(const char *path, const void *data, size_t n) {
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    size_t w = fwrite(data, 1, n, f);
    fclose(f);
    return w == n ? 0 : -1;
}

int main(int argc, char **argv) {
    cli_args_t a = {0};
    a.safety_radius = 6;
    a.max_sweeps    = 200;

    static struct option opts[] = {
        {"map",           required_argument, 0, 'm'},
        {"goal",          required_argument, 0, 'g'},
        {"safety-radius", required_argument, 0, 's'},
        {"threshold",     required_argument, 0, 't'},
        {"max-sweeps",    required_argument, 0, 'n'},
        {"out-value",     required_argument, 0, 'V'},
        {"out-action",    required_argument, 0, 'A'},
        {"verify",        no_argument,       0, 'y'},
        {"mock",          no_argument,       0, 'M'},
        {"help",          no_argument,       0, 'h'},
        {0,0,0,0},
    };
    int c;
    while ((c = getopt_long(argc, argv, "v", opts, NULL)) != -1) {
        switch (c) {
        case 'm': a.map_path = optarg; break;
        case 'g':
            if (parse_goal(optarg, &a.gx, &a.gy, &a.gt) < 0) { usage(argv[0]); return 1; }
            break;
        case 's': a.safety_radius = atoi(optarg); break;
        case 't': a.threshold     = atoi(optarg); break;
        case 'n': a.max_sweeps    = atoi(optarg); break;
        case 'V': a.out_value     = optarg; break;
        case 'A': a.out_action    = optarg; break;
        case 'y': a.verify        = 1; break;
        case 'M': a.mock          = 1; break;
        case 'v': a.verbose       = 1; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }
    if (!a.map_path) { usage(argv[0]); return 1; }

    signal(SIGINT, on_sigint);

    /* --- Load map --- */
    pgm_map_t map = {0};
    if (map_pgm_load(a.map_path, &map) < 0) {
        fprintf(stderr, "failed to load map %s\n", a.map_path);
        return 1;
    }
    if (a.verbose) printf("map: %dx%d res=%.3f\n", map.w, map.h, map.resolution);

    /* --- Open device --- */
    const vi_device_ops_t *ops;
    void *ctx;
    if (a.mock) {
        ops = &vi_mock_ops;
        ctx = vi_mock_ctx_new();
    } else {
#ifndef VI_MOCK_ONLY
        ops = &vi_linux_ops;
        ctx = NULL;  /* linux ops uses a singleton internal context */
#else
        fprintf(stderr, "built without linux ops; rerun with --mock\n");
        map_pgm_free(&map);
        return 2;
#endif
    }
    vi_device_t *dev = vi_open(ops, ctx);
    if (!dev) {
        fprintf(stderr, "vi_open failed\n");
        map_pgm_free(&map);
        if (a.mock) vi_mock_ctx_free(ctx);
        return 2;
    }

    /* --- Initialise tables --- */
    size_t nv, np, nt;
    uint16_t *val = vi_value_buffer(dev, &nv);
    uint16_t *pen = vi_penalty_buffer(dev, &np);
    uint32_t *tr  = vi_trans_buffer(dev, &nt);

    if ((size_t)map.w * map.h * VI_N_THETA > nv ||
        (size_t)map.w * map.h > np) {
        fprintf(stderr, "map too large for preallocated buffers\n");
        vi_close(dev);
        if (a.mock) vi_mock_ctx_free(ctx);
        map_pgm_free(&map);
        return 1;
    }

    for (size_t i = 0; i < (size_t)map.w * map.h * VI_N_THETA; i++) val[i] = 0xFFFF;
    penalty_build(&map, a.safety_radius, a.gx, a.gy, pen);

    /* Mark goal value = 0 */
    if (a.gx >= 0 && a.gx < map.w && a.gy >= 0 && a.gy < map.h) {
        if (a.gt >= 0 && a.gt < VI_N_THETA) {
            val[((size_t)a.gy * map.w + a.gx) * VI_N_THETA + a.gt] = 0;
        } else {
            for (int it = 0; it < VI_N_THETA; it++)
                val[((size_t)a.gy * map.w + a.gx) * VI_N_THETA + it] = 0;
        }
    }

    transitions_compute(map.resolution, tr);

    /* --- Run --- */
    vi_run_config_t cfg = { map.w, map.h, (uint16_t)a.threshold, a.max_sweeps };
    vi_run_stats_t stats = {0};
    int rc = vi_run_until_converged(dev, &cfg, &stats);

    printf("sweeps=%d final_delta=%u converged=%d elapsed=%.3fs\n",
           stats.sweeps, (unsigned)stats.final_delta, stats.converged, stats.elapsed_sec);

    int exit_code = (rc == VI_OK) ? 0 : 3;

    /* --- Verify against reference --- */
    if (a.verify) {
        size_t n = (size_t)map.w * map.h * VI_N_THETA;
        uint16_t *ref_val = malloc(n * sizeof(uint16_t));
        for (size_t i = 0; i < n; i++) ref_val[i] = 0xFFFF;
        if (a.gx >= 0 && a.gx < map.w && a.gy >= 0 && a.gy < map.h)
            for (int it = 0; it < VI_N_THETA; it++)
                ref_val[((size_t)a.gy * map.w + a.gx) * VI_N_THETA + it] = 0;
        vi_reference_run(ref_val, pen, tr, map.w, map.h, (uint16_t)a.threshold, a.max_sweeps);

        int mismatches = 0;
        for (size_t i = 0; i < n; i++)
            if (ref_val[i] != val[i]) mismatches++;
        printf("verify: %s (%d mismatches)\n",
               mismatches == 0 ? "PASS" : "FAIL", mismatches);
        if (mismatches) exit_code = 3;
        free(ref_val);
    }

    /* --- Outputs --- */
    if (a.out_value)
        write_bin(a.out_value, val, (size_t)map.w * map.h * VI_N_THETA * 2);

    if (a.out_action) {
        uint8_t *act = malloc((size_t)map.w * map.h * VI_N_THETA);
        if (vi_compute_action_table(dev, map.w, map.h, act) == VI_OK)
            write_bin(a.out_action, act, (size_t)map.w * map.h * VI_N_THETA);
        free(act);
    }

    vi_close(dev);
    if (a.mock) vi_mock_ctx_free(ctx);
    map_pgm_free(&map);
    return exit_code;
}
```

- [ ] **Step 2: Build vi_cli_mock (mock-only, since vi_device_linux.c is not yet written)**

```bash
cd host && make cli-mock && cd ..
```

Expected: `host/vi_cli_mock` produced.

- [ ] **Step 3: Smoke-run the CLI against the tiny fixture**

```bash
cd host
./vi_cli_mock --map test/data/tiny.yaml --goal 2,2 --mock --verify --max-sweeps 50
```

Expected: `verify: PASS`, exit code 0.

- [ ] **Step 4: Commit**

```bash
git add host/src/vi_cli.c
git commit -m "feat: vi_cli CLI tool with verify mode"
```

---

## Task 10: Sync HLS Register Header

Copy the HLS-generated `xvi_sweep_hw.h` into `driver/uio/generated/` so `vi_device_linux.c` can include it.

**Files:**
- Create: `driver/uio/generated/xvi_sweep_hw.h` (via make target)

- [ ] **Step 1: Run sync**

```bash
cd driver/uio && make sync-hw-header
```

Expected: `generated/xvi_sweep_hw.h` created. If the HLS build directory does not exist, run `cd fpga/scripts && make hls` first.

- [ ] **Step 2: Inspect and record the actual offsets**

```bash
grep -E "CONTROL_ADDR.*OFFSET" driver/uio/generated/xvi_sweep_hw.h
```

Compare the offsets to the `VI_OFF_*` constants in `driver/uio/libvi_sweep.c`. If any offset differs, update the constants in `libvi_sweep.c` to match, then rebuild and re-run `test-host` to confirm nothing broke.

- [ ] **Step 3: Commit the synced header**

```bash
git add driver/uio/generated/xvi_sweep_hw.h
git commit -m "chore: sync xvi_sweep_hw.h from HLS output"
```

---

## Task 11: vi_device_linux.c — Real UIO + u-dma-buf Implementation

The on-target device ops. Cannot be unit-tested on the host, so it will be smoke-tested on the Ultra96 via `vi_cli` in Task 13.

**Files:**
- Create: `driver/uio/vi_device_linux.c`

- [ ] **Step 1: Write driver/uio/vi_device_linux.c**

```c
/* vi_device_linux.c — Linux implementation of vi_device_ops using UIO + u-dma-buf. */

#ifndef VI_MOCK_ONLY

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
        char npath[256], nbuf[128];
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
        if (num < 0) return VI_ERR_OPEN;
        char path[64];
        snprintf(path, sizeof path, "/dev/uio%d", num);
        c->uio_fd[i] = open(path, O_RDWR);
        if (c->uio_fd[i] < 0) return VI_ERR_OPEN;
        c->ctrl[i] = mmap(NULL, VI_CTRL_SIZE, PROT_READ | PROT_WRITE,
                          MAP_SHARED, c->uio_fd[i], 0);
        if (c->ctrl[i] == MAP_FAILED) return VI_ERR_MMAP;
    }

    /* --- udmabuf nodes --- */
    c->udma_value_fd   = open(UDMA_VALUE_DEV, O_RDWR);
    c->udma_pendata_fd = open(UDMA_PENDATA_DEV, O_RDWR);
    if (c->udma_value_fd < 0 || c->udma_pendata_fd < 0) return VI_ERR_OPEN;

    c->value_size  = read_udma_size(UDMA_VALUE_SYS);
    c->pendata_size = read_udma_size(UDMA_PENDATA_SYS);
    if (c->value_size == 0 || c->pendata_size == 0) return VI_ERR_OPEN;
    c->value_phys  = read_udma_phys(UDMA_VALUE_SYS);
    c->pendata_phys = read_udma_phys(UDMA_PENDATA_SYS);

    c->value_mmap   = mmap(NULL, c->value_size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, c->udma_value_fd, 0);
    c->pendata_mmap = mmap(NULL, c->pendata_size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, c->udma_pendata_fd, 0);
    if (c->value_mmap == MAP_FAILED || c->pendata_mmap == MAP_FAILED) return VI_ERR_MMAP;

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
    if (c->value_mmap   && c->value_mmap   != MAP_FAILED) munmap(c->value_mmap,   c->value_size);
    if (c->pendata_mmap && c->pendata_mmap != MAP_FAILED) munmap(c->pendata_mmap, c->pendata_size);
    if (c->udma_value_fd   >= 0) close(c->udma_value_fd);
    if (c->udma_pendata_fd >= 0) close(c->udma_pendata_fd);
    for (int i = 0; i < VI_NUM_CU; i++) {
        if (c->ctrl[i]  && c->ctrl[i]  != MAP_FAILED) munmap((void*)c->ctrl[i], VI_CTRL_SIZE);
        if (c->uio_fd[i] >= 0) close(c->uio_fd[i]);
    }
    memset(c, 0, sizeof *c);
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
    struct pollfd pfd = { .fd = c->uio_fd[cu], .events = POLLIN };
    int rc = poll(&pfd, 1, timeout_ms);
    if (rc <= 0) return VI_ERR_IRQ;
    uint32_t count;
    if (read(c->uio_fd[cu], &count, 4) != 4) return VI_ERR_IRQ;
    /* ack ISR bit 0 (W1C), re-arm UIO */
    c->ctrl[cu][0x0C / 4] = 0x1;
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
```

- [ ] **Step 2: Build full libvi_sweep + vi_cli for target**

```bash
cd driver/uio && make clean && make && cd ../..
cd host && make clean && make && cd ..
```

Expected: `driver/uio/libvi_sweep.{a,so}` and `host/vi_cli` produced without warnings.

- [ ] **Step 3: Commit**

```bash
git add driver/uio/vi_device_linux.c
git commit -m "feat: Linux vi_device ops (UIO + u-dma-buf)"
```

---

## Task 12: Device Tree

**Files:**
- Create: `driver/dts/vi_sweep.dtsi`
- Create: `driver/dts/README.md`

- [ ] **Step 1: Look up SPI numbers recorded in Task 1**

```bash
cat fpga/vivado/ultra96v2/irq_notes.txt
```

Copy the two SPI numbers into the `interrupts = <0 <N> IRQ_TYPE_LEVEL_HIGH>` lines below.

- [ ] **Step 2: Write driver/dts/vi_sweep.dtsi**

Replace `<SPI0>` and `<SPI1>` with the numbers from the previous step.

```dts
/include/ "dt-bindings/interrupt-controller/irq.h"

/ {
    reserved-memory {
        #address-cells = <2>;
        #size-cells    = <2>;
        ranges;

        vi_value_rsv: vi_value@20000000 {
            reg = <0x0 0x20000000  0x0 0x56000000>;
            no-map;
        };

        vi_pendata_rsv: vi_pendata@76000000 {
            reg = <0x0 0x76000000  0x0 0x02000000>;
            no-map;
        };
    };

    udmabuf_value {
        compatible = "ikwzm,u-dma-buf";
        device-name = "udmabuf_value";
        minor-number = <0>;
        size = <0x56000000>;
        memory-region = <&vi_value_rsv>;
        sync-mode = <1>;
    };

    udmabuf_pendata {
        compatible = "ikwzm,u-dma-buf";
        device-name = "udmabuf_pendata";
        minor-number = <1>;
        size = <0x02000000>;
        memory-region = <&vi_pendata_rsv>;
        sync-mode = <1>;
    };
};

&amba_pl {
    vi_sweep_cu0: vi_sweep@a0000000 {
        compatible = "generic-uio";
        reg = <0x0 0xa0000000 0x0 0x10000>;
        interrupt-parent = <&gic>;
        interrupts = <0 <SPI0> IRQ_TYPE_LEVEL_HIGH>;
    };
    vi_sweep_cu1: vi_sweep@a0010000 {
        compatible = "generic-uio";
        reg = <0x0 0xa0010000 0x0 0x10000>;
        interrupt-parent = <&gic>;
        interrupts = <0 <SPI1> IRQ_TYPE_LEVEL_HIGH>;
    };
};
```

- [ ] **Step 3: Write driver/dts/README.md**

```markdown
# vi_sweep Device Tree overlay

Include `vi_sweep.dtsi` from `project-spec/meta-user/recipes-bsp/device-tree/files/system-user.dtsi`:

```dts
/include/ "vi_sweep.dtsi"
```

## Kernel configuration

Enable the following in `petalinux-config -c kernel`:

- `CONFIG_UIO=y`
- `CONFIG_UIO_PDRV_GENIRQ=y`
- `CONFIG_CMA_SIZE_MBYTES=1536` (or higher; reserved-memory `no-map` requires CMA headroom)

Add to bootargs (`petalinux-config`, subsystem → boot args):

```
uio_pdrv_genirq.of_id=generic-uio
```

## u-dma-buf module

Add ikwzm's `u-dma-buf` as an external module under `meta-user/recipes-modules/u-dma-buf/`.
On first boot, `modprobe u-dma-buf` (or add to `/etc/modules`) and confirm:

```
ls -l /dev/udmabuf_value /dev/udmabuf_pendata
cat /sys/class/u-dma-buf/udmabuf_value/phys_addr
```

## Verification after boot

```
ls -l /dev/uio*
dmesg | grep -i uio
dmesg | grep -i udma
```

`uio0` and `uio1` should appear with names `vi_sweep_cu0` and `vi_sweep_cu1`.
```

- [ ] **Step 4: Commit**

```bash
git add driver/dts/vi_sweep.dtsi driver/dts/README.md
git commit -m "feat: Petalinux device tree overlay for vi_sweep + udmabuf"
```

---

## Task 13: HW Integration Test Scripts

Shell scripts that SSH into the Ultra96 and run `vi_cli` with a synthesized tiny map and with a large map, asserting PASS and timing.

**Files:**
- Create: `host/test/hw/make_tiny_map.py`
- Create: `host/test/hw/run_smoke.sh`
- Create: `host/test/hw/run_big.sh`

- [ ] **Step 1: Write make_tiny_map.py**

```python
#!/usr/bin/env python3
"""Generate a tiny synthetic PGM + YAML for HW smoke testing."""
import sys, os

W, H = 40, 40
resolution = 0.05
pixels = bytearray(255 for _ in range(W * H))
# Add a vertical wall near the middle with a gap
for y in range(H):
    if 15 <= y < 25: continue
    pixels[y * W + 20] = 0

out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
os.makedirs(out_dir, exist_ok=True)

with open(os.path.join(out_dir, "smoke.pgm"), "wb") as f:
    f.write(f"P5\n{W} {H}\n255\n".encode())
    f.write(bytes(pixels))

with open(os.path.join(out_dir, "smoke.yaml"), "w") as f:
    f.write(f"image: smoke.pgm\nresolution: {resolution}\n"
            f"origin: [0.0, 0.0, 0.0]\n"
            f"occupied_thresh: 0.65\nfree_thresh: 0.196\nnegate: 0\n")

print(f"Wrote {out_dir}/smoke.{{pgm,yaml}}")
```

- [ ] **Step 2: Write run_smoke.sh**

```bash
#!/usr/bin/env bash
set -euo pipefail
: "${VI_TARGET_HOST:?set VI_TARGET_HOST to the Ultra96 hostname}"

TMPDIR=$(mktemp -d)
python3 host/test/hw/make_tiny_map.py "$TMPDIR"

# Copy CLI + map to target
scp host/vi_cli "$TMPDIR"/smoke.pgm "$TMPDIR"/smoke.yaml \
    "$VI_TARGET_HOST":/tmp/

ssh "$VI_TARGET_HOST" '
    cd /tmp &&
    ./vi_cli --map smoke.yaml --goal 35,20 --verify \
             --threshold 0 --max-sweeps 100
' | tee "$TMPDIR/smoke.log"

if grep -q "verify: PASS" "$TMPDIR/smoke.log"; then
    echo "=== HW smoke test PASSED ==="
else
    echo "=== HW smoke test FAILED ==="
    exit 1
fi
```

- [ ] **Step 3: Write run_big.sh**

```bash
#!/usr/bin/env bash
set -euo pipefail
: "${VI_TARGET_HOST:?set VI_TARGET_HOST}"
: "${VI_BIG_MAP_YAML:?set VI_BIG_MAP_YAML to the 700m campus map}"
: "${VI_BIG_GOAL:?set VI_BIG_GOAL as 'GX,GY[,GT]'}"

BIG_DIR=$(dirname "$VI_BIG_MAP_YAML")
PGM_NAME=$(awk '/^image:/ {print $2}' "$VI_BIG_MAP_YAML")

scp host/vi_cli "$VI_BIG_MAP_YAML" "$BIG_DIR/$PGM_NAME" \
    "$VI_TARGET_HOST":/tmp/

TARGET_YAML=/tmp/$(basename "$VI_BIG_MAP_YAML")

ssh "$VI_TARGET_HOST" "
    cd /tmp &&
    /usr/bin/time -v ./vi_cli --map $(basename $VI_BIG_MAP_YAML) \
        --goal $VI_BIG_GOAL \
        --threshold 0 --max-sweeps 50 -v
" 2>&1 | tee /tmp/vi_big.log

elapsed=$(grep -oP 'elapsed=\K[0-9.]+' /tmp/vi_big.log | tail -1)
echo "elapsed seconds: $elapsed"

if awk "BEGIN {exit !($elapsed < 60.0)}"; then
    echo "=== HW big-map test PASSED (under 60 s) ==="
else
    echo "=== HW big-map test FAILED (>= 60 s) ==="
    exit 1
fi
```

- [ ] **Step 4: Make scripts executable**

```bash
chmod +x host/test/hw/make_tiny_map.py host/test/hw/run_smoke.sh host/test/hw/run_big.sh
```

- [ ] **Step 5: Commit**

```bash
git add host/test/hw/make_tiny_map.py host/test/hw/run_smoke.sh host/test/hw/run_big.sh
git commit -m "feat: HW integration test scripts (SSH-driven)"
```

---

## Task 14: Full Verification Pass

Final end-to-end checks.

**Files:** none (verification only)

- [ ] **Step 1: Full host build and test**

```bash
make clean
make driver
make host
make test-host
```

Expected: `=== All host tests PASSED ===`.

- [ ] **Step 2: Build Petalinux image**

From the Petalinux project directory (user's own), rebuild the image with the new DT overlay:

```bash
petalinux-build
petalinux-package --boot --fsbl images/linux/zynqmp_fsbl.elf \
    --fpga ../fpga/pynq/vi_bd_wrapper.bit --u-boot --force
```

Flash to SD card and boot.

- [ ] **Step 3: Verify device nodes on target**

```bash
ssh $VI_TARGET_HOST "
    ls -l /dev/uio0 /dev/uio1 /dev/udmabuf_value /dev/udmabuf_pendata &&
    cat /sys/class/uio/uio0/name /sys/class/uio/uio1/name &&
    cat /sys/class/u-dma-buf/udmabuf_value/phys_addr \
        /sys/class/u-dma-buf/udmabuf_pendata/phys_addr
"
```

Expected: both UIO names, both udma phys addresses present.

- [ ] **Step 4: Run HW smoke test**

```bash
export VI_TARGET_HOST=<ultra96-host>
make test-hw
```

Expected: `=== HW smoke test PASSED ===`.

- [ ] **Step 5: Run HW big map test**

```bash
export VI_BIG_MAP_YAML=/path/to/campus700m.yaml
export VI_BIG_GOAL=13900,400
bash host/test/hw/run_big.sh
```

Expected: collaboration with Phase 1-2 optimizations should complete within 60 s as per spec §12 acceptance criteria. If it exceeds, consult spec §10 Risks (BRAM/timing fallback) before declaring Phase 3 done.

- [ ] **Step 6: Sign-off**

When all Acceptance Criteria in spec §12 are checked off, Phase 3 is complete. Move on to Phase 4 (ROS2 node) via a new brainstorming session.
