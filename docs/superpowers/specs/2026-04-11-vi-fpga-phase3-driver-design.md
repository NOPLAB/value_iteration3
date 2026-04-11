# Value Iteration FPGA — Phase 3 Linux UIO Driver Design Spec

**Date:** 2026-04-11
**Target Board:** Ultra96-V2 (Zynq UltraScale+ ZU3EG)
**Target OS:** Petalinux (self-built)
**Parent spec:** `2026-04-10-value-iteration-fpga-design.md` (Phase 1-2)
**Goal:** Phase 2 で生成したビットストリーム (`vi_sweep` x2 CU) を Linux ユーザ空間から
UIO + u-dma-buf 経由で制御し、収束まで Sweep を回す C ライブラリと CLI ツールを提供する。
ROS2 ノード (Phase 4) から `#include "libvi_sweep.h"` でリンクできる基盤。

---

## 1. Scope

### In Scope
- Petalinux 向けデバイスツリー (`vi_sweep.dtsi`)
- `libvi_sweep` C ライブラリ (静的 + 共有)
  - UIO 経由のレジスタ R/W・割り込み待機
  - u-dma-buf 経由の DDR バッファ確保・mmap
  - 収束 Sweep ループ制御
  - 収束後の最適アクションテーブル計算 (ARM 側 argmin)
- `vi_cli` CLI ツール
  - ROS map_server 互換 PGM + YAML マップ読み込み
  - 参照実装との verify モード
  - モックモード (実機なしでホスト検証)
- ホストユニットテスト (モック経由) + 実機統合テスト

### Out of Scope
- ROS2 ノード (Phase 4)
- Petalinux/Yocto レシピ (meta-user/recipes-apps) — TODO として記載のみ
- Warm start / ランタイムアクション差し替え
- Bitstream ホットリロード

### Prerequisites (Phase 2 追補)
以下は Phase 3 着手前に片付ける小さな Phase 2 追補とする:

1. **Vivado BD の割り込み配線追加** — `fpga/vivado/ultra96v2/create_bd.tcl` に `xlconcat` IP を追加し、
   `vi_sweep_cu0/interrupt` + `vi_sweep_cu1/interrupt` を `zynq_ps/pl_ps_irq0[1:0]` に接続。
2. **ビットストリーム再生成** — BD 変更後 `make all` で `.bit` / `.hwh` を再出力。
3. HLS IP 自体は `ap_ctrl_hs` + interrupt 出力付きで生成済み(`xvi_sweep.h` の
   `InterruptEnable/Clear/GetStatus` ヘルパー存在から確認済み)のため、
   HLS コード変更は不要。

前提条件の完了確認は Phase 3 実装開始時のチェックリスト冒頭に置く。

---

## 2. Architecture

### 2.1 Layering

```
┌───────────────────────────────────────────────────────────┐
│  vi_cli (host/src/vi_cli.c)                               │
│    argv parse, PGM/YAML load, penalty/trans compute,      │
│    golden compare (小マップ時)                            │
└────────────┬──────────────────────────────────────────────┘
             │ libvi_sweep API (libvi_sweep.h)
┌────────────▼──────────────────────────────────────────────┐
│  libvi_sweep (driver/uio/libvi_sweep.c)                   │
│    vi_open / vi_close                                     │
│    vi_{value,penalty,trans}_buffer                        │
│    vi_run_until_converged                                 │
│    vi_compute_action_table                                │
└────────────┬──────────────────────────────────────────────┘
             │ vi_device_ops_t (vi_device.h)
    ┌────────┴─────────────┐
    │                      │
┌───▼──────────────┐  ┌────▼─────────────┐
│ vi_device_linux  │  │ vi_device_mock   │
│  (本番)          │  │  (ホストテスト)   │
│  UIO + udmabuf   │  │  内部で参照実装  │
│  + xvi_sweep_hw.h│  │  を走らせる      │
└───┬──────────────┘  └──────────────────┘
    │
┌───▼───────────────────────────────────┐
│ Linux Kernel (Petalinux)              │
│   uio_pdrv_genirq (2 instances)       │
│   u-dma-buf    (2 instances)          │
│   reserved-memory (CMA 1.5 GB)        │
└───┬───────────────────────────────────┘
    │
┌───▼───────────────────────────────────┐
│ PL: vi_sweep CU0/CU1 + HP0 + GP0      │
└───────────────────────────────────────┘
```

### 2.2 Key Design Decisions

| # | Decision | Rationale |
|---|---|---|
| 1 | Petalinux を自前ビルド | 最小構成・DT 自由度・UIO/u-dma-buf 組み込み容易 |
| 2 | u-dma-buf (ikwzm) でバッファ確保 | Petalinux 界隈で実績多、`no-map` reserved-memory と連携 |
| 3 | バッファを 2 ノードに分離 (Value / Penalty+Trans) | HLS の gmem0/gmem1 バンドル分離と対称、DMA 競合回避 |
| 4 | UIO 割り込み (`read(/dev/uioX)` ブロック) | CPU 負荷低減、UIO 本来の使い方 |
| 5 | C ライブラリ + CLI ツール | ROS2 ノードから直接リンク可能、実機単体リグレッション可能 |
| 6 | `vi_device_ops_t` による薄い抽象化 | モック差し替えでホスト CI が可能、Phase 4 でも再利用 |
| 7 | HLS 生成 `xvi_sweep_hw.h` を vendoring | レジスタオフセットハードコード撤去、HLS 再生成時の自動追随 |
| 8 | HLS 生成 `xvi_sweep.c/linux.c` は **使わない** | `static uio_info` のマルチインスタンスバグ・udmabuf 非対応のため、init/mmap/IRQ は自前実装 |
| 9 | マップ入力: ROS map_server 互換 PGM + YAML | 既存コースマップがこの形式、ROS2 統合で自然に接続 |

---

## 3. Device Tree

`driver/dts/vi_sweep.dtsi`:

```dts
/ {
    reserved-memory {
        #address-cells = <2>;
        #size-cells    = <2>;
        ranges;

        /* Value table: 1.34 GB 本体 + ガード, R/W */
        vi_value_rsv: vi_value@20000000 {
            reg = <0x0 0x20000000  0x0 0x56000000>;  /* ~1.375 GB */
            no-map;
        };

        /* Penalty + Trans: 32 MB, RO */
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
        sync-mode = <1>;  /* SYNC_MODE_NONCACHED */
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
        interrupts = <0 89 IRQ_TYPE_LEVEL_HIGH>;  /* pl_ps_irq0[0], SPI 番号は実装時に Vivado Address Editor で確認 */
    };
    vi_sweep_cu1: vi_sweep@a0010000 {
        compatible = "generic-uio";
        reg = <0x0 0xa0010000 0x0 0x10000>;
        interrupt-parent = <&gic>;
        interrupts = <0 90 IRQ_TYPE_LEVEL_HIGH>;  /* pl_ps_irq0[1], SPI 番号は実装時に Vivado Address Editor で確認 */
    };
};
```

**ポイント:**
- `no-map` で Linux の linear map から除外 → u-dma-buf が物理連続 1.375 GB を確保
- `sync-mode = <1>` (non-cached) により CPU/PL 間のキャッシュ一貫性を明示的 sync なしで担保
- `0x20000000` 以降を予約: PS DDR 2GB のうち下位 ~512 MB が Linux 用、上位 ~1.4 GB が FPGA 用
- SPI 番号は Zynq UltraScale+ の `pl_ps_irq0[1:0]` に対応(実際の番号は実装時に Vivado の IP Integrator で確認)
- `uio_pdrv_genirq.of_id=generic-uio` を bootargs に追加し `generic-uio` ノードを UIO として認識

**Petalinux 側の追加設定:**
- `petalinux-config -c kernel`: `CONFIG_UIO=y`, `CONFIG_UIO_PDRV_GENIRQ=y`
- `u-dma-buf` を `meta-user/recipes-modules/u-dma-buf/` に recipe 追加(ikwzm 公式 bb ファイル)
- `project-spec/meta-user/recipes-bsp/device-tree/files/system-user.dtsi` から本 dtsi を include
- `/etc/modules` に `u-dma-buf` を追加

Petalinux 設定変更の手順は `driver/dts/README.md` に実装時に記載する。

---

## 4. libvi_sweep Public API

### 4.1 Header `driver/uio/libvi_sweep.h`

```c
#ifndef LIBVI_SWEEP_H
#define LIBVI_SWEEP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VI_N_THETA    60
#define VI_N_ACTIONS   6
#define VI_TILE_W     32
#define VI_TILE_H     32
#define VI_NUM_CU      2

/* 最大マップサイズ(バッファ確保時の worst case, spec §3 の 700m x 40m 基準) */
#define VI_MAX_MAP_X    14000
#define VI_MAX_MAP_Y    800

typedef struct vi_device      vi_device_t;
typedef struct vi_device_ops  vi_device_ops_t;  /* §5 参照 */

typedef struct {
    int       map_x;
    int       map_y;
    uint16_t  threshold;
    int       max_sweeps;
} vi_run_config_t;

typedef struct {
    int       sweeps;
    uint16_t  final_delta;
    double    elapsed_sec;
    int       converged;
} vi_run_stats_t;

/* Lifecycle */
vi_device_t* vi_open(const vi_device_ops_t *ops, void *ctx);
void         vi_close(vi_device_t *dev);

/* Direct buffer access (zero-copy, user writes here) */
uint16_t* vi_value_buffer(vi_device_t *dev,   size_t *n_u16);
uint16_t* vi_penalty_buffer(vi_device_t *dev, size_t *n_u16);
uint32_t* vi_trans_buffer(vi_device_t *dev,   size_t *n_u32);

/* Run */
int vi_run_until_converged(vi_device_t *dev,
                           const vi_run_config_t *cfg,
                           vi_run_stats_t *stats);

/* Post-convergence argmin pass */
int vi_compute_action_table(vi_device_t *dev,
                            int map_x, int map_y,
                            uint8_t *action_out);

/* Errors */
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

### 4.2 Sweep ループ内部動作

`vi_run_until_converged` は以下を繰り返す:

1. 両 CU のレジスタ設定(両方とも同じ `map_x/map_y/num_tiles_x/num_tiles_y`、
   `value_table`/`penalty_table`/`trans_table` 物理アドレスは `vi_*_buffer` で取得した値)
2. `cu_id` を CU0=0, CU1=1 で設定
3. 両 CU で IER に `ap_done` enable bit を書き、GIE レジスタで global interrupt enable
4. `AP_CTRL[0]=1` で両 CU 同時起動
5. `poll()` で `/dev/uio0` と `/dev/uio1` を同時待機、両方完了で次へ
6. `read(fd, 4)` で割り込み件数を消費、ISR に 1 を書いて ack(W1C)
7. `max_delta` を両 CU から読み、`max(d0, d1)` を閾値比較
8. `max_sweeps` 超過なら `VI_ERR_NOT_CONV`、それ以外は次 Sweep へ
9. `write(fd, 1, 4)` で UIO 割り込み再アーム

### 4.3 ライブラリビルド

- `driver/uio/Makefile` で `libvi_sweep.a` と `libvi_sweep.so` 両方を生成
- 依存: libc + pthread のみ
- `-fvisibility=hidden` で公開シンボルを `libvi_sweep.h` の関数だけに絞る

---

## 5. Device Ops Abstraction

### 5.1 Interface `driver/uio/vi_device.h`

```c
#ifndef VI_DEVICE_H
#define VI_DEVICE_H

#include <stddef.h>
#include <stdint.h>

typedef struct vi_device_ops {
    /* Called from vi_open; returns 0 on success, negative on failure. */
    int      (*init)(void *ctx);

    /* Release all resources. */
    void     (*shutdown)(void *ctx);

    /* Control register R/W for CU 0 or 1. */
    uint32_t (*read_reg) (void *ctx, int cu, uint32_t off);
    void     (*write_reg)(void *ctx, int cu, uint32_t off, uint32_t v);

    /* Block until CU[cu] raises its interrupt, then ack.
       Returns 0 on success, negative on timeout/error. */
    int      (*wait_irq)(void *ctx, int cu, int timeout_ms);

    /* Return a mmapped buffer usable from CPU.
       buf_id: 0=value, 1=penalty, 2=trans.
       On success, *size is the buffer byte size and *phys is the
       physical address to program into the CU registers. */
    void*    (*map_buf)  (void *ctx, int buf_id,
                          size_t *size, uint64_t *phys);
} vi_device_ops_t;

extern const vi_device_ops_t vi_linux_ops;  /* vi_device_linux.c */
extern const vi_device_ops_t vi_mock_ops;   /* vi_device_mock.c  */

#endif
```

### 5.2 Linux 実装 `driver/uio/vi_device_linux.c`

HLS 生成ドライバを **部分的に** 利用する:

- **使うもの:**
  - `generated/xvi_sweep_hw.h` — レジスタオフセットの正規定義
  - `xvi_sweep.h` の Interrupt* 関数のロジック(コピペで再実装、static 依存を除く)

- **使わないもの:**
  - `xvi_sweep.c` — Set/Get 関数は薄いラッパーで再実装が不要なら不要
  - `xvi_sweep_linux.c` — `static uio_info` がマルチインスタンス非対応、udmabuf 非対応

**実装スケルトン:**

```c
#include "vi_device.h"
#include "generated/xvi_sweep_hw.h"

#define VI_BUF_VALUE    0
#define VI_BUF_PENALTY  1
#define VI_BUF_TRANS    2

typedef struct {
    int       uio_fd[2];
    volatile uint32_t *ctrl[2];  /* mmap'd control region per CU */
    size_t    ctrl_size[2];

    int       udma_value_fd;
    int       udma_pendata_fd;
    void     *value_mmap;   size_t value_size;   uint64_t value_phys;
    void     *pen_mmap;     size_t pen_size;     uint64_t pen_phys;
    void     *trans_mmap;   size_t trans_size;   uint64_t trans_phys;
    /* pen_mmap と trans_mmap は udmabuf_pendata 内の offset 分割 */
} vi_linux_ctx_t;

static int find_uio_by_name(const char *name) {
    /* /sys/class/uio/uio*/name を走査して一致する番号を返す */
}

static uint64_t read_udma_phys(const char *name) {
    /* /sys/class/u-dma-buf/<name>/phys_addr から読む */
}

static int linux_init(void *ctx) {
    vi_linux_ctx_t *c = ctx;

    /* --- 2 UIO nodes --- */
    for (int i = 0; i < 2; i++) {
        char path[64];
        int uio_num = find_uio_by_name(i == 0 ? "vi_sweep_cu0"
                                               : "vi_sweep_cu1");
        if (uio_num < 0) return VI_ERR_OPEN;
        snprintf(path, sizeof path, "/dev/uio%d", uio_num);
        c->uio_fd[i] = open(path, O_RDWR);
        if (c->uio_fd[i] < 0) return VI_ERR_OPEN;

        c->ctrl_size[i] = 0x10000;
        c->ctrl[i] = mmap(NULL, c->ctrl_size[i],
                          PROT_READ | PROT_WRITE, MAP_SHARED,
                          c->uio_fd[i], 0);
        if (c->ctrl[i] == MAP_FAILED) return VI_ERR_MMAP;
    }

    /* --- 2 udmabuf nodes --- */
    c->udma_value_fd   = open("/dev/udmabuf_value",   O_RDWR);
    c->udma_pendata_fd = open("/dev/udmabuf_pendata", O_RDWR);
    if (c->udma_value_fd < 0 || c->udma_pendata_fd < 0) return VI_ERR_OPEN;

    c->value_size = 0x56000000;
    c->value_mmap = mmap(NULL, c->value_size, PROT_READ | PROT_WRITE,
                         MAP_SHARED, c->udma_value_fd, 0);
    c->value_phys = read_udma_phys("udmabuf_value");

    /* Penalty + Trans は 1 つの udmabuf 内にオフセット分割 */
    size_t pendata_size = 0x02000000;
    void  *pendata_map  = mmap(NULL, pendata_size, PROT_READ | PROT_WRITE,
                               MAP_SHARED, c->udma_pendata_fd, 0);
    uint64_t pendata_phys = read_udma_phys("udmabuf_pendata");

    /* 最悪ケースの Penalty/Trans サイズ(spec §3 の 700m x 40m フル) */
    c->pen_size = VI_MAX_MAP_X * VI_MAX_MAP_Y * sizeof(uint16_t);
    c->pen_mmap = pendata_map;
    c->pen_phys = pendata_phys;

    c->trans_size = VI_N_ACTIONS * VI_N_THETA * sizeof(uint32_t);
    c->trans_mmap = (char*)pendata_map + c->pen_size;
    c->trans_phys = pendata_phys + c->pen_size;

    return 0;
}

static void linux_write_reg(void *ctx, int cu, uint32_t off, uint32_t v) {
    vi_linux_ctx_t *c = ctx;
    c->ctrl[cu][off / 4] = v;
}

static uint32_t linux_read_reg(void *ctx, int cu, uint32_t off) {
    vi_linux_ctx_t *c = ctx;
    return c->ctrl[cu][off / 4];
}

static int linux_wait_irq(void *ctx, int cu, int timeout_ms) {
    vi_linux_ctx_t *c = ctx;
    struct pollfd pfd = { .fd = c->uio_fd[cu], .events = POLLIN };
    int rc = poll(&pfd, 1, timeout_ms);
    if (rc <= 0) return VI_ERR_IRQ;

    uint32_t count;
    if (read(c->uio_fd[cu], &count, 4) != 4) return VI_ERR_IRQ;

    /* ap_done ISR clear */
    linux_write_reg(ctx, cu, XVI_SWEEP_CONTROL_ADDR_ISR, 0x1);

    /* Re-arm */
    uint32_t one = 1;
    write(c->uio_fd[cu], &one, 4);
    return 0;
}

static void* linux_map_buf(void *ctx, int buf_id,
                           size_t *size, uint64_t *phys) {
    vi_linux_ctx_t *c = ctx;
    switch (buf_id) {
    case VI_BUF_VALUE:   *size = c->value_size; *phys = c->value_phys;
                         return c->value_mmap;
    case VI_BUF_PENALTY: *size = c->pen_size;   *phys = c->pen_phys;
                         return c->pen_mmap;
    case VI_BUF_TRANS:   *size = c->trans_size; *phys = c->trans_phys;
                         return c->trans_mmap;
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
```

### 5.3 Mock 実装 `driver/uio/vi_device_mock.c`

- 内部に擬似レジスタ配列・malloc したバッファを持ち、`vi_reference_solve` を呼んで
  Sweep を 1 回進める
- `write_reg(AP_CTRL, 0x1)` を受けた時点で参照実装を走らせ、`max_delta` を擬似レジスタに書き込む
- `wait_irq` は即座に 0 を返す
- ホストテスト用、実機非依存

### 5.4 HLS ヘッダ同期

`driver/uio/Makefile` に `sync-hw-header` ターゲットを追加:

```make
HLS_HW_HEADER = ../../fpga/hls/vi_sweep/hls_build/hls/impl/ip/drivers/vi_sweep_v1_0/src/xvi_sweep_hw.h

sync-hw-header:
	install -D $(HLS_HW_HEADER) generated/xvi_sweep_hw.h
```

HLS を再生成したときに手動で `make sync-hw-header` を実行する運用。
ヘッダ差分は `git diff` で追跡できるため、オフセット変更に気付ける。

---

## 6. CLI Tool `vi_cli`

### 6.1 Usage

```
vi_cli --map <path.yaml> --goal <gx,gy[,gt]> [options]

必須:
  --map PATH           ROS map_server YAML + PGM
  --goal GX,GY[,GT]    ゴールセル (GT 省略時は全 theta ゴール)

オプション:
  --action-params FILE   forward/rotation テーブル (省略時 spec §2.3 既定)
  --safety-radius N      障害物膨張半径 (デフォルト 6 cells)
  --threshold N          収束判定 max_delta (デフォルト 0)
  --max-sweeps N         Sweep 上限 (デフォルト 200)
  --out-value PATH       収束後 ValueTable 書き出し (raw uint16)
  --out-action PATH      最適アクションテーブル書き出し (raw uint8)
  --verify               CPU 参照実装と比較して PASS/FAIL
  --mock                 UIO/udmabuf を叩かずモック経由で実行
  -v                     Sweep 毎の max_delta / 経過時間を表示
```

### 6.2 Exit codes

| Code | Meaning |
|---|---|
| 0 | 成功(verify 時は一致) |
| 1 | 引数エラー / マップ読み込み失敗 |
| 2 | デバイス open 失敗 |
| 3 | Sweep 失敗 / 発散 / verify 不一致 |

### 6.3 共有ロジック

| ファイル | 内容 | 備考 |
|---|---|---|
| `host/src/map_pgm.{h,c}` | PGM P5 + 最小 YAML パーサ | libyaml 不使用、キー取り出しのみ |
| `host/src/penalty.{h,c}` | 安全距離膨張 + ゴール設定 | 既存 ROS1 `value_iteration` のロジック移植 |
| `host/src/transitions.{h,c}` | action × theta → (dix,diy,dit) | `fpga/pynq/demo_vi.py::compute_transitions` の C ポート |
| `host/src/vi_reference.{h,c}` | 参照 VI Solver | Phase 1 `fpga/hls/vi_sweep/tb/vi_reference.cpp` の C ポート(`extern "C"` 経由でリンクでも可) |

---

## 7. Error Handling

- **libvi_sweep**: 全公開関数は負の enum エラーコードを返す。内部で `errno` を保持し、
  `vi_strerror(code)` が `"VI_ERR_MMAP: <strerror(errno)>"` 形式で整形
- **リソース解放**: `vi_open` が途中失敗したら確保済みリソースを逆順で解放して NULL を返す。
  `vi_close` は冪等 (NULL 許容、二重 close 安全)
- **IRQ タイムアウト**: `poll()` に 60 秒タイムアウト。超過で `VI_ERR_IRQ` → PL hang 早期検出
- **発散**: `max_sweeps` 到達で `VI_ERR_NOT_CONV`。`vi_run_stats_t` は埋めるためユーザは途中結果を
  読める
- **シグナル**: CLI は `SIGINT` を捕まえてフラグを立て、現在の Sweep 完了後にループを抜ける
- **アサーション**: 引数 NULL・範囲外は `VI_ERR_BAD_ARG`。`assert()` は開発時のみ(`NDEBUG` で無効化)

---

## 8. Testing Strategy

### 8.1 Host Unit Tests (`host/test/`)

| テストファイル | 対象 | モック? |
|---|---|---|
| `test_map_pgm.c` | PGM P5 + YAML パース | no |
| `test_penalty.c` | 膨張・ゴール設定 | no |
| `test_transitions.c` | action × theta → offset | no |
| `test_vi_run_mock.c` | `vi_run_until_converged` の収束ループ | yes |
| `test_action_table.c` | 収束後 argmin | no |
| `test_reference_eq.c` | mock 経由で小マップ Solve → 参照一致 | yes |

- gtest/Catch2 不使用、自前の単純アサーションマクロで依存最小
- `make test-host` でホスト PC 上で全部走らせる
- CI は GitHub Actions 相当で `make test-host` を走らせれば実機なしで退行検知

### 8.2 HW Integration Tests (`host/test/hw/`)

| スクリプト | 内容 |
|---|---|
| `run_smoke.sh` | 40x40x60 合成マップ → `vi_cli --verify` で PASS |
| `run_big.sh` | 700m x 40m 実コース → 収束時間計測、60 秒以内を assert |

- Makefile ターゲット `make test-hw` が SSH 越しに実機で実行
- 実機 IP は環境変数 `VI_TARGET_HOST` で指定

---

## 9. Project Structure

```
value_iteration_fpga/
├── driver/
│   ├── uio/
│   │   ├── libvi_sweep.h
│   │   ├── libvi_sweep.c
│   │   ├── vi_device.h
│   │   ├── vi_device_linux.c
│   │   ├── vi_device_mock.c
│   │   ├── generated/
│   │   │   └── xvi_sweep_hw.h      # HLS から sync
│   │   └── Makefile                # libvi_sweep.{a,so} + sync-hw-header
│   └── dts/
│       ├── vi_sweep.dtsi
│       └── README.md                # Petalinux 組み込み手順
├── host/
│   ├── src/
│   │   ├── vi_cli.c
│   │   ├── map_pgm.{h,c}
│   │   ├── penalty.{h,c}
│   │   ├── transitions.{h,c}
│   │   └── vi_reference.{h,c}
│   ├── test/
│   │   ├── test_map_pgm.c
│   │   ├── test_penalty.c
│   │   ├── test_transitions.c
│   │   ├── test_vi_run_mock.c
│   │   ├── test_action_table.c
│   │   ├── test_reference_eq.c
│   │   └── hw/
│   │       ├── run_smoke.sh
│   │       └── run_big.sh
│   └── Makefile
└── docs/superpowers/specs/
    └── 2026-04-11-vi-fpga-phase3-driver-design.md  # 本ファイル
```

**トップレベル Makefile:** `make driver` / `make host` / `make test-host` / `make test-hw`
をそれぞれのサブディレクトリに委譲。

---

## 10. Risks and Mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| Petalinux で `no-map` + 1.5GB 予約が取れない(CMA 不足) | 起動失敗 | カーネル `CONFIG_CMA_SIZE_MBYTES=1536` で拡張、ダメなら 1 GB に縮小し map を縮小 |
| `u-dma-buf` が sync-mode=1 でもキャッシュ問題を起こす | Value table 破損 | mmap 時に `MAP_SHARED` + memory barrier、最悪 sync-mode=2(write-through)に変更 |
| HLS 再生成でレジスタオフセットが変わる | Silent 破壊 | `sync-hw-header` 後の `git diff` を必須確認、CI で diff 検出時に警告 |
| SPI 番号 89/90 が Ultra96-V2 DT の他デバイスと衝突 | 割り込み動作不能 | Vivado の `pl_ps_irq0` 割り当てを確認、必要に応じて番号調整 |
| udmabuf の phys アドレスが HP0 の SAXIGP2 レンジ外 | DMA 失敗 | DT の `reg` 値を `0x0_2000_0000` 以降に固定(HP0 は下位 32bit 全域アクセス可なので問題なし) |
| HLS IP がインターフェース変更で `ap_ctrl_chain` 対応を要求 | 割り込み動作変更 | Phase 2 追補の BD 再生成時に HLS インターフェース種別を確認(`ap_ctrl_hs` 想定) |

---

## 11. Out of Scope (Future Specs)

- Phase 4: ROS2 ノード統合 (`ros2/` ディレクトリ)
- Petalinux/Yocto レシピ (`meta-user/recipes-apps/vi-sweep/`)
- Warm start(前回 Value table 再利用)
- ランタイムアクションパラメータ変更のホットリロード
- Bitstream PR(Partial Reconfiguration)での IP 差し替え
- 動的マップサイズ変更(現状コンパイル時固定の `MAP_X_MAX` / `MAP_Y_MAX`)

---

## 12. Acceptance Criteria

Phase 3 完了判定:

- [ ] `make driver` で `libvi_sweep.a` / `libvi_sweep.so` がビルドされる
- [ ] `make host` で `vi_cli` がビルドされる
- [ ] `make test-host` で全ホストテストが PASS
- [ ] Petalinux イメージが DT 込みで起動し `/dev/uio0`, `/dev/uio1`,
      `/dev/udmabuf_value`, `/dev/udmabuf_pendata` が生える
- [ ] `vi_cli --map tiny.yaml --goal 20,20 --verify --mock` が PASS
- [ ] 実機で `vi_cli --map tiny.yaml --goal 20,20 --verify` が PASS
- [ ] 実機で `vi_cli --map campus700m.yaml --goal ... --max-sweeps 50` が 60 秒以内に収束
