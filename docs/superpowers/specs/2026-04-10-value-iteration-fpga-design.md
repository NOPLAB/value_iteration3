# Value Iteration FPGA Accelerator Design Spec

**Date:** 2026-04-10
**Target Board:** Ultra96-V2 (Zynq UltraScale+ ZU3EG)
**Goal:** 700m campus course map (14,000 x 800 cells, theta=60) のValue Iterationを60秒以内で完了

---

## 1. Overview

ROS1パッケージ `value_iteration` と同等のValue Iterationプランナーを、Vitis HLSを用いてC/C++からVerilogを生成し、Ultra96-V2上で動作させる。最終的にはLinuxデバイスドライバを介してROS2パッケージとして統合する。

### Phase構成

| Phase | 内容 | 成果物 |
|-------|------|--------|
| 1 | HLSカーネル + テストベンチ | Cシミュレーション・合成通過 |
| 2 | Vivado統合 + ビットストリーム | Ultra96-V2上でPYNQ経由動作確認 |
| 3 | Linuxデバイスドライバ | UIOベースでユーザ空間から制御 |
| 4 | ROS2パッケージ | ドライバを使ったROS2ノード |

本specはPhase 1-2を対象とし、Phase 3-4は別specとする。

---

## 2. Algorithm

### 2.1 Value Iteration

3D状態空間 (x, y, theta) 上でBellman方程式を反復的に解く:

```
V(s) = min_a [ V(s') + penalty(s') ]
```

- `s = (ix, iy, it)`: 離散化された状態
- `a`: アクション (6種, 固定)
- `s'`: アクション `a` による遷移先 (決定的, 1アクションにつき1遷移先)
- `penalty(s')`: 障害物・安全距離に基づくコスト

収束条件: `max_delta < threshold` (全状態での最大変化量が閾値未満)

### 2.2 Deterministic Transition Model

確率的遷移を排除し、各 (action, theta) の組に対して1つの遷移先のみを持つ:

```
delta[action][theta] = (dix, diy, dit)
```

- 6 actions x 60 theta = 360 entries x 3 bytes = 1,080 bytes
- レジスタに完全パーティション可能 (BRAM不要)
- ARM側で事前計算し、HLSカーネル起動前にDDRの小バッファ経由でBRAMへロード (アクションパラメータ変更に対応)

### 2.3 Actions (固定, コンパイル時決定)

既存コードのlaunchファイルから典型的な6アクション:

| # | Name | forward (m) | rotation (deg) |
|---|------|-------------|----------------|
| 0 | forward | 0.3 | 0 |
| 1 | backward | -0.2 | 0 |
| 2 | left | 0.0 | 20 |
| 3 | right | 0.0 | -20 |
| 4 | forward-left | 0.3 | 20 |
| 5 | forward-right | 0.3 | -20 |

### 2.4 Data Types

| Data | Type | Bits | Range / Notes |
|------|------|------|---------------|
| Value (cost) | `ap_uint<16>` | 16 | 0-65535, 十分な値域か要テストベンチ検証 |
| Penalty | `ap_uint<16>` | 16 | 同上 |
| Optimal Action | `ap_uint<3>` | 3 | 0-5 (6 actions) |
| Transition offset | `ap_int<8>` x 3 | 24 | dix, diy, dit |
| max_delta | `ap_uint<16>` | 16 | Sweep内最大変化量 |

16bitに収まらない場合のフォールバック: `ap_uint<24>` (1.34GB → 2.0GB, DDR上限ぎりぎり)

---

## 3. Architecture

### 3.1 System Overview

```
ARM (PS)                              FPGA (PL)
===========                           ==========
遷移テーブル事前計算                    ┌──────┐ ┌──────┐
マップ読み込み                         │ CU 0 │ │ CU 1 │
ペナルティ計算                         │      │ │      │
Value Table初期化     AXI-Lite ctrl    │ LOAD │ │ LOAD │
                    ─────────────→    │  ↓   │ │  ↓   │
Sweep開始指示                          │ COMP │ │ COMP │
                    ←─────────────    │  ↓   │ │  ↓   │
収束判定 (max_delta)                   │STORE │ │STORE │
                                      └──┬───┘ └──┬───┘
                                         │        │
                                    AXI Master  AXI Master
                                         │        │
                                      ┌──┴────────┴──┐
                                      │   DDR4 (2GB)  │
                                      └───────────────┘
```

### 3.2 Compute Unit Internal: 3-Stage DATAFLOW Pipeline

各CU内部はHLS DATAFLOWで3段パイプライン:

**Stage 1 - LOAD:**
- DDRからValue Tableのタイル (32x32x60) をBRAMへバースト読み出し
- ハロー領域 (周囲数セル) も読み出し
- ペナルティテーブルのタイル分も読み出し

**Stage 2 - COMPUTE:**
- Bellman更新を実行
- 6アクション完全UNROLL, II=1 で毎サイクル1状態を処理
- タイル内はGauss-Seidel的に更新済み値を即時利用 (収束加速)
- タイル内max_deltaを計算

**Stage 3 - STORE:**
- 更新済みValueをDDRへバースト書き戻し (Action TableはSweepでは書かない)
- max_deltaをFIFO経由で累積

### 3.3 Multi-CU: Checkerboard Tile Assignment

マップを32x32タイルに分割し、チェッカーボードパターンで2CUに割り当て:

```
┌────┬────┬────┬────┐
│CU0 │CU1 │CU0 │CU1 │
├────┼────┼────┼────┤
│CU1 │CU0 │CU1 │CU0 │
├────┼────┼────┼────┤
│CU0 │CU1 │CU0 │CU1 │
└────┴────┴────┴────┘
```

利点:
- 同時処理タイル同士が隣接しない → ハロー領域の書き込み競合なし
- CU間同期が不要

### 3.4 DDR Memory Layout

全コースマップをDDR上に配置 (区間分割不要):

```
Address         Content                    Size (700m x 40m map)
0x1000_0000     Value Table                14,000 x 800 x 60 x 2B = 1.34GB
                [map_y][map_x][theta]
                ap_uint<16> per entry

0x6400_0000     Penalty Table              14,000 x 800 x 2B = 22.4MB
                [map_y][map_x]
                ap_uint<16> per entry
                (theta方向は共通)

0x6580_0000     Transition Table           6 x 60 x 3B = 1,080B
                [action][theta][3]
                初回Sweep前にBRAMへ一括ロード

Total: ~1.36GB (DDR 2GB に対して十分な余裕)
```

Action Table はDDR上に持たない。収束後にARM側でValue Tableを読み、各状態の最適アクションを1パスで計算する (各状態につき6アクション分のnext stateを比較するだけ)。これによりDDR消費を1.36GBに抑える。

### 3.5 Halo Region

ロボットの1アクション移動距離:
- forward 0.3m / 0.05m resolution = 6 cells
- ハロー幅: 6 cells (各方向)

タイルサイズ32x32の場合:
- ハロー含み読み出し: (32+12) x (32+12) x 60 = 116,160 states x 2B = 227KB
- タイル本体: 32 x 32 x 60 x 2B = 120KB
- 1CUあたりBRAM: ダブルバッファ考慮で ~700KB

### 3.6 Transition Table

決定的遷移テーブルはDDRからBRAMへ初回ロード後、完全パーティション:

```cpp
// ARM側で事前計算してDDRに配置
// HLSカーネル起動時にDDR→レジスタへ一括ロード
int8_t delta[6][60][3]; // dix, diy, dit
#pragma HLS ARRAY_PARTITION variable=delta complete dim=0
load_transitions(trans_table, delta); // DDRからバーストリード
```

360 entries x 3 bytes = 1,080 bytes → LUTに収まる。
ランタイムロードにより、アクションパラメータ変更時にビットストリーム再生成不要。

---

## 4. HLS Kernel Interface

### 4.1 Top-Level Function

```cpp
void vi_sweep(
    // DDR Data Ports
    ap_uint<16> *value_table,              // Value Table R/W (gmem0)
    const ap_uint<16> *penalty_table,      // Penalty Table R  (gmem1, 読み取り専用で分離)
    const ap_uint<32> *trans_table,        // Transition Table R (gmem1, 初回BRAMロード用)
    
    // Control Registers (AXI-Lite)
    int map_x,                             // マップ幅 (cells)
    int map_y,                             // マップ高さ (cells)
    int num_tiles_x,                       // X方向タイル数
    int num_tiles_y,                       // Y方向タイル数
    int cu_id,                             // CU ID (0 or 1, チェッカーボード用)
    ap_uint<16> *max_delta                 // 出力: Sweep内最大変化量
);
```

### 4.2 HLS Pragmas

```cpp
// DDR ports — value_table (R/W) と penalty/trans (R) を別バンドルに分離
// value_table は Sweep中に読み書き両方行うため専用ポート
// penalty/trans は読み取り専用のため同一バンドルで可
#pragma HLS INTERFACE m_axi port=value_table   bundle=gmem0 \
    offset=slave depth=672000000
#pragma HLS INTERFACE m_axi port=penalty_table bundle=gmem1 \
    offset=slave depth=11200000
#pragma HLS INTERFACE m_axi port=trans_table   bundle=gmem1 \
    offset=slave depth=360

// Control
#pragma HLS INTERFACE s_axilite port=map_x
#pragma HLS INTERFACE s_axilite port=map_y
#pragma HLS INTERFACE s_axilite port=num_tiles_x
#pragma HLS INTERFACE s_axilite port=num_tiles_y
#pragma HLS INTERFACE s_axilite port=cu_id
#pragma HLS INTERFACE s_axilite port=max_delta
#pragma HLS INTERFACE s_axilite port=return

// DATAFLOW
#pragma HLS DATAFLOW
```

### 4.3 DATAFLOW Internal

```cpp
void vi_sweep(...) {
    #pragma HLS DATAFLOW

    // 遷移テーブル: DDRから初回ロードし、完全パーティションでレジスタ化
    int8_t delta[6][60][3];
    #pragma HLS ARRAY_PARTITION variable=delta complete dim=0
    load_transitions(trans_table, delta);

    // Stage間 stream
    // TileData: タイル本体 (32x32x60 value) + ハロー領域 + penalty
    // TileResult: 更新済みタイル (32x32x60 value) + tile max_delta
    hls::stream<TileData>   s_load_to_compute("load2comp");
    hls::stream<TileResult> s_compute_to_store("comp2store");
    #pragma HLS STREAM variable=s_load_to_compute depth=2
    #pragma HLS STREAM variable=s_compute_to_store depth=2

    load_tiles(value_table, penalty_table,
               map_x, map_y,
               num_tiles_x, num_tiles_y, cu_id,
               s_load_to_compute);

    compute_bellman(delta,
                    s_load_to_compute,
                    s_compute_to_store);

    store_tiles(value_table,
                map_x, map_y,
                num_tiles_x, num_tiles_y, cu_id,
                s_compute_to_store,
                max_delta);
}
```

---

## 5. Vivado Block Design

### 5.1 Components

- Zynq UltraScale+ PS (ARM Cortex-A53)
- vi_sweep HLS IP x 2 (CU0, CU1)
- AXI SmartConnect (Control) — GP0 → CU0 ctrl, CU1 ctrl
- AXI SmartConnect (Data) — CU0 gmem0/gmem1, CU1 gmem0/gmem1 → HP0 (DDR)
- Processor System Reset

### 5.2 Clock

`pl_clk0` を使用。目標 150MHz (合成結果でタイミング未達なら100MHzにフォールバック)。

### 5.3 Address Map

| Master | Slave | Address Range |
|--------|-------|--------------|
| CU0 gmem0 (Value R/W) | PS HP0 DDR | 0x0000_0000 - 0x7FFF_FFFF |
| CU0 gmem1 (Penalty/Trans R) | PS HP0 DDR | 0x0000_0000 - 0x7FFF_FFFF |
| CU1 gmem0 (Value R/W) | PS HP0 DDR | 0x0000_0000 - 0x7FFF_FFFF |
| CU1 gmem1 (Penalty/Trans R) | PS HP0 DDR | 0x0000_0000 - 0x7FFF_FFFF |
| PS GP0 | CU0 s_axi_control | 0xA000_0000 |
| PS GP0 | CU1 s_axi_control | 0xA001_0000 |

---

## 6. ARM Host Software

### 6.1 Initialization

```
1. Load occupancy map from file
2. Compute penalty table from map (safety radius)
3. Compute deterministic transition table for each (action, theta)
4. Initialize value table: goal states = 0, others = MAX_VALUE
5. Write penalty table to DDR
6. Write value table to DDR
```

### 6.2 Sweep Loop

```
do {
    // Set params for CU0 (checkerboard even tiles)
    cu0.map_x = map_x;
    cu0.map_y = map_y;
    cu0.cu_id = 0;
    cu0.start();

    // Set params for CU1 (checkerboard odd tiles)
    cu1.map_x = map_x;
    cu1.map_y = map_y;
    cu1.cu_id = 1;
    cu1.start();

    // Wait for both CUs
    while (!cu0.done() || !cu1.done());

    delta = max(cu0.max_delta, cu1.max_delta);
    sweep_count++;
} while (delta > threshold);
```

### 6.3 Post-Convergence

```
// Optimal action computation on ARM
// (value table is in DDR, ARM reads and computes argmin for each state)
for each state (ix, iy, it):
    best_action = argmin_a value[next_state(ix, iy, it, a)]
```

---

## 7. Testbench Strategy

### 7.1 HLS C Simulation Testbench

`vi_sweep_tb.cpp`:
1. 小マップ (16x16x60 = 15,360 states) を生成
2. ゴール状態を中央に設定
3. 障害物をいくつか配置
4. CPU参照実装で収束まで計算 (golden reference)
5. HLSカーネルでSweepを反復実行
6. 収束後のValue Tableを参照解と比較
7. 許容誤差内 (16bit量子化誤差を考慮) で一致を確認

### 7.2 Validation Items

- [ ] 16bit精度で32bit/64bitと同等の経路が得られるか
- [ ] ハロー領域の境界処理が正しいか
- [ ] チェッカーボードパターンで正しいタイルが処理されるか
- [ ] ゴール状態の値が0に維持されるか
- [ ] 障害物セルが更新されないか
- [ ] 収束判定 (max_delta) が正しいか

---

## 8. Project Structure

```
value_iteration_fpga/
├── fpga/
│   ├── hls/
│   │   └── vi_sweep/
│   │       ├── src/
│   │       │   ├── vi_sweep_top.cpp       # Top-level DATAFLOW
│   │       │   ├── vi_sweep_top.h
│   │       │   ├── vi_types.h             # ap_uint types, constants
│   │       │   ├── load_tiles.cpp         # Stage 1: DDR → BRAM
│   │       │   ├── load_tiles.h
│   │       │   ├── compute_bellman.cpp    # Stage 2: Bellman update
│   │       │   ├── compute_bellman.h
│   │       │   ├── store_tiles.cpp        # Stage 3: BRAM → DDR
│   │       │   └── store_tiles.h
│   │       ├── tb/
│   │       │   ├── vi_sweep_tb.cpp        # Testbench
│   │       │   └── vi_reference.cpp       # CPU reference implementation
│   │       ├── hls_config.cfg
│   │       └── vitis-comp.json
│   ├── vivado/
│   │   └── ultra96v2/
│   │       ├── create_project.tcl
│   │       ├── create_bd.tcl              # Block design (2 CU)
│   │       ├── constraints.xdc
│   │       └── ip_repo/
│   ├── pynq/
│   │   ├── vi_overlay.py                  # PYNQ validation
│   │   └── demo_vi.py
│   └── scripts/
│       ├── Makefile
│       ├── export_hls_ip.tcl
│       └── build_vivado.tcl
├── driver/                                 # Phase 3
│   ├── uio/
│   │   ├── vi_sweep_uio.c
│   │   └── vi_sweep_uio.h
│   └── dts/
│       └── vi_sweep.dtsi
├── host/
│   ├── src/
│   │   ├── vi_host.cpp                    # ARM control logic
│   │   ├── vi_host.h
│   │   ├── map_loader.cpp
│   │   └── map_loader.h
│   ├── test/
│   │   └── test_vi_host.cpp
│   └── CMakeLists.txt
├── ros2/                                   # Phase 4
│   ├── src/
│   │   ├── vi_fpga_node.cpp
│   │   └── vi_fpga_node.h
│   ├── launch/
│   │   └── vi_fpga.launch.py
│   ├── config/
│   │   └── vi_fpga_params.yaml
│   ├── package.xml
│   └── CMakeLists.txt
└── docs/
    └── superpowers/
        └── specs/
            └── 2026-04-10-value-iteration-fpga-design.md
```

---

## 9. Performance Estimate

### 9.1 Target Map

- Course: 700m x 40m
- Resolution: 0.05m
- Cells: 14,000 x 800 x 60 = 672,000,000 states
- Value Table: 672M x 2B = 1.34GB

### 9.2 Per-Sweep

| Parameter | Value |
|-----------|-------|
| States per tile | 32 x 32 x 60 = 61,440 |
| Cycles per state | 2 (6 actions UNROLL, II=1) |
| Tile count | ceil(14000/32) x ceil(800/32) = 438 x 25 = 10,950 |
| Tiles per CU | ~5,475 |
| Time per tile (150MHz) | 61,440 x 2 / 150M = 0.82ms |
| DATAFLOW overhead | Memory latency hidden |
| **1 Sweep (2 CU)** | **5,475 x 0.82ms = 4.49s** |

### 9.3 Convergence

| Sweep count | Total time | Notes |
|-------------|-----------|-------|
| 10 | 45s | 楽観的 (Gauss-Seidel + 良好な初期値) |
| 13 | 58s | 目標60s圏内 |
| 20 | 90s | やや超過 |
| 30 | 135s | 超過、追加最適化が必要 |

### 9.4 Convergence Acceleration Strategies

1. **Gauss-Seidel within tile**: タイル内で更新済み値を即座に利用。標準Jacobiと比べSweep回数を約半減
2. **Sweep direction alternation**: Sweep方向を毎回変更 (forward/reverse) し、値の双方向伝播を加速
3. **Warm start**: ゴール変更時に前回のValue Tableを初期値として再利用
4. **Priority sweep**: ゴール近傍タイルを先に処理し、値の早期伝播を促進

### 9.5 Resource Estimate (per CU)

| Resource | Estimated | Available (ZU3EG) | Utilization |
|----------|-----------|-------------------|-------------|
| LUT | ~15,000 | 70,560 | 21% |
| BRAM (36Kb) | ~80 | 216 | 37% |
| DSP48E2 | ~20 | 360 | 6% |

2 CU total: LUT 42%, BRAM 74%, DSP 12% — 実現可能。

---

## 10. Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|-----------|
| 16bit精度不足 | 経路品質劣化 | テストベンチで検証。24bitへフォールバック (DDR 2.0GB) |
| DDR容量不足 (24bit化時) | 2.0GBでぎりぎり | 16bit維持で1.36GB、Action計算はARM側post-process |
| タイミング未達 (150MHz) | Sweep時間増 | 100MHzフォールバック (目標120sに緩和) |
| BRAM不足 (2CU) | 1CUに制限 | 1CU構成でも2x程度の速度低下 |
| 収束Sweep数が想定超過 | 60s超過 | 収束加速策を段階的に導入 |
| ハロー領域DDR帯域 | メモリ律速 | タイルサイズ拡大でハロー比率低減 |

---

## 11. Out of Scope (Future Specs)

- Local Value Iteration (センサーベース動的更新)
- Phase 3: Linux device driver (UIO)
- Phase 4: ROS2 package integration
- Multiple map resolution support
- Runtime action configuration
