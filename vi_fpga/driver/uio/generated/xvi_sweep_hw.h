// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
// Tool Version Limit: 2025.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
// control
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read/COR)
//        bit 7  - auto_restart (Read/Write)
//        bit 9  - interrupt (Read)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0 - enable ap_done interrupt (Read/Write)
//        bit 1 - enable ap_ready interrupt (Read/Write)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0 - ap_done (Read/TOW)
//        bit 1 - ap_ready (Read/TOW)
//        others - reserved
// 0x10 : Data signal of value_table
//        bit 31~0 - value_table[31:0] (Read/Write)
// 0x14 : Data signal of value_table
//        bit 31~0 - value_table[63:32] (Read/Write)
// 0x18 : reserved
// 0x1c : Data signal of penalty_table
//        bit 31~0 - penalty_table[31:0] (Read/Write)
// 0x20 : Data signal of penalty_table
//        bit 31~0 - penalty_table[63:32] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of trans_table
//        bit 31~0 - trans_table[31:0] (Read/Write)
// 0x2c : Data signal of trans_table
//        bit 31~0 - trans_table[63:32] (Read/Write)
// 0x30 : reserved
// 0x34 : Data signal of map_x
//        bit 31~0 - map_x[31:0] (Read/Write)
// 0x38 : reserved
// 0x3c : Data signal of map_y
//        bit 31~0 - map_y[31:0] (Read/Write)
// 0x40 : reserved
// 0x44 : Data signal of num_tiles_x
//        bit 31~0 - num_tiles_x[31:0] (Read/Write)
// 0x48 : reserved
// 0x4c : Data signal of num_tiles_y
//        bit 31~0 - num_tiles_y[31:0] (Read/Write)
// 0x50 : reserved
// 0x54 : Data signal of cu_id
//        bit 31~0 - cu_id[31:0] (Read/Write)
// 0x58 : reserved
// 0x5c : Data signal of max_delta
//        bit 15~0 - max_delta[15:0] (Read)
//        others   - reserved
// 0x60 : Control signal of max_delta
//        bit 0  - max_delta_ap_vld (Read/COR)
//        others - reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XVI_SWEEP_CONTROL_ADDR_AP_CTRL            0x00
#define XVI_SWEEP_CONTROL_ADDR_GIE                0x04
#define XVI_SWEEP_CONTROL_ADDR_IER                0x08
#define XVI_SWEEP_CONTROL_ADDR_ISR                0x0c
#define XVI_SWEEP_CONTROL_ADDR_VALUE_TABLE_DATA   0x10
#define XVI_SWEEP_CONTROL_BITS_VALUE_TABLE_DATA   64
#define XVI_SWEEP_CONTROL_ADDR_PENALTY_TABLE_DATA 0x1c
#define XVI_SWEEP_CONTROL_BITS_PENALTY_TABLE_DATA 64
#define XVI_SWEEP_CONTROL_ADDR_TRANS_TABLE_DATA   0x28
#define XVI_SWEEP_CONTROL_BITS_TRANS_TABLE_DATA   64
#define XVI_SWEEP_CONTROL_ADDR_MAP_X_DATA         0x34
#define XVI_SWEEP_CONTROL_BITS_MAP_X_DATA         32
#define XVI_SWEEP_CONTROL_ADDR_MAP_Y_DATA         0x3c
#define XVI_SWEEP_CONTROL_BITS_MAP_Y_DATA         32
#define XVI_SWEEP_CONTROL_ADDR_NUM_TILES_X_DATA   0x44
#define XVI_SWEEP_CONTROL_BITS_NUM_TILES_X_DATA   32
#define XVI_SWEEP_CONTROL_ADDR_NUM_TILES_Y_DATA   0x4c
#define XVI_SWEEP_CONTROL_BITS_NUM_TILES_Y_DATA   32
#define XVI_SWEEP_CONTROL_ADDR_CU_ID_DATA         0x54
#define XVI_SWEEP_CONTROL_BITS_CU_ID_DATA         32
#define XVI_SWEEP_CONTROL_ADDR_MAX_DELTA_DATA     0x5c
#define XVI_SWEEP_CONTROL_BITS_MAX_DELTA_DATA     16
#define XVI_SWEEP_CONTROL_ADDR_MAX_DELTA_CTRL     0x60

