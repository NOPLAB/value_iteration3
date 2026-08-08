#ifndef VI_REGS_H
#define VI_REGS_H

/* vi_sweep AXI-Lite register offsets (s_axi_control layout).
   Hand-maintained from the HLS-generated map; shared by libvi_sweep.c
   and vi_device_mock.c so the two cannot drift. */

#define VI_OFF_AP_CTRL       0x00
#define VI_OFF_GIE           0x04
#define VI_OFF_IER           0x08
#define VI_OFF_VALUE_TABLE   0x10  /* 64-bit, 0x10 lo / 0x14 hi */
#define VI_OFF_PENALTY_TABLE 0x1C
#define VI_OFF_TRANS_TABLE   0x28
#define VI_OFF_MAP_X         0x34
#define VI_OFF_MAP_Y         0x3C
#define VI_OFF_NUM_TILES_X   0x44
#define VI_OFF_NUM_TILES_Y   0x4C
#define VI_OFF_CU_ID         0x54
#define VI_OFF_MAX_DELTA     0x5C

#endif /* VI_REGS_H */
