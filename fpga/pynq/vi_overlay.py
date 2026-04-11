"""Value Iteration FPGA overlay for PYNQ on Ultra96-V2.

Usage:
    Copy vi_bd_wrapper.bit, vi_bd_wrapper.hwh, and this file to Ultra96-V2.

    from vi_overlay import VIOverlay
    vi = VIOverlay("vi_bd_wrapper.bit")
    sweeps = vi.run(value_table, penalty_table, trans_table, map_x, map_y)
"""

import struct
import numpy as np
from pynq import Overlay, allocate

# AXI-Lite register offsets (from HLS-generated driver header)
# These will be confirmed after HLS synthesis from the *_hw.h file.
AP_CTRL          = 0x00
ADDR_VALUE_TABLE = 0x10  # 64-bit address
ADDR_PENALTY     = 0x1C  # 64-bit address
ADDR_TRANS       = 0x28  # 64-bit address
ADDR_MAP_X       = 0x34
ADDR_MAP_Y       = 0x3C
ADDR_NUM_TILES_X = 0x44
ADDR_NUM_TILES_Y = 0x4C
ADDR_CU_ID       = 0x54
ADDR_MAX_DELTA   = 0x5C

TILE_W = 32
TILE_H = 32
N_THETA = 60


def _write_addr64(ip, offset, addr):
    ip.write(offset, addr & 0xFFFFFFFF)
    ip.write(offset + 4, (addr >> 32) & 0xFFFFFFFF)


class VIOverlay:
    def __init__(self, bitstream_path: str):
        self.ol = Overlay(bitstream_path)
        self.cu0 = self.ol.vi_sweep_cu0
        self.cu1 = self.ol.vi_sweep_cu1

    def run(
        self,
        value_np: np.ndarray,
        penalty_np: np.ndarray,
        trans_np: np.ndarray,
        map_x: int,
        map_y: int,
        threshold: int = 0,
        max_sweeps: int = 200,
    ) -> int:
        """Run Value Iteration on FPGA until convergence.

        Args:
            value_np: shape (map_y, map_x, N_THETA), uint16. Modified in-place.
            penalty_np: shape (map_y, map_x), uint16.
            trans_np: shape (360,), uint32. Packed transitions.
            map_x, map_y: map dimensions.
            threshold: convergence threshold for max_delta.
            max_sweeps: maximum sweep iterations.

        Returns:
            Number of sweeps executed.
        """
        num_tiles_x = (map_x + TILE_W - 1) // TILE_W
        num_tiles_y = (map_y + TILE_H - 1) // TILE_H

        # Allocate contiguous DMA buffers
        val_buf = allocate(shape=value_np.shape, dtype=np.uint16)
        pen_buf = allocate(shape=penalty_np.shape, dtype=np.uint16)
        trans_buf = allocate(shape=trans_np.shape, dtype=np.uint32)

        np.copyto(val_buf, value_np)
        np.copyto(pen_buf, penalty_np)
        np.copyto(trans_buf, trans_np)
        val_buf.sync_to_device()
        pen_buf.sync_to_device()
        trans_buf.sync_to_device()

        for cu in [self.cu0, self.cu1]:
            _write_addr64(cu, ADDR_VALUE_TABLE, val_buf.device_address)
            _write_addr64(cu, ADDR_PENALTY, pen_buf.device_address)
            _write_addr64(cu, ADDR_TRANS, trans_buf.device_address)
            cu.write(ADDR_MAP_X, map_x)
            cu.write(ADDR_MAP_Y, map_y)
            cu.write(ADDR_NUM_TILES_X, num_tiles_x)
            cu.write(ADDR_NUM_TILES_Y, num_tiles_y)

        sweep = 0
        for sweep in range(max_sweeps):
            # Start both CUs (checkerboard)
            self.cu0.write(ADDR_CU_ID, 0)
            self.cu1.write(ADDR_CU_ID, 1)
            self.cu0.write(AP_CTRL, 0x01)
            self.cu1.write(AP_CTRL, 0x01)

            # Wait for both to finish
            while not (self.cu0.read(AP_CTRL) & 0x02):
                pass
            while not (self.cu1.read(AP_CTRL) & 0x02):
                pass

            d0 = self.cu0.read(ADDR_MAX_DELTA)
            d1 = self.cu1.read(ADDR_MAX_DELTA)
            max_delta = max(d0, d1)

            if max_delta <= threshold:
                break

        # Copy results back
        val_buf.sync_from_device()
        np.copyto(value_np, val_buf)

        val_buf.freebuffer()
        pen_buf.freebuffer()
        trans_buf.freebuffer()

        return sweep + 1
