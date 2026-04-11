"""Demo: run Value Iteration on a small test map via FPGA."""

import numpy as np
import math
import time
from vi_overlay import VIOverlay

N_ACTIONS = 6
N_THETA = 60
MAX_VALUE = 0xFFFF
PENALTY_OBSTACLE = 0xFFFF
PENALTY_GOAL = 0xFFFE

ACTION_FW = [0.3, -0.2, 0.0, 0.0, 0.3, 0.3]
ACTION_ROT = [0.0, 0.0, 20.0, -20.0, 20.0, -20.0]


def compute_transitions(xy_res: float) -> np.ndarray:
    t_res = 360.0 / N_THETA
    packed = np.zeros(N_ACTIONS * N_THETA, dtype=np.uint32)

    for a in range(N_ACTIONS):
        for it in range(N_THETA):
            theta = (it * t_res + t_res * 0.5) * math.pi / 180.0
            dx = ACTION_FW[a] * math.cos(theta)
            dy = ACTION_FW[a] * math.sin(theta)
            dix = int(math.floor(dx / xy_res))
            diy = int(math.floor(dy / xy_res))

            new_theta = it * t_res + t_res * 0.5 + ACTION_ROT[a]
            while new_theta < 0: new_theta += 360
            while new_theta >= 360: new_theta -= 360
            new_it = int(math.floor(new_theta / t_res))
            dit = new_it - it
            if dit > N_THETA // 2: dit -= N_THETA
            if dit < -N_THETA // 2: dit += N_THETA

            w = (dix & 0xFF) | ((diy & 0xFF) << 8) | ((dit & 0xFF) << 16)
            packed[a * N_THETA + it] = w

    return packed


def main():
    MAP_X, MAP_Y = 40, 40
    XY_RES = 0.05

    print(f"Map: {MAP_X}x{MAP_Y}, resolution={XY_RES}m")

    # Build penalty table
    penalty = np.zeros((MAP_Y, MAP_X), dtype=np.uint16)
    # Border obstacles
    penalty[0, :] = PENALTY_OBSTACLE
    penalty[-1, :] = PENALTY_OBSTACLE
    penalty[:, 0] = PENALTY_OBSTACLE
    penalty[:, -1] = PENALTY_OBSTACLE
    # Goal at (30, 30)
    penalty[30, 30] = PENALTY_GOAL

    # Value table
    value = np.full((MAP_Y, MAP_X, N_THETA), MAX_VALUE, dtype=np.uint16)
    value[30, 30, :] = 0

    # Transitions
    trans = compute_transitions(XY_RES)

    print("Loading overlay...")
    vi = VIOverlay("vi_bd_wrapper.bit")

    print("Running VI on FPGA...")
    t0 = time.time()
    sweeps = vi.run(value, penalty, trans, MAP_X, MAP_Y, threshold=0)
    elapsed = time.time() - t0

    print(f"Converged in {sweeps} sweeps, {elapsed:.3f}s")
    print(f"Value at (5,5,0): {value[5, 5, 0]}")
    print(f"Value at (20,20,0): {value[20, 20, 0]}")


if __name__ == "__main__":
    main()
