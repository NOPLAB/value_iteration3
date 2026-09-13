"""Greedy descent on a 2-D value field: the metric that matters (does a path
come out?) and what the pyramid driver uses to move the robot.

The θ-min field has ~2 % local minima under 8-neighbour steps (the robot's real
step is 3 cells, and the penalty is charged on *entering* a cell), so the
descent looks at every line-of-sight cell within `radius` cells instead."""
from __future__ import annotations

import numpy as np

RADIUS = 3  # 0.3 m forward step at 0.1 m/cell


def _clear(value: np.ndarray, y, x, ny, nx) -> bool:
    n = 2 * max(abs(ny - y), abs(nx - x))
    for t in np.linspace(0, 1, n + 1)[1:]:
        py, px = y + int(round((ny - y) * t)), x + int(round((nx - x) * t))  # round the offset: parity-independent
        if not np.isfinite(value[py, px]):
            return False
    return True


def greedy(value: np.ndarray, start: tuple[int, int], goal_mask: np.ndarray,
           max_steps: int | None = None, radius: int = RADIUS):
    """Follow the steepest descent of `value` (NaN = blocked) from `start` until a
    `goal_mask` cell. Returns (reached, path). Stops at a local minimum."""
    H, W = value.shape
    max_steps = max_steps or 2 * (H + W)
    y, x = start
    path = [(y, x)]
    for _ in range(max_steps):
        if goal_mask[y, x]:
            return True, path
        best, bv = None, value[y, x]
        for ny in range(max(0, y - radius), min(H, y + radius + 1)):
            for nx in range(max(0, x - radius), min(W, x + radius + 1)):
                v = value[ny, nx]
                if v < bv and _clear(value, y, x, ny, nx):
                    best, bv = (ny, nx), v
        if best is None:
            return False, path
        y, x = best
        path.append((y, x))
    return bool(goal_mask[y, x]), path


def length(path) -> float:
    p = np.asarray(path, float)
    return float(np.hypot(*(p[1:] - p[:-1]).T).sum()) if len(p) > 1 else 0.0


def evaluate(pred: np.ndarray, true: np.ndarray, starts: list[tuple[int, int]]):
    """Success rate of greedy descent on `pred` (goal = true==0 cells) and the
    mean path-length ratio vs descent on `true`, over `starts`."""
    goal = true == 0
    ok, ratios = 0, []
    for s in starts:
        r_t, p_t = greedy(true, s, goal)
        r_p, p_p = greedy(pred, s, goal)
        if r_p:
            ok += 1
            if r_t and length(p_t) > 0:
                ratios.append(length(p_p) / length(p_t))
    return ok / max(1, len(starts)), (float(np.mean(ratios)) if ratios else np.nan)
