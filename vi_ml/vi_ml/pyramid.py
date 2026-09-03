"""Pyramid rollout on 128² maps with the 64²-trained U-Net.

Coarse: obstacle-dominant 2× pool → 64², goal in coarse cells → coarse value.
Fine: a 64² window around the robot, its border values taken from the
upsampled coarse field (×2: a coarse cell is two fine cells of travel), goal
mask only if the goal is inside → fine value → greedy steps → re-window.

    python -m vi_ml.pyramid out/maps128.npz out/unet.pt
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch

from . import maps, rollout, vi
from .eval import load_model
from .model import encode, predict
from .train import RING, WIN

STEPS_PER_WINDOW = 8
COARSE_SCALE_S = 2.0  # ponytail: travel scales with cell size, the penalty part does not


def coarse_value(model, free, goal):
    H, W = free.shape
    cf = free.reshape(H // 2, 2, W // 2, 2).all(axis=(1, 3))
    cg = (goal[0] // 2, goal[1] // 2)
    cf[cg] = True  # the goal cell must exist at the coarse level
    v = predict(model, encode(cf, cg))
    up = np.repeat(np.repeat(v, 2, 0), 2, 1) * COARSE_SCALE_S
    up[~free] = np.nan
    return up


def window_value(model, free, goal, coarse, pos):
    H, W = free.shape
    y0 = int(np.clip(pos[0] - WIN // 2, 0, H - WIN))
    x0 = int(np.clip(pos[1] - WIN // 2, 0, W - WIN))
    f = free[y0:y0 + WIN, x0:x0 + WIN]
    g = (goal[0] - y0, goal[1] - x0)
    g = g if 0 <= g[0] < WIN and 0 <= g[1] < WIN else None
    b = np.full((WIN, WIN), np.nan, np.float32)
    c = coarse[y0:y0 + WIN, x0:x0 + WIN]
    b[:RING], b[-RING:], b[:, :RING], b[:, -RING:] = c[:RING], c[-RING:], c[:, :RING], c[:, -RING:]
    if g is not None:  # goal in view: no boundary needed, trust the whole-map mode
        b[:] = np.nan
    return predict(model, encode(f, g, b)), (y0, x0)


def drive(model, free, goal, start, max_windows: int = 40):
    """Returns (reached, path)."""
    coarse = coarse_value(model, free, goal)
    goal_mask = np.zeros_like(free)
    goal_mask[goal] = True
    pos, path = start, [start]
    for _ in range(max_windows):
        v, (y0, x0) = window_value(model, free, goal, coarse, pos)
        gm = goal_mask[y0:y0 + WIN, x0:x0 + WIN]
        reached, p = rollout.greedy(v, (pos[0] - y0, pos[1] - x0), gm, STEPS_PER_WINDOW)
        if len(p) == 1:
            return False, path  # local minimum: stuck
        path += [(y + y0, x + x0) for y, x in p[1:]]
        pos = path[-1]
        if reached:
            return True, path
    return False, path


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("data", type=Path)
    ap.add_argument("weights", type=Path)
    ap.add_argument("--n", type=int, default=100, help="maps (taken from the end, unseen by window training)")
    ap.add_argument("--starts", type=int, default=4)
    a = ap.parse_args(argv)
    d = np.load(a.data)
    model = load_model(a.weights)
    torch.set_num_threads(1)
    rng = np.random.default_rng(1)
    rows = []
    for i in range(len(d["free"]) - a.n, len(d["free"])):
        free, goal, true = d["free"][i], tuple(int(v) for v in d["goal"][i]), d["value"][i]
        m = free & np.isfinite(true)
        gmask = true == 0
        _, vi_ms = vi.solve(free, goal)  # re-timed alone
        for _ in range(a.starts):
            s = maps.pick_free(rng, m)
            r_t, p_t = rollout.greedy(true, s, gmask)
            t0 = time.perf_counter()
            r_p, p_p = drive(model, free, goal, s)
            ms = (time.perf_counter() - t0) * 1e3
            # goal *cell* vs goal *region*: count reaching any true-zero cell as success
            r_p = r_p or bool(gmask[p_p[-1]])
            rows.append((str(d["kind"][i]), r_p, r_t, rollout.length(p_p) / max(1e-9, rollout.length(p_t)) if r_p and r_t else np.nan, ms, vi_ms))
    print("| kind | runs | success(pyramid) | success(true greedy) | len ratio | pyramid ms | vi ms |")
    print("|---|---|---|---|---|---|---|")
    for k in sorted({r[0] for r in rows}) + ["all"]:
        rs = [r for r in rows if k == "all" or r[0] == k]
        c = np.array([r[1:] for r in rs], float)
        print(f"| {k} | {len(rs)} | {c[:, 0].mean():.3f} | {c[:, 1].mean():.3f} | {np.nanmean(c[:, 2]):.2f} | "
              f"{np.median(c[:, 3]):.0f} | {np.median(c[:, 4]):.0f} |")


if __name__ == "__main__":
    main()
