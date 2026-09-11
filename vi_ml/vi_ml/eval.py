"""Compare a value-field predictor against the exact solver on held-out 64² maps:
value error, greedy-path success / length ratio, and wall-clock (bench_map
solve vs prediction). `run` is shared by the U-Net regressor and the diffusion
model.

    python -m vi_ml.eval out/train64.npz out/unet.pt
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import torch

from . import maps, rollout, vi
from .model import UNet, encode, predict


def load_model(path: Path) -> UNet:
    m = UNet()
    m.load_state_dict(torch.load(path, map_location="cpu"))
    return m.eval()


def run(d, predict_fn, n: int, starts: int, label: str = "cnn"):
    """`predict_fn(x: (4,H,W) float32) -> value seconds (H,W), NaN on obstacles`."""
    torch.set_num_threads(1)  # batch=1 latency, like the planner would see it
    rng = np.random.default_rng(0)
    rows = []
    for i in range(len(d["free"]) - n, len(d["free"])):
        free, goal, true = d["free"][i], tuple(d["goal"][i]), d["value"][i]
        x = encode(free, goal)
        t0 = time.perf_counter()
        pred = predict_fn(x)
        ms = (time.perf_counter() - t0) * 1e3
        m = free & np.isfinite(true)
        rel = np.abs(pred[m] - true[m]) / (true[m] + 1.0)
        ss = [maps.pick_free(rng, m) for _ in range(starts)]
        succ, ratio = rollout.evaluate(pred, true, ss)
        succ_true, _ = rollout.evaluate(true, true, ss)
        _, vi_ms = vi.solve(free, goal)  # re-timed alone: dataset times were taken 6-way parallel
        rows.append((str(d["kind"][i]), float(rel.mean()), succ, succ_true, ratio, ms, vi_ms))
    kinds = sorted({r[0] for r in rows})
    print(f"| kind | n | rel.err | success(pred) | success(true) | len ratio | {label} ms | vi ms |")
    print("|---|---|---|---|---|---|---|---|")
    for k in kinds + ["all"]:
        rs = [r for r in rows if k == "all" or r[0] == k]
        c = np.array([r[1:] for r in rs], float)
        print(f"| {k} | {len(rs)} | {c[:, 0].mean():.3f} | {c[:, 1].mean():.3f} | {c[:, 2].mean():.3f} | "
              f"{np.nanmean(c[:, 3]):.2f} | {np.median(c[:, 4]):.1f} | {np.median(c[:, 5]):.1f} |")
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("data", type=Path)
    ap.add_argument("weights", type=Path)
    ap.add_argument("--n", type=int, default=400, help="held-out samples (taken from the end)")
    ap.add_argument("--starts", type=int, default=8)
    a = ap.parse_args(argv)
    model = load_model(a.weights)
    run(np.load(a.data), lambda x: predict(model, x), a.n, a.starts)


if __name__ == "__main__":
    main()
