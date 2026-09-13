"""Does a CNN field make the exact solver converge faster? Cold solve vs the
same solver warm-started (`bench_map --init-value`) from the true θ-min field
and from the U-Net field, on held-out 64² maps.

    python -m vi_ml.warmstart out/train64.npz out/unet.pt --n 50
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch

from . import vi
from .eval import load_model
from .model import encode, predict


MAX_ITERS = 2000  # cold converges in ~60-230 rounds; a warm start that needs more is a hang, not a win


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("data", type=Path)
    ap.add_argument("weights", type=Path)
    ap.add_argument("--n", type=int, default=50)
    a = ap.parse_args(argv)
    d = np.load(a.data)
    model = load_model(a.weights)
    torch.set_num_threads(1)
    rows = []
    for i in range(len(d["free"]) - a.n, len(d["free"])):
        free, goal, true = d["free"][i], tuple(int(v) for v in d["goal"][i]), d["value"][i]
        pred = predict(model, encode(free, goal))
        r = [str(d["kind"][i])]
        for init in (None, true, pred):
            st = {}
            v, ms = vi.solve(free, goal, init=init, stats=st, max_iters=MAX_ITERS)
            r += [st["iters"], st["updates"], ms, float(np.nanmax(np.abs(v - true))), st["converged"]]
        rows.append(r)
    c = np.array([r[1:] for r in rows], float)
    print(f"| init | iters (median) | updates M (median) | solve ms (median) | max |Δ| vs cold (s) | not converged in {MAX_ITERS} |")
    print("|---|---|---|---|---|---|")
    for j, name in enumerate(("cold (MAX_COST)", "warm: true θ-min field", "warm: U-Net field")):
        k = 5 * j
        print(f"| {name} | {np.median(c[:, k]):.0f} | {np.median(c[:, k + 1]) / 1e6:.2f} | "
              f"{np.median(c[:, k + 2]):.1f} | {c[:, k + 3].max():.4f} | {int((c[:, k + 4] == 0).sum())}/{len(c)} |")


if __name__ == "__main__":
    main()
