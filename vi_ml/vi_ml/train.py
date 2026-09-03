"""Train the U-Net on whole 64² maps plus 64² windows cut from 128² maps (with
their true border values as the boundary channels).

    python -m vi_ml.train out/train64.npz out/maps128.npz --out out/unet.pt
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch

from .model import UNet, encode, encode_value

RING = 2  # border cells whose true value is fed in as the window boundary
WIN = 64


def whole_samples(d: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    xs = np.stack([encode(f, tuple(g)) for f, g in zip(d["free"], d["goal"])])
    ys = encode_value(d["value"]).astype(np.float32)
    ws = (d["free"] & np.isfinite(d["value"])).astype(np.float32)
    return xs, ys, ws


def window_samples(d: dict, per_map: int, rng: np.random.Generator):
    """Random WIN² crops of solved larger maps; boundary = true value on the RING."""
    xs, ys, ws = [], [], []
    for free, goal, value in zip(d["free"], d["goal"], d["value"]):
        H, W = free.shape
        for _ in range(per_map):
            y0, x0 = int(rng.integers(0, H - WIN + 1)), int(rng.integers(0, W - WIN + 1))
            f = free[y0:y0 + WIN, x0:x0 + WIN]
            v = value[y0:y0 + WIN, x0:x0 + WIN]
            if f.mean() < 0.1:
                continue
            g = (goal[0] - y0, goal[1] - x0)
            g = g if 0 <= g[0] < WIN and 0 <= g[1] < WIN else None
            b = np.full_like(v, np.nan)
            b[:RING], b[-RING:], b[:, :RING], b[:, -RING:] = v[:RING], v[-RING:], v[:, :RING], v[:, -RING:]
            xs.append(encode(f, g, b))
            ys.append(encode_value(v))
            ws.append(f & np.isfinite(v))
    return np.stack(xs), np.stack(ys).astype(np.float32), np.stack(ws).astype(np.float32)


def masked_l1(pred, y, w):
    return (torch.abs(pred - y) * w).sum() / w.sum().clamp(min=1)


def train(xs, ys, ws, epochs: int, out: Path, val_frac: float = 0.1, seed: int = 0, batch: int = 32):
    torch.manual_seed(seed)
    n = len(xs)
    idx = np.random.default_rng(seed).permutation(n)
    nv = int(n * val_frac)
    va, tr = idx[:nv], idx[nv:]
    X, Y, Wt = (torch.from_numpy(a) for a in (xs, ys, ws))
    model = UNet()
    opt = torch.optim.AdamW(model.parameters(), 1e-3, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, 2e-3, total_steps=epochs * ((len(tr) + batch - 1) // batch))
    best = np.inf
    for ep in range(epochs):
        model.train()
        t0 = time.time()
        perm = np.random.permutation(tr)
        tl = 0.0
        for i in range(0, len(perm), batch):
            b = perm[i:i + batch]
            x, y, w = X[b], Y[b], Wt[b]
            if np.random.rand() < 0.5:  # flip augmentation (goal/boundary flip with it)
                x, y, w = x.flip(-1), y.flip(-1), w.flip(-1)
            loss = masked_l1(model(x), y, w)
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            tl += loss.item() * len(b)
        model.eval()
        with torch.no_grad():
            vl = float(np.mean([masked_l1(model(X[va[i:i + 64]]), Y[va[i:i + 64]], Wt[va[i:i + 64]]).item()
                                for i in range(0, nv, 64)])) if nv else np.nan
        print(f"epoch {ep + 1}/{epochs} train {tl / len(tr):.4f} val {vl:.4f} ({time.time() - t0:.0f}s)", file=sys.stderr)
        if vl < best:
            best = vl
            torch.save(model.state_dict(), out)
    return model, va


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("train64", type=Path)
    ap.add_argument("maps128", type=Path, nargs="?")
    ap.add_argument("--out", type=Path, default=Path("out/unet.pt"))
    ap.add_argument("--epochs", type=int, default=25)
    ap.add_argument("--windows-per-map", type=int, default=4)
    ap.add_argument("--holdout", type=int, nargs=2, default=(400, 100),
                    help="samples left out at the END of train64 / maps128 (eval.py / pyramid.py read those)")
    a = ap.parse_args(argv)
    head = lambda d, n: {k: d[k][:-n] for k in d.files}
    xs, ys, ws = whole_samples(head(np.load(a.train64), a.holdout[0]))
    if a.maps128:
        xw, yw, ww = window_samples(head(np.load(a.maps128), a.holdout[1]), a.windows_per_map, np.random.default_rng(0))
        xs, ys, ws = np.concatenate([xs, xw]), np.concatenate([ys, yw]), np.concatenate([ws, ww])
    print(f"{len(xs)} samples", file=sys.stderr)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    train(xs, ys, ws, a.epochs, a.out)


if __name__ == "__main__":
    main()
