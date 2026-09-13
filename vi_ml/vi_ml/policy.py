"""Policy head: predict the greedy *jump* per cell instead of the value.

Label = the best line-of-sight jump (dy, dx) ∈ [-R, R]² the radius-R greedy
descent would take on the true field (`rollout.greedy`'s rule, vectorised),
as one of (2R+1)² classes. Rollout takes the highest-logit jump whose line is
clear, so there is no value to get stuck in — the failure mode becomes loops,
which the step cap catches.

    python -m vi_ml.policy train out/train64.npz --out out/policy.pt
    python -m vi_ml.policy eval  out/train64.npz out/policy.pt
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from . import maps, rollout, vi
from .model import UNet, cpu_state_dict, device, encode
from .rollout import RADIUS

K = 2 * RADIUS + 1
OFFSETS = [(dy, dx) for dy in range(-RADIUS, RADIUS + 1) for dx in range(-RADIUS, RADIUS + 1)]
CENTER = OFFSETS.index((0, 0))


def _line_points(dy, dx):
    """Interior+end points `rollout._clear` checks for a jump (dy, dx)."""
    n = 2 * max(abs(dy), abs(dx))
    return sorted({(int(np.round(dy * t)), int(np.round(dx * t))) for t in np.linspace(0, 1, n + 1)[1:]})


def _shift(a: np.ndarray, dy: int, dx: int, fill):
    """out[y, x] = a[y + dy, x + dx] (fill outside)."""
    H, W = a.shape
    out = np.full_like(a, fill)
    ys, yd = (slice(dy, H), slice(0, H - dy)) if dy >= 0 else (slice(0, H + dy), slice(-dy, H))
    xs, xd = (slice(dx, W), slice(0, W - dx)) if dx >= 0 else (slice(0, W + dx), slice(-dx, W))
    out[yd, xd] = a[ys, xs]
    return out


def jump_costs(value: np.ndarray) -> np.ndarray:
    """(K², H, W): value of the target of each jump, +inf where the line is blocked."""
    fin = np.isfinite(value)
    out = np.full((K * K, *value.shape), np.inf, np.float32)
    for k, (dy, dx) in enumerate(OFFSETS):
        if (dy, dx) == (0, 0):
            continue
        clear = np.ones_like(fin)
        for py, px in _line_points(dy, dx):
            clear &= _shift(fin, py, px, False)
        out[k] = np.where(clear, _shift(np.nan_to_num(value, nan=np.inf), dy, dx, np.inf), np.inf)
    return out


def labels(value: np.ndarray) -> np.ndarray:
    """Per-cell best-jump class, -1 where no descending jump exists (goal, unreachable, obstacle)."""
    jc = jump_costs(value)
    best = jc.argmin(0)
    ok = np.isfinite(value) & (jc.min(0) < np.nan_to_num(value, nan=-np.inf))
    return np.where(ok, best, -1)


def train(xs, ys, epochs: int, out: Path, val_frac: float = 0.1, seed: int = 0, batch: int = 32):
    torch.manual_seed(seed)
    idx = np.random.default_rng(seed).permutation(len(xs))
    nv = int(len(xs) * val_frac)
    va, tr = idx[:nv], idx[nv:]
    X, Y = torch.from_numpy(xs), torch.from_numpy(ys.astype(np.int64))
    dev = device()
    print(f"device {dev}", file=sys.stderr)
    model = UNet(cout=K * K).to(dev)
    opt = torch.optim.AdamW(model.parameters(), 1e-3, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, 2e-3, total_steps=epochs * ((len(tr) + batch - 1) // batch))

    def loss_of(b):
        return F.cross_entropy(model(X[b].to(dev)), Y[b].to(dev), ignore_index=-1)

    best = np.inf
    for ep in range(epochs):
        model.train()
        t0 = time.time()
        perm = np.random.permutation(tr)
        tl = 0.0
        for i in range(0, len(perm), batch):
            b = perm[i:i + batch]
            loss = loss_of(b)
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            tl += loss.item() * len(b)
        model.eval()
        with torch.no_grad():
            vl = float(np.mean([loss_of(va[i:i + 64]).item() for i in range(0, nv, 64)]))
        print(f"epoch {ep + 1}/{epochs} train {tl / len(tr):.4f} val {vl:.4f} ({time.time() - t0:.0f}s)", file=sys.stderr)
        if vl < best:
            best = vl
            torch.save(cpu_state_dict(model), out)


def load_model(path: Path) -> UNet:
    m = UNet(cout=K * K)
    m.load_state_dict(torch.load(path, map_location="cpu", weights_only=True))
    return m.eval()


def logits(model, x: np.ndarray) -> np.ndarray:
    with torch.no_grad():
        return model(torch.from_numpy(x)[None])[0].numpy()


def follow(lg: np.ndarray, free: np.ndarray, start, goal_mask, max_steps: int | None = None):
    """Take, at each cell, the highest-logit jump whose line is clear. (reached, path)."""
    H, W = free.shape
    max_steps = max_steps or 2 * (H + W)
    fin = free.astype(np.float32)
    fin[~free] = np.nan  # rollout._clear wants NaN = blocked
    y, x = start
    path = [(y, x)]
    for _ in range(max_steps):
        if goal_mask[y, x]:
            return True, path
        for k in np.argsort(-lg[:, y, x]):
            dy, dx = OFFSETS[k]
            ny, nx = y + dy, x + dx
            if (dy, dx) == (0, 0) or not (0 <= ny < H and 0 <= nx < W) or not free[ny, nx]:
                continue
            if rollout._clear(fin, y, x, ny, nx):
                y, x = ny, nx
                break
        else:
            return False, path
        path.append((y, x))
    return bool(goal_mask[y, x]), path


def evaluate(d, model, n: int, starts: int):
    torch.set_num_threads(1)
    rng = np.random.default_rng(0)
    rows = []
    for i in range(len(d["free"]) - n, len(d["free"])):
        free, goal, true = d["free"][i], tuple(d["goal"][i]), d["value"][i]
        t0 = time.perf_counter()
        lg = logits(model, encode(free, goal))
        ms = (time.perf_counter() - t0) * 1e3
        m = free & np.isfinite(true)
        gm = true == 0
        lab = labels(true)
        acc = float((lg.argmax(0) == lab)[lab >= 0].mean()) if (lab >= 0).any() else np.nan
        ok, ratios = 0, []
        for _ in range(starts):
            s = maps.pick_free(rng, m)
            r_t, p_t = rollout.greedy(true, s, gm)
            r_p, p_p = follow(lg, free, s, gm)
            ok += r_p
            if r_p and r_t and rollout.length(p_t) > 0:
                ratios.append(rollout.length(p_p) / rollout.length(p_t))
        _, vi_ms = vi.solve(free, goal)
        rows.append((str(d["kind"][i]), acc, ok / starts, float(np.mean(ratios)) if ratios else np.nan, ms, vi_ms))
    print("| kind | n | jump acc | success(policy) | len ratio | policy ms | vi ms |")
    print("|---|---|---|---|---|---|---|")
    for k in sorted({r[0] for r in rows}) + ["all"]:
        rs = [r for r in rows if k == "all" or r[0] == k]
        c = np.array([r[1:] for r in rs], float)
        print(f"| {k} | {len(rs)} | {np.nanmean(c[:, 0]):.3f} | {c[:, 1].mean():.3f} | {np.nanmean(c[:, 2]):.2f} | "
              f"{np.median(c[:, 3]):.1f} | {np.median(c[:, 4]):.1f} |")


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    tr = sub.add_parser("train")
    tr.add_argument("train64", type=Path)
    tr.add_argument("--out", type=Path, default=Path("out/policy.pt"))
    tr.add_argument("--epochs", type=int, default=25)
    tr.add_argument("--holdout", type=int, default=400)
    ev = sub.add_parser("eval")
    ev.add_argument("data", type=Path)
    ev.add_argument("weights", type=Path)
    ev.add_argument("--n", type=int, default=400)
    ev.add_argument("--starts", type=int, default=8)
    a = ap.parse_args(argv)
    if a.cmd == "train":
        d = np.load(a.train64)
        sl = slice(None, -a.holdout)
        xs = np.stack([encode(f, tuple(g)) for f, g in zip(d["free"][sl], d["goal"][sl])])
        ys = np.stack([labels(v) for v in d["value"][sl]])
        print(f"{len(xs)} samples, labelled cells {np.mean(ys >= 0):.2f}", file=sys.stderr)
        a.out.parent.mkdir(parents=True, exist_ok=True)
        train(xs, ys, a.epochs, a.out)
    else:
        evaluate(np.load(a.data), load_model(a.weights), a.n, a.starts)


if __name__ == "__main__":
    main()
