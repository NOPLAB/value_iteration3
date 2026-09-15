"""One small U-Net, any map size: a value+policy pyramid.

The 64²-window model is fully convolutional, so it runs on a map of any size;
what it lacks is the direction of a goal beyond its receptive field. The
pyramid supplies that: the map is majority-pooled 2× per level until it fits
the top level (≤ TOP cells), the model predicts a value field there (goal in
view, no guidance), and each finer level gets the coarser prediction upsampled
as a dense *guidance* channel (the model's boundary channels, now filled
everywhere) and predicts its own value and, at the finest level, the policy
that is followed. One pass per level, the finest tiled with a halo; total cost
≈ 1.33 passes over the finest level. Weights are shared across levels.

    python -m vi_ml.multiscale prepare out/ms.npz out/train64.npz out/maps128.npz out/maps256.npz
    python -m vi_ml.multiscale train   out/ms.npz --out out/ms.pt
    python -m vi_ml.multiscale eval    out/maps256.npz out/ms.pt --n 40
    python -m vi_ml.multiscale real    assets/map_tsudanuma.yaml out/tsudanuma_s2.bin out/ms.pt --scale 2
"""
from __future__ import annotations

import argparse
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from . import maps, rollout, vi
from .model import UNet, cpu_state_dict, decode_value, device, encode, encode_value
from .policy import K, follow, labels

TOP = 64          # top level fits the training window: goal always in view
WIN = 64
COUT = 1 + K * K  # value + policy logits


# ---------------------------------------------------------------- pyramid ops
def pool_free(free: np.ndarray) -> np.ndarray:
    """Majority 2×2 pooling (odd edges padded with obstacle). Keeps corridors
    wider than a cell, loses walls thinner than one — the finer level corrects."""
    H, W = free.shape
    f = np.zeros((H + H % 2, W + W % 2), bool)
    f[:H, :W] = free
    return f.reshape(f.shape[0] // 2, 2, f.shape[1] // 2, 2).sum(axis=(1, 3)) >= 2


def pool_free_conservative(free: np.ndarray) -> np.ndarray:
    """Obstacle-dominant 2×2 pooling (any obstacle → obstacle), for the exact top
    level: walls survive, corridors narrower than a cell close (on Tsudanuma at
    0.1 m: 0.4 m cells keep the goal reachable, 0.8 m cells isolate it)."""
    H, W = free.shape
    f = np.zeros((H + H % 2, W + W % 2), bool)
    f[:H, :W] = free
    return f.reshape(f.shape[0] // 2, 2, f.shape[1] // 2, 2).all(axis=(1, 3))


def pool_value(value: np.ndarray, pooled_free: np.ndarray) -> np.ndarray:
    """nan-min 2×2 pooling of a value field, NaN where the pooled map is an obstacle."""
    H, W = value.shape
    v = np.full((H + H % 2, W + W % 2), np.nan, np.float32)
    v[:H, :W] = value
    with np.errstate(all="ignore"):
        p = np.nanmin(v.reshape(v.shape[0] // 2, 2, v.shape[1] // 2, 2), axis=(1, 3))
    p[~pooled_free] = np.nan
    return p


def upsample(v: np.ndarray, shape) -> np.ndarray:
    """Bilinear 2× (NaN filled with a large value first). Nearest upsampling leaves
    2×2 plateaus that greedy descent cannot cross: on 256² maps the *true* 2×-pooled
    field reaches the goal 14 % of the time nearest vs 91 % bilinear."""
    fill = float(np.nanmax(v)) * 2 if np.isfinite(v).any() else 0.0
    x = torch.from_numpy(np.nan_to_num(v, nan=fill).astype(np.float32))[None, None]
    y = F.interpolate(x, scale_factor=2, mode="bilinear", align_corners=False)[0, 0].numpy()
    return y[:shape[0], :shape[1]]


def pyramid(free, goal, value=None, top: int = TOP):
    """[(free, goal, value)] from the given level up to one with max side ≤ top."""
    levels = [(free, goal, value)]
    while max(free.shape) > top:
        free = pool_free(free)
        goal = (goal[0] // 2, goal[1] // 2)
        free[goal] = True
        value = pool_value(value, free) if value is not None else None
        if value is not None:
            value[goal] = 0.0
        levels.append((free, goal, value))
    return levels


# ---------------------------------------------------------------- samples
def _window(free, goal, value, lab, guide, y0, x0, rng):
    f = free[y0:y0 + WIN, x0:x0 + WIN]
    v = value[y0:y0 + WIN, x0:x0 + WIN]
    g = (goal[0] - y0, goal[1] - x0)
    g = g if 0 <= g[0] < WIN and 0 <= g[1] < WIN else None
    b = None
    if guide is not None:
        b = guide[y0:y0 + WIN, x0:x0 + WIN]
    return (encode(f, g, b), encode_value(v).astype(np.float32),
            (f & np.isfinite(v)).astype(np.float32), lab[y0:y0 + WIN, x0:x0 + WIN].astype(np.int16))


def coarse_guide(free, goal, levels: int):
    """The guide exactly as inference builds it: obstacle-dominant pooling `levels`
    times, exact VI there (step scaled so it still covers 3 cells, value scaled back
    to fine-scale seconds), bilinear upsampling back to the fine grid."""
    shapes, fc = [free.shape], free
    for _ in range(levels):
        fc = pool_free_conservative(fc)
        shapes.append(fc.shape)
    g = (goal[0] >> levels, goal[1] >> levels)
    fc[g] = True
    v, ms = vi.solve(fc, g, res=vi.RES_M * 2 ** levels, action_scale=2 ** levels)
    v = v * 2 ** levels
    v[~fc] = np.nan
    for l in range(levels - 1, -1, -1):
        v = upsample(v, shapes[l])
    v[~free] = np.nan
    return v, ms


def samples_for_map(args):
    """One solved map → 64² windows of (map, goal, coarse guide) → (value, policy).

    Two levels only: the guide always comes from an exact coarse solve, never from
    another prediction, so nothing compounds. `levels` varies so one model refines
    a 2× or a 4× coarser guide."""
    free, goal, value, seed, per_map = args
    rng = np.random.default_rng(seed)
    lab = labels(value)
    H, W = free.shape
    out = []
    for levels in (1, 2):
        if min(H, W) >> levels < 16:
            continue
        guide, _ = coarse_guide(free, goal, levels)
        for _ in range(per_map):
            y0, x0 = int(rng.integers(0, H - WIN + 1)), int(rng.integers(0, W - WIN + 1))
            if free[y0:y0 + WIN, x0:x0 + WIN].mean() < 0.1:
                continue
            out.append(_window(free, goal, value, lab, guide, y0, x0, rng))
    return out


def _map_jobs(npzs, per_map: int, holdout: int):
    jobs = []
    for p in npzs:
        d = np.load(p)
        F, G, V = d["free"], d["goal"], d["value"]  # materialise once (NpzFile re-decompresses per access)
        n = len(F) - (holdout if F.shape[1] > 64 else 400)
        jobs += [(F[i], tuple(int(v) for v in G[i]), V[i], hash((str(p), i)) & 0xFFFFFFFF, per_map)
                 for i in range(n)]
    return jobs


def prepare(out: Path, npzs: list[Path], per_map: int = 3, holdout: int = 40, workers: int = 4,
            shard: int = 0, shards: int = 1):
    """Windows for maps `shard::shards`, written to `out`. Sharding keeps each slurm
    task short — this cluster's controller is reached over a VPN whose DNS drops
    under load, and a requeue then costs one shard instead of the whole run."""
    if out.exists():
        print(f"{out} exists, skipping", file=sys.stderr)
        return
    jobs = _map_jobs(npzs, per_map, holdout)[shard::shards]
    print(f"shard {shard}/{shards}: {len(jobs)} maps", file=sys.stderr)
    with ProcessPoolExecutor(workers) as ex:
        rows = [s for chunk in ex.map(samples_for_map, jobs, chunksize=4) for s in chunk]
    xs, ys, ws, ls = (np.stack(c) for c in zip(*rows))
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, x=xs, y=ys, w=ws, lab=ls)
    print(f"{out}: {len(xs)} windows ({xs.nbytes / 1e6:.0f} MB)", file=sys.stderr)


def merge(out: Path, parts: list[Path]):
    acc = {k: [] for k in ("x", "y", "w", "lab")}
    for p in sorted(parts):
        d = np.load(p)
        for k in acc:
            acc[k].append(d[k])
    merged = {k: np.concatenate(v) for k, v in acc.items()}
    np.savez(out, **merged)
    print(f"{out}: {len(merged['x'])} windows from {len(parts)} shards "
          f"({merged['x'].nbytes / 1e6:.0f} MB)", file=sys.stderr)


# ---------------------------------------------------------------- train
def train(d, epochs: int, out: Path, val_frac: float = 0.1, seed: int = 0, batch: int = 32):
    torch.manual_seed(seed)
    X, Y, Wt, L = (torch.from_numpy(d[k]) for k in ("x", "y", "w", "lab"))
    idx = np.random.default_rng(seed).permutation(len(X))
    nv = int(len(X) * val_frac)
    va, tr = idx[:nv], idx[nv:]
    dev = device()
    print(f"device {dev}, {len(X)} windows", file=sys.stderr)
    model = UNet(cout=COUT).to(dev)
    opt = torch.optim.AdamW(model.parameters(), 1e-3, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, 2e-3, total_steps=epochs * ((len(tr) + batch - 1) // batch))

    def loss_of(b):
        out = model(X[b].to(dev))
        y, w, lab = Y[b].to(dev), Wt[b].to(dev), L[b].long().to(dev)
        lv = (torch.abs(out[:, 0] - y) * w).sum() / w.sum().clamp(min=1)
        lp = F.cross_entropy(out[:, 1:], lab, ignore_index=-1)
        return lv + lp, lv, lp

    best = np.inf
    for ep in range(epochs):
        model.train()
        t0 = time.time()
        perm = np.random.permutation(tr)
        for i in range(0, len(perm), batch):
            loss, _, _ = loss_of(perm[i:i + batch])
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
        model.eval()
        with torch.no_grad():
            v = np.mean([[a.item() for a in loss_of(va[i:i + 64])] for i in range(0, nv, 64)], axis=0)
        print(f"epoch {ep + 1}/{epochs} val {v[0]:.4f} (value {v[1]:.4f} policy {v[2]:.4f}) ({time.time() - t0:.0f}s)", file=sys.stderr)
        if v[0] < best:
            best = v[0]
            torch.save(cpu_state_dict(model), out)


def load_model(path: Path) -> UNet:
    m = UNet(cout=COUT)
    m.load_state_dict(torch.load(path, map_location="cpu", weights_only=True))
    return m.eval()


# ---------------------------------------------------------------- inference
@torch.no_grad()
def forward_tiled(model, x: np.ndarray, tile: int = 512, halo: int = 32) -> np.ndarray:
    """Fully-convolutional forward of (4,H,W) in tiles with a halo; output (COUT,H,W)."""
    _, H, W = x.shape
    Hp, Wp = -(-H // 16) * 16, -(-W // 16) * 16
    xp = np.zeros((4, Hp + 2 * halo, Wp + 2 * halo), np.float32)
    xp[:, halo:halo + H, halo:halo + W] = x
    out = np.zeros((COUT, Hp, Wp), np.float32)
    for y0 in range(0, Hp, tile):
        for x0 in range(0, Wp, tile):
            y1, x1 = min(y0 + tile, Hp), min(x0 + tile, Wp)
            t = torch.from_numpy(xp[:, y0:y1 + 2 * halo, x0:x1 + 2 * halo])[None]
            o = model(t)[0].numpy()
            out[:, y0:y1, x0:x1] = o[:, halo:halo + (y1 - y0), halo:halo + (x1 - x0)]
    return out[:, :H, :W]


def solve_map(model, free: np.ndarray, goal, top: int = TOP, refine: bool = True):
    """→ (policy logits (K²,H,W), value seconds (H,W), [coarse ms, refine ms]).

    Exact VI on the map pooled down to `top`, then one fully-convolutional pass of
    the small model over the whole fine map with that guide. Two levels, one pass:
    nothing compounds, and the cost is one forward over the fine grid."""
    levels = 0
    f = free
    while max(f.shape) > top:
        f = pool_free_conservative(f)
        levels += 1
    t0 = time.perf_counter()
    guide, _ = coarse_guide(free, goal, levels)
    t1 = time.perf_counter()
    if not refine:
        return None, guide, [(t1 - t0) * 1e3, 0.0]
    out = forward_tiled(model, encode(free, goal, guide))
    val = decode_value(out[0])
    val[~free] = np.nan
    return out[1:], val, [(t1 - t0) * 1e3, (time.perf_counter() - t1) * 1e3]


def evaluate_maps(model, items, starts: int, label: str, top: int = TOP, refine: bool = True):
    """items: iterable of (kind, free, goal, true_value, vi_ms)."""
    rows = []
    rng = np.random.default_rng(0)
    for kind, free, goal, true, vi_ms in items:
        t0 = time.perf_counter()
        lg, val, per_level = solve_map(model, free, goal, top=top, refine=refine)
        guide = val if lg is None else None
        ms = (time.perf_counter() - t0) * 1e3
        m = free & np.isfinite(true)
        gm = true == 0
        ok, okv, ratios = 0, 0, []
        for _ in range(starts):
            s = maps.pick_free(rng, m)
            r_t, p_t = rollout.greedy(true, s, gm)
            r_p, p_p = follow(lg, free, s, gm) if lg is not None else rollout.greedy(val, s, gm)
            r_v, _ = rollout.greedy(val, s, gm)
            ok += r_p
            okv += r_v
            if r_p and r_t and rollout.length(p_t) > 0:
                ratios.append(rollout.length(p_p) / rollout.length(p_t))
        rel = float(np.nanmean(np.abs(val[m] - true[m]) / (true[m] + 1)))
        rows.append((kind, free.shape, ok / starts, okv / starts, float(np.mean(ratios)) if ratios else np.nan, rel, ms, vi_ms))
        print(f"  {kind} {free.shape} success policy {ok / starts:.2f} value-greedy {okv / starts:.2f} ratio {rows[-1][4]:.2f} "
              f"rel.err {rel:.2f} {label} {ms:.0f} ms (levels {[round(t) for t in per_level]}) vi {vi_ms:.0f} ms", file=sys.stderr)
    print(f"| kind | maps | success(policy) | success(value greedy) | len ratio | rel.err | {label} ms | vi ms |")
    print("|---|---|---|---|---|---|---|---|")
    for k in sorted({r[0] for r in rows}) + ["all"]:
        rs = [r for r in rows if k == "all" or r[0] == k]
        c = np.array([r[2:] for r in rs], float)
        print(f"| {k} | {len(rs)} | {c[:, 0].mean():.3f} | {c[:, 1].mean():.3f} | {np.nanmean(c[:, 2]):.2f} | {c[:, 3].mean():.2f} | "
              f"{np.median(c[:, 4]):.0f} | {np.median(c[:, 5]):.0f} |")


def load_real(yaml: Path, scale: int) -> np.ndarray:
    """ROS map_server PGM → free grid the way `bench_map` builds it: unknown = obstacle,
    image flipped so row 0 is y = origin, obstacle-dominant ×scale pooling."""
    meta = dict(l.split(":", 1) for l in yaml.read_text().splitlines() if ":" in l)
    raw = (yaml.parent / meta["image"].strip()).read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:  # P5 header with optional comment lines
        end = raw.index(b"\n", pos)
        line = raw[pos:end].strip()
        pos = end + 1
        if line and not line.startswith(b"#"):
            tokens += line.split()
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(raw[pos:pos + w * h], np.uint8).reshape(h, w)
    occ = (255 - img.astype(np.float32)) / 255.0
    free = (occ < float(meta["free_thresh"]))[::-1]
    if bool(int(meta.get("negate", "0"))):
        free = ~free
    H, W = -(-h // scale), -(-w // scale)
    f = np.zeros((H * scale, W * scale), bool)
    f[:h, :w] = free
    return f.reshape(H, scale, W, scale).all(axis=(1, 3))


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    pr = sub.add_parser("prepare")
    pr.add_argument("out", type=Path)
    pr.add_argument("npzs", type=Path, nargs="+")
    pr.add_argument("--per-map", type=int, default=3)
    pr.add_argument("--workers", type=int, default=4)
    pr.add_argument("--shard", type=int, default=0)
    pr.add_argument("--shards", type=int, default=1)
    mg = sub.add_parser("merge")
    mg.add_argument("out", type=Path)
    mg.add_argument("parts", type=Path, nargs="+")
    tr = sub.add_parser("train")
    tr.add_argument("samples", type=Path)
    tr.add_argument("--out", type=Path, default=Path("out/ms.pt"))
    tr.add_argument("--epochs", type=int, default=25)
    ev = sub.add_parser("eval")
    ev.add_argument("data", type=Path)
    ev.add_argument("weights", type=Path, nargs="?")
    ev.add_argument("--n", type=int, default=40)
    ev.add_argument("--starts", type=int, default=8)
    ev.add_argument("--no-refine", dest="refine", action="store_false", help="coarse exact VI only, no model")
    ev.add_argument("--top", type=int, default=TOP, help="pool until the long side fits this; exact VI there")
    re_ = sub.add_parser("real")
    re_.add_argument("yaml", type=Path)
    re_.add_argument("truth", type=Path, help="bench_map --dump-value of the same map/scale/goal")
    re_.add_argument("weights", type=Path, nargs="?")
    re_.add_argument("--scale", type=int, default=2)
    re_.add_argument("--vi-ms", type=float, default=np.nan)
    re_.add_argument("--starts", type=int, default=20)
    re_.add_argument("--no-refine", dest="refine", action="store_false", help="coarse exact VI only, no model")
    re_.add_argument("--top", type=int, default=768, help="Tsudanuma at 0.1 m: 768 → exact VI at 0.4 m (736×500)")
    a = ap.parse_args(argv)
    if a.cmd == "prepare":
        prepare(a.out, a.npzs, a.per_map, workers=a.workers, shard=a.shard, shards=a.shards)
    elif a.cmd == "merge":
        merge(a.out, a.parts)
    elif a.cmd == "train":
        a.out.parent.mkdir(parents=True, exist_ok=True)
        train(np.load(a.samples), a.epochs, a.out)
    elif a.cmd == "eval":
        d = np.load(a.data)
        model = load_model(a.weights) if a.refine else None

        def items():
            for i in range(len(d["free"]) - a.n, len(d["free"])):
                free, goal = d["free"][i], tuple(int(v) for v in d["goal"][i])
                _, vi_ms = vi.solve(free, goal)
                yield str(d["kind"][i]), free, goal, d["value"][i], vi_ms
        evaluate_maps(model, items(), a.starts, "pyramid", a.top, a.refine)
    else:
        free = load_real(a.yaml, a.scale)
        true = vi.read_dump(a.truth)
        assert true.shape == free.shape, (true.shape, free.shape)
        gy, gx = np.argwhere(true == 0).mean(axis=0).round().astype(int)
        goal = (int(gy), int(gx))
        if not free[goal]:
            goal = tuple(int(v) for v in np.argwhere(true == 0)[0])
        evaluate_maps(load_model(a.weights) if a.refine else None,
                      [(a.yaml.stem, free, goal, true, a.vi_ms)], a.starts, "pyramid", a.top, a.refine)


if __name__ == "__main__":
    main()
