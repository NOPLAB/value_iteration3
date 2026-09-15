"""Generate (free, goal, value) samples with the exact solver → one .npz.

    python -m vi_ml.dataset out/train64.npz --n 4000 --size 64
    python -m vi_ml.dataset out/maps128.npz --n 800 --size 128
"""
from __future__ import annotations

import argparse
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

from . import maps, vi

HOUSE = vi.REPO / "assets" / "tb3_house" / "map.pgm"
MIX = {"rooms": 0.4, "maze": 0.2, "blobs": 0.2, "crop": 0.2}


def load_house(scale: int = 2) -> np.ndarray:
    """tb3_house at 0.05 m, obstacle-dominant pooled ×2 to the 0.1 m cell used everywhere."""
    raw = HOUSE.read_bytes()
    parts = raw.split(b"\n", 3)
    w, h = map(int, parts[1].split())
    img = np.frombuffer(parts[3][-w * h:], np.uint8).reshape(h, w)
    free = img > 250
    H, W = h // scale * scale, w // scale * scale
    free = free[:H, :W].reshape(H // scale, scale, W // scale, scale).all(axis=(1, 3))
    return free[::-1]  # image row 0 is max-y; grid row = world row like the loader


def make_map(rng: np.random.Generator, size: int, house: np.ndarray):
    kind = str(rng.choice(list(MIX), p=list(MIX.values())))
    if kind == "crop" and min(house.shape) < size:
        kind = "rooms"  # the real map is smaller than the requested crop
    free = maps.crop(rng, house, size) if kind == "crop" else maps.KINDS[kind](rng, size)
    free = maps.thicken(free)
    goal = maps.pick_free(rng, free)
    free = maps.largest_component(free, goal)
    return kind, free, goal


def sample(seed: int, size: int, house: np.ndarray):
    rng = np.random.default_rng(seed)
    kind, free, goal = make_map(rng, size, house)
    value, ms = vi.solve(free, goal)
    return kind, free, goal, value, ms


def generate(n: int, size: int, seed0: int = 0, workers: int = 4) -> dict:
    house = load_house()
    with ThreadPoolExecutor(workers) as ex:
        rows = list(ex.map(lambda s: sample(s, size, house), range(seed0, seed0 + n)))
    kinds, frees, goals, values, mss = zip(*rows)
    return dict(kind=np.array(kinds), free=np.stack(frees), goal=np.array(goals, np.int16),
                value=np.stack(values), solve_ms=np.array(mss, np.float32))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--n", type=int, default=4000)
    ap.add_argument("--size", type=int, default=64)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=4)
    a = ap.parse_args(argv)
    d = generate(a.n, a.size, a.seed, a.workers)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(a.out, **d)
    kinds = dict(zip(*np.unique(d["kind"], return_counts=True)))
    reach = np.isfinite(d["value"]).mean()
    print(f"{a.out}: {a.n}x{a.size}^2 kinds={kinds} reachable={reach:.2f} "
          f"solve_ms median={np.median(d['solve_ms']):.1f}", file=sys.stderr)


if __name__ == "__main__":
    main()
