"""Do the solvers agree on a real map? `frontier2d` (canonical — the conformance suite
gates it bit-exact against the 本家 reference) vs `frontier2d_sparse`, cold and warm.

    python -m vi_ml.solvercheck ../assets/map_tsudanuma.yaml 152.55 97.55 --scale 4
"""
import argparse
from pathlib import Path

import numpy as np

from . import vi
from .multiscale import load_real
from .warmstart import turn_cost

ap = argparse.ArgumentParser()
ap.add_argument("yaml", type=Path)
ap.add_argument("goal", type=float, nargs=2, help="goal x y in map world metres")
ap.add_argument("--scale", type=int, default=4)
a = ap.parse_args()

meta = dict(l.split(":", 1) for l in a.yaml.read_text().splitlines() if ":" in l)
ox, oy = [float(v) for v in meta["origin"].strip(" []\n").split(",")[:2]]
res = float(meta["resolution"]) * a.scale
free = load_real(a.yaml, a.scale)
goal = (int((a.goal[1] - oy) / res), int((a.goal[0] - ox) / res))
print(f"grid {free.shape} at {res} m, goal cell {goal}, free={free[goal]}", flush=True)

f = {}
for s in ("frontier2d", "frontier2d_sparse"):
    st = {}
    f[s], ms = vi.solve(free, goal, solver=s, res=res, stats=st)
    print(f"{s}: {ms:.0f} ms iters={st['iters']}", flush=True)
f["sparse warm(frontier2d+turn)"], _ = vi.solve(free, goal, init=f["frontier2d"] + turn_cost(free) * 1.0, res=res)
ref = f["frontier2d"]
for k, v in f.items():
    both = np.isfinite(ref) & np.isfinite(v)
    d = np.abs(v - ref)[both]
    print(f"{k:30s} vs frontier2d: cells>1e-3 {int((d > 1e-3).sum())}, max {d.max():.4f} s, "
          f"reachability differs {int((np.isfinite(ref) != np.isfinite(v)).sum())}", flush=True)
