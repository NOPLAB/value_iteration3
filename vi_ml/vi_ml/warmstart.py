"""Does a CNN field make the exact solver faster? Held-out 64² maps, `bench_map
--init-value` (a θ-min field copied into all 60 θ).

Copying θ-min into every θ is optimistic at 59/60 headings, and VI seeded below
V* must climb back up (slowly, and the truncating update can stall there). So
each seed is also tried *lifted*: + the cost of turning in place to the best
heading, 9 steps × (1 s + the safety penalty of that cell). That makes the true
field an upper bound, which is what a descending VI needs.

Two questions: (1) run to convergence — fewer updates than cold, same answer?
(2) cap the rounds at k — how often does greedy descent reach the goal?

    python -m vi_ml.warmstart out/train64.npz out/unet.pt --n 50
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch

from . import maps, rollout, vi
from .eval import load_model
from .model import encode, predict


MAX_ITERS = 2000  # cold converges in ~60-230 rounds; a warm start that needs more is a hang, not a win
TURN_STEPS = 9    # 180° at 20°/step
CAPS = (5, 10, 20, 40)


def turn_cost(free: np.ndarray) -> np.ndarray:
    """Seconds to turn in place to any heading: 9 steps × (1 + penalty), penalty inside the
    safety band (L∞ ≤ 2 cells of an obstacle, the same square test State::from_occupancy uses)."""
    r = round(vi.SAFETY_RADIUS_M / vi.RES_M)
    occ = np.pad(~free, r, constant_values=True)
    win = np.lib.stride_tricks.sliding_window_view(occ, (2 * r + 1, 2 * r + 1)).any(axis=(2, 3))
    return TURN_STEPS * (1.0 + vi.SAFETY_PENALTY * win)


def real(yaml: Path, truth: Path, scale: int, starts: int, top: int):
    """Tsudanuma: dense frontier2d_sparse cold vs warm from the truth (+turn, the best any
    seed could do) and from the coarse exact field `multiscale` already builds (+turn)."""
    from .multiscale import load_real, solve_map
    free = load_real(yaml, scale)
    true = vi.read_dump(truth)
    goal = tuple(int(v) for v in np.argwhere(true == 0)[len(np.argwhere(true == 0)) // 2])
    tc = turn_cost(free)
    st = {}
    cold, cold_ms = vi.solve(free, goal, stats=st)
    print(f"cold: iters={st['iters']} updates={st['updates'] / 1e6:.0f}M solve={cold_ms:.0f} ms, "
          f"max|Δ| vs truth dump {np.nanmax(np.abs(cold - true)):.4f}", flush=True)
    _, guide, t = solve_map(None, free, goal, top=top, mode="exact")
    print(f"coarse exact field: {t[0]:.0f} ms", flush=True)
    m = free & np.isfinite(cold)
    rng = np.random.default_rng(0)
    ss = [maps.pick_free(rng, m) for _ in range(starts)]
    print("| seed | min(seed−V*) s | iters | updates M | solve ms | max |Δ| vs cold (s) | converged |")
    print("|---|---|---|---|---|---|---|")
    for name, init in (("true+turn", true + tc), ("coarse+turn", guide + tc), ("coarse", guide)):
        s = {}
        v, ms = vi.solve(free, goal, init=init, stats=s, max_iters=3 * st["iters"])
        print(f"| {name} | {np.nanmin(init[m] - cold[m]):.1f} | {s['iters']} | {s['updates'] / 1e6:.0f} | {ms:.0f} | "
              f"{np.nanmax(np.abs(v[m] - cold[m])):.4f} | {s['converged']} |", flush=True)
    print(f"\n| k rounds (cold converges in {st['iters']}) | cold reach | cold ms | coarse+turn reach | coarse+turn ms |")
    print("|---|---|---|---|---|")
    for k in (0, st["iters"] // 8, st["iters"] // 4, st["iters"] // 2):
        row = [str(k)]
        for init in (None, guide + tc):
            if k == 0:
                v, ms = (np.full_like(cold, np.nan) if init is None else init), 0.0
            else:
                v, ms = vi.solve(free, goal, init=init, max_iters=k)
            row += [f"{rollout.evaluate(v, cold, ss)[0]:.2f}", f"{ms:.0f}"]
        print("| " + " | ".join(row) + " |", flush=True)


def main(argv=None):
    import sys
    if (argv or sys.argv[1:])[:1] == ["real"]:
        ap = argparse.ArgumentParser()
        ap.add_argument("cmd")
        ap.add_argument("yaml", type=Path)
        ap.add_argument("truth", type=Path)
        ap.add_argument("--scale", type=int, default=2)
        ap.add_argument("--starts", type=int, default=20)
        ap.add_argument("--top", type=int, default=768)
        a = ap.parse_args(argv)
        return real(a.yaml, a.truth, a.scale, a.starts, a.top)
    ap = argparse.ArgumentParser()
    ap.add_argument("data", type=Path)
    ap.add_argument("weights", type=Path)
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--starts", type=int, default=8)
    a = ap.parse_args(argv)
    d = np.load(a.data)
    model = load_model(a.weights)
    torch.set_num_threads(1)
    rng = np.random.default_rng(0)
    seeds = ("cold", "true", "true+turn", "unet", "unet+turn", "unet×1.2+turn")
    conv = {s: [] for s in seeds}
    capped = {(s, k): [] for s in ("cold", "unet", "unet+turn") for k in (0,) + CAPS}
    for i in range(len(d["free"]) - a.n, len(d["free"])):
        free, goal, true = d["free"][i], tuple(int(v) for v in d["goal"][i]), d["value"][i]
        pred = predict(model, encode(free, goal))
        tc = turn_cost(free)
        init = {"cold": None, "true": true, "true+turn": true + tc, "unet": pred,
                "unet+turn": pred + tc, "unet×1.2+turn": 1.2 * pred + tc}
        m = free & np.isfinite(true)
        for s in seeds:
            st = {}
            v, ms = vi.solve(free, goal, init=init[s], stats=st, max_iters=MAX_ITERS)
            conv[s].append((st["iters"], st["updates"], ms, float(np.nanmax(np.abs(v[m] - true[m]))),
                            st["converged"], float(np.nanmin(init[s][m] - true[m])) if init[s] is not None else 0.0))
        ss = [maps.pick_free(rng, m) for _ in range(a.starts)]
        for s in ("cold", "unet", "unet+turn"):
            for k in (0,) + CAPS:
                if k == 0:
                    v, ms = (np.full_like(true, np.nan) if init[s] is None else init[s]), 0.0
                else:
                    v, ms = vi.solve(free, goal, init=init[s], max_iters=k)
                capped[(s, k)].append((rollout.evaluate(v, true, ss)[0], ms))

    print(f"### run to convergence (n={a.n}, cap {MAX_ITERS})\n")
    print("| seed | min(seed−V*) s | iters | updates M | solve ms | max |Δ| vs V* (s) | not converged |")
    print("|---|---|---|---|---|---|---|")
    for s in seeds:
        c = np.array(conv[s], float)
        print(f"| {s} | {c[:, 5].min():.1f} | {np.median(c[:, 0]):.0f} | {np.median(c[:, 1]) / 1e6:.2f} | "
              f"{np.median(c[:, 2]):.1f} | {c[:, 3].max():.4f} | {int((c[:, 4] == 0).sum())}/{len(c)} |")
    print("\n### k rounds only: greedy reach (solve ms median)\n")
    print("| seed | " + " | ".join(f"k={k}" for k in (0,) + CAPS) + " |")
    print("|---|" + "---|" * (1 + len(CAPS)))
    for s in ("cold", "unet", "unet+turn"):
        cells = []
        for k in (0,) + CAPS:
            c = np.array(capped[(s, k)], float)
            cells.append(f"{c[:, 0].mean():.3f} ({np.median(c[:, 1]):.0f})")
        print(f"| {s} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main()
