"""Figures comparing the multiscale solver against the exact VI.

    python -m vi_ml.figures out/figs --scale 2
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager


def _use_japanese_font():
    """Register a CJK face so labels are not tofu. Prefers a system Noto/IPA font,
    else a copy fetched into assets/ (no root needed either way)."""
    for pat in ("NotoSansCJK", "NotoSansJP", "NotoSerifCJK", "ipaexg", "ipag", "TakaoPGothic", "VL-PGothic"):
        hits = [f for f in font_manager.findSystemFonts() if pat.lower() in Path(f).name.lower()]
        if hits:
            font_manager.fontManager.addfont(hits[0])
            name = font_manager.FontProperties(fname=hits[0]).get_name()
            plt.rcParams["font.family"] = name
            return name
    local = Path(__file__).resolve().parents[1] / "assets" / "NotoSansJP-Regular.ttf"
    if not local.exists():
        import urllib.request
        local.parent.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(
            "https://github.com/notofonts/noto-cjk/raw/main/Sans/SubsetOTF/JP/NotoSansJP-Regular.otf",
            local.with_suffix(".otf"))
        local = local.with_suffix(".otf")
    font_manager.fontManager.addfont(str(local))
    name = font_manager.FontProperties(fname=str(local)).get_name()
    plt.rcParams["font.family"] = name
    return name
import numpy as np
from matplotlib.colors import LinearSegmentedColormap, ListedColormap

from . import maps, rollout, vi
from .multiscale import (RADIUS, coarse_guide, follow_hybrid, load_model, load_real,
                         pool_free_conservative, relative_guide, solve_map, upsample)
from .policy import OFFSETS

INK = "#1c1c1f"
ACCENT = "#e4572e"
ACCENT2 = "#2b6cb0"
VMAP = plt.get_cmap("magma").copy()
VMAP.set_bad("#e8e6e1")


def _style(ax, title=None):
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color("#c9c7c2")
    if title:
        ax.set_title(title, fontsize=10, color=INK, pad=6)


def show_map(ax, free, title=None):
    ax.imshow(np.where(free, 1.0, 0.0), cmap=ListedColormap([INK, "#f6f5f2"]),
              origin="lower", interpolation="nearest")
    _style(ax, title)


def show_value(ax, v, free, title=None, vmax=None):
    m = np.ma.masked_invalid(np.where(free, v, np.nan))
    im = ax.imshow(m, cmap=VMAP, origin="lower", interpolation="nearest",
                   vmin=0, vmax=vmax if vmax else np.nanmax(v))
    _style(ax, title)
    return im


def local_minima(guide, free):
    """Cells with no line-of-sight descending jump — where plain descent dies."""
    H, W = guide.shape
    g = np.where(np.isfinite(guide), guide, np.inf)
    best = np.full_like(g, np.inf)
    for dy in range(-RADIUS, RADIUS + 1):
        for dx in range(-RADIUS, RADIUS + 1):
            if (dy, dx) == (0, 0):
                continue
            sh = np.full_like(g, np.inf)
            ys, yd = (slice(dy, H), slice(0, H - dy)) if dy >= 0 else (slice(0, H + dy), slice(-dy, H))
            xs, xd = (slice(dx, W), slice(0, W - dx)) if dx >= 0 else (slice(0, W + dx), slice(-dx, W))
            sh[yd, xd] = g[ys, xs]
            best = np.minimum(best, sh)
    return free & np.isfinite(guide) & (guide > 0) & (best >= g)


def fig_fields(out, free, goal, true, guide, refined_ok, tag, vi_ms, ours_ms):
    fig, axs = plt.subplots(1, 4, figsize=(16, 4.4), constrained_layout=True)
    show_map(axs[0], free, f"占有格子 {free.shape[1]}×{free.shape[0]}")
    axs[0].plot(goal[1], goal[0], "*", color=ACCENT, ms=16, mec="white", mew=1.0)
    vmax = float(np.nanmax(true))
    show_value(axs[1], true, free, f"厳密 VI（{vi_ms:.0f} ms）", vmax)
    show_value(axs[2], guide, free, f"粗い厳密 VI を拡大（誘導）", vmax)
    err = np.abs(guide - true)
    m = np.ma.masked_invalid(np.where(free, err, np.nan))
    im = axs[3].imshow(m, cmap="cividis", origin="lower", vmin=0, vmax=np.nanpercentile(err, 99))
    _style(axs[3], "誘導と厳密解の差 [s]")
    fig.colorbar(im, ax=axs[3], fraction=0.046, shrink=0.9)
    fig.savefig(out / f"fields_{tag}.png", dpi=110, facecolor="white")
    plt.close(fig)


def fig_paths(out, free, goal, true, guide, lg, tag, starts):
    gm = true == 0
    fig, axs = plt.subplots(1, 2, figsize=(13, 6.2), constrained_layout=True)
    for ax, (fld, lgl, name) in zip(axs, [(true, None, "厳密 VI の値場を降下"),
                                          (guide, lg, "誘導＋モデルで脱出（本手法）")]):
        show_value(ax, true, free, name, float(np.nanmax(true)))
        for i, s in enumerate(starts):
            if lgl is None and fld is true:
                ok, p = rollout.greedy(fld, s, gm)
            else:
                ok, p, _ = follow_hybrid(lgl, fld, free, s, gm)
            p = np.asarray(p)
            ax.plot(p[:, 1], p[:, 0], "-", lw=1.8, color=ACCENT if ok else "#8a8a8a",
                    alpha=0.95, solid_capstyle="round")
            ax.plot(p[0, 1], p[0, 0], "o", color=ACCENT2, ms=5, mec="white", mew=0.8)
        ax.plot(goal[1], goal[0], "*", color="white", ms=18, mec=INK, mew=1.2)
    fig.savefig(out / f"paths_{tag}.png", dpi=110, facecolor="white")
    plt.close(fig)


def fig_minima(out, free, guide, true, tag):
    lm = local_minima(guide, free)
    fig, axs = plt.subplots(1, 2, figsize=(12.5, 5.2), constrained_layout=True)
    show_map(axs[0], free, f"誘導の局所最小 {lm.sum():,} セル（自由セルの {100*lm.sum()/free.sum():.1f}%）")
    ys, xs = np.nonzero(lm)
    axs[0].scatter(xs, ys, s=1.2, c=ACCENT, alpha=0.5, linewidths=0)
    lm_true = local_minima(np.where(free, true, np.nan), free)
    show_map(axs[1], free, f"厳密解の局所最小 {lm_true.sum():,} セル")
    ys, xs = np.nonzero(lm_true)
    axs[1].scatter(xs, ys, s=1.2, c=ACCENT2, alpha=0.6, linewidths=0)
    fig.savefig(out / f"minima_{tag}.png", dpi=110, facecolor="white")
    plt.close(fig)


def fig_policy(out, free, lg, guide, tag, half=170):
    """Where the guide stalls, and which way the model jumps to get out.
    The crop is centred on the densest cluster of guide local minima — the cells
    the model is actually consulted on."""
    lm = local_minima(guide, free)
    k = 64
    H, W = free.shape
    dens = lm[:H // k * k, :W // k * k].reshape(H // k, k, W // k, k).sum(axis=(1, 3))
    cy, cx = np.unravel_index(dens.argmax(), dens.shape)
    cy, cx = cy * k + k // 2, cx * k + k // 2
    y0, y1 = max(0, cy - half), min(H, cy + half)
    x0, x1 = max(0, cx - half), min(W, cx + half)
    f = free[y0:y1, x0:x1]
    sub_lm = lm[y0:y1, x0:x1]
    k_idx = lg[:, y0:y1, x0:x1].argmax(0)
    ang = np.array([np.arctan2(OFFSETS[i][0], OFFSETS[i][1]) for i in range(len(OFFSETS))])[k_idx]

    fig, axs = plt.subplots(1, 3, figsize=(15.5, 5.6), constrained_layout=True)
    show_map(axs[0], f, "窓の占有格子")
    g = np.ma.masked_invalid(np.where(f, guide[y0:y1, x0:x1], np.nan))
    axs[1].imshow(g, cmap=VMAP, origin="lower", interpolation="nearest")
    ys, xs = np.nonzero(sub_lm)
    axs[1].scatter(xs, ys, s=2.5, c=ACCENT, alpha=0.75, linewidths=0)
    _style(axs[1], f"誘導の値と、その局所最小 {sub_lm.sum():,} セル")
    axs[2].imshow(np.where(f, 1.0, 0.0), cmap=ListedColormap([INK, "#f6f5f2"]),
                  origin="lower", interpolation="nearest")
    m = np.ma.masked_where(~(f & sub_lm), ang)
    im = axs[2].imshow(m, cmap="twilight", origin="lower", vmin=-np.pi, vmax=np.pi)
    step = max(1, (y1 - y0) // 26)
    yy, xx = np.mgrid[0:y1 - y0:step, 0:x1 - x0:step]
    dy = np.array([OFFSETS[i][0] for i in range(len(OFFSETS))])[k_idx][::step, ::step]
    dx = np.array([OFFSETS[i][1] for i in range(len(OFFSETS))])[k_idx][::step, ::step]
    mask = (f & sub_lm)[::step, ::step]
    axs[2].quiver(xx[mask], yy[mask], dx[mask], dy[mask], color=INK,
                  scale=34, width=0.006, alpha=0.95)
    _style(axs[2], "モデルが出す脱出ジャンプの向き")
    fig.colorbar(im, ax=axs[2], fraction=0.046, shrink=0.88, label="方位 [rad]")
    fig.savefig(out / f"policy_{tag}.png", dpi=110, facecolor="white")
    plt.close(fig)


def fig_noexact(out, free, goal, true, guide, tag):
    """What the model produces with no exact solve anywhere, next to the guide."""
    from .multiscale import load_model as _lm  # noqa: F401
    fig, axs = plt.subplots(1, 3, figsize=(15.5, 5.0), constrained_layout=True)
    vmax = float(np.nanmax(true))
    panels = [(true, "厳密 VI"), (guide, "粗い厳密 VI を拡大（誘導）"), (None, "モデルのみ（厳密 VI なし）")]
    return fig, axs, panels, vmax


def fig_upsample(out, free, goal, true):
    """Why bilinear: nearest leaves 2×2 plateaus that greedy descent cannot cross."""
    fc = pool_free_conservative(pool_free_conservative(free))
    g = (goal[0] >> 2, goal[1] >> 2)
    fc[g] = True
    v, _ = vi.solve(fc, g, res=vi.RES_M * 4, action_scale=4)
    v = v * 4
    v[~fc] = np.nan
    near = np.repeat(np.repeat(np.repeat(np.repeat(v, 2, 0), 2, 1), 2, 0), 2, 1)[:free.shape[0], :free.shape[1]]
    near[~free] = np.nan
    bil = upsample(upsample(v, (free.shape[0] // 2, free.shape[1] // 2)), free.shape)
    bil[~free] = np.nan
    gm = true == 0
    rng = np.random.default_rng(3)
    ss = [maps.pick_free(rng, free & np.isfinite(true)) for _ in range(12)]
    res = {}
    fig, axs = plt.subplots(1, 2, figsize=(12.5, 5.6), constrained_layout=True)
    for ax, (fld, name) in zip(axs, [(near, "最近傍で拡大"), (bil, "双一次で拡大")]):
        ok = sum(rollout.greedy(fld, s, gm)[0] for s in ss)
        res[name] = ok / len(ss)
        show_value(ax, fld, free, f"{name} — 到達 {ok}/{len(ss)}", float(np.nanmax(true)))
        for s in ss:
            o, p = rollout.greedy(fld, s, gm)
            p = np.asarray(p)
            ax.plot(p[:, 1], p[:, 0], "-", lw=1.6, color=ACCENT if o else "#7a7a7a", alpha=0.9)
    fig.savefig(out / "upsample.png", dpi=110, facecolor="white")
    plt.close(fig)
    return res


def fig_examples(out, d, model, n=3, top=128):
    fig, axs = plt.subplots(n, 4, figsize=(15.5, 3.9 * n), constrained_layout=True)
    rng = np.random.default_rng(0)
    for r in range(n):
        i = len(d["free"]) - 1 - r
        free, goal, true = d["free"][i], tuple(int(v) for v in d["goal"][i]), d["value"][i]
        lg, guide, _ = solve_map(model, free, goal, top=top)
        gm = true == 0
        m = free & np.isfinite(true)
        ss = [maps.pick_free(rng, m) for _ in range(6)]
        show_map(axs[r][0], free, f"{d['kind'][i]} {free.shape[1]}×{free.shape[0]}")
        axs[r][0].plot(goal[1], goal[0], "*", color=ACCENT, ms=15, mec="white", mew=1)
        vmax = float(np.nanmax(true))
        show_value(axs[r][1], true, free, "厳密 VI", vmax)
        show_value(axs[r][2], guide, free, "誘導（粗い厳密 VI）", vmax)
        show_value(axs[r][3], true, free, "経路: 誘導のみ / 本手法", vmax)
        for s in ss:
            o1, p1 = rollout.greedy(guide, s, gm)
            o2, p2, _ = follow_hybrid(lg, guide, free, s, gm)
            for p, o, c in ((p1, o1, ACCENT2), (p2, o2, ACCENT)):
                p = np.asarray(p)
                axs[r][3].plot(p[:, 1], p[:, 0], "-", lw=1.6, color=c if o else "#8a8a8a",
                               alpha=0.9 if o else 0.4)
    fig.savefig(out / "examples256.png", dpi=105, facecolor="white")
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--weights", type=Path, default=Path("out/ms2.pt"))
    ap.add_argument("--truth", type=Path, default=Path("out/tsudanuma_s2.bin"))
    ap.add_argument("--yaml", type=Path, default=Path("../assets/map_tsudanuma.yaml"))
    ap.add_argument("--scale", type=int, default=2)
    ap.add_argument("--top", type=int, default=768)
    ap.add_argument("--vi-ms", type=float, default=18143.0)
    ap.add_argument("--maps256", type=Path, default=Path("out/maps256.npz"))
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    print("font:", _use_japanese_font())
    plt.rcParams["axes.unicode_minus"] = False
    model = load_model(a.weights)
    stats = {}

    free = load_real(a.yaml, a.scale)
    true = vi.read_dump(a.truth)
    gy, gx = np.argwhere(true == 0).mean(axis=0).round().astype(int)
    goal = (int(gy), int(gx))
    t0 = time.perf_counter()
    lg, guide, per = solve_map(model, free, goal, top=a.top)
    ours_ms = (time.perf_counter() - t0) * 1e3
    stats["tsudanuma"] = dict(shape=list(free.shape), vi_ms=a.vi_ms, ours_ms=ours_ms,
                              levels=[round(p) for p in per], free=int(free.sum()),
                              reachable=int(np.isfinite(true).sum()), max_s=float(np.nanmax(true)))

    gm = true == 0
    m = free & np.isfinite(true)
    rng = np.random.default_rng(1)
    starts = [maps.pick_free(rng, m) for _ in range(20)]
    ok_g = ok_h = 0
    esc = 0
    ratios_g, ratios_h = [], []
    for s in starts:
        rt, pt = rollout.greedy(true, s, gm)
        L = max(1e-9, rollout.length(pt))
        og, pg = rollout.greedy(guide, s, gm)
        oh, ph, e = follow_hybrid(lg, guide, free, s, gm)
        ok_g += og
        ok_h += oh
        esc += e
        if og:
            ratios_g.append(rollout.length(pg) / L)
        if oh:
            ratios_h.append(rollout.length(ph) / L)
    stats["tsudanuma"].update(guide_success=ok_g / len(starts), hybrid_success=ok_h / len(starts),
                              escapes=esc, guide_ratio=float(np.mean(ratios_g)),
                              hybrid_ratio=float(np.mean(ratios_h)),
                              rel_err=float(np.nanmean(np.abs(guide[m] - true[m]) / (true[m] + 1))))

    fig_fields(a.out, free, goal, true, guide, True, "tsudanuma", a.vi_ms, ours_ms)
    fig_paths(a.out, free, goal, true, guide, lg, "tsudanuma", starts[:8])
    fig_minima(a.out, free, guide, true, "tsudanuma")
    fig_policy(a.out, free, lg, guide, "tsudanuma")
    stats["upsample"] = fig_upsample(a.out, free, goal, true)

    d = np.load(a.maps256)
    fig_examples(a.out, d, model)
    (a.out / "stats.json").write_text(json.dumps(stats, indent=2, ensure_ascii=False))
    print(json.dumps(stats, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
