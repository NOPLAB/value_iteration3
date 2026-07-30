#!/usr/bin/env python3
"""anytime_bench の CSV から anytime プロファイル図を作る。

左: 解の正しさ（V* と一致した自由状態の割合）vs 経過時間
右: 貪欲方策がゴールに到達できた開始姿勢の割合 vs 経過時間

本家は「行動はすぐ引けるが正しくない」、提案は「行動が引ける範囲は狭いが
その範囲では厳密に最適」という対比を 1 枚で示すのが狙い。

usage: anytime_plot.py <csv> <out.eps> [--title T]
"""
import sys
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = "reference"
PROP = "sparse"
LABEL = {BASE: "Reference (full sweeps)", PROP: "Proposed (active set)"}
STYLE = {BASE: dict(color="#c0392b", marker="o", ms=3, lw=1.4),
         PROP: dict(color="#1f5fa8", marker="s", ms=3, lw=1.4)}


def load(path):
    rows = {BASE: [], PROP: []}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if r["solver"] in rows:
                rows[r["solver"]].append(r)
    for k in rows:
        rows[k].sort(key=lambda r: float(r["t_sec"]))
    return rows


def series(rows, key):
    t = [float(r["t_sec"]) for r in rows]
    v = [float(r[key]) * 100.0 for r in rows]
    return t, v


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    csv_path, out_path = sys.argv[1], sys.argv[2]
    title = None
    if "--title" in sys.argv:
        title = sys.argv[sys.argv.index("--title") + 1]

    rows = load(csv_path)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 2.9))

    for k in (BASE, PROP):
        if not rows[k]:
            continue
        ax1.plot(*series(rows[k], "policy_agree"), label=LABEL[k], **STYLE[k])
        ax2.plot(*series(rows[k], "reach_rate"), label=LABEL[k], **STYLE[k])
        if k == BASE:
            # 「行動はすぐ引けるが正しくない」を示す補助線。
            t, v = series(rows[k], "frac_policy")
            ax1.plot(t, v, color=STYLE[k]["color"], lw=1.0, ls=":",
                     label="Reference: an action exists")

    ax1.set_xlabel("wall-clock time [s]")
    ax1.set_ylabel("states with the optimal action [%]")
    ax1.set_ylim(-3, 103)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=6.0, loc="lower right")

    ax2.set_xlabel("wall-clock time [s]")
    ax2.set_ylabel("start poses reaching the goal [%]")
    ax2.set_ylim(-3, 103)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=6.0, loc="lower right")

    if title:
        fig.suptitle(title, fontsize=9)
    fig.tight_layout()
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, format=Path(out_path).suffix.lstrip("."), dpi=300,
                bbox_inches="tight")
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
