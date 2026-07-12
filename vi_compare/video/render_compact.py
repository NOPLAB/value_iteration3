#!/usr/bin/env python3
"""bench_map の `frontier2d_sparse_compact` ディスク出力を value 図にする。

既存の VI value オーバーレイ図（`vi_compare/benches/tsudanuma/plot.py` の
`fig_map_overlay`）と同じフォーマットで描く:
  - 背景 = グレー占有マップ（free=0.92 / obstacle=0.18）
  - value = turbo 線形（vmin=0, vmax=到達セルの P90）を alpha 0.92 で重畳
  - ゴール ★(lime) ・ロボット始点 ●(magenta) ・最適経路（白線）

入力（`bench_map --solver frontier2d_sparse_compact --compact-out-dir <dir>
--dump-value <dir>/value_field.f32` が出すもの）:

  value_field.f32  : header i32(ow), i32(oh) + ow*oh の f32（θ最小の value[秒]、
                     到達不能=NaN、索引 v[ix + ow*iy] = world bottom-up）。

占有マップ背景は元の PGM を value グリッド解像度（`--scale` で down-sample）へ
bench_map と同じ規則（保守プーリング＋上下反転）でプールして描く。`--map` 省略時は
壁を描けないため一様グレー背景にフォールバックする。

使い方:
  # tsudanuma を scale 5 で解いた compact 出力を、元マップ重畳で描く
  python3 render_compact.py <out_dir> --map assets/map_tsudanuma.yaml --scale 5

  # 始点を与えると最適経路（V 降下）と始点マーカも描く
  python3 render_compact.py <out_dir> --map ... --scale 5 --start-x 35.2 --start-y 162.8
"""
import argparse
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ---------- inputs ----------
def load_value_field(path):
    """value_field.f32 (header i32 ow, i32 oh + ow*oh f32) を (oh, ow) bottom-up で返す。"""
    with open(path, "rb") as fh:
        ow, oh = (int(v) for v in np.fromfile(fh, dtype="<i4", count=2))
        vf = np.fromfile(fh, dtype="<f4", count=ow * oh).reshape(oh, ow)
    return vf, ow, oh


def parse_yaml(path):
    """map YAML の必要フィールドだけ拾う簡易パーサ。"""
    cfg = {"resolution": 0.05, "origin": [0.0, 0.0, 0.0],
           "occupied_thresh": 0.65, "free_thresh": 0.196, "negate": 0, "image": None}
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            k, v = (s.strip() for s in line.split(":", 1))
            if k == "image":
                cfg["image"] = v
            elif k == "origin":
                cfg["origin"] = [float(x) for x in v.strip("[]").split(",")]
            elif k in ("resolution", "occupied_thresh", "free_thresh"):
                cfg[k] = float(v)
            elif k == "negate":
                cfg["negate"] = int(v)
    return cfg


def load_pgm(path):
    """P5 PGM を (h, w) uint8 (top-down) で返す。"""
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5", "not a binary PGM (P5)"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        int(f.readline())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return data


def build_occupancy(pixels, scale, cfg, unknown_as_obstacle):
    """PGM(top-down) を value グリッド解像度 (oh, ow) bottom-up の occupied(bool) にプール。

    bench_map::build_occupancy と同じ:
      occ_prob = (negate? p : 255-p)/255、obstacle: occ>=occupied_thresh、
      free: occ<free_thresh、それ以外は unknown。出力セルはブロック内に obstacle
      (unknown_as_obstacle なら unknown も) が 1 つでもあれば blocked（保守プーリング）。
      上下反転で出力 row iy=0 を world y=origin_y に合わせる。
    """
    h, w = pixels.shape
    p = pixels.astype(np.float64)
    occ_prob = (p / 255.0) if cfg["negate"] else ((255.0 - p) / 255.0)
    obstacle = occ_prob >= cfg["occupied_thresh"]
    if unknown_as_obstacle:
        unknown = (occ_prob >= cfg["free_thresh"]) & (occ_prob < cfg["occupied_thresh"])
        obstacle = obstacle | unknown
    obstacle = obstacle[::-1, :]  # top-down -> bottom-up (world y up)
    ow = -(-w // scale)  # ceil
    oh = -(-h // scale)
    padded = np.zeros((oh * scale, ow * scale), dtype=bool)
    padded[:h, :w] = obstacle
    return padded.reshape(oh, scale, ow, scale).any(axis=(1, 3)), ow, oh


# ---------- optimal path via V descent (plot.py と同じ手法) ----------
def _clear_los(v_img, r0, c0, r1, c1):
    n = int(max(abs(r1 - r0), abs(c1 - c0))) + 1
    rs = np.round(np.linspace(r0, r1, n)).astype(int)
    cs = np.round(np.linspace(c0, c1, n)).astype(int)
    return bool(np.all(np.isfinite(v_img[rs, cs])))


def descend_path(v_img, sr, sc, gr, gc, max_steps=100000):
    """V のコスト到達場を降下してゴールまで経路を辿る（壁を横切らない line-of-sight）。"""
    h, w = v_img.shape
    path = [(sr, sc)]
    r, c = sr, sc
    visited = {(r, c)}
    for _ in range(max_steps):
        if abs(r - gr) <= 2 and abs(c - gc) <= 2:
            path.append((gr, gc))
            break
        cur = v_img[r, c]
        nr, nc = r, c
        for R in (2, 3, 4, 6, 8):
            r0, r1 = max(0, r - R), min(h, r + R + 1)
            c0, c1 = max(0, c - R), min(w, c + R + 1)
            sub = v_img[r0:r1, c0:c1]
            mask = np.isfinite(sub) & (sub < cur)
            if not mask.any():
                continue
            order = np.argsort(np.where(mask, sub, np.inf), axis=None)
            for k in order:
                yy, xx = np.unravel_index(k, sub.shape)
                if not mask[yy, xx]:
                    break
                cand = (r0 + yy, c0 + xx)
                if cand in visited:
                    continue
                if _clear_los(v_img, r, c, cand[0], cand[1]):
                    nr, nc = cand
                    break
            if (nr, nc) != (r, c):
                break
        if (nr, nc) == (r, c):
            break  # truly stuck
        r, c = nr, nc
        visited.add((r, c))
        path.append((r, c))
    return np.array(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir", help="bench_map --compact-out-dir に渡したディレクトリ")
    ap.add_argument("--map", default=None, help="占有マップ背景用の YAML か PGM（推奨: 解いた時の YAML）")
    ap.add_argument("--scale", type=int, default=1, help="bench_map に渡した down-sample 係数（既定 1）")
    ap.add_argument("--unknown", choices=("obstacle", "free"), default="obstacle",
                    help="map_server の unknown(灰)セルの扱い（既定 obstacle、bench_map と合わせる）")
    ap.add_argument("--nt", type=int, default=60, help="θ セル数（タイトルの状態数表示用、既定 60）")
    ap.add_argument("--vmax-pct", type=float, default=90.0, help="value カラースケール上限の百分位（既定 90）")
    ap.add_argument("--px-per-cell", type=float, default=1.0,
                    help="value グリッド1セルあたりの出力ピクセル数（既定 1.0=地図と等倍）。"
                         "大きいほど高精細・大きい画像。文字/マーカは図サイズに比例して自動拡大。")
    ap.add_argument("--goal-x", type=float, default=None, help="ゴール world X[m]（既定: V 最小セル）")
    ap.add_argument("--goal-y", type=float, default=None, help="ゴール world Y[m]（既定: V 最小セル）")
    ap.add_argument("--start-x", type=float, default=None, help="始点 world X[m]（与えると経路と始点を描く）")
    ap.add_argument("--start-y", type=float, default=None, help="始点 world Y[m]")
    ap.add_argument("--out", default=None, help="出力 PNG（既定 <out_dir>/compact_viz.png）")
    ap.add_argument("--title", default=None, help="図タイトル上書き")
    args = ap.parse_args()

    vf_path = f"{args.out_dir}/value_field.f32"
    if not os.path.exists(vf_path):
        sys.exit(f"error: {vf_path} がありません。bench_map に --dump-value {vf_path} を付けて実行してください。")
    vf, ow, oh = load_value_field(vf_path)  # (oh, ow) bottom-up
    reach_mask = np.isfinite(vf)
    n_reach = int(reach_mask.sum())
    if n_reach == 0:
        sys.exit("error: 到達可能セルが 0 です（ゴールが孤立しているか field が空）。")

    # --- 占有マップ背景（value グリッド解像度へプール） ---
    cfg = None
    res_grid = None
    free_bu = None  # bottom-up free(bool)
    if args.map:
        if args.map.endswith((".yaml", ".yml")):
            cfg = parse_yaml(args.map)
            pgm_path = os.path.join(os.path.dirname(os.path.abspath(args.map)), cfg["image"])
        else:
            cfg = {"resolution": 0.05, "origin": [0.0, 0.0, 0.0],
                   "occupied_thresh": 0.65, "free_thresh": 0.196, "negate": 0}
            pgm_path = args.map
        res_grid = cfg["resolution"] * args.scale
        occ_bu, mow, moh = build_occupancy(load_pgm(pgm_path), args.scale, cfg,
                                           args.unknown == "obstacle")
        if (mow, moh) != (ow, oh):
            print(f"WARN: map grid {mow}x{moh} != value grid {ow}x{oh} "
                  f"(--scale {args.scale} 不一致?)。背景はベストエフォートで描画。", file=sys.stderr)
            # value グリッドに合わせて切り詰め/パディング
            occ_fixed = np.ones((oh, ow), dtype=bool)
            hh, ww = min(oh, moh), min(ow, mow)
            occ_fixed[:hh, :ww] = occ_bu[:hh, :ww]
            occ_bu = occ_fixed
        free_bu = ~occ_bu

    # --- 表示は plot.py に合わせ top-down (origin='upper') ---
    v_img = vf[::-1, :]                  # bottom-up -> top-down
    free_img = free_bu[::-1, :] if free_bu is not None else None

    fin = vf[reach_mask]
    vmax = float(np.percentile(fin, args.vmax_pct))
    if not np.isfinite(vmax) or vmax <= 0:
        vmax = float(np.nanmax(fin))
    print(f"grid {ow}x{oh}x{args.nt}  reachable {n_reach:,} cells  "
          f"V p50={np.percentile(fin,50):.1f}s p{args.vmax_pct:g}={vmax:.1f}s max={np.nanmax(fin):.1f}s")

    # --- goal / start を image(row,col) top-down へ ---
    def world_to_img(wx, wy):
        ix = (wx - cfg["origin"][0]) / res_grid
        iy = (wy - cfg["origin"][1]) / res_grid
        return (oh - 0.5) - iy, ix - 0.5  # (row, col)

    if args.goal_x is not None and args.goal_y is not None and cfg is not None:
        gr, gc = world_to_img(args.goal_x, args.goal_y)
    else:
        # V 最小セル（ゴールは V≈0）を bottom-up で見つけ top-down へ
        flat = np.where(reach_mask, vf, np.inf)
        giy, gix = np.unravel_index(np.argmin(flat), flat.shape)
        gr, gc = oh - 1 - giy, gix

    path = None
    start_rc = None
    if args.start_x is not None and args.start_y is not None and cfg is not None:
        sr, sc = world_to_img(args.start_x, args.start_y)
        sr, sc = int(round(sr)), int(round(sc))
        sr = min(max(sr, 0), oh - 1)
        sc = min(max(sc, 0), ow - 1)
        if not np.isfinite(v_img[sr, sc]):  # 近傍の到達セルへスナップ
            ys, xs = np.where(np.isfinite(v_img))
            d = (ys - sr) ** 2 + (xs - sc) ** 2
            sr, sc = int(ys[d.argmin()]), int(xs[d.argmin()])
        start_rc = (sr, sc)
        path = descend_path(v_img, sr, sc, int(round(gr)), int(round(gc)))

    # --- 描画: 図サイズを value グリッド解像度に合わせる（既定 1px/cell）。値場を縮小せず
    #     ネイティブ解像度で描き、文字・マーカは図の拡大率 fs に比例させて読めるサイズを保つ。 ---
    DPI = 100
    pxc = args.px_per_cell
    fig_w_in = ow * pxc / DPI * 1.18  # +余白（右の colorbar・軸ラベル）
    fig_h_in = oh * pxc / DPI * 1.12  # +余白（上の title・軸ラベル）
    fs = max(fig_w_in / 11.0, 1.0)    # 旧 11in レイアウト基準に対する文字/マーカ拡大率
    print(f"figure: {fig_w_in*DPI:.0f}x{fig_h_in*DPI:.0f}px (px/cell={pxc:g}, font scale x{fs:.1f})")
    fig, ax = plt.subplots(figsize=(fig_w_in, fig_h_in), dpi=DPI)
    if free_img is not None:
        ax.imshow(np.where(free_img, 0.92, 0.18), cmap="gray", vmin=0, vmax=1,
                  origin="upper", interpolation="nearest")
    else:
        ax.set_facecolor("0.85")
        print("note: --map 未指定。壁は描けません（一様グレー背景）。", file=sys.stderr)
    hm = ax.imshow(np.ma.masked_invalid(v_img), cmap="turbo", origin="upper",
                   alpha=0.92, vmin=0, vmax=vmax, interpolation="nearest")
    if path is not None and len(path) > 1:
        ax.plot(path[:, 1], path[:, 0], "-", color="white", lw=1.6 * fs, alpha=0.9,
                label="optimal path (V descent)")
    if start_rc is not None:
        sv = float(v_img[start_rc])
        ax.plot(start_rc[1], start_rc[0], "o", color="magenta", ms=13 * fs, mec="black",
                mew=1.5 * fs, label=f"robot start (V={sv:.0f}s)", zorder=5)
    ax.plot(gc, gr, "*", color="lime", ms=22 * fs, mec="black", mew=1.5 * fs,
            label="goal", zorder=5)

    cb = fig.colorbar(hm, ax=ax, shrink=0.82)
    cb.set_label(f"cost-to-go V* [s] (min over θ, P{args.vmax_pct:g} scale)", fontsize=12 * fs)
    cb.ax.tick_params(labelsize=10 * fs)

    res_str = f", {res_grid:.2f} m/cell" if res_grid is not None else ""
    nstates = ow * oh * args.nt
    title = args.title or (
        f"compact mapped output ({ow}×{oh}×{args.nt} = {nstates/1e6:.0f}M states{res_str})"
        f"\nVI value function V* (frontier2d_sparse_compact) · reachable: {n_reach:,} cells")
    ax.set_title(title, fontsize=14 * fs)
    ax.set_xlabel("x [cell]", fontsize=12 * fs)
    ax.set_ylabel("y [cell]", fontsize=12 * fs)
    ax.tick_params(labelsize=10 * fs)
    if path is not None or start_rc is not None:
        ax.legend(loc="upper right", fontsize=9 * fs)

    fig.tight_layout()
    out_png = args.out or f"{args.out_dir}/compact_viz.png"
    fig.savefig(out_png, dpi=DPI)
    print("wrote", out_png)


if __name__ == "__main__":
    main()
