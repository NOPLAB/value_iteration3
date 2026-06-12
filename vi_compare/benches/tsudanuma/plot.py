#!/usr/bin/env python3
"""4者速度比較プロット + lite マップへの結果オーバーレイ (matplotlib)。
入力 (RES=vi_compare/results/tsudanuma):
  sweep_vi_rs.csv, sweep_ros1_nodeB.csv, sweep_ros1_b2.csv, sweep_ros1_nodeA.csv
  lite/map_tsudanuma_lite.pgm
出力: lite/fig_speed_compare.png, lite/fig_map_overlay.png
マップオーバーレイは「ゴールからの測地距離(コスト到達場の代理)」を free セルに heatmap 表示。
"""
import csv, os, sys
from collections import deque
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RES = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__)) + '/../../results/tsudanuma'
RES = os.path.abspath(RES)
RESO = 0.15  # m/cell


def read(path):
    out = {}
    try:
        with open(path) as f:
            for r in csv.DictReader(f):
                out[int(r['m'])] = r
    except FileNotFoundError:
        pass
    return out


def load_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        int(f.readline())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape((h, w))
    return data  # top-down image; 255=free, 0=obstacle


# ---------- Fig 1: 4-way speed comparison ----------
def speed_fig():
    vi = read(f'{RES}/sweep_vi_rs.csv')
    nodes = [
        ('frontier2d_par (vi_rs)', vi, 'total_s', 'converged', None, 's-', '#1f77b4'),
        ('Node B (full-sweep)', read(f'{RES}/sweep_ros1_nodeB.csv'), 'elapsed_sec', 'converged', 'resid_s', 'o--', '#d62728'),
        ('Node B-2 (partitioned)', read(f'{RES}/sweep_ros1_b2.csv'), 'elapsed_sec', 'converged', 'resid_s', '^--', '#ff7f0e'),
        ('Node A (online)', read(f'{RES}/sweep_ros1_nodeA.csv'), 'elapsed_sec', 'converged', 'resid_s', 'v--', '#9467bd'),
    ]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))

    # 左: 収束時間 vs m
    for name, d, tcol, ccol, rcol, style, col in nodes:
        if not d:
            continue
        ms = sorted(d)
        conv_m = [m for m in ms if str(d[m][ccol]).lower() in ('y', 'true', '1')]
        ncon_m = [m for m in ms if m not in conv_m]
        if conv_m:
            ax1.plot(conv_m, [float(d[m][tcol]) for m in conv_m], style, color=col, label=name, ms=7)
        # 未収束は cap(300s) に中空マーカ
        for m in ncon_m:
            ax1.plot(m, float(d[m][tcol]), style[0], color=col, mfc='none', ms=9)
    ax1.axhspan(300, 330, color='gray', alpha=0.12)
    ax1.text(2, 312, '300 s cap (ros1: not converged)', fontsize=8, color='gray')
    ax1.set_xlabel('threads m'); ax1.set_ylabel('time to converge (dV<0.1s) [s]')
    ax1.set_title('Convergence time (lite, paper-scale)'); ax1.grid(alpha=.3); ax1.legend(fontsize=8)
    ax1.set_xticks([1, 2, 4, 6, 8, 10, 12, 16])

    # 右: 本家 到達残差 vs m (log)
    ax2.axhline(0.1, color='green', ls=':', label='target dV=0.1s')
    ax2.axhline(1.0, color='gray', ls='--', alpha=.5, label='≈1 step (1s) floor')
    for name, d, tcol, ccol, rcol, style, col in nodes:
        if not d or rcol is None:
            continue
        ms = sorted(d)
        ax2.plot(ms, [float(d[m][rcol]) for m in ms], style, color=col, label=name, ms=7)
    ax2.set_yscale('log'); ax2.set_xlabel('threads m'); ax2.set_ylabel('residual dV reached [s]')
    ax2.set_title('ros1 nodes: residual floor (never <0.1s)'); ax2.grid(alpha=.3, which='both')
    ax2.legend(fontsize=8); ax2.set_xticks([4, 8, 16])
    fig.tight_layout()
    out = f'{RES}/lite/fig_speed_compare.png'
    fig.savefig(out, dpi=140)
    print('wrote', out)


# ---------- Fig 1b: house map speed comparison (paper Fig.21/22 setup) ----------
def house_fig():
    """論文 Fig.21/22 の実セットアップ (house.pgm 0.05m, 本家が収束する規模) での
    収束時間 vs m と速度向上率 vs m。goal=(6.0,-2.0,90deg)=round_trip 実験の先頭ゴール。"""
    hdir = os.path.abspath(f'{RES}/../house')
    ros1 = read(f'{hdir}/sweep_ros1_house.csv')
    virs = read(f'{hdir}/sweep_vi_rs_house.csv')
    vuns = read(f'{hdir}/sweep_vi_rs_unsafe_house.csv')
    vsp = read(f'{hdir}/sweep_vi_rs_sparse_house.csv')
    if not (ros1 and virs):
        print('house plot skipped (missing CSVs in', hdir, ')')
        return
    ms_r = sorted(ros1)
    ms_v = sorted(virs)
    t_r = [float(ros1[m]['elapsed_sec']) for m in ms_r]
    t_v = [float(virs[m]['total_s']) for m in ms_v]
    ms_u = sorted(vuns) if vuns else []
    t_u = [float(vuns[m]['total_s']) for m in ms_u]
    ms_s = sorted(vsp) if vsp else []
    t_s = [float(vsp[m]['total_s']) for m in ms_s]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
    ax1.plot(ms_r, t_r, 'o--', color='#d62728', label='ROS1 vi_node Node B (dV<0.1s)', ms=7)
    ax1.plot(ms_v, t_v, 's-', color='#1f77b4', label='vi_rs frontier2d_par (exact fixpoint)', ms=7)
    if ms_u:
        ax1.plot(ms_u, t_u, '^-', color='#2ca02c',
                 label='vi_rs frontier2d_par_unsafe (async G-S, exact fixpoint)', ms=7)
    if ms_s:
        ax1.plot(ms_s, t_s, 'D-', color='#9467bd',
                 label='vi_rs frontier2d_sparse (+fused layout, theta-mask)', ms=6)
    ax1.set_xlabel('threads m'); ax1.set_ylabel('time to converge [s]')
    ax1.set_title('house (384x384x60, 0.05m) - convergence time vs m\n(paper Fig.21 setup; goal (6.0,-2.0,90deg))')
    ax1.grid(alpha=.3); ax1.legend(fontsize=8); ax1.set_xticks(ms_r); ax1.set_ylim(bottom=0)

    sp_r = [t_r[0] / t for t in t_r]
    sp_v = [t_v[0] / t for t in t_v]
    ax2.plot(ms_r, ms_r, ':', color='gray', label='ideal (linear)')
    ax2.plot(ms_r, sp_r, 'o--', color='#d62728', label='ROS1 Node B', ms=7)
    ax2.plot(ms_v, sp_v, 's-', color='#1f77b4', label='vi_rs frontier2d_par', ms=7)
    if ms_u:
        sp_u = [t_u[0] / t for t in t_u]
        ax2.plot(ms_u, sp_u, '^-', color='#2ca02c', label='vi_rs frontier2d_par_unsafe', ms=7)
    if ms_s:
        sp_s = [t_s[0] / t for t in t_s]
        ax2.plot(ms_s, sp_s, 'D-', color='#9467bd', label='vi_rs frontier2d_sparse', ms=6)
    ax2.set_xlabel('threads m'); ax2.set_ylabel('speed rate T(1)/T(m)')
    ax2.set_title('speed-up rate vs m (paper Fig.22 style)')
    ax2.grid(alpha=.3); ax2.legend(fontsize=8); ax2.set_xticks(ms_r)
    fig.tight_layout()
    out = f'{hdir}/fig_house_speed.png'
    fig.savefig(out, dpi=140)
    print('wrote', out)
    for m, tr in zip(ms_r, t_r):
        tv = dict(zip(ms_v, t_v)).get(m)
        tu = dict(zip(ms_u, t_u)).get(m)
        print(f'  m={m:2d}: ros1={tr:7.2f}s  vi_rs={tv if tv is None else round(tv,2)}s'
              f'  unsafe={tu if tu is None else round(tu,2)}s')


# ---------- Fig 2: map overlay (TRUE value function V from vi_rs) ----------
def load_value(path):
    with open(path, 'rb') as f:
        ow = int(np.frombuffer(f.read(4), '<i4')[0])
        oh = int(np.frombuffer(f.read(4), '<i4')[0])
        v = np.frombuffer(f.read(ow * oh * 4), '<f4').reshape(oh, ow)  # world bottom-up
    return v  # v[iy, ix], seconds, NaN=unreachable


def load_path(path, img_h):
    """bench_map --dump-path の方策追従経路 ([i32 n][f32 x][f32 y]*n, world m) を
    image (row,col) へ変換して返す。本家 vi_node decision() と同じ closed-loop
    方策追従 (10Hz cmd_vel 積分) で生成された軌跡。"""
    if not os.path.exists(path):
        return None
    with open(path, 'rb') as f:
        n = int(np.frombuffer(f.read(4), '<i4')[0])
        xy = np.frombuffer(f.read(n * 8), '<f4').reshape(n, 2)
    cols = xy[:, 0] / RESO - 0.5
    rows = (img_h - 0.5) - xy[:, 1] / RESO
    return np.stack([rows, cols], axis=1)


def _clear_los(v_img, r0, c0, r1, c1):
    """(r0,c0)-(r1,c1) 線分上のセルが全て finite-V (free) か = 壁を横切らないか。"""
    n = int(max(abs(r1 - r0), abs(c1 - c0))) + 1
    rs = np.round(np.linspace(r0, r1, n)).astype(int)
    cs = np.round(np.linspace(c0, c1, n)).astype(int)
    return bool(np.all(np.isfinite(v_img[rs, cs])))


def descend_path(v_img, sr, sc, gr, gc, max_steps=100000):
    """V のコスト到達場を降下してゴールまで経路を辿る。VI の実方策は前進2セル等なので
    8近傍では「アクション間セル」で局所停止する。改善が出るまで半径を 2→8 に適応拡大し、
    かつ移動先までの線分が free セルだけを通る (line-of-sight, 壁を横切らない) ものだけ採用。"""
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


def map_fig():
    img = load_pgm(f'{RES}/lite/map_tsudanuma_lite.pgm')  # top-down, 255=free
    h, w = img.shape
    free = img == 255
    # ベンチマークと同一条件 (penalty=100000) の V を使用。
    v_grid = load_value(f'{RES}/lite/lite_value_g2.bin')        # world bottom-up
    v_img = v_grid[::-1, :]                                 # flip to top-down (match PGM)
    reachable = int(np.isfinite(v_img).sum())
    # goal: world grid (ix=382,iy=440) -> image (row=h-1-440, col=382)
    gr, gc = h - 1 - 440, 382
    # robot start: world grid (ix=0,iy=0) corner -> image (row=h-1-0, col=0)
    sr, sc = h - 1 - 0, 0
    if not np.isfinite(v_img[sr, sc]):  # 念のため到達 free セルへスナップ
        ys, xs = np.where(np.isfinite(v_img))
        d = (ys - sr) ** 2 + (xs - sc) ** 2
        sr, sc = int(ys[d.argmin()]), int(xs[d.argmin()])
    start_v = float(v_img[sr, sc])
    path = load_path(f'{RES}/lite/lite_path.bin', h)
    path_src = 'policy rollout'
    if path is None:
        path = descend_path(v_img, sr, sc, gr, gc)
        path_src = 'V descent'

    reached = bool(abs(path[-1][0]-gr)<=3 and abs(path[-1][1]-gc)<=3)
    fin = v_img[np.isfinite(v_img)]
    vmax = float(np.nanpercentile(fin, 90))
    print('LITE V[s]: p50=%.1f p90=%.1f path_len=%d (%s) reached_goal=%s'
          % (np.nanpercentile(fin,50), np.nanpercentile(fin,90), len(path), path_src, reached))

    fig, ax = plt.subplots(figsize=(6.8, 6.6))
    ax.imshow(np.where(free, 0.92, 0.18), cmap='gray', vmin=0, vmax=1, origin='upper')
    hm = ax.imshow(v_img, cmap='turbo', origin='upper', alpha=0.92, vmin=0, vmax=vmax)
    if len(path) > 1:
        ax.plot(path[:, 1], path[:, 0], '-', color='white', lw=1.8, alpha=0.9,
                label=f'optimal path ({path_src})')
    ax.plot(gc, gr, '*', color='lime', ms=22, mec='black', mew=1.5, label='goal', zorder=5)
    ax.plot(sc, sr, 'o', color='magenta', ms=13, mec='black', mew=1.5,
            label=f'robot start (V={start_v:.0f}s)', zorder=5)
    cb = fig.colorbar(hm, ax=ax, shrink=0.82)
    cb.set_label('cost-to-go V* [s] (min over theta, benchmark cfg: safety_penalty=1e5)')
    ax.set_title(f'lite Tsudanuma ({w}x{h}, 0.15m) - VI value function V* (vi_rs frontier2d_par)\n'
                 f'reachable: {reachable:,} cells (of {int(free.sum()):,} free)  '
                 f'/ goal world(57.4, 66.1), start world(0.1, 0.1)')
    ax.set_xlabel('x [cell]'); ax.set_ylabel('y [cell]'); ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    out = f'{RES}/lite/fig_map_overlay.png'
    fig.savefig(out, dpi=140)
    print('wrote', out, 'reachable', reachable)


def map_fig_full():
    """full assets インスタンス (1963x1334, 732k free) の V オーバーレイ。"""
    pgm = f'{RES}/full/map_tsudanuma_015.pgm'
    vbin = f'{RES}/full/full_value.bin'
    if not (os.path.exists(pgm) and os.path.exists(vbin)):
        print('full plot skipped (missing', pgm, 'or', vbin, ')')
        return
    img = load_pgm(pgm)
    h, w = img.shape
    free = img == 255
    v_img = load_value(vbin)[::-1, :]
    reachable = int(np.isfinite(v_img).sum())
    gr, gc = 1042, 1199   # goal image (row,col) from distance-transform finder
    sr, sc = 248, 234     # start image (row,col)
    if not np.isfinite(v_img[sr, sc]):
        ys, xs = np.where(np.isfinite(v_img))
        d = (ys - sr) ** 2 + (xs - sc) ** 2
        sr, sc = int(ys[d.argmin()]), int(xs[d.argmin()])
    start_v = float(v_img[sr, sc])
    path = load_path(f'{RES}/full/full_path.bin', h)
    path_src = 'policy rollout'
    if path is None:
        path = descend_path(v_img, sr, sc, gr, gc)
        path_src = 'V descent'
    fin = v_img[np.isfinite(v_img)]
    vmax = float(np.nanpercentile(fin, 90))
    reached = bool(abs(path[-1][0]-gr)<=3 and abs(path[-1][1]-gc)<=3)
    print('FULL V[s]: p50=%.0f p90=%.0f path_len=%d (%s) reached_goal=%s reach=%d'
          % (np.nanpercentile(fin,50), vmax, len(path), path_src, reached, reachable))

    fig, ax = plt.subplots(figsize=(11, 7.6))
    ax.imshow(np.where(free, 0.92, 0.18), cmap='gray', vmin=0, vmax=1, origin='upper')
    hm = ax.imshow(v_img, cmap='turbo', origin='upper', alpha=0.92, vmin=0, vmax=vmax)
    if len(path) > 1:
        ax.plot(path[:, 1], path[:, 0], '-', color='white', lw=1.5, alpha=0.9,
                label=f'optimal path ({path_src})')
    ax.plot(gc, gr, '*', color='lime', ms=22, mec='black', mew=1.5, label='goal', zorder=5)
    ax.plot(sc, sr, 'o', color='magenta', ms=13, mec='black', mew=1.5,
            label=f'robot start (V={start_v:.0f}s)', zorder=5)
    cb = fig.colorbar(hm, ax=ax, shrink=0.82)
    cb.set_label('cost-to-go V* [s] (min over theta, safety_penalty=1e5)')
    ax.set_title(f'FULL Tsudanuma assets ({w}x{h}, 0.15m, 732,683 free) - VI value function V*\n'
                 f'reachable: {reachable:,} cells  /  goal world(179.9, 43.7), start world(35.2, 162.8) 187m away\n'
                 f'solved by vi_rs frontier2d_sparse in 11.3 s (12T, exact fixpoint; bit-exact w/ ROS1 original)')
    ax.set_xlabel('x [cell]'); ax.set_ylabel('y [cell]'); ax.legend(loc='upper right', fontsize=9)
    fig.tight_layout()
    out = f'{RES}/full/fig_map_overlay_full.png'
    fig.savefig(out, dpi=120)
    print('wrote', out)


# ---------- Fig 4: tsukuba assets full-map overlay (PoC) ----------
def map_fig_tsukuba():
    """map_tsukuba (13250x7100 @0.05m, origin (-553.84,-60.609)) を x5 プール (0.25m) した
    インスタンスの V オーバーレイ。tsudanuma と異なり origin 非ゼロなので座標変換を持つ。"""
    tdir = os.path.abspath(f'{RES}/../tsukuba')
    pgm = f'{tdir}/map_tsukuba_pooled.pgm'
    vbin = f'{tdir}/value.bin'
    if not (os.path.exists(pgm) and os.path.exists(vbin)):
        print('tsukuba plot skipped (missing inputs in', tdir, ')')
        return
    TRES, TOX, TOY = 0.25, -553.840, -60.609
    img = load_pgm(pgm)
    h, w = img.shape
    free = img == 255
    v_img = load_value(vbin)[::-1, :]
    reachable = int(np.isfinite(v_img).sum())
    gr, gc = h - 1 - 238, 2297   # goal world(20.5, -1.0) -> cell(2297, iy=238) 右下開放点
    sr, sc = h - 1 - 1370, 568   # start world(-411.7, 282.0) -> cell(568, iy=1370)
    start_v = float(v_img[h - 1 - 1370 + 0, sc]) if np.isfinite(v_img[sr, sc]) else float('nan')
    path = None
    try:
        with open(f'{tdir}/path.bin', 'rb') as f:
            n = int(np.frombuffer(f.read(4), '<i4')[0])
            xy = np.frombuffer(f.read(n * 8), '<f4').reshape(n, 2)
        cols = (xy[:, 0] - TOX) / TRES - 0.5
        rows = (h - 0.5) - (xy[:, 1] - TOY) / TRES
        path = np.stack([rows, cols], axis=1)
    except FileNotFoundError:
        pass
    fin = v_img[np.isfinite(v_img)]
    vmax = float(np.nanpercentile(fin, 90))
    print('TSUKUBA V[s]: p50=%.0f p90=%.0f reach=%d path=%s'
          % (np.nanpercentile(fin, 50), vmax, reachable,
             'none' if path is None else len(path)))

    fig, ax = plt.subplots(figsize=(13, 7.6))
    ax.imshow(np.where(free, 0.92, 0.18), cmap='gray', vmin=0, vmax=1, origin='upper')
    hm = ax.imshow(v_img, cmap='turbo', origin='upper', alpha=0.92, vmin=0, vmax=vmax)
    if path is not None and len(path) > 1:
        ax.plot(path[:, 1], path[:, 0], '-', color='white', lw=1.5, alpha=0.9,
                label='optimal path (policy rollout)')
    ax.plot(gc, gr, '*', color='lime', ms=22, mec='black', mew=1.5, label='goal', zorder=5)
    ax.plot(sc, sr, 'o', color='magenta', ms=13, mec='black', mew=1.5,
            label=f'robot start (V={start_v:.0f}s)', zorder=5)
    cb = fig.colorbar(hm, ax=ax, shrink=0.82)
    cb.set_label('cost-to-go V* [s] (min over theta, safety_penalty=1e5)')
    ax.set_title(f'Tsukuba assets ({w}x{h}, 0.25m pooled x5, 391,070 free) - VI value function V*\n'
                 f'reachable: {reachable:,} cells / goal world(20.5, -1.0), start world(-411.7, 282.0)\n'
                 f'solved by vi_rs frontier2d_sparse in 7.7 s (12T, exact fixpoint, 226M states)')
    ax.set_xlabel('x [cell]'); ax.set_ylabel('y [cell]'); ax.legend(loc='upper right', fontsize=9)
    fig.tight_layout()
    out = f'{tdir}/fig_map_overlay_tsukuba.png'
    fig.savefig(out, dpi=120)
    print('wrote', out)


if __name__ == '__main__':
    import sys as _sys
    mode = _sys.argv[2] if len(_sys.argv) > 2 else 'all'
    if mode in ('all', 'speed'):
        speed_fig()
    if mode in ('all', 'house'):
        house_fig()
    if mode in ('all', 'lite'):
        map_fig()
    if mode in ('all', 'full'):
        map_fig_full()
    if mode == 'tsukuba':
        map_fig_tsukuba()
