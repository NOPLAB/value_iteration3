#!/usr/bin/env python3
"""ROS1 本家 vs vi_rs frontier2d_sparse のスイープ進行を左右並列で描画する
動画フレームレンダラ (PNG 連番出力; mp4 化は host の ffmpeg)。

usage: render_frames.py {house,tsudanuma,tsukuba}

マップ別の定数 (グリッド, PGM, goal セル, タイトル, 壁閾値, キャプション) は
MAPS 表に集約。タイムラインは 2 形態:
  - house: 両側とも収束する規模 → 全編スローモーション ×8。両側の時刻軸を
    「スナップショット無しのクリーン計測値」(sweep CSV 記載) に一様リスケール。
  - tsudanuma/tsukuba: ROS1 はこの規模では収束しない → intro → real-time →
    TIMELAPSE ×40 → end card。津田沼は vi_rs 側のみクリーン計測 11.93 s へ正規化
    (ROS1 は正規化先が無く計測ランの生 wall-clock)、tsukuba は両側とも実測
    wall-clock (snapshotWorker 時計; セットアップ除外)。

入力 (results/<map>/): frames_ros1/ frames_sparse/ (snap_NNNNN.bin: f32 min-θ
値 [s], 未確定=inf + times.csv)、背景 PGM。フレームが大きいマップでも動くよう
全ロードせず逐次読み (アクセスは単調)。出力: results/<map>/video_frames/。
"""
import glob
import json
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = os.environ.get('VI_ROOT', '/work')  # docker mount: repo root
FPS = 30
GARBAGE = 1.0e6          # 本家の未確定折返しゴミ値はこの閾値で未到達扱い
INTRO_SEC, END_SEC = 3.0, 4.0
LAPSE = 40.0             # lapse モード phase2 の倍率

MAPS = {
    'house': dict(
        mode='slowmo', res='house', pgm='house.pgm', nx=384, ny=384,
        wall_lt=250 * 0.35, goal=(320, 160),  # goal world(6,-2) / res 0.05 / origin(-10,-10)
        slowmo=8.0,
        t_ros1=2.559,       # clean wall-clock, sweep_ros1_house.csv m=4 (本家最良)
        t_sparse=0.363,     # clean wall-clock, sweep_vi_rs_sparse_house.csv m=12
        t_ros1_snap=3.431,  # スナップショット付きランの収束時刻 (client elapsed)
        sweeps_total=6, rounds_total=68,
        title='Value Iteration sweep — ROS1 (C++) vs vi_rs (Rust)',
        subtitle='house map 384×384×60 = 8.8M states · identical problem & '
                 'transition model · goal (6.0, −2.0, 90°)',
        label_l='ROS1 value_iteration  (C++, 4 threads, best config)',
        label_r='vi_rs frontier2d_sparse  (Rust, 12 threads)',
        footer='slow motion ×8 · timelines normalized to clean-run wall clock '
               '· colors: cost-to-go (s)',
        banner='Same optimal value function (exact fixed point)\n'
               'vi_rs: 0.36 s   vs   ROS1: 2.56 s   →   7.1× faster',
    ),
    'tsudanuma': dict(
        mode='lapse', res='tsudanuma', pgm='full/map_tsudanuma_015.pgm',
        nx=1963, ny=1334,
        wall_lt=250 * 0.35, goal=(1199, 291),  # world(179.925,43.725)/res0.15/origin(0,0)
        t_sparse=11.93,     # clean 16T wall-clock → vi_rs タイムラインをここへ正規化
        t_phase1=13.5, t_end=600.0,
        subtitle='Tsudanuma campus 1963×1334×60 = 157M states (0.15 m/cell) '
                 '· identical problem · goal (179.9, 43.7, 0°)',
        footer='colors: cost-to-go (s) · vi_rs timeline normalized to clean-run wall '
               'clock; ROS1 timeline = instrumented-run wall clock · ROS1 stop '
               'criterion ΔV<0.1 s/sweep (vi_rs reaches the stricter exact fixed point)',
    ),
    'tsukuba': dict(
        mode='lapse', res='tsukuba', pgm='map_tsukuba_pooled.pgm',
        nx=4417, ny=2367,
        wall_lt=128, goal=(3828, 397),  # world(20.5,-1.0)/res0.15/origin(-553.84,-60.609)
        t_sparse=None, t_phase1=None, t_end=None,  # 実測 (times.csv) から導出
        subtitle='Tsukuba campus 4417×2367×60 = 627M states (0.15 m/cell) '
                 '· identical problem · goal (20.5, -1.0, 0°)',
        footer='colors: cost-to-go (s) · both timelines = measured wall clock · '
               'ROS1 stop criterion never reached on this map '
               '(vi_rs reaches the exact fixed point)',
    ),
}


class Side:
    """snap_NNNNN.bin 列の逐次リーダ (単調アクセス前提、1 枚キャッシュ)。"""

    def __init__(self, d, ny, nx, scale=1.0):
        self.ny, self.nx = ny, nx
        self.files = sorted(glob.glob(f'{d}/snap_*.bin'))
        ts, rounds = [], []
        with open(f'{d}/times.csv') as f:
            next(f)
            for line in f:
                parts = line.strip().split(',')
                if len(parts) < 3:
                    continue
                ts.append(float(parts[1]) * scale)
                rounds.append(int(parts[2]))
        n = min(len(self.files), len(ts))
        self.files, self.ts, self.rounds = self.files[:n], np.array(ts[:n]), rounds[:n]
        self.cache_i = -1
        self.cache = None

    def at(self, t):
        """時刻 t 直前のフレーム (無ければ None) と round。"""
        i = int(np.searchsorted(self.ts, t, side='right')) - 1
        if i < 0:
            return None, 0
        if i != self.cache_i:
            self.cache = np.fromfile(self.files[i], dtype='<f4').reshape(self.ny, self.nx)
            self.cache_i = i
        return self.cache, self.rounds[i]

    def last(self):
        return self.at(self.ts[-1] + 1)[0]


def main():
    cfg = MAPS[sys.argv[1]]
    res_dir = f'{ROOT}/vi_compare/results/{cfg["res"]}'
    out_dir = f'{res_dir}/video_frames'
    ny, nx = cfg['ny'], cfg['nx']
    slowmo = cfg['mode'] == 'slowmo'
    os.makedirs(out_dir, exist_ok=True)
    with open(f'{res_dir}/{cfg["pgm"]}', 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        pgm = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    # map_server は PGM 行を上下反転して OccupancyGrid にする → states と揃える
    occ_wall = np.flipud(pgm < cfg['wall_lt'])

    if slowmo:
        L = Side(f'{res_dir}/frames_ros1', ny, nx, scale=cfg['t_ros1'] / cfg['t_ros1_snap'])
        R = Side(f'{res_dir}/frames_sparse', ny, nx)
        R.ts = R.ts * (cfg['t_sparse'] / R.ts[-1])
        t_ros1, t_sparse = cfg['t_ros1'], cfg['t_sparse']
        resid_s = 0.0
        print(f'frames L={len(L.files)} R={len(R.files)}')
    else:
        L = Side(f'{res_dir}/frames_ros1', ny, nx)
        R = Side(f'{res_dir}/frames_sparse', ny, nx)
        if cfg['t_sparse'] is not None:
            t_sparse = cfg['t_sparse']
            R.ts = R.ts * (t_sparse / R.ts[-1])
            t_phase1, t_end = cfg['t_phase1'], cfg['t_end']
        else:
            t_sparse = float(R.ts[-1])          # vi_rs clean wall-clock (収束時刻)
            t_phase1 = t_sparse + 1.1           # 収束を見せてから timelapse へ
            t_end = float(np.ceil(L.ts[-1]))
        meta = json.load(open(f'{res_dir}/snap_run/ros1_m16.json'))
        resid_s = (meta.get('last_max_delta') or 0.0) / 262144.0
        print(f'L(ROS1) frames={len(L.files)} (last t={L.ts[-1]:.0f}s) '
              f'R(vi_rs) frames={len(R.files)} T_SPARSE={t_sparse:.2f}s '
              f'resid={resid_s:.0f}s rounds={R.rounds[-1]}')
    n_round_r = R.rounds[-1]

    final_r = R.last().copy()
    fin = np.isfinite(final_r)
    vals = final_r[fin]
    if slowmo:
        # P99 でスケール (safety-penalty 域 V~1e5 s などの上位 1% は赤に飽和)
        vmax = float(np.percentile(vals, 99.0))
    else:
        # 壁沿い safety-penalty セル (V>1e5 s) が混じるので、通常走行域 (<5000 s)
        # の P99 でスケールし penalty 域は赤に飽和させる。
        drive = vals[vals < 5000.0]
        vmax = float(np.percentile(drive, 99.0)) if drive.size else float(np.percentile(vals, 90.0))
    print(f'vmax={vmax:.1f}s  reachable={int(fin.sum())}')
    cmap = plt.get_cmap('turbo')

    wall_rgb = np.float32((0.55, 0.55, 0.58))

    def render_field(a):
        """f32 値場 → RGB。壁=明灰、未到達=暗、到達=turbo(sqrt スケール)。"""
        img = np.zeros((ny, nx, 3), dtype=np.float32)
        img[:] = (0.06, 0.06, 0.09)
        img[occ_wall] = wall_rgb
        if a is not None:
            reached = np.isfinite(a) & (a < GARBAGE)
            v = np.sqrt(np.clip(a[reached] / vmax, 0, 1))
            img[reached] = cmap(v)[:, :3].astype(np.float32)
        return img

    # --- figure (1920x1080) ---
    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor='#101014')
    if slowmo:
        rects = [(0.035, 0.10, 0.44, 0.73), (0.525, 0.10, 0.44, 0.73)]
        label_y, timer_y, footer_y = 0.875, 0.045, 0.012
        title_fs, sub_fs, label_fs, timer_fs, banner_fs = 26, 15, 17, 30, 40
    else:
        rects = [(0.025, 0.10, 0.46, 0.72), (0.515, 0.10, 0.46, 0.72)]
        label_y, timer_y, footer_y = 0.86, 0.05, 0.012
        title_fs, sub_fs, label_fs, timer_fs, banner_fs = 25, 14, 16, 27, 34
    axL = fig.add_axes(rects[0])
    axR = fig.add_axes(rects[1])
    for ax in (axL, axR):
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_color('#444')
    imL = axL.imshow(render_field(None), origin='lower', interpolation='nearest')
    imR = axR.imshow(render_field(None), origin='lower', interpolation='nearest')
    gx, gy = cfg['goal']
    for ax in (axL, axR):
        ax.plot(gx, gy, marker='*', ms=16 if not slowmo else 18,
                mfc='white', mec='black', mew=1.0, zorder=5)

    fig.text(0.5, 0.955, cfg.get(
        'title', 'Value Iteration sweep — ROS1 (C++) vs vi_rs (Rust), 16 threads each'),
        ha='center', color='white', fontsize=title_fs, fontweight='bold')
    fig.text(0.5, 0.915, cfg['subtitle'], ha='center', color='#aaaaaa', fontsize=sub_fs)
    fig.text(0.26, label_y, cfg.get('label_l', 'ROS1 value_iteration  (C++, 16 threads)'),
             ha='center', color='#ff9966', fontsize=label_fs, fontweight='bold')
    fig.text(0.74, label_y, cfg.get('label_r', 'vi_rs frontier2d_sparse  (Rust, 16 threads)'),
             ha='center', color='#66ccff', fontsize=label_fs, fontweight='bold')

    timerL = fig.text(0.26, timer_y, '', ha='center', color='white', fontsize=timer_fs,
                      family='monospace', fontweight='bold')
    timerR = fig.text(0.74, timer_y, '', ha='center', color='white', fontsize=timer_fs,
                      family='monospace', fontweight='bold')
    lapse = fig.text(0.5, label_y, '', ha='center', color='#ffee66', fontsize=20,
                     fontweight='bold')
    fig.text(0.5, footer_y, cfg['footer'], ha='center', color='#777777',
             fontsize=12 if slowmo else 11)
    banner = fig.text(0.5, 0.5, '', ha='center', va='center', color='#ffee66',
                      fontsize=banner_fs, fontweight='bold',
                      bbox=dict(boxstyle='round,pad=0.6', fc='#101014', ec='#ffee66',
                                lw=2, alpha=0.93))
    banner.set_visible(False)
    intro = fig.text(0.5, 0.52, '', ha='center', va='center', color='white',
                     fontsize=28 if slowmo else 26)

    frame_no = 0

    def save():
        nonlocal frame_no
        fig.savefig(f'{out_dir}/frame_{frame_no:05d}.png', facecolor=fig.get_facecolor())
        frame_no += 1

    # --- intro ---
    if slowmo:
        intro.set_text('Both nodes solve the SAME 3-D (x, y, θ) value iteration\n'
                       'to the same optimal policy (verified cell-by-cell).\n\n'
                       'Left:  original ROS1 node — every thread sweeps the whole grid\n'
                       'Right: vi_rs sparse solver — frontier + θ-mask sparse evaluation\n\n'
                       'Watch the cost-to-go wave expand from the goal ★')
        timerL.set_text('t = 0.000 s')
        timerR.set_text('t = 0.000 s')
    else:
        intro.set_text('Same 3-D (x, y, θ) value iteration, same 16-thread budget.\n\n'
                       'Left:  original ROS1 node — full-grid sweeps\n'
                       'Right: vi_rs sparse solver — frontier + θ-mask evaluation\n\n'
                       'First segment plays in REAL TIME.')
        timerL.set_text('t =   0.0 s')
        timerR.set_text('t =   0.0 s')
    for _ in range(int(INTRO_SEC * FPS)):
        save()
    intro.set_visible(False)

    # --- main run ---
    if slowmo:
        n_main = int(np.ceil(t_ros1 * cfg['slowmo'] * FPS)) + 1
        for k in range(n_main):
            t = k / FPS / cfg['slowmo']
            if t >= t_ros1:
                imL.set_data(render_field(L.last()))
                timerL.set_text(f'CONVERGED  {t_ros1:.2f} s')
                timerL.set_color('#66ff88')
            else:
                fl, rl = L.at(t)
                imL.set_data(render_field(fl))
                timerL.set_text(f't = {t:6.3f} s   sweep {rl}/{cfg["sweeps_total"]}')
            if t >= t_sparse:
                imR.set_data(render_field(final_r))
                timerR.set_text(f'CONVERGED  {t_sparse:.2f} s')
                timerR.set_color('#66ff88')
            else:
                fr, rr = R.at(t)
                imR.set_data(render_field(fr))
                timerR.set_text(f't = {t:6.3f} s   round {rr}/{cfg["rounds_total"]}')
            save()
    else:
        # phase1 real-time, phase2 timelapse
        k1 = int(t_phase1 * FPS)
        n2 = int(np.ceil((t_end - t_phase1) / LAPSE * FPS))
        for k in range(k1 + n2 + 1):
            if k <= k1:
                t = k / FPS
                if k == 0:
                    lapse.set_text('REAL TIME')
            else:
                t = min(t_phase1 + (k - k1) / FPS * LAPSE, t_end)
                lapse.set_text(f'TIMELAPSE ×{int(LAPSE)}')
                lapse.set_color('#ff6666')
            # left: ROS1, ずっと未収束
            fl, rl = L.at(t)
            imL.set_data(render_field(fl))
            timerL.set_text(f't = {t:5.1f} s   sweep {rl}')
            # right: sparse
            if t >= t_sparse:
                imR.set_data(render_field(final_r))
                timerR.set_text(f'CONVERGED (exact)  {t_sparse:.1f} s')
                timerR.set_color('#66ff88')
            else:
                fr, rr = R.at(t)
                imR.set_data(render_field(fr))
                timerR.set_text(f't = {t:5.1f} s   round {rr}/{n_round_r}')
            save()

    # --- end card ---
    if slowmo:
        banner.set_text(cfg['banner'])
    else:
        timerL.set_text(f'NOT CONVERGED after {int(t_end)} s   (ΔV ≈ {resid_s:.0f} s)')
        timerL.set_color('#ff6666')
        banner.set_text(f'vi_rs: exact fixed point in {t_sparse:.1f} s\n'
                        f'ROS1: still ΔV ≈ {resid_s:.0f} s after {int(t_end)} s\n'
                        f'→  ≥ {int(t_end / t_sparse)}× faster on this map')
    banner.set_visible(True)
    for _ in range(int(END_SEC * FPS)):
        save()
    print(f'wrote {frame_no} frames to {out_dir}')


if __name__ == '__main__':
    main()
