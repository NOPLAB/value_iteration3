#!/usr/bin/env python3
"""ROS1 本家 vs vi_rs frontier2d_sparse のスイープ進行を左右並列で描画する
動画フレームレンダラ (PNG 連番出力; mp4 化は host の ffmpeg)。

入力 (vi_compare/results/house/):
  frames_ros1/   snap_NNNNN.bin (f32 384x384, min-θ 値 [s], 未確定=inf) + times.csv
  frames_sparse/ 同上 (vi_rs 側; t はダンプ自身の時間を除いた純ソルバ時刻)
  house.pgm      壁描画用

タイムライン正規化: スナップショット取得はどちらの計測も乱すので、各側の
時刻軸を「スナップショット無しのクリーン計測値」(ROS1 m=4: 2.559 s /
sparse m=12: 0.363 s, sweep CSV 記載) に一様リスケールする。
"""
import os
import glob
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

ROOT = '/work'  # docker mount: repo root
HOUSE = f'{ROOT}/vi_compare/results/house'
OUT = f'{ROOT}/vi_compare/results/house/video_frames'
NX = NY = 384
FPS = 30
SLOWMO = 8.0
T_ROS1 = 2.559   # clean wall-clock, sweep_ros1_house.csv m=4 (本家最良)
T_SPARSE = 0.363  # clean wall-clock, sweep_vi_rs_sparse_house.csv m=12
T_ROS1_SNAP = 3.431  # スナップショット付きランの収束時刻 (client elapsed)
GARBAGE = 1.0e6  # 本家の未確定折返しゴミ値はこの閾値で未到達扱い

INTRO_SEC = 3.0
END_SEC = 4.0


def load_side(d, t_scale_src, t_scale_dst):
    files = sorted(glob.glob(f'{d}/snap_*.bin'))
    times = {}
    with open(f'{d}/times.csv') as f:
        next(f)
        for line in f:
            i, t, r = line.strip().split(',')
            times[int(i)] = (float(t), int(r))
    frames, ts, rounds = [], [], []
    for fp in files:
        idx = int(os.path.basename(fp)[5:10])
        if idx not in times:
            continue
        a = np.fromfile(fp, dtype='<f4').reshape(NY, NX)
        frames.append(a)
        ts.append(times[idx][0] * t_scale_dst / t_scale_src)
        rounds.append(times[idx][1])
    return frames, np.array(ts), rounds


def main():
    os.makedirs(OUT, exist_ok=True)
    # --- inputs ---
    with open(f'{HOUSE}/house.pgm', 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxv = int(f.readline())
        pgm = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    # map_server は PGM 行を上下反転して OccupancyGrid にする → states と揃える
    occ_wall = np.flipud(pgm < 250 * 0.35)  # occupied (dark pixels)

    fr_l, ts_l, rd_l = load_side(f'{HOUSE}/frames_ros1', T_ROS1_SNAP, T_ROS1)
    fr_r, ts_r, rd_r = load_side(f'{HOUSE}/frames_sparse', 1.0, 1.0)
    # sparse 側は times.csv の最終時刻→T_SPARSE へリスケール
    ts_r = ts_r * (T_SPARSE / ts_r[-1])

    final_r = fr_r[-1]
    fin = np.isfinite(final_r)
    # P99 でスケール (safety-penalty 域 V~1e5 s などの上位 1% は赤に飽和)
    vmax = float(np.percentile(final_r[fin], 99.0))
    print(f'vmax={vmax:.1f}s  frames L={len(fr_l)} R={len(fr_r)}')

    cmap = plt.get_cmap('turbo')

    def render_field(a):
        """f32 値場 → RGB。壁=明灰、未到達=暗、到達=turbo(sqrt スケール)。"""
        img = np.zeros((NY, NX, 3), dtype=np.float32)
        img[:] = (0.06, 0.06, 0.09)            # unreached free
        img[occ_wall] = (0.55, 0.55, 0.58)     # walls
        reached = np.isfinite(a) & (a < GARBAGE)
        v = np.sqrt(np.clip(a[reached] / vmax, 0, 1))
        img[reached] = cmap(v)[:, :3]
        return img

    # --- figure (1920x1080) ---
    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor='#101014')
    axL = fig.add_axes([0.035, 0.10, 0.44, 0.73])
    axR = fig.add_axes([0.525, 0.10, 0.44, 0.73])
    for ax in (axL, axR):
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_color('#444')
    imL = axL.imshow(render_field(fr_l[0] * np.nan), origin='lower', interpolation='nearest')
    imR = axR.imshow(render_field(fr_r[0] * np.nan), origin='lower', interpolation='nearest')
    gx, gy = 320, 160  # goal world(6,-2) / res 0.05 / origin(-10,-10)
    for ax in (axL, axR):
        ax.plot(gx, gy, marker='*', ms=18, mfc='white', mec='black', mew=1.0, zorder=5)

    fig.text(0.5, 0.955, 'Value Iteration sweep — ROS1 (C++) vs vi_rs (Rust)',
             ha='center', color='white', fontsize=26, fontweight='bold')
    fig.text(0.5, 0.915,
             'house map 384×384×60 = 8.8M states · identical problem & '
             'transition model · goal (6.0, −2.0, 90°)',
             ha='center', color='#aaaaaa', fontsize=15)

    tL = fig.text(0.255, 0.875, 'ROS1 value_iteration  (C++, 4 threads, best config)',
                  ha='center', color='#ff9966', fontsize=17, fontweight='bold')
    tR = fig.text(0.745, 0.875, 'vi_rs frontier2d_sparse  (Rust, 12 threads)',
                  ha='center', color='#66ccff', fontsize=17, fontweight='bold')

    timerL = fig.text(0.255, 0.045, '', ha='center', color='white', fontsize=30,
                      family='monospace', fontweight='bold')
    timerR = fig.text(0.745, 0.045, '', ha='center', color='white', fontsize=30,
                      family='monospace', fontweight='bold')
    noteC = fig.text(0.5, 0.012, f'slow motion ×{int(SLOWMO)} · timelines '
                     'normalized to clean-run wall clock · colors: cost-to-go (s)',
                     ha='center', color='#777777', fontsize=12)
    banner = fig.text(0.5, 0.5, '', ha='center', va='center', color='#ffee66',
                      fontsize=40, fontweight='bold',
                      bbox=dict(boxstyle='round,pad=0.6', fc='#101014', ec='#ffee66',
                                lw=2, alpha=0.92))
    banner.set_visible(False)

    introA = fig.text(0.5, 0.52, '', ha='center', va='center', color='white', fontsize=28)
    introA.set_visible(False)

    frame_no = 0

    def save():
        nonlocal frame_no
        fig.savefig(f'{OUT}/frame_{frame_no:05d}.png', facecolor=fig.get_facecolor())
        frame_no += 1

    def pick(frames, ts, t):
        i = int(np.searchsorted(ts, t, side='right')) - 1
        return None if i < 0 else frames[i], max(i, 0)

    empty = render_field(np.full((NY, NX), np.inf, dtype=np.float32))

    # --- intro ---
    imL.set_data(empty)
    imR.set_data(empty)
    introA.set_visible(True)
    introA.set_text('Both nodes solve the SAME 3-D (x, y, θ) value iteration\n'
                    'to the same optimal policy (verified cell-by-cell).\n\n'
                    'Left:  original ROS1 node — every thread sweeps the whole grid\n'
                    'Right: vi_rs sparse solver — frontier + θ-mask sparse evaluation\n\n'
                    'Watch the cost-to-go wave expand from the goal ★')
    timerL.set_text('t = 0.000 s')
    timerR.set_text('t = 0.000 s')
    for _ in range(int(INTRO_SEC * FPS)):
        save()
    introA.set_visible(False)

    # --- main run ---
    n_main = int(np.ceil(T_ROS1 * SLOWMO * FPS)) + 1
    doneR_shown = False
    for k in range(n_main):
        t = k / FPS / SLOWMO
        # left
        if t >= T_ROS1:
            imL.set_data(render_field(fr_l[-1]))
            timerL.set_text(f'CONVERGED  {T_ROS1:.2f} s')
            timerL.set_color('#66ff88')
        else:
            fl, il = pick(fr_l, ts_l, t)
            imL.set_data(render_field(fl) if fl is not None else empty)
            timerL.set_text(f't = {t:6.3f} s   sweep {rd_l[il] if fl is not None else 0}/6')
        # right
        if t >= T_SPARSE:
            imR.set_data(render_field(fr_r[-1]))
            timerR.set_text(f'CONVERGED  {T_SPARSE:.2f} s')
            timerR.set_color('#66ff88')
            if not doneR_shown:
                doneR_shown = True
        else:
            fr, ir = pick(fr_r, ts_r, t)
            imR.set_data(render_field(fr) if fr is not None else empty)
            timerR.set_text(f't = {t:6.3f} s   round {rd_r[ir] if fr is not None else 0}/68')
        save()

    # --- end card ---
    banner.set_visible(True)
    banner.set_text(f'Same optimal value function (exact fixed point)\n'
                    f'vi_rs: {T_SPARSE:.2f} s   vs   ROS1: {T_ROS1:.2f} s'
                    f'   →   7.1× faster')
    for _ in range(int(END_SEC * FPS)):
        save()

    print(f'wrote {frame_no} frames to {OUT}')


if __name__ == '__main__':
    main()
