#!/usr/bin/env python3
"""津田沼フル (1963×1334×60 = 157M 状態) の ROS1 vs vi_rs sparse 並列スイープ動画。

house 版 (render_frames.py) との違い:
  - 両側 16 スレッド同士の比較。
  - ROS1 はこの規模では収束しない → フェーズ構成:
      intro → real-time ×1 (sparse が 11.9 s で厳密収束するまで) →
      TIMELAPSE ×40 (ROS1 が 600 s 走っても未収束) → end card。
  - フレームは 10.5 MB/枚 なので全ロードせず逐次読み (アクセスは単調)。

タイムライン: vi_rs はスナップショット時刻 (ダンプ時間除外済) をクリーン計測
11.93 s へ正規化。ROS1 は収束しないので正規化先が無く、計測ラン (スナップ
ショット帯域汚染込み) の生 wall-clock をそのまま使う (フッターに明記)。
"""
import os
import glob
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = '/work'
RES = f'{ROOT}/vi_compare/results/tsudanuma'
OUT = f'{RES}/video_frames'
PGM = f'{RES}/full/map_tsudanuma_015.pgm'
NX, NY = 1963, 1334
FPS = 30
T_SPARSE = 11.93        # clean 16T wall-clock (この日の直接計測)
T_PHASE1 = 13.5         # real-time 区間の長さ [s]
LAPSE = 40.0            # phase2 の倍率
T_END = 600.0           # ROS1 打ち切り [s]
GARBAGE = 1.0e6
INTRO_SEC, END_SEC = 3.0, 4.0
GX, GY = 1199, 291      # goal world(179.925, 43.725) / res 0.15 / origin(0,0)


class Side:
    """snap_NNNNN.bin 列の逐次リーダ (単調アクセス前提、1 枚キャッシュ)。"""

    def __init__(self, d, scale=1.0):
        self.files = sorted(glob.glob(f'{d}/snap_*.bin'))
        ts, rounds = [], []
        with open(f'{d}/times.csv') as f:
            next(f)
            for line in f:
                i, t, r = line.strip().split(',')
                ts.append(float(t) * scale)
                rounds.append(int(r))
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
            self.cache = np.fromfile(self.files[i], dtype='<f4').reshape(NY, NX)
            self.cache_i = i
        return self.cache, self.rounds[i]

    def last(self):
        return self.at(self.ts[-1] + 1)[0]


def main():
    os.makedirs(OUT, exist_ok=True)
    with open(PGM, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        pgm = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    occ_wall = np.flipud(pgm < 250 * 0.35)

    L = Side(f'{RES}/frames_ros1')
    R0 = Side(f'{RES}/frames_sparse')
    R = Side(f'{RES}/frames_sparse', scale=T_SPARSE / R0.ts[-1])
    meta = json.load(open(f'{RES}/snap_run/ros1_m16.json'))
    resid_s = meta['last_max_delta'] / 262144.0
    n_round_r = R.rounds[-1]
    print(f'L frames={len(L.files)} (last t={L.ts[-1]:.0f}s) R frames={len(R.files)} '
          f'resid={resid_s:.0f}s rounds={n_round_r}')

    final_r = R.last().copy()
    fin = np.isfinite(final_r)
    # 津田沼は壁沿い safety-penalty セルが到達セルの ~8% を占める (V>1e5 s) ので、
    # 通常走行域 (<5000 s) の P99 でスケールし penalty 域は赤に飽和させる。
    vals = final_r[fin]
    vmax = float(np.percentile(vals[vals < 5000.0], 99.0))
    print(f'vmax={vmax:.1f}s')
    cmap = plt.get_cmap('turbo')

    wall_rgb = np.float32((0.55, 0.55, 0.58))

    def render_field(a):
        img = np.zeros((NY, NX, 3), dtype=np.float32)
        img[:] = (0.06, 0.06, 0.09)
        img[occ_wall] = wall_rgb
        if a is not None:
            reached = np.isfinite(a) & (a < GARBAGE)
            v = np.sqrt(np.clip(a[reached] / vmax, 0, 1))
            img[reached] = cmap(v)[:, :3].astype(np.float32)
        return img

    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor='#101014')
    axL = fig.add_axes([0.025, 0.13, 0.46, 0.70])
    axR = fig.add_axes([0.515, 0.13, 0.46, 0.70])
    for ax in (axL, axR):
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_color('#444')
    imL = axL.imshow(render_field(None), origin='lower', interpolation='nearest')
    imR = axR.imshow(render_field(None), origin='lower', interpolation='nearest')
    for ax in (axL, axR):
        ax.plot(GX, GY, marker='*', ms=16, mfc='white', mec='black', mew=1.0, zorder=5)

    fig.text(0.5, 0.955, 'Value Iteration sweep — ROS1 (C++) vs vi_rs (Rust), 16 threads each',
             ha='center', color='white', fontsize=25, fontweight='bold')
    fig.text(0.5, 0.915,
             'Tsudanuma campus 1963×1334×60 = 157M states (0.15 m/cell) · identical problem '
             '· goal (179.9, 43.7, 0°)',
             ha='center', color='#aaaaaa', fontsize=14)
    fig.text(0.26, 0.875, 'ROS1 value_iteration  (C++, 16 threads)',
             ha='center', color='#ff9966', fontsize=16, fontweight='bold')
    fig.text(0.74, 0.875, 'vi_rs frontier2d_sparse  (Rust, 16 threads)',
             ha='center', color='#66ccff', fontsize=16, fontweight='bold')

    timerL = fig.text(0.26, 0.065, '', ha='center', color='white', fontsize=27,
                      family='monospace', fontweight='bold')
    timerR = fig.text(0.74, 0.065, '', ha='center', color='white', fontsize=27,
                      family='monospace', fontweight='bold')
    lapse = fig.text(0.5, 0.875, '', ha='center', color='#ffee66', fontsize=20,
                     fontweight='bold')
    fig.text(0.5, 0.018,
             'colors: cost-to-go (s) · vi_rs timeline normalized to clean-run wall clock; '
             'ROS1 timeline = instrumented-run wall clock · ROS1 stop criterion ΔV<0.1 s/sweep '
             '(vi_rs reaches the stricter exact fixed point)',
             ha='center', color='#777777', fontsize=11)
    banner = fig.text(0.5, 0.5, '', ha='center', va='center', color='#ffee66',
                      fontsize=34, fontweight='bold',
                      bbox=dict(boxstyle='round,pad=0.6', fc='#101014', ec='#ffee66',
                                lw=2, alpha=0.93))
    banner.set_visible(False)
    intro = fig.text(0.5, 0.52, '', ha='center', va='center', color='white', fontsize=26)

    frame_no = 0

    def save():
        nonlocal frame_no
        fig.savefig(f'{OUT}/frame_{frame_no:05d}.png', facecolor=fig.get_facecolor())
        frame_no += 1

    # --- intro ---
    intro.set_text('Same 3-D (x, y, θ) value iteration, same 16-thread budget.\n\n'
                   'Left:  original ROS1 node — full-grid sweeps\n'
                   'Right: vi_rs sparse solver — frontier + θ-mask evaluation\n\n'
                   'First segment plays in REAL TIME.')
    timerL.set_text('t =   0.0 s')
    timerR.set_text('t =   0.0 s')
    for _ in range(int(INTRO_SEC * FPS)):
        save()
    intro.set_visible(False)

    # --- main: phase1 real-time, phase2 timelapse ---
    k1 = int(T_PHASE1 * FPS)
    n2 = int(np.ceil((T_END - T_PHASE1) / LAPSE * FPS))
    for k in range(k1 + n2 + 1):
        if k <= k1:
            t = k / FPS
            if k == 0:
                lapse.set_text('REAL TIME')
        else:
            t = min(T_PHASE1 + (k - k1) / FPS * LAPSE, T_END)
            lapse.set_text(f'TIMELAPSE ×{int(LAPSE)}')
            lapse.set_color('#ff6666')
        # left: ROS1, ずっと未収束
        fl, rl = L.at(t)
        imL.set_data(render_field(fl))
        timerL.set_text(f't = {t:5.1f} s   sweep {rl}')
        # right: sparse
        if t >= T_SPARSE:
            imR.set_data(render_field(final_r))
            timerR.set_text(f'CONVERGED (exact)  {T_SPARSE:.1f} s')
            timerR.set_color('#66ff88')
        else:
            fr, rr = R.at(t)
            imR.set_data(render_field(fr))
            timerR.set_text(f't = {t:5.1f} s   round {rr}/{n_round_r}')
        save()

    # --- end card ---
    timerL.set_text(f'NOT CONVERGED after {int(T_END)} s   (ΔV ≈ {resid_s:.0f} s)')
    timerL.set_color('#ff6666')
    banner.set_visible(True)
    banner.set_text(f'vi_rs: exact fixed point in {T_SPARSE:.1f} s\n'
                    f'ROS1: still ΔV ≈ {resid_s:.0f} s after {int(T_END)} s\n'
                    f'→  ≥ {int(T_END / T_SPARSE)}× faster on this map')
    for _ in range(int(END_SEC * FPS)):
        save()
    print(f'wrote {frame_no} frames to {OUT}')


if __name__ == '__main__':
    main()
