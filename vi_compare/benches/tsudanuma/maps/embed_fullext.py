#!/usr/bin/env python3
"""論文 Ueda 2023 の津田沼評価条件（全体地図 156.9M 状態のうち自由は 9.9M のみ =
黒線で舗装路に制限）に探索条件を一致させた **full-extent** インスタンスを生成する。

構成: 既存 pooled full 地図 (map_tsudanuma_015.pgm, 1963x1334, 0.15 m) の寸法のまま、
lite クロップ窓 (pooled 格子 x∈[712,1252), y∈[397,937)) の外を全て occupied にする。
窓内は lite と同一 → 自由セル 162,130 (論文 165,076 の 98.2%)、
総状態 1963*1334*60 = 157,118,520 (論文 156,920,760 の +0.13%)。

lite 世界座標 + (106.8, 59.55) = fullext 世界座標
（lite goal (57.375,66.075) → fullext goal (164.175,125.625)）。

usage: embed_fullext.py FULL_PGM LITE_PGM OUT_DIR
"""
import sys, os
import numpy as np

CROP_X0, CROP_X1 = 712, 1252   # pooled 格子 (world bottom-up) の lite 窓
CROP_Y0, CROP_Y1 = 397, 937
RES = 0.15


def load_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        int(f.readline())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape((h, w))
    return w, h, data


def main():
    full_pgm = sys.argv[1]
    lite_pgm = sys.argv[2]
    out_dir = sys.argv[3]

    fw, fh, full_img = load_pgm(full_pgm)   # top-down: image row r = pooled row fh-1-r
    lw, lh, lite_img = load_pgm(lite_pgm)
    assert (lw, lh) == (CROP_X1 - CROP_X0, CROP_Y1 - CROP_Y0), (lw, lh)

    # lite image row r ↔ pooled crop row (lh-1-r) ↔ pooled full row CROP_Y0+(lh-1-r)
    # ↔ full image row fh-1-(CROP_Y0+lh-1-r) = (fh-CROP_Y1)+r
    r0 = fh - CROP_Y1
    window = full_img[r0:r0 + lh, CROP_X0:CROP_X1]
    if not np.array_equal(window, lite_img):
        diff = int((window != lite_img).sum())
        raise SystemExit(f'lite window mismatch vs pooled full: {diff} px differ')

    out = np.zeros_like(full_img)           # 0 = occupied (black)
    out[r0:r0 + lh, CROP_X0:CROP_X1] = lite_img

    free = int((out == 255).sum())
    total_states = fw * fh * 60
    print(f'fullext grid: {fw}x{fh}  free_cells={free}  free_states={free*60}  '
          f'total_states={total_states}  area={free*RES*RES:.1f} m^2')

    os.makedirs(out_dir, exist_ok=True)
    pgm = os.path.join(out_dir, 'map_tsudanuma_fullext.pgm')
    yaml = os.path.join(out_dir, 'map_tsudanuma_fullext.yaml')
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (fw, fh))
        f.write(out.tobytes())
    with open(yaml, 'w') as f:
        f.write('image: map_tsudanuma_fullext.pgm\n')
        f.write(f'resolution: {RES:.6f}\n')
        f.write('origin: [0.000000, 0.000000, 0.000000]\n')
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(f'wrote {pgm} and {yaml}')

    # goal 検証: lite goal (57.375,66.075) → fullext (164.175,125.625) → cell (1094,837)
    gx, gy = int(164.175 / RES), int(125.625 / RES)
    img_row = fh - 1 - gy
    v = out[img_row, gx]
    print(f'goal world(164.175,125.625) -> cell ({gx},{gy}) {"free" if v == 255 else "BLOCKED"}')


if __name__ == '__main__':
    main()
