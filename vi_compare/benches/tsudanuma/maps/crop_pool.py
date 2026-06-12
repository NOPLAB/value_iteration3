#!/usr/bin/env python3
"""assets/map_tsudanuma.pgm を ×3 min-pool した後、中心付近の正方窓で crop し、
論文 Ueda 2023 Table 2 "Actual" 列と同規模 (自由セル ~165k / 自由状態 ~9.9M) の
0.15 m/cell インスタンスを生成する。pool 部は pool_tsudanuma.py / bench_map::build_occupancy
と bit 一致 (div_ceil, obstacle-if-any, unknown->obstacle, 上下反転)。

usage:
  crop_pool.py SRC.pgm probe                 # 各 half-size の自由セル数を表示
  crop_pool.py SRC.pgm write HALF OUT_DIR     # 中心(982,667)±HALF の窓を書き出す
出力: map_tsudanuma_lite.pgm / .yaml (origin 0,0, res 0.15)。
"""
import sys, os
import numpy as np

SCALE = 3
FULL_RES = 0.05
RES = FULL_RES * SCALE  # 0.15
# pooled grid 上のクロップ中心 = 既存ベンチのゴールセル (982,667)（地図中心の自由領域）。
CX, CY = 982, 667


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


def pool_full(src):
    w, h, pix = load_pgm(src)
    p = pix.astype(np.float64)
    occ_prob = (255.0 - p) / 255.0  # negate=0
    is_obs = occ_prob > 0.65
    is_free = occ_prob < 0.196
    blocked_full = is_obs | (~is_obs & ~is_free)  # unknown -> obstacle
    ow = -(-w // SCALE)
    oh = -(-h // SCALE)
    occ = np.zeros((oh, ow), dtype=np.uint8)  # world bottom-up, 1=blocked
    for oy in range(oh):
        iy0 = oy * SCALE
        iy1 = min(iy0 + SCALE, h)
        src_rows = [h - 1 - iy for iy in range(iy0, iy1)]  # 上下反転
        rows = np.array(src_rows)[:, None]
        for ox in range(ow):
            ix0 = ox * SCALE
            ix1 = min(ix0 + SCALE, w)
            block = blocked_full[rows, np.arange(ix0, ix1)[None, :]]
            occ[oy, ox] = 1 if block.any() else 0
    return occ  # (oh, ow)


def crop(occ, half):
    oh, ow = occ.shape
    x0, x1 = max(0, CX - half), min(ow, CX + half)
    y0, y1 = max(0, CY - half), min(oh, CY + half)
    return occ[y0:y1, x0:x1]


def main():
    src = sys.argv[1]
    mode = sys.argv[2] if len(sys.argv) > 2 else 'probe'
    occ = pool_full(src)
    oh, ow = occ.shape
    print(f'full pooled grid: {ow}x{oh}  free={int((occ==0).sum())}')

    if mode == 'probe':
        for half in (300, 350, 400, 450, 500, 550, 600, 700):
            sub = crop(occ, half)
            free = int((sub == 0).sum())
            print(f'half={half:4d} -> {sub.shape[1]}x{sub.shape[0]}  free_cells={free}  '
                  f'free_states={free*60}  area={free*RES*RES:.0f} m^2')
        return

    # write mode
    half = int(sys.argv[3])
    out_dir = sys.argv[4] if len(sys.argv) > 4 else 'maps_lite'
    sub = crop(occ, half)  # world bottom-up
    sh, sw = sub.shape
    free = int((sub == 0).sum())
    print(f'lite grid: {sw}x{sh}  free_cells={free}  free_states={free*60}  area={free*RES*RES:.1f} m^2')

    # PGM image top-down: row r := pooled row (sh-1-r); free->255, blocked->0
    img = np.where(sub[::-1, :] == 0, 255, 0).astype(np.uint8)
    os.makedirs(out_dir, exist_ok=True)
    pgm = os.path.join(out_dir, 'map_tsudanuma_lite.pgm')
    yaml = os.path.join(out_dir, 'map_tsudanuma_lite.yaml')
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (sw, sh))
        f.write(img.tobytes())
    with open(yaml, 'w') as f:
        f.write('image: map_tsudanuma_lite.pgm\n')
        f.write(f'resolution: {RES:.6f}\n')
        f.write('origin: [0.000000, 0.000000, 0.000000]\n')
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    # goal = crop 中心 (世界座標)。bench_map/本家 とも origin 0。
    gx, gy = sw // 2, sh // 2
    gwx, gwy = (gx + 0.5) * RES, (gy + 0.5) * RES
    occ_c = sub[gy, gx]
    print(f'wrote {pgm}  ({sw}x{sh})')
    print(f'goal center cell ({gx},{gy}) world ({gwx:.3f},{gwy:.3f}) occ={"BLOCKED" if occ_c else "free"}')


if __name__ == '__main__':
    main()
