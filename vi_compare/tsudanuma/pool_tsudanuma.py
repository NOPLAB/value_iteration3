#!/usr/bin/env python3
"""map_tsudanuma.pgm を ×scale で min-pool (obstacle-dominant) し、本家 map_server が
読める 0.15 m/cell PGM+YAML を出力する。プーリングは vi_bench/bench_map.rs::build_occupancy
と bit 一致させる:
  - ow = ceil(w/scale), oh = ceil(h/scale)
  - 出力セルは scale×scale ブロック内に obstacle が 1 つでもあれば blocked(=100)、なければ free(=0)
  - unknown(-1) は obstacle 扱い (paper config: --unknown obstacle)
  - occ index は world bottom-up (oy=0 が世界下端)。map_server は画像を上下反転して
    grid(0,0)=左下にするので、PGM 画像 row r には pooled row (oh-1-r) を書く。

出力 PGM 画素: free=255 (white), blocked=0 (black) → map_server 既定閾値で
  occ_prob=(255-p)/255: free if <0.196 (p>205), obstacle if >0.65 (p<89) と一致。
"""
import sys, os
import numpy as np


def load_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxv = int(f.readline())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape((h, w))
    return w, h, maxv, data


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else '../../assets/map_tsudanuma.pgm'
    scale = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    out_dir = sys.argv[3] if len(sys.argv) > 3 else 'maps_processed'
    # full-res yaml (resolution/origin/thresholds)
    full_res = 0.05
    origin = (-100.0, -100.0, 0.0)
    occupied_thresh = 0.65
    free_thresh = 0.196
    negate = 0

    w, h, _maxv, pix = load_pgm(src)
    # classify full-res pixels: occ_prob, obstacle/free/unknown
    p = pix.astype(np.float64)
    occ_prob = (p / 255.0) if negate else ((255.0 - p) / 255.0)
    is_obs_full = (occ_prob > occupied_thresh)
    is_free_full = (occ_prob < free_thresh)
    is_unknown_full = ~is_obs_full & ~is_free_full
    # unknown -> obstacle
    blocked_full = is_obs_full | is_unknown_full   # shape (h, w), PGM top-down

    ow = -(-w // scale)  # ceil
    oh = -(-h // scale)
    # bench_map: iy is world bottom-up; src_row = h-1-iy (top-down). Build occ[oy][ox]
    # in world bottom-up orientation = "blocked if ANY source in block".
    occ = np.zeros((oh, ow), dtype=np.uint8)  # 0 free, 1 blocked, world bottom-up
    for oy in range(oh):
        for ox in range(ow):
            iy0 = oy * scale
            ix0 = ox * scale
            # world rows iy in [iy0, iy0+scale) -> top-down src rows h-1-iy
            iy1 = min(iy0 + scale, h)
            ix1 = min(ix0 + scale, w)
            # top-down rows for this world block:
            src_rows = [h - 1 - iy for iy in range(iy0, iy1)]
            block = blocked_full[np.array(src_rows)[:, None], np.arange(ix0, ix1)[None, :]]
            occ[oy, ox] = 1 if block.any() else 0

    free_cells = int((occ == 0).sum())
    print(f'pooled grid: {ow}x{oh}  free_cells={free_cells}  (scale={scale}, res={full_res*scale})')

    # PGM image (top-down): image row r := pooled row (oh-1-r), so map_server's flip
    # reproduces occ[gy][gx] exactly. free->255, blocked->0.
    img = np.empty((oh, ow), dtype=np.uint8)
    for r in range(oh):
        img[r, :] = np.where(occ[oh - 1 - r, :] == 0, 255, 0)

    os.makedirs(out_dir, exist_ok=True)
    pgm_path = os.path.join(out_dir, 'map_tsudanuma_015.pgm')
    yaml_path = os.path.join(out_dir, 'map_tsudanuma_015.yaml')
    with open(pgm_path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (ow, oh))
        f.write(img.tobytes())
    res = full_res * scale
    with open(yaml_path, 'w') as f:
        f.write(f'image: map_tsudanuma_015.pgm\n')
        f.write(f'resolution: {res:.6f}\n')
        f.write(f'origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]\n')
        f.write(f'negate: {negate}\n')
        f.write(f'occupied_thresh: {occupied_thresh}\n')
        f.write(f'free_thresh: {free_thresh}\n')
    print(f'wrote {pgm_path} and {yaml_path}')

    # goal (0,0) feasibility in pooled grid (world bottom-up occ)
    gx = int((0.0 - origin[0]) / res)
    gy = int((0.0 - origin[1]) / res)
    if 0 <= gx < ow and 0 <= gy < oh:
        print(f'goal world(0,0) -> cell ({gx},{gy}) occ={"BLOCKED" if occ[gy,gx] else "free"}')
    else:
        print(f'goal world(0,0) -> cell ({gx},{gy}) OUT OF RANGE')


if __name__ == '__main__':
    main()
