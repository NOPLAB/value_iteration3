#!/usr/bin/env python3
"""map_server 形式マップ (yaml+PGM) を ×scale で obstacle-dominant min-pool し、
本家 map_server が読める pooled PGM+YAML を出力する (ベクトル化版。
tsudanuma / tsukuba 共用)。

bench_map.rs::build_occupancy と bit 一致させる:
  - ow = ceil(w/scale), oh = ceil(h/scale)
  - 出力セルは world-block (scale×scale) 内に obstacle が 1 つでもあれば blocked
  - unknown は obstacle 扱い (--unknown obstacle)
  - occ index は world bottom-up。PGM 画像 row r には pooled row (oh-1-r) を書き、
    map_server の上下反転が occ[gy][gx] を再現する。
  - resolution / origin / negate / 閾値はソース yaml から読み、origin はそのまま
    継承 (本家 setStateValues の goal 円盤は origin 依存なので、goal world->cell
    が bench_map と一致するため)。

usage: pool_tsukuba.py SRC_YAML SCALE OUT_PGM OUT_YAML
"""
import os
import sys
import numpy as np


def load_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        _maxv = int(f.readline())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape((h, w))
    return w, h, data


def load_map_yaml(path):
    """map_server yaml の単純な key: value 行だけ読む (pyyaml 非依存)。"""
    m = {}
    with open(path) as f:
        for line in f:
            line = line.split('#', 1)[0].strip()
            if ':' in line:
                k, v = line.split(':', 1)
                m[k.strip()] = v.strip()
    return m


def main():
    src_yaml = sys.argv[1]
    scale = int(sys.argv[2])
    out_pgm = sys.argv[3]
    out_yaml = sys.argv[4]
    meta = load_map_yaml(src_yaml)
    src = os.path.join(os.path.dirname(src_yaml), meta['image'])
    full_res = float(meta['resolution'])
    origin = [float(x) for x in meta['origin'].strip('[]').split(',')]
    occupied_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.196))
    negate = int(meta.get('negate', 0))

    w, h, pix = load_pgm(src)
    p = pix.astype(np.float64)
    occ_prob = (p / 255.0) if negate else ((255.0 - p) / 255.0)
    is_obs = occ_prob > occupied_thresh
    is_free = occ_prob < free_thresh
    is_unknown = ~is_obs & ~is_free
    blocked_full = is_obs | is_unknown            # (h, w) top-down

    ow = -(-w // scale)
    oh = -(-h // scale)

    # world bottom-up: bu[iy] = blocked_full[h-1-iy]
    bu = blocked_full[::-1, :]
    # pad to (oh*scale, ow*scale) with False (out-of-range cells contribute nothing)
    padded = np.zeros((oh * scale, ow * scale), dtype=bool)
    padded[:h, :w] = bu
    occ = padded.reshape(oh, scale, ow, scale).any(axis=(1, 3))   # (oh, ow) bottom-up

    free_cells = int((~occ).sum())
    res = full_res * scale
    print(f'pooled grid: {ow}x{oh}  free_cells={free_cells}  '
          f'(scale={scale}, res={res})')

    # PGM image top-down: img[r] = occ[oh-1-r]; free->255, blocked->0
    img = np.where(occ[::-1, :], np.uint8(0), np.uint8(255))
    with open(out_pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (ow, oh))
        f.write(img.tobytes())
    with open(out_yaml, 'w') as f:
        f.write(f'image: {out_pgm.rsplit("/", 1)[-1]}\n')
        f.write(f'resolution: {res:.6f}\n')
        f.write(f'origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]\n')
        f.write(f'negate: {negate}\n')
        f.write(f'occupied_thresh: {occupied_thresh}\n')
        f.write(f'free_thresh: {free_thresh}\n')
    print(f'wrote {out_pgm} and {out_yaml}')

    # goal feasibility report for a few candidate goals (world -> pooled cell)
    for (gxw, gyw) in [(20.5, -1.0), (0.0, 0.0)]:
        gx = int((gxw - origin[0]) / res)
        gy = int((gyw - origin[1]) / res)
        if 0 <= gx < ow and 0 <= gy < oh:
            print(f'goal world({gxw},{gyw}) -> cell ({gx},{gy}) '
                  f'occ={"BLOCKED" if occ[gy, gx] else "free"}')
        else:
            print(f'goal world({gxw},{gyw}) -> cell ({gx},{gy}) OUT OF RANGE')


if __name__ == '__main__':
    main()
