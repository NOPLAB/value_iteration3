#!/usr/bin/env python3
"""vi_reference (本家 u64 忠実移植) 比較ドライバ。

ros2 bench_client と **完全に同一の** map_server 意味論で house.pgm を OccupancyGrid 化し、
その raw int8 (h*w, row-major) を Rust ハーネス `vi_ref_bench` に渡す。ハーネスが
value_ref.npy / policy_ref.npy / timing_ref.json を out_dir に書く。

使い方:
  ref_bench.py <params.yaml> <map.pgm> <out_dir> <vi_ref_bench_bin>
"""
import sys, os, subprocess
import numpy as np
import yaml


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


def load_map_yaml(pgm_path):
    yaml_path = os.path.splitext(pgm_path)[0] + '.yaml'
    with open(yaml_path) as f:
        m = yaml.safe_load(f)
    origin = m.get('origin', [0.0, 0.0, 0.0])
    return dict(resolution=float(m['resolution']),
                ox=float(origin[0]), oy=float(origin[1]),
                occupied_thresh=float(m.get('occupied_thresh', 0.65)),
                free_thresh=float(m.get('free_thresh', 0.196)),
                negate=int(m.get('negate', 0)))


def to_occupancy(w, h, pgm, meta):
    """ros2 bench_client.to_occupancy と同一: map_server 意味論 + flipud。
    返り値は (h, w) int8 (0=free, 100=occ, -1=unknown), 行優先 (iy=0 が下端)。"""
    p = pgm.astype(np.float64)
    occ_prob = (p / 255.0) if meta['negate'] else ((255.0 - p) / 255.0)
    occ = np.full((h, w), -1, dtype=np.int8)
    occ[occ_prob < meta['free_thresh']] = 0
    occ[occ_prob > meta['occupied_thresh']] = 100
    occ = np.flipud(occ)  # ROS OccupancyGrid は原点左下・下から上
    return occ


def main():
    params_path, map_path, out_dir, bin_path = sys.argv[1:5]
    with open(params_path) as f:
        p = yaml.safe_load(f)
    w, h, pgm = load_pgm(map_path)
    meta = load_map_yaml(map_path)
    occ = to_occupancy(w, h, pgm, meta)  # (h, w) int8

    os.makedirs(out_dir, exist_ok=True)
    occ_raw = os.path.join(out_dir, 'occ_ref.raw')
    # row-major (C-order) int8 を書き出す。Rust 側は data[iy*w+ix] で参照。
    np.ascontiguousarray(occ, dtype=np.int8).tofile(occ_raw)

    g = p['goal']
    pl = p['planning']
    cl = p['client']
    cmd = [
        bin_path, occ_raw, str(w), str(h),
        repr(meta['resolution']), repr(meta['ox']), repr(meta['oy']),
        repr(float(g['x'])), repr(float(g['y'])), repr(float(g['yaw_deg'])),
        str(int(pl['theta_cell_num'])), repr(float(pl['safety_radius'])),
        repr(float(pl['safety_radius_penalty'])), repr(float(pl['goal_margin_radius'])),
        str(int(pl['goal_margin_theta'])),
        str(int(cl['max_sweeps'])), repr(float(cl['delta_threshold'])),
        out_dir,
    ]
    print('[ref_bench] running:', ' '.join(cmd), flush=True)
    subprocess.run(cmd, check=True)
    # cleanup intermediate
    try:
        os.remove(occ_raw)
    except OSError:
        pass
    print('[ref_bench] done -> %s/{value_ref,policy_ref}.npy, timing_ref.json' % out_dir)


if __name__ == '__main__':
    main()
