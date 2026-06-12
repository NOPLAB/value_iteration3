#!/usr/bin/env python3
"""vi_reference u64 高速ソルバ 比較ドライバ。

ros2 bench_client / ref_bench と **完全に同一の** map_server 意味論で house.pgm を
OccupancyGrid 化し、その raw int8 (h*w, row-major) を Rust ハーネス `vi_u64_bench` に
指定ソルバで渡す。ハーネスが value_<solver>.npy / policy_<solver>.npy / timing_<solver>.json
を out_dir に書く。

使い方:
  u64_bench.py <solver> <params.yaml> <map.pgm> <out_dir> <vi_u64_bench_bin>
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
    """ros2 bench_client / ref_bench の to_occupancy と同一。返り値 (h,w) int8。"""
    p = pgm.astype(np.float64)
    occ_prob = (p / 255.0) if meta['negate'] else ((255.0 - p) / 255.0)
    occ = np.full((h, w), -1, dtype=np.int8)
    occ[occ_prob < meta['free_thresh']] = 0
    occ[occ_prob > meta['occupied_thresh']] = 100
    occ = np.flipud(occ)
    return occ


def main():
    solver, params_path, map_path, out_dir, bin_path = sys.argv[1:6]
    with open(params_path) as f:
        p = yaml.safe_load(f)
    w, h, pgm = load_pgm(map_path)
    meta = load_map_yaml(map_path)
    occ = to_occupancy(w, h, pgm, meta)

    os.makedirs(out_dir, exist_ok=True)
    occ_raw = os.path.join(out_dir, f'occ_{solver}.raw')
    np.ascontiguousarray(occ, dtype=np.int8).tofile(occ_raw)

    g = p['goal']
    pl = p['planning']
    cl = p['client']
    cmd = [
        bin_path, solver, occ_raw, str(w), str(h),
        repr(meta['resolution']), repr(meta['ox']), repr(meta['oy']),
        repr(float(g['x'])), repr(float(g['y'])), repr(float(g['yaw_deg'])),
        str(int(pl['theta_cell_num'])), repr(float(pl['safety_radius'])),
        repr(float(pl['safety_radius_penalty'])), repr(float(pl['goal_margin_radius'])),
        str(int(pl['goal_margin_theta'])),
        str(int(cl['max_sweeps'])),
        out_dir,
    ]
    print('[u64_bench] running:', ' '.join(cmd), flush=True)
    subprocess.run(cmd, check=True)
    try:
        os.remove(occ_raw)
    except OSError:
        pass
    print('[u64_bench] done -> %s/{value,policy}_%s.npy, timing_%s.json' % (out_dir, solver, solver))


if __name__ == '__main__':
    main()
