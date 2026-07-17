#!/usr/bin/env python3
"""ベンチ CSV (rep 列付き/なし両対応) を m ごとに集計して mean±std を表示する。

usage: aggregate_stats.py CSV [value_column]
  value_column 省略時: total_s (vi_rs) → elapsed_sec (ROS1) の順で自動検出。
コメント行 (#) はスキップ。出力例:
  m= 8 n=10 mean=4.236 std=0.112 min=4.05 max=4.41 conv=Y
"""
import csv
import statistics
import sys


def main():
    path = sys.argv[1]
    col = sys.argv[2] if len(sys.argv) > 2 else None
    with open(path) as f:
        rows = list(csv.DictReader(r for r in f if not r.startswith('#')))
    if not rows:
        print(f'{path}: no rows')
        return
    if col is None:
        col = 'total_s' if 'total_s' in rows[0] else 'elapsed_sec'
    bym = {}
    for r in rows:
        bym.setdefault(int(r['m']), []).append(r)
    print(f'{path}  ({col}, {len(rows)} runs)')
    for m in sorted(bym):
        v = [float(r[col]) for r in bym[m]]
        conv = 'Y' if all(r.get('converged', 'Y') == 'Y' for r in bym[m]) else 'N!'
        std = statistics.stdev(v) if len(v) > 1 else 0.0
        print(f'  m={m:2d} n={len(v):2d} mean={statistics.mean(v):8.3f} std={std:6.3f} '
              f'min={min(v):7.2f} max={max(v):7.2f} conv={conv}')


if __name__ == '__main__':
    main()
