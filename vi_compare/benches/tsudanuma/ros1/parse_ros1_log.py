#!/usr/bin/env python3
"""ros1_run.log から最終 sweep/delta/elapsed を抽出し ros1_parallel.{json,csv} を生成。

bench_client_tsudanuma.py が json を書く前に手動停止 (docker stop) した場合の保険。
クライアントの feedback ログ行 `t=<sec>s sweep=<n> max_delta=<v>` をパースする。
converged は最終 max_delta==0 のときのみ True。
"""
import re, json, os, sys

D = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(D, '../../../results/tsudanuma')
LOG = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RES, 'logs', 'ros1_run.log')
THREAD_NUM = int(sys.argv[2]) if len(sys.argv) > 2 else 16

pat = re.compile(r't=([0-9.]+)s sweep=([0-9-]+) max_delta=([0-9.eE+-]+)')
last = None
with open(LOG) as f:
    for line in f:
        m = pat.search(line)
        if m:
            last = (float(m.group(1)), int(m.group(2)), float(m.group(3)))

if last is None:
    print('no feedback lines found; node may not have started VI yet')
    sys.exit(1)

elapsed, sweeps, delta = last
converged = (delta == 0.0)
timing = dict(elapsed_sec=elapsed, sweeps=sweeps, converged=converged,
              last_max_delta=delta, thread_num=THREAD_NUM, delta_threshold=0,
              goal=[147.375, 100.125, 0], side='ros1',
              map='map_tsudanuma_015 (0.15m, scale3)',
              note='parsed from ros1_run.log (manual stop); elapsed = last feedback timestamp')
os.makedirs(RES, exist_ok=True)
with open(os.path.join(RES, 'ros1_parallel.json'), 'w') as f:
    json.dump(timing, f, indent=2)
with open(os.path.join(RES, 'ros1_parallel.csv'), 'w') as f:
    f.write('solver,sweeps,elapsed_sec,converged,thread_num\n')
    f.write('ros1_parallel,%d,%.3f,%s,%d\n' % (sweeps, elapsed, 'Y' if converged else 'N', THREAD_NUM))
print(json.dumps(timing, indent=2))
