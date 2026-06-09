#!/usr/bin/env python3
"""u64 高速ソルバ群 (vi_reference solvers) vs 本家ROS1 の一覧レポート report_u64.md を生成。

compare.py のヘルパ (align / value_metrics / policy_agreement) を再利用し、各 u64 ソルバの
value_<solver>.npy / policy_<solver>.npy を本家 value_ros1.npy と比較。厳密ソルバなので
RMSE 0 / 方策 100% (bit-exact) を期待。速度は本家比。

  make_u64_report.py <out_dir>
"""
import sys, os, json
import numpy as np
import compare as C

SOLVERS = ['reference', 'frontier3d', 'frontier2d', 'frontier2d_soa', 'frontier2d_pad', 'frontier2d_par', 'frontier_stack', 'block_refine', 'pyramid_sweep']
ROS1_UNREACH = 1e6  # u64 モデル sentinel 検出閾値


def main():
    out_dir = sys.argv[1]
    v1 = np.load(os.path.join(out_dir, 'value_ros1.npy')).astype(np.float64)
    p1 = np.load(os.path.join(out_dir, 'policy_ros1.npy')).astype(np.float64)
    with open(os.path.join(out_dir, 'timing_ros1.json')) as f:
        t1 = json.load(f)
    t1_elapsed = float(t1['elapsed_sec'])

    rows = []
    for s in SOLVERS:
        vpath = os.path.join(out_dir, f'value_{s}.npy')
        if not os.path.exists(vpath):
            continue
        v2 = np.load(vpath).astype(np.float64)
        p2 = np.load(os.path.join(out_dir, f'policy_{s}.npy')).astype(np.float64)
        with open(os.path.join(out_dir, f'timing_{s}.json')) as f:
            t2 = json.load(f)

        u1 = v1 >= ROS1_UNREACH
        u2 = v2 >= 1e6
        v1a, tname = C.align(v1, v2, u1, u2)
        p1a = C._TRANSFORMS[tname](p1)
        u1a = C._TRANSFORMS[tname](u1)
        reach = (~u1a) & (~u2)
        vm = C.value_metrics(v1a, v2, reach)
        pa = C.policy_agreement(p1a, p2)
        bit_exact = (vm['rmse'] == 0.0 and abs(pa - 1.0) < 1e-12)
        elapsed = float(t2['elapsed_sec'])
        speedup = t1_elapsed / elapsed if elapsed else float('nan')
        rows.append(dict(solver=s, elapsed=elapsed, iters=int(t2.get('iters', t2.get('sweeps', 0))),
                         updates=int(t2.get('updates', 0)), speedup=speedup,
                         rmse=vm['rmse'], policy=pa, converged=bool(t2.get('converged', False)),
                         bit_exact=bit_exact, align=tname))

    lines = []
    lines.append("# u64 高速ソルバ群 vs 本家ROS1 — bit-exact & 速度\n")
    lines.append(f"house.pgm (384×384×60)。本家 elapsed={t1_elapsed:.3f}s (単スレッド)。"
                 "ソルバは原則単スレッド、`frontier2d_par` のみ決定的マルチスレッド (Jacobi) で例外。")
    lines.append("各ソルバは本家と同一 u64 コストモデル上で frontier/block を走らせる "
                 "(vi_reference solvers)。厳密ソルバなので RMSE 0 / 方策 100% を期待。\n")
    lines.append("| ソルバ | elapsed[s] | 反復 | updates | 本家比速度 | RMSE | 方策一致 | converged | bit-exact |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        lines.append(
            f"| {r['solver']} | {r['elapsed']:.3f} | {r['iters']} | {r['updates']} | "
            f"{r['speedup']:.2f}x | {r['rmse']:.4f} | {r['policy']*100:.2f}% | "
            f"{r['converged']} | {'✓' if r['bit_exact'] else '✗'} |")
    lines.append("\n- 整列はすべて本家へ identity（u64 reference は既存 ref と同一出力）。")
    lines.append("- bit-exact (RMSE 0 & 方策 100%) = 高速アルゴリズムが本家の固定点に完全一致。")
    lines.append("- 速度は更新順序の違いのみ（コスト数式は本家と同一）。")

    report = "\n".join(lines) + "\n"
    with open(os.path.join(out_dir, 'report_u64.md'), 'w') as f:
        f.write(report)
    print(report)


if __name__ == '__main__':
    main()
