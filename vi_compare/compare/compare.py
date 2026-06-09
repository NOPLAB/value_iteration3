#!/usr/bin/env python3
"""Align ROS1 と 2nd-side (ros2=vi_node / ref=vi_reference) の value & policy を
整列して比較レポートを出す。

  compare.py <out_dir> [side]   # side: ros2 (既定) | ref
"""
import sys, json, os
import numpy as np

ROS1_UNREACH = 1e6   # detection threshold; actual ROS1 sentinel is ~1e9 (max_cost_/prob_base_)

# 2nd-side ごとの設定。ros2=vi_node(16bit), ref=vi_reference(u64 忠実移植)。
SIDES = {
    'ros2': dict(vfile='value_ros2.npy', pfile='policy_ros2.npy', tfile='timing_ros2.json',
                 unreach=65535, label='ROS2(vi_node)', report='report.md',
                 model_note='ROS2 は 16bit 飽和で障害物のみ sentinel'),
    'ref':  dict(vfile='value_ref.npy', pfile='policy_ref.npy', tfile='timing_ref.json',
                 unreach=1e6, label='ref(vi_reference u64忠実)', report='report_ref.md',
                 model_note='ref は本家と同じ u64+sentinel(~1e9) モデル → 価値はほぼ一致するはず'),
    'f3d':  dict(vfile='value_f3d.npy', pfile='policy_f3d.npy', tfile='timing_f3d.json',
                 unreach=65535, label='f3d(Frontier3D 16bit)', report='report_f3d.md',
                 model_note='f3d は vi_rs の Frontier3D ソルバ・16bit 飽和で障害物のみ sentinel'
                            '（ros2 reference と同一の数値モデルで、ソルバのみ異なる実装）'),
}

# vi_u64_bench が出す u64 高速ソルバ群。本家と同一の u64 コストモデル上で frontier/block を
# 走らせるため、厳密ソルバは本家と bit-exact（RMSE 0）になるはず。出力名は value_<solver>.npy。
_U64_SOLVERS = ['reference', 'frontier3d', 'frontier2d', 'frontier2d_soa', 'frontier2d_pad',
                'frontier2d_par', 'frontier_stack', 'block_refine', 'pyramid_sweep']
for _s in _U64_SOLVERS:
    SIDES[_s] = dict(
        vfile=f'value_{_s}.npy', pfile=f'policy_{_s}.npy', tfile=f'timing_{_s}.json',
        unreach=1e6, label=f'u64:{_s}', report=f'report_u64_{_s}.md',
        model_note='本家と同一 u64 コストモデル（vi_reference）上の高速ソルバ → bit-exact を期待')

# 8 dihedral spatial transforms on the (H, W) plane (theta axis preserved).
# Order matters: when two transforms tie on the unreachable-mask score, the
# first one in this dict wins (strict < comparison).  Simple/natural transforms
# (identity, flips, transpose) are listed before rotations so that the
# semantically correct alignment is preferred on symmetric grids.
_TRANSFORMS = {
    'identity':      lambda a: a,
    'fliplr':        lambda a: a[:, ::-1, :],
    'flipud':        lambda a: a[::-1, :, :],
    'transpose':     lambda a: np.transpose(a, (1, 0, 2)),
    'antitranspose': lambda a: np.transpose(a, (1, 0, 2))[::-1, ::-1, :],
    'rot90':         lambda a: np.rot90(a, 1, axes=(0, 1)),
    'rot180':        lambda a: np.rot90(a, 2, axes=(0, 1)),
    'rot270':        lambda a: np.rot90(a, 3, axes=(0, 1)),
}

def align(ros1, ros2, ros1_unreach, ros2_unreach):
    """Find spatial transform of ros1 that best matches ros2's unreachable mask.
    Returns (transformed_ros1, transform_name)."""
    best_name, best_disagree = 'identity', 1.0
    scores = {}
    for name, fn in _TRANSFORMS.items():
        if fn(ros1_unreach).shape != ros2_unreach.shape:
            continue
        disagree = (fn(ros1_unreach) != ros2_unreach).mean()
        scores[name] = disagree
        if disagree < best_disagree:
            best_disagree, best_name = disagree, name
    # sanity: best should be clearly better than 2nd best (unless near-perfect)
    ordered = sorted(scores.values())
    if len(ordered) > 1 and best_disagree > 0.02 and (ordered[1] - ordered[0]) < 0.01:
        print(f"WARN: ambiguous orientation (scores={scores})", file=sys.stderr)
    return _TRANSFORMS[best_name](ros1), best_name

def value_metrics(ros1, ros2, reach):
    a = ros1[reach].astype(np.float64)
    b = ros2[reach].astype(np.float64)
    n = a.size
    if n == 0:
        return dict(n=0, rmse=float('nan'), mae=float('nan'),
                    max_abs=float('nan'), pearson=float('nan'), spearman=float('nan'))
    diff = a - b
    rmse = float(np.sqrt(np.mean(diff ** 2)))
    mae = float(np.mean(np.abs(diff)))
    max_abs = float(np.max(np.abs(diff)))
    nondegenerate = n > 1 and a.std() > 0 and b.std() > 0
    pearson = float(np.corrcoef(a, b)[0, 1]) if nondegenerate else float('nan')
    # ordinal tie-breaking (no scipy): slightly differs from avg-rank Spearman on tied data
    ra = np.argsort(np.argsort(a))
    rb = np.argsort(np.argsort(b))
    spearman = float(np.corrcoef(ra, rb)[0, 1]) if nondegenerate else float('nan')
    return dict(n=int(n), rmse=rmse, mae=mae, max_abs=max_abs,
                pearson=pearson, spearman=spearman)

def policy_agreement(pol1, pol2):
    valid = (pol1 >= 0) & (pol2 >= 0)
    if valid.sum() == 0:
        return float('nan')
    return float((pol1[valid] == pol2[valid]).mean())

def directional_unreach_agreement(u_small, u_big):
    """Fraction of the smaller unreachable set that is also unreachable on the
    other side. Robust orientation/obstacle-alignment check: obstacles are
    unreachable on BOTH sides, so this should be ~1.0 when correctly aligned,
    regardless of the two sides' differing 'far-unreachable' semantics.
    Pass the side with FEWER unreachable cells as u_small."""
    n = int(u_small.sum())
    if n == 0:
        return float('nan')
    return float((u_small & u_big).sum()) / n

def main():
    out_dir = sys.argv[1]
    side = sys.argv[2] if len(sys.argv) > 2 else 'ros2'
    if side not in SIDES:
        print(f"unknown side '{side}' (choose: {', '.join(SIDES)})", file=sys.stderr)
        sys.exit(2)
    cfg = SIDES[side]

    v1 = np.load(os.path.join(out_dir, 'value_ros1.npy')).astype(np.float64)
    v2 = np.load(os.path.join(out_dir, cfg['vfile'])).astype(np.float64)
    p1 = np.load(os.path.join(out_dir, 'policy_ros1.npy')).astype(np.float64)
    p2 = np.load(os.path.join(out_dir, cfg['pfile'])).astype(np.float64)

    u1 = v1 >= ROS1_UNREACH
    u2 = v2 >= cfg['unreach']
    v1a, tname = align(v1, v2, u1, u2)
    # apply the SAME transform to policy and unreachable mask
    p1a = _TRANSFORMS[tname](p1)
    u1a = _TRANSFORMS[tname](u1)

    reach = (~u1a) & (~u2)
    vm = value_metrics(v1a, v2, reach)
    pa_all = policy_agreement(p1a, p2)
    pa_t0 = policy_agreement(p1a[:, :, 0:1], p2[:, :, 0:1])

    n_u1 = int(u1a.sum())
    n_u2 = int(u2.sum())
    n_reach = int(reach.sum())
    total = u2.size
    if n_u1 <= n_u2:
        u_small, u_other = u1a, u2
    else:
        u_small, u_other = u2, u1a
    align_ok = directional_unreach_agreement(u_small, u_other)

    with open(os.path.join(out_dir, 'timing_ros1.json')) as f:
        t1 = json.load(f)
    with open(os.path.join(out_dir, cfg['tfile'])) as f:
        t2 = json.load(f)

    label2 = cfg['label']
    lines = []
    lines.append(f"# VI 比較レポート (本家ROS1 vs {label2})\n")
    lines.append(f"- 整列変換 (ROS1→2nd): **{tname}**")
    lines.append(f"- 整列確認 (小さい方の到達不能集合が他方でも到達不能な割合): {align_ok*100:.2f}%  (~100%なら整列OK)")
    lines.append(f"- 到達不能セル一致率(参考): {(u1a==u2).mean()*100:.2f}%\n")
    lines.append("## 到達可能性 (モデル差)\n")
    lines.append(f"- ROS1(本家) 到達可能セル: {total-n_u1} / {total}")
    lines.append(f"- {label2} 到達可能セル: {total-n_u2} / {total}")
    lines.append(f"- 両者で到達可能(価値比較の対象): {n_reach}")
    lines.append(f"- 注: ROS1 は u64 + sentinel(~1e9) で「ゴールへの有限経路なし」を到達不能とする。{cfg['model_note']}。\n")
    lines.append("## 速度\n")
    lines.append("| 側 | elapsed[s] | sweeps | converged | threads |")
    lines.append("|---|---|---|---|---|")
    lines.append(f"| ROS1(本家) | {t1['elapsed_sec']:.3f} | {t1['sweeps']} | {t1['converged']} | {t1['thread_num']} |")
    lines.append(f"| {label2} | {t2['elapsed_sec']:.3f} | {t2['sweeps']} | {t2['converged']} | {t2['thread_num']} |")
    speedup = (t1['elapsed_sec'] / t2['elapsed_sec']) if t2['elapsed_sec'] else float('nan')
    lines.append(f"\n- 速度比 (ROS1/2nd): **{speedup:.2f}x**\n")
    lines.append("## 価値一致度 (両者可達セルのみ, ステップ単位)\n")
    lines.append(f"- 対象セル数: {vm['n']}")
    lines.append(f"- RMSE: {vm['rmse']:.4f},  MAE: {vm['mae']:.4f},  最大差: {vm['max_abs']:.4f}")
    lines.append(f"- Pearson: {vm['pearson']:.4f},  Spearman: {vm['spearman']:.4f}\n")
    lines.append("## 方策一致度 (両者可達セルのみ)\n")
    lines.append(f"- 全 theta: {pa_all*100:.2f}%")
    lines.append(f"- theta=0 スライス: {pa_t0*100:.2f}%")

    report = "\n".join(lines) + "\n"
    with open(os.path.join(out_dir, cfg['report']), 'w') as f:
        f.write(report)
    print(report)

if __name__ == '__main__':
    main()
