#!/usr/bin/env python3
"""Align ROS1/ROS2 value & policy dumps and emit a comparison report."""
import sys, json, os
import numpy as np

ROS2_UNREACH = 65535
ROS1_UNREACH = 1e6

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
    pearson = float(np.corrcoef(a, b)[0, 1]) if n > 1 and a.std() > 0 and b.std() > 0 else float('nan')
    ra = np.argsort(np.argsort(a))
    rb = np.argsort(np.argsort(b))
    spearman = float(np.corrcoef(ra, rb)[0, 1]) if n > 1 else float('nan')
    return dict(n=int(n), rmse=rmse, mae=mae, max_abs=max_abs,
                pearson=pearson, spearman=spearman)

def policy_agreement(pol1, pol2):
    valid = (pol1 >= 0) & (pol2 >= 0)
    if valid.sum() == 0:
        return float('nan')
    return float((pol1[valid] == pol2[valid]).mean())

def main():
    out_dir = sys.argv[1]
    v1 = np.load(os.path.join(out_dir, 'value_ros1.npy')).astype(np.float64)
    v2 = np.load(os.path.join(out_dir, 'value_ros2.npy')).astype(np.float64)
    p1 = np.load(os.path.join(out_dir, 'policy_ros1.npy')).astype(np.float64)
    p2 = np.load(os.path.join(out_dir, 'policy_ros2.npy')).astype(np.float64)

    u1 = v1 >= ROS1_UNREACH
    u2 = v2 >= ROS2_UNREACH
    v1a, tname = align(v1, v2, u1, u2)
    # apply the SAME transform to policy and unreachable mask
    p1a = _TRANSFORMS[tname](p1)
    u1a = _TRANSFORMS[tname](u1)

    reach = (~u1a) & (~u2)
    vm = value_metrics(v1a, v2, reach)
    pa_all = policy_agreement(p1a, p2)
    pa_t0 = policy_agreement(p1a[:, :, 0:1], p2[:, :, 0:1])

    t1 = json.load(open(os.path.join(out_dir, 'timing_ros1.json')))
    t2 = json.load(open(os.path.join(out_dir, 'timing_ros2.json')))

    lines = []
    lines.append("# VI 比較レポート (本家ROS1 vs vi_ros2 ROS2)\n")
    lines.append(f"- 整列変換 (ROS1→ROS2): **{tname}**")
    lines.append(f"- unreachable 一致率: {(u1a == u2).mean()*100:.2f}%  (整列の妥当性指標)\n")
    lines.append("## 速度\n")
    lines.append("| 側 | elapsed[s] | sweeps | converged | threads |")
    lines.append("|---|---|---|---|---|")
    lines.append(f"| ROS1(本家) | {t1['elapsed_sec']:.3f} | {t1['sweeps']} | {t1['converged']} | {t1['thread_num']} |")
    lines.append(f"| ROS2(vi_node) | {t2['elapsed_sec']:.3f} | {t2['sweeps']} | {t2['converged']} | {t2['thread_num']} |")
    speedup = (t1['elapsed_sec'] / t2['elapsed_sec']) if t2['elapsed_sec'] else float('nan')
    lines.append(f"\n- 速度比 (ROS1/ROS2): **{speedup:.2f}x**\n")
    lines.append("## 価値一致度 (両者可達セルのみ, ステップ単位)\n")
    lines.append(f"- 対象セル数: {vm['n']}")
    lines.append(f"- RMSE: {vm['rmse']:.4f},  MAE: {vm['mae']:.4f},  最大差: {vm['max_abs']:.4f}")
    lines.append(f"- Pearson: {vm['pearson']:.4f},  Spearman: {vm['spearman']:.4f}\n")
    lines.append("## 方策一致度 (両者可達セルのみ)\n")
    lines.append(f"- 全 theta: {pa_all*100:.2f}%")
    lines.append(f"- theta=0 スライス: {pa_t0*100:.2f}%")

    report = "\n".join(lines) + "\n"
    with open(os.path.join(out_dir, 'report.md'), 'w') as f:
        f.write(report)
    print(report)

if __name__ == '__main__':
    main()
