#!/usr/bin/env python3
"""3者 (本家ROS1 / f3d / ref) + ros2 のクロスチェック。

compare.py は常に本家ROS1 をベースラインに各 side を比較する。本スクリプトは追加で
ベースライン非依存のペア比較を行い、特に「f3d (Frontier3D 16bit) と ros2 (Reference 16bit) が
**同一の16bit固定点に bit 一致で到達するか**」を検証する。両者は vi_node の同一 bridge で
VIContext を構築しているため、ソルバ (Frontier3D vs Reference) が正しければ bit 一致するはず。

f3d / ros2 は共に vi_node::npy::write_u16 が書く (H, W, N_THETA) C-order u16 なので、整列不要で
直接比較できる。ref / ros1 は u64 忠実モデル (f64) なので 16bit 側との差はモデル差を含む。

  cross_check.py <out_dir>
"""
import sys, os
import numpy as np


def load(out_dir, name):
    return np.load(os.path.join(out_dir, name)).astype(np.float64)


def pair(a, b, reach=None):
    if reach is not None:
        a = a[reach]; b = b[reach]
    a = a.ravel(); b = b.ravel()
    diff = a - b
    n = a.size
    rmse = float(np.sqrt(np.mean(diff**2))) if n else float('nan')
    mae = float(np.mean(np.abs(diff))) if n else float('nan')
    mx = float(np.max(np.abs(diff))) if n else float('nan')
    exact = int(np.count_nonzero(diff == 0))
    return dict(n=int(n), rmse=rmse, mae=mae, max_abs=mx,
                exact_frac=exact / n if n else float('nan'))


def pol_agree(p1, p2, mask=None):
    valid = (p1 >= 0) & (p2 >= 0)
    if mask is not None:
        valid &= mask
    if valid.sum() == 0:
        return float('nan')
    return float((p1[valid] == p2[valid]).mean())


def main():
    out_dir = sys.argv[1]
    v_f3d = load(out_dir, 'value_f3d.npy')
    v_ros2 = load(out_dir, 'value_ros2.npy')
    p_f3d = load(out_dir, 'policy_f3d.npy')
    p_ros2 = load(out_dir, 'policy_ros2.npy')

    print("# クロスチェック: f3d(Frontier3D 16bit) vs ros2(Reference 16bit)\n")
    if v_f3d.shape != v_ros2.shape:
        print(f"!! shape mismatch: f3d={v_f3d.shape} ros2={v_ros2.shape}")
        return
    print(f"- 形状: {v_f3d.shape} (両者とも vi_node bridge の同一 VIContext)\n")

    # 16bit 双方の到達可能 (= obstacle/unreachable sentinel 65535 でない) セルで比較。
    reach = (v_f3d < 65535) & (v_ros2 < 65535)
    m = pair(v_f3d, v_ros2, reach)
    print("## 価値 (両者到達可能セル)")
    print(f"- 対象セル数: {m['n']}")
    print(f"- RMSE: {m['rmse']:.6f},  MAE: {m['mae']:.6f},  最大差: {m['max_abs']:.6f}")
    print(f"- 完全一致セル割合: {m['exact_frac']*100:.4f}%")
    bitexact_v = (v_f3d == v_ros2)
    print(f"- 全セル bit 一致: {'YES (Frontier3D ≡ Reference 固定点)' if bitexact_v.all() else 'NO'}"
          f"  (不一致セル {int((~bitexact_v).sum())})\n")

    print("## 方策")
    pa = pol_agree(p_f3d, p_ros2)
    print(f"- 方策一致率 (両者有効): {pa*100:.4f}%")
    bitexact_p = (p_f3d == p_ros2)
    print(f"- 全セル bit 一致: {'YES' if bitexact_p.all() else 'NO'}"
          f"  (不一致セル {int((~bitexact_p).sum())})\n")

    # 参考: f3d vs ref(u64) の到達可能集合差 (モデル差) — 本家との比較は compare.py 参照。
    if os.path.exists(os.path.join(out_dir, 'value_ref.npy')):
        v_ref = load(out_dir, 'value_ref.npy')
        if v_ref.shape == v_f3d.shape:
            r_f3d = int((v_f3d < 65535).sum())
            r_ref = int((v_ref < 1e6).sum())
            print("## 参考: 到達可能セル数 (モデル差)")
            print(f"- f3d (16bit, <65535):     {r_f3d}")
            print(f"- ref (u64,   <1e6 step):  {r_ref}")


if __name__ == '__main__':
    main()
