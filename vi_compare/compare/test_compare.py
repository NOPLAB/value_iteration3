#!/usr/bin/env python3
import numpy as np
import compare as C

def test_orientation_recovers_transpose():
    rng = np.random.default_rng(0)
    H = W = 6; T = 3
    ros2 = rng.integers(0, 50, size=(H, W, T)).astype(np.float64)
    unreach2 = np.zeros((H, W, T), bool)
    unreach2[0, :, :] = True  # a distinctive border
    ros2[unreach2] = 65535
    # ros1 is ros2 spatially transposed + value sentinel 1e9
    ros1 = np.transpose(ros2.copy(), (1, 0, 2))
    ros1[ros1 >= 65535] = 1e9
    aligned, name = C.align(ros1, ros2, ros1_unreach=ros1 >= 1e6, ros2_unreach=ros2 >= 65535)
    assert name == 'transpose', name
    # after alignment the unreachable borders coincide
    assert ((aligned >= 1e6) == (ros2 >= 65535)).mean() > 0.99

def test_value_metrics_identity():
    H = W = 5; T = 2
    a = np.arange(H * W * T, dtype=np.float64).reshape(H, W, T)
    m = C.value_metrics(a, a, reach=np.ones((H, W, T), bool))
    assert abs(m['rmse']) < 1e-9
    assert abs(m['pearson'] - 1.0) < 1e-9

def test_policy_agreement():
    a = np.array([[[0, 1], [2, -1]]], dtype=np.float64)   # shape (1,2,2)
    b = np.array([[[0, 3], [2, -1]]], dtype=np.float64)
    # valid cells (both>=0): (0,0,0)=0==0 ok, (0,0,1)=1!=3, (0,1,0)=2==2 ok ; (0,1,1) excluded(-1)
    assert abs(C.policy_agreement(a, b) - (2 / 3)) < 1e-9

if __name__ == '__main__':
    test_orientation_recovers_transpose()
    test_value_metrics_identity()
    test_policy_agreement()
    print("OK")
