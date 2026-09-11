import numpy as np
import pytest
import torch

from vi_ml import maps, rollout, vi
from vi_ml.model import UNet, decode_value, encode, encode_value


def test_generators_have_walls_and_free_space():
    rng = np.random.default_rng(0)
    for k, gen in maps.KINDS.items():
        f = maps.thicken(gen(rng, 64))
        assert f.shape == (64, 64) and not f[0].any() and not f[:, 0].any(), k
        assert 0.15 < f.mean() < 0.95, k


def test_thicken_and_component():
    f = np.ones((8, 8), bool)
    f[4, :] = False           # 1-cell wall
    t = maps.thicken(f)
    assert not t[3:6].any()   # now 3 thick
    c = maps.largest_component(t, (1, 1))
    assert c[1:3].any() and not c[6:].any()


def test_greedy_reaches_goal_on_a_distance_field():
    yy, xx = np.mgrid[0:16, 0:16]
    v = np.hypot(yy - 8.0, xx - 8.0)
    ok, p = rollout.greedy(v, (0, 0), v == 0)
    assert ok and p[-1] == (8, 8) and len(p) == 4  # 3-cell jumps


def test_value_codec_roundtrip():
    v = np.array([0.0, 1.0, 100.0, 4000.0, np.nan], np.float32)
    back = decode_value(encode_value(v))
    assert np.allclose(back[:4], v[:4], rtol=1e-3)


def test_unet_shape():
    m = UNet()
    x = torch.from_numpy(encode(np.ones((64, 64), bool), (3, 3)))[None]
    assert m(x).shape == (1, 64, 64)


@pytest.mark.skipif(not vi.BENCH_MAP.exists(), reason="bench_map not built")
def test_exact_solver_roundtrip():
    rng = np.random.default_rng(0)
    f = maps.thicken(maps.rooms(rng, 32))
    g = maps.pick_free(rng, f)
    f = maps.largest_component(f, g)
    v, ms = vi.solve(f, g)
    assert v.shape == f.shape and v[g] == 0 and ms > 0
    assert np.isfinite(v[f]).mean() > 0.9 and np.isnan(v[~f]).all()
    ok, _ = rollout.greedy(v, maps.pick_free(rng, f & np.isfinite(v)), v == 0)
    assert ok


def test_ddpm_sample_shape_and_range():
    from vi_ml.diffusion import DDPM
    m = DDPM().eval()
    cond = torch.from_numpy(encode(np.ones((64, 64), bool), (3, 3)))[None]
    x = m.sample(cond, steps=2)
    assert x.shape == (1, 64, 64) and torch.isfinite(x).all()
