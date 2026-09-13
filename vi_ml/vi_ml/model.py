"""U-Net value surrogate + the shared encoding of a (free, goal, boundary) sample.

Inputs (4 channels): free, goal mask, boundary value (normalised, where known),
boundary mask. The boundary channels let one model serve both a whole map
(boundary empty) and a window whose border values come from a coarser level —
that is what makes the pyramid rollout possible without a goal in the window.
Output: normalised log value; `decode` turns it back into seconds."""
from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

V_MAX_S = 5000.0  # log-scale ceiling; maze fields reach ~4000 s
_LOG_MAX = float(np.log1p(V_MAX_S))


def encode_value(v_s: np.ndarray) -> np.ndarray:
    return np.log1p(np.clip(np.nan_to_num(v_s, nan=0.0), 0, V_MAX_S)) / _LOG_MAX


def decode_value(y: np.ndarray) -> np.ndarray:
    return np.expm1(np.clip(y, 0, 1) * _LOG_MAX)


def encode(free: np.ndarray, goal: tuple[int, int] | None, boundary_s: np.ndarray | None = None) -> np.ndarray:
    """→ float32 (4, H, W). `boundary_s`: value in seconds where known, NaN elsewhere."""
    H, W = free.shape
    x = np.zeros((4, H, W), np.float32)
    x[0] = free
    if goal is not None:
        x[1, goal[0], goal[1]] = 1.0
    if boundary_s is not None:
        known = np.isfinite(boundary_s)
        x[2] = np.where(known, encode_value(boundary_s), 0.0)
        x[3] = known
    return x


def device() -> torch.device:
    """Training device: the DirectML adapter (Radeon iGPU) when torch-directml is
    installed and VI_ML_DEVICE is not "cpu". Inference stays on the CPU (faster at
    batch=1, and that is what the planner would run)."""
    import os
    if os.environ.get("VI_ML_DEVICE", "").lower() == "cpu":
        return torch.device("cpu")
    try:
        import torch_directml
        return torch_directml.device()
    except ImportError:
        return torch.device("cpu")


def cpu_state_dict(model: nn.Module) -> dict:
    return {k: v.detach().cpu() for k, v in model.state_dict().items()}


def _block(cin, cout):
    return nn.Sequential(nn.Conv2d(cin, cout, 3, padding=1), nn.BatchNorm2d(cout), nn.ReLU(inplace=True),
                         nn.Conv2d(cout, cout, 3, padding=1), nn.BatchNorm2d(cout), nn.ReLU(inplace=True))


class UNet(nn.Module):
    def __init__(self, cin: int = 4, base: int = 24, depth: int = 4, cout: int = 1):
        super().__init__()
        chs = [base * 2 ** i for i in range(depth)]
        self.enc = nn.ModuleList()
        c = cin
        for co in chs:
            self.enc.append(_block(c, co))
            c = co
        self.mid = _block(c, c * 2)
        self.up = nn.ModuleList()
        self.dec = nn.ModuleList()
        c = c * 2
        for co in reversed(chs):
            self.up.append(nn.ConvTranspose2d(c, co, 2, stride=2))
            self.dec.append(_block(co * 2, co))
            c = co
        self.head = nn.Conv2d(c, cout, 1)
        self.cout = cout

    def forward(self, x):
        skips = []
        for e in self.enc:
            x = e(x)
            skips.append(x)
            x = F.max_pool2d(x, 2)
        x = self.mid(x)
        for u, d, s in zip(self.up, self.dec, reversed(skips)):
            x = d(torch.cat([u(x), s], 1))
        y = self.head(x)
        return y[:, 0] if self.cout == 1 else y


def predict(model: nn.Module, x: np.ndarray) -> np.ndarray:
    """One sample (4,H,W) → value in seconds (H,W); obstacles come back as NaN."""
    model.eval()
    with torch.no_grad():
        y = model(torch.from_numpy(x)[None])[0].numpy()
    v = decode_value(y)
    v[x[0] == 0] = np.nan
    return v
