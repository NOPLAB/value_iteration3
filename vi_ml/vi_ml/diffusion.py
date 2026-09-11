"""Conditional diffusion (DDPM-trained, DDIM-sampled) over the normalised value
field, conditioned on the same 4 channels the U-Net regressor sees. Same U-Net
body; the noisy field and a constant t/T plane are appended as 2 extra input
channels and the net predicts the clean field x0 directly (a regression target
that stays sane at few sampling steps).

    python -m vi_ml.diffusion train out/train64.npz --out out/ddpm.pt
    python -m vi_ml.diffusion eval  out/train64.npz out/ddpm.pt --steps 20
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch

from .model import UNet, decode_value, encode
from .train import masked_l1, whole_samples

T = 1000


def alpha_bars(n: int = T) -> torch.Tensor:
    """Cosine schedule (Nichol & Dhariwal) → ᾱ_t for t = 0..n (ᾱ_0 = 1)."""
    s = 0.008
    t = torch.linspace(0, n, n + 1) / n
    f = torch.cos((t + s) / (1 + s) * np.pi / 2) ** 2
    return f / f[0]


class DDPM(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.net = UNet(cin=6)
        self.register_buffer("ab", alpha_bars())

    def forward(self, cond, x_t, t):
        """cond (B,4,H,W), x_t (B,H,W) in [-1,1], t (B,) ints → x0 prediction (B,H,W)."""
        tt = (t.float() / T)[:, None, None, None].expand(-1, 1, *x_t.shape[1:])
        return self.net(torch.cat([cond, x_t[:, None], tt], 1))

    @torch.no_grad()
    def sample(self, cond, steps: int = 20, seed: int = 0):
        """Deterministic DDIM (η=0) from pure noise → x0 in [-1,1]."""
        g = torch.Generator().manual_seed(seed)
        B, _, H, W = cond.shape
        x = torch.randn(B, H, W, generator=g)
        ts = torch.linspace(T, 0, steps + 1).round().long()
        for t, t_prev in zip(ts[:-1], ts[1:]):
            ab, ab_prev = self.ab[t], self.ab[t_prev]
            x0 = self(cond, x, t.expand(B)).clamp(-1, 1)
            eps = (x - ab.sqrt() * x0) / (1 - ab).sqrt()
            x = ab_prev.sqrt() * x0 + (1 - ab_prev).sqrt() * eps
        return x


def predict(model: DDPM, x: np.ndarray, steps: int = 20) -> np.ndarray:
    """One sample (4,H,W) → value in seconds (H,W); obstacles NaN."""
    model.eval()
    y = model.sample(torch.from_numpy(x)[None], steps)[0].numpy()
    v = decode_value((y + 1) / 2)
    v[x[0] == 0] = np.nan
    return v


def train(xs, ys, ws, epochs: int, out: Path, val_frac: float = 0.1, seed: int = 0, batch: int = 32):
    torch.manual_seed(seed)
    idx = np.random.default_rng(seed).permutation(len(xs))
    nv = int(len(xs) * val_frac)
    va, tr = idx[:nv], idx[nv:]
    X, Y, Wt = (torch.from_numpy(a) for a in (xs, ys * 2 - 1, ws))  # field in [-1,1]
    model = DDPM()
    opt = torch.optim.AdamW(model.net.parameters(), 1e-3, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, 2e-3, total_steps=epochs * ((len(tr) + batch - 1) // batch))

    def step(b, train_mode):
        cond, x0, w = X[b], Y[b], Wt[b]
        if train_mode and np.random.rand() < 0.5:
            cond, x0, w = cond.flip(-1), x0.flip(-1), w.flip(-1)
        t = torch.randint(1, T + 1, (len(b),))
        ab = model.ab[t][:, None, None]
        x_t = ab.sqrt() * x0 + (1 - ab).sqrt() * torch.randn_like(x0)
        return masked_l1(model(cond, x_t, t), x0, w)

    best = np.inf
    for ep in range(epochs):
        model.train()
        t0 = time.time()
        perm = np.random.permutation(tr)
        tl = 0.0
        for i in range(0, len(perm), batch):
            b = perm[i:i + batch]
            loss = step(b, True)
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            tl += loss.item() * len(b)
        model.eval()
        with torch.no_grad():
            vl = float(np.mean([step(va[i:i + 64], False).item() for i in range(0, nv, 64)]))
        print(f"epoch {ep + 1}/{epochs} train {tl / len(tr):.4f} val {vl:.4f} ({time.time() - t0:.0f}s)", file=sys.stderr)
        if vl < best:
            best = vl
            torch.save(model.state_dict(), out)


def load_model(path: Path) -> DDPM:
    m = DDPM()
    m.load_state_dict(torch.load(path, map_location="cpu"))
    return m.eval()


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    tr = sub.add_parser("train")
    tr.add_argument("train64", type=Path)
    tr.add_argument("--out", type=Path, default=Path("out/ddpm.pt"))
    tr.add_argument("--epochs", type=int, default=30)
    tr.add_argument("--holdout", type=int, default=400)
    ev = sub.add_parser("eval")
    ev.add_argument("data", type=Path)
    ev.add_argument("weights", type=Path)
    ev.add_argument("--steps", type=int, default=20)
    ev.add_argument("--n", type=int, default=400)
    ev.add_argument("--starts", type=int, default=8)
    a = ap.parse_args(argv)
    if a.cmd == "train":
        d = np.load(a.train64)
        xs, ys, ws = whole_samples({k: d[k][:-a.holdout] for k in d.files})
        print(f"{len(xs)} samples", file=sys.stderr)
        a.out.parent.mkdir(parents=True, exist_ok=True)
        train(xs, ys, ws, a.epochs, a.out)
    else:
        from .eval import run
        model = load_model(a.weights)
        run(np.load(a.data), lambda x: predict(model, x, a.steps), a.n, a.starts, f"ddpm{a.steps}")


if __name__ == "__main__":
    main()
