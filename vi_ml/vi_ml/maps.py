"""Random occupancy maps for the VI surrogate. Every generator returns a bool
`free` array (H, W), True = free. Structure over randomness: rooms+corridors
(the campus/indoor distribution), mazes (long horizons), blobs (curved walls),
and crops of a real map (the domain itself)."""
from __future__ import annotations

import numpy as np


def rooms(rng: np.random.Generator, n: int = 64, min_room: int = 10, door: tuple[int, int] = (4, 9)) -> np.ndarray:
    """BSP split into rooms separated by 1-cell walls, each wall pierced by one door."""
    free = np.ones((n, n), bool)
    free[0, :] = free[-1, :] = free[:, 0] = free[:, -1] = False

    def split(y0, y1, x0, x1, depth):
        h, w = y1 - y0, x1 - x0
        if depth > 5 or (h < 2 * min_room and w < 2 * min_room) or rng.random() < 0.15 * depth:
            return
        horiz = h > w or (h == w and rng.random() < 0.5)
        if (horiz and h < 2 * min_room) or (not horiz and w < 2 * min_room):
            horiz = not horiz
        d = int(rng.integers(door[0], door[1]))
        if horiz:
            y = int(rng.integers(y0 + min_room, y1 - min_room + 1))
            free[y, x0:x1] = False
            dx = int(rng.integers(x0, max(x0 + 1, x1 - d)))
            free[y, dx:dx + d] = True
            split(y0, y, x0, x1, depth + 1)
            split(y + 1, y1, x0, x1, depth + 1)
        else:
            x = int(rng.integers(x0 + min_room, x1 - min_room + 1))
            free[y0:y1, x] = False
            dy = int(rng.integers(y0, max(y0 + 1, y1 - d)))
            free[dy:dy + d, x] = True
            split(y0, y1, x0, x, depth + 1)
            split(y0, y1, x + 1, x1, depth + 1)

    split(1, n - 1, 1, n - 1, 0)
    for _ in range(int(rng.integers(0, 6))):  # a little furniture
        h, w = (int(v) for v in rng.integers(1, 4, size=2))
        y, x = int(rng.integers(1, n - 1 - h)), int(rng.integers(1, n - 1 - w))
        free[y:y + h, x:x + w] = False
    return free


def maze(rng: np.random.Generator, n: int = 64, cell: int = 7) -> np.ndarray:
    """Recursive-backtracker maze whose passages are `cell` wide (walls 1 wide),
    plus a few knocked-out walls so it has cycles like a real building."""
    step = cell + 1
    m = (n - 1) // step
    visited = np.zeros((m, m), bool)
    free = np.zeros((n, n), bool)

    def carve(cy, cx):
        free[1 + cy * step:1 + cy * step + cell, 1 + cx * step:1 + cx * step + cell] = True

    start = (int(rng.integers(m)), int(rng.integers(m)))
    stack = [start]
    visited[start] = True
    carve(*start)
    while stack:
        cy, cx = stack[-1]
        nbrs = [(cy + dy, cx + dx) for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= cy + dy < m and 0 <= cx + dx < m and not visited[cy + dy, cx + dx]]
        if not nbrs:
            stack.pop()
            continue
        ny, nx = nbrs[int(rng.integers(len(nbrs)))]
        visited[ny, nx] = True
        carve(ny, nx)
        y0, y1 = sorted((1 + cy * step, 1 + ny * step))
        x0, x1 = sorted((1 + cx * step, 1 + nx * step))
        free[y0:y1 + cell, x0:x1 + cell] = True
        stack.append((ny, nx))
    walls = np.argwhere(~free[1:-1, 1:-1]) + 1
    for y, x in walls[rng.permutation(len(walls))[:m]]:
        free[y, x] = True
    return free


def blobs(rng: np.random.Generator, n: int = 64, k: int = 6, thresh: float = 0.55) -> np.ndarray:
    """Threshold of low-frequency noise (bilinear-upsampled k×k grid) → curved obstacles."""
    c = rng.random((k + 1, k + 1))
    t = np.linspace(0, k, n)
    i0 = np.minimum(t.astype(int), k - 1)
    f = t - i0
    fy, fx = f[:, None], f[None, :]
    y0, x0 = i0[:, None], i0[None, :]
    v = (c[y0, x0] * (1 - fy) * (1 - fx) + c[y0 + 1, x0] * fy * (1 - fx)
         + c[y0, x0 + 1] * (1 - fy) * fx + c[y0 + 1, x0 + 1] * fy * fx)
    free = v < thresh
    free[0, :] = free[-1, :] = free[:, 0] = free[:, -1] = False
    return free


def crop(rng: np.random.Generator, real_free: np.ndarray, n: int = 64, min_free: float = 0.3) -> np.ndarray:
    """Random n×n crop (+ random rot90/flip) of a real map with enough free space."""
    H, W = real_free.shape
    for _ in range(200):
        y, x = int(rng.integers(0, H - n + 1)), int(rng.integers(0, W - n + 1))
        f = real_free[y:y + n, x:x + n].copy()
        f = np.rot90(f, int(rng.integers(4)))
        if rng.random() < 0.5:
            f = f[:, ::-1]
        f = np.ascontiguousarray(f)
        f[0, :] = f[-1, :] = f[:, 0] = f[:, -1] = False
        if f.mean() >= min_free:
            return f
    raise RuntimeError("no crop with enough free space")


def thicken(free: np.ndarray) -> np.ndarray:
    """Grow every obstacle by one cell (8-neighbourhood). The VI checks only the
    *target* cell of a 3-cell forward step, so 1-cell walls are permeable; every
    generator therefore builds thin and thickens here."""
    obs = ~free
    out = obs.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            out |= np.roll(np.roll(obs, dy, 0), dx, 1)
    return ~out


def largest_component(free: np.ndarray, seed: tuple[int, int]) -> np.ndarray:
    """4-connected flood fill from `seed`; everything else becomes an obstacle."""
    H, W = free.shape
    out = np.zeros_like(free)
    if not free[seed]:
        return out
    out[seed] = True
    stack = [seed]
    while stack:
        y, x = stack.pop()
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if 0 <= ny < H and 0 <= nx < W and free[ny, nx] and not out[ny, nx]:
                out[ny, nx] = True
                stack.append((ny, nx))
    return out


def pick_free(rng: np.random.Generator, free: np.ndarray) -> tuple[int, int]:
    ys, xs = np.nonzero(free)
    i = int(rng.integers(len(ys)))
    return int(ys[i]), int(xs[i])


KINDS = {"rooms": rooms, "maze": maze, "blobs": blobs}
