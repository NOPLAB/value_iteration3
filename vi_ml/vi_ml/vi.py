"""Run the exact u64 solver (`bench_map` from vi_rs) on a free grid and read back
the θ-min value field in seconds (NaN = unreachable) and the solve wall-clock."""
from __future__ import annotations

import os
import re
import subprocess
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
BENCH_MAP = REPO / "vi_rs" / "target" / "release" / ("bench_map.exe" if os.name == "nt" else "bench_map")
RES_M = 0.1            # cell size; 0.3 m max step = 3 cells
SAFETY_RADIUS_M = 0.2  # Ueda 2023 launch
SAFETY_PENALTY = 30    # seconds per penalised cell (本家 launch; bench_map's 100000 default is a unit typo)
GOAL_RADIUS_M = 0.2


def write_map(free: np.ndarray, d: Path, res: float = RES_M) -> Path:
    """ROS map_server PGM+YAML with origin (0,0); grid row iy = array row (the
    loader flips the image, so we flip on write)."""
    img = np.where(free, 254, 0).astype(np.uint8)[::-1]
    pgm = d / "map.pgm"
    with open(pgm, "wb") as f:
        f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
        f.write(img.tobytes())
    yaml = d / "map.yaml"
    yaml.write_text(f"image: map.pgm\nresolution: {res}\norigin: [0.0, 0.0, 0.0]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return yaml


def read_dump(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    w, h = np.frombuffer(raw[:8], np.int32)
    return np.frombuffer(raw[8:], np.float32).reshape(h, w).copy()


def write_dump(v: np.ndarray, path: Path) -> None:
    """Inverse of `read_dump` (also the `--init-value` input format)."""
    h, w = v.shape
    path.write_bytes(np.array([w, h], np.int32).tobytes() + np.ascontiguousarray(v, np.float32).tobytes())


def solve(free: np.ndarray, goal: tuple[int, int], solver: str = "frontier2d_sparse",
          res: float = RES_M, init: np.ndarray | None = None, stats: dict | None = None,
          max_iters: int | None = None, action_scale: float = 1.0) -> tuple[np.ndarray, float]:
    """(value[H,W] in seconds, solve_ms). `goal` is (iy, ix) in grid cells.
    `init`: warm-start field (seconds, NaN = unknown). `stats`, if given, receives iters/updates."""
    if not BENCH_MAP.exists():
        raise FileNotFoundError(f"{BENCH_MAP}: build with `cargo build --release -p vi_bench --bin bench_map`")
    gy, gx = goal
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        yaml = write_map(free, d, res)
        dump = d / "value.bin"
        cmd = [str(BENCH_MAP), "--map", str(yaml), "--solver", solver,
               "--goal-x", str((gx + 0.5) * res), "--goal-y", str((gy + 0.5) * res),
               "--goal-radius-m", str(max(GOAL_RADIUS_M, 2 * res) if res > RES_M else GOAL_RADIUS_M),
               "--safety-radius-m", str(SAFETY_RADIUS_M), "--safety-penalty", str(SAFETY_PENALTY),
               "--dump-value", str(dump)]
        if init is not None:
            write_dump(init, d / "init.bin")
            cmd += ["--init-value", str(d / "init.bin")]
        if max_iters is not None:
            cmd += ["--max-iters", str(max_iters)]
        if action_scale != 1.0:  # coarse levels: keep "3 cells per step" as the cell grows
            cmd += ["--action-scale", str(action_scale)]
        p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
        if p.returncode != 0:
            raise RuntimeError(p.stderr[-2000:])
        m = re.search(r"iters=(\d+) updates=(\d+) total_ms=([\d.]+) converged=(\w)", p.stderr)
        if stats is not None:
            stats.update(iters=int(m.group(1)), updates=int(m.group(2)), converged=m.group(4) == "Y")
        return read_dump(dump), float(m.group(3))
