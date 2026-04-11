#!/usr/bin/env python3
"""Generate a tiny synthetic PGM + YAML for HW smoke testing."""
import os
import sys

W, H = 40, 40
resolution = 0.05

pixels = bytearray(255 for _ in range(W * H))
# Vertical wall near the middle with a gap
for y in range(H):
    if 15 <= y < 25:
        continue
    pixels[y * W + 20] = 0

out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
os.makedirs(out_dir, exist_ok=True)

with open(os.path.join(out_dir, "smoke.pgm"), "wb") as f:
    f.write(f"P5\n{W} {H}\n255\n".encode())
    f.write(bytes(pixels))

with open(os.path.join(out_dir, "smoke.yaml"), "w") as f:
    f.write(
        f"image: smoke.pgm\n"
        f"resolution: {resolution}\n"
        f"origin: [0.0, 0.0, 0.0]\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
        f"negate: 0\n"
    )

print(f"Wrote {out_dir}/smoke.{{pgm,yaml}}")
