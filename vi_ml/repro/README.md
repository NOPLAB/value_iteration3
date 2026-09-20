# `frontier2d_sparse` mismatch repro

64×64 @ 0.1 m (vi_ml `blobs` map, `out/train64.npz` index 3985). Goal world (4.45, 2.15).
`reference` and `frontier2d` solve all 1776 free cells; `frontier2d_sparse` leaves 215 of them
at `MAX_COST` and is up to 20.186 s low on the cells it does reach.

```sh
for s in reference frontier2d frontier2d_sparse; do
  ../../vi_rs/target/release/bench_map --map map.yaml --solver $s \
    --goal-x 4.45 --goal-y 2.15 --goal-radius-m 0.2 \
    --safety-radius-m 0.2 --safety-penalty 30 --dump-value "$s.bin"
done
python - <<'PY'
import numpy as np
def read(p):
    b = open(p,'rb').read(); w,h = np.frombuffer(b[:8], '<i4')
    return np.frombuffer(b[8:], '<f4').reshape(h, w)
r, f, s = (read(f"{n}.bin") for n in ("reference", "frontier2d", "frontier2d_sparse"))
for n, v in (("frontier2d", f), ("frontier2d_sparse", s)):
    both = np.isfinite(r) & np.isfinite(v)
    print(n, "finite", int(np.isfinite(v).sum()), "of", int(np.isfinite(r).sum()),
          "max|diff| where both finite", float(np.abs(r - v)[both].max()))
PY
```

Warm-starting the sparse solver from the `frontier2d` field (`--init-value`) makes it agree, so the
missing work is propagation, not the update rule. Found while measuring warm start (see ../README.md).
