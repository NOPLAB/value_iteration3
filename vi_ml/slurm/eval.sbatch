#!/bin/bash
# Held-out 256² maps, then Tsudanuma against a VI baseline measured on this machine.
#SBATCH --job-name=vi-eval
#SBATCH --nodelist=mechanigaharakizashi
#SBATCH --output=/mnt/workspace/nop/value_iteration3/vi_ml/slurm/%x-%j.out
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=16
#SBATCH --mem=96G
#SBATCH --time=480
set -euo pipefail
cd /mnt/workspace/nop/value_iteration3/vi_ml
export PYTHONUTF8=1

echo "=== VI baseline on Tsudanuma (scale 2, this machine, ${SLURM_CPUS_PER_TASK} threads) ==="
VI_THREADS=${SLURM_CPUS_PER_TASK} ../vi_rs/target/release/bench_map \
    --map ../assets/map_tsudanuma.yaml --scale 2 --solver frontier2d_sparse_compact \
    --goal-x 152.55 --goal-y 97.55 --goal-theta-deg 0 --goal-radius-m 0.2 \
    --safety-radius-m 0.2 --safety-penalty 30 --unknown obstacle \
    --dump-value out/tsudanuma_s2.bin 2>&1 | grep -E 'grid|iters=|dumped'
VI_MS=$(VI_THREADS=${SLURM_CPUS_PER_TASK} ../vi_rs/target/release/bench_map \
    --map ../assets/map_tsudanuma.yaml --scale 2 --solver frontier2d_sparse_compact \
    --goal-x 152.55 --goal-y 97.55 --goal-theta-deg 0 --goal-radius-m 0.2 \
    --safety-radius-m 0.2 --safety-penalty 30 --unknown obstacle 2>&1 \
    | grep -oP 'total_ms=\K[\d.]+')
echo "VI baseline: ${VI_MS} ms"

echo "=== 256² held-out: coarse exact VI only ==="
.venv/bin/python -m vi_ml.multiscale eval out/maps256.npz --no-refine --n 40 --starts 8 --top 128

echo "=== 256² held-out: + learned refinement ==="
.venv/bin/python -m vi_ml.multiscale eval out/maps256.npz out/ms2.pt --n 40 --starts 8 --top 128

echo "=== Tsudanuma: coarse exact VI only ==="
.venv/bin/python -m vi_ml.multiscale real ../assets/map_tsudanuma.yaml out/tsudanuma_s2.bin \
    --no-refine --scale 2 --top 768 --starts 20 --vi-ms "${VI_MS}"

echo "=== Tsudanuma: + learned refinement ==="
.venv/bin/python -m vi_ml.multiscale real ../assets/map_tsudanuma.yaml out/tsudanuma_s2.bin out/ms2.pt \
    --scale 2 --top 768 --starts 20 --vi-ms "${VI_MS}"
