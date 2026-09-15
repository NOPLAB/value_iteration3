#!/bin/bash
# Tsudanuma only: the headline number, against a VI baseline measured on this machine.
#SBATCH --job-name=vi-tsudanuma
#SBATCH --nodelist=mechanigaharakizashi
#SBATCH --output=/mnt/workspace/nop/value_iteration3/vi_ml/slurm/%x-%j.out
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=16
#SBATCH --mem=96G
#SBATCH --time=240
set -euo pipefail
cd /mnt/workspace/nop/value_iteration3/vi_ml
export PYTHONUTF8=1
VI_MS=$(VI_THREADS=${SLURM_CPUS_PER_TASK} ../vi_rs/target/release/bench_map \
    --map ../assets/map_tsudanuma.yaml --scale 2 --solver frontier2d_sparse_compact \
    --goal-x 152.55 --goal-y 97.55 --goal-theta-deg 0 --goal-radius-m 0.2 \
    --safety-radius-m 0.2 --safety-penalty 30 --unknown obstacle 2>&1 \
    | grep -oP 'total_ms=\K[\d.]+')
echo "VI baseline: ${VI_MS} ms (${SLURM_CPUS_PER_TASK} threads)"
echo "=== coarse exact VI only ==="
.venv/bin/python -m vi_ml.multiscale real ../assets/map_tsudanuma.yaml out/tsudanuma_s2.bin \
    --no-refine --scale 2 --top 768 --starts 20 --vi-ms "${VI_MS}"
echo "=== + learned refinement (GPU) ==="
.venv/bin/python -m vi_ml.multiscale real ../assets/map_tsudanuma.yaml out/tsudanuma_s2.bin out/ms2.pt \
    --scale 2 --top 768 --starts 20 --vi-ms "${VI_MS}"
