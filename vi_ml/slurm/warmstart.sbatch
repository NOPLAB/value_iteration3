#!/bin/bash
# Tsudanuma: can a coarse field warm-start the exact (dense frontier2d_sparse) solve?
#SBATCH --job-name=vi-warmstart
#SBATCH --nodelist=mechanigaharakizashi
#SBATCH --output=/mnt/workspace/nop/value_iteration3/vi_ml/slurm/%x-%j.out
#SBATCH --cpus-per-task=16
#SBATCH --mem=120G
#SBATCH --time=300
set -euo pipefail
cd /mnt/workspace/nop/value_iteration3/vi_ml
export PYTHONUTF8=1 VI_THREADS=${SLURM_CPUS_PER_TASK}
.venv/bin/python -m vi_ml.warmstart real ../assets/map_tsudanuma.yaml out/tsudanuma_s2.bin --scale 2 --starts 20
