#!/bin/bash
# Multiscale training windows, as an array of short shards (each map runs a coarse
# exact VI through bench_map). Short tasks so a requeue costs one shard, not the run.
#SBATCH --job-name=vi-prepare
#SBATCH --nodelist=mechanigaharakizashi
#SBATCH --output=/mnt/workspace/nop/value_iteration3/vi_ml/slurm/%x-%A_%a.out
#SBATCH --array=0-15%4
#SBATCH --cpus-per-task=4
#SBATCH --mem=24G
#SBATCH --time=120
set -euo pipefail
cd /mnt/workspace/nop/value_iteration3/vi_ml
export PYTHONUTF8=1 VI_THREADS=1
.venv/bin/python -m vi_ml.multiscale prepare     "out/ms2_parts/$(printf '%02d' "${SLURM_ARRAY_TASK_ID}").npz"     out/train64.npz out/maps128.npz out/maps256.npz     --workers 4 --shard "${SLURM_ARRAY_TASK_ID}" --shards "${SLURM_ARRAY_TASK_COUNT}"
