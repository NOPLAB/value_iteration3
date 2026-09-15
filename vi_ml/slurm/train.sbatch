#!/bin/bash
# Train the shared value+policy refiner on the GPU.
#SBATCH --job-name=vi-train
#SBATCH --nodelist=mechanigaharakizashi
#SBATCH --output=/mnt/workspace/nop/value_iteration3/vi_ml/slurm/%x-%j.out
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=96G
#SBATCH --time=240
set -euo pipefail
cd /mnt/workspace/nop/value_iteration3/vi_ml
export PYTHONUTF8=1
.venv/bin/python -m vi_ml.multiscale train out/ms2.npz \
    --out out/ms2.pt --epochs "${EPOCHS:-40}"
