#!/bin/bash
# Concatenate the prepare shards into one training file.
#SBATCH --job-name=vi-merge
#SBATCH --nodelist=mechanigaharakizashi
#SBATCH --output=/mnt/workspace/nop/value_iteration3/vi_ml/slurm/%x-%j.out
#SBATCH --cpus-per-task=2
#SBATCH --mem=96G
#SBATCH --time=60
set -euo pipefail
cd /mnt/workspace/nop/value_iteration3/vi_ml
export PYTHONUTF8=1
.venv/bin/python -m vi_ml.multiscale merge out/ms2.npz out/ms2_parts/*.npz
