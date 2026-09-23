#!/bin/bash
#SBATCH --job-name=vi-noexact
#SBATCH --nodelist=mechanigaharakizashi
#SBATCH --output=/mnt/workspace/nop/value_iteration3/vi_ml/slurm/%x-%j.out
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=16
#SBATCH --mem=96G
#SBATCH --time=120
set -euo pipefail
cd /mnt/workspace/nop/value_iteration3/vi_ml
export PYTHONUTF8=1 MPLCONFIGDIR=/tmp/mpl-$USER
.venv/bin/python noexact.py
