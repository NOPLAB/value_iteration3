#!/bin/bash
#SBATCH --job-name=vi-solvercheck
#SBATCH --nodelist=mechanigaharakizashi
#SBATCH --output=/mnt/workspace/nop/value_iteration3/vi_ml/slurm/%x-%j.out
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=120
set -euo pipefail
cd /mnt/workspace/nop/value_iteration3/vi_ml
export PYTHONUTF8=1 VI_THREADS=${SLURM_CPUS_PER_TASK}
.venv/bin/python -m vi_ml.solvercheck ../assets/map_tsudanuma.yaml 152.55 97.55 --scale 4
