#!/bin/bash
# Render the comparison figures (needs the GPU for the refiner pass).
#SBATCH --job-name=vi-figs
#SBATCH --nodelist=mechanigaharakizashi
#SBATCH --output=/mnt/workspace/nop/value_iteration3/vi_ml/slurm/%x-%j.out
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=16
#SBATCH --mem=96G
#SBATCH --time=180
set -euo pipefail
cd /mnt/workspace/nop/value_iteration3/vi_ml
export PYTHONUTF8=1 MPLCONFIGDIR=/tmp/mpl-$USER PATH="$HOME/.local/bin:$PATH"
# the venv's interpreter is this node's, so install from here rather than the login node
.venv/bin/python -c 'import matplotlib' 2>/dev/null || VIRTUAL_ENV=$PWD/.venv uv pip install matplotlib
.venv/bin/python -m vi_ml.figures out/figs --scale 2 --top 768
ls -la out/figs
