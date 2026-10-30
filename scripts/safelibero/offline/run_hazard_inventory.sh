#!/usr/bin/env bash
# Offline check of DASH module 1 (scene hazard inventory); no planner, no Codex quota.
# Exports 8 suites x tasks 0-3 x episodes 0-1 and grounds every non-task object.
set -u
cd /homeL/dong/Projects/dash-vla-libero
ROOT=/homeL/dong/Projects/dash-vla-libero
OUT="${1:-$ROOT/runs/offline/hazard_inventory}"
export SAFELIBERO_ROOT=$ROOT/third_party/vlsa-aegis/safelibero
export SAFELIBERO_ROBOSUITE_ROOT=$ROOT/third_party/robosuite-stock/robosuite-1.4.1
export SAFELIBERO_CONFIG_PATH=$ROOT/runs/safelibero/libero_config
export LIBERO_CONFIG_PATH=$SAFELIBERO_CONFIG_PATH
export LIBERO_SAFETY_ROOT=$ROOT/third_party/LIBERO-Safety
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
export SAM3_CHECKPOINT_PATH=$ROOT/checkpoints/sam3/sam3.pt
export HF_HOME=/homeL/dong/from-gpu6/cache/hf
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
RPENT=$ROOT/vendor/RPent-harness
export RPENT_REPO_ROOT=$RPENT
CELLS=()
for suite in spatial_I spatial_II goal_I goal_II object_I object_II long_I long_II; do
  for task in 0 1 2 3; do
    for seed in 0 1; do
      CELLS+=("safelibero_${suite}:${task}:${seed}")
    done
  done
done
mkdir -p "$OUT/scenes"
PYTHONPATH="$RPENT:$SAFELIBERO_ROOT:$SAFELIBERO_ROBOSUITE_ROOT" \
  .venv-libero-safety/bin/python scripts/safelibero/offline/export_scenes.py --out "$OUT/scenes" "${CELLS[@]}" || exit 1
PYTHONPATH="$RPENT" .venv-pi05/bin/python scripts/safelibero/offline/hazard_inventory.py "$OUT/scenes" \
  --out "$OUT/summary.json"
