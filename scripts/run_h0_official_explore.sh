#!/usr/bin/env bash
# Official RPent exploration on one LIBERO-Safety reference cell.
# Success writes task-specific audit/recipe and merges task-family drafts.
# This is not the read-only evaluation.
set -u
cd /homeL/dong/Projects/dash-vla-libero
unset CUDA_VISIBLE_DEVICES
unset MUJOCO_EGL_DEVICE_ID
unset PI05_DISCRETE_STATE
# fullshot horizon is 10. Execute the whole horizon, not the libero preset's 5.
export PI05_ACTION_CHUNK=10
export H0_CUDA_DEVICE="${H0_CUDA_DEVICE:-0}"
export LIBERO_CONFIG_PATH=/homeL/dong/Projects/dash-vla-libero/runs/g5_libero/libero_config_safety
export LIBERO_SAFETY_ROOT=/homeL/dong/Projects/dash-vla-libero/third_party/LIBERO-Safety
export LIBERO_SAFETY_PYTHON=/homeL/dong/Projects/dash-vla-libero/.venv-libero-safety/bin/python
export PI05_PYTHON=/homeL/dong/Projects/dash-vla-libero/.venv-pi05/bin/python
export SAM3_PYTHON="$PI05_PYTHON"
export PI05_EXTRA_SITE="$(${LIBERO_SAFETY_PYTHON} -c 'import site; print(site.getsitepackages()[0])')"
export PI05_CHECKPOINT_PATH=/homeL/dong/Projects/dash-vla-libero/checkpoints/RLinf-Pi05-LIBERO-130-fullshot-SFT
export PI05_NORM_STATS_PATH=/homeL/dong/Projects/dash-vla-libero/checkpoints/RLinf-Pi05-LIBERO-130-fullshot-SFT/physical-intelligence/libero
export PI05_CUDA_DEVICE="$H0_CUDA_DEVICE"
export SAM3_CUDA_DEVICE="$H0_CUDA_DEVICE"
export SAM3_CHECKPOINT_PATH=/homeL/dong/Projects/dash-vla-libero/checkpoints/sam3/sam3.pt
export OPENPI_CLIENT_SRC=/homeL/dong/Projects/dash-vla-libero/third_party/openpi/packages/openpi-client/src
export OPENPI_DATA_HOME=/homeL/dong/from-gpu6/cache/openpi
export HF_HOME=/homeL/dong/from-gpu6/cache/hf
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
export RPENT_REPO_ROOT=/homeL/dong/Projects/dash-vla-libero/vendor/RPent-harness
export RPENT_OFFICIAL_TOOLSET=1
export CODEX_HOME=/homeL/dong/.codex
export PYTHONPATH="${RPENT_REPO_ROOT}${PYTHONPATH:+:$PYTHONPATH}"

PY="$LIBERO_SAFETY_PYTHON"
ROOT=/homeL/dong/Projects/dash-vla-libero
SUITE=affordance
TASK=0
SEED=0
OUT="$ROOT/runs/g5_libero/h0_official/explore/${SUITE}_t${TASK}_s${SEED}"
mkdir -p "$OUT"
echo "BEGIN explore ${SUITE} ${TASK} ${SEED} $(date -Is)"
"$PY" "$RPENT_REPO_ROOT/rpent/cli/main.py" \
  --robot libero \
  --planner codex \
  --model gpt-6.1-sol \
  --reasoning-effort low \
  --explore \
  --explore-sessions 3 \
  --explore-attempts-per-session 5 \
  --suite "$SUITE" \
  --task "$TASK" \
  --seed "$SEED" \
  --memory-profile local \
  --memory-dir "$ROOT/runs/g5_libero/l52b/memory" \
  --auto-merge-memory \
  --max-turns 100 \
  --max-episode-steps 10000 \
  --planner-timeout-s 7200 \
  --cuda-device "$H0_CUDA_DEVICE" \
  --output-dir "$OUT"
code=$?
echo "END explore ${SUITE} ${TASK} ${SEED} exit=${code} $(date -Is)"
exit "$code"
