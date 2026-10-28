#!/usr/bin/env bash
# L5.3c then L5.3d. Does not write into runs/g5_libero/l52b.
set -u
cd /homeL/dong/Projects/dash-vla-libero
unset CUDA_VISIBLE_DEVICES
unset MUJOCO_EGL_DEVICE_ID
export LIBERO_CONFIG_PATH=/homeL/dong/Projects/dash-vla-libero/runs/g5_libero/libero_config_safety
export LIBERO_SAFETY_ROOT=/homeL/dong/Projects/dash-vla-libero/third_party/LIBERO-Safety
export LIBERO_SAFETY_PYTHON=/homeL/dong/Projects/dash-vla-libero/.venv-libero-safety/bin/python
export PI05_PYTHON=/homeL/dong/Projects/dash-vla-libero/.venv-pi05/bin/python
export SAM3_PYTHON="$PI05_PYTHON"
export PI05_EXTRA_SITE="$(${LIBERO_SAFETY_PYTHON} -c 'import site; print(site.getsitepackages()[0])')"
export PI05_CHECKPOINT_PATH=/homeL/dong/Projects/dash-vla-libero/checkpoints/pi05_libero_safety_pytorch
export PI05_NORM_STATS_PATH=/homeL/dong/Projects/dash-vla-libero/checkpoints/pi05_libero_safety_pytorch/assets/lerobot
export PI05_ACTION_CHUNK=10
export PI05_DISCRETE_STATE=0
export PI05_CUDA_DEVICE=1
export SAM3_CUDA_DEVICE=0
export SAM3_CHECKPOINT_PATH=/homeL/dong/Projects/dash-vla-libero/checkpoints/sam3/sam3.pt
export OPENPI_CLIENT_SRC=/homeL/dong/Projects/dash-vla-libero/third_party/openpi/packages/openpi-client/src
export OPENPI_DATA_HOME=/homeL/dong/from-gpu6/cache/openpi
export HF_HOME=/homeL/dong/from-gpu6/cache/hf
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
export RPENT_REPO_ROOT=/homeL/dong/Projects/dash-vla-libero/vendor/RPent-harness
export CODEX_HOME=/homeL/dong/.codex
export L52B_MEMORY_DIR=/homeL/dong/Projects/dash-vla-libero/runs/g5_libero/l52b/memory
export L52B_MAX_TURNS=20
export L52B_MAX_EPISODE_STEPS=600
export L52B_PLANNER_TIMEOUT_S=1500
export L52B_MODEL=gpt-6.1-sol
export L52B_EFFORT=low
export PYTHONPATH="${RPENT_REPO_ROOT}${PYTHONPATH:+:$PYTHONPATH}"

PY="$LIBERO_SAFETY_PYTHON"
echo "PI05_EXTRA_SITE=$PI05_EXTRA_SITE"
"$PY" - <<'PY'
import os
os.environ.setdefault("RPENT_REPO_ROOT", "/homeL/dong/Projects/dash-vla-libero/vendor/RPent-harness")
from robots.libero.tools import LiberoPrimitives
from rpent.tools import iter_tools
names = sorted(item.name for item in iter_tools(LiberoPrimitives))
print("tools", names)
assert "full_task_vla" in names
assert "pi0_pick" in names
assert "pi0_doubled" in names
assert "move_to" in names
PY

run_one() {
  local label="$1" suite="$2" task="$3" seed="$4"
  echo "BEGIN ${label} ${suite} ${task} ${seed} $(date -Is)"
  "$PY" scripts/run_l53_planner.py "$suite" "$task" "$seed" "$label"
  local code=$?
  echo "END ${label} ${suite} ${task} ${seed} exit=${code} $(date -Is)"
  return 0
}

# L5.3c: same six cells, no safety supervisor, unified tool set.
run_one unified obstacle_avoidance 0 0
run_one unified obstacle_avoidance 0 1
run_one unified obstacle_avoidance 0 2
run_one unified human_safety 6 0
run_one unified human_safety 6 1
run_one unified human_safety 6 2
# L5.3d: one bounded reverse check. P1 full-sentence VLA already failed at 300 and 600.
run_one reverse_p1 obstacle_avoidance_human 9 0
echo "L53_BATCH_DONE $(date -Is)"
