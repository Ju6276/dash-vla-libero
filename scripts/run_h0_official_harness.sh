#!/usr/bin/env bash
# Official RPent evaluation entry on LIBERO-Safety.
# Uses rpent/cli/main.py, prompts/evaluate.py, and the paper checkpoint.
# full_task_vla is hidden. The safety-adapted prompts are not used.
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
OUT_ROOT="$ROOT/runs/g5_libero/h0_official"
mkdir -p "$OUT_ROOT"

"$PY" - <<'PY'
import os
os.environ.setdefault("RPENT_REPO_ROOT", "/homeL/dong/Projects/dash-vla-libero/vendor/RPent-harness")
os.environ["RPENT_OFFICIAL_TOOLSET"] = "1"
import robots.libero.tools as libero_tools
from rpent.tools import iter_tools
names = sorted(item.name for item in iter_tools(libero_tools.LiberoPrimitives))
assert "full_task_vla" in names
hidden = "full_task_vla" if os.environ.get("RPENT_OFFICIAL_TOOLSET") == "1" else None
visible = [name for name in names if name != hidden]
assert "full_task_vla" not in visible
assert "pi0_pick" in visible and "pi0_doubled" in visible and "segment" in visible
print("official_primitives", " ".join(visible))
PY

run_one() {
  local suite="$1" task="$2" seed="$3"
  local out="$OUT_ROOT/${H0_OUT_SUBDIR:-retry}/${suite}_t${task}_s${seed}"
  mkdir -p "$out"
  echo "BEGIN official ${suite} ${task} ${seed} $(date -Is)"
  "$PY" "$RPENT_REPO_ROOT/rpent/cli/main.py" \
    --robot libero \
    --planner codex \
    --model gpt-6.1-sol \
    --reasoning-effort low \
    --suite "$suite" \
    --task "$task" \
    --seed "$seed" \
    --memory-profile local \
    --memory-dir "$ROOT/runs/g5_libero/l52b/memory" \
    --max-turns 100 \
    --max-episode-steps 10000 \
    --planner-timeout-s 7200 \
    --cuda-device "$H0_CUDA_DEVICE" \
    --output-dir "$out"
  local code=$?
  echo "END official ${suite} ${task} ${seed} exit=${code} $(date -Is)"
  return 0
}

# Suites without init files cannot start. Stove L0/L1 seeds 0-4, banana, moka,
# and sake seed 0 stay untouched.
if [ -n "${H0_CELLS:-}" ]; then
  for cell in $H0_CELLS; do
    IFS=: read -r suite task seed <<<"$cell"
    run_one "$suite" "$task" "$seed"
  done
else
  run_one affordance 0 0
  run_one human_safety 0 0
  run_one obstacle_avoidance_human 0 0
fi
echo "H0_OFFICIAL_BATCH_DONE $(date -Is)"
