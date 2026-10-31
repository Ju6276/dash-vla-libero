#!/usr/bin/env bash
# AEGIS baseline (run_aegis.py) in its own environment (.venv-aegis), with the
# same VLA checkpoint and server as the other arms.
set -u
cd /homeL/dong/Projects/dash-vla-libero
unset CUDA_VISIBLE_DEVICES
unset MUJOCO_EGL_DEVICE_ID
unset PI05_DISCRETE_STATE
unset PI05_ACTION_CHUNK
export H0_CUDA_DEVICE="${H0_CUDA_DEVICE:-0}"
ROOT=/homeL/dong/Projects/dash-vla-libero
export SAFELIBERO_ROOT=$ROOT/third_party/vlsa-aegis/safelibero
export SAFELIBERO_ROBOSUITE_ROOT=$ROOT/third_party/robosuite-stock/robosuite-1.4.1
export SAFELIBERO_CONFIG_PATH=$ROOT/runs/safelibero/libero_config
export LIBERO_CONFIG_PATH=$SAFELIBERO_CONFIG_PATH
export PI05_PYTHON=$ROOT/.venv-pi05/bin/python
export PI05_EXTRA_SITE="$($ROOT/.venv-libero-safety/bin/python -c 'import site; print(site.getsitepackages()[0])')"
export PI05_CHECKPOINT_PATH=$ROOT/checkpoints/RLinf-Pi05-LIBERO-130-fullshot-SFT
export PI05_NORM_STATS_PATH=$ROOT/checkpoints/RLinf-Pi05-LIBERO-130-fullshot-SFT/physical-intelligence/libero
export OPENPI_CLIENT_SRC=$ROOT/third_party/openpi/packages/openpi-client/src
export OPENPI_DATA_HOME=/homeL/dong/from-gpu6/cache/openpi
# GroundingDINO's text encoder (bert-base-uncased) is fetched once into its own cache.
export HF_HOME=$ROOT/checkpoints/hf_aegis
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
# The benchmark's init-state files predate torch's weights_only default (as in scene_env.py).
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
export AEGIS_CODEX_HOME="${SL_CODEX_HOME:-/homeL/dong/.codex-tams}"
export PYTHONPATH="$ROOT/vendor/RPent-harness:$SAFELIBERO_ROOT:$SAFELIBERO_ROBOSUITE_ROOT:$OPENPI_CLIENT_SRC"
exec $ROOT/.venv-aegis/bin/python $ROOT/scripts/safelibero/run_aegis.py "$@"
