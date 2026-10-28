#!/usr/bin/env bash
# Group A on SafeLIBERO: the Harness's frozen fullshot VLA alone.
set -u
cd /homeL/dong/Projects/dash-vla-libero
unset CUDA_VISIBLE_DEVICES
unset MUJOCO_EGL_DEVICE_ID
unset PI05_DISCRETE_STATE
export PI05_ACTION_CHUNK=10
export H0_CUDA_DEVICE="${H0_CUDA_DEVICE:-0}"
ROOT=/homeL/dong/Projects/dash-vla-libero
export SAFELIBERO_ROOT=$ROOT/third_party/vlsa-aegis/safelibero
export SAFELIBERO_ROBOSUITE_ROOT=$ROOT/third_party/robosuite-stock/robosuite-1.4.1
export SAFELIBERO_CONFIG_PATH=$ROOT/runs/safelibero/libero_config
export LIBERO_SAFETY_ROOT=$ROOT/third_party/LIBERO-Safety
export LIBERO_SAFETY_PYTHON=$ROOT/.venv-libero-safety/bin/python
export PI05_PYTHON=$ROOT/.venv-pi05/bin/python
export PI05_EXTRA_SITE="$(${LIBERO_SAFETY_PYTHON} -c 'import site; print(site.getsitepackages()[0])')"
export PI05_CHECKPOINT_PATH=$ROOT/checkpoints/RLinf-Pi05-LIBERO-130-fullshot-SFT
export PI05_NORM_STATS_PATH=$ROOT/checkpoints/RLinf-Pi05-LIBERO-130-fullshot-SFT/physical-intelligence/libero
export OPENPI_CLIENT_SRC=$ROOT/third_party/openpi/packages/openpi-client/src
export OPENPI_DATA_HOME=/homeL/dong/from-gpu6/cache/openpi
export HF_HOME=/homeL/dong/from-gpu6/cache/hf
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
export RPENT_REPO_ROOT=$ROOT/vendor/RPent-harness
export PYTHONPATH="${RPENT_REPO_ROOT}:${SAFELIBERO_ROOT}:${SAFELIBERO_ROBOSUITE_ROOT}"
exec "$LIBERO_SAFETY_PYTHON" $ROOT/scripts/safelibero/run_vla_only.py "$@"
