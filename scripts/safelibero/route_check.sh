#!/usr/bin/env bash
# Offline SafeHarness-style route check on recorded Harness runs.
# Usage: route_check.sh [RUN_ROOT]   (default: the B0 pilot)
set -u
cd /homeL/dong/Projects/dash-vla-libero
ROOT=/homeL/dong/Projects/dash-vla-libero
RUNS="${1:-runs/p2/original}"
AUDIT="${2:-runs/p2_audit/original}"
export SAFELIBERO_ROOT=$ROOT/third_party/vlsa-aegis/safelibero
export SAFELIBERO_ROBOSUITE_ROOT=$ROOT/third_party/robosuite-stock/robosuite-1.4.1
export SAFELIBERO_CONFIG_PATH=$ROOT/runs/safelibero/libero_config
export LIBERO_CONFIG_PATH=$SAFELIBERO_CONFIG_PATH
export LIBERO_SAFETY_ROOT=$ROOT/third_party/LIBERO-Safety
export LIBERO_SAFETY_PYTHON=$ROOT/.venv-libero-safety/bin/python
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
export RPENT_REPO_ROOT=$ROOT/vendor/RPent-harness
export PYTHONPATH="${RPENT_REPO_ROOT}:${SAFELIBERO_ROOT}:${SAFELIBERO_ROBOSUITE_ROOT}"
GEOMETRY=runs/p2/scene_geometry.json
# Every (suite, seed) the runs used.
# Run dirs are named <suite>_t<task>_s<seed>.
CELLS=$(for t in "$AUDIT"/*/*/safety_trace.json; do
  name=$(basename "$(dirname "$t")")
  rest=${name##*_t}
  echo "${name%_t*}:${rest%%_s*}:${rest##*_s}" | sed 's/^scene_/safelibero_/'
done | sort -u)
"$LIBERO_SAFETY_PYTHON" scripts/safelibero/scene_geometry.py --out "$GEOMETRY" $CELLS || exit 1
"$LIBERO_SAFETY_PYTHON" scripts/safelibero/route_check.py "$RUNS" --audit "$AUDIT" --geometry "$GEOMETRY" \
  --out "$RUNS/route_check.json"
