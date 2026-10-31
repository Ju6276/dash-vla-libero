#!/usr/bin/env bash
# Memory control experiment (report 05): B0 original Harness, eval only, on the
# four scenes where p2 B0 evals collided, episodes 1-5. Each (scene, condition)
# has its own memory copy built by build_memctl.py from the final p2 B0 memory:
#   own   p2 B0 memory as is (the scene's own, collided explore)
#   none  same memory without anything the scene's explore merged
#   b1    scene part replaced by the B1 explore's notes (spatial_I, goal_II only;
#         B1 explored those scenes without collision, its notes name the obstacle)
# Tags are neutral (k01..k10) because the planner sees its memory and output
# paths. Runs interleave conditions within each episode. Usage limits and the
# budget stop are handled as in run_p2.sh.
set -u
cd /homeL/dong/Projects/dash-vla-libero
PY=.venv-libero-safety/bin/python
SRC=runs/p2/memory_original
B1=runs/p2/memory_prompt
WAIT="${SL_QUOTA_WAIT:-1800}"
export SL_CODEX_HOME="${SL_CODEX_HOME:-/homeL/dong/.codex-tams}"
export SL_BUDGET_STOP="${SL_BUDGET_STOP:-80}"
# tag scene condition
CELLS=(
  "k01 spatial_I own"  "k02 spatial_I none"  "k03 spatial_I b1"
  "k04 spatial_II own" "k05 spatial_II none"
  "k06 goal_I own"     "k07 goal_I none"
  "k08 goal_II own"    "k09 goal_II none"    "k10 goal_II b1"
)
for cell in "${CELLS[@]}"; do
  read -r tag scene cond <<< "$cell"
  mem="runs/p2/memory_original_$tag"
  [ -d "$mem" ] && continue
  case $cond in
    own)  $PY scripts/safelibero/build_memctl.py "$SRC" "$mem" ;;
    none) $PY scripts/safelibero/build_memctl.py "$SRC" "$mem" --scene "$scene" ;;
    b1)   $PY scripts/safelibero/build_memctl.py "$SRC" "$mem" --scene "$scene" --swap-from "$B1" ;;
  esac || { echo "BUILD FAILED $cell"; exit 1; }
  echo "BUILT $cell"
done
for seed in ${SL_SEEDS:-1 2 3 4 5}; do
  for cell in "${CELLS[@]}"; do
    read -r tag scene cond <<< "$cell"
    while true; do
      SL_ARM=original SL_MEM_TAG=$tag SL_EXPLORE=0 SL_SUITES="safelibero_$scene" SL_TASKS=0 \
        SL_EVAL_SEEDS=$seed scripts/safelibero/run_batch.sh
      code=$?
      [ $code -eq 76 ] && { echo "MEMCTL BUDGET STOP $(date -Is)"; exit 76; }
      [ $code -ne 75 ] && break
      echo "QUOTA WAIT ${WAIT}s $cell s$seed $(date -Is)"
      sleep "$WAIT"
    done
  done
done
echo "MEMCTL DONE $(date -Is)"
