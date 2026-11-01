#!/usr/bin/env bash
# Repeats of AEGIS-SFT, H and H+M on the same 16 eval episodes (8 scenes x
# episode 1, 2), to measure run-to-run spread before the method changes.
# Same protocol as reports 08 and 09; each repeat has its own output tree and
# memory copy, and the harness arms now log the filter per step (cbf_steps in
# safety_trace.json).
#   AEGIS-SFT  runs/aegis/p2_900_<rep>
#   H          runs/p2/aegiscbf_nomem_<rep>   (base memory, no explore)
#   H+M        runs/p2/aegiscbf_<rep>         (explore episode 0, filter record)
#   SL_REPEATS="r2 r3"
set -u
cd /homeL/dong/Projects/dash-vla-libero
SUITES=(safelibero_spatial_I safelibero_spatial_II safelibero_goal_I safelibero_goal_II
        safelibero_object_I safelibero_object_II safelibero_long_I safelibero_long_II)
REPEATS=(${SL_REPEATS:-r2 r3})
WAIT="${SL_QUOTA_WAIT:-1800}"
export SL_CODEX_HOME="${SL_CODEX_HOME:-/homeL/dong/.codex-tams}"
export SL_BUDGET_STOP="${SL_BUDGET_STOP:-95}"
batch() {  # env assignments for run_batch.sh, retried on usage limits
  while true; do
    env "$@" SL_TASKS=0 SL_EVAL_SEEDS="1 2" scripts/safelibero/run_batch.sh
    code=$?
    [ $code -eq 76 ] && { echo "ROUND BUDGET STOP $(date -Is)"; exit 76; }
    [ $code -ne 75 ] && return
    echo "QUOTA WAIT ${WAIT}s $* $(date -Is)"
    sleep "$WAIT"
  done
}
cells=(); for s in "${SUITES[@]}"; do cells+=("$s:0"); done
for rep in "${REPEATS[@]}"; do
  scripts/safelibero/run_aegis.sh --output-dir runs/aegis/p2_900_$rep --cells "${cells[@]}" --episodes 1 2 --max-steps 900
  echo "AEGIS SFT DONE $rep $(date -Is)"
  for suite in "${SUITES[@]}"; do
    batch SL_ARM=aegiscbf SL_MEM_TAG=$rep SL_CBF_MEMORY=1 SL_EXPLORE=1 SL_SUITES=$suite
    batch SL_ARM=aegiscbf SL_MEM_TAG=nomem_$rep SL_EXPLORE=0 SL_SUITES=$suite
  done
  echo "REPEAT DONE $rep $(date -Is)"
done
echo "REPEAT ROUND DONE $(date -Is)"
