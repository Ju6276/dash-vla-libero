#!/usr/bin/env bash
# DASH-VLA first round (report 07): AEGIS baseline, then per scene the full method
# (explore with memory write, eval 1 and 2) and the no-memory ablation (eval 1 and 2).
# Usage limits and the budget stop are handled as in run_p2.sh.
# SL_ROUND_ARM=aegiscbf runs the same two arms with AEGIS's filter (report 08);
# SL_SKIP_AEGIS=1 skips the AEGIS baseline.
set -u
cd /homeL/dong/Projects/dash-vla-libero
SUITES=(${SL_SUITES:-safelibero_spatial_I safelibero_spatial_II safelibero_goal_I safelibero_goal_II
        safelibero_object_I safelibero_object_II safelibero_long_I safelibero_long_II})
WAIT="${SL_QUOTA_WAIT:-1800}"
export SL_CODEX_HOME="${SL_CODEX_HOME:-/homeL/dong/.codex-tams}"
export SL_BUDGET_STOP="${SL_BUDGET_STOP:-80}"
ARM="${SL_ROUND_ARM:-cbf}"
if [ "${SL_SKIP_AEGIS:-0}" != 1 ]; then
  cells=(); for s in "${SUITES[@]}"; do cells+=("$s:0"); done
  scripts/safelibero/run_aegis.sh --output-dir runs/aegis/p2 --cells "${cells[@]}" --episodes 1 2
  echo "AEGIS DONE $(date -Is)"
fi
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
for suite in "${SUITES[@]}"; do
  batch SL_ARM=$ARM SL_CBF_MEMORY=1 SL_EXPLORE=1 SL_SUITES=$suite
  batch SL_ARM=$ARM SL_MEM_TAG=nomem SL_EXPLORE=0 SL_SUITES=$suite
done
echo "CBF ROUND DONE $ARM $(date -Is)"
