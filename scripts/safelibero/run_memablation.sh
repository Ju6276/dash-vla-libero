#!/usr/bin/env bash
# Report 10: is the memory gain of H+M (report 08) the safety-filter record or
# RPent's own explore notes? Same arm and protocol as H+M (explore on episode 0,
# eval 1 and 2), but the filter record is not written (SL_CBF_MEMORY=0).
#   runs/p2/aegiscbf_exploreonly, memory in runs/p2/memory_aegiscbf_exploreonly
set -u
cd /homeL/dong/Projects/dash-vla-libero
SUITES=(safelibero_spatial_I safelibero_spatial_II safelibero_goal_I safelibero_goal_II
        safelibero_object_I safelibero_object_II safelibero_long_I safelibero_long_II)
WAIT="${SL_QUOTA_WAIT:-1800}"
export SL_CODEX_HOME="${SL_CODEX_HOME:-/homeL/dong/.codex-tams}"
export SL_BUDGET_STOP="${SL_BUDGET_STOP:-95}"
for suite in "${SUITES[@]}"; do
  while true; do
    env SL_ARM=aegiscbf SL_MEM_TAG=exploreonly SL_CBF_MEMORY=0 SL_EXPLORE=1 SL_SUITES=$suite \
      SL_TASKS=0 SL_EVAL_SEEDS="1 2" scripts/safelibero/run_batch.sh
    code=$?
    [ $code -eq 76 ] && { echo "ROUND BUDGET STOP $(date -Is)"; exit 76; }
    [ $code -ne 75 ] && break
    echo "QUOTA WAIT ${WAIT}s $suite $(date -Is)"
    sleep "$WAIT"
  done
done
echo "MEMABLATION DONE $(date -Is)"
