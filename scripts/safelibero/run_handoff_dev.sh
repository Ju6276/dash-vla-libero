#!/usr/bin/env bash
# Report 12, development runs of the hand-over check and the relative safety
# memory. Development episodes only: 0, 3 and 4 of the eight scenes' task 0
# (episodes 1 and 2 are the ones reports 07-11 looked at; the final evaluation
# uses episodes and tasks not seen in development).
#   H-dev        runs/p2/aegiscbf_nomem_dev        AEGIS filter + feedback, eval 0 3 4
#   H+C-dev      runs/p2/aegiscbf_handoff_dev      + hand-over check, eval 0 3 4
#   H+C+M2-dev   runs/p2/aegiscbf_full_dev         + relative memory: explore 0, eval 3 4
set -u
cd /homeL/dong/Projects/dash-vla-libero
SUITES=(safelibero_long_I safelibero_long_II safelibero_object_I safelibero_goal_II
        safelibero_spatial_I safelibero_spatial_II safelibero_goal_I safelibero_object_II)
WAIT="${SL_QUOTA_WAIT:-1800}"
export SL_CODEX_HOME="${SL_CODEX_HOME:-/homeL/dong/.codex-tams}"
export SL_BUDGET_STOP="${SL_BUDGET_STOP:-99}"
batch() {  # env assignments for run_batch.sh, retried on usage limits
  while true; do
    env "$@" SL_TASKS=0 scripts/safelibero/run_batch.sh
    code=$?
    if [ $code -eq 76 ]; then
      # SL_BUDGET_RESUME=1: wait for the quota to come back (a reset) instead of stopping
      [ "${SL_BUDGET_RESUME:-0}" = 1 ] || { echo "ROUND BUDGET STOP $(date -Is)"; exit 76; }
      echo "BUDGET PAUSE ${WAIT}s $* $(date -Is)"
      sleep "$WAIT"
      continue
    fi
    [ $code -ne 75 ] && return
    echo "QUOTA WAIT ${WAIT}s $* $(date -Is)"
    sleep "$WAIT"
  done
}
for suite in "${SUITES[@]}"; do
  batch SL_ARM=aegiscbf SL_MEM_TAG=nomem_dev SL_EXPLORE=0 SL_EVAL_SEEDS="0 3 4" SL_SUITES=$suite
  batch SL_ARM=aegiscbf SL_HANDOFF=1 SL_MEM_TAG=handoff_dev SL_EXPLORE=0 SL_EVAL_SEEDS="0 3 4" SL_SUITES=$suite
  batch SL_ARM=aegiscbf SL_HANDOFF=1 SL_CBF_MEMORY=2 SL_MEM_TAG=full_dev SL_EXPLORE=1 SL_EVAL_SEEDS="3 4" SL_SUITES=$suite
  echo "SUITE DONE $suite $(date -Is)"
done
echo "HANDOFF DEV DONE $(date -Is)"
