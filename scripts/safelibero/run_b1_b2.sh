#!/usr/bin/env bash
# B1 (prompt) and B2 (safeharness) on the B0 pilot episodes: task 0 of every
# suite, explore on episode 0, evaluate on 1 and 2. Per suite B1 runs first,
# then B2, so a partial queue still compares both arms on the same scenes.
# On a Codex usage limit (run_batch.sh exit 75) wait and resume; finished runs
# are skipped.
set -u
cd /homeL/dong/Projects/dash-vla-libero
SUITES=(${SL_SUITES:-safelibero_spatial_I safelibero_spatial_II safelibero_goal_I safelibero_goal_II
        safelibero_object_I safelibero_object_II safelibero_long_I safelibero_long_II})
WAIT="${SL_QUOTA_WAIT:-1800}"
for suite in "${SUITES[@]}"; do
  for arm in prompt safeharness; do
    while true; do
      SL_ARM=$arm SL_SUITES=$suite SL_TASKS=0 SL_EVAL_SEEDS="1 2" scripts/safelibero/run_batch.sh
      [ $? -ne 75 ] && break
      echo "QUOTA WAIT ${WAIT}s $arm $suite $(date -Is)"
      sleep "$WAIT"
    done
  done
done
echo "B1 B2 QUEUE DONE $(date -Is)"
