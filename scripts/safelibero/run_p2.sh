#!/usr/bin/env bash
# Formal round (runs/p2): task 0 of every suite, evaluated on episodes 1 and 2.
# Per suite, in turn: B0 original and B1 prompt explore on episode 0 first;
# B2 safeharness skips exploration and uses the published obstacle-free memory,
# as in the SafeHarness paper. A partial queue still compares all three arms on
# the same scenes. On a Codex usage limit or overload (run_batch.sh exit 75)
# wait and resume; finished runs are skipped. The queue plans with the TAMS
# workspace login and stops for good once its weekly usage reaches
# SL_BUDGET_STOP percent (run_batch.sh exit 76).
set -u
cd /homeL/dong/Projects/dash-vla-libero
SUITES=(${SL_SUITES:-safelibero_spatial_I safelibero_spatial_II safelibero_goal_I safelibero_goal_II
        safelibero_object_I safelibero_object_II safelibero_long_I safelibero_long_II})
WAIT="${SL_QUOTA_WAIT:-1800}"
export SL_CODEX_HOME="${SL_CODEX_HOME:-/homeL/dong/.codex-tams}"
export SL_BUDGET_STOP="${SL_BUDGET_STOP:-80}"
for suite in "${SUITES[@]}"; do
  for arm in original prompt safeharness; do
    explore=1
    [ "$arm" = safeharness ] && explore=0
    while true; do
      SL_ARM=$arm SL_EXPLORE=$explore SL_SUITES=$suite SL_TASKS=0 SL_EVAL_SEEDS="1 2" \
        scripts/safelibero/run_batch.sh
      code=$?
      [ $code -eq 76 ] && { echo "P2 QUEUE BUDGET STOP $(date -Is)"; exit 76; }
      [ $code -ne 75 ] && break
      echo "QUOTA WAIT ${WAIT}s $arm $suite $(date -Is)"
      sleep "$WAIT"
    done
  done
done
missing=$(for arm in original prompt safeharness; do
  for suite in "${SUITES[@]}"; do
    modes="eval"; [ "$arm" != safeharness ] && modes="explore eval"
    for mode in $modes; do
      seeds="1 2"; [ "$mode" = explore ] && seeds=0
      for seed in $seeds; do
        dir="runs/p2/$arm/$mode/${suite/safelibero_/scene_}_t0_s$seed"
        [ -f "$dir/.done" ] || echo "  $arm $mode $suite s$seed"
      done
    done
  done
done)
[ -n "$missing" ] && printf 'MISSING RUNS\n%s\n' "$missing"
echo "P2 QUEUE DONE $(date -Is)"
