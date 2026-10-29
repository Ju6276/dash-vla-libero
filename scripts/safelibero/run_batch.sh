#!/usr/bin/env bash
# Batch of SafeLIBERO Harness runs: per (task, suite), explore on episode
# SL_EXPLORE_SEED, then evaluate on SL_EVAL_SEEDS.
#   SL_ARM=original|prompt|safeharness   (B0 / B1 / B2)
#   SL_SUITES="safelibero_spatial_I ..."   default: all eight
#   SL_TASKS="0 1 2 3"                     default: 0 1 2 3
#   SL_EVAL_SEEDS="1 2 3 4 5"              default: 1 2
#   SL_EXPLORE_SEED=0
#   SL_MEM_TAG=x               separate memory copy and output tree (memory experiment)
# A finished run leaves .done and is skipped on rerun. On a Codex usage limit the
# partial run moves to runs/_invalid_quota/ and the batch stops with exit 75;
# rerun the same command once the quota resets.
set -u
cd /homeL/dong/Projects/dash-vla-libero
ARM="${SL_ARM:-original}"
TAG="${SL_MEM_TAG:-}"
SUITES=(${SL_SUITES:-safelibero_spatial_I safelibero_spatial_II safelibero_goal_I safelibero_goal_II
        safelibero_object_I safelibero_object_II safelibero_long_I safelibero_long_II})
TASKS=(${SL_TASKS:-0 1 2 3})
EVAL_SEEDS=(${SL_EVAL_SEEDS:-1 2})
EXPLORE_SEED="${SL_EXPLORE_SEED:-0}"
run() {
  local mode=$1 suite=$2 task=$3 seed=$4
  local out="runs/safelibero/harness/${ARM}${TAG:+_$TAG}/${mode}/${suite}_t${task}_s${seed}"
  [ -f "$out/.done" ] && { echo "SKIP $mode $suite t$task s$seed"; return; }
  local log
  log=$(mktemp)
  SL_ARM=$ARM SL_MEM_TAG=$TAG SL_MODE=$mode SL_SUITE=$suite SL_TASK=$task SL_SEED=$seed \
    scripts/safelibero/run_safelibero_harness.sh 2>&1 | tee "$log"
  local code=${PIPESTATUS[0]}
  if grep -q usageLimitExceeded "$log"; then
    mkdir -p runs/_invalid_quota
    mv "$out" "runs/_invalid_quota/$(basename "$out")_$(date +%s)"
    rm -f "$log"
    echo "QUOTA STOP $(date -Is)"
    exit 75
  fi
  rm -f "$log"
  [ "$code" -eq 0 ] && touch "$out/.done"
}
for task in "${TASKS[@]}"; do
  for suite in "${SUITES[@]}"; do
    run explore "$suite" "$task" "$EXPLORE_SEED"
    for seed in "${EVAL_SEEDS[@]}"; do
      run eval "$suite" "$task" "$seed"
    done
  done
done
echo "ALL DONE $(date -Is)"
