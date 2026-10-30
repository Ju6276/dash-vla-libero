#!/usr/bin/env bash
# Batch of SafeLIBERO Harness runs: per (task, suite), explore on episode
# SL_EXPLORE_SEED, then evaluate on SL_EVAL_SEEDS. Layout as in
# run_safelibero_harness.sh (runs/p2, audit in runs/p2_audit).
#   SL_ARM=original|prompt|safeharness   (B0 / B1 / B2)
#   SL_SUITES="safelibero_spatial_I ..."   default: all eight
#   SL_TASKS="0 1 2 3"                     default: 0 1 2 3
#   SL_EVAL_SEEDS="1 2 3 4 5"              default: 1 2
#   SL_EXPLORE_SEED=0
#   SL_MEM_TAG=x               separate memory copy and output tree (memory experiment)
#   SL_EXPLORE=0               evaluate on the published memory without exploring
# A finished run leaves .done and is skipped on rerun. On a Codex usage limit or
# server overload the partial run moves to runs/_invalid_quota/ and the batch
# stops with exit 75; rerun the same command later. Evaluation is skipped while
# the explore run is unfinished, since its memory was never merged.
set -u
cd /homeL/dong/Projects/dash-vla-libero
ARM="${SL_ARM:-original}"
TAG="${SL_MEM_TAG:-}"
SUITES=(${SL_SUITES:-safelibero_spatial_I safelibero_spatial_II safelibero_goal_I safelibero_goal_II
        safelibero_object_I safelibero_object_II safelibero_long_I safelibero_long_II})
TASKS=(${SL_TASKS:-0 1 2 3})
EVAL_SEEDS=(${SL_EVAL_SEEDS:-1 2})
EXPLORE_SEED="${SL_EXPLORE_SEED:-0}"
EXPLORE="${SL_EXPLORE:-1}"
cell_dir() {  # root mode suite task seed
  echo "$1/${ARM}${TAG:+_$TAG}/$2/${3/safelibero_/scene_}_t$4_s$5"
}
run() {
  local mode=$1 suite=$2 task=$3 seed=$4
  local out audit
  out=$(cell_dir runs/p2 "$mode" "$suite" "$task" "$seed")
  audit=$(cell_dir runs/p2_audit "$mode" "$suite" "$task" "$seed")
  [ -f "$out/.done" ] && { echo "SKIP $mode $suite t$task s$seed"; return; }
  local log
  log=$(mktemp)
  SL_ARM=$ARM SL_MEM_TAG=$TAG SL_MODE=$mode SL_SUITE=$suite SL_TASK=$task SL_SEED=$seed \
    scripts/safelibero/run_safelibero_harness.sh 2>&1 | tee "$log"
  local code=${PIPESTATUS[0]}
  if grep -qE 'usageLimitExceeded|serverOverloaded' "$log"; then
    mkdir -p runs/_invalid_quota
    local stamp
    stamp=$(date +%s)
    mv "$out" "runs/_invalid_quota/$(basename "$out")_$stamp"
    # The attempt log appends; a rerun must not inherit the aborted attempts.
    [ -d "$audit" ] && mv "$audit" "runs/_invalid_quota/$(basename "$out")_${stamp}_audit"
    rm -f "$log"
    echo "QUOTA STOP $(date -Is)"
    exit 75
  fi
  rm -f "$log"
  [ "$code" -eq 0 ] && touch "$out/.done"
}
for task in "${TASKS[@]}"; do
  for suite in "${SUITES[@]}"; do
    if [ "$EXPLORE" = 1 ]; then
      run explore "$suite" "$task" "$EXPLORE_SEED"
      if [ ! -f "$(cell_dir runs/p2 explore "$suite" "$task" "$EXPLORE_SEED")/.done" ]; then
        echo "SKIP EVAL $suite t$task: explore unfinished"
        continue
      fi
    fi
    for seed in "${EVAL_SEEDS[@]}"; do
      run eval "$suite" "$task" "$seed"
    done
  done
done
echo "ALL DONE $(date -Is)"
