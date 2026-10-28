#!/usr/bin/env bash
# B0 pilot: original Harness on the 8 task-0 SafeLIBERO scenes.
# Per scene: explore on episode 0, then evaluate on episodes 1 and 2.
# A finished run leaves .done in its output dir and is skipped on rerun.
set -u
cd /homeL/dong/Projects/dash-vla-libero
ARM="${SL_ARM:-original}"
SUITES=(safelibero_spatial_I safelibero_spatial_II safelibero_goal_I safelibero_goal_II
        safelibero_object_I safelibero_object_II safelibero_long_I safelibero_long_II)
run() {
  local mode=$1 suite=$2 seed=$3
  local out="runs/safelibero/harness/${ARM}/${mode}/${suite}_t0_s${seed}"
  [ -f "$out/.done" ] && { echo "SKIP $mode $suite s$seed"; return; }
  SL_ARM=$ARM SL_MODE=$mode SL_SUITE=$suite SL_TASK=0 SL_SEED=$seed \
    scripts/safelibero/run_safelibero_harness.sh && touch "$out/.done"
}
for suite in "${SUITES[@]}"; do
  run explore "$suite" 0
  run eval "$suite" 1
  run eval "$suite" 2
done
echo "ALL DONE $(date -Is)"
