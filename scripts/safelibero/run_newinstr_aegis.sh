#!/usr/bin/env bash
# Waits for the repeat round to pause on the quota (or finish), then runs AEGIS-SFT on
# the new-instruction variants: 6 variants x levels I, II x episodes 0-3, 900 steps.
cd /homeL/dong/Projects/dash-vla-libero
while true; do
  last=$(grep -E '^(BEGIN|END|BUDGET PAUSE|REPEAT ROUND DONE)' runs/plan_driver.log | tail -1)
  case "$last" in "BUDGET PAUSE"*|"REPEAT ROUND DONE"*) break;; esac
  sleep 120
done
sleep 60
echo "START $(date -Is) after: $last"
cells=()
for v in spatial_bowl_stove spatial_bowl_cabinet goal_cheese_plate goal_bowl_stove_cheese_plate object_butter_ketchup long_tomato_ketchup; do
  cells+=("$v@I" "$v@II")
done
scripts/safelibero/run_aegis.sh --output-dir runs/newinstr/aegis_sft_900 --variants runs/newinstr/bddl/variants.json \
  --cells "${cells[@]}" --episodes 0 1 2 3 --max-steps 900
echo "NEWINSTR DONE $(date -Is)"
