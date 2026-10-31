#!/usr/bin/env bash
# Report 09: the VLA-only arms at the harness arms' 900-step cap, on the same 16
# eval episodes (8 scenes x episode 1, 2). Results within the official 300 / 550
# budget are read off the same runs (success step and collision step).
#   A      the SFT VLA alone                      runs/vla_only/p2_900
#   AEGIS  SFT VLA + AEGIS                        runs/aegis/p2_900
#   AEGIS  official pi05_libero + AEGIS (README)  runs/aegis/p2_official_900
set -u
cd /homeL/dong/Projects/dash-vla-libero
SUITES=(safelibero_spatial_I safelibero_spatial_II safelibero_goal_I safelibero_goal_II
        safelibero_object_I safelibero_object_II safelibero_long_I safelibero_long_II)
cells=(); for s in "${SUITES[@]}"; do cells+=("$s:0"); done
scripts/safelibero/run_aegis.sh --output-dir runs/aegis/p2_official_900 --cells "${cells[@]}" --episodes 1 2 --policy official --max-steps 900
echo "AEGIS OFFICIAL DONE $(date -Is)"
scripts/safelibero/run_aegis.sh --output-dir runs/aegis/p2_900 --cells "${cells[@]}" --episodes 1 2 --max-steps 900
echo "AEGIS SFT DONE $(date -Is)"
scripts/safelibero/run_vla_only.sh --output-dir runs/vla_only/p2_900 --cells "${cells[@]}" --episodes 1 2 --max-steps 900
echo "BUDGET ROUND DONE $(date -Is)"
