#!/usr/bin/env bash
# Report 13: the filter with the forearm and wrist links (SL_CBF_LINKS=1) on the dev
# episodes where H-dev or H+C-dev hit the obstacle with link5 / link6, plus two
# collision-free scenes to see whether the extra constraints hold tasks back.
# Same setting as H-dev (runs/p2/aegiscbf_nomem_dev): base memory, no explore.
set -u
cd /homeL/dong/Projects/dash-vla-libero
export SL_CODEX_HOME="${SL_CODEX_HOME:-/homeL/dong/.codex-tams}"
export SL_BUDGET_STOP="${SL_BUDGET_STOP:-99}"
run() { env SL_ARM=aegiscbf SL_CBF_LINKS=1 SL_MEM_TAG=links_dev SL_EXPLORE=0 SL_TASKS=0 SL_SUITES=$1 SL_EVAL_SEEDS="$2" scripts/safelibero/run_batch.sh; }
run safelibero_long_I "0 3 4"
run safelibero_goal_I "4"
run safelibero_goal_II "4"
run safelibero_object_I "0"
run safelibero_spatial_I "0"
echo "LINK CHECK DONE $(date -Is)"
