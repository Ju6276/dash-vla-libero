#!/usr/bin/env bash
# One safety cell: explore seed 0, then evaluate seed 1 only if distillation published.
set -u
cd /homeL/dong/Projects/dash-vla-libero
ROOT=/homeL/dong/Projects/dash-vla-libero
MEM="$ROOT/runs/g5_libero/l52b/memory"
AUDIT="$MEM/task-specific/affordance_t0_s0.json"
RECIPE="$MEM/task-specific/affordance_t0_s0_recipe.jsonl"
FAMILY="$MEM/task-family/task-family_affordance_L0_t0.md"

bash "$ROOT/scripts/run_h0_official_explore.sh"
code=$?
if [ "$code" -ne 0 ] || [ ! -f "$AUDIT" ] || [ ! -f "$RECIPE" ] || [ ! -f "$FAMILY" ]; then
  echo "STOP_AT_EXPLORATION exit=${code} audit=$([ -f "$AUDIT" ] && echo yes || echo no) recipe=$([ -f "$RECIPE" ] && echo yes || echo no) family=$([ -f "$FAMILY" ] && echo yes || echo no)"
  exit "$code"
fi
echo "MEMORY_DISTILLED $(date -Is)"
H0_CELLS=affordance:0:1 H0_OUT_SUBDIR=eval_s1 \
  bash "$ROOT/scripts/run_h0_official_harness.sh"
exit $?
