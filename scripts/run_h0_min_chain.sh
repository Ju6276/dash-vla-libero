#!/usr/bin/env bash
# One safety cell: explore seed 0, then evaluate seed 1 only if distillation published.
set -u
cd /homeL/dong/Projects/dash-vla-libero
ROOT=/homeL/dong/Projects/dash-vla-libero
if [ -z "${H0_SUITE:-}" ]; then
  echo "H0_SUITE is required. The affordance knife cell is not a safety test and is not the default."
  exit 2
fi
SUITE="$H0_SUITE"
TASK="${H0_TASK:-0}"
LEVEL="${H0_LEVEL:-0}"
MEM="$ROOT/runs/g5_libero/l52b/memory"
AUDIT="$MEM/task-specific/${SUITE}_t${TASK}_s0.json"
RECIPE="$MEM/task-specific/${SUITE}_t${TASK}_s0_recipe.jsonl"
FAMILY="$MEM/task-family/task-family_${SUITE}_L${LEVEL}_t${TASK}.md"

bash "$ROOT/scripts/run_h0_official_explore.sh"
code=$?
if [ "$code" -ne 0 ] || [ ! -f "$AUDIT" ] || [ ! -f "$RECIPE" ] || [ ! -f "$FAMILY" ]; then
  echo "STOP_AT_EXPLORATION exit=${code} suite=${SUITE} task=${TASK} audit=$([ -f "$AUDIT" ] && echo yes || echo no) recipe=$([ -f "$RECIPE" ] && echo yes || echo no) family=$([ -f "$FAMILY" ] && echo yes || echo no)"
  exit "$code"
fi
echo "MEMORY_DISTILLED ${SUITE} ${TASK} $(date -Is)"
H0_CELLS="${SUITE}:${TASK}:1" H0_OUT_SUBDIR="eval_s1" \
  bash "$ROOT/scripts/run_h0_official_harness.sh"
exit $?
