#!/usr/bin/env bash
# One later start after the Codex usage window named in the 05:34 failure.
set -u
cd /homeL/dong/Projects/dash-vla-libero
delay=$(( $(date -d 'today 08:06:30' +%s) - $(date +%s) ))
if [ "$delay" -lt 0 ]; then
  echo "RETRY_SKIP past 08:06:30"
  exit 0
fi
echo "RETRY_SLEEP ${delay}s until 08:06:30 $(date -Is)"
sleep "$delay"
if [ -d runs/g5_libero/h0_official/explore/affordance_t0_s0 ]; then
  mv runs/g5_libero/h0_official/explore/affordance_t0_s0 \
    runs/g5_libero/h0_official/explore/affordance_t0_s0_quota_before_0806
fi
echo "RETRY_BEGIN $(date -Is)"
exec bash scripts/run_h0_min_chain.sh
