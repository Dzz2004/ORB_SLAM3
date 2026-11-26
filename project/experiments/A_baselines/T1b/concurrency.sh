#!/usr/bin/env bash
# T1b: effect of concurrent runs on ORB-SLAM3 results (easy, IMU_RGBD / IMU_MONOCULAR, IMU seed 0).
# Usage: concurrency.sh OUT_DIR [levels...]   level 1 = solo (3 sequential runs per mode)
# Level N>1 launches N runs at once, half per mode.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT="$1"; shift; LEVELS=("${@:-1 16 32 48}")
SEQ="$ROOT/data/seqs/office_easy_p004"
export IMU_SEED=0 RUN_TIMEOUT=900
mkdir -p "$OUT"
load() { echo "$(date +%T) $1 $(cut -d' ' -f1-3 /proc/loadavg)" >> "$OUT/load.log"; }
for N in ${LEVELS[@]}; do
  if [ "$N" = 1 ]; then
    for r in 0 1 2; do for m in IMU_RGBD IMU_MONOCULAR; do
      load "c1 start $m r$r"
      "$ROOT/scripts/run_orbslam.sh" $m "$SEQ" "$OUT/c1/${m}__r$r"
    done; done
  else
    load "c$N start"; pids=()
    ( while sleep 20; do load "c$N during"; done ) & mon=$!
    for ((i = 0; i < N; i++)); do
      m=$([ $((i % 2)) = 0 ] && echo IMU_RGBD || echo IMU_MONOCULAR)
      "$ROOT/scripts/run_orbslam.sh" $m "$SEQ" "$OUT/c$N/${m}__r$((i / 2))" & pids+=($!)
    done
    for p in "${pids[@]}"; do wait $p; done
    kill $mon 2>/dev/null
  fi
  load "c$N end"
done
