#!/usr/bin/env bash
# Usage: run_orbslam.sh MODE SEQ_DIR OUT_DIR [runner options: --rate R --imu FILE --max-frames N --start S]
# MODE: MONOCULAR | IMU_MONOCULAR | RGBD | IMU_RGBD. Env: CALIB (default SEQ_DIR/calib.yaml), RUN_TIMEOUT (s, default 1800),
# IMU_SEED=s -> --imu SEQ_DIR/imu_noisy_s<s>.csv (formal B/C experiments; default is the noise-free imu.csv)
set -uo pipefail
ulimit -c 0  # ORB-SLAM3 may segfault at shutdown; do not leave core files
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[ $# -ge 3 ] || { sed -n 2,3p "$0"; exit 2; }
MODE="$1"; SEQ="$(cd "$2" && pwd)" || exit 2
mkdir -p "$3"; OUT="$(cd "$3" && pwd)"; shift 3
CALIB="${CALIB:-$SEQ/calib.yaml}"
if [ -n "${IMU_SEED:-}" ]; then
  [ -f "$SEQ/imu_noisy_s$IMU_SEED.csv" ] || { echo "missing $SEQ/imu_noisy_s$IMU_SEED.csv" >&2; exit 2; }
  set -- --imu "$SEQ/imu_noisy_s$IMU_SEED.csv" "$@"
fi
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
cd "$OUT"
timeout -k 10 "${RUN_TIMEOUT:-1800}" "$ROOT/bin/run_orbslam" "$MODE" \
  "$ROOT/src/ORB_SLAM3/Vocabulary/ORBvoc.txt" "$CALIB" "$SEQ" "$OUT" "$@" > "$OUT/run.log" 2>&1
code=$?
cat > "$OUT/run_info.json" <<J
{"mode": "$MODE", "sequence": "$SEQ", "calib": "$CALIB", "args": "$*", "imu_seed": "${IMU_SEED:-}", "exit_code": $code,
 "orbslam3_commit": "$(git -C "$ROOT/src/ORB_SLAM3" rev-parse HEAD)"}
J
exit $code
