#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
if [[ -z "${A_ROOT:-}" ]]; then
  A_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"
  export A_ROOT
fi
source "${REPO_ROOT}/scripts/A/activate_a_orb.sh"

root="${A_ROOT}"
python_bin="${A_CONDA_ENV}/bin/python"
runner="${A_RUNNER:-${root}/build/A_baselines_repo/run_tartanair}"
vocab="${REPO_ROOT}/Vocabulary/ORBvoc.txt"
settings="${REPO_ROOT}/configs/A/tartanair.yaml"
data_root="${TARTANAIR_ROOT:-${root}/data/TartanAirV2}"

prepare_case() {
  local difficulty="$1"
  local trajectory="$2"
  local start_frame="$3"
  local inputs="$4"
  local sequence="${data_root}/OldTownSummer/Data_${difficulty}/${trajectory}"

  if [[ ! -f "${inputs}/manifest.csv" ]]; then
    "${python_bin}" "${REPO_ROOT}/experiments/A_baselines/prepare_sequence.py" \
      --sequence-root "${sequence}" \
      --start-frame "${start_frame}" \
      --num-frames 100 \
      --output "${inputs}"
  fi
}

run_case() {
  local difficulty="$1"
  local trajectory="$2"
  local start_frame="$3"
  local inputs="$4"
  local run_root="$5"

  prepare_case "${difficulty}" "${trajectory}" "${start_frame}" "${inputs}"
  for mode in rgb rgb_imu rgbd_imu; do
    for seed in 0 1 2; do
      local run_dir="${run_root}/${mode}_seed${seed}"
      if [[ -f "${run_dir}/metrics.json" ]]; then
        echo "SKIP ${trajectory} ${difficulty} ${mode} ${seed}"
        continue
      fi

      "${python_bin}" "${REPO_ROOT}/experiments/A_baselines/run_baseline.py" \
        --binary "${runner}" \
        --vocab "${vocab}" \
        --settings "${settings}" \
        --mode "${mode}" \
        --manifest "${inputs}/manifest.csv" \
        --imu "${inputs}/imu.csv" \
        --output "${run_dir}" \
        --seed "${seed}" \
        --orb-root "${REPO_ROOT}"
      local run_status=$?
      echo "RUN ${trajectory} ${difficulty} ${mode} ${seed}: exit=${run_status}"
      if [[ ! -s "${run_dir}/tracking.csv" ]]; then
        printf '{"status":"runner_failed_no_tracking","error":"runner exit %s; tracking.csv is missing or empty","ate_status":"unavailable","rpe_translation_status":"unavailable","rpe_rotation_status":"unavailable"}\n' \
          "${run_status}" > "${run_dir}/metrics.json"
        continue
      fi

      local alignment="se3"
      if [[ "${mode}" == "rgb" ]]; then
        alignment="sim3"
      fi
      "${python_bin}" "${REPO_ROOT}/experiments/A_baselines/evaluate_run.py" \
        --estimated "${run_dir}/trajectory_est.tum" \
        --reference "${inputs}/trajectory_gt.tum" \
        --tracking "${run_dir}/tracking.csv" \
        --output "${run_dir}/metrics.json" \
        --alignment "${alignment}" \
        --plot "${run_dir}/trajectory_overlay.png"
      echo "EVAL ${trajectory} ${difficulty} ${mode} ${seed}: exit=$?"
    done
  done
}

run_case \
  easy P000 200 \
  "${REPO_ROOT}/results/A/inputs_final/OldTownSummer/easy/P000_2p0s_9p9s" \
  "${REPO_ROOT}/results/A/baseline_final/easy/P000_window_200_299"

run_case \
  easy P002 200 \
  "${REPO_ROOT}/results/A/validation/P002/easy/window_200_299" \
  "${REPO_ROOT}/results/A/validation/P002/easy"

# The hard interval start is selected from map continuity and coverage only.
# Numerical accuracy is evaluated separately on the common 12.9-14.9 s window.
run_case \
  hard P001 100 \
  "${REPO_ROOT}/results/A/inputs_final/OldTownSummer/hard/P001_start100" \
  "${REPO_ROOT}/results/A/baseline_final/hard/P001_window_start100"

"${python_bin}" "${REPO_ROOT}/experiments/A_baselines/evaluate_hard_common.py" \
  --root "${REPO_ROOT}"
"${python_bin}" "${REPO_ROOT}/experiments/A_baselines/evaluate_map_segments.py" \
  --root "${REPO_ROOT}"
"${python_bin}" "${REPO_ROOT}/experiments/A_baselines/summarize_baseline.py" \
  --root "${REPO_ROOT}"

bash "${REPO_ROOT}/experiments/A_baselines/run_missing_sequences.sh"
"${python_bin}" "${REPO_ROOT}/experiments/A_baselines/summarize_all_sequences.py" \
  --root "${REPO_ROOT}"
"${python_bin}" "${REPO_ROOT}/experiments/A_baselines/plot_available_trajectories.py" \
  --root "${REPO_ROOT}"
