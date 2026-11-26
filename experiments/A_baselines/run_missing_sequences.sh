#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
A_ROOT="${A_ROOT:-$(cd "${REPO_ROOT}/../.." && pwd)}"
source "${REPO_ROOT}/scripts/A/activate_a_orb.sh"

python_bin="${A_CONDA_ENV}/bin/python"
runner="${A_RUNNER:-${A_ROOT}/build/A_baselines_repo/run_tartanair}"
vocab="${REPO_ROOT}/Vocabulary/ORBvoc.txt"
settings="${REPO_ROOT}/configs/A/tartanair.yaml"
data_root="${TARTANAIR_ROOT:-${A_ROOT}/data/TartanAirV2}"
output_root="${REPO_ROOT}/results/A"

run_case() {
  local sample_id="$1"
  local difficulty="$2"
  local trajectory="$3"
  local start_frame="$4"
  local inputs="${output_root}/full_sequences/inputs/${sample_id}"
  local runs="${output_root}/full_sequences/${sample_id}"

  if [[ ! -f "${inputs}/manifest.csv" ]]; then
    "${python_bin}" "${REPO_ROOT}/experiments/A_baselines/prepare_sequence.py" \
      --sequence-root "${data_root}/OldTownSummer/Data_${difficulty}/${trajectory}" \
      --start-frame "${start_frame}" \
      --num-frames 100 \
      --output "${inputs}"
  fi

  for mode in rgb rgb_imu rgbd_imu; do
    for seed in 0 1 2; do
      local run_dir="${runs}/${mode}_seed${seed}"
      if [[ -f "${run_dir}/metrics.json" ]]; then
        echo "SKIP ${sample_id} ${mode} ${seed}"
        continue
      fi
      "${python_bin}" "${REPO_ROOT}/experiments/A_baselines/run_baseline.py" \
        --binary "${runner}" --vocab "${vocab}" --settings "${settings}" \
        --mode "${mode}" --manifest "${inputs}/manifest.csv" \
        --imu "${inputs}/imu.csv" --output "${run_dir}" --seed "${seed}" \
        --orb-root "${REPO_ROOT}"
      local run_status=$?
      echo "RUN ${sample_id} ${mode} ${seed} exit=${run_status}"
      if [[ ! -s "${run_dir}/tracking.csv" ]]; then
        printf '{"status":"runner_failed_no_tracking","error":"runner exit %s; tracking.csv is missing or empty","ate_status":"unavailable","rpe_translation_status":"unavailable","rpe_rotation_status":"unavailable"}\n' \
          "${run_status}" > "${run_dir}/metrics.json"
        continue
      fi
      local alignment="se3"
      [[ "${mode}" == "rgb" ]] && alignment="sim3"
      "${python_bin}" "${REPO_ROOT}/experiments/A_baselines/evaluate_run.py" \
        --estimated "${run_dir}/trajectory_est.tum" \
        --reference "${inputs}/trajectory_gt.tum" \
        --tracking "${run_dir}/tracking.csv" \
        --output "${run_dir}/metrics.json" \
        --alignment "${alignment}" \
        --plot "${run_dir}/trajectory_overlay.png"
      echo "EVAL ${sample_id} ${mode} ${seed} exit=$?"
    done
  done
}

run_case easy_P001 easy P001 100
run_case hard_P000 hard P000 100
run_case hard_P002 hard P002 100
