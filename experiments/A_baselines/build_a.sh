#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"

if [[ -z "${A_ROOT:-}" ]]; then
  echo "Set A_ROOT to the workspace containing data/, conda_envs/, and install/."
  exit 2
fi

source "${REPO_ROOT}/scripts/A/activate_a_orb.sh"

if [[ ! -f "${A_PANGOLIN_PREFIX}/lib/libpangolin.so" ]]; then
  echo "Pangolin is missing at ${A_PANGOLIN_PREFIX}."
  echo "Build Pangolin v0.6 first and set A_PANGOLIN_PREFIX if necessary."
  exit 2
fi

VOCAB="${REPO_ROOT}/Vocabulary/ORBvoc.txt"
if [[ ! -s "${VOCAB}" ]]; then
  tar -xzf "${REPO_ROOT}/Vocabulary/ORBvoc.txt.tar.gz" -C "${REPO_ROOT}/Vocabulary"
fi

BUILD_ROOT="${A_ROOT}/build"
RUNNER_BUILD_ROOT="${A_RUNNER_BUILD_ROOT:-${BUILD_ROOT}/A_baselines_repo}"
cmake -S "${REPO_ROOT}/Thirdparty/DBoW2" \
  -B "${BUILD_ROOT}/DBoW2" \
  -G Ninja \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_ROOT}/DBoW2" --parallel 8

cmake -S "${REPO_ROOT}" \
  -B "${BUILD_ROOT}/ORB_SLAM3" \
  -G Ninja \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_ROOT}/ORB_SLAM3" --target ORB_SLAM3 --parallel 8

cmake -S "${REPO_ROOT}/experiments/A_baselines" \
  -B "${RUNNER_BUILD_ROOT}" \
  -G Ninja \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_BUILD_TYPE=Release \
  -DORB_SLAM3_ROOT="${REPO_ROOT}"
cmake --build "${RUNNER_BUILD_ROOT}" --parallel 8

echo "A runner: ${RUNNER_BUILD_ROOT}/run_tartanair"
