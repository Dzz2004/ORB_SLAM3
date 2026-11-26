#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}/../.." rev-parse --show-toplevel)"

if [[ -z "${A_ROOT:-}" ]]; then
  echo "Set A_ROOT before building Pangolin."
  exit 2
fi

source "${REPO_ROOT}/scripts/A/activate_a_orb.sh"

PANGOLIN_SOURCE="${PANGOLIN_SOURCE:-${A_ROOT}/src/Pangolin}"
PANGOLIN_BUILD="${PANGOLIN_BUILD:-${A_ROOT}/build/Pangolin}"
export A_PANGOLIN_PREFIX="${A_PANGOLIN_PREFIX:-${A_ROOT}/install/pangolin}"

if [[ ! -d "${PANGOLIN_SOURCE}/.git" ]]; then
  mkdir -p "$(dirname "${PANGOLIN_SOURCE}")"
  git clone --depth 1 --branch v0.6 \
    https://github.com/stevenlovegrove/Pangolin.git "${PANGOLIN_SOURCE}"
fi

cmake -S "${PANGOLIN_SOURCE}" -B "${PANGOLIN_BUILD}" -G Ninja \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${A_PANGOLIN_PREFIX}" \
  -DCMAKE_CXX_FLAGS="-include cstdint -include limits" \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_TOOLS=OFF \
  -DBUILD_PANGOLIN_PYTHON=OFF \
  -DBUILD_PANGOLIN_VIDEO=OFF \
  -DBUILD_PANGOLIN_FFMPEG=OFF \
  -DBUILD_PANGOLIN_V4L=OFF \
  -DBUILD_PANGOLIN_LIBDC1394=OFF
cmake --build "${PANGOLIN_BUILD}" --parallel 8
cmake --install "${PANGOLIN_BUILD}"

echo "Pangolin installed at ${A_PANGOLIN_PREFIX}"
