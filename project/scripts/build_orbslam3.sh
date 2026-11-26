#!/usr/bin/env bash
# Build ORB-SLAM3 (Thirdparty DBoW2/g2o/Sophus + library + examples) against ROOT/env.
# Usage: scripts/build_orbslam3.sh [jobs]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
E="$ROOT/env"
SRC="$ROOT/src/ORB_SLAM3"
JOBS="${1:-32}"

export PATH="$E/bin:$PATH"
export CC="$E/bin/x86_64-conda-linux-gnu-gcc"
export CXX="$E/bin/x86_64-conda-linux-gnu-g++"
# The env is not activated, so pass its include/lib paths explicitly.
export CPPFLAGS="-isystem $E/include ${CPPFLAGS:-}"
export CXXFLAGS="-isystem $E/include ${CXXFLAGS:-}"
export LDFLAGS="-L$E/lib -Wl,-rpath,$E/lib ${LDFLAGS:-}"

# CMake 4 dropped compatibility with cmake_minimum_required(VERSION < 3.5).
CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
            -DCMAKE_PREFIX_PATH="$E")

for lib in DBoW2 g2o Sophus; do
  echo "== Thirdparty/$lib"
  cmake -S "$SRC/Thirdparty/$lib" -B "$SRC/Thirdparty/$lib/build" "${CMAKE_ARGS[@]}"
  cmake --build "$SRC/Thirdparty/$lib/build" -j "$JOBS"
done

if [ ! -f "$SRC/Vocabulary/ORBvoc.txt" ]; then
  echo "== Uncompress vocabulary"
  tar -xf "$SRC/Vocabulary/ORBvoc.txt.tar.gz" -C "$SRC/Vocabulary"
fi

echo "== ORB_SLAM3"
cmake -S "$SRC" -B "$SRC/build" "${CMAKE_ARGS[@]}"
cmake --build "$SRC/build" -j "$JOBS"
