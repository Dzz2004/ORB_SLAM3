#!/usr/bin/env bash
# Build scripts/run_orbslam.cc -> bin/run_orbslam (after scripts/build_orbslam3.sh). Env BIN overrides the output path.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
E="$ROOT/env"; S="$ROOT/src/ORB_SLAM3"
mkdir -p "$ROOT/bin"
export PATH="$E/bin:$PATH"
# -O3 -march=native must match the library (Eigen alignment ABI).
"$E/bin/x86_64-conda-linux-gnu-g++" -std=c++11 -O3 -march=native -DCOMPILEDWITHC11 -w \
  -I"$S" -I"$S/include" -I"$S/include/CameraModels" -I"$S/Thirdparty/Sophus" \
  -I"$E/include/eigen3" -I"$E/include/opencv4" -I"$E/include" \
  "$ROOT/scripts/run_orbslam.cc" -o "${BIN:-$ROOT/bin/run_orbslam}" \
  -L"$S/lib" -lORB_SLAM3 -L"$S/Thirdparty/DBoW2/lib" -lDBoW2 -L"$S/Thirdparty/g2o/lib" -lg2o \
  -L"$E/lib" -lpangolin -lopencv_imgcodecs -lopencv_core -lboost_serialization -lcrypto -lGLEW -lGL -lpthread \
  -Wl,-rpath-link,"$E/lib" -Wl,-rpath,"$S/lib:$S/Thirdparty/DBoW2/lib:$S/Thirdparty/g2o/lib:$E/lib"
