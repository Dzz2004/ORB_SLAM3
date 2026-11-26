#!/usr/bin/env bash
# Source this file after exporting A_ROOT to the workspace containing
# conda_envs/, install/, and data/.

if [[ -z "${A_ROOT:-}" ]]; then
  echo "A_ROOT is not set. Example:"
  echo "  export A_ROOT=/path/to/ORB_SLAM3_A"
  return 2 2>/dev/null || exit 2
fi

export A_CONDA_ENV="${A_CONDA_ENV:-${A_ROOT}/conda_envs/a_orb}"
export A_PANGOLIN_PREFIX="${A_PANGOLIN_PREFIX:-${A_ROOT}/install/pangolin}"

if [[ ! -x "${A_CONDA_ENV}/bin/python" ]]; then
  echo "Conda environment not found: ${A_CONDA_ENV}"
  return 2 2>/dev/null || exit 2
fi

export CONDA_PREFIX="${A_CONDA_ENV}"
export PATH="${A_CONDA_ENV}/bin:${A_PANGOLIN_PREFIX}/bin:${PATH}"
export CMAKE_PREFIX_PATH="${A_CONDA_ENV}:${A_PANGOLIN_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${A_CONDA_ENV}/lib:${A_PANGOLIN_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export LIBRARY_PATH="${A_CONDA_ENV}/lib${LIBRARY_PATH:+:${LIBRARY_PATH}}"
export CPATH="${A_CONDA_ENV}/include${CPATH:+:${CPATH}}"
export PKG_CONFIG_PATH="${A_CONDA_ENV}/lib/pkgconfig:${A_PANGOLIN_PREFIX}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
export LDFLAGS="-L${A_CONDA_ENV}/lib -Wl,-rpath,${A_CONDA_ENV}/lib -Wl,-rpath-link,${A_CONDA_ENV}/lib${LDFLAGS:+ ${LDFLAGS}}"

export TMPDIR="${A_ROOT}/tmp"
export XDG_CACHE_HOME="${A_ROOT}/cache/xdg"
export PIP_CACHE_DIR="${A_ROOT}/cache/pip"
export MPLCONFIGDIR="${A_ROOT}/cache/matplotlib"
export PYTHONPYCACHEPREFIX="${A_ROOT}/cache/pycache"
export CONDA_PKGS_DIRS="${A_ROOT}/conda_pkgs"
export MPLBACKEND="Agg"

export CC="${A_CONDA_ENV}/bin/x86_64-conda-linux-gnu-cc"
export CXX="${A_CONDA_ENV}/bin/x86_64-conda-linux-gnu-c++"

mkdir -p "${TMPDIR}" "${XDG_CACHE_HOME}" "${PIP_CACHE_DIR}"
mkdir -p "${MPLCONFIGDIR}" "${PYTHONPYCACHEPREFIX}" "${CONDA_PKGS_DIRS}"
