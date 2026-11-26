#!/usr/bin/env bash
# Run a B matrix with core dumps off and all HOME-based caches redirected to a scratch dir.
# Usage: run_B.sh <matrix yaml> <workers> [extra run_matrix.py args...]
# Each invocation gets its own scratch dir under $SCRATCH_ROOT (default ROOT/.scratch), removed on exit.
set -uo pipefail
ulimit -c 0
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
[ $# -ge 2 ] || { sed -n 2,4p "$0"; exit 2; }
MATRIX="$1"; W="$2"; shift 2
mkdir -p "${SCRATCH_ROOT:=$ROOT/.scratch}" && SCR="$(mktemp -d -p "$SCRATCH_ROOT")" || exit 1
export TMPDIR="$SCR" TMP="$SCR" TEMP="$SCR" MPLCONFIGDIR="$SCR/mpl" XDG_CACHE_HOME="$SCR/cache" \
       XDG_CONFIG_HOME="$SCR/config" PIP_CACHE_DIR="$SCR/pip" npm_config_cache="$SCR/npm" CONDA_PKGS_DIRS="$SCR/conda"
mkdir -p "$MPLCONFIGDIR" "$XDG_CACHE_HOME" "$XDG_CONFIG_HOME"
cd "$ROOT"
echo "[run_B] $(date '+%F %T') matrix=$MATRIX workers=$W scratch=$SCR"
env_data/bin/python scripts/run_matrix.py "$MATRIX" --workers "$W" --corrupt-workers 1 --python "$ROOT/env_data/bin/python" "$@"
code=$?
rm -rf -- "$SCR"
echo "[run_B] $(date '+%F %T') exit=$code"
exit $code
