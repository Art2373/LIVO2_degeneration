#!/usr/bin/env bash
# Command wrapper; do not source. Explicit, isolated runtime environment.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="${LIVO2_WS:-$(cd -- "$project_dir/../.." && pwd)}"
[[ -f "$workspace_dir/devel/setup.bash" ]] || { echo "Build the independent workspace first." >&2; exit 2; }
exec env -i HOME="$HOME" USER="${USER:-jetson}" LOGNAME="${USER:-jetson}" \
  PATH=/usr/bin:/bin LANG=C.UTF-8 SHELL=/bin/bash \
  LIVO2_REGRESSION_ENV="${LIVO2_REGRESSION_ENV:-}" LIVO2_WS="$workspace_dir" ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11361}" \
  ROS_IP=127.0.0.1 ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace_dir/log/ros}" \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /bin/bash --noprofile --norc -c '
    set -e
    task_command=("$@")
    set --
    source /opt/ros/noetic/setup.bash
    source "$LIVO2_WS/devel/setup.bash" --extend
    export PYTHONPATH="$LIVO2_WS/runtime_python:${PYTHONPATH:-}"
    export LD_LIBRARY_PATH="$LIVO2_WS/deps/sophus/lib:${LD_LIBRARY_PATH:-}"
    mkdir -p "$ROS_LOG_DIR"
    if ((${#task_command[@]})); then exec "${task_command[@]}"; else exec /bin/bash --noprofile --norc -i; fi
  ' bash "$@"
