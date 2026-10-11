#!/usr/bin/env bash
# Independent source build. No inherited ROS overlay or prebuilt core library.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="${LIVO2_WS:-$(cd -- "$project_dir/../.." && pwd)}"
if [[ "$project_dir" != "$workspace_dir"/src/* ]]; then
  echo "Place LIVO2_degeneration below <workspace>/src, or set LIVO2_WS to that workspace." >&2; exit 2
fi
if [[ "${LIVO2_BUILD_ISOLATED:-}" != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-jetson}" LOGNAME="${USER:-jetson}" \
    PATH=/usr/bin:/bin LANG=C.UTF-8 SHELL=/bin/bash LIVO2_BUILD_ISOLATED=1 \
    LIVO2_WS="$workspace_dir" LIVO2_JOBS="${LIVO2_JOBS:-2}" \
    LIVO2_OPENCV_DIR="${LIVO2_OPENCV_DIR:-/usr/lib/aarch64-linux-gnu/cmake/opencv4}" \
    LIVO2_DRIVER2="${LIVO2_DRIVER2:-ON}" LIVO2_OPENCV_PYTHON_DIR="${LIVO2_OPENCV_PYTHON_DIR:-/usr/lib/python3.8/dist-packages}" \
    /bin/bash "$project_dir/tools/build.sh" "$@"
fi
set +u
source /opt/ros/noetic/setup.bash
set -u
[[ -f "$LIVO2_OPENCV_DIR/OpenCVConfig.cmake" ]] || { echo "Set LIVO2_OPENCV_DIR to matching system OpenCV CMake directory." >&2; exit 2; }
# Keep source dependencies inside this release, exposed as a sibling catkin tree.
deps_link="$workspace_dir/src/livo2_third_party"
if [[ -e "$deps_link" || -L "$deps_link" ]]; then
  [[ -L "$deps_link" && "$(readlink -f -- "$deps_link")" == "$project_dir/third_party/catkin" ]] || { echo "Refusing existing non-matching $deps_link" >&2; exit 2; }
else
  ln -s "$project_dir/third_party/catkin" "$deps_link"
fi
mkdir -p "$workspace_dir/deps" "$workspace_dir/runtime_python"
cv2_source="$LIVO2_OPENCV_PYTHON_DIR/cv2"
[[ -e "$cv2_source" ]] || { echo "Matching cv2 source absent: $cv2_source" >&2; exit 2; }
if [[ -e "$workspace_dir/runtime_python/cv2" || -L "$workspace_dir/runtime_python/cv2" ]]; then
  [[ "$(readlink -f -- "$workspace_dir/runtime_python/cv2")" == "$(readlink -f -- "$cv2_source")" ]] || { echo "Conflicting runtime Python cv2; use a fresh workspace." >&2; exit 2; }
else
  ln -s "$cv2_source" "$workspace_dir/runtime_python/cv2"
fi
export PYTHONPATH="$workspace_dir/runtime_python:${PYTHONPATH:-}"
cmake -S "$project_dir/third_party/sophus" -B "$workspace_dir/build_sophus" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$workspace_dir/deps/sophus"
cmake --build "$workspace_dir/build_sophus" --parallel "$LIVO2_JOBS"
cmake --install "$workspace_dir/build_sophus"
cd "$workspace_dir"
catkin_make -j"$LIVO2_JOBS" -l"$LIVO2_JOBS" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=OFF \
  -DCMAKE_PREFIX_PATH=/opt/ros/noetic \
  -DSophus_DIR="$workspace_dir/deps/sophus/share/Sophus/cmake" \
  -DOpenCV_DIR="$LIVO2_OPENCV_DIR" \
  -DFASTLIVO_USE_LIVOX_ROS_DRIVER2="$LIVO2_DRIVER2" "$@"
echo "Built: $workspace_dir/devel/lib/livo2_degeneration/fusion_mapping"
