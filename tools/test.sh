#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="${LIVO2_WS:-$(cd -- "$project_dir/../.." && pwd)}"
build_dir="$workspace_dir/build/$(basename -- "$project_dir")"
[[ -f "$build_dir/CTestTestfile.cmake" ]] || { echo "Build the project with LIVO2_BUILD_TESTS=ON first." >&2; exit 2; }
(cd "$build_dir" && "$project_dir/tools/env.sh" /usr/bin/ctest --output-on-failure)
"$project_dir/tools/env.sh" python3 "$project_dir/tools/check_environment.py"
"$project_dir/tools/env.sh" python3 "$project_dir/tools/regression/test_runtime.py"
"$project_dir/tools/env.sh" python3 "$project_dir/tools/regression/test_analysis.py"
