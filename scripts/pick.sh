#!/usr/bin/env bash
# CV 선택 작업의 Python 진입점. 서버/카메라는 별도 터미널에서 준비한다.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
demo_require_ubuntu
demo_source_workspace
exec python3 "$DEMO_WS/src/fr3_moveit_demo/examples/pick_and_return.py" "$@"
