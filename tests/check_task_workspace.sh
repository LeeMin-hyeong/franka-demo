#!/usr/bin/env bash
# 초보자 안내의 새 ament_python 작업 패키지 생성/빌드/실행 진입점을 검증한다.
# 공식 의존성은 기존 프로젝트 install을 사용하며 로봇 서버/FCI를 시작하지 않는다.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/../scripts/common.sh"
live=false
case "${1:-}" in
  '') ;;
  --live) live=true ;;
  *) demo_die '사용법: bash tests/check_task_workspace.sh [--live]. --live는 실행 중인 가상 서버에서 상태를 읽습니다.' ;;
esac
[[ $# -le 1 ]] || demo_die '추가 인자를 지정할 수 없습니다.'
demo_require_ubuntu
demo_source_workspace
fixture="$(mktemp -d /tmp/fr3-task-ws-XXXXXX)"
cleanup() {
  # mktemp로 만든 독립 경로만 삭제한다. 사용자 워크스페이스에는 손대지 않는다.
  [[ "$fixture" == /tmp/fr3-task-ws-* && -d "$fixture/src/fr3_tasks" ]] || return
  rm -rf -- "$fixture"
}
trap cleanup EXIT
mkdir "$fixture/src"
cd "$fixture/src"
ros2 pkg create fr3_tasks --build-type ament_python \
  --node-name read_state --dependencies fr3_moveit_demo > "$fixture/create.log"
cp "$DEMO_WS/src/fr3_moveit_demo/examples/read_state.py" fr3_tasks/fr3_tasks/read_state.py
cd "$fixture"
colcon build --symlink-install --packages-select fr3_tasks
set +u
source install/setup.bash
set -u
ros2 pkg executables fr3_tasks | grep -qx 'fr3_tasks read_state'
python3 - <<'PY'
from importlib.metadata import distribution
from fr3_moveit_demo import Robot
entry = next(e for e in distribution('fr3_tasks').entry_points if e.name == 'read_state')
assert entry.group == 'console_scripts' and callable(entry.load())
print('TASK WORKSPACE PASS: package creation, build, installed console entry. No motion/FCI.')
PY
if "$live"; then
  # 기본 fake backend를 사용하므로 실물 서버에 연결하면 API가 거부한다.
  ros2 run fr3_tasks read_state
fi
