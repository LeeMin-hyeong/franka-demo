#!/usr/bin/env bash
# Fake Hardware만 선택하여 RGB-D/보정/집기 작업을 headless 검증한다.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
[[ $# == 0 ]] || demo_die '사용법: bash scripts/test_pick.sh (항상 가상/headless)'
demo_require_ubuntu
demo_source_workspace
cd "$DEMO_ROOT"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-131}" ROS_LOCALHOST_ONLY=1
server_log="$DEMO_WS/pick-server.log"
client_log="$DEMO_WS/pick-client.log"
server_pid=''
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$server_pid" ]]; then
    kill -INT "$server_pid" 2>/dev/null || true
    wait "$server_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid env --default-signal=INT bash scripts/run.sh server --no-rviz > "$server_log" 2>&1 &
server_pid=$!
ready=false
for attempt in $(seq 1 110); do
  if grep -q '서버 준비 완료' "$server_log"; then ready=true; break; fi
  if ! kill -0 "$server_pid" 2>/dev/null; then cat "$server_log"; demo_die '가상 서버가 종료됐습니다.'; fi
  sleep 1
done
"$ready" || { cat "$server_log"; demo_die '가상 서버 준비 timeout'; }
client=tests/check_pick_cycle.py
if [[ "${FR3_PICK_GUI:-}" == main ]]; then client=tests/check_pick_gui.py; fi
if ! python3 "$client" > "$client_log" 2>&1; then
  cat "$client_log"
  demo_die '보정 좌표 기반 집기 작업 실패'
fi
grep -E 'CALIBRATED SELECT PASS|PICK CYCLE PASS|PICK GUI PASS' "$client_log"
