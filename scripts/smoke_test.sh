#!/usr/bin/env bash
# 설치 후 자동 확인: Fake Hardware의 Python 팔/그리퍼 예제와 별도 최종 상태 검사.
# 실제 드라이버를 선택할 인자가 없다. 이 테스트의 launch만 정리한다.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
[[ $# == 0 ]] || demo_die '사용법: ./scripts/smoke_test.sh (항상 가상/headless)'
demo_require_ubuntu
demo_source_workspace
cd "$DEMO_ROOT"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-79}" ROS_LOCALHOST_ONLY=1
server_log="$DEMO_WS/smoke-server.log"
client_log="$DEMO_WS/smoke-client.log"
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
if ! bash scripts/run.sh example-command --duration 1 > "$client_log" 2>&1; then
  cat "$client_log"
  demo_die 'Python 작업 예제 실패'
fi
grep 'SEQUENCE COMPLETE' "$client_log"
python3 tests/check_fake_state.py
printf 'SMOKE TEST PASS: 실제 FCI 연결 없이 팔/그리퍼 제어와 최종 상태를 확인했습니다.\n'
printf '실행/종료 로그: %s\n' "$server_log"
