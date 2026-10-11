#!/usr/bin/env bash
# 실제 설치 대신 임시 하위 스크립트로 start.sh 옵션 전달/실패 중단을 확인한다.
# GUI가 없는 설치 전용 실행이 로봇/카메라 실행으로 이어지지 않는지 검증한다.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
fixture="$(mktemp -d /tmp/fr3-install-entrypoint-XXXXXX)"
cleanup() {
  [[ "$fixture" == /tmp/fr3-install-entrypoint-* && -d "$fixture/scripts" ]] || return
  rm -f -- "$fixture/scripts"/* "$fixture/events" "$fixture/output"
  rmdir -- "$fixture/scripts" "$fixture"
}
trap cleanup EXIT
mkdir "$fixture/scripts"
cp "$repo_root/scripts/start.sh" "$fixture/scripts/start.sh"
cat > "$fixture/scripts/common.sh" <<'COMMON'
set -euo pipefail
DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
demo_require_ubuntu() { :; }
demo_die() { printf '%s\n' "$*" >&2; exit 1; }
COMMON
for name in setup build doctor run; do
  cat > "$fixture/scripts/$name.sh" <<'STUB'
#!/usr/bin/env bash
printf '%s %s\n' "$(basename -- "$0" .sh)" "$*" >> "$EVENTS"
[[ "${FAIL_STEP:-}" != "$(basename -- "$0" .sh)" ]] || exit 42
STUB
done
export EVENTS="$fixture/events"
run_case() {
  local label="$1" expected="$2" expected_events="$3"
  shift 3
  : > "$EVENTS"
  local status=0
  env -u DISPLAY -u WAYLAND_DISPLAY bash "$fixture/scripts/start.sh" "$@" > "$fixture/output" 2>&1 || status=$?
  [[ "$status" == "$expected" ]] || { cat "$fixture/output"; printf 'FAIL %s: %s\n' "$label" "$status"; exit 1; }
  [[ "$(cat "$EVENTS")" == "$expected_events" ]] || { cat "$EVENTS"; printf 'FAIL %s: 호출 순서/옵션\n' "$label"; exit 1; }
  printf 'PASS: %s\n' "$label"
}
base_events=$'setup \nbuild \ndoctor '
camera_events=$'setup --with-camera\nbuild \ndoctor --camera'
run_case install-only 0 "$base_events" --install-only
run_case all-software 0 "$camera_events" --install-only --with-camera
run_case headless-example 0 "$base_events"$'\nrun example --no-rviz' --no-rviz
run_case camera-headless-example 0 "$camera_events"$'\nrun example --no-rviz' --with-camera --no-rviz
run_case missing-gui 1 ''
run_case unknown-option 1 '' --real
FAIL_STEP=build run_case stop-after-build-failure 42 $'setup --with-camera\nbuild ' --with-camera --install-only
FAIL_STEP=doctor run_case stop-after-doctor-failure 42 "$camera_events" --with-camera --no-rviz
# 잘못된 설치 조합은 apt/ROS 환경 조회 전에 거부한다.
if bash "$repo_root/scripts/setup.sh" --sources-only --with-camera > "$fixture/output" 2>&1; then
  printf 'FAIL: conflicting setup options accepted\n'; exit 1
fi
grep -q '함께 사용할 수 없습니다' "$fixture/output"
printf 'PASS: conflicting setup options rejected before installation\n'
printf 'Install entrypoint: 9/9 PASS (실제 설치/노드 실행 없음)\n'
