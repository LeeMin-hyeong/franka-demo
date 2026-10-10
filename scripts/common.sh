#!/usr/bin/env bash
# 스크립트가 공유하는 경로와 Ubuntu/ROS 환경 확인. 제어 코드는 실행하지 않는다.
set -euo pipefail

DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEMO_WS="$DEMO_ROOT/ros2_ws"
DEMO_VENDOR="$DEMO_WS/src/vendor"
DEMO_FRANKA_ROS2_REVISION=3b50164eabbe2d461baca007bbbbe03c024387bd
DEMO_PACKAGES=(
  libfranka franka_description franka_msgs franka_hardware
  franka_semantic_components franka_robot_state_broadcaster franka_gripper
  franka_bringup franka_fr3_moveit_config fr3_moveit_demo fr3_offline_preview
)

demo_die() {
  printf '오류: %s\n' "$*" >&2
  exit 1
}

demo_require_ubuntu() {
  [[ -f /etc/os-release ]] || demo_die 'Ubuntu 22.04 터미널에서 실행하세요.'
  # Ubuntu의 배포판 정보만 읽으며 시스템 파일을 수정하지 않는다.
  source /etc/os-release
  [[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] ||
    demo_die '이 데모의 지원 환경은 Ubuntu 22.04 + ROS2 Humble입니다.'
}

demo_source_ros() {
  [[ -f /opt/ros/humble/setup.bash ]] ||
    demo_die 'ROS2 Humble을 먼저 설치하세요. README 4절의 설치 명령을 참고하세요.'
  # ROS의 환경 스크립트는 정의되지 않은 변수를 읽을 수 있으므로 source 동안만 해제한다.
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  [[ "${ROS_DISTRO:-}" == humble ]] || demo_die 'ROS_DISTRO가 humble이 아닙니다.'
}

demo_source_workspace() {
  demo_source_ros
  [[ -f "$DEMO_WS/install/setup.bash" ]] ||
    demo_die '빌드 결과가 없습니다. ./scripts/build.sh를 먼저 실행하세요.'
  set +u
  source "$DEMO_WS/install/setup.bash"
  set -u
}

demo_check_sources() {
  [[ -d "$DEMO_VENDOR/franka_ros2/.git" ]] ||
    demo_die '공식 소스가 없습니다. ./scripts/setup.sh를 먼저 실행하세요.'
  local actual
  actual="$(git -C "$DEMO_VENDOR/franka_ros2" rev-parse HEAD)"
  [[ "$actual" == "$DEMO_FRANKA_ROS2_REVISION" ]] ||
    demo_die "기존 franka_ros2가 검증한 Humble 커밋과 다릅니다: $actual. 기존 파일은 보존했습니다."
  local source_name expected_tag
  for source_name in franka_description libfranka; do
    case "$source_name" in
      franka_description) expected_tag=2.8.1 ;;
      libfranka) expected_tag=0.20.4 ;;
    esac
    [[ -d "$DEMO_VENDOR/$source_name/.git" ]] ||
      demo_die "$source_name 소스가 없습니다. setup.sh를 실행하세요."
    actual="$(git -C "$DEMO_VENDOR/$source_name" rev-parse HEAD)"
    [[ "$actual" == "$(git -C "$DEMO_VENDOR/$source_name" rev-parse "$expected_tag^{commit}")" ]] ||
      demo_die "$source_name 소스가 공식 dependency.repos의 $expected_tag 버전과 다릅니다. 기존 파일은 보존했습니다."
  done
}
