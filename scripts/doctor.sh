#!/usr/bin/env bash
# 시스템/설치만 확인한다. FCI 접속이나 이동 목표를 보내지 않는다.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
real=false
case "${1:-}" in
  '') ;;
  --real) real=true ;;
  -h|--help)
    printf '사용법: ./scripts/doctor.sh [--real]\n'
    printf '설치/빌드 확인. --real은 native Ubuntu의 RT 커널과 현재 셸 권한도 확인합니다.\n'
    exit 0 ;;
  *) demo_die '알 수 없는 doctor 옵션입니다.' ;;
esac
[[ $# -le 1 ]] || demo_die '옵션이 너무 많습니다.'
demo_require_ubuntu
demo_source_workspace
demo_check_sources
for package in fr3_moveit_demo fr3_offline_preview franka_fr3_moveit_config franka_hardware franka_gripper franka_msgs std_srvs rviz2; do
  ros2 pkg prefix "$package" >/dev/null || demo_die "패키지가 없습니다: $package. setup/build를 확인하세요."
done
python3 -c 'from fr3_moveit_demo import Robot; from franka_msgs.action import Move; from std_srvs.srv import Trigger; Move.Goal(width=0.04, speed=0.01); Trigger.Request(); print("Python Robot API / real message types import: OK")'
printf 'Ubuntu 22.04 / ROS Humble / 공식 소스 / 데모 설치: OK\n'
if "$real"; then
  demo_require_real_pc
  printf '실물 PC의 RT 커널/현재 셸 권한: OK\n'
  printf '로봇 IP, System Image >=5.9.0, FCI 활성화, 직결 네트워크는 docs/real-robot.md에서 확인하세요.\n'
  printf '이 점검은 로봇 연결이나 실제 동작 검증을 수행하지 않았습니다.\n'
elif [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  printf 'GUI 없음: ./scripts/run.sh example --no-rviz로 가상 제어를 확인할 수 있습니다.\n'
fi
