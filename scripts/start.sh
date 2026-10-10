#!/usr/bin/env bash
# 최초 사용자 진입점: ROS 설치 -> 공식 의존성 -> 빌드 -> Python 가상 제어.
# 실물 서버는 자동 시작하지 않는다. 실물 준비와 명령은 별도 run.sh real-* 모드다.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
case "${1:-}" in
  -h|--help)
    printf '사용법: ./scripts/start.sh [--no-rviz]\n'
    printf 'ROS가 없는 Ubuntu 22.04부터 설치/빌드하고 Python 팔+가상 그리퍼 예제를 실행합니다.\n'
    printf '시스템 패키지를 설치하며 일반 사용자로 실행하면 sudo 비밀번호를 요청합니다.\n'
    exit 0 ;;
  ''|--no-rviz) ;;
  *) demo_die 'start.sh는 가상 예제용입니다. 선택 옵션은 --no-rviz뿐입니다.' ;;
esac
[[ $# -le 1 ]] || demo_die '옵션이 너무 많습니다.'
demo_require_ubuntu
if [[ $# == 0 && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  demo_die 'GUI 세션이 없습니다. Ubuntu Desktop/WSLg에서 실행하거나 --no-rviz를 사용하세요.'
fi
cd "$DEMO_ROOT"
bash scripts/setup.sh
bash scripts/build.sh
bash scripts/doctor.sh
exec bash scripts/run.sh example "$@"
