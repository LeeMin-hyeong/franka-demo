#!/usr/bin/env bash
# 최초 사용자 진입점: ROS 설치 -> 공식 의존성 -> 빌드 -> Python 가상 제어.
# 실물 서버는 자동 시작하지 않는다. 실물 준비와 명령은 별도 run.sh real-* 모드다.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
with_camera=false
install_only=false
no_rviz=false
while (($#)); do
case "$1" in
  -h|--help)
    printf '사용법: ./scripts/start.sh [--with-camera] [--install-only] [--no-rviz]\n'
    printf 'ROS가 없는 Ubuntu 22.04부터 설치/빌드하고 Python 팔+가상 그리퍼 예제를 실행합니다.\n'
    printf '시스템 패키지를 설치하며 일반 사용자로 실행하면 sudo 비밀번호를 요청합니다.\n'
    printf '%s\n' '--with-camera: D435i SDK/드라이버 포함. --install-only: 설치/빌드/점검 후 종료.'
    exit 0 ;;
  --with-camera) with_camera=true ;;
  --install-only) install_only=true ;;
  --no-rviz) no_rviz=true ;;
  *) demo_die "알 수 없는 옵션: $1. --help를 확인하세요." ;;
esac
shift
done
demo_require_ubuntu
if ! "$install_only" && ! "$no_rviz" && [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  demo_die 'GUI 세션이 없습니다. Ubuntu Desktop에서 실행하거나 --no-rviz를 사용하세요.'
fi
cd "$DEMO_ROOT"
setup_args=()
doctor_args=()
if "$with_camera"; then
  setup_args+=(--with-camera)
  doctor_args+=(--camera)
fi
bash scripts/setup.sh "${setup_args[@]}"
bash scripts/build.sh
bash scripts/doctor.sh "${doctor_args[@]}"
if "$install_only"; then
  printf '설치/빌드/환경 점검 완료. 카메라와 로봇 서버는 실행하지 않았습니다.\n'
  exit 0
fi
run_args=()
if "$no_rviz"; then run_args+=(--no-rviz); fi
exec bash scripts/run.sh example "${run_args[@]}"
