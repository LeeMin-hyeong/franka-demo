#!/usr/bin/env bash
# Ubuntu 22.04용 의존성 설치와 공식 소스 다운로드. 실제 로봇에 연결하지 않는다.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"

sources_only=false
case "${1:-}" in
  '') ;;
  --sources-only) sources_only=true ;;
  -h|--help)
    printf '사용법: ./scripts/setup.sh [--sources-only]\n'
    printf '기본 실행은 apt와 rosdep으로 시스템 패키지를 설치합니다.\n'
    printf '%s\n' '--sources-only는 이미 설치한 git/vcs를 사용해 공식 소스만 가져옵니다.'
    exit 0 ;;
  *) demo_die "알 수 없는 옵션: $1" ;;
esac
[[ $# -le 1 ]] || demo_die '옵션이 너무 많습니다.'
demo_require_ubuntu
demo_source_ros

if ! "$sources_only"; then
  printf 'apt/rosdep이 시스템 패키지를 추가합니다. 기존 워크스페이스는 삭제하지 않습니다.\n'
  sudo apt-get update
  sudo apt-get install -y \
    build-essential cmake git libeigen3-dev libpoco-dev libfmt-dev \
    libtinyxml2-dev libconsole-bridge-dev \
    python3-colcon-common-extensions python3-vcstool python3-rosdep \
    ros-humble-moveit ros-humble-geometric-shapes ros-humble-random-numbers \
    ros-humble-rclcpp ros-humble-rclcpp-action ros-humble-rclpy ros-humble-moveit-msgs \
    ros-humble-ament-cmake-python \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-pinocchio ros-humble-xacro ros-humble-rviz2 \
    ros-humble-rviz-common ros-humble-rviz-rendering ros-humble-rviz-default-plugins \
    ros-humble-robot-state-publisher ros-humble-joint-state-publisher
fi
command -v git >/dev/null || demo_die 'git이 없습니다.'
command -v vcs >/dev/null || demo_die 'vcs가 없습니다. python3-vcstool을 설치하세요.'
mkdir -p "$DEMO_VENDOR"

# --skip-existing은 이미 있는 소스를 checkout/reset/pull하지 않는다.
vcs import "$DEMO_VENDOR" --recursive --skip-existing < "$DEMO_ROOT/scripts/upstream.repos"
[[ -f "$DEMO_VENDOR/franka_ros2/dependency.repos" ]] ||
  demo_die '공식 dependency.repos가 없습니다.'
[[ "$(git -C "$DEMO_VENDOR/franka_ros2" rev-parse HEAD)" == "$DEMO_FRANKA_ROS2_REVISION" ]] ||
  demo_die '기존 franka_ros2의 커밋이 다릅니다. 자동으로 변경하지 않습니다.'
vcs import "$DEMO_VENDOR" --recursive --skip-existing < "$DEMO_VENDOR/franka_ros2/dependency.repos"
demo_check_sources
vcs export "$DEMO_VENDOR" --exact > "$DEMO_WS/sources.lock.repos"

if ! "$sources_only"; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update --rosdistro humble
  dependency_paths=("$DEMO_VENDOR/libfranka" "$DEMO_VENDOR/franka_description")
  for package in "${DEMO_PACKAGES[@]}"; do
    [[ -d "$DEMO_VENDOR/franka_ros2/$package" ]] &&
      dependency_paths+=("$DEMO_VENDOR/franka_ros2/$package")
  done
  dependency_paths+=("$DEMO_WS/src/fr3_moveit_demo" "$DEMO_WS/src/fr3_offline_preview")
  # bringup이 선언하는 모바일/예제 컨트롤러는 팔의 fake launch에서 사용하지 않는다.
  # 테스트/문서 의존성은 설치하지 않아 Gazebo 등 범위 밖 도구의 설치를 피한다.
  rosdep install --from-paths "${dependency_paths[@]}" --ignore-src \
    --rosdistro humble -y -t build -t buildtool -t build_export \
    -t buildtool_export -t exec \
    --skip-keys 'franka_example_controllers franka_mobile franka_mobile_sensors franka_gazebo_bringup'
fi
printf '공식 소스 준비 완료: franka_description 2.8.1, libfranka 0.20.4\n'
printf '다운로드 경로: %s\n' "$DEMO_VENDOR"
printf '다음 명령: ./scripts/build.sh\n'
