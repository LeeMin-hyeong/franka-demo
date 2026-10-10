#!/usr/bin/env bash
# Python API/예제와 C++17 학습 프로그램, 필요한 공식 ROS 패키지를 설치/빌드한다.
# 이 스크립트는 제어를 실행하지 않는다.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"

offline_only=false
case "${1:-}" in
  '') ;;
  --offline-only) offline_only=true ;;
  -h|--help)
    printf '사용법: ./scripts/build.sh [--offline-only]\n'
    printf '기본 실행: Python API/예제 + ROS/C++ 패키지 + 궤적 생성기 + 실제 API 컴파일 검증\n'
    printf '%s\n' '--offline-only: ROS/libfranka 없이 궤적 생성기와 CTest만 빌드'
    printf 'FR3_BUILD_JOBS=2가 기본값이며 메모리가 부족하면 1로 줄이세요.\n'
    exit 0 ;;
  *) demo_die "알 수 없는 옵션: $1" ;;
esac
[[ $# -le 1 ]] || demo_die '옵션이 너무 많습니다.'
demo_require_ubuntu
build_jobs="${FR3_BUILD_JOBS:-2}"
[[ "$build_jobs" =~ ^[1-9][0-9]*$ ]] || demo_die 'FR3_BUILD_JOBS는 양의 정수여야 합니다.'
command -v cmake >/dev/null || demo_die 'cmake가 없습니다.'

if ! "$offline_only"; then
  demo_source_ros
  demo_check_sources
  command -v colcon >/dev/null || demo_die 'colcon이 없습니다. setup.sh를 실행하세요.'
  # 공식 저장소에 포함된 모바일/Gazebo 코드는 데모 빌드 대상에서 제외한다.
  # --packages-select로 필요한 패키지와 그 내부 의존성만 명시한다.
  build_paths=()
  for package in "${DEMO_PACKAGES[@]}"; do
    if [[ -d "$DEMO_VENDOR/franka_ros2/$package" ]]; then
      build_paths+=("$DEMO_VENDOR/franka_ros2/$package")
    elif [[ -d "$DEMO_VENDOR/$package" ]]; then
      build_paths+=("$DEMO_VENDOR/$package")
    else
      build_paths+=("$DEMO_WS/src/$package")
    fi
  done
  cd "$DEMO_WS"
  CMAKE_BUILD_PARALLEL_LEVEL="$build_jobs" colcon build --symlink-install --executor sequential \
    --paths "${build_paths[@]}" \
    --packages-select "${DEMO_PACKAGES[@]}" \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
  demo_source_workspace
fi

standalone_args=(-DCMAKE_BUILD_TYPE=Release)
if "$offline_only"; then
  standalone_args+=(-DCOMPILE_REAL_API_EXAMPLE=OFF)
else
  standalone_args+=(-DCOMPILE_REAL_API_EXAMPLE=ON "-DFranka_DIR=$DEMO_WS/install/libfranka/lib/cmake/Franka")
fi
cmake -S "$DEMO_ROOT/standalone/libfranka_demo" \
  -B "$DEMO_ROOT/standalone/libfranka_demo/build" "${standalone_args[@]}"
cmake --build "$DEMO_ROOT/standalone/libfranka_demo/build" --parallel "$build_jobs"
ctest --test-dir "$DEMO_ROOT/standalone/libfranka_demo/build" --output-on-failure
printf '빌드 완료. 실제 Robot 연결은 수행하지 않았습니다.\n'
