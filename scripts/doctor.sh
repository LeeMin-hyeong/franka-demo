#!/usr/bin/env bash
# 시스템/설치만 확인한다. FCI 접속이나 이동 목표를 보내지 않는다.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
real=false
camera=false
while (($#)); do
case "$1" in
  --real) real=true ;;
  --camera) camera=true ;;
  -h|--help)
    printf '사용법: ./scripts/doctor.sh [--real] [--camera]\n'
    printf '설치/빌드 확인. --real은 native Ubuntu의 RT 커널과 현재 셸 권한도 확인합니다.\n'
    printf '%s\n' '--camera는 D435i SDK/드라이버 설치와 launch 옵션을 확인하며 장치는 연결하지 않습니다.'
    exit 0 ;;
  *) demo_die '알 수 없는 doctor 옵션입니다.' ;;
esac
shift
done
demo_require_ubuntu
demo_source_workspace
demo_check_sources
for package in fr3_moveit_demo fr3_offline_preview franka_fr3_moveit_config franka_hardware franka_gripper franka_msgs std_srvs rviz2; do
  ros2 pkg prefix "$package" >/dev/null || demo_die "패키지가 없습니다: $package. setup/build를 확인하세요."
done
python3 "$DEMO_ROOT/tests/check_installed_executables.py"
python3 -c 'from fr3_moveit_demo import Robot; from fr3_moveit_demo.pick_place import PickAndReturn; from franka_msgs.action import Move, Grasp; from std_srvs.srv import Trigger; Move.Goal(width=0.04, speed=0.01); Grasp.Goal(width=0.03, speed=0.01, force=20.0); Trigger.Request(); print("Python Robot / pick task / real message types import: OK")'
python3 - <<'PY'
# import만 통과해도 NumPy/cv_bridge ABI가 맞지 않을 수 있어 영상 왕복도 확인한다.
import cv2
import numpy as np
import message_filters
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from fr3_moveit_demo.calibration import validate_transform
from fr3_moveit_demo.handeye import checkerboard_points
image = np.zeros((2, 2, 3), dtype=np.uint8)
bridge = CvBridge()
np.testing.assert_array_equal(bridge.imgmsg_to_cv2(bridge.cv2_to_imgmsg(image, 'bgr8'), 'bgr8'), image)
validate_transform(np.eye(4))
assert checkerboard_points((10, 7), 25).shape == (70, 3)
print(f'Calibration Python / cv_bridge image conversion: OK (OpenCV {cv2.__version__}, NumPy {np.__version__})')
PY
for script in "$DEMO_ROOT/calibration/record.py" "$DEMO_ROOT/calibration/solve.py" \
  "$DEMO_WS/src/fr3_moveit_demo/examples/click_camera_point.py" \
  "$DEMO_WS/src/fr3_moveit_demo/examples/pick_and_return.py"; do
  python3 "$script" --help >/dev/null
done
[[ -x "$DEMO_ROOT/standalone/libfranka_demo/build/offline_demo" ]] ||
  demo_die 'C++ 오프라인 실행 파일이 없습니다. build.sh를 실행하세요.'
if "$camera"; then
  source "$DEMO_ROOT/scripts/camera_setup.sh"
  demo_camera_rules_present || demo_die 'RealSense USB 권한 규칙이 없습니다. setup.sh --with-camera를 실행하세요.'
  # SDK는 일반 CMake 라이브러리이므로 ros2 pkg 목록에 등록되지 않는다.
  # 설치한 apt 패키지를 확인하고 실제 노드의 공유 라이브러리 누락도 검사한다.
  dpkg-query -W -f='${Status}' ros-humble-librealsense2 2>/dev/null | grep -q 'install ok installed' ||
    demo_die 'D435i SDK apt 패키지가 없습니다. setup.sh --with-camera를 실행하세요.'
  for package in realsense2_camera realsense2_description; do
    ros2 pkg prefix "$package" >/dev/null || demo_die "카메라 패키지가 없습니다: $package. setup.sh --with-camera를 실행하세요."
  done
  camera_executable="$(python3 - <<'PY'
from ros2run.api import get_executable_path
path = get_executable_path(package_name='realsense2_camera', executable_name='realsense2_camera_node')
if path is None:
    raise SystemExit('RealSense 실행 파일이 없습니다. setup.sh --with-camera를 확인하세요.')
print(path)
PY
  )"
  camera_libraries="$(ldd "$camera_executable")"
  if grep -q 'not found' <<< "$camera_libraries"; then
    printf '%s\n' "$camera_libraries" >&2
    demo_die 'D435i 노드의 공유 라이브러리가 누락됐습니다. setup.sh --with-camera를 확인하세요.'
  fi
  ros2 launch realsense2_camera rs_launch.py --show-args >/dev/null
  printf 'D435i SDK/드라이버/launch 설치: OK (장치 연결/영상 수신은 미검사)\n'
fi
printf 'Ubuntu 22.04 / ROS Humble / 공식 소스 / 데모 설치: OK\n'
if "$real"; then
  demo_require_real_pc
  printf '실물 PC의 RT 커널/현재 셸 권한: OK\n'
  printf '로봇 IP, System Image >=5.9.0, FCI 활성화, 직결 네트워크는 docs/real-robot.md에서 확인하세요.\n'
  printf '이 점검은 로봇 연결이나 실제 동작 검증을 수행하지 않았습니다.\n'
elif [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  printf 'GUI 없음: ./scripts/run.sh example --no-rviz로 가상 제어를 확인할 수 있습니다.\n'
fi
