#!/usr/bin/env bash
# setup.sh/doctor.sh에서 읽는 D435i 설치 함수. 장치를 열거나 펌웨어를 변경하지 않는다.

demo_camera_rules_present() {
  [[ -s /etc/udev/rules.d/99-realsense-libusb.rules ||
     -s /lib/udev/rules.d/99-realsense-libusb.rules ]]
}

demo_setup_camera() {
  # 공식 ROS apt SDK/Wrapper 조합을 사용한다. 별도 SDK 소스 설치와 섞지 않는다.
  demo_as_root apt-get install -y --no-remove \
    ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-description udev
  if demo_camera_rules_present; then
    printf '기존 RealSense USB 권한 규칙을 보존합니다.\n'
  else
    # ROS SDK deb에는 USB 권한 규칙이 없어 일반 사용자 접근용 공식 규칙을 따로 준비한다.
    local rules_file
    rules_file="$(mktemp /tmp/fr3-realsense-rules-XXXXXX)"
    if ! curl -fsSL --retry 3 --connect-timeout 20 -o "$rules_file" \
      https://raw.githubusercontent.com/realsenseai/librealsense/v2.58.4/config/99-realsense-libusb.rules; then
      rm -f -- "$rules_file"
      demo_die 'RealSense USB 규칙 다운로드 실패. 네트워크를 확인하세요.'
    fi
    if ! demo_as_root install -D -m 644 "$rules_file" /etc/udev/rules.d/99-realsense-libusb.rules; then
      rm -f -- "$rules_file"
      demo_die 'RealSense USB 규칙 설치 실패. 관리자 권한을 확인하세요.'
    fi
    rm -f -- "$rules_file"
  fi
  # 다른 USB 장치를 재설정하는 trigger는 실행하지 않는다. 카메라 재연결 시 적용된다.
  if [[ -S /run/udev/control ]]; then
    demo_as_root udevadm control --reload-rules
  else
    printf '현재 udev 데몬 없음: 실제 Ubuntu 부팅 후 규칙이 적용됩니다.\n'
  fi
  printf 'D435i USB 케이블을 다시 연결한 뒤 일반 사용자로 카메라를 실행하세요.\n'
}
