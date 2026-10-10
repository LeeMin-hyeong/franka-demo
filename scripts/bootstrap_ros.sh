#!/usr/bin/env bash
# setup.sh에서 읽는 ROS 설치 함수. ROS가 없는 Ubuntu 22.04의 최초 설치를 담당한다.
# 공식 ros2-apt-source 패키지로 저장소/키를 관리하며 shellrc나 기존 ROS 소스를 덮어쓰지 않는다.

demo_ros_repository_present() {
  # 기존 수동 설정은 그대로 사용한다. Signed-By가 다른 중복 저장소를 추가하지 않는다.
  local repository="${1:-ros2}" source_file pattern
  pattern="https?://packages[.]ros[.]org/${repository}(/|[[:space:]])"
  if [[ -f /etc/apt/sources.list ]]; then
    awk -v pattern="$pattern" '{sub(/#.*/, "")} $1 == "deb" && $0 ~ pattern {found=1} END {exit !found}' /etc/apt/sources.list && return 0
  fi
  for source_file in /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; do
    [[ -f "$source_file" ]] || continue
    if [[ "$source_file" == *.list ]]; then
      awk -v pattern="$pattern" '{sub(/#.*/, "")} $1 == "deb" && $0 ~ pattern {found=1} END {exit !found}' "$source_file" && return 0
    else
      # deb822 문단마다 Enabled: no와 주석을 제외한 URI를 확인한다.
      awk -v RS='' -v pattern="$pattern" '{record=$0; gsub(/#[^\n]*/, "", record); binary=0;
        count=split(record, lines, "\n"); for (i=1; i<=count; i++) if (tolower(lines[i]) ~ /^[[:space:]]*types:/) {
          fields=split(lines[i], types, /[[:space:]]+/); for (j=1; j<=fields; j++) if (types[j]=="deb") binary=1}
        if (binary && tolower(record) !~ /(^|\n)enabled:[[:space:]]*no([[:space:]]|$)/ && record ~ pattern) found=1}
        END {exit !found}' "$source_file" && return 0
    fi
  done
  return 1
}

demo_bootstrap_ros() {
  if demo_has_ros; then
    printf '기존 ROS2 Humble 설치를 사용합니다.\n'
    return
  fi
  printf 'ROS가 없는 Ubuntu 22.04: 공식 ROS2 Humble부터 설치합니다.\n'
  printf '시스템 패키지 업데이트, UTF-8 locale, Universe와 ROS apt 설정을 준비합니다.\n'
  demo_as_root apt-get update
  demo_as_root apt-get install -y --no-remove \
    locales software-properties-common curl ca-certificates python3 git
  if ! locale -a | grep -qi '^en_US\.utf8$'; then
    demo_as_root locale-gen en_US en_US.UTF-8
  fi
  # 이번 설치 프로세스에만 적용한다. 사용자의 기본 로케일과 .bashrc는 보존한다.
  export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
  demo_as_root add-apt-repository -y universe
  if { dpkg-query -W -f='${Status}' ros2-testing-apt-source 2>/dev/null | grep -q 'install ok installed'; } ||
     demo_ros_repository_present ros2-testing; then
    demo_die 'ROS testing 저장소가 있습니다. 안정 Humble 저장소와의 혼합을 먼저 확인하세요.'
  fi
  if dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
    printf '기존 ros2-apt-source 저장소 설정을 사용합니다.\n'
  elif demo_ros_repository_present; then
    printf '기존 수동 ROS apt 설정을 보존하고 사용합니다. 저장소/키 오류는 apt 출력에서 확인하세요.\n'
  else
    local release_version package_file
    release_version="$(curl -fsSL --retry 3 --connect-timeout 20 \
      https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | \
      python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
    [[ "$release_version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || demo_die 'ROS apt-source 공식 release 버전을 확인하지 못했습니다.'
    package_file="$(mktemp /tmp/fr3-ros2-apt-source-XXXXXX.deb)"
    if ! curl -fL --retry 3 --connect-timeout 20 -o "$package_file" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${release_version}/ros2-apt-source_${release_version}.jammy_all.deb"; then
      rm -f -- "$package_file"
      demo_die 'ROS apt-source 다운로드 실패. 네트워크를 확인하고 setup.sh를 다시 실행하세요.'
    fi
    if ! demo_as_root dpkg -i "$package_file"; then
      rm -f -- "$package_file"
      demo_die 'ROS apt-source 설치 실패. 위 dpkg 오류를 확인하세요.'
    fi
    rm -f -- "$package_file"
  fi
  demo_as_root apt-get update
  # 공식 Humble 문서는 신규 Jammy의 systemd/udev 갱신을 ROS 설치보다 먼저 요구한다.
  # --no-remove는 의존성 해결 중 기존 패키지를 제거하려는 작업을 중단시킨다.
  demo_as_root apt-get upgrade -y --no-remove
  demo_as_root apt-get install -y --no-remove ros-humble-desktop ros-dev-tools
  demo_has_ros || demo_die 'ROS 설치 뒤 setup.bash가 없습니다. apt 오류를 확인하세요.'
  printf 'ROS2 Humble 최초 설치 완료.\n'
}
