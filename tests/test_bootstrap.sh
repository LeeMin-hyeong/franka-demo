#!/usr/bin/env bash
# ROS 최초 설치의 분기와 실패 처리를 검사한다.
# apt, dpkg, 다운로드를 shell 함수로 바꾸므로 시스템이나 네트워크를 변경하지 않는다.
# 실제 Ubuntu 신규 설치 검증을 대신하지 않으며 Bash/Python3가 있는 환경에서 실행한다.
set -euo pipefail

test_script="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/$(basename -- "${BASH_SOURCE[0]}")"
repo_root="$(cd -- "$(dirname -- "$test_script")/.." && pwd)"

if [[ "${1:-}" == --case ]]; then
  scenario="$2"
  event_file="$3"
  source "$repo_root/scripts/common.sh"
  source "$repo_root/scripts/bootstrap_ros.sh"
  has_ros=false
  [[ "$scenario" != existing ]] || has_ros=true

  record() { printf '%s\n' "$*" >> "$event_file"; }
  demo_has_ros() { "$has_ros"; }
  demo_ros_repository_present() {
    case "${1:-ros2}" in
      ros2) [[ "$scenario" == manual-source ]] ;;
      ros2-testing) [[ "$scenario" == manual-testing ]] ;;
      *) return 1 ;;
    esac
  }
  locale() {
    [[ "$1" == -a ]] || return 1
    if [[ "$scenario" != fresh && "$scenario" != rerun ]]; then
      printf 'en_US.utf8\n'
    fi
  }
  dpkg-query() {
    record "QUERY $*"
    case "${!#}" in
      ros2-testing-apt-source)
        [[ "$scenario" == testing ]] || return 1 ;;
      ros2-apt-source)
        [[ "$scenario" == installed-source ]] || return 1 ;;
      *) return 1 ;;
    esac
    printf 'install ok installed'
  }
  demo_as_root() {
    # 허용된 설치 명령만 기록한다. 실제 명령은 어떤 경우에도 호출하지 않는다.
    record "ROOT $*"
    case "$1" in
      apt-get)
        if [[ "$scenario" == failed-update && "$2" == update ]]; then return 42; fi
        if [[ "$scenario" == failed-upgrade && "$2" == upgrade ]]; then return 42; fi
        if [[ "$*" == *ros-humble-desktop* ]]; then
          [[ "$scenario" != failed-install ]] || return 42
          [[ "$scenario" == missing-result ]] || has_ros=true
        fi ;;
      dpkg) [[ "$scenario" != failed-dpkg ]] || return 42 ;;
      locale-gen|add-apt-repository) ;;
      *) printf '테스트에서 허용하지 않은 설치 명령: %s\n' "$1" >&2; return 99 ;;
    esac
  }
  mktemp() {
    # bootstrap이 만드는 임시 deb도 테스트 소유 폴더 안에서만 생성한다.
    [[ "$1" == /tmp/fr3-ros2-apt-source-XXXXXX.deb ]] || return 99
    command mktemp "$(dirname -- "$event_file")/fr3-ros2-apt-source-XXXXXX.deb"
  }
  curl() {
    record "CURL $*"
    if [[ "${!#}" == https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest ]]; then
      [[ "$scenario" != failed-api ]] || return 22
      if [[ "$scenario" == bad-release ]]; then
        printf '{"tag_name":"invalid"}\n'
      else
        printf '{"tag_name":"1.2.0"}\n'
      fi
      return
    fi
    [[ "$scenario" != failed-download ]] || return 22
    local output_file=''
    while (($#)); do
      if [[ "$1" == -o ]]; then output_file="$2"; break; fi
      shift
    done
    [[ "$output_file" == "$(dirname -- "$event_file")"/fr3-ros2-apt-source-*.deb ]] || return 99
    printf 'fake package for command-stub test\n' > "$output_file"
  }

  # 조건식 안에서 함수를 호출하면 Bash errexit가 함수 전체에 적용되지 않을 수 있다.
  # 별도 Bash의 일반 명령으로 호출하여 실제 setup과 같은 실패 종료를 검사한다.
  demo_bootstrap_ros
  if [[ "$scenario" == rerun ]]; then demo_bootstrap_ros; fi
  exit 0
fi

[[ $# == 0 ]] || { printf '사용법: bash tests/test_bootstrap.sh\n' >&2; exit 1; }
test_dir="$(mktemp -d /tmp/fr3-bootstrap-tests-XXXXXX)"
cleanup() {
  # 이 테스트가 만든 평면 폴더만 정리한다. 기존 워크스페이스는 읽거나 삭제하지 않는다.
  [[ "$test_dir" == /tmp/fr3-bootstrap-tests-* && -d "$test_dir" ]] || return
  rm -f -- "$test_dir"/*
  rmdir -- "$test_dir"
}
trap cleanup EXIT

fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }
has_event() { grep -Fq -- "$2" "$test_dir/$1.events"; }
expect_event() { has_event "$1" "$2" || fail "$1: 필요한 명령 없음: $2"; }
expect_no_event() { if has_event "$1" "$2"; then fail "$1: 실행되면 안 되는 명령: $2"; fi; }
expect_before() {
  local earlier later
  earlier="$(grep -Fn -- "$2" "$test_dir/$1.events" | head -n 1 | cut -d: -f1)"
  later="$(grep -Fn -- "$3" "$test_dir/$1.events" | head -n 1 | cut -d: -f1)"
  [[ -n "$earlier" && -n "$later" && "$earlier" -lt "$later" ]] ||
    fail "$1: 명령 순서 오류: $2 -> $3"
}

scenarios=(existing fresh rerun installed-source manual-source testing manual-testing
  failed-update failed-upgrade failed-install failed-api bad-release
  failed-download failed-dpkg missing-result)
for scenario in "${scenarios[@]}"; do
  : > "$test_dir/$scenario.events"
  status=0
  bash "$test_script" --case "$scenario" "$test_dir/$scenario.events" \
    > "$test_dir/$scenario.out" 2>&1 || status=$?
  case "$scenario" in
    existing|fresh|rerun|installed-source|manual-source) expected=0 ;;
    failed-update|failed-upgrade|failed-install) expected=42 ;;
    *) expected=1 ;;
  esac
  if [[ "$status" != "$expected" ]]; then
    cat "$test_dir/$scenario.out" >&2
    fail "$scenario: 종료값 $status, 예상 $expected"
  fi
  case "$scenario" in
    existing)
      [[ ! -s "$test_dir/$scenario.events" ]] || fail '기존 ROS에서 설치 명령 실행' ;;
    fresh|rerun)
      expect_event "$scenario" 'ROOT locale-gen en_US en_US.UTF-8'
      expect_event "$scenario" 'ros2-apt-source_1.2.0.jammy_all.deb'
      expect_before "$scenario" 'ROOT add-apt-repository -y universe' 'ROOT dpkg -i'
      expect_before "$scenario" 'ROOT dpkg -i' 'ROOT apt-get upgrade -y --no-remove'
      expect_before "$scenario" 'ROOT apt-get upgrade -y --no-remove' 'ros-humble-desktop ros-dev-tools'
      # 초기 설치와 ROS 설치에 패키지 제거 방지 옵션이 모두 있어야 한다.
      if grep '^ROOT apt-get \(install\|upgrade\)' "$test_dir/$scenario.events" |
         grep -qv -- '--no-remove'; then fail "$scenario: --no-remove 누락"; fi
      [[ "$(grep -c '^ROOT apt-get install .*ros-humble-desktop' "$test_dir/$scenario.events")" == 1 ]] ||
        fail "$scenario: ROS 중복 설치" ;;
    installed-source|manual-source)
      expect_no_event "$scenario" 'CURL '
      expect_no_event "$scenario" 'ROOT dpkg '
      expect_no_event "$scenario" 'ROOT locale-gen '
      expect_event "$scenario" 'ros-humble-desktop ros-dev-tools' ;;
    testing|manual-testing)
      expect_no_event "$scenario" 'CURL '
      expect_no_event "$scenario" 'ROOT dpkg '
      expect_no_event "$scenario" 'ROOT apt-get upgrade' ;;
    failed-update)
      [[ "$(wc -l < "$test_dir/$scenario.events")" == 1 ]] || fail 'apt update 실패 뒤 실행 지속' ;;
    failed-upgrade)
      expect_no_event "$scenario" 'ros-humble-desktop ros-dev-tools' ;;
    failed-api|bad-release|failed-download|failed-dpkg)
      expect_no_event "$scenario" 'ROOT apt-get upgrade'
      expect_no_event "$scenario" 'ros-humble-desktop ros-dev-tools' ;;
    missing-result)
      expect_event "$scenario" 'ros-humble-desktop ros-dev-tools' ;;
  esac
  printf 'PASS: %s\n' "$scenario"
done
if compgen -G "$test_dir/fr3-ros2-apt-source-*.deb" > /dev/null; then
  fail '성공/실패 뒤 임시 deb가 남았습니다.'
fi
printf 'Bootstrap command-stub tests: %s/%s PASS (실제 apt/다운로드 없음)\n' "${#scenarios[@]}" "${#scenarios[@]}"
