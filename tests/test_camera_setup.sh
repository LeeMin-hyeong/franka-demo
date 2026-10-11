#!/usr/bin/env bash
# 카메라 설치 함수의 기존 규칙 보존/실패 중단을 검사한다. apt와 시스템 파일은 변경하지 않는다.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
fixture="$(mktemp -d /tmp/fr3-camera-setup-test-XXXXXX)"
cleanup() {
  [[ "$fixture" == /tmp/fr3-camera-setup-test-* && -d "$fixture" ]] || return
  rm -f -- "$fixture"/*
  rmdir -- "$fixture"
}
trap cleanup EXIT
if [[ "${1:-}" == --case ]]; then
  scenario="$2"
  events="$3"
  source "$repo_root/scripts/common.sh"
  source "$repo_root/scripts/camera_setup.sh"
  demo_camera_rules_present() { [[ "$scenario" == existing ]]; }
  demo_as_root() {
    printf '%s\n' "$*" >> "$events"
    case "$1" in
      apt-get) [[ "$scenario" != failed-apt ]] || return 42 ;;
      install) [[ "$scenario" != failed-install ]] || return 42 ;;
      udevadm) ;;
      *) return 99 ;;
    esac
  }
  mktemp() { command mktemp "$(dirname -- "$events")/rules-XXXXXX"; }
  curl() {
    printf 'DOWNLOAD\n' >> "$events"
    [[ "$scenario" != failed-download ]] || return 22
    local previous='' value
    for value in "$@"; do
      if [[ "$previous" == -o ]]; then printf 'mock rule\n' > "$value"; fi
      previous="$value"
    done
  }
  demo_setup_camera
  exit 0
fi
for scenario in existing fresh failed-apt failed-download failed-install; do
  : > "$fixture/$scenario.events"
  status=0
  bash "$0" --case "$scenario" "$fixture/$scenario.events" > "$fixture/$scenario.out" 2>&1 || status=$?
  case "$scenario" in
    existing|fresh) expected=0 ;;
    failed-apt) expected=42 ;;
    *) expected=1 ;;
  esac
  [[ "$status" == "$expected" ]] || { cat "$fixture/$scenario.out"; exit 1; }
  case "$scenario" in
    existing|failed-apt)
      if grep -q 'DOWNLOAD\|^install ' "$fixture/$scenario.events"; then exit 1; fi ;;
    fresh)
      grep -q '^install -D -m 644' "$fixture/$scenario.events" ;;
    failed-download)
      if grep -q '^install ' "$fixture/$scenario.events"; then exit 1; fi ;;
  esac
  if compgen -G "$fixture/rules-*" >/dev/null; then printf '임시 규칙 파일이 남았습니다.\n'; exit 1; fi
  printf 'PASS: %s\n' "$scenario"
done
printf 'Camera setup: 5/5 PASS (실제 apt/USB 변경 없음)\n'
