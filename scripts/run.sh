#!/usr/bin/env bash
# Fake Hardware 또는 오프라인 ROS 시각화만 실행한다. robot_ip 옵션은 제공하지 않는다.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"

usage() {
  cat <<'USAGE'
사용법:
  ./scripts/run.sh
  ./scripts/run.sh moveit [옵션]
  ./scripts/run.sh moveit-command [옵션]
  ./scripts/run.sh server [--no-rviz]
  ./scripts/run.sh example [--duration SEC] [--no-rviz]
  ./scripts/run.sh example-command [--duration SEC]
  ./scripts/run.sh libfranka-preview [옵션]

인자가 없으면 Python MoveIt 데모를 시작한다.
moveit            Fake Hardware/RViz를 시작하고 팔 또는 가상 그리퍼를 이동한다.
moveit-command    이미 실행 중인 Fake Hardware에 이동 명령만 보낸다.
server            Fake Hardware/RViz만 준비하며 자동으로 움직이지 않는다.
example           Fake Hardware/RViz와 짧은 Python API 예제를 실행한다.
example-command   이미 실행 중인 서버에 Python API 예제만 실행한다.
libfranka-preview FCI 없이 C++ 궤적을 /joint_states로 재생한다.

옵션:
  --language LANG MoveIt 클라이언트 언어: python 또는 cpp (기본값: python)
                  C++ 버전은 --language cpp로 선택한다. example은 Python 전용이다.
  --gripper-width METERS
                  가상 그리퍼의 두 손가락 사이 전체 폭 [m], 0~0.08
                  MoveIt 전용. --joint/--delta/--home과 함께 사용할 수 없다.
  --joint N       1부터 7까지의 관절 번호 (기본값: 4)
  --delta RAD     현재값 대비 이동량 [rad] (기본값: 0.1)
  --duration SEC  이동 시간 [s] (기본값: 3.0)
                  MoveIt에서는 실행 시간을 이 값 이상으로 늦출 수 있다.
  --rate HZ       Offline Preview 발행 주기 [Hz] (기본값: 30.0)
  --home          MoveIt의 공식 ready 자세로 이동한다.
  --no-rviz       RViz 창을 열지 않는다 (headless 검증용).
  -h, --help      도움말

MoveIt2와 Offline Preview는 같은 ROS_DOMAIN_ID에서 동시에 실행하지 마세요.
USAGE
}

mode=moveit
case "${1:-}" in
  '') ;;
  -h|--help) usage; exit 0 ;;
  moveit|moveit-command|server|example|example-command|libfranka-preview) mode="$1"; shift ;;
  -*) ;;  # 실행 방식을 생략하고 옵션만 입력해도 기본 moveit을 사용한다.
  *) demo_die "알 수 없는 실행 방식: $1. --help를 참고하세요." ;;
esac
joint=4
delta=0.1
duration=3.0
rate=30.0
home=false
rviz=true
rate_given=false
language=python
language_given=false
gripper_width=-1.0
gripper_width_given=false
joint_given=false
delta_given=false
home_given=false
duration_given=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --joint|--delta|--duration|--rate|--language|--gripper-width)
      [[ $# -ge 2 ]] || demo_die "$1 뒤에 값이 필요합니다."
      case "$1" in
        --joint) joint="$2"; joint_given=true ;;
        --delta) delta="$2"; delta_given=true ;;
        --duration) duration="$2"; duration_given=true ;;
        --rate) rate="$2"; rate_given=true ;;
        --language) language="$2"; language_given=true ;;
        --gripper-width) gripper_width="$2"; gripper_width_given=true ;;
      esac
      shift 2 ;;
    --home) home=true; home_given=true; shift ;;
    --no-rviz) rviz=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) demo_die "알 수 없는 옵션: $1" ;;
  esac
done
case "$language" in
  cpp) moveit_executable=moveit_demo; language_label='C++' ;;
  python) moveit_executable=moveit_demo.py; language_label=Python ;;
  *) demo_die '--language는 cpp 또는 python이어야 합니다.' ;;
esac
case "$mode" in
  server)
    if "$language_given" || "$joint_given" || "$delta_given" || "$home_given" ||
       "$gripper_width_given" || "$duration_given" || "$rate_given"; then
      demo_die 'server는 자동 이동 없이 스택만 준비합니다. --no-rviz만 사용할 수 있습니다.'
    fi ;;
  example|example-command)
    [[ "$language" == python ]] || demo_die 'example과 example-command는 Python 전용입니다.'
    if "$joint_given" || "$delta_given" || "$home_given" || "$gripper_width_given" || "$rate_given"; then
      demo_die 'example은 정해진 API 순서를 실행합니다. joint/delta/home/gripper-width/rate 옵션을 사용할 수 없습니다.'
    fi
    if [[ "$mode" == example-command ]] && ! "$rviz"; then
      demo_die '--no-rviz는 서버를 시작하는 실행 방식에서만 사용할 수 있습니다.'
    fi ;;
esac
if [[ "$mode" == libfranka-preview ]] && "$language_given"; then
  demo_die 'libfranka-preview는 C++ 프리뷰이며 --language를 사용할 수 없습니다.'
fi
if "$gripper_width_given"; then
  [[ "$mode" != libfranka-preview ]] ||
    demo_die '--gripper-width는 moveit 또는 moveit-command에서만 사용할 수 있습니다.'
  if "$joint_given" || "$delta_given" || "$home_given"; then
    demo_die '--gripper-width는 --joint/--delta/--home과 함께 사용할 수 없습니다.'
  fi
  [[ "$gripper_width" =~ ^[-+]?[0-9]+([.][0-9]+)?$ ]] ||
    demo_die '--gripper-width에는 유한한 소수 또는 정수를 입력하세요.'
  awk -v value="$gripper_width" 'BEGIN { exit !(value >= 0 && value <= 0.08) }' ||
    demo_die '--gripper-width는 전체 손가락 간 폭으로 0부터 0.08 m까지여야 합니다.'
fi
[[ "$joint" =~ ^[1-7]$ ]] || demo_die '--joint는 1부터 7까지의 정수여야 합니다.'
for value in "$delta" "$duration" "$rate"; do
  [[ "$value" =~ ^[-+]?[0-9]+([.][0-9]+)?$ ]] ||
    demo_die 'delta/duration/rate에는 소수 또는 정수를 입력하세요.'
done
awk -v value="$duration" 'BEGIN { exit !(value > 0) }' ||
  demo_die '--duration은 0보다 커야 합니다.'
awk -v value="$rate" 'BEGIN { exit !(value >= 1 && value <= 1000) }' ||
  demo_die '--rate는 1부터 1000까지여야 합니다.'
if [[ "$mode" != libfranka-preview ]] && "$rate_given"; then
  demo_die '--rate는 libfranka-preview에서만 사용할 수 있습니다.'
fi
if [[ "$mode" == libfranka-preview ]] && "$home"; then
  demo_die 'Offline Preview는 매 실행마다 ready 초기 자세에서 시작합니다. --home은 MoveIt 전용입니다.'
fi
# ROS 파라미터의 YAML 정수/실수 구분 때문에 3을 3.0으로 전달한다.
[[ "$delta" == *.* ]] || delta="$delta.0"
[[ "$duration" == *.* ]] || duration="$duration.0"
[[ "$rate" == *.* ]] || rate="$rate.0"
[[ "$gripper_width" == *.* ]] || gripper_width="$gripper_width.0"
demo_require_ubuntu
demo_source_workspace

run_client_command() {
  if [[ "$mode" == example || "$mode" == example-command ]]; then
    printf 'Python API 예제 basic_sequence.py를 실행합니다. 실제 로봇에 연결하지 않습니다.\n'
    ros2 run fr3_moveit_demo basic_sequence.py --ros-args -p "duration:=$duration"
    return
  fi
  if "$gripper_width_given"; then
    printf '가상 그리퍼 목표 전체 폭: %s m. 실제 그리퍼에 연결하지 않습니다.\n' "$gripper_width"
  fi
  ros2 run fr3_moveit_demo "$moveit_executable" --ros-args \
    -p "joint:=$joint" -p "delta:=$delta" -p "duration:=$duration" \
    -p "home:=$home" -p "gripper_width:=$gripper_width" -p wait_timeout:=30.0
}

if [[ "$mode" == moveit-command || "$mode" == example-command ]]; then
  printf '실행 중인 FR3 Fake Hardware에 %s 클라이언트로 계획/실행을 요청합니다.\n' "$language_label"
  run_client_command
  exit 0
fi

# 환경이 겹치면 서로 다른 상태가 같은 토픽에 섞이므로 실행 전에 확인한다.
if ! nodes="$(ros2 node list --no-daemon)"; then
  demo_die '현재 ROS 그래프를 조회하지 못했습니다. 위의 ROS 환경 오류를 확인하세요.'
fi
if grep -Eq '^/(move_group|fr3_joint_state_preview|controller_manager)$' <<< "$nodes"; then
  demo_die '같은 ROS_DOMAIN_ID에 데모 노드가 이미 있습니다. 기존 데모를 종료하거나 moveit-command/example-command를 사용하세요.'
fi
if [[ "$mode" == libfranka-preview ]]; then
  printf 'Offline Preview (C++): 실제 libfranka FCI 제어를 실행하지 않습니다.\n'
  printf '직접 계산한 관절 궤적을 ROS2 JointState로 발행합니다. Ctrl+C로 종료하세요.\n'
  exec ros2 launch fr3_offline_preview preview.launch.py \
    "joint:=$joint" "delta:=$delta" "duration:=$duration" \
    "rate:=$rate" "rviz:=$rviz"
fi

if [[ "$mode" == server ]]; then
  printf 'FR3 서버: Fake Hardware/RViz만 시작합니다. 자동 이동과 실제 로봇 연결은 없습니다.\n'
else
  printf 'FR3 MoveIt2 (%s): Fake Hardware를 시작합니다. 실제 로봇에 연결하지 않습니다.\n' "$language_label"
fi
# 별도 프로세스 그룹으로 launch를 시작하고 종료 신호는 launch에만 보낸다.
# launch가 자식 노드에 전달하므로 각 노드가 같은 SIGINT를 두 번 받지 않는다.
# 비대화형 Bash의 background 작업이 상속하는 SIGINT 무시 상태도 해제한다.
setsid env --default-signal=INT ros2 launch fr3_moveit_demo fake_moveit.launch.py "rviz:=$rviz" &
launch_pid=$!
cleanup() {
  trap - EXIT INT TERM
  kill -INT "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

printf 'Fake Hardware 컨트롤러 준비를 기다립니다 (최대 90초).\n'
deadline=$((SECONDS + 90))
ready=false
while ((SECONDS < deadline)); do
  kill -0 "$launch_pid" 2>/dev/null || demo_die 'MoveIt launch가 종료되었습니다. 위의 오류 로그를 확인하세요.'
  # 서비스 호출은 CLI 데몬 없이 직접 노드를 사용한다.
  # MoveIt 액션 서버 대기는 선택한 C++/Python 클라이언트가 담당한다.
  controllers="$(timeout 6 ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers '{}' 2>/dev/null || true)"
  if grep -Fq "name='fr3_arm_controller', state='active'" <<< "$controllers" &&
     grep -Fq "name='fr3_hand_controller', state='active'" <<< "$controllers" &&
     grep -Fq "name='joint_state_broadcaster', state='active'" <<< "$controllers"; then
    ready=true
    break
  fi
  sleep 1
done
"$ready" || demo_die '90초 안에 컨트롤러가 준비되지 않았습니다.'
if "$rviz"; then
  printf 'RViz 모델을 읽고 초기 자세를 확인하도록 10초 기다립니다.\n'
  sleep 10
fi
if [[ "$mode" == server ]]; then
  printf '서버 준비 완료. 자동 이동 없이 현재 자세를 유지합니다. Ctrl+C로 종료하세요.\n'
  printf '다른 터미널에서 ./scripts/run.sh moveit-command 또는 ./scripts/run.sh example-command를 실행하세요.\n'
else
  run_client_command
  printf '이동 완료. RViz와 Fake Hardware는 계속 실행됩니다. Ctrl+C로 종료하세요.\n'
  if [[ "$mode" == example ]]; then
    printf '다른 터미널에서 ./scripts/run.sh example-command --duration %s로 예제를 다시 실행할 수 있습니다.\n' "$duration"
  else
    printf '다른 터미널에서 ./scripts/run.sh moveit-command --language %s --joint 4 --delta -0.1로 추가 이동할 수 있습니다.\n' "$language"
  fi
fi
wait "$launch_pid"
