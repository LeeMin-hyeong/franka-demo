# Python 기본 FR3 RViz 데모와 C++ 비교

이 문서는 가상 RViz 데모의 실행과 코드 확장을 설명한다. 기본 데모는 공식 FR3의 Joint 4를 ready 자세에서 +0.1 rad 이동한다. Python과 C++ 모두 기존 MoveIt 플래너와 ros2_control GenericSystem을 사용한다. Python 예제는 공통 Robot API의 짧은 함수 호출로 동작을 구성한다. 아래 가상 명령은 FCI를 호출하지 않는다. 실물은 별도의 [실물 연결 가이드](real-robot.md)와 real-* 명령을 사용하며 Not Tested다. Isaac Sim은 구현하지 않았다.

## 준비와 실행

Ubuntu 22.04 Bash에서 저장소 루트로 이동한다. ROS도 없는 환경에서의 Git 설치/복제는 [README 설치 가이드](../README.md#4-설치-가이드)를 따른다. 설치부터 가상 작업 실행까지 한 번에 수행하려면 다음 명령을 쓴다.

```bash
./scripts/start.sh
```

setup → build → doctor → Python example을 수행하고 RViz를 유지한다. Home → Joint 4 +0.1 rad → 그리퍼 40 mm → Home 순서다. start는 시스템 패키지를 설치하고 최초 ROS 환경에서는 시스템 업데이트도 수행한다. 이미 설치/빌드가 끝났다면 run으로 실행하면 된다. 개별 설치/빌드는 다음과 같다.

```bash
./scripts/setup.sh
./scripts/build.sh
./scripts/doctor.sh
```

준비 후 한 관절만 확인하려면 다음 명령으로 실행한다. Python 클라이언트가 기본값이다.

```bash
./scripts/run.sh
```

같은 기본 동작을 옵션과 함께 쓰거나 C++으로 비교할 수 있다. 이전 스택을 Ctrl+C로 종료한 뒤 다음 중 하나만 실행한다.

```bash
./scripts/run.sh moveit --joint 4 --delta 0.1 --duration 3
./scripts/run.sh moveit --language cpp --joint 4 --delta 0.1 --duration 3
```

컨트롤러 준비와 RViz 초기화 대기 후 이동한다. 시작/목표/최종 7개 관절각과 계획/실행 결과를 출력하고 RViz를 유지한다. Joint 4의 시작값은 약 -2.356194 rad, 목표값은 약 -2.256194 rad이다. 관절 번호는 1~7, 각도는 rad, duration은 최소 실행 시간 s다. 도움말은 `./scripts/run.sh --help`이며 무인자 실행은 도움말 대신 데모를 시작한다.

RViz의 PlanningScene은 Fake Hardware에서 실행된 현재 자세를 보여준다. Trajectory의 계획 경로 표시와 현재 자세는 서로 다른 데이터다. 실행 성공은 화면만 보고 판정하지 않고 실행 후 받은 JointState가 목표와 0.002 rad 이내인지 확인한다.

## 서버만 시작하고 짧은 예제 실행

터미널 A에서 준비한다. 이 모드는 RViz와 가상 컨트롤러만 띄우고 자동 이동을 요청하지 않는다.

```bash
./scripts/run.sh server
```

터미널 B에서 설치된 Python 예제를 실행한다. 별도의 스택을 중복 실행하지 않는다.

```bash
./scripts/run.sh example-command --duration 3
```

예제의 순서는 Home → Joint 4 +0.1 rad → 그리퍼 전체 폭 0.04 m → Home이다. 마지막 Home은 팔만 ready로 복귀시키므로 그리퍼는 40 mm 폭을 유지한다. 중간 단계가 실패하면 뒤의 단계는 실행하지 않는다. 최종 출력은 `SEQUENCE COMPLETE`와 실제 관절/그리퍼 상태다.

아무 스택도 실행 중이지 않을 때는 시작부터 예제까지 한 명령으로 묶을 수 있다.

```bash
./scripts/run.sh example --duration 3
```

이 명령도 실행 후 RViz를 유지한다. example은 Python 전용이고 duration은 각 이동의 최소 시간이다. joint/delta/home/gripper-width/rate 옵션은 정해진 예제 순서와 충돌하므로 거부한다. 순서를 바꾸려면 `examples/basic_sequence.py`의 함수 호출을 수정한다.

현재 상태만 읽으려면 A의 server를 유지하고 B에서 다음 명령을 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run fr3_moveit_demo read_state.py
```

`read_state.py`는 이동을 요청하지 않는다. 이름별 위치와 전체 그리퍼 폭을 출력하며 `STATE READ COMPLETE`를 확인한다. run.sh는 자식 셸 안에서 환경을 설정하므로 직접 ros2 명령을 쓰는 터미널에는 위 source가 필요하다.

## 공통 Python Robot API

설치된 `fr3_moveit_demo/robot.py`가 ROS 상태, 모델 검사, 계획, 실행을 담당한다. CLI와 짧은 예제가 같은 구현을 재사용한다.

```python
from fr3_moveit_demo import Robot

with Robot() as robot:
    robot.home(duration=3.0)
    robot.move_joint(joint=4, delta=0.1, duration=3.0)
    robot.set_gripper(width=0.04, duration=3.0)
    robot.home(duration=3.0)
    positions = robot.get_joint_positions()
    print(positions["fr3_joint4"])
```

| 함수 | 입력과 결과 |
|---|---|
| `home(duration=...)` | 팔을 공식 SRDF ready로 이동하고 수신한 상태 dict 반환. 그리퍼 폭은 유지 |
| `move_joint(joint=..., delta=..., duration=...)` | 관절 번호 1~7의 현재값에서 delta [rad]만큼 이동하고 상태 dict 반환 |
| `set_gripper(width=..., duration=...)` | 전체 폭 [m]을 지정하고 양쪽 손가락의 실제 상태를 확인한 dict 반환 |
| `get_joint_positions()` | 새로운 JointState에서 이름별 위치 dict 반환. 이동 요청 없음 |

duration의 단위는 s다. 위 메서드는 완료 또는 실패까지 기다리므로 한 스레드에서 차례로 호출한다. Robot은 첫 제어/상태 조회 시 서버에 연결하고 공식 모델을 읽어 재사용한다. `with`를 벗어나면 진행 중인 가상 목표와 자신의 ROS 통신 자원을 정리한다. Python의 이 Robot 클래스는 실제 FCI 연결을 수행하는 libfranka의 `franka::Robot`과 다르다.

예제 파일은 `ros2_ws/src/fr3_moveit_demo/examples/basic_sequence.py`와 `read_state.py`에 있으며 colcon 설치 후 ros2 run으로 실행한다. 직접 sequence를 실행할 때도 먼저 server를 켜 두고 환경을 source한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run fr3_moveit_demo basic_sequence.py --ros-args -p duration:=3.0
```

API 입력/반환값과 확장 방법은 [Python API 가이드](python-api.md)를 참고한다.

이 문서의 Robot()은 기본 fake backend다. 실물 backend를 선택하면 기존 가상 모델을 그대로 제어하지 않고 공식 실물 서버의 플러그인과 IP가 일치하는지 검사한다. C++ 학습 클라이언트는 계속 가상 전용으로 유지한다.

## 실행 중 추가 명령

첫 터미널에서 RViz 스택이 실행 중일 때 다른 Ubuntu 터미널에서 사용한다.

```bash
./scripts/run.sh moveit-command --joint 4 --delta -0.1 --duration 3
./scripts/run.sh moveit-command --language cpp --home --duration 5
```

클라이언트 언어를 바꾸어도 같은 서버와 현재 상태를 사용한다. 명령이 끝난 뒤 다음 명령을 보낸다. 처음 +0.1을 이동한 뒤 -0.1을 요청하면 초기 자세로 돌아간다. --home은 공식 SRDF의 ready를 사용한다. 두 명령 클라이언트를 동시에 실행하지 않는다.

## 가상 그리퍼 열기와 닫기

새 스택을 시작하며 기본 Python으로 80 mm까지 연다. 다른 스택이 켜져 있다면 먼저 종료한다.

```bash
./scripts/run.sh moveit --gripper-width 0.08 --duration 3
```

RViz를 유지한 채 다른 터미널에서 Python으로 닫거나 중간 폭으로 연다.

```bash
./scripts/run.sh moveit-command --gripper-width 0 --duration 3
./scripts/run.sh moveit-command --gripper-width 0.04 --duration 3
```

MoveIt의 팔/그리퍼 명령에서는 cpp 또는 python을 선택할 수 있다. 폭은 두 손가락 사이의 전체 거리 [m]이며 허용 범위는 0~0.08 m이다. 0.04는 40 mm 전체 폭, 즉 각 손가락 위치 0.02 m를 뜻한다. 초기 폭은 공식 SRDF open인 0.07 m다. `--gripper-width`와 팔 옵션 `--joint`, `--delta`, `--home`을 함께 지정하면 거부한다. 팔과 그리퍼 명령은 차례로 따로 보낸다.

MoveIt의 fr3_hand 그룹에서 finger_joint1만 계획하고 공식 URDF의 mimic 관계로 finger_joint2가 따라간다. 팔 자세를 유지하면서 두 손가락이 대칭으로 움직이는 것을 RViz에서 확인한다. 터미널은 손가락 START/GOAL/FINAL [m]과 `gripper_width`를 출력하며 새 상태의 각 위치가 목표와 0.0002 m 이내인지 비교한다. force, 접촉, 물체 grasp는 재현하지 않는다. 실물 그리퍼 API/드라이버를 호출하지 않는다.

직접 ROS 파라미터를 사용하는 경우에도 전체 폭을 double로 전달한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run fr3_moveit_demo moveit_demo --ros-args \
  -p gripper_width:=0.08 -p duration:=3.0
ros2 run fr3_moveit_demo moveit_demo.py --ros-args \
  -p gripper_width:=0.0 -p duration:=3.0
```

위 두 명령은 이미 실행 중인 서버에 차례로 보내는 예다. 그리퍼 옵션이 없는 명령은 현재 그리퍼 폭을 유지한 채 팔을 이동한다. libfranka-preview는 기존 팔 전용 프리뷰이며 그리퍼 옵션을 거부한다.

## 코드 읽는 순서

| 순서 | C++ | Python | 하는 일 |
|---|---|---|---|
| 1 | `src/moveit_demo.cpp` | `scripts/moveit_demo.py` → `fr3_moveit_demo/robot.py` | CLI 입력과 공통 API 연결 |
| 2 | MoveGroupInterface의 상태 모니터와 별도 구독 | Robot의 rclpy JointState 구독 | 팔 7개, 그리퍼 모드에서는 손가락 상태도 확인 |
| 3 | 현재값에서 상대 목표 또는 ready 설정 | 현재값에서 상대 목표 또는 ready 설정 | 모델의 관절 위치 제한 검사 |
| 4 | `move_group.plan(plan)` | `MoveGroup` 액션, `plan_only=True` | 경로 계획만 요청하고 성공 검사 |
| 5 | `move_group.execute(plan)` | `ExecuteTrajectory` 액션 | Fake Hardware 컨트롤러에 실행 요청 |
| 6 | 실행 후 Current State 조회 | 실행 후 새 JointState 대기 | 수신한 최종값을 출력하고 목표와 비교 |

소스 경로의 기준은 `ros2_ws/src/fr3_moveit_demo/`다. Python CLI는 Robot을 호출하는 입구이고 ROS 액션 처리 코드는 robot.py에 모여 있다. Python은 자체 플래너를 만들지 않고 공식 ROS2 메시지/액션으로 같은 MoveIt 서버에 요청한다. 충돌 검사와 경로 계획은 서버가 담당한다.

코드를 수정한 뒤 빌드한다.

```bash
./scripts/build.sh
```

새 실행 옵션을 만들지 않고 관절과 이동량만 바꾸려면 --joint/--delta/--duration을 사용하면 된다. 최종 위치가 모델 제한 밖이면 계획 전에 거부한다. 모든 관절을 0으로 만드는 자세는 Joint 4와 Joint 6의 제한 밖이다.

## ROS 명령으로 직접 실행

앞의 run.sh server와 같은 구성을 직접 ROS 명령으로 띄우는 방법이다. 아래 첫 명령은 이동 명령을 자동 전송하지 않는다.

터미널 A:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch fr3_moveit_demo fake_moveit.launch.py
```

터미널 B에서 둘 중 하나를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 run fr3_moveit_demo moveit_demo --ros-args \
  -p joint:=4 -p delta:=0.1 -p duration:=3.0

ros2 run fr3_moveit_demo moveit_demo.py --ros-args \
  -p joint:=4 -p delta:=0.1 -p duration:=3.0
```

위 클라이언트를 차례로 모두 실행하면 현재값에서 +0.1씩 두 번 이동한다. 한 버전만 시험하려면 해당 명령만 실행한다. ROS의 double 파라미터는 3 대신 3.0처럼 전달한다. 서버와 클라이언트는 같은 ROS_DOMAIN_ID를 사용해야 한다.

## 상태 확인과 종료

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /joint_states sensor_msgs/msg/JointState --once --no-daemon
ros2 topic info /joint_states --verbose --no-daemon
```

팔 7개와 손가락 2개, 총 9개 이름과 같은 순서의 position이 있어야 하고 발행자는 1개여야 한다. 배열 순서는 달라질 수 있으므로 이름으로 확인한다. 클라이언트는 상태 미수신, 모델 불일치, 중복 발행자, 제한 초과, 계획/실행 실패 시 비정상 종료 코드를 반환한다.

서버 실행 터미널의 Ctrl+C로 종료한다. Python 클라이언트만 실행하는 터미널 B에서 이동 중 Ctrl+C를 누르면 본인의 가상 궤적에 stop 이벤트를 요청하고 PREEMPTED 정지 결과를 확인한 뒤 종료한다. 목표 전송 직후 끊긴 경우에도 목표 수락 응답을 회수해 취소를 요청하며 정지/취소 응답을 확인하지 못하면 경고를 출력한다.

기존 Humble MoveIt/RViz 종료 소멸자 오류는 [검증 기록](validation.md)에 기록되어 있으며, 이동 성공과 오류 없는 종료를 구분한다.

## 실물 RViz와의 관계

native Ubuntu 제어 PC의 RT/권한, 시스템 버전, Control LAN과 Desk FCI 준비를 끝낸 뒤에만 실물 서버를 시작한다. 가상 데모를 먼저 종료한다. IP는 실제 Control 주소로 바꾼다.

터미널 A:

```bash
./scripts/doctor.sh --real
./scripts/run.sh real-server --robot-ip 172.16.0.2
```

터미널 B:

```bash
./scripts/run.sh real-state --robot-ip 172.16.0.2
```

실물 RViz는 측정한 현재 자세를 표시한다. ready로 초기화하거나 자동 이동하지 않으며 공식 arm controller의 현재 자세 유지 제어는 시작된다. 손이 실제 장착되었을 때만 A의 서버에 --with-gripper를 사용한다. 팔만 연결하면 7개 상태를, 손을 포함하면 9개 상태를 표시한다. real-state는 추가 이동을 요청하지 않는다.

실제로 움직일 때는 별도 real-command로 한 목표를 명시한다. 이 경로의 연결/동작/정지 성능과 실물 RViz 표시는 Not Tested다. 본 문서의 가상 검증 화면이 실물 검증 결과는 아니다. 단계별 준비와 명령은 [실물 가이드](real-robot.md)를 따른다.

## C++ Offline Preview와의 구분

MoveIt 스택을 종료한 뒤 다음 명령으로 별도의 프리뷰를 실행한다.

```bash
./scripts/run.sh libfranka-preview --joint 4 --delta 0.1 --duration 3 --rate 30
```

이 모드는 기존 C++ 5차 궤적 계산기로 팔 7개 관절만 재생한다. Python Robot API나 MoveIt 계획/실행 서버를 사용하지 않고 FCI도 호출하지 않는다. language와 gripper-width 옵션은 거부한다. MoveIt과 프리뷰는 같은 ROS_DOMAIN_ID에서 동시에 실행하지 않는다.

## 공식 API 자료

- [MoveIt2 Humble MoveGroupInterface](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html)
- [ROS2 Humble Python 액션 클라이언트](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [moveit_msgs 2.2.1 MoveGroup](https://github.com/moveit/moveit_msgs/blob/2.2.1/action/MoveGroup.action)
- [moveit_msgs 2.2.1 ExecuteTrajectory](https://github.com/moveit/moveit_msgs/blob/2.2.1/action/ExecuteTrajectory.action)
- [공식 Franka Hand URDF 제한과 mimic](https://github.com/frankarobotics/franka_description/blob/2.8.1/end_effectors/common/franka_hand.xacro)
- [공식 Franka Hand SRDF open/close](https://github.com/frankarobotics/franka_description/blob/2.8.1/end_effectors/franka_hand/franka_hand.srdf.xacro)
- [Humble GenericSystem의 mimic 지원](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html)
