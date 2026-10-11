# Python으로 시작하고 확장하기

처음 설치부터 실행하려면 `./scripts/start.sh`, 설치 후 기본 관절 데모는 `./scripts/run.sh`다. Python 클라이언트가 공식 FR3 모델을 사용해 MoveIt에 계획과 실행을 요청하고, Fake Hardware의 결과를 RViz로 확인한다. 기본값은 실제 로봇에 연결하지 않는다. 설치는 [README](../README.md#4-설치-가이드), 실물 준비는 [실물 연결 가이드](real-robot.md)를 따른다. [공식 환경에 추가하는 구성안](official-addon.md)은 향후 설계로만 남겨둔다.

## 가장 짧은 작업 코드

터미널 A에서 저장소 루트의 `./scripts/run.sh server`를 실행한다. 자동 이동 없이 Fake Hardware와 RViz를 준비한다. 터미널 B에서 ROS와 이 저장소의 install을 source한 뒤 자신의 Python 파일을 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/franka-demo/ros2_ws/install/setup.bash
python3 my_task.py
```

`my_task.py`의 최소 내용:

```python
from fr3_moveit_demo import Robot

with Robot() as robot:
    robot.home()
    robot.move_joint(joint=4, delta=0.1, duration=3.0)
    robot.set_gripper(width=0.04)
    positions = robot.get_joint_positions()
    print(positions['fr3_joint4'])
```

실행 가능한 같은 흐름은 [basic_sequence.py](../ros2_ws/src/fr3_moveit_demo/examples/basic_sequence.py)에 있다. `./scripts/run.sh example`으로 서버와 함께 실행하거나 이미 실행 중인 서버에 `./scripts/run.sh example-command`로 보낸다. 예제 마지막 Home은 팔만 복귀시키므로 그리퍼 폭은 40 mm로 유지된다. 상태만 읽는 [read_state.py](../ros2_ws/src/fr3_moveit_demo/examples/read_state.py)는 `ros2 run fr3_moveit_demo read_state.py`로 실행한다.

## 함수 계약

| 호출 | 입력과 의미 | 반환 / 실패 |
|---|---|---|
| `Robot(args=None, wait_timeout=30.0, backend='fake', robot_ip=None)` | ROS 인자, 대기 시간 [s] 0 초과 300 이하. backend는 fake 또는 real, real에는 명시한 robot_ip 필요 | 본인 Context, Node, executor 생성. 모델 연결은 첫 요청 때 수행 |
| `connect(wait_timeout=None)` | 선택적 대기 시간 [s]. 지정하면 이후 요청에도 적용. 보통 직접 호출할 필요 없음 | 모델과 액션 서버를 확인하고 자기 객체 반환. 연결 실패 시 예외 |
| `home(duration=3.0)` | 각 이동의 최소 시간 [s] | 공식 SRDF ready로 팔을 이동하고 최종 상태 dict 반환 |
| `move_joint(joint=4, delta=0.1, duration=3.0)` | 관절 번호 1~7, 현재값 대비 이동량 [rad], 최소 시간 [s] | 해당 관절 목표를 계획/실행하고 최종 상태 dict 반환 |
| `set_gripper(width=0.04, duration=3.0)` | 전체 폭 [m] 0~0.08. fake에서는 최소 시간 [s], real에서는 속도 계산 기준 [s] | fake는 MoveIt 계획/실행, real은 공식 Move 액션. 최종 측정 상태 dict 반환 |
| `grasp(width, force, speed=0.01, epsilon=0.003)` | 물체 폭/허용 오차 [m], 힘 [N], 속도 [m/s]. 0<폭≤0.08, 힘 5~70, 0<speed≤0.05, 0<epsilon<폭/2 및 ≤0.005 | real은 공식 Grasp의 성공/새 측정 폭 검사. fake는 폭 이동. 최종 상태 dict 반환. 물체 유지/미끄러짐 검증은 아님 |
| `get_joint_positions()` | 없음 | 새 상태의 `{관절 이름: 위치}`. 팔은 rad, 손가락은 m |
| `stop()` | 없음 | 본인의 진행 중 목표에 정지/취소를 요청하고 응답 확인. real에서 결과가 불확실하면 같은 객체의 후속 이동 차단 |
| `close()` | 없음 | 본인의 목표와 executor/Node/Context 정리. 반복 호출 가능 |

기본 가상 손 모델에서는 상태 dict에 팔 7개와 손가락 2개가 있다. 손 없는 실물 서버에서는 팔 7개만 있다. 배열 순서 대신 `positions['fr3_joint4']`처럼 이름으로 읽는다. 전체 폭은 `fr3_finger_joint1 + fr3_finger_joint2`다. 손가락 키가 없는 모델에서 set_gripper는 실패한다. duration은 양수이며 최대 3600초다. 팔과 가상 손의 MoveIt 계획이 요청보다 길면 줄이지 않는다. API 자체의 duration 기본값은 3초이고 `run.sh real-command`의 기본값은 5초다.

이미 목표에 도달한 요청은 이동을 생략할 수 있다. MoveIt이 시간 0인 단일 정지점을 반환하면 컨트롤러에 전송하지 않고 새 실제 상태를 목표와 비교한다. 팔 허용 오차는 0.002 rad, 손가락과 전체 폭은 0.0002 m다. 목표와 다르면 실패하며, 정상 확인은 `Goal SUCCESS`로 출력해 궤적 실행과 구분한다. duration은 이미 도달한 자세에서 기다리는 시간을 뜻하지 않는다.

서버 부재, 현재 상태 미수신/오래된 상태, 중복 상태 발행자, backend/공식 모델/IP 불일치, 제한 초과, 계획/실행 실패, 최종 오차 초과는 예외로 전달된다. 잘못된 입력을 잡아 수정할 수 있지만 계획이나 실행 실패를 무시하고 후속 작업을 진행하지 않는다. `with`를 벗어날 때 예외가 발생해도 API 소유 자원은 정리된다.

## 실물 backend의 선택

[실물 준비](real-robot.md)를 완료하고 native Ubuntu 제어 PC에서 `./scripts/doctor.sh --real`을 통과한 뒤 서버를 시작한다. IP는 실제 Control LAN 주소로 바꾼다. 현재 실물 동작은 Not Tested다.

터미널 A:

```bash
./scripts/run.sh real-server --robot-ip 172.16.0.2
```

터미널 B의 작업 파일:

```python
from fr3_moveit_demo import Robot

with Robot(backend='real', robot_ip='172.16.0.2') as robot:
    print(robot.get_joint_positions())
    robot.move_joint(joint=4, delta=0.01, duration=5.0)
```

Robot은 실물 FCI 소켓을 직접 소유하지 않는다. 공식 franka_hardware가 연결한 서버에 요청하고, 서버 URDF의 정확한 `franka_hardware/FrankaHardwareInterface` 플러그인과 IP 일치를 확인한다. 기본 fake 객체는 이 서버를 거부하고 real 객체도 가상 서버를 거부한다. 이 검사는 설정 일치를 확인하며 실제 로봇의 설치 상태나 환경을 검증하는 기능은 아니다.

실물 상태는 팔과 선택한 손의 raw 측정을 별도로 확인한다. 새 timestamp, 최근 1초 이내의 측정/수신, 각 토픽의 단일 발행자가 필요하다. 집계 `/joint_states`의 timestamp만 새로 바뀌면 거부한다. `get_joint_positions()`의 반환값과 최종 목표 비교에는 raw 위치를 사용하며 팔/손 실행 요청 직전에 측정 상태를 다시 검사한다.

ROS 인자 `-p backend:=real -p robot_ip:=...`도 명시적인 real 선택이다. 생성자의 args가 None이면 프로그램의 ROS 실행 인자를 사용한다. ROS 파라미터가 생성자 backend/IP 값을 덮어쓸 수 있으므로 두 방법을 섞지 않는다. 기본 가상 예제는 가상 학습용으로 사용하고 실물 작업 파일에는 의도한 상태 조회와 동작을 직접 적는다.

실물 서버를 `--with-gripper`로 시작하면 `set_gripper`는 `/franka_gripper/move`에 전체 폭과 속도를 보낸다. `speed = min(0.05, abs(target-current)/duration)` m/s를 사용한다. duration은 속도 계산 기준이며 정확한 완료 시간을 보장하지 않는다. 이미 목표 폭이면 이동 요청을 생략한다. 결과 success/error와 새 손가락 상태를 검사한다. grasp/force/자동 homing은 호출하지 않으며 이 직접 액션은 MoveIt 손 경로 충돌 검사를 거치지 않는다. [고정 Franka Move 구현](https://github.com/frankarobotics/franka_ros2/blob/3b50164eabbe2d461baca007bbbbe03c024387bd/franka_gripper/src/gripper_action_server.cpp)

## 작은 구조를 유지하는 방법

| 바꾸려는 내용 | 먼저 수정할 파일 |
|---|---|
| 관절, 폭, 작업 순서, 반복 횟수 | `examples/basic_sequence.py` 또는 새 작업 파일 |
| 모든 작업이 공유할 제어 기능 | `fr3_moveit_demo/robot.py` |
| 터미널 옵션과 기본값 | `scripts/run.sh`, ROS 파라미터 변환은 패키지의 `scripts/moveit_demo.py` |
| 가상 모델, 플래너, 컨트롤러, RViz 구성 | `launch/fake_moveit.launch.py` |
| 공식 실물 드라이버, 상태 합침, 손 포함 여부 | `launch/real_moveit.launch.py` |
| C++ MoveGroupInterface 비교 | `src/moveit_demo.cpp` |
| 직접 궤적을 계산하는 원리 | 별도 `standalone/libfranka_demo`와 `fr3_offline_preview` |

위 패키지 내 경로의 기준은 `ros2_ws/src/fr3_moveit_demo/`다. 새 예제를 `ros2 run`으로 제공하려면 `CMakeLists.txt`의 `install(PROGRAMS ...)`에 추가하고 `./scripts/build.sh`로 설치한다. source 폴더에 작성한 파일을 `python3`로 실행하는 경우에도 공통 모듈의 최초 빌드와 환경 source는 필요하다.

예제에는 작업 의도만 적는다. 현재 상태 확인, 모델 제한, 계획/실행 분리, 최종 위치 검사, 취소 처리는 공통 API에서 유지한다. 새 함수는 기존의 공식 MoveIt 메시지/액션을 재사용하고 입력의 단위와 반환값을 주석으로 설명한다.

## 새 작업을 만드는 두 경로

**프로젝트 안에서 시작:** 터미널 A는 `./scripts/run.sh server`로 가상 서버를 유지한다. 터미널 B에서 상태 조회 예제를 복사한다. 처음 실행은 이동 목표를 보내지 않는다.

```bash
cd ~/franka-demo
mkdir -p tasks
cp -n ros2_ws/src/fr3_moveit_demo/examples/read_state.py tasks/my_task.py
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source ~/franka-demo/ros2_ws/install/setup.bash
python3 tasks/my_task.py
```

`STATE READ COMPLETE`와 관절값을 확인한다. `tasks/my_task.py`의 `with Robot(...)` 안에 필요한 호출을 순서대로 추가한다. 예를 들어 가상에서 `positions = robot.get_joint_positions()` 앞에 `robot.move_joint(joint=4, delta=0.1, duration=3.0)`를 넣고 다시 실행한다. 작업 파일만 수정하면 재빌드가 필요 없고 공통 패키지를 수정하면 `./scripts/build.sh`가 필요하다. 실물로 옮길 때는 해당 작업에서 `Robot(args=[], backend='real', robot_ip='실제 Control IP')`를 직접 지정하고 이동량과 주변 환경을 다시 확인한다.

**새 ROS2 워크스페이스로 분리:** 먼저 프로젝트 빌드를 완료한다. 비어 있는 `~/fr3_task_ws`에서 아래처럼 상태 조회 패키지를 만든다. 이미 같은 이름의 패키지가 있다면 새 이름/폴더를 사용한다.

```bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source ~/franka-demo/ros2_ws/install/setup.bash
mkdir -p ~/fr3_task_ws/src
cd ~/fr3_task_ws/src
ros2 pkg create fr3_tasks --build-type ament_python \
  --node-name read_state --dependencies fr3_moveit_demo
cp ~/franka-demo/ros2_ws/src/fr3_moveit_demo/examples/read_state.py \
  fr3_tasks/fr3_tasks/read_state.py
cd ~/fr3_task_ws
colcon build --symlink-install --packages-select fr3_tasks
source install/setup.bash
ros2 run fr3_tasks read_state
```

마지막 명령은 A의 가상 서버가 실행 중일 때 새 상태를 읽는다. `--node-name`이 Python 실행 진입점을 만들고 `--dependencies`가 package.xml에 프로젝트 API 의존성을 기록한다. `fr3_tasks/fr3_tasks/read_state.py`를 수정해 작업을 추가한다. 새 터미널은 Humble → 프로젝트 install → 작업 워크스페이스 install 순서로 source한다. 두 터미널의 `ROS_DOMAIN_ID`를 같게 유지하고 두 번째 로봇 서버를 시작하지 않는다.

이 분리 방법도 프로젝트 API를 재사용한다. 프로젝트 없이 구성하려면 공식 franka_ros2/MoveIt 환경을 설치하고 MoveGroup/ExecuteTrajectory/Franka 그리퍼 액션 클라이언트를 직접 작성해야 한다. Python의 `Robot`은 공식 libfranka 클래스가 아니라 이 프로젝트가 제공하는 편의 API다. [공식 Humble 패키지 작성 안내](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)를 참고한다.

## 연결과 실행의 수명

`Robot`은 독립 ROS Context와 executor를 소유한다. import만으로 ROS를 초기화하거나 서버에 연결하지 않는다. 첫 동작에서 URDF/SRDF와 액션 서버를 확인한 뒤 같은 객체가 이를 재사용한다. 각 이동과 상태 조회는 새 JointState를 확인한다. 전역 rclpy Context를 종료하지 않으므로 외부 ROS 코드의 자원과 구분된다.

이 API는 한 스레드에서 차례로 호출하는 동기 방식이다. 메서드가 돌아오기 전에 다음 동작을 시작하지 않는다. 별도 터미널에서 동시에 다른 클라이언트로 명령을 보내지 않는다. 프로그램 종료에는 `with` 또는 `try/finally`의 `close()`를 사용한다. Ctrl+C는 본인의 실행에 정지/취소를 요청하고 결과를 확인한다. 실물의 물리적 비상 정지와는 별개이며 응답을 못 받으면 정지 성공으로 간주하지 않는다.

실물 실행의 정지 결과가 불확실하면 같은 Robot 객체의 home/move_joint/set_gripper를 막는다. get_joint_positions와 stop 재시도는 허용하며 종료 결과를 확인한 뒤에만 이동을 다시 허용한다. 새 객체나 새 CLI로 실패 상태를 우회하지 말고 실제 로봇 상태와 현장 정지 절차를 먼저 확인한다. 이 객체 내부의 차단은 로봇 전체의 별도 안전 시스템이 아니다.

실물 실행 실패 응답도 목표를 유지하고 정지를 요청한다. 팔의 ABORTED/CONTROL_FAILED/TIMED_OUT은 정지 확인으로 보지 않는다. 손은 공식 `/franka_gripper/stop` 서비스까지 요청하며 목표 종료와 서비스 성공을 함께 확인해야 차단을 해제한다. 수락 대기 상태가 미확인이거나 서비스가 실패하면 차단을 유지한다. 서비스 성공은 물리적 정지 성능의 검증이 아니다.

현재 backend는 GenericSystem의 fake와 공식 FrankaHardwareInterface의 real이다. 카메라는 [팔 장착 D435i 캘리브레이션](calibration.md)으로 촬영, 보정 계산과 베이스 좌표 확인을 할 수 있다. 별도 [CV 선택 집기 작업](pick-and-return.md)의 `PickAndReturn`이 TCP Pose 계획과 직선 접근을 제공하고, `Robot.grasp(width, force, speed=0.01, epsilon=0.003)`가 공식 Grasp를 호출한다. 단위는 폭/허용 오차 m, 힘 N, 속도 m/s다. fake는 손가락 위치만 움직인다. Isaac Sim 연결, 임의 집기 자세 탐색과 미끄러짐 검출은 구현하지 않았다. 가상 재현과 실제 FCI 연결/동작 검증은 별개다.
