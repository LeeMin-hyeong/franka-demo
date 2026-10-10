# Python으로 시작하고 확장하기

기본 실행은 `./scripts/run.sh`다. Python 클라이언트가 공식 FR3 모델을 사용해 MoveIt에 계획과 실행을 요청하고, Fake Hardware의 결과를 RViz로 확인한다. 실제 로봇에 연결하지 않는다. 설치는 [README](../README.md#4-설치-가이드)를 따른다.

## 가장 짧은 작업 코드

터미널 A에서 저장소 루트의 `./scripts/run.sh server`를 실행한다. 자동 이동 없이 Fake Hardware와 RViz를 준비한다. 터미널 B에서 ROS와 이 저장소의 install을 source한 뒤 자신의 Python 파일을 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
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
| `Robot(args=None, wait_timeout=30.0)` | ROS 인자, 연결/상태 대기 시간 [s], 0 초과 300 이하. None은 실행 인자를 사용 | 본인 Context, Node, executor 생성. 모델 연결은 첫 요청 때 수행 |
| `connect(wait_timeout=None)` | 선택적 대기 시간 [s]. 지정하면 이후 요청에도 적용. 보통 직접 호출할 필요 없음 | 모델과 액션 서버를 확인하고 자기 객체 반환. 연결 실패 시 예외 |
| `home(duration=3.0)` | 각 이동의 최소 시간 [s] | 공식 SRDF ready로 팔을 이동하고 최종 상태 dict 반환 |
| `move_joint(joint=4, delta=0.1, duration=3.0)` | 관절 번호 1~7, 현재값 대비 이동량 [rad], 최소 시간 [s] | 해당 관절 목표를 계획/실행하고 최종 상태 dict 반환 |
| `set_gripper(width=0.04, duration=3.0)` | 두 손가락 사이 전체 폭 [m], 0~0.08. 최소 시간 [s] | 각 손가락 목표는 width/2. 최종 상태 dict 반환 |
| `get_joint_positions()` | 없음 | 새 상태의 `{관절 이름: 위치}`. 팔은 rad, 손가락은 m |
| `stop()` | 없음 | 본인의 진행 중 목표에 정지/취소를 요청하고 응답 확인. 완료 여부는 로그로 구분 |
| `close()` | 없음 | 본인의 목표와 executor/Node/Context 정리. 반복 호출 가능 |

기본 손 모델에서는 상태 dict에 팔 7개와 손가락 2개가 있다. 배열 순서 대신 `positions['fr3_joint4']`처럼 이름으로 읽는다. 전체 폭은 `fr3_finger_joint1 + fr3_finger_joint2`다. 이동의 duration은 양수이며 최대 3600초다. MoveIt 계획이 요청보다 길면 줄이지 않는다.

이미 목표에 도달한 요청은 이동을 생략할 수 있다. MoveIt이 시간 0인 단일 정지점을 반환하면 컨트롤러에 전송하지 않고 새 실제 상태를 목표와 비교한다. 팔 허용 오차는 0.002 rad, 손가락과 전체 폭은 0.0002 m다. 목표와 다르면 실패하며, 정상 확인은 `Goal SUCCESS`로 출력해 궤적 실행과 구분한다. duration은 이미 도달한 자세에서 기다리는 시간을 뜻하지 않는다.

서버 부재, 현재 상태 미수신/오래된 상태, 중복 상태 발행자, 공식 모델 불일치, 제한 초과, 계획/실행 실패, 최종 오차 초과는 예외로 전달된다. 잘못된 입력을 잡아 수정할 수 있지만 계획이나 실행 실패를 무시하고 후속 작업을 진행하지 않는다. `with`를 벗어날 때 예외가 발생해도 API 소유 자원은 정리된다.

## 작은 구조를 유지하는 방법

| 바꾸려는 내용 | 먼저 수정할 파일 |
|---|---|
| 관절, 폭, 작업 순서, 반복 횟수 | `examples/basic_sequence.py` 또는 새 작업 파일 |
| 모든 작업이 공유할 제어 기능 | `fr3_moveit_demo/robot.py` |
| 터미널 옵션과 기본값 | `scripts/run.sh`, ROS 파라미터 변환은 패키지의 `scripts/moveit_demo.py` |
| 모델, 플래너, 컨트롤러, RViz 구성 | `launch/fake_moveit.launch.py` |
| C++ MoveGroupInterface 비교 | `src/moveit_demo.cpp` |
| 직접 궤적을 계산하는 원리 | 별도 `standalone/libfranka_demo`와 `fr3_offline_preview` |

위 패키지 내 경로의 기준은 `ros2_ws/src/fr3_moveit_demo/`다. 새 예제를 `ros2 run`으로 제공하려면 `CMakeLists.txt`의 `install(PROGRAMS ...)`에 추가하고 `./scripts/build.sh`로 설치한다. source 폴더에 작성한 파일을 `python3`로 실행하는 경우에도 공통 모듈의 최초 빌드와 환경 source는 필요하다.

예제에는 작업 의도만 적는다. 현재 상태 확인, 모델 제한, 계획/실행 분리, 최종 위치 검사, 취소 처리는 공통 API에서 유지한다. 새 함수는 기존의 공식 MoveIt 메시지/액션을 재사용하고 입력의 단위와 반환값을 주석으로 설명한다.

## 연결과 실행의 수명

`Robot`은 독립 ROS Context와 executor를 소유한다. import만으로 ROS를 초기화하거나 서버에 연결하지 않는다. 첫 동작에서 URDF/SRDF와 액션 서버를 확인한 뒤 같은 객체가 이를 재사용한다. 각 이동과 상태 조회는 새 JointState를 확인한다. 전역 rclpy Context를 종료하지 않으므로 외부 ROS 코드의 자원과 구분된다.

이 API는 한 스레드에서 차례로 호출하는 동기 방식이다. 메서드가 돌아오기 전에 다음 동작을 시작하지 않는다. 별도 터미널에서 동시에 다른 클라이언트로 명령을 보내지 않는다. 프로그램 종료에는 `with` 또는 `try/finally`의 `close()`를 사용한다. Ctrl+C는 진행 중인 본인의 가상 실행을 정리한다.

이 버전은 GenericSystem 전용이다. Isaac Sim이나 실물로 확장할 때는 환경의 상태 발행자, 시간, 컨트롤러 액션, 모델 확인 조건을 함께 검토해야 한다. 관련 백엔드 선택 기능은 아직 없다. Pose 목표, 카메라, 물체 작업도 구현하지 않았다. 현재 API가 제공하는 관절/가상 그리퍼 동작부터 작업 파일에 조합하면 된다.
