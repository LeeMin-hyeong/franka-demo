# 공식 Franka 환경에 작업 노드를 추가하는 구성안

**문서로 남기는 향후 설계안이다. 이번 변경에서는 설치 스크립트나 실행 구조를 바꾸지 않는다.** 현재 실행은 [처음 사용하기](getting-started.md)를 따른다. 아래 구성을 복제만으로 자동 실행하는 기능은 아직 제공하지 않는다.

## 권장 구조

공식 `franka_ros2` Humble 환경을 먼저 설치하고, 같은 워크스페이스에 `franka-demo`를 복제해 카메라와 작업 노드를 추가한다. 공식 저장소를 fork하거나 내부 파일을 수정하는 방식은 피한다.

```text
franka_ros2_ws/src/
├── franka_ros2/           공식 로봇 연결과 MoveIt 설정
├── libfranka/            공식 dependency.repos 의존성
├── franka_description/   공식 모델
└── franka-demo/          카메라 보정과 Python 작업
```

```mermaid
flowchart LR
  A[Python 작업 노드] --> B[공식 MoveIt]
  B --> C[공식 franka_ros2 드라이버]
  C --> D[libfranka / FCI]
  D --> E[실제 FR3와 Hand]
  F[공식 D435i ROS 드라이버] --> G[보정 / 물체 좌표]
  G --> A
```

| 담당 | 사용할 구성 |
|---|---|
| 로봇 연결, 상태, 팔/손 드라이버 | 공식 franka_ros2 |
| 모델, 경로 계획, 실행 | 공식 description와 MoveIt 설정 |
| D435i 영상과 정렬 깊이 | 공식 realsense2_camera |
| 촬영, 손눈 보정, 좌표 변환 | franka-demo의 calibration 코드 |
| 클릭 집기와 새 작업 순서 | franka-demo의 Python 작업 노드 |

## 현재 저장소에서 재사용할 코드

패키지 기준 경로는 `ros2_ws/src/fr3_moveit_demo/`다.

| 기능 | 파일 |
|---|---|
| 상태/계획/실행 Python 호출 | `fr3_moveit_demo/robot.py` |
| 작업 순서 | `examples/`, 사용자가 작성하는 `tasks/` |
| 촬영과 손눈 보정 | 저장소 루트의 `calibration/record.py`, `solve.py` |
| 클릭 좌표 | `examples/click_camera_point.py` |
| 집기와 원위치 복귀 | `examples/pick_and_return.py`, `fr3_moveit_demo/pick_place.py` |

새 기능은 작업 노드부터 추가한다. 재사용할 제어 호출이 필요할 때만 공통 Python API를 수정한다. 프로젝트가 커지면 별도 ROS2 작업 패키지로 분리하고 공식 환경을 source해 사용한다.

## 적용 전에 필요한 정리

현재 스크립트는 저장소 내부 `ros2_ws/install`과 자체 설치 경로를 기준으로 한다. 공식 환경에 그대로 복제해 기존 setup을 실행하면 의존성을 중복 준비할 수 있다. 향후 적용하려면 다음을 정리해야 한다.

1. 추가 패키지만 빌드하고 공식 소스/설치 결과를 재사용하는 설치 경로.
2. 공식 워크스페이스의 환경을 선택하는 실행 경로.
3. 로봇 서버는 공식 launch로 시작하고 작업 클라이언트만 추가하는 터미널 안내.
4. 공식 서버와 Python 작업, 카메라, 보정, 집기 간의 통합 검증.

Ubuntu 22.04에서는 [공식 Humble 안내](https://github.com/frankarobotics/franka_ros2/tree/humble)를 사용한다. 로봇 시스템과 libfranka는 해당 브랜치의 `dependency.repos` 및 공식 호환표에 맞춘다. 기존 저장소의 검증 조합은 Humble 커밋 `3b50164eabbe2d461baca007bbbbe03c024387bd`, libfranka 0.20.4, description 2.8.1이다.

공식 Humble Fake Hand는 고정 손가락 상태만 발행한다. 현재의 가상 손 집기 예제를 공식 fake 팔에 그대로 연결할 수 있다고 가정하지 않는다. 실물 손은 공식 그리퍼 액션을 사용한다.

공식 드라이버를 재사용해도 RT 커널/권한, Desk/IP/FCI 준비, 도구 하중과 카메라 보정은 필요하다. 이번 구성안의 전체 통합과 실제 FR3/Hand/D435i 동작은 미검증이다.
