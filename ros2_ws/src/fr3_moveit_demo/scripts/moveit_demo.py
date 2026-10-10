#!/usr/bin/env python3
"""ROS 파라미터를 공통 Robot API 호출로 바꾸는 짧은 CLI 진입점.

입력: joint(1~7), delta(rad), duration(s), home, wait_timeout(s), gripper_width(m).
gripper_width=-1은 팔 모드, 0 이상은 두 손가락 사이의 전체 개방 폭이다.
출력: API의 실제 상태/계획/실행 로그, 성공0, 실패1, Ctrl+C130 종료 코드.
제어 검증은 fr3_moveit_demo/robot.py에 있으며 사용자 Python 예제도 같은 API를 쓴다.
이 파일은 ROS 파라미터 해석과 명령 선택만 수행하고 실제 FCI 연결은 하지 않는다.
"""

import math
import sys

from fr3_moveit_demo import Robot


def run(robot):
    """CLI 파라미터를 읽어 한 동작을 요청한다. 실패는 예외로 main에 전달한다.

    robot은 독립 ROS 자원을 소유한 공통 API다. duration은 최소 이동 시간(s),
    wait_timeout은 응답 대기 한도(s)이며 home과 그리퍼 폭 목표는 함께 지정할 수 없다.
    반환값은 API가 검증한 실제 전체 최종 관절 상태 dict다.
    """
    joint = robot.declare_parameter('joint', 4).value
    delta = robot.declare_parameter('delta', 0.1).value
    duration = robot.declare_parameter('duration', 3.0).value
    home = robot.declare_parameter('home', False).value
    timeout = robot.declare_parameter('wait_timeout', 30.0).value
    width = robot.declare_parameter('gripper_width', -1.0).value
    if not math.isfinite(width) or (width < 0 and width != -1.0) or (width >= 0 and home):
        raise ValueError('gripper_width must be -1 (arm mode) or a finite nonnegative width; home is arm-only.')
    robot.connect(wait_timeout=timeout)
    if width >= 0:
        return robot.set_gripper(width=width, duration=duration)
    if home:
        return robot.home(duration=duration)
    return robot.move_joint(joint=joint, delta=delta, duration=duration)


def main(args=None):
    """독립 ROS 연결을 열고 CLI를 실행한다. with가 정상/실패/Ctrl+C 자원을 정리한다."""
    try:
        with Robot(args=args) as robot:
            run(robot)
        return 0
    except KeyboardInterrupt:
        print('Interrupted. The Robot context closes the active Fake Hardware goal.', file=sys.stderr)
        return 130
    except Exception as error:
        print(f'MoveIt demo failed: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
