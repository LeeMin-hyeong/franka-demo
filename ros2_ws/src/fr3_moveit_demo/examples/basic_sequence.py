#!/usr/bin/env python3
"""짧은 Python 함수 호출로 FR3 팔과 가상 그리퍼를 순서대로 움직이는 예제.

입력은 duration ROS 파라미터[s], 출력은 제어 결과와 최종 관절 상태다.
Robot의 상태/제한/계획/실행 검사를 재사용하며 별도 플래너를 작성하지 않는다.
scripts/run.sh example이 Fake Hardware와 RViz를 준비한 뒤 이 파일을 실행한다.
작업을 확장하려면 run_sequence의 호출 순서를 수정한다. 실제 FCI 연결은 없다.
"""

import sys

from fr3_moveit_demo import Robot


def run_sequence(robot, duration):
    """Home -> 상대 이동 -> 그리퍼 -> Home을 수행한다.

    robot은 연결된 공통 제어 API, duration은 각 이동의 최소 시간[s]이다.
    최종 관절 위치 dict를 반환한다. 상태/모델/목표/계획/실행 오류는 예외로
    전달되므로 실패 이후 단계는 실행되지 않는다. 그리퍼 폭의 단위는 m다.
    """
    robot.home(duration=duration)
    robot.move_joint(joint=4, delta=0.1, duration=duration)
    robot.set_gripper(width=0.04, duration=duration)
    # Home은 팔만 ready로 복귀시킨다. 그리퍼는 앞서 지정한 40 mm 폭을 유지한다.
    robot.home(duration=duration)
    return robot.get_joint_positions()


def main(args=None):
    """ROS 인자로 예제를 실행한다. 성공0, 실패1, Ctrl+C130이며 자원을 정리한다."""
    try:
        # 같은 Robot 연결을 모든 단계에서 재사용한다. with를 벗어나면 진행 중인
        # 목표를 정리하고 이 API가 소유한 ROS 통신 자원을 닫는다.
        with Robot(args=args) as robot:
            duration = robot.declare_parameter('duration', 3.0).value
            final = run_sequence(robot, duration)
            print('SEQUENCE COMPLETE: arm at ready, gripper width 0.04 m. No FCI connection.')
            print(f'FINAL fr3_joint4 = {final["fr3_joint4"]:.6f} rad')
            print(f'FINAL gripper_width = {final["fr3_finger_joint1"] + final["fr3_finger_joint2"]:.6f} m')
        return 0
    except KeyboardInterrupt:
        print('Sequence interrupted. The Robot context closes the active fake goal.', file=sys.stderr)
        return 130
    except Exception as error:
        print(f'Sequence failed: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
