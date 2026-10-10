#!/usr/bin/env python3
"""움직임을 요청하지 않고 공식 FR3의 현재 관절 상태를 읽는 Python 입문 예제.

입력은 실행 중인 Fake Hardware의 /joint_states, 출력은 이름별 위치다.
팔은 rad, 손가락은 m이며 두 손가락 위치의 합이 전체 벌림 폭이다.
run.sh server로 스택을 시작하고 ros2 run fr3_moveit_demo read_state.py로 실행한다.
공통 Robot API가 실제 메시지 수신과 모델을 확인하며 FCI는 사용하지 않는다.
"""

import sys

from fr3_moveit_demo import Robot


def main(args=None):
    """새 상태 dict를 출력한다. 성공0, 상태/연결 오류1, Ctrl+C130을 반환한다."""
    try:
        with Robot(args=args) as robot:
            positions = robot.get_joint_positions()
            for name, value in positions.items():
                unit = 'm' if 'finger' in name else 'rad'
                print(f'{name} = {value:.6f} {unit}')
            width = positions['fr3_finger_joint1'] + positions['fr3_finger_joint2']
            print(f'gripper_width = {width:.6f} m')
            print('STATE READ COMPLETE. No motion requested. No FCI connection.')
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f'State read failed: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
