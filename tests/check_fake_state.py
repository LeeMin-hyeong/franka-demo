#!/usr/bin/env python3
"""가상 예제의 결과를 별도 새 구독으로 검사한다. FCI/실물 backend는 선택하지 않는다."""
import math

from fr3_moveit_demo import Robot

with Robot(args=[]) as robot:
    positions = robot.get_joint_positions()
    assert len(positions) == 9, 'Expected seven arm joints and two fingers'
    ready = [0, -math.pi / 4, 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
    for index, expected in enumerate(ready, 1):
        assert abs(positions[f'fr3_joint{index}'] - expected) < 0.002, positions
    left, right = positions['fr3_finger_joint1'], positions['fr3_finger_joint2']
    assert abs(left - right) < 0.0002, 'Finger mimic differs'
    assert abs(left + right - 0.04) < 0.0002, positions
    print(f'FAKE STATE PASS: 9 joints, ready arm, gripper width {left + right:.8f} m, one publisher')
