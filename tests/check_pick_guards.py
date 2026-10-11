#!/usr/bin/env python3
"""물체/깊이 입력, 부분 경로, 집기 실패와 GUI 중복 클릭을 연결 없이 검사한다."""

import importlib.util
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
from unittest.mock import patch

from geometry_msgs.msg import Quaternion
from fr3_moveit_demo import Robot
from fr3_moveit_demo.pick_place import PickAndReturn, CancellableRobot, downward_pose, validate_selection


def main():
    """위험한 후속 실행이 차단되는지 확인한다. ROS 노드/FCI를 생성하지 않는다."""
    assert validate_selection([.45, 0, .25], [.04, .03, .05], .20, .1, .05, 20) == [.45, 0, .225]
    for kwargs in ({'top': [.45, 0, float('nan')]}, {'table_z': .15}, {'size': [.04, .08, .05]},
                   {'lift': .2}, {'force': 0}, {'top': [1., 0, .25]}):
        parameters = dict(top=[.45, 0, .25], size=[.04, .03, .05], table_z=.20,
                          clearance=.10, lift=.05, force=20.)
        parameters.update(kwargs)
        try:
            validate_selection(**parameters)
        except ValueError:
            pass
        else:
            raise AssertionError(f'Invalid object accepted: {kwargs}')
    quaternion = downward_pose([.45, 0., .25]).orientation
    assert quaternion == Quaternion(x=1., y=0., z=0., w=0.)
    print('PASS: top/depth/dimensions/table/workspace/force limits and unit quaternion')

    task = object.__new__(PickAndReturn)
    sent = []
    robot = NS(_ensure_motion_allowed=lambda: None, _fresh_state=lambda: {},
               required_names=[], stop=lambda: sent.append('stop'))
    task.robot = robot
    task.call = lambda *args: NS(error_code=NS(val=1), fraction=.95)
    try:
        task.move(downward_pose([.45, 0, .225]), cartesian=True)
    except RuntimeError as error:
        assert 'incomplete' in str(error)
    else:
        raise AssertionError('Partial Cartesian trajectory executed')
    assert sent == ['stop']
    print('PASS: incomplete Cartesian path rejected before execution')

    events = []
    task.robot = NS(_fresh_state=lambda: {},
                    set_gripper=lambda *a, **k: events.append('open'),
                    grasp=lambda *a, **k: (_ for _ in ()).throw(RuntimeError('No object grasped')))
    task.pose = lambda *args: None
    task.prepare_scene = lambda *args: None
    task.mark = lambda stage: events.append(stage)
    task.move = lambda *args, **kwargs: events.append('move')
    task.attach = lambda *args: events.append('attach')
    task.duration = 1
    try:
        task.run([.45, 0, .25], [.04, .03, .05], .20)
    except RuntimeError as error:
        assert str(error) == 'No object grasped'
    else:
        raise AssertionError('Grasp failure was ignored')
    assert events == ['initial', 'open', 'move', 'approach', 'move', 'at_object']
    print('PASS: grasp failure blocks attachment/lift/release/automatic return')

    cancelled = object.__new__(CancellableRobot)
    cancelled.cancel_event = threading.Event()
    cancelled.cancel_event.set()
    try:
        cancelled.check_cancel()
    except KeyboardInterrupt:
        pass
    else:
        raise AssertionError('Cancellation was ignored')
    with patch.object(Robot, 'stop', lambda self: self.check_cancel()):
        cancelled.stop()  # 중단 이벤트가 stop 응답 대기를 방해하면 실패한다.
    assert cancelled.cancel_event.is_set()
    print('PASS: GUI cancellation is honored while stop acknowledgement can still finish')

    examples = Path(__file__).resolve().parents[1] / 'ros2_ws/src/fr3_moveit_demo/examples'
    sys.path.insert(0, str(examples))
    spec = importlib.util.spec_from_file_location('pick_gui', examples / 'pick_and_return.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    select = object.__new__(module.SelectObject)
    select.busy = True
    select.project = lambda *args: (_ for _ in ()).throw(AssertionError('Extra click projected while moving'))
    import cv2
    select.on_click(cv2.EVENT_LBUTTONDOWN, 320, 240, 0, None)
    print('PASS: additional clicks while a task is running are refused')

    # 생성자/ActionClient를 대체해 실제 Grasp 메시지와 측정 폭 검사를 확인한다.
    from fr3_moveit_demo.robot import FINGER_NAMES
    real = object.__new__(Robot)
    real._closed, real._motion_uncertain, real.backend = False, False, 'real'
    real._owned_context = NS(ok=lambda: True)
    real.connect = lambda: real
    real.finger_bounds, real.timeout = [(0., .04)] * 2, 1.
    real.gripper_grasp_client, real._action_clients = None, []
    real.get_logger = lambda: NS(info=lambda message: None)
    stopped, sent = [], []
    real.stop = lambda: stopped.append(True)
    real._fresh_state = lambda: dict(zip(FINGER_NAMES, [.025, .025]))

    def success(client, goal, timeout, label):
        sent.append((goal, label))
        real._fresh_state = lambda: dict(zip(FINGER_NAMES, [.015, .015]))

    real._action_result = success
    with patch('fr3_moveit_demo.robot.ActionClient', return_value=NS(wait_for_server=lambda **k: True)):
        real.grasp(.03, 20., speed=.01, epsilon=.003)
    goal, label = sent[0]
    assert label == 'GripperGrasp' and (goal.width, goal.force, goal.speed) == (.03, 20., .01)
    assert goal.epsilon.inner == goal.epsilon.outer == .003 and not stopped
    real._fresh_state = lambda: dict(zip(FINGER_NAMES, [.025, .025]))
    real._action_result = lambda *a: (_ for _ in ()).throw(RuntimeError('Synthetic grasp failed'))
    try:
        real.grasp(.03, 20.)
    except RuntimeError:
        pass
    else:
        raise AssertionError('Real Grasp failure ignored')
    assert stopped == [True]
    print('PASS: official Grasp goal/force/epsilon, measured width and failure stop (no connection)')


if __name__ == '__main__':
    main()
