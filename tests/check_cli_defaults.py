#!/usr/bin/env python3
"""ROS 통신 없이 CLI의 가상 기본 목표와 실물 명시적 목표 선택을 검사한다.

CLI run 함수만 AST에서 읽고 파라미터/Robot 메서드를 대체한다. 실물 모델이나
노드를 만들지 않는다. 실제 로봇 연결과 제어 성공을 검증하는 테스트가 아니다.
"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

source = Path(__file__).resolve().parents[1] / 'ros2_ws/src/fr3_moveit_demo/scripts/moveit_demo.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
run_function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'run')
scope = {'math': math}
exec(compile(ast.Module(body=[run_function], type_ignores=[]), str(source), 'exec'), scope)


class ParametersOnly:
    """요청 파라미터와 호출 기록만 보관한다. ROS나 실제 Robot은 생성하지 않는다."""

    def __init__(self, backend, values=None):
        self.backend, self.values, self.calls = backend, values or {}, []

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=self.values.get(name, default))

    def connect(self, **values):
        self.calls.append(('connect', values))

    def home(self, **values):
        self.calls.append(('home', values))

    def move_joint(self, **values):
        self.calls.append(('move_joint', values))

    def set_gripper(self, **values):
        self.calls.append(('set_gripper', values))


for parameters in [{}, {'joint': 4}, {'delta': 0.01}, {'joint': 4, 'delta': math.nan}]:
    robot = ParametersOnly('real', parameters)
    try:
        scope['run'](robot)
    except ValueError:
        assert not robot.calls, 'Unspecified real motion reached the server'
    else:
        raise AssertionError('Unspecified real motion was accepted')

for parameters, method in [({'joint': 4, 'delta': 0.01}, 'move_joint'),
                           ({'home': True}, 'home'),
                           ({'gripper_width': 0.04}, 'set_gripper')]:
    robot = ParametersOnly('real', parameters)
    scope['run'](robot)
    assert robot.calls[-1][0] == method and robot.calls[-1][1]['duration'] == 5.0

robot = ParametersOnly('fake')
scope['run'](robot)
assert robot.calls[-1] == ('move_joint', {'joint': 4, 'delta': 0.1, 'duration': 3.0})
print('CLI DEFAULTS PASS: fake defaults, explicit real arm/home/width, no server call for missing real targets.')
