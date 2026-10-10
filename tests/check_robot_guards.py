#!/usr/bin/env python3
"""ROS 및 네트워크 없이 Robot API의 모델/정지/측정 검사를 재현한다.

입력은 설치 스크립트가 받은 공식 vendor 테스트 URDF와 가짜 응답/측정 메시지다.
출력은 일곱 검사 묶음의 PASS 로그이며 잘못된 검사 결과는 AssertionError로 종료한다.
실행: 저장소 루트에서 python3 tests/check_robot_guards.py.

소스 AST에서 필요한 함수와 Robot 클래스만 가져와 rclpy 및 ROS 메시지를 import하지
않는다. Node 생성자를 호출하지 않고 응답과 시각을 대체하므로 DDS 통신, 실제 로봇 연결,
FCI 호출을 시작하지 않는다. 공식 URDF는 setup 이후 vendor에서 읽으며 복사하지 않는다.
이 검사는 입력 거부와 상태 처리 로직의 회귀 검사다. 실제 ROS 통신, FCI 실시간 성능,
실물 정지 또는 물리적 안전을 검증하는 테스트가 아니다.
"""

import ast
import math
from pathlib import Path
import py_compile
import sys
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
api = root / 'ros2_ws/src/fr3_moveit_demo/fr3_moveit_demo/robot.py'
files = [api, api.with_name('__init__.py'), api.parents[1] / 'scripts/moveit_demo.py']
for path in files:
    py_compile.compile(str(path), doraise=True)

# 모듈의 import/ROS context 생성은 실행하지 않고 순수 모델 검사 함수만 추출한다.
tree = ast.parse(api.read_text(encoding='utf-8'))
selected = [node for node in tree.body if isinstance(node, ast.FunctionDef) and
            node.name in ('parse_model', 'check_bounds')]
names = [f'fr3_joint{i}' for i in range(1, 8)]
scope = {'ET': ET, 'math': math, 'JOINT_NAMES': names,
         'FINGER_NAMES': ['fr3_finger_joint1', 'fr3_finger_joint2']}
exec(compile(ast.Module(body=selected, type_ignores=[]), str(api), 'exec'), scope)
official_urdf = root / 'ros2_ws/src/vendor/franka_ros2/franka_hardware/test/fr3.urdf'
if not official_urdf.is_file():
    raise SystemExit('공식 테스트 URDF가 없습니다. 먼저 ./scripts/setup.sh를 실행하세요.')
urdf = ET.parse(official_urdf).getroot()
urdf.find("./ros2_control/hardware/param[@name='robot_ip']").text = '172.16.0.2'
srdf = ET.Element('robot', name='fr3')
group = ET.SubElement(srdf, 'group', name='fr3_arm')
ET.SubElement(group, 'chain', base_link='fr3_link0', tip_link='fr3_link8')
ready = ET.SubElement(srdf, 'group_state', name='ready', group='fr3_arm')
for name, value in zip(names, [0, -math.pi / 4, 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]):
    ET.SubElement(ready, 'joint', name=name, value=str(value))
srdf_text = ET.tostring(srdf, encoding='unicode')
real_text = ET.tostring(urdf, encoding='unicode')
parse = scope['parse_model']
bounds, home, fingers = parse(real_text, srdf_text, backend='real', robot_ip='172.16.0.2')
scope['check_bounds'](home, bounds, 'ready')
assert len(bounds) == 7 and not fingers
for arguments in ({'backend': 'fake'}, {'backend': 'real'},
                  {'backend': 'real', 'robot_ip': '172.16.0.3'}, {'backend': 'other'}):
    try:
        parse(real_text, srdf_text, **arguments)
    except ValueError:
        continue
    raise AssertionError(f'Unsafe model selection accepted: {arguments}')
urdf.find('./ros2_control/hardware/plugin').text = 'mock_components/GenericSystem'
fake_text = ET.tostring(urdf, encoding='unicode')
assert len(parse(fake_text, srdf_text)[0]) == 7
try:
    parse(fake_text, srdf_text, backend='real', robot_ip='172.16.0.2')
except ValueError:
    pass
else:
    raise AssertionError('real accepted Fake Hardware')
print('PASS: Python syntax, official arm model/ready bounds, fake/real plugin separation, required IP, IP mismatch rejection.')

# 실제 Node/액션 서버 대신 응답을 대체해 정지 불확실 상태의 차단 정책을 확인한다.
scope.update({
    'Node': object,
    'GoalStatus': NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6),
    'MoveItErrorCodes': NS(SUCCESS=1, PREEMPTED=-7),
    'CancelGoal': NS(Response=NS(ERROR_NONE=0, ERROR_GOAL_TERMINATED=3)),
    'String': lambda **values: NS(**values),
})
robot_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Robot')
exec(compile(ast.Module(body=[robot_class], type_ignores=[]), str(api), 'exec'), scope)
Robot = scope['Robot']


def robot_for_stop(kind, result=None, pending=False, backend='real'):
    """ROS 생성자 없이 목표와 응답을 대체한 Robot 객체를 반환한다."""
    robot = object.__new__(Robot)
    robot._closed = False
    robot._node_ready = True
    robot._owned_context = NS(ok=lambda: True)
    robot.backend = backend
    robot._motion_uncertain = False
    robot.goal_kind = kind
    robot.pending_goal = object() if pending else None
    robot.active_result = object() if not pending else None
    cancel_token = object()
    handle = NS(accepted=True, cancel_goal_async=lambda: cancel_token,
                get_result_async=lambda: robot.active_result)
    robot.active_goal = None if pending else handle
    robot.events = []
    robot.stop_publisher = NS(publish=lambda message: robot.events.append(message.data))
    robot.get_logger = lambda: NS(info=lambda text: None, warning=lambda text: None)
    robot._request_real_gripper_stop = lambda: True

    def wait(future, timeout, label):
        if future is cancel_token:
            return NS(return_code=0, goals_canceling=[1])
        if result is None:
            raise TimeoutError('Synthetic missing stop result')
        return result

    robot._wait_future = wait
    return robot


uncertain = robot_for_stop('Execution')
uncertain.stop()
assert uncertain._motion_uncertain and uncertain.active_goal is not None
for motion in (uncertain.home, uncertain.move_joint, uncertain.set_gripper):
    try:
        motion()
    except RuntimeError as error:
        assert 'Further motion is blocked' in str(error)
    else:
        raise AssertionError('Motion allowed after unconfirmed real stop')
uncertain.connect = lambda: uncertain
uncertain._fresh_state = lambda: {'fr3_joint1': 0.0}
assert uncertain.get_joint_positions() == {'fr3_joint1': 0.0}

for kind, result in (
        ('Execution', NS(status=5, result=NS(error_code=NS(val=-7)))),
        ('Execution', NS(status=4, result=NS(error_code=NS(val=1)))),
        ('GripperMove', NS(status=5, result=NS(success=False, error=''))),
        ('GripperMove', NS(status=4, result=NS(success=True, error='')))):
    stopped = robot_for_stop(kind, result)
    stopped._motion_uncertain = True
    stopped.stop()
    assert not stopped._motion_uncertain and stopped.active_goal is None
    if kind == 'GripperMove':
        assert not stopped.events

pending = robot_for_stop('GripperMove', pending=True)
pending.stop()
assert pending._motion_uncertain and pending.pending_goal is not None and not pending.events
planning = robot_for_stop('Planning', pending=True)
planning.stop()
assert not planning._motion_uncertain and planning.pending_goal is None and not planning.events
fake = robot_for_stop('Execution', backend='fake')
fake.stop()
assert not fake._motion_uncertain and fake.active_goal is None
print('PASS: no-network stop uncertainty latch, all motion blocked, state queries allowed, terminal stop clearing, gripper-only cancellation, planning/fake unaffected.')

scope['RAW_TOPICS'] = {'arm': '/franka/joint_states', 'hand': '/franka_gripper/joint_states'}
scope['time'] = NS(monotonic=lambda: 100.0)
raw_robot = object.__new__(Robot)
raw_robot.backend = 'real'
raw_robot._raw_states = {}
raw_robot._raw_state_counts = {'arm': 0, 'hand': 0}
raw_robot.arm_bounds, raw_robot.finger_bounds = bounds, []
raw_robot.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000_000))
raw_robot.count_publishers = lambda topic: 1 if topic == '/franka/joint_states' else 0
arm_message = NS(name=names, position=home, header=NS(stamp=NS(sec=1000, nanosec=0)))
raw_robot._on_raw_joint_state('arm', arm_message)
raw_robot._on_raw_joint_state('arm', arm_message)
assert raw_robot._raw_state_counts['arm'] == 1
raw_robot._check_real_raw_state()  # 팔만 있는 모델은 그리퍼 측정을 요구하지 않는다.


def raw_check_must_fail(label):
    """잘못된 실물 측정을 거부하는지 확인한다. 거부하지 않으면 AssertionError다."""
    try:
        raw_robot._check_real_raw_state()
    except (RuntimeError, ValueError):
        return
    raise AssertionError(f'Unsafe raw state accepted: {label}')


raw_robot.count_publishers = lambda topic: 2
raw_check_must_fail('duplicate raw publishers')
raw_robot.count_publishers = lambda topic: 1
raw_robot.finger_bounds = [(0.0, 0.04), (0.0, 0.04)]
raw_check_must_fail('missing required hand measurements')
hand_message = NS(name=scope['FINGER_NAMES'], position=[0.02, 0.02],
                  header=NS(stamp=NS(sec=1000, nanosec=0)))
raw_robot._on_raw_joint_state('hand', hand_message)
raw_robot._check_real_raw_state()
raw_robot.latest_state = NS(header=NS(stamp=NS(sec=1000, nanosec=0)))
raw_robot._raw_states['arm']['stamp_ns'] = 998_000_000_000
raw_check_must_fail('new aggregate timestamp hides stale arm measurement')
raw_robot._raw_states['arm']['stamp_ns'] = 1_000_000_000_000
raw_robot._raw_states['arm']['received_at'] = 98.0
raw_check_must_fail('recent stamp hides old monotonic receive time')
raw_robot._raw_states['arm']['received_at'] = 100.0
raw_robot._raw_states['hand']['positions'][scope['FINGER_NAMES'][1]] = 0.03
raw_check_must_fail('measured hand mimic mismatch')
fake.backend = 'fake'
fake._check_real_raw_state()  # fake에서는 raw 상태 필드를 요구하지 않는다.
print('PASS: no-network raw measurement liveness, duplicate timestamp rejection, arm-only support, required hand state, publisher uniqueness, stale raw/receive detection, measured mimic, fake unaffected.')

# raw 측정이 멈춘 뒤 집계 메시지만 갱신되어도 fresh 상태가 되어서는 안 된다.
raw_robot.finger_bounds = []
raw_robot.timeout, raw_robot.state_count = 0.3, 0
raw_robot._owned_context = NS(ok=lambda: True)
clock = [100.0]
scope['time'] = NS(monotonic=lambda: clock[0])


def aggregate_only_spin(timeout_sec):
    """시각과 집계 수신만 갱신해 raw 측정 중단을 재현한다."""
    clock[0] += max(0.001, timeout_sec)
    raw_robot.state_count += 1


raw_robot._owned_executor = NS(spin_once=aggregate_only_spin)
try:
    raw_robot._fresh_state()
except TimeoutError as error:
    assert 'Cached aggregate state is refused' in str(error)
else:
    raise AssertionError('Aggregate-only updates accepted as fresh real measurements')

raw_robot._raw_states['arm']['stamp_ns'] = 998_000_000_000
sent = []
client = NS(send_goal_async=lambda goal: sent.append(goal))
for kind in ('Execution', 'GripperMove'):
    try:
        raw_robot._action_result(client, object(), 1.0, kind)
    except RuntimeError as error:
        assert 'stale' in str(error)
    else:
        raise AssertionError('Execution accepted with stale raw measurements')
assert not sent
print('PASS: aggregate-only freshness timeout and stale raw arm/gripper rejection before action goal transmission.')

raw_robot.required_names = names
new_arm = NS(name=names, position=home, header=NS(stamp=NS(sec=1000, nanosec=1_000_000)))
cached_aggregate = NS(name=names, position=[0.0] * 7, header=NS(stamp=NS(sec=1000, nanosec=0)))


def live_raw_spin(timeout_sec):
    """새 raw 측정과 오래된 집계 위치를 함께 공급해 반환값의 출처를 확인한다."""
    clock[0] += 0.001
    raw_robot._on_raw_joint_state('arm', new_arm)
    raw_robot._on_joint_state(cached_aggregate)


raw_robot._owned_executor = NS(spin_once=live_raw_spin)
assert raw_robot._fresh_state() == dict(zip(names, home))
print('PASS: fresh real state returns measured raw positions rather than aggregate cache values.')


def robot_with_action_result(kind, response, backend='real'):
    """액션 수락/결과/취소 응답을 대체한다. 실제 클라이언트는 만들지 않는다."""
    robot = robot_for_stop(kind, backend=backend)
    robot.timeout = 1.0
    robot._check_real_raw_state = lambda: None
    handle = NS(accepted=True, get_result_async=lambda: object(), cancel_goal_async=lambda: object())
    robot.client = NS(send_goal_async=lambda goal: object())

    def response_for_label(future, timeout, label):
        if label.endswith('goal response'):
            return handle
        if label == 'Cancel goal':
            return NS(return_code=3, goals_canceling=[])
        return response

    robot._wait_future = response_for_label
    try:
        robot._action_result(robot.client, object(), 1.0, kind)
    except RuntimeError:
        pass
    else:
        raise AssertionError('Failed action result accepted as successful')
    return robot


for error_code in (-4, -6):  # MoveIt CONTROL_FAILED / TIMED_OUT는 정지 확인이 아니다.
    arm_failure = robot_with_action_result('Execution', NS(status=6, result=NS(error_code=NS(val=error_code))))
    assert arm_failure._motion_uncertain and arm_failure.active_goal is not None
    arm_failure.stop()
    assert arm_failure._motion_uncertain and arm_failure.active_goal is not None

for status in (6, 4):  # ABORTED 및 모순된 SUCCEEDED+success=False도 모두 실패다.
    gripper_failure = robot_with_action_result('GripperMove', NS(status=status, result=NS(success=False, error='Synthetic failure')))
    assert gripper_failure._motion_uncertain and gripper_failure.active_goal is not None
    gripper_failure._request_real_gripper_stop = lambda: False
    gripper_failure.stop()
    assert gripper_failure._motion_uncertain and gripper_failure.active_goal is not None
    gripper_failure._request_real_gripper_stop = lambda: True
    gripper_failure.stop()
    assert not gripper_failure._motion_uncertain and gripper_failure.active_goal is None
    assert not gripper_failure.events

unknown_gripper = robot_for_stop('GripperMove', pending=True)
unknown_gripper._request_real_gripper_stop = lambda: True
unknown_gripper.stop()
assert unknown_gripper._motion_uncertain and unknown_gripper.pending_goal is not None
failed_fake = robot_with_action_result('Execution', NS(status=6, result=NS(error_code=NS(val=-4))), backend='fake')
assert not failed_fake._motion_uncertain and failed_fake.active_goal is None
failed_planning = robot_with_action_result('Planning', NS(status=6, result=NS(error_code=NS(val=-1))))
assert not failed_planning._motion_uncertain and failed_planning.active_goal is None
print('PASS: failed real arm/gripper result retention, ABORTED/CONTROL_FAILED/TIMED_OUT latch, failed success flag, gripper stop failure/success, pending uncertainty, fake/planning unaffected.')

# Trigger의 import 대상도 대체해 실제 ROS 서비스를 생성하거나 호출하지 않는다.
fake_service_module = ModuleType('std_srvs.srv')
fake_service_module.Trigger = NS(Request=lambda: object())
with patch.dict(sys.modules, {'std_srvs': ModuleType('std_srvs'), 'std_srvs.srv': fake_service_module}):
    service_robot = robot_for_stop('GripperMove')
    requested_services = []
    service = NS(wait_for_service=lambda timeout_sec: True, call_async=lambda request: object())
    service_robot.gripper_stop_client = None

    def create_service_client(service_type, name):
        requested_services.append(name)
        return service

    service_robot.create_client = create_service_client
    service_robot._wait_future = lambda future, timeout, label: NS(success=True, message='')
    assert Robot._request_real_gripper_stop(service_robot)
    assert requested_services == ['/franka_gripper/stop']
    service_robot._wait_future = lambda future, timeout, label: NS(success=False, message='Synthetic failure')
    assert not Robot._request_real_gripper_stop(service_robot)
    service.wait_for_service = lambda timeout_sec: False
    assert not Robot._request_real_gripper_stop(service_robot)
    service.wait_for_service = lambda timeout_sec: True

    def service_timeout(future, timeout, label):
        raise TimeoutError('Synthetic service timeout')

    service_robot._wait_future = service_timeout
    assert not Robot._request_real_gripper_stop(service_robot)

    destroyed = []
    service_robot.goal_kind = None
    service_robot.active_goal = None
    service_robot._action_clients = []
    service_robot._owned_executor = None
    service_robot._owned_context.try_shutdown = lambda: None
    service_robot.destroy_client = lambda client: destroyed.append(client)
    service_robot.destroy_node = lambda: None
    service_robot.close()
    assert destroyed == [service] and service_robot.gripper_stop_client is None
print('PASS: lazy official gripper stop client, substituted Trigger success/failure/unavailable/timeout, stop client cleanup without ROS or FCI.')
