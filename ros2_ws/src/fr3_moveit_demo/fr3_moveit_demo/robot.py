"""공식 FR3 팔과 가상 그리퍼의 공통 Python 제어 API.

라이브러리: rclpy, moveit_msgs, sensor_msgs와 Python 표준 XML 파서.
입력: 메서드의 관절 번호, 상대 이동량(rad), 전체 그리퍼 폭(m), 최소 이동 시간(s).
출력: START/GOAL/FINAL 로그와 검증한 실제 관절 위치 dict, 실패 시 예외.
흐름: 서버 모델 확인 -> 최신 JointState -> 관절 제한 검사 -> 계획 -> 실행 -> 검증.
fake_moveit.launch.py의 MoveIt 서버를 CLI와 사용자 예제에서 함께 사용한다.
Humble에서는 MoveGroupInterface의 Python 래퍼 대신 공식 MoveGroup 액션과
ExecuteTrajectory 액션을 사용한다. 자체 IK나 FCI 연결을 구현하지 않는다.
모델은 첫 연결에서 캐시하고 동작마다 새 상태를 받는다. Robot의 독립 ROS context와
executor는 with 블록 동안 유지되며, close()는 다른 노드의 context를 종료하지 않는다.
"""

import math
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Duration
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from sensor_msgs.msg import JointState
from std_msgs.msg import String


JOINT_NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
FINGER_NAMES = ['fr3_finger_joint1', 'fr3_finger_joint2']


def parse_fake_model(urdf_text, srdf_text, gripper_mode=False):
    """팔 제한(rad), ready 자세, 손 제한(m)을 반환한다. 실물/다른 모델은 예외로 거부한다.

    모델을 새로 정의하지 않고 실행 중인 서버와 같은 공식 모델에서 값을 읽는다.
    fr3_arm의 공식 chain을 URDF에서 따라가며 일곱 관절 이름도 확인한다.
    gripper_mode이면 fr3_hand의 독립 관절과 URDF mimic 관계도 검사한다.
    """
    urdf, srdf = ET.fromstring(urdf_text), ET.fromstring(srdf_text)
    plugins = [plugin.text.strip() if plugin.text else ''
               for plugin in urdf.findall('./ros2_control/hardware/plugin')]
    if not plugins or any(plugin != 'mock_components/GenericSystem' for plugin in plugins):
        raise ValueError('This demo requires GenericSystem Fake Hardware; real hardware is refused.')
    group = srdf.find("./group[@name='fr3_arm']")
    if group is None or len(group.findall('chain')) != 1:
        raise ValueError('Official fr3_arm chain was not loaded.')
    chain = group.find('chain')
    joints_by_child = {joint.find('child').get('link'): joint for joint in urdf.findall('joint')}
    link, names, visited = chain.get('tip_link'), [], set()
    while link != chain.get('base_link'):
        if link in visited or link not in joints_by_child:
            raise ValueError('SRDF chain does not match the URDF.')
        visited.add(link)
        joint = joints_by_child[link]
        if joint.get('type') != 'fixed':
            names.append(joint.get('name'))
        link = joint.find('parent').get('link')
    if list(reversed(names)) != JOINT_NAMES:
        raise ValueError('Expected fr3_joint1..7 in the official fr3_arm group.')
    bounds = []
    for name in JOINT_NAMES:
        joint = urdf.find(f"./joint[@name='{name}']")
        limit = joint.find('limit')
        if joint.get('type') != 'revolute' or limit is None:
            raise ValueError(f'Position bounds are missing: {name}')
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f'Invalid URDF bounds: {name}')
        bounds.append((lower, upper))
    ready_state = srdf.find("./group_state[@name='ready'][@group='fr3_arm']")
    if ready_state is None:
        raise ValueError('Official SRDF ready named state is missing.')
    ready = {joint.get('name'): float(joint.get('value')) for joint in ready_state.findall('joint')}
    if set(ready) != set(JOINT_NAMES):
        raise ValueError('Official SRDF ready named state must contain seven joints.')
    finger_bounds = []
    if gripper_mode:
        hand = srdf.find("./group[@name='fr3_hand']")
        if hand is None or [joint.get('name') for joint in hand.findall('joint')] != [FINGER_NAMES[0]]:
            raise ValueError('Official fr3_hand group with one independent finger joint is missing.')
        if {link.get('name') for link in hand.findall('link')} != {
                'fr3_hand', 'fr3_leftfinger', 'fr3_rightfinger'}:
            raise ValueError('Official Franka Hand links were not loaded.')
        for name in FINGER_NAMES:
            joint = urdf.find(f"./joint[@name='{name}']")
            if joint is None or joint.get('type') != 'prismatic' or joint.find('limit') is None:
                raise ValueError(f'Official prismatic finger joint is missing: {name}')
            limit = joint.find('limit')
            lower, upper = float(limit.get('lower')), float(limit.get('upper'))
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError(f'Invalid finger URDF bounds: {name}')
            finger_bounds.append((lower, upper))
        mimic = urdf.find(f"./joint[@name='{FINGER_NAMES[1]}']/mimic")
        if (mimic is None or mimic.get('joint') != FINGER_NAMES[0] or
                float(mimic.get('multiplier', '1')) != 1.0 or float(mimic.get('offset', '0')) != 0.0):
            raise ValueError('Official finger2 mimic relation is missing.')
    return bounds, [ready[name] for name in JOINT_NAMES], finger_bounds


def check_bounds(positions, bounds, label, names=JOINT_NAMES):
    """팔(rad) 또는 손(m) 위치를 URDF 제한과 비교한다. 현재 상태/목표 검사 실패는 예외다."""
    if len(positions) != len(names) or len(bounds) != len(names):
        raise ValueError('Joint positions and bounds must match the selected joint names.')
    for name, position, (lower, upper) in zip(names, positions, bounds):
        if not math.isfinite(position) or not lower <= position <= upper:
            raise ValueError(f'{label} exceeds official joint bounds: {name}')


def slow_plan(trajectory, requested, names=JOINT_NAMES):
    """MoveIt 경로를 유지하며 최소 requested 초로 늦춘다. 실행 시간(s)을 반환한다.

    시간은 scale배, 속도는 1/scale배, 가속도는 1/scale²배로 바꾼다.
    MoveIt이 계획한 궤적을 더 빠르게 실행하지 않는다. 비어 있거나 잘못된 계획은 거부한다.
    이미 목표 근처인 경우의 단일 정지점은 0을 반환한다. 호출자는 실행하지 않고
    새 실제 상태로 목표 도달을 확인해야 한다.
    """
    joints = trajectory.joint_trajectory
    if joints.joint_names != names or not joints.points or trajectory.multi_dof_joint_trajectory.points:
        raise ValueError('Planning returned an empty or unexpected FR3 joint trajectory.')
    previous_ns = -1
    for point in joints.points:
        stamp = point.time_from_start
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp.sec < 0 or stamp.nanosec >= 1_000_000_000 or stamp_ns <= previous_ns:
            raise ValueError('Planning returned invalid trajectory times.')
        count = len(names)
        if len(point.positions) != count or len(point.velocities) not in (0, count) or len(point.accelerations) not in (0, count):
            raise ValueError('Planning returned incomplete trajectory points.')
        if not all(math.isfinite(value) for value in [*point.positions, *point.velocities, *point.accelerations]):
            raise ValueError('Planning returned nonfinite trajectory values.')
        previous_ns = stamp_ns
    original = previous_ns / 1_000_000_000
    if original == 0 and len(joints.points) == 1:
        point = joints.points[0]
        if any(abs(value) > 1e-12 for value in [*point.velocities, *point.accelerations]):
            raise ValueError('A stationary plan must have zero velocity and acceleration.')
        return 0.0
    if original <= 0:
        raise ValueError('Planning returned a trajectory without a positive duration.')
    scale = max(1.0, requested / original)
    for point in joints.points:
        stamp_ns = round((point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec) * scale)
        point.time_from_start = Duration(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
        point.velocities = [velocity / scale for velocity in point.velocities]
        point.accelerations = [acceleration / (scale * scale) for acceleration in point.accelerations]
    return original * scale


class Robot(Node):
    """동기 메서드로 Fake Hardware를 조작하는 ROS 노드. 한 호출 스레드에서 사용한다."""

    def __init__(self, args=None, wait_timeout=30.0):
        """독립 ROS 통신 자원을 만든다. 서버 연결은 최초 API 호출 시 수행한다.

        args는 rclpy의 ROS 인자 목록이며 None이면 sys.argv를 사용한다.
        wait_timeout은 서버/현재 상태 응답 대기 한도(s), 범위는 0초 초과 300초 이하다.
        생성 중 오류가 나면 이미 만든 자원도 정리한 뒤 예외를 전달한다.
        """
        self._closed = False
        self._node_ready = False
        self._owned_context = Context()
        self._owned_executor = None
        self._action_clients = []
        self.latest_state = None
        self.state_count = 0
        self.active_goal = None
        self.pending_goal = None
        self.active_result = None
        self.goal_kind = None
        self._connected = False
        self.required_names = JOINT_NAMES[:]
        self.arm_bounds = []
        self.ready = []
        self.finger_bounds = []
        try:
            self.timeout = self._positive_seconds(wait_timeout, 'wait_timeout', 300.0)
            # Context.init을 직접 호출해 외부 노드의 전역 context와 신호 정책을 유지한다.
            # 종료 후 이 context를 재사용하지 않고 새 Robot을 생성한다.
            self._owned_context.init(args=args)
            super().__init__('fr3_moveit_python_demo', context=self._owned_context)
            self._node_ready = True
            self._owned_executor = SingleThreadedExecutor(context=self._owned_context)
            self._owned_executor.add_node(self)
            self.subscription = self.create_subscription(
                JointState, '/joint_states', self._on_joint_state, qos_profile_sensor_data)
            self.parameters_client = self.create_client(GetParameters, '/move_group/get_parameters')
            self.planner = ActionClient(self, MoveGroup, '/move_action')
            self._action_clients.append(self.planner)
            self.execution_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
            self._action_clients.append(self.execution_client)
            self.stop_publisher = self.create_publisher(
                String, '/trajectory_execution_event', qos_profile_services_default)
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _positive_seconds(value, label, maximum):
        """유한한 양수 시간(s)을 반환한다. 잘못된 입력은 ROS 요청 전에 ValueError다."""
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= maximum:
            raise ValueError(f'{label} must be finite and satisfy 0 < {label} <= {maximum}.')
        return float(value)

    def _ensure_open(self):
        """닫힌 Robot의 재사용을 거부한다. 새 연결이 필요하면 새 Robot을 생성한다."""
        if self._closed or not self._owned_context.ok():
            raise RuntimeError('Robot is closed. Create a new Robot context to reconnect.')

    def __enter__(self):
        """with 안에서 같은 ROS 연결을 재사용한다. 서버를 기다리지 않고 self를 반환한다."""
        self._ensure_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """정상/실패/Ctrl+C 모두 자원을 정리한다. 원래 예외를 호출자에게 전달한다."""
        self.close()
        return False

    def connect(self, wait_timeout=None):
        """서버의 공식 모델과 Fake Hardware를 한 번 확인하고 self를 반환한다.

        wait_timeout을 지정하면 이후 요청의 대기 한도(s)로 사용한다.
        모델/서버가 없거나 실제 하드웨어 모델이면 예외다. 상태는 각 메서드에서 새로 받는다.
        """
        self._ensure_open()
        if wait_timeout is not None:
            self.timeout = self._positive_seconds(wait_timeout, 'wait_timeout', 300.0)
        if self._connected:
            return self
        if not self.parameters_client.wait_for_service(timeout_sec=self.timeout):
            raise TimeoutError('/move_group parameter service timeout. Start Fake Hardware first.')
        request = GetParameters.Request(names=['robot_description', 'robot_description_semantic'])
        response = self._wait_future(self.parameters_client.call_async(request), self.timeout, 'URDF/SRDF')
        if len(response.values) != 2 or any(value.type != ParameterType.PARAMETER_STRING for value in response.values):
            raise ValueError('move_group did not provide URDF and SRDF strings.')
        urdf_text, srdf_text = [value.string_value for value in response.values]
        has_hand = ET.fromstring(srdf_text).find("./group[@name='fr3_hand']") is not None
        self.arm_bounds, self.ready, self.finger_bounds = parse_fake_model(
            urdf_text, srdf_text, gripper_mode=has_hand)
        check_bounds(self.ready, self.arm_bounds, 'Official ready state')
        # 손이 있으면 팔 메서드도 양쪽 손가락을 포함한 하나의 최신 메시지를 확인한다.
        self.required_names = JOINT_NAMES + FINGER_NAMES if has_hand else JOINT_NAMES[:]
        for client in self._action_clients:
            if not client.wait_for_server(timeout_sec=self.timeout):
                raise TimeoutError('MoveIt action server timeout. Start Fake Hardware first.')
        self._connected = True
        self.get_logger().info('Python Robot API connected. GenericSystem Fake Hardware only. No FCI connection.')
        return self

    def _on_joint_state(self, message):
        """새 JointState의 이름, 유한값, 시간(s)을 확인한다. 오래되거나 잘못된 상태는 버린다.

        name과 position은 같은 index의 관절을 뜻한다. 배열 순서를 가정하지 않고
        이름으로 매핑한다. best effort 구독은 일반적인 두 발행 QoS와 호환된다.
        executor가 응답을 기다리는 동안 이 콜백도 수행하므로 연속 동작에서 상태가 갱신된다.
        """
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            return
        positions = dict(zip(message.name, message.position))
        if any(name not in positions or not math.isfinite(positions[name]) for name in self.required_names):
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000
        if stamp_ns <= 0 or not -0.1 <= age <= 1.0:
            return
        self.latest_state = message
        self.state_count += 1

    def _wait_future(self, future, timeout, label):
        """독립 executor로 timeout 초 동안 응답과 구독을 처리한다. 응답을 반환하고 미수신은 예외다.

        콜백 안에서는 blocking 요청을 하지 않는다. 외부 노드의 전역 executor를
        사용하지 않으므로 이 API의 종료가 다른 ROS 통신을 종료하지 않는다.
        """
        self._owned_executor.spin_until_future_complete(future, timeout_sec=timeout)
        if not future.done() or future.cancelled():
            raise TimeoutError(f'{label} timeout. Motion is not considered successful.')
        response = future.result()
        if response is None:
            raise RuntimeError(f'{label} returned no response.')
        return response

    def _fresh_state(self):
        """다음 새 JointState를 기다리고 실제 전체 관절 dict를 반환한다.

        팔 위치는 rad, 손가락은 m다. 대기 한도는 timeout(s)이며 발행자 중복,
        현재 관절 제한 위반, 손가락 mimic 불일치 또는 미수신은 예외다.
        """
        marker, deadline = self.state_count, time.monotonic() + self.timeout
        while self._owned_context.ok() and self.state_count == marker and time.monotonic() < deadline:
            self._owned_executor.spin_once(timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if self.state_count == marker:
            raise TimeoutError('Current Robot State not received for all required FR3 joints. Motion cancelled.')
        if self.count_publishers('/joint_states') != 1:
            raise RuntimeError('Exactly one /joint_states publisher required; stop the other demo.')
        received = dict(zip(self.latest_state.name, self.latest_state.position))
        positions = {name: received[name] for name in self.required_names}
        check_bounds([positions[name] for name in JOINT_NAMES], self.arm_bounds, 'Current arm state')
        if self.finger_bounds:
            check_bounds([positions[name] for name in FINGER_NAMES], self.finger_bounds, 'Current finger state', FINGER_NAMES)
            if abs(positions[FINGER_NAMES[0]] - positions[FINGER_NAMES[1]]) > 0.0002:
                raise ValueError('Received finger states do not satisfy the official mimic relation.')
        return positions

    def get_joint_positions(self):
        """이 호출 이후 받은 실제 관절 위치 dict를 반환한다. 팔 rad, 손가락 m다.

        첫 호출은 자동 연결한다. 상태/모델 검사는 수행하지만 이동 목표는 보내지 않는다.
        손 모델이 있으면 9개, 팔만 있으면 7개 관절이며 미수신을 성공으로 처리하지 않는다.
        """
        self.connect()
        return self._fresh_state()

    def home(self, duration=3.0):
        """팔을 공식 SRDF ready로 복귀시키고 실제 전체 최종 상태 dict를 반환한다.

        duration은 최소 이동 시간(s)이다. 손가락 목표는 변경하지 않는다.
        모델/상태/제한/계획/실행 실패는 예외이며 진행 중인 본인 목표는 정지 요청한다.
        """
        duration = self._positive_seconds(duration, 'duration', 3600.0)
        initial = self.get_joint_positions()
        self.get_logger().info('HOME: official SRDF named state ready')
        return self._execute_motion(self.ready[:], duration, initial)

    def move_joint(self, joint=4, delta=0.1, duration=3.0):
        """관절 joint(1~7)을 직전 실제 상태에서 delta rad만큼 이동한다.

        duration은 최소 시간(s), 반환값은 실제 전체 최종 상태 dict다.
        잘못된 입력과 공식 관절 제한 밖 목표는 계획 요청 전에 예외로 거부한다.
        """
        duration = self._positive_seconds(duration, 'duration', 3600.0)
        if isinstance(joint, bool) or not isinstance(joint, int) or not 1 <= joint <= 7:
            raise ValueError('joint must be an integer in 1..7.')
        if isinstance(delta, bool) or not isinstance(delta, (int, float)) or not math.isfinite(delta):
            raise ValueError('delta must be finite (rad).')
        initial = self.get_joint_positions()
        goal = [initial[name] for name in JOINT_NAMES]
        goal[joint - 1] += delta
        return self._execute_motion(goal, duration, initial)

    def set_gripper(self, width=0.04, duration=3.0):
        """전체 벌림 폭 width(m)를 계획하고 실제 전체 최종 상태 dict를 반환한다.

        duration은 최소 시간(s), 공식 손가락 한계는 일반적으로 각각 0..0.04m다.
        각 손가락 목표는 width/2이며 두 제한 모두 확인한다. 손 없는 모델은 예외다.
        """
        duration = self._positive_seconds(duration, 'duration', 3600.0)
        if isinstance(width, bool) or not isinstance(width, (int, float)) or not math.isfinite(width) or width < 0:
            raise ValueError('width must be finite and nonnegative (m).')
        self.connect()
        if not self.finger_bounds:
            raise ValueError('Official fr3_hand group is not loaded. Start the stack with the Franka Hand model.')
        check_bounds([width / 2.0, width / 2.0], self.finger_bounds, 'Gripper width', FINGER_NAMES)
        return self._execute_motion([width / 2.0], duration, self._fresh_state(), gripper_mode=True)

    def _print_positions(self, label, positions, names, gripper_mode=False):
        """선택한 목표/실제 위치를 rad 또는 m로 출력한다. 손은 실제 양쪽 값과 폭도 표시한다."""
        unit = 'm' if gripper_mode else 'rad'
        for name in names:
            self.get_logger().info(f'{label} {name} = {positions[name]:.6f} {unit}')
        if gripper_mode:
            finger2 = positions[FINGER_NAMES[1]]
            width = positions[FINGER_NAMES[0]] + finger2
            self.get_logger().info(f'{label} {FINGER_NAMES[1]} = {finger2:.6f} m')
            self.get_logger().info(f'{label} gripper_width = {width:.6f} m')

    def _execute_motion(self, goal_positions, duration, initial, gripper_mode=False):
        """검사된 시작 상태에서 목표를 계획/실행/검증한다. 실제 최종 dict를 반환한다.

        goal_positions는 독립 관절 순서의 rad 또는 m, duration은 최소 시간(s)이다.
        손의 mimic 관절은 상태에 포함되지만 목표 constraint/trajectory에는 포함하지 않는다.
        예외 시 본인 목표의 정지/취소를 요청하고 호출자가 다음 동작을 결정하게 한다.
        """
        names = [FINGER_NAMES[0]] if gripper_mode else JOINT_NAMES
        bounds = [self.finger_bounds[0]] if gripper_mode else self.arm_bounds
        tolerance, unit = (0.0002, 'm') if gripper_mode else (0.002, 'rad')
        group_name = 'fr3_hand' if gripper_mode else 'fr3_arm'
        check_bounds(goal_positions, bounds, 'Target', names)
        goals = dict(initial)
        goals.update(zip(names, goal_positions))
        if gripper_mode:
            goals[FINGER_NAMES[1]] = goal_positions[0]
        self.get_logger().info(f'Planning group: {group_name}')
        self._print_positions('START', initial, names, gripper_mode)
        for name, (lower, upper) in zip(names, bounds):
            self.get_logger().info(f'LIMIT {name} [{lower:.4f}, {upper:.4f}] {unit}')
        self._print_positions('GOAL', goals, names, gripper_mode)
        if all(abs(initial[name] - goals[name]) < 1e-9 for name in names):
            self.get_logger().info('Already at target. No trajectory needed.')
            self._print_positions('FINAL', initial, names, gripper_mode)
            return initial
        try:
            # plan_only=True로 계산만 요청한다. 서버 기본값 False는 실행까지 요청하므로
            # 이 API에서는 계획 결과 확인 뒤 별도의 ExecuteTrajectory로 실행한다.
            plan_goal = MoveGroup.Goal()
            motion = plan_goal.request
            motion.group_name = group_name
            motion.num_planning_attempts = 1
            motion.allowed_planning_time = 10.0
            motion.max_velocity_scaling_factor = 0.1
            motion.max_acceleration_scaling_factor = 0.1
            motion.start_state.joint_state = JointState(
                header=self.latest_state.header, name=self.required_names,
                position=[initial[name] for name in self.required_names])
            motion.start_state.is_diff = True
            motion.goal_constraints = [Constraints(joint_constraints=[
                JointConstraint(joint_name=name, position=position,
                                tolerance_above=1e-5 if gripper_mode else 1e-4,
                                tolerance_below=1e-5 if gripper_mode else 1e-4, weight=1.0)
                for name, position in zip(names, goal_positions)])]
            plan_goal.planning_options.plan_only = True
            plan_goal.planning_options.planning_scene_diff.is_diff = True
            plan_goal.planning_options.planning_scene_diff.robot_state.is_diff = True
            planned = self._action_result(self.planner, plan_goal, self.timeout + 10.0, 'Planning')
            execution_duration = slow_plan(planned.planned_trajectory, duration, names)
            if execution_duration == 0:
                # MoveIt의 목표 허용 오차 때문에 ready 근처의 재호출은 한 정지점만
                # 반환할 수 있다. 0초 궤적을 컨트롤러에 보내지 않고 현재 상태를 검증한다.
                point = planned.planned_trajectory.joint_trajectory.points[0]
                if any(abs(position - initial[name]) > tolerance
                       for name, position in zip(names, point.positions)):
                    raise ValueError('Stationary plan differs from the starting state.')
                self.get_logger().info('Planning SUCCESS. Stationary plan; checking actual goal without execution.')
            else:
                self.get_logger().info(
                    f'Planning SUCCESS. Requested minimum {duration:.3f} s; execution {execution_duration:.3f} s')
                if self.count_publishers('/joint_states') != 1:
                    raise RuntimeError('Another JointState publisher appeared. Execution cancelled.')
                execution = ExecuteTrajectory.Goal(trajectory=planned.planned_trajectory)
                self._action_result(self.execution_client, execution, self.timeout + execution_duration * 1.2 + 0.5, 'Execution')
            # 실행 성공 또는 정지 계획 확인 후 새 실제 상태로 목표를 검사한다.
            final = self._fresh_state()
            self._print_positions('FINAL', final, names, gripper_mode)
            if any(abs(final[name] - goals[name]) > tolerance for name in names):
                raise RuntimeError(f'Final joint state differs from target (tolerance {tolerance} {unit}).')
            if gripper_mode:
                finger1, finger2 = [final[name] for name in FINGER_NAMES]
                if (abs(finger2 - goals[FINGER_NAMES[1]]) > tolerance or
                        abs(finger1 - finger2) > tolerance or
                        abs(finger1 + finger2 - 2.0 * goal_positions[0]) > tolerance):
                    raise RuntimeError('Final gripper width or mimic finger differs from target.')
            result_label = 'Execution' if execution_duration > 0 else 'Goal'
            self.get_logger().info(
                f'{result_label} SUCCESS. Final JointState matches goal (tolerance {tolerance:.4f} {unit}).')
            return final
        except BaseException:
            self.stop()
            raise

    def _action_result(self, client, goal, timeout, label):
        """목표 수락/action 상태/MoveIt 결과를 구분한다. timeout 초 안의 성공 결과만 반환한다."""
        self.goal_kind = label
        self.pending_goal = client.send_goal_async(goal)
        handle = self._wait_future(self.pending_goal, self.timeout, f'{label} goal response')
        self.pending_goal = None
        if not handle.accepted:
            self.goal_kind = None
            raise RuntimeError(f'{label} goal was rejected.')
        self.active_goal = handle
        self.active_result = handle.get_result_async()
        response = self._wait_future(self.active_result, timeout, f'{label} result')
        self.active_goal = None
        self.active_result = None
        self.goal_kind = None
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'{label} failed: action status={response.status}, MoveIt error={response.result.error_code.val}')
        return response.result

    def stop(self):
        """본인의 진행 중 목표에 정지/취소를 요청한다. 응답별 최대 3초 대기하며 반환값은 없다.

        실행 목표일 때만 공식 MoveGroupInterface.stop()과 같은 stop 이벤트를 보낸다.
        Planning 취소는 전역 정지 이벤트를 보내지 않는다. 미확인/거부는 로그로 표시한다.
        닫힌 Robot과 완료된 목표에는 동작하지 않는다. 이 동기 API는 한 스레드에서 사용한다.
        """
        if self._closed or not self._node_ready or not self._owned_context.ok():
            return
        stop_needed = self.goal_kind == 'Execution' and (
            self.pending_goal is not None or self.active_goal is not None)
        if self.pending_goal is not None:
            try:
                handle = self._wait_future(self.pending_goal, 3.0, 'Pending goal response')
                if handle.accepted:
                    self.active_goal = handle
                else:
                    stop_needed = False
            except Exception as error:
                self.get_logger().warning(f'Pending goal status is unknown: {error}')
            self.pending_goal = None
        if stop_needed:
            self.stop_publisher.publish(String(data='stop'))
            self.get_logger().info('Fake trajectory stop requested (MoveGroupInterface.stop event).')
        if self.active_goal is not None:
            try:
                response = self._wait_future(self.active_goal.cancel_goal_async(), 3.0, 'Cancel goal')
                if response.return_code == CancelGoal.Response.ERROR_NONE and response.goals_canceling:
                    self.get_logger().info('Goal cancellation acknowledged by MoveIt.')
                elif response.return_code == CancelGoal.Response.ERROR_GOAL_TERMINATED:
                    self.get_logger().info('Goal already finished before cancellation.')
                else:
                    self.get_logger().warning(f'Goal cancellation rejected: code={response.return_code}')
            except Exception as error:
                self.get_logger().warning(f'Goal cancellation was not confirmed: {error}')
            if self.goal_kind == 'Execution':
                try:
                    future = self.active_result or self.active_goal.get_result_async()
                    response = self._wait_future(future, 3.0, 'Execution stop result')
                    if response.result.error_code.val == MoveItErrorCodes.PREEMPTED:
                        self.get_logger().info('Fake trajectory stop confirmed by MoveIt (PREEMPTED).')
                    elif response.result.error_code.val == MoveItErrorCodes.SUCCESS:
                        self.get_logger().info('Fake trajectory finished before the stop request.')
                    else:
                        self.get_logger().warning(f'Execution ended with MoveIt error={response.result.error_code.val}')
                except Exception as error:
                    self.get_logger().warning(f'Fake trajectory stop was not confirmed: {error}')
        self.active_goal = None
        self.active_result = None
        self.goal_kind = None

    def close(self):
        """목표와 소유 자원을 정리한다. 여러 번 호출해도 안전하고 외부 context는 유지한다.

        응답을 기다릴 수 있는 동안 정지 요청 후 액션 클라이언트, 노드, executor,
        소유 context 순서로 해제한다. 생성 중 실패한 부분 자원도 같은 경로로 정리한다.
        """
        if self._closed:
            return
        try:
            self.stop()
        finally:
            self._closed = True
            for client in self._action_clients:
                client.destroy()
            if self._owned_executor is not None and self._node_ready:
                self._owned_executor.remove_node(self)
            if self._node_ready:
                self.destroy_node()
                self._node_ready = False
            if self._owned_executor is not None:
                self._owned_executor.shutdown(timeout_sec=1.0)
            self._owned_context.try_shutdown()
