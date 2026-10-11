"""공식 FR3 팔과 Franka Hand의 공통 Python 제어 API.

라이브러리: rclpy, moveit_msgs, sensor_msgs와 Python 표준 XML 파서.
입력: 관절 번호, 상대 이동량(rad), 그리퍼 폭(m), 시간(s)과 명시적 집기 힘(N).
출력: START/GOAL/FINAL 로그와 검증한 실제 관절 위치 dict, 실패 시 예외.
흐름: 서버 모델 확인 -> 최신 JointState -> 관절 제한 검사 -> 계획 -> 실행 -> 검증.
기본 fake MoveIt 서버와 명시적으로 선택한 real 서버를 CLI와 예제에서 함께 사용한다.
Humble에서는 MoveGroupInterface의 Python 래퍼 대신 공식 MoveGroup 액션과
ExecuteTrajectory 액션을 사용한다. 실물 팔의 FCI 연결은 공식 ros2_control 드라이버가
담당한다. 자유 공간 폭 이동은 Move, 명시적인 물체 집기는 Grasp 액션을 사용한다.
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
RAW_TOPICS = {'arm': '/franka/joint_states', 'hand': '/franka_gripper/joint_states'}


def parse_model(urdf_text, srdf_text, gripper_mode=False, backend='fake', robot_ip=None):
    """팔 제한(rad), ready 자세, 손 제한(m)을 반환한다. 다른 backend/모델은 거부한다.

    모델을 새로 정의하지 않고 실행 중인 서버와 같은 공식 모델에서 값을 읽는다.
    fr3_arm의 공식 chain을 URDF에서 따라가며 일곱 관절 이름도 확인한다.
    gripper_mode이면 fr3_hand의 독립 관절과 URDF mimic 관계도 검사한다.
    backend는 fake 또는 real이다. 모델 확인은 서버 설정 검사이며 물리적 안전 보증은 아니다.
    real이면 명시한 robot_ip와 서버 URDF의 연결 주소가 정확히 일치해야 한다.
    """
    urdf, srdf = ET.fromstring(urdf_text), ET.fromstring(srdf_text)
    plugins = [plugin.text.strip() if plugin.text else ''
               for plugin in urdf.findall('./ros2_control/hardware/plugin')]
    expected_plugins = {
        'fake': 'mock_components/GenericSystem',
        'real': 'franka_hardware/FrankaHardwareInterface',
    }
    if backend not in expected_plugins:
        raise ValueError("backend must be 'fake' or 'real'.")
    if plugins != [expected_plugins[backend]]:
        raise ValueError(f'Backend {backend} requires exactly one {expected_plugins[backend]} hardware plugin.')
    if backend == 'real':
        configured_ip = urdf.find("./ros2_control/hardware/param[@name='robot_ip']")
        if not isinstance(robot_ip, str) or not robot_ip.strip() or robot_ip.strip() == 'dont-care':
            raise ValueError('Real backend requires an explicit robot_ip.')
        if configured_ip is None or (configured_ip.text or '').strip() != robot_ip.strip():
            raise ValueError('Real hardware URDF robot_ip differs from the explicitly requested robot_ip.')
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
    """동기 메서드로 선택한 FR3 backend를 조작하는 ROS 노드. 한 호출 스레드에서 사용한다."""

    def __init__(self, args=None, wait_timeout=30.0, backend='fake', robot_ip=None):
        """독립 ROS 통신 자원을 만든다. 서버 연결은 최초 API 호출 시 수행한다.

        args는 rclpy의 ROS 인자 목록이며 None이면 sys.argv를 사용한다.
        wait_timeout은 서버/현재 상태 응답 대기 한도(s), 범위는 0초 초과 300초 이하다.
        backend는 기본 fake이며 real을 명시하면 실제 동작 요청을 허용한다.
        ROS 인자 -p backend:=real도 같은 명시적 선택이다. 실물 서버는 별도로 시작해야 한다.
        robot_ip는 real에서 필수인 대상 주소다. ROS 인자 -p robot_ip:=주소도 지원한다.
        명시한 backend/주소는 ROS 파라미터 인자로 덮어쓸 수 있으므로 두 방법을 섞지 않는다.
        생성 중 오류가 나면 이미 만든 자원도 정리한 뒤 예외를 전달한다.
        """
        self._closed = False
        self._node_ready = False
        self._owned_context = Context()
        self._owned_executor = None
        self._action_clients = []
        self.gripper_move_client = None
        self.gripper_grasp_client = None
        self.gripper_stop_client = None
        self.backend = backend
        self.latest_state = None
        self.state_count = 0
        self._raw_states = {}
        self._raw_state_counts = {'arm': 0, 'hand': 0}
        self._raw_hand_subscription = None
        self.active_goal = None
        self.pending_goal = None
        self.active_result = None
        self.goal_kind = None
        self._motion_uncertain = False
        self._connected = False
        self.required_names = JOINT_NAMES[:]
        self.arm_bounds = []
        self.ready = []
        self.finger_bounds = []
        try:
            if backend not in ('fake', 'real'):
                raise ValueError("backend must be 'fake' or 'real'.")
            self.timeout = self._positive_seconds(wait_timeout, 'wait_timeout', 300.0)
            # Context.init을 직접 호출해 외부 노드의 전역 context와 신호 정책을 유지한다.
            # 종료 후 이 context를 재사용하지 않고 새 Robot을 생성한다.
            self._owned_context.init(args=args)
            super().__init__('fr3_moveit_python_demo', context=self._owned_context)
            self._node_ready = True
            self.backend = self.declare_parameter('backend', backend).value
            if self.backend not in ('fake', 'real'):
                raise ValueError("backend must be 'fake' or 'real'.")
            self.robot_ip = self.declare_parameter('robot_ip', robot_ip or '').value
            if self.backend == 'real' and (
                    not isinstance(self.robot_ip, str) or not self.robot_ip.strip() or self.robot_ip.strip() == 'dont-care'):
                raise ValueError('Real backend requires an explicit robot_ip.')
            self._owned_executor = SingleThreadedExecutor(context=self._owned_context)
            self._owned_executor.add_node(self)
            self.subscription = self.create_subscription(
                JointState, '/joint_states', self._on_joint_state, qos_profile_sensor_data)
            if self.backend == 'real':
                # joint_state_publisher는 raw 측정을 캐시한 채 현재 시각으로 다시 발행한다.
                # 집계 /joint_states가 계속 와도 실물 드라이버가 멈췄을 수 있으므로
                # 실제 팔 측정 토픽을 별도로 확인한다. fake에는 이 구독을 만들지 않는다.
                self._raw_arm_subscription = self.create_subscription(
                    JointState, RAW_TOPICS['arm'],
                    lambda message: self._on_raw_joint_state('arm', message), qos_profile_sensor_data)
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

    def _ensure_motion_allowed(self):
        """실물 정지 결과가 불확실하면 다음 이동을 거부한다. 상태 조회는 계속 허용한다.

        이 차단은 같은 Robot 객체에 적용된다. 새 객체 생성은 물리적 정지 확인이 아니며
        정지 미확인 시 비상정지와 현장 확인 절차를 먼저 수행해야 한다.
        """
        self._ensure_open()
        if self.backend == 'real' and self._motion_uncertain:
            raise RuntimeError(
                'Previous real motion stop was not confirmed. Further motion is blocked; '
                'verify physical stop before creating another Robot or issuing robot commands.')

    def __enter__(self):
        """with 안에서 같은 ROS 연결을 재사용한다. 서버를 기다리지 않고 self를 반환한다."""
        self._ensure_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """정상/실패/Ctrl+C 모두 자원을 정리한다. 원래 예외를 호출자에게 전달한다."""
        self.close()
        return False

    def connect(self, wait_timeout=None):
        """서버의 공식 모델과 선택한 hardware plugin을 확인하고 self를 반환한다.

        wait_timeout을 지정하면 이후 요청의 대기 한도(s)로 사용한다.
        모델/서버가 없거나 backend와 plugin이 다르면 예외다. 상태는 각 메서드에서 새로 받는다.
        """
        self._ensure_open()
        if wait_timeout is not None:
            self.timeout = self._positive_seconds(wait_timeout, 'wait_timeout', 300.0)
        if self._connected:
            return self
        if not self.parameters_client.wait_for_service(timeout_sec=self.timeout):
            raise TimeoutError(f'/move_group parameter service timeout. Start the {self.backend} server first.')
        request = GetParameters.Request(names=['robot_description', 'robot_description_semantic'])
        response = self._wait_future(self.parameters_client.call_async(request), self.timeout, 'URDF/SRDF')
        if len(response.values) != 2 or any(value.type != ParameterType.PARAMETER_STRING for value in response.values):
            raise ValueError('move_group did not provide URDF and SRDF strings.')
        urdf_text, srdf_text = [value.string_value for value in response.values]
        has_hand = ET.fromstring(srdf_text).find("./group[@name='fr3_hand']") is not None
        self.arm_bounds, self.ready, self.finger_bounds = parse_model(
            urdf_text, srdf_text, gripper_mode=has_hand, backend=self.backend, robot_ip=self.robot_ip)
        check_bounds(self.ready, self.arm_bounds, 'Official ready state')
        # 손이 있으면 팔 메서드도 양쪽 손가락을 포함한 하나의 최신 메시지를 확인한다.
        self.required_names = JOINT_NAMES + FINGER_NAMES if has_hand else JOINT_NAMES[:]
        if self.backend == 'real' and has_hand and self._raw_hand_subscription is None:
            self._raw_hand_subscription = self.create_subscription(
                JointState, RAW_TOPICS['hand'],
                lambda message: self._on_raw_joint_state('hand', message), qos_profile_sensor_data)
        for client in self._action_clients:
            if not client.wait_for_server(timeout_sec=self.timeout):
                raise TimeoutError(f'MoveIt action server timeout. Start the {self.backend} server first.')
        self._connected = True
        if self.backend == 'fake':
            self.get_logger().info('Python Robot API connected. GenericSystem Fake Hardware only. No FCI connection.')
        else:
            self.get_logger().warning(
                f'Python Robot API connected to REAL hardware at {self.robot_ip}. Motion methods move the physical robot.')
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

    def _on_raw_joint_state(self, source, message):
        """실물 팔/손의 새 측정만 저장한다. 위치는 rad/m, timestamp는 ns다.

        source는 arm 또는 hand이며 이름/유한값/측정 시각을 확인한다. 같은 timestamp를
        반복 발행해도 새 측정으로 세지 않는다. monotonic 수신 시각도 저장해 ROS 시각으로
        집계 메시지 헤더만 갱신되는 경우와 raw 측정 중단을 구분한다.
        """
        names = JOINT_NAMES if source == 'arm' else FINGER_NAMES
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            return
        positions = dict(zip(message.name, message.position))
        if any(name not in positions or not math.isfinite(positions[name]) for name in names):
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000
        previous = self._raw_states.get(source)
        if stamp_ns <= 0 or not -0.1 <= age <= 1.0 or (previous and stamp_ns <= previous['stamp_ns']):
            return
        self._raw_states[source] = {
            'stamp_ns': stamp_ns, 'received_at': time.monotonic(),
            'header': message.header,
            'positions': {name: positions[name] for name in names},
        }
        self._raw_state_counts[source] += 1

    def _check_real_raw_state(self):
        """실물 raw 측정의 생존 여부와 위치 제한을 확인한다. 성공 반환값은 없다.

        측정/수신 모두 1초 이내이고 토픽별 발행자가 정확히 하나여야 한다. 손 없는 모델은
        팔만 요구한다. fake에는 적용하지 않으며 stale/중복/관절 제한 위반은 실행 전에 예외다.
        집계 메시지의 새 timestamp는 raw 측정 시각을 대신할 수 없다.
        """
        if self.backend != 'real':
            return
        sources = ['arm', 'hand'] if self.finger_bounds else ['arm']
        now_ns, received_now = self.get_clock().now().nanoseconds, time.monotonic()
        for source in sources:
            topic, raw = RAW_TOPICS[source], self._raw_states.get(source)
            if self.count_publishers(topic) != 1:
                raise RuntimeError(f'Exactly one real raw JointState publisher required: {topic}')
            if raw is None:
                raise RuntimeError(f'No measured real raw JointState received: {topic}')
            age = (now_ns - raw['stamp_ns']) / 1_000_000_000
            if not -0.1 <= age <= 1.0 or not 0 <= received_now - raw['received_at'] <= 1.0:
                raise RuntimeError(f'Real raw JointState is stale: {topic}. Motion cancelled.')
            names = JOINT_NAMES if source == 'arm' else FINGER_NAMES
            bounds = self.arm_bounds if source == 'arm' else self.finger_bounds
            check_bounds([raw['positions'][name] for name in names], bounds, 'Measured raw state', names)
            if source == 'hand' and abs(raw['positions'][FINGER_NAMES[0]] - raw['positions'][FINGER_NAMES[1]]) > 0.0002:
                raise ValueError('Measured real raw finger states do not satisfy the official mimic relation.')

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
        real은 집계 메시지뿐 아니라 팔/필요한 손의 새 raw 측정도 기다린다.
        """
        marker, deadline = self.state_count, time.monotonic() + self.timeout
        raw_marker = dict(self._raw_state_counts)
        raw_sources = (['arm', 'hand'] if self.finger_bounds else ['arm']) if self.backend == 'real' else []
        while self._owned_context.ok() and time.monotonic() < deadline:
            if self.state_count != marker and all(self._raw_state_counts[source] > raw_marker[source] for source in raw_sources):
                break
            self._owned_executor.spin_once(timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if self.state_count == marker:
            raise TimeoutError('Current Robot State not received for all required FR3 joints. Motion cancelled.')
        if any(self._raw_state_counts[source] == raw_marker[source] for source in raw_sources):
            raise TimeoutError('New measured real raw JointState not received. Cached aggregate state is refused.')
        self._check_real_raw_state()
        if self.count_publishers('/joint_states') != 1:
            raise RuntimeError('Exactly one /joint_states publisher required; stop the other demo.')
        received = dict(zip(self.latest_state.name, self.latest_state.position))
        if self.backend == 'real':
            # 집계기는 RViz/MoveIt에 여러 소스를 연결하는 역할을 유지한다. 이 API가
            # 반환하고 최종 목표와 비교하는 위치는 방금 확인한 실제 raw 측정을 쓴다.
            # 집계 캐시가 한 주기 늦어도 이전 위치를 실물 최종 위치로 보고하지 않는다.
            for source in raw_sources:
                received.update(self._raw_states[source]['positions'])
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
        self._ensure_motion_allowed()
        duration = self._positive_seconds(duration, 'duration', 3600.0)
        initial = self.get_joint_positions()
        self.get_logger().info('HOME: official SRDF named state ready')
        return self._execute_motion(self.ready[:], duration, initial)

    def move_joint(self, joint=4, delta=0.1, duration=3.0):
        """관절 joint(1~7)을 직전 실제 상태에서 delta rad만큼 이동한다.

        duration은 최소 시간(s), 반환값은 실제 전체 최종 상태 dict다.
        잘못된 입력과 공식 관절 제한 밖 목표는 계획 요청 전에 예외로 거부한다.
        """
        self._ensure_motion_allowed()
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
        real에서는 공식 Move 액션을 사용한다. duration은 속도 계산 기준이며 정확한
        이동 시간을 보장하지 않는다. grasp/자동 homing은 호출하지 않는다.
        """
        self._ensure_motion_allowed()
        duration = self._positive_seconds(duration, 'duration', 3600.0)
        if isinstance(width, bool) or not isinstance(width, (int, float)) or not math.isfinite(width) or width < 0:
            raise ValueError('width must be finite and nonnegative (m).')
        self.connect()
        if not self.finger_bounds:
            raise ValueError('Official fr3_hand group is not loaded. Start the stack with the Franka Hand model.')
        check_bounds([width / 2.0, width / 2.0], self.finger_bounds, 'Gripper width', FINGER_NAMES)
        initial = self._fresh_state()
        if self.backend == 'real':
            return self._move_real_gripper(width, duration, initial)
        return self._execute_motion([width / 2.0], duration, initial, gripper_mode=True)

    def _move_real_gripper(self, width, duration, initial):
        """공식 Move 액션으로 전체 폭(m)을 이동하고 측정된 최종 상태 dict를 반환한다.

        duration(s)과 현재 폭으로 속도(m/s)를 정하며 0.05m/s 이하로 제한한다.
        공식 GripperCommand는 축소 요청을 grasp로 바꾸므로 이 자유 공간 예제에서는
        직접 Move를 사용한다. 접촉 시 힘 제어 성공으로 간주하지 않고 실패를 전달한다.
        액션 부재/거부/실패/시간 초과/최종 상태 불일치는 예외다.
        """
        current_width = initial[FINGER_NAMES[0]] + initial[FINGER_NAMES[1]]
        goals = dict(initial)
        goals.update({name: width / 2.0 for name in FINGER_NAMES})
        self._print_positions('START', initial, [FINGER_NAMES[0]], gripper_mode=True)
        self._print_positions('GOAL', goals, [FINGER_NAMES[0]], gripper_mode=True)
        tolerance = 0.0002
        if abs(width - current_width) <= tolerance:
            self.get_logger().info('Gripper already at target width. No motion requested.')
            self._print_positions('FINAL', initial, [FINGER_NAMES[0]], gripper_mode=True)
            return initial
        # fake 실행은 franka_msgs나 실제 드라이버 없이도 동작한다. 실물 손을 요청하는
        # 시점에만 별도의 액션 클라이언트를 만들어 팔만 설치한 서버도 사용할 수 있다.
        from franka_msgs.action import Move
        if self.gripper_move_client is None:
            self.gripper_move_client = ActionClient(self, Move, '/franka_gripper/move')
            self._action_clients.append(self.gripper_move_client)
        if not self.gripper_move_client.wait_for_server(timeout_sec=self.timeout):
            raise TimeoutError('/franka_gripper/move unavailable. Install/enable the official Franka Hand driver.')
        speed = min(0.05, abs(width - current_width) / duration)
        estimated_duration = abs(width - current_width) / speed
        self.get_logger().info(
            f'REAL free-space gripper Move: width={width:.6f} m, speed={speed:.6f} m/s. '
            'Duration is a speed reference; no grasp or automatic homing.')
        try:
            self._action_result(
                self.gripper_move_client, Move.Goal(width=width, speed=speed),
                self.timeout + estimated_duration * 1.2 + 0.5, 'GripperMove')
            final = self._fresh_state()
            self._print_positions('FINAL', final, [FINGER_NAMES[0]], gripper_mode=True)
            finger1, finger2 = [final[name] for name in FINGER_NAMES]
            if (abs(finger1 - width / 2.0) > tolerance or abs(finger2 - width / 2.0) > tolerance or
                    abs(finger1 - finger2) > tolerance or abs(finger1 + finger2 - width) > tolerance):
                raise RuntimeError('Final real gripper width or finger state differs from target.')
            self.get_logger().info('GripperMove SUCCESS. Final measured width matches goal (tolerance 0.0002 m).')
            return final
        except BaseException:
            self.stop()
            raise

    def grasp(self, width, force, speed=0.01, epsilon=0.003):
        """물체 폭[m], 힘[N], 속도[m/s], 폭 허용 오차[m]로 집고 측정 상태를 반환한다.

        real은 공식 Franka Grasp의 성공 응답과 측정 폭을 모두 요구한다. fake는
        손가락 위치만 바꾸며 접촉/힘/미끄러짐을 재현하지 않는다. 실패 시 들어 올리지
        않도록 예외를 전달한다. 자동 homing이나 재시도, 손 열기는 수행하지 않는다.
        """
        self._ensure_motion_allowed()
        for value, label, maximum in ((width, 'width', 0.08), (force, 'force', 70.0),
                                      (speed, 'speed', 0.05), (epsilon, 'epsilon', 0.005)):
            self._positive_seconds(value, label, maximum)
        if epsilon >= width / 2:
            raise ValueError('Grasp epsilon must be smaller than half the object width.')
        if force < 5:
            raise ValueError('Explicit grasp force must satisfy 5 <= force <= 70 N.')
        self.connect()
        if not self.finger_bounds:
            raise ValueError('Grasp requires the official Franka Hand.')
        check_bounds([width / 2] * 2, self.finger_bounds, 'Grasp width', FINGER_NAMES)
        if self.backend == 'fake':
            return self.set_gripper(width, duration=max(1.0, 0.08 / speed))
        from franka_msgs.action import Grasp
        from franka_msgs.msg import GraspEpsilon
        if self.gripper_grasp_client is None:
            self.gripper_grasp_client = ActionClient(self, Grasp, '/franka_gripper/grasp')
            self._action_clients.append(self.gripper_grasp_client)
        if not self.gripper_grasp_client.wait_for_server(timeout_sec=self.timeout):
            raise TimeoutError('Official Franka Grasp action unavailable.')
        initial = self._fresh_state()
        if initial[FINGER_NAMES[0]] + initial[FINGER_NAMES[1]] <= width:
            raise ValueError('Open the fingers wider than the object before grasping.')
        try:
            self._action_result(self.gripper_grasp_client,
                                Grasp.Goal(width=float(width), speed=float(speed), force=float(force),
                                           epsilon=GraspEpsilon(inner=float(epsilon), outer=float(epsilon))),
                                self.timeout + 0.08 / speed, 'GripperGrasp')
            final = self._fresh_state()
            measured = final[FINGER_NAMES[0]] + final[FINGER_NAMES[1]]
            if not width - epsilon < measured < width + epsilon:
                raise RuntimeError('Grasp action succeeded but measured width is outside epsilon.')
            self.get_logger().info(f'Grasp SUCCESS: measured width={measured:.6f} m. Slip is not monitored.')
            return final
        except BaseException:
            self.stop()
            raise

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
                header=self._raw_states['arm']['header'] if self.backend == 'real' else self.latest_state.header,
                name=self.required_names,
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
        """목표 수락/action 상태/서버 결과를 구분한다. timeout 초 안의 성공 결과만 반환한다.

        팔과 가상 손은 MoveIt error_code, 실물 손 Move/Grasp는 success/error를 사용한다.
        액션 타입마다 공식 결과 필드가 다르므로 성공 기준도 따로 확인한다.
        실물 실행 요청 직전 raw 상태를 재검사해 계획 중 드라이버 측정이 중단되면 거부한다.
        """
        if label in ('Execution', 'GripperMove', 'GripperGrasp'):
            self._check_real_raw_state()
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
        if label in ('GripperMove', 'GripperGrasp'):
            success = response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
            failure = f'{label} failed: action status={response.status}, error={response.result.error}'
        else:
            success = response.status == GoalStatus.STATUS_SUCCEEDED and response.result.error_code.val == MoveItErrorCodes.SUCCESS
            failure = f'{label} failed: action status={response.status}, MoveIt error={response.result.error_code.val}'
        if not success and self.backend == 'real' and label in ('Execution', 'GripperMove', 'GripperGrasp'):
            # ABORTED는 ROS 목표 종료를 뜻하지만 실물 정지 확인은 아니다. 예를 들어
            # 그리퍼 서버는 네트워크 예외에도 ABORTED를 반환하며 stop을 호출하지 않는다.
            # 목표 정보를 남겨 호출자의 예외 처리와 close에서 별도 정지를 요청하게 한다.
            self._motion_uncertain = True
        else:
            self.active_goal = None
            self.active_result = None
            self.goal_kind = None
        if not success:
            raise RuntimeError(failure)
        return response.result

    def _request_real_gripper_stop(self):
        """공식 그리퍼 stop 서비스 응답을 반환한다. 성공 True, 미확인/실패 False다.

        이미 ABORTED된 Move/Grasp 목표는 CancelGoal로 정지할 수 없으므로 별도 Trigger를 쓴다.
        서비스 대기와 응답은 각각 최대 3초다. 성공은 드라이버의 stop 명령 응답이며
        물리적 정지 성능의 검증이 아니다. 실제 필요한 시점에만 클라이언트를 만든다.
        """
        try:
            from std_srvs.srv import Trigger
            if self.gripper_stop_client is None:
                self.gripper_stop_client = self.create_client(Trigger, '/franka_gripper/stop')
            if not self.gripper_stop_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warning('Official gripper stop service unavailable. Stop is not confirmed.')
                return False
            response = self._wait_future(
                self.gripper_stop_client.call_async(Trigger.Request()), 3.0, 'Official gripper stop response')
            if response.success:
                self.get_logger().info(
                    'Official gripper driver acknowledged the stop command. Physical stopping performance is not verified.')
                return True
            self.get_logger().warning(f'Official gripper stop command failed: {response.message}')
        except Exception as error:
            self.get_logger().warning(f'Official gripper stop response was not confirmed: {error}')
        return False

    def stop(self):
        """본인의 진행 중 목표에 정지/취소를 요청한다. 응답별 최대 3초 대기하며 반환값은 없다.

        팔/가상 손 실행에만 공식 MoveGroupInterface.stop()과 같은 stop 이벤트를 보낸다.
        실물 손 Move/Grasp는 목표 취소와 별도 stop 서비스로 드라이버에 정지를 요청한다.
        Planning 취소와 실물 손에는 전역 이벤트를 보내지 않는다. 미확인/거부는 로그로 표시한다.
        이 소프트웨어 요청은 실물 로봇의 비상정지를 대체하지 않는다.
        real의 수락/최종 정지 결과가 불확실하면 같은 객체의 다음 이동을 차단한다.
        미확인 목표는 close()에서도 한 번 더 정지를 요청할 수 있도록 유지한다.
        닫힌 Robot과 성공/정지가 확인된 목표에는 동작하지 않는다. 한 스레드에서 사용한다.
        """
        if self._closed or not self._node_ready or not self._owned_context.ok():
            return
        real_motion = self.backend == 'real' and self.goal_kind in ('Execution', 'GripperMove', 'GripperGrasp')
        if real_motion and (self.pending_goal is not None or self.active_goal is not None):
            # CancelGoal 수락은 실제 종료 확인이 아니다. 최종 action 결과를 받기 전까지
            # 보수적으로 다음 실물 이동을 막고, 상태 조회와 정지 재시도만 허용한다.
            self._motion_uncertain = True
        preserve_goal = False
        gripper_result_known = False
        stop_needed = self.goal_kind == 'Execution' and (
            self.pending_goal is not None or self.active_goal is not None)
        if self.pending_goal is not None:
            try:
                handle = self._wait_future(self.pending_goal, 3.0, 'Pending goal response')
                if handle.accepted:
                    self.active_goal = handle
                else:
                    stop_needed = False
                    if real_motion:
                        self._motion_uncertain = False
                self.pending_goal = None
            except Exception as error:
                self.get_logger().warning(f'Pending goal status is unknown: {error}')
                preserve_goal = real_motion
                if not real_motion:
                    self.pending_goal = None
        if stop_needed:
            self.stop_publisher.publish(String(data='stop'))
            self.get_logger().info(f'{self.backend.capitalize()} trajectory stop requested (MoveGroupInterface.stop event).')
        gripper_stop_required = real_motion and self.goal_kind in ('GripperMove', 'GripperGrasp') and (
            self.pending_goal is not None or self.active_goal is not None)
        if self.active_goal is not None:
            try:
                response = self._wait_future(self.active_goal.cancel_goal_async(), 3.0, 'Cancel goal')
                if response.return_code == CancelGoal.Response.ERROR_NONE and response.goals_canceling:
                    server = 'Franka gripper' if self.goal_kind in ('GripperMove', 'GripperGrasp') else 'MoveIt'
                    self.get_logger().info(f'Goal cancellation acknowledged by {server}.')
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
                    if (response.result.error_code.val == MoveItErrorCodes.PREEMPTED or
                            response.status == GoalStatus.STATUS_CANCELED):
                        self._motion_uncertain = False
                        reason = 'PREEMPTED' if response.result.error_code.val == MoveItErrorCodes.PREEMPTED else 'CANCELED'
                        self.get_logger().info(f'{self.backend.capitalize()} trajectory stop confirmed by MoveIt ({reason}).')
                    elif (response.status == GoalStatus.STATUS_SUCCEEDED and
                          response.result.error_code.val == MoveItErrorCodes.SUCCESS):
                        self._motion_uncertain = False
                        self.get_logger().info(f'{self.backend.capitalize()} trajectory finished before the stop request.')
                    else:
                        self.get_logger().warning(f'Execution ended with MoveIt error={response.result.error_code.val}')
                        preserve_goal = real_motion
                except Exception as error:
                    self.get_logger().warning(f'{self.backend.capitalize()} trajectory stop was not confirmed: {error}')
                    preserve_goal = real_motion
            elif self.goal_kind in ('GripperMove', 'GripperGrasp'):
                try:
                    future = self.active_result or self.active_goal.get_result_async()
                    response = self._wait_future(future, 3.0, 'GripperMove stop result')
                    gripper_result_known = response.status in (
                        GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
                    if response.status == GoalStatus.STATUS_CANCELED:
                        self.get_logger().info('Real gripper action cancellation confirmed by the official driver.')
                    elif response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success:
                        self.get_logger().info('Real gripper action finished before cancellation.')
                    else:
                        self.get_logger().warning(
                            f'Gripper action ended: status={response.status}, error={response.result.error}')
                        preserve_goal = real_motion
                except Exception as error:
                    self.get_logger().warning(f'Real gripper stop was not confirmed: {error}')
                    preserve_goal = real_motion
        if gripper_stop_required:
            stop_acknowledged = self._request_real_gripper_stop()
            if stop_acknowledged and gripper_result_known and self.pending_goal is None:
                self._motion_uncertain = False
                preserve_goal = False
            else:
                # 정지 서비스 성공 뒤에도 수락 대기 목표가 새로 실행될 수 있다.
                # 목표의 수락/종료가 불확실하거나 stop 응답이 실패하면 차단을 유지한다.
                self._motion_uncertain = True
                preserve_goal = True
        if not preserve_goal:
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
            if self.gripper_stop_client is not None and self._node_ready:
                self.destroy_client(self.gripper_stop_client)
                self.gripper_stop_client = None
            if self._owned_executor is not None and self._node_ready:
                self._owned_executor.remove_node(self)
            if self._node_ready:
                self.destroy_node()
                self._node_ready = False
            if self._owned_executor is not None:
                self._owned_executor.shutdown(timeout_sec=1.0)
            self._owned_context.try_shutdown()
