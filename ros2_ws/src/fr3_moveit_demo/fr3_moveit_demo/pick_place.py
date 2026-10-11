"""보정된 직육면체를 집고 원래 자리에 내려놓는 작은 MoveIt 작업.

입력은 베이스 기준 상단 중심[m], 물체 크기[m], 작업대 높이[m]와 집기 힘[N]이다.
출력은 단계별 TCP/관절 측정과 완료 보고서다. IK/충돌 검사는 MoveIt 서버가 수행한다.
CV 선택 예제와 가상 통합 검사가 이 파일을 재사용한다. 실물 접속은 Robot이 담당한다.
Fake Hardware에서는 Planning Scene의 붙임/분리만 재현하며 물리적 집기 검증은 아니다.
"""

import copy
import math
import time

from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject, BoundingVolume,
                            CollisionObject, Constraints, MoveItErrorCodes, OrientationConstraint,
                            PlanningScene, PlanningSceneComponents, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionFK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from .robot import Robot, FINGER_NAMES, JOINT_NAMES, check_bounds, slow_plan


BASE, TCP = 'fr3_link0', 'fr3_hand_tcp'
BLOCK, SUPPORT = 'fr3_demo_block', 'fr3_demo_support'
FINGERS = ['fr3_leftfinger', 'fr3_rightfinger']


class CancellableRobot(Robot):
    """GUI 스레드의 중단 요청을 작업 스레드에서 처리한다. 다른 노드 context는 유지한다."""

    cancel_event = None

    def check_cancel(self):
        """ESC/Ctrl+C 요청 시 예외로 작업을 중단한다. stop 결과 대기 중에는 검사하지 않는다."""
        if self.cancel_event is not None and self.cancel_event.is_set():
            raise KeyboardInterrupt('Pick task cancellation requested.')

    def _wait_future(self, future, timeout, label):
        """최대 100 ms씩 ROS 응답을 처리해 GUI 중단을 반영한다. 미수신은 TimeoutError다."""
        deadline = time.monotonic() + timeout
        while not future.done() and self._owned_context.ok() and time.monotonic() < deadline:
            self.check_cancel()
            self._owned_executor.spin_once(timeout_sec=min(.1, max(0., deadline - time.monotonic())))
        self.check_cancel()
        if not future.done() or future.cancelled():
            raise TimeoutError(f'{label} timeout.')
        if future.result() is None:
            raise RuntimeError(f'{label} returned no response.')
        return future.result()

    def _fresh_state(self):
        """새 측정 상태를 읽기 전에 중단 요청을 반영한다. 반환값은 Robot과 같다."""
        self.check_cancel()
        return super()._fresh_state()

    def stop(self):
        """중단 요청이 있어도 기존 Robot의 취소/정지 결과 확인을 끝까지 수행한다."""
        event, self.cancel_event = self.cancel_event, None
        try:
            return super().stop()
        finally:
            self.cancel_event = event


def validate_selection(top, size, table_z, clearance, lift, force):
    """위치/치수[m]와 힘[N]을 검사하고 상단 클릭과 수평 작업대 가정을 확인한다.

    실행 전에 호출하며 잘못된 값은 ValueError다. 범위는 이 교육용 작업의 제한이며
    실제 로봇의 안전 영역을 정의하지 않는다. 물체 X/Y는 베이스 축과 평행해야 한다.
    """
    values = [*top, *size, table_z, clearance, lift, force]
    if len(top) != 3 or len(size) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('Target and object size must contain three finite values.')
    if not all(0.01 <= v <= 0.10 for v in size) or not size[1] < 0.07:
        raise ValueError('Object dimensions must be 10..100 mm, grasp width Y below 70 mm.')
    if not (0.25 <= top[0] <= 0.65 and abs(top[1]) <= 0.25 and 0.10 <= top[2] <= 0.50):
        raise ValueError('Selected point is outside this demo workspace (base metres).')
    if not 0.03 <= table_z <= 0.45 or abs(top[2] - size[2] - table_z) > 0.005:
        raise ValueError('Clicked top minus object height must match measured table Z within 5 mm.')
    if not 0.05 <= clearance <= 0.15 or not 0.03 <= lift <= clearance:
        raise ValueError('Require clearance 0.05..0.15 m and lift 0.03..clearance m.')
    if not 5 <= force <= 70:
        raise ValueError('Explicit Franka grasp force must be 5..70 N; choose for the object.')
    center = [top[0], top[1], top[2] - size[2] / 2]
    return center


def downward_pose(xyz):
    """베이스 XYZ[m]에서 아래를 향한 TCP Pose를 만든다. quaternion은 XYZW다.

    X축 180도 회전으로 TCP +Z를 베이스 -Z로 향하게 하고 Y 방향의 물체 면을 집는다.
    """
    pose = Pose(orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0))
    pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
    return pose


class PickAndReturn:
    """Robot의 상태/정지 검사를 재사용하며 TCP 계획과 필요한 Scene 처리를 추가한다."""

    def __init__(self, robot, duration=5.0):
        """Robot과 각 이동의 최소 시간[s]을 받는다. 공식 Hand/TCP가 없으면 거부한다."""
        self.robot = robot.connect()
        self.duration = robot._positive_seconds(duration, 'duration', 3600.0)
        if not robot.finger_bounds:
            raise ValueError('Pick task requires the official Franka Hand model.')
        self.clients = {}
        self.trace = []

    def call(self, service, name, request):
        """같은 executor로 MoveIt 서비스 응답을 기다린다. 미기동/timeout은 예외다."""
        new_client = name not in self.clients
        if new_client:
            self.clients[name] = self.robot.create_client(service, name)
        client = self.clients[name]
        if not client.wait_for_service(timeout_sec=self.robot.timeout):
            raise TimeoutError(f'MoveIt service unavailable: {name}')
        if new_client:
            # Fast DDS는 서비스 발견 직후 응답 채널의 발견이 아직 끝나지 않을 수 있다.
            # 첫 요청 전에 executor를 돌려 응답 채널을 준비한다. timeout 후 동작 재시도는 없다.
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if isinstance(self.robot, CancellableRobot):
                    self.robot.check_cancel()
                self.robot._owned_executor.spin_once(timeout_sec=.05)
        return self.robot._wait_future(client.call_async(request), self.robot.timeout, name)

    def state(self, positions):
        """측정 dict로 RobotState의 JointState를 만든다. attached 물체는 Scene에 유지한다."""
        return JointState(name=self.robot.required_names,
                          position=[positions[n] for n in self.robot.required_names])

    def pose(self, positions=None, link=TCP):
        """최신 측정 관절로 지정 링크의 베이스 Pose를 구한다. 기본은 TCP, 외부 실측은 아니다."""
        positions = self.robot._fresh_state() if positions is None else positions
        request = GetPositionFK.Request(header=Header(frame_id=BASE), fk_link_names=[link])
        request.robot_state.joint_state = self.state(positions)
        request.robot_state.is_diff = True
        result = self.call(GetPositionFK, '/compute_fk', request)
        if (result.error_code.val != MoveItErrorCodes.SUCCESS or result.fk_link_names != [link]
                or len(result.pose_stamped) != 1 or result.pose_stamped[0].header.frame_id != BASE):
            raise RuntimeError('Official TCP forward kinematics failed.')
        return result.pose_stamped[0].pose

    def scene(self):
        """world, attached 물체와 충돌 허용 행렬을 읽는다. 기존 Scene은 보존한다."""
        request = GetPlanningScene.Request(components=PlanningSceneComponents(
            components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                       PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                       PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))
        return self.call(GetPlanningScene, '/get_planning_scene', request).scene

    def apply(self, scene):
        """동기식 Scene diff가 수락된 뒤 다음 계획으로 진행한다. 실패는 예외다."""
        scene.is_diff = True
        scene.robot_state.is_diff = True
        if not self.call(ApplyPlanningScene, '/apply_planning_scene',
                         ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError('Planning Scene update failed.')

    @staticmethod
    def box(name, size, pose, frame=BASE):
        """지정 프레임의 pose와 치수[m]로 BOX 충돌 물체를 만들어 반환한다."""
        return CollisionObject(id=name, header=Header(frame_id=frame),
                               primitives=[SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))],
                               primitive_poses=[pose], operation=CollisionObject.ADD)

    def prepare_scene(self, center, size, table_z):
        """물체/작은 받침대를 추가하고 손가락-물체, 물체-받침대 접촉만 허용한다."""
        current = self.scene()
        if any(o.id in (BLOCK, SUPPORT) for o in current.world.collision_objects) or any(
                o.object.id in (BLOCK, SUPPORT) for o in current.robot_state.attached_collision_objects):
            raise RuntimeError('Previous pick scene remains. Inspect it before another task; no automatic deletion.')
        acm = current.allowed_collision_matrix
        for name in [BLOCK, SUPPORT, *FINGERS]:
            if name not in acm.entry_names:
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_names.append(name)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
        for other in [SUPPORT, *FINGERS]:
            a, b = acm.entry_names.index(BLOCK), acm.entry_names.index(other)
            acm.entry_values[a].enabled[b] = acm.entry_values[b].enabled[a] = True
        object_pose = Pose(orientation=Quaternion(w=1.0))
        object_pose.position.x, object_pose.position.y, object_pose.position.z = map(float, center)
        table_pose = copy.deepcopy(object_pose)
        table_pose.position.z = float(table_z - 0.02)
        diff = PlanningScene(allowed_collision_matrix=acm)
        diff.world.collision_objects = [self.box(BLOCK, size, object_pose),
                                       self.box(SUPPORT, [0.30, 0.30, 0.04], table_pose)]
        self.apply(diff)

    def move(self, target, cartesian=False):
        """TCP 자세를 계획/실행하고 측정 관절의 FK로 도달을 확인한다. 부분 경로는 거부한다.

        cartesian=True는 하강/상승용이다. 충돌 검사와 관절 jump 검사를 활성화한다.
        일반 이동은 공식 MoveGroup/OMPL을 사용하며 IK/보간은 서버가 수행한다.
        """
        robot = self.robot
        robot._ensure_motion_allowed()
        initial = robot._fresh_state()
        try:
            if cartesian:
                request = GetCartesianPath.Request(header=Header(frame_id=BASE), group_name='fr3_arm',
                    link_name=TCP, waypoints=[target], max_step=0.005, jump_threshold=5.0,
                    revolute_jump_threshold=0.20, avoid_collisions=True,
                    max_velocity_scaling_factor=0.1, max_acceleration_scaling_factor=0.1)
                request.start_state.joint_state = self.state(initial)
                request.start_state.is_diff = True
                result = self.call(GetCartesianPath, '/compute_cartesian_path', request)
                if result.error_code.val != MoveItErrorCodes.SUCCESS or not math.isfinite(result.fraction) or result.fraction < 0.999999:
                    raise RuntimeError(f'Cartesian path incomplete: fraction={result.fraction}; no execution.')
                trajectory = result.solution
            else:
                goal = MoveGroup.Goal()
                motion = goal.request
                motion.group_name, motion.num_planning_attempts = 'fr3_arm', 3
                motion.allowed_planning_time = 10.0
                motion.max_velocity_scaling_factor = motion.max_acceleration_scaling_factor = 0.1
                motion.start_state.joint_state = self.state(initial)
                motion.start_state.is_diff = True
                motion.goal_constraints = [Constraints(position_constraints=[PositionConstraint(
                    header=Header(frame_id=BASE), link_name=TCP, weight=1.0,
                    constraint_region=BoundingVolume(primitives=[SolidPrimitive(
                        type=SolidPrimitive.SPHERE, dimensions=[0.001])], primitive_poses=[target]))],
                    orientation_constraints=[OrientationConstraint(header=Header(frame_id=BASE), link_name=TCP,
                        orientation=target.orientation, absolute_x_axis_tolerance=0.01,
                        absolute_y_axis_tolerance=0.01, absolute_z_axis_tolerance=0.01, weight=1.0)])]
                goal.planning_options.plan_only = True
                goal.planning_options.planning_scene_diff.is_diff = True
                goal.planning_options.planning_scene_diff.robot_state.is_diff = True
                trajectory = robot._action_result(robot.planner, goal, robot.timeout + 10, 'Planning').planned_trajectory
            duration = slow_plan(trajectory, self.duration)
            for point in trajectory.joint_trajectory.points:
                check_bounds(point.positions, robot.arm_bounds, 'Planned pick trajectory')
            if any(abs(p - initial[n]) > 0.01 for n, p in zip(JOINT_NAMES, trajectory.joint_trajectory.points[0].positions)):
                raise RuntimeError('Trajectory start differs from measured joints.')
            if robot.count_publishers('/joint_states') != 1:
                raise RuntimeError('Duplicate state publisher before pick execution.')
            before_execution = robot._fresh_state()
            if any(abs(before_execution[n] - initial[n]) > 0.01 for n in JOINT_NAMES):
                raise RuntimeError('Robot moved during planning; trajectory is refused.')
            if duration > 0:
                robot._action_result(robot.execution_client, ExecuteTrajectory.Goal(trajectory=trajectory),
                                     robot.timeout + duration * 1.2 + 0.5, 'Execution')
            actual = self.pose()
            distance = math.dist([getattr(actual.position, n) for n in 'xyz'],
                                 [getattr(target.position, n) for n in 'xyz'])
            dot = abs(sum(getattr(actual.orientation, n) * getattr(target.orientation, n) for n in 'xyzw'))
            if distance > 0.003 or 2 * math.acos(min(1.0, dot)) > 0.04:
                raise RuntimeError(f'Measured TCP differs from goal: distance={distance:.6f} m.')
        except BaseException:
            robot.stop()
            raise

    def attach(self, size, center):
        """집기 성공 후 물체를 TCP에 붙인다. 실제 물체의 유지/미끄러짐 검사는 아니다."""
        tcp = self.pose()
        # 목표 회전에 근사하지 않고 측정한 TCP 자세로 물체의 상대 좌표를 구한다.
        from .calibration import transform_message_to_matrix
        from geometry_msgs.msg import Transform, Vector3
        import numpy as np
        matrix = transform_message_to_matrix(Transform(
            translation=Vector3(x=tcp.position.x, y=tcp.position.y, z=tcp.position.z), rotation=tcp.orientation))
        block_base = np.eye(4)
        block_base[:3, 3] = center
        block_tcp = np.linalg.inv(matrix) @ block_base
        # 물체는 베이스에서 무회전이므로 TCP quaternion의 역회전을 사용한다.
        pose = Pose(orientation=Quaternion(x=-tcp.orientation.x, y=-tcp.orientation.y,
                                          z=-tcp.orientation.z, w=tcp.orientation.w))
        pose.position.x, pose.position.y, pose.position.z = map(float, block_tcp[:3, 3])
        diff = PlanningScene()
        # MoveIt은 같은 ID의 world 물체를 attach 시 자동 제거한다.
        # 같은 diff에 REMOVE를 추가하면 이미 제거한 물체를 두 번 지워 수락이 실패한다.
        diff.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name=TCP, touch_links=FINGERS, object=self.box(BLOCK, size, pose, TCP))]
        self.apply(diff)

    def detach(self, size, center):
        """물체를 원래 world 위치로 되돌린다. 분리 후 손을 열고 받침대 접촉은 허용한다."""
        pose = Pose(orientation=Quaternion(w=1.0))
        pose.position.x, pose.position.y, pose.position.z = map(float, center)
        diff = PlanningScene()
        diff.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name=TCP, object=CollisionObject(id=BLOCK, operation=CollisionObject.REMOVE))]
        diff.world.collision_objects = [self.box(BLOCK, size, pose)]
        self.apply(diff)

    def mark(self, stage):
        """단계별 측정 TCP와 손가락 폭, Scene의 attached 상태를 기록하고 출력한다."""
        state = self.robot._fresh_state()
        pose = self.pose(state)
        attached = any(o.object.id == BLOCK for o in self.scene().robot_state.attached_collision_objects)
        record = dict(stage=stage, tcp=[getattr(pose.position, n) for n in 'xyz'],
                      width=sum(state[n] for n in FINGER_NAMES), attached=attached)
        self.trace.append(record)
        self.robot.get_logger().info(f'PICK STAGE {record}')

    def block_center(self):
        """world 물체 중심[m]을 반환한다. Humble의 object pose와 shape pose를 함께 적용한다."""
        from geometry_msgs.msg import Transform, Vector3
        from .calibration import transform_message_to_matrix, transform_point_to_base
        block = next((o for o in self.scene().world.collision_objects if o.id == BLOCK), None)
        if block is None or len(block.primitive_poses) != 1 or not block.header.frame_id:
            raise RuntimeError('Released block is missing from the scene.')
        pose = block.pose
        matrix = transform_message_to_matrix(Transform(
            translation=Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z), rotation=pose.orientation))
        if block.header.frame_id != BASE:
            # 서버가 물체를 planning frame(world)으로 정규화할 수 있다.
            # 베이스와 같은 원점이라고 가정하지 않고 공식 FK로 다시 변환한다.
            frame = self.pose(link=block.header.frame_id)
            base_frame = transform_message_to_matrix(Transform(translation=Vector3(
                x=frame.position.x, y=frame.position.y, z=frame.position.z), rotation=frame.orientation))
            matrix = base_frame @ matrix
        local = block.primitive_poses[0].position
        return list(map(float, transform_point_to_base([local.x, local.y, local.z], matrix)))

    def run(self, top, size, table_z, clearance=0.10, lift=0.05, force=20.0, hold=1.0):
        """선택→접근→집기→상승→원위치 하강→해제→시작 관절 자세. 완료 보고서 dict를 반환한다.

        단위는 validate_selection과 같다. 실패하면 후속 이동과 자동 손 열기를 하지
        않고 Scene을 유지하며 예외를 전달한다. 현장에서 유지/정지 상태를 확인해야 한다.
        """
        center = validate_selection(top, size, table_z, clearance, lift, force)
        if not math.isfinite(hold) or not 0 <= hold <= 10:
            raise ValueError('Hold time must be 0..10 s.')
        original = self.robot._fresh_state()
        self.pose(original)  # 움직이기 전에 공식 TCP와 프레임을 확인한다.
        self.prepare_scene(center, size, table_z)
        self.mark('initial')
        above = downward_pose([center[0], center[1], center[2] + clearance])
        grasp = downward_pose(center)
        raised = downward_pose([center[0], center[1], center[2] + lift])
        self.robot.set_gripper(min(0.08, size[1] + 0.02), duration=self.duration)
        # 현재 IK 가지를 유지하며 접근한다. 별도의 OMPL 자세 목표는 같은 TCP에도
        # 다른 관절 해를 선택해 다음 수직 하강이 끊길 수 있다.
        self.move(above, cartesian=True)
        self.mark('approach')
        self.move(grasp, cartesian=True)
        self.mark('at_object')
        self.robot.grasp(size[1], force, speed=0.01)
        self.attach(size, center)
        self.mark('grasped')
        self.move(raised, cartesian=True)
        self.mark('lifted')
        deadline = time.monotonic() + hold
        while time.monotonic() < deadline:
            self.robot._fresh_state()  # 드라이버 측정 중단을 확인한다. 미끄러짐 검사는 아니다.
        self.move(grasp, cartesian=True)
        self.mark('lowered')
        self.detach(size, center)
        self.robot.set_gripper(min(0.08, size[1] + 0.02), duration=self.duration)
        self.mark('released')
        self.move(above, cartesian=True)
        self.robot._execute_motion([original[n] for n in JOINT_NAMES], self.duration, self.robot._fresh_state())
        self.mark('returned')
        final = self.robot.get_joint_positions()
        if any(abs(final[n] - original[n]) > 0.002 for n in JOINT_NAMES):
            raise RuntimeError('Initial arm joints were not restored.')
        return_error = math.dist(self.trace[0]['tcp'], self.trace[-1]['tcp'])
        if return_error > 0.003:
            raise RuntimeError('Returned TCP differs from initial measured TCP.')
        if any(o.object.id == BLOCK for o in self.scene().robot_state.attached_collision_objects):
            raise RuntimeError('Block is still attached after release.')
        placed = self.block_center()
        if math.dist(placed, center) > 0.003:
            raise RuntimeError('Released Scene object differs from original center.')
        return dict(backend=self.robot.backend, stages=self.trace, original_joints=original,
                    final_joints=final, selected_top=list(top), object_center=center,
                    placed_scene_center=placed, return_tcp_error_m=return_error,
                    physical_grasp_verified=False)

    def close(self):
        """소유한 서비스 클라이언트를 정리한다. Scene과 물리적 유지 상태는 바꾸지 않는다."""
        for client in self.clients.values():
            self.robot.destroy_client(client)
        self.clients.clear()
