"""공식 FR3 팔/Franka Hand 모델과 MoveIt 설정을 재사용하는 Fake Hardware launch.

팔과 그리퍼 모두 위치 명령을 GenericSystem의 상태로 반영한다. 공식 그리퍼
노드는 실행하지 않고 가상 손가락용 JointTrajectoryController를 사용한다.
입력은 설치된 공식 URDF/SRDF/YAML, 출력은 MoveIt/컨트롤러/TF/RViz 노드다.
공식 형상/관절/mimic은 보존하고 부족한 가상 제어 인터페이스만 메모리에 추가한다.
실물 플러그인 선택 인자가 없으며 C++/Python 데모가 같은 서버를 사용한다.
"""

import os
import xml.etree.ElementTree as ET
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """공식 Xacro, 플래너, 컨트롤러 연결 설정으로 가상 실행 노드를 구성한다."""
    config = get_package_share_directory('franka_fr3_moveit_config')
    description = get_package_share_directory('franka_description')
    bringup = get_package_share_directory('franka_bringup')

    def read_config(filename):
        with open(os.path.join(config, 'config', filename), encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    mappings = {'robot_type': 'fr3', 'hand': 'true', 'ee_id': 'franka_hand',
                'robot_ip': 'dont-care', 'use_fake_hardware': 'true',
                'fake_sensor_commands': 'false'}
    urdf = ET.fromstring(xacro.process_file(
        os.path.join(bringup, 'urdf', 'franka_arm.urdf.xacro'), mappings=mappings).toxml())
    srdf = xacro.process_file(os.path.join(description, 'robots', 'fr3', 'fr3.srdf.xacro'),
                             mappings=mappings).toxml()
    # 공식 bringup은 손가락의 ros2_control 인터페이스를 포함하지 않는다.
    # URDF 형상을 새로 만들지 않고 기존 GenericSystem에 가상 인터페이스만 추가한다.
    systems = urdf.findall('ros2_control')
    if len(systems) != 1 or systems[0].findtext('hardware/plugin') != 'mock_components/GenericSystem':
        raise RuntimeError('Exactly one official GenericSystem is required; no real driver allowed.')
    initial_hand = ET.fromstring(srdf).find("group_state[@name='open'][@group='fr3_hand']")
    if initial_hand is None:
        raise RuntimeError('Official FR3 hand open state is missing.')
    initial = {joint.attrib['name']: joint.attrib['value'] for joint in initial_hand.findall('joint')}
    for name in ['fr3_finger_joint1', 'fr3_finger_joint2']:
        model_joint = urdf.find(f"joint[@name='{name}']")
        if model_joint is None or model_joint.attrib['type'] != 'prismatic':
            raise RuntimeError(f'Official hand joint is missing: {name}')
        interface = ET.SubElement(systems[0], 'joint', name=name)
        mimic = model_joint.find('mimic')
        if mimic is None:
            ET.SubElement(interface, 'command_interface', name='position')
        else:
            # Humble GenericSystem은 URDF mimic 외에 제어 블록의 mimic 파라미터도 필요하다.
            ET.SubElement(interface, 'param', name='mimic').text = mimic.attrib['joint']
            ET.SubElement(interface, 'param', name='multiplier').text = mimic.attrib.get('multiplier', '1')
        state = ET.SubElement(interface, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = initial[name]
        state = ET.SubElement(interface, 'state_interface', name='velocity')
        ET.SubElement(state, 'param', name='initial_value').text = '0.0'
    robot_description = {'robot_description': ET.tostring(urdf, encoding='unicode')}
    semantic = {'robot_description_semantic': srdf}
    kinematics = read_config('kinematics.yaml')
    # Humble의 공식 moveit.launch.py와 동일한 기존 OMPL 어댑터를 사용한다.
    pipeline = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                            'default_planner_request_adapters/ResolveConstraintFrames '
                            'default_planner_request_adapters/FixWorkspaceBounds '
                            'default_planner_request_adapters/FixStartStateBounds '
                            'default_planner_request_adapters/FixStartStateCollision '
                            'default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1,
    }}
    pipeline['move_group'].update(read_config('ompl_planning.yaml'))
    moveit_controllers = read_config('fr3_controllers.yaml')
    # 실물 GripperCommand 드라이버 대신 손가락 1개의 가상 궤적 컨트롤러에 연결한다.
    # 손가락 2는 공식 mimic 관계로 따라가므로 별도 목표를 보내지 않는다.
    moveit_controllers['controller_names'] = ['fr3_arm_controller', 'fr3_hand_controller']
    moveit_controllers.pop('franka_gripper', None)
    moveit_controllers['fr3_hand_controller'] = {
        'action_ns': 'follow_joint_trajectory', 'type': 'FollowJointTrajectory',
        'default': True, 'joints': ['fr3_finger_joint1'],
    }
    controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': moveit_controllers,
    }
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[robot_description, semantic, kinematics, pipeline, controllers, {
            'moveit_manage_controllers': True,
            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
            'trajectory_execution.allowed_goal_duration_margin': 0.5,
            'trajectory_execution.allowed_start_tolerance': 0.01,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        }])
    control = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[robot_description, os.path.join(config, 'config', 'fr3_ros_controllers.yaml'),
                    {'fr3_hand_controller.type': 'joint_trajectory_controller/JointTrajectoryController'}])
    # spawner는 컨트롤러 이름을 가진 노드에 파라미터를 적용한다.
    # 공식 관절, 상태 인터페이스, 게인을 그대로 읽고 위치 명령 한 항목만 바꾼다.
    arm_parameters = read_config('fr3_ros_controllers.yaml')['/**']['fr3_arm_controller']
    arm_parameters['ros__parameters']['command_interfaces'] = ['position']
    # /tmp의 수명은 launch 프로세스가 종료될 때까지 유지되며 소스 파일은 불변이다.
    import tempfile
    controller_override = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    hand_parameters = {'ros__parameters': {
        'joints': ['fr3_finger_joint1'], 'command_interfaces': ['position'],
        'state_interfaces': ['position', 'velocity'],
    }}
    yaml.safe_dump({'fr3_arm_controller': arm_parameters,
                    'fr3_hand_controller': hand_parameters}, controller_override)
    controller_override.close()
    arm = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['fr3_arm_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60', '--param-file', controller_override.name])
    broadcaster = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'])
    hand = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['fr3_hand_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60', '--param-file', controller_override.name])
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[robot_description])
    with open(os.path.join(config, 'rviz', 'moveit.rviz'), encoding='utf-8') as stream:
        rviz_parameters = yaml.safe_load(stream)
    # CLI가 목표/계획/실행을 담당하므로 비활성 GUI 명령 패널을 생성하지 않는다.
    # 공식 파일의 현재 상태와 계획 경로 디스플레이는 그대로 사용한다.
    rviz_parameters['Visualization Manager']['Displays'] = [
        display for display in rviz_parameters['Visualization Manager']['Displays']
        if display.get('Class') != 'moveit_rviz_plugin/MotionPlanning'
    ]
    # 공식 파일의 큰 창과 화면 밖 좌표를 WSL에서도 보이는 시작 크기로 맞춘다.
    rviz_parameters['Window Geometry'].update({'Width': 1200, 'Height': 800, 'X': 0, 'Y': 0})
    view = rviz_parameters['Visualization Manager']['Views']['Current']
    view['Distance'] = 2.0
    view['Focal Point'].update({'X': 0.0, 'Y': 0.0, 'Z': 0.5})
    rviz_file = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
    yaml.safe_dump(rviz_parameters, rviz_file)
    rviz_file.close()
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', rviz_file.name],
        parameters=[robot_description, semantic, kinematics, pipeline],
        condition=IfCondition(LaunchConfiguration('rviz')))

    def clean_temporary_parameters(context):
        for filename in [controller_override.name, rviz_file.name]:
            if os.path.exists(filename):
                os.unlink(filename)
        return []

    def process_exited(event, context):
        if not context.is_shutdown and event.returncode != 0:
            raise RuntimeError(f'Fake Hardware process failed with exit code {event.returncode}.')
        return []

    def control_exited(event, context):
        process_exited(event, context)
        return [Shutdown()]

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        state_publisher, move_group, control, broadcaster, arm, hand, rviz,
        RegisterEventHandler(OnProcessExit(target_action=control, on_exit=control_exited)),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=process_exited)),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=process_exited)),
        RegisterEventHandler(OnProcessExit(target_action=hand, on_exit=process_exited)),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=clean_temporary_parameters)])),
    ])
