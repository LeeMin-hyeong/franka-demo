"""공식 FR3 모델/플래너/실물 드라이버로 구성하는 명시적 FCI 연결 launch.

입력은 robot_ip, with_gripper, rviz다. 실제 로봇과 연결하고 공식 effort 컨트롤러를
활성화한다. 초기 ready/이동 목표/그리퍼 homing을 자동 전송하지 않는다.
가상 launch의 position 변환이나 손가락 GenericSystem을 사용하지 않는다.
실물 IP와 RT 환경은 run.sh에서도 확인하며, 직접 launch할 때도 여기서 검사한다.
"""

import ipaddress
import os
import resource
import tempfile
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def check_real_pc():
    """실물 FCI 서버의 native Linux/RT/권한을 검사한다. 네트워크에 연결하지 않는다."""
    with open('/proc/sys/kernel/osrelease', encoding='utf-8') as stream:
        if 'microsoft' in stream.read().lower():
            raise RuntimeError('Real FCI requires a native Ubuntu PREEMPT_RT control PC.')
    rt = False
    if os.path.isfile('/sys/kernel/realtime'):
        with open('/sys/kernel/realtime', encoding='utf-8') as stream:
            rt = stream.read().strip() == '1'
    with open('/proc/version', encoding='utf-8') as stream:
        rt = rt or 'PREEMPT_RT' in stream.read()
    if not rt:
        raise RuntimeError('PREEMPT_RT kernel required. Read docs/real-robot.md.')
    for limit, minimum in [(resource.RLIMIT_RTPRIO, 99), (resource.RLIMIT_MEMLOCK, 102400 * 1024)]:
        soft, _ = resource.getrlimit(limit)
        if soft != resource.RLIM_INFINITY and soft < minimum:
            raise RuntimeError('Realtime permissions are insufficient. Read docs/real-robot.md and log in again.')


def robot_configuration(robot_ip, with_gripper):
    """공식 모델/YAML을 읽어 서버 설정을 반환한다. 이 함수는 드라이버를 실행하지 않는다.

    robot_ip는 FCI IPv4 주소, with_gripper는 공식 Franka Hand 설치 여부다.
    출력 URDF에는 실제 plugin만 있으며 가상 초기 자세나 손가락 제어를 추가하지 않는다.
    """
    address = ipaddress.IPv4Address(robot_ip)
    if address.is_loopback or address.is_unspecified or address.is_multicast or address.is_reserved:
        raise ValueError('Specify the Control FCI IPv4 address.')
    config = get_package_share_directory('franka_fr3_moveit_config')
    description = get_package_share_directory('franka_description')
    bringup = get_package_share_directory('franka_bringup')

    def read_config(filename):
        with open(os.path.join(config, 'config', filename), encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    mappings = {'robot_type': 'fr3', 'hand': str(with_gripper).lower(),
                'ee_id': 'franka_hand' if with_gripper else 'none',
                'robot_ip': robot_ip, 'use_fake_hardware': 'false', 'fake_sensor_commands': 'false'}
    urdf = xacro.process_file(os.path.join(bringup, 'urdf', 'franka_arm.urdf.xacro'), mappings=mappings).toxml()
    semantic = xacro.process_file(os.path.join(description, 'robots', 'fr3', 'fr3.srdf.xacro'), mappings=mappings).toxml()
    model = ET.fromstring(urdf)
    if [p.text for p in model.findall('./ros2_control/hardware/plugin')] != ['franka_hardware/FrankaHardwareInterface']:
        raise RuntimeError('Expected exactly one official real FrankaHardwareInterface.')
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
    # 실물 손은 MoveIt GripperCommand의 닫기/grasp 경로를 사용하지 않는다.
    # API의 set_gripper가 공식 Move action에 자유 공간 목표를 직접 전달한다.
    controller_map = read_config('fr3_controllers.yaml')
    controller_map['controller_names'] = ['fr3_arm_controller']
    controller_map.pop('franka_gripper', None)
    return config, urdf, semantic, read_config('kinematics.yaml'), pipeline, controller_map


def create_nodes(context):
    """명시한 실물 IP에 연결할 ROS 노드를 만든다. 자동 이동 목표는 만들지 않는다."""
    check_real_pc()
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    hand_value = LaunchConfiguration('with_gripper').perform(context).lower()
    rviz_value = LaunchConfiguration('rviz').perform(context).lower()
    if hand_value not in ('true', 'false') or rviz_value not in ('true', 'false'):
        raise ValueError('with_gripper and rviz must be true or false.')
    with_gripper = hand_value == 'true'
    config, urdf, srdf, kinematics, pipeline, controller_map = robot_configuration(robot_ip, with_gripper)
    robot = {'robot_description': urdf}
    semantic = {'robot_description_semantic': srdf}
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen', parameters=[
            robot, semantic, kinematics, pipeline, {
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                'moveit_simple_controller_manager': controller_map,
                'moveit_manage_controllers': True,
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.allowed_start_tolerance': 0.01,
                'publish_planning_scene': True, 'publish_geometry_updates': True,
                'publish_state_updates': True, 'publish_transforms_updates': True,
            }]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[robot, os.path.join(config, 'config', 'fr3_ros_controllers.yaml')],
             remappings=[('joint_states', '/franka/joint_states')], on_exit=Shutdown()),
        # aggregator는 임의 초기 위치를 만들지 않고 실제 상태 토픽만 합친다.
        Node(package='joint_state_publisher', executable='joint_state_publisher', output='screen', parameters=[{
            'source_list': ['/franka/joint_states'] + (['/franka_gripper/joint_states'] if with_gripper else []),
            'rate': 30, 'use_robot_description': False,
        }]),
    ]
    for name in ['joint_state_broadcaster', 'franka_robot_state_broadcaster', 'fr3_arm_controller']:
        nodes.append(Node(package='controller_manager', executable='spawner', output='screen', arguments=[
            name, '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60']))
    if with_gripper:
        # 공식 Move action을 제공하는 실제 그리퍼 서버다. homing은 자동 호출하지 않는다.
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('franka_gripper'), 'launch', 'gripper.launch.py')),
            launch_arguments={'robot_ip': robot_ip, 'use_fake_hardware': 'false', 'namespace': ''}.items()))
    if rviz_value == 'true':
        with open(os.path.join(config, 'rviz', 'moveit.rviz'), encoding='utf-8') as stream:
            rviz_parameters = yaml.safe_load(stream)
        # CLI가 목표를 보내므로 오래된 공식 파일의 비활성 GUI 명령 패널을 제외한다.
        # 현재 모델과 MoveIt 계획 경로 디스플레이는 공식 설정을 그대로 재사용한다.
        rviz_parameters['Visualization Manager']['Displays'] = [
            display for display in rviz_parameters['Visualization Manager']['Displays']
            if display.get('Class') != 'moveit_rviz_plugin/MotionPlanning'
        ]
        rviz_parameters['Window Geometry'].update({'Width': 1200, 'Height': 800, 'X': 0, 'Y': 0})
        view = rviz_parameters['Visualization Manager']['Views']['Current']
        view['Distance'] = 2.0
        view['Focal Point'].update({'X': 0.0, 'Y': 0.0, 'Z': 0.5})
        with tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False) as stream:
            yaml.safe_dump(rviz_parameters, stream)
            rviz_filename = stream.name

        def clean_rviz_file(shutdown_context):
            """launch 종료 시 이 실행에서 만든 RViz 설정만 삭제한다."""
            if os.path.exists(rviz_filename):
                os.unlink(rviz_filename)
            return []

        nodes.append(Node(package='rviz2', executable='rviz2', output='log',
                          arguments=['-d', rviz_filename],
                          parameters=[robot, semantic, kinematics, pipeline]))
        nodes.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=clean_rviz_file)])))
    return nodes


def generate_launch_description():
    """IP 없는 기본 호출은 연결할 수 없다. --show-args는 하드웨어를 실행하지 않는다."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='Required Control FCI IPv4 address'),
        DeclareLaunchArgument('with_gripper', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=create_nodes),
    ])
