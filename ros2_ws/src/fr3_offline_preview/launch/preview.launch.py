"""공식 FR3 URDF와 ready SRDF를 사용하는 오프라인 시각화 구성.

Python은 ROS2 launch 설정만 담당한다. 궤적 계산과 JointState 발행은 C++ 노드다.
joint_state_publisher GUI, ros2_control, move_group, libfranka Robot은 실행하지 않는다.
"""

import os
import xml.etree.ElementTree as ET
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """모델과 초기 자세는 설치된 공식 description에서 읽고 C++ 노드에 전달한다."""
    description = get_package_share_directory('franka_description')
    package = get_package_share_directory('fr3_offline_preview')
    mappings = {'hand': 'false', 'ee_id': 'none'}
    urdf = xacro.process_file(os.path.join(description, 'robots', 'fr3', 'fr3.urdf.xacro'),
                             mappings=mappings).toxml()
    srdf = xacro.process_file(os.path.join(description, 'robots', 'fr3', 'fr3.srdf.xacro'),
                             mappings=mappings).toxml()
    ready = ET.fromstring(srdf).find("group_state[@name='ready'][@group='fr3_arm']")
    if ready is None:
        raise RuntimeError('Official FR3 SRDF ready state is missing.')
    positions = {joint.attrib['name']: float(joint.attrib['value']) for joint in ready.findall('joint')}
    initial = [positions[f'fr3_joint{i}'] for i in range(1, 8)]
    robot_description = {'robot_description': urdf}
    arguments = [
        DeclareLaunchArgument('joint', default_value='4'),
        DeclareLaunchArgument('delta', default_value='0.1'),
        DeclareLaunchArgument('duration', default_value='3.0'),
        DeclareLaunchArgument('rate', default_value='100.0'),
        # WSL에서 mesh와 RViz 패널이 준비된 후 움직임을 볼 수 있도록 초기 자세를 유지한다.
        DeclareLaunchArgument('start_delay', default_value='15.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]
    preview = Node(
        package='fr3_offline_preview', executable='joint_state_preview', output='screen',
        parameters=[robot_description, {
            'initial_positions': initial,
            'joint': ParameterValue(LaunchConfiguration('joint'), value_type=int),
            'delta': ParameterValue(LaunchConfiguration('delta'), value_type=float),
            'duration': ParameterValue(LaunchConfiguration('duration'), value_type=float),
            'rate': ParameterValue(LaunchConfiguration('rate'), value_type=float),
            'start_delay': ParameterValue(LaunchConfiguration('start_delay'), value_type=float),
        }])
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[robot_description])
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', os.path.join(package, 'rviz', 'preview.rviz')],
        parameters=[robot_description], condition=IfCondition(LaunchConfiguration('rviz')))

    def preview_exited(event, context):
        # 단순 Shutdown만 반환하면 실패한 노드도 ros2 launch 종료값 0이 된다.
        # 사용자 종료가 아닌 입력/노드 오류는 launch 호출자에게도 실패로 전달한다.
        if not context.is_shutdown and event.returncode != 0:
            raise RuntimeError(f'Offline preview failed with exit code {event.returncode}.')
        return [Shutdown()]

    return LaunchDescription(arguments + [
        LogInfo(msg='OFFLINE PREVIEW: no franka::Robot, no Robot::control(), no FCI connection.'),
        state_publisher, preview, rviz,
        RegisterEventHandler(OnProcessExit(target_action=preview, on_exit=preview_exited)),
    ])
