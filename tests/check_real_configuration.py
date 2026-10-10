#!/usr/bin/env python3
"""실제 FCI 접속 없이 real launch의 공식 모델/설정과 backend 경계를 검사한다.

입력: 저장소의 launch/API 소스, source한 ROS install의 공식 Xacro/YAML.
출력: unittest 결과. 성공0, 실패1이며 실제 하드웨어 동작은 검증하지 않는다.
robot_configuration만 호출한다. create_nodes/ROS 초기화/소켓 연결은 금지한다.
경로는 이 파일의 저장소 위치와 ament 패키지 인덱스로 찾는다.
실행: source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
      python3 tests/check_real_configuration.py
"""

import importlib.util
from pathlib import Path
import socket
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from rclpy.context import Context
from rclpy.node import Node
import xacro
import yaml


REPOSITORY = Path(__file__).resolve().parents[1]
PACKAGE = REPOSITORY / 'ros2_ws' / 'src' / 'fr3_moveit_demo'
ARM_NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
FINGER_NAMES = ['fr3_finger_joint1', 'fr3_finger_joint2']
ROBOT_IP = '192.0.2.2'  # 문서용 주소이며 이 테스트는 어떤 주소에도 연결하지 않는다.


def load_source_module(name, path):
    """저장소 파일을 모듈로 읽는다. main/launch 실행 함수는 호출하지 않는다."""
    specification = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


real_launch = load_source_module('real_configuration_under_test', PACKAGE / 'launch' / 'real_moveit.launch.py')
robot_api = load_source_module('real_model_guard_under_test', PACKAGE / 'fr3_moveit_demo' / 'robot.py')


def read_yaml(path):
    """공식 설정을 안전한 YAML loader로 읽는다. 파일 부재/오류는 테스트 실패다."""
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


class RealConfigurationTests(unittest.TestCase):
    """모델과 제어 경계만 검사한다. ROS node나 로봇 객체를 생성하지 않는다."""

    def check_configuration(self, with_gripper):
        """손 포함 여부별 모델을 검사하고 뒤의 독립 검사를 위해 설정을 반환한다."""
        config, urdf, srdf, kinematics, pipeline, controllers = real_launch.robot_configuration(
            ROBOT_IP, with_gripper)
        model, semantic = ET.fromstring(urdf), ET.fromstring(srdf)
        control_blocks = model.findall('./ros2_control')
        self.assertEqual(len(control_blocks), 1)
        hardware = control_blocks[0].find('hardware')
        self.assertEqual([plugin.text.strip() for plugin in hardware.findall('plugin')],
                         ['franka_hardware/FrankaHardwareInterface'])
        self.assertEqual(hardware.find("param[@name='robot_ip']").text.strip(), ROBOT_IP)
        self.assertNotIn('mock_components/GenericSystem', urdf)
        self.assertNotIn('mock_sensor_commands', urdf)
        self.assertNotIn('state_following_offset', urdf)

        # 실제 팔 인터페이스에는 손가락용 GenericSystem/mimic 설정이 들어오면 안 된다.
        self.assertEqual([joint.get('name') for joint in control_blocks[0].findall('joint')], ARM_NAMES)
        self.assertFalse(control_blocks[0].findall(".//param[@name='mimic']"))
        self.assertFalse(control_blocks[0].findall(".//param[@name='multiplier']"))
        for name in ARM_NAMES:
            self.assertEqual(model.find(f"./joint[@name='{name}']").get('type'), 'revolute')
            interfaces = control_blocks[0].find(f"joint[@name='{name}']").findall('command_interface')
            self.assertIn('effort', [interface.get('name') for interface in interfaces])

        fingers = [joint for joint in model.findall('joint') if joint.get('name') in FINGER_NAMES]
        self.assertEqual(len(fingers), 2 if with_gripper else 0)
        self.assertEqual(len(semantic.findall("./group[@name='fr3_hand']")), int(with_gripper))
        if with_gripper:
            self.assertEqual([joint.get('type') for joint in fingers], ['prismatic', 'prismatic'])
            mimic = model.find("./joint[@name='fr3_finger_joint2']/mimic")
            self.assertEqual(mimic.get('joint'), 'fr3_finger_joint1')
            self.assertEqual(float(mimic.get('multiplier', '1')), 1.0)
            self.assertEqual(float(mimic.get('offset', '0')), 0.0)

        bounds, ready, finger_bounds = robot_api.parse_model(
            urdf, srdf, gripper_mode=with_gripper, backend='real', robot_ip=ROBOT_IP)
        self.assertEqual(len(bounds), 7)
        self.assertEqual(len(ready), 7)
        self.assertEqual(len(finger_bounds), 2 if with_gripper else 0)
        robot_api.check_bounds(ready, bounds, 'Official ready')

        self.assertEqual(controllers['controller_names'], ['fr3_arm_controller'])
        self.assertEqual(set(controllers), {'controller_names', 'fr3_arm_controller'})
        self.assertEqual(controllers['fr3_arm_controller']['type'], 'FollowJointTrajectory')
        self.assertEqual(controllers['fr3_arm_controller']['joints'], ARM_NAMES)
        self.assertEqual(controllers['fr3_arm_controller']['action_ns'], 'follow_joint_trajectory')
        self.assertEqual(kinematics, read_yaml(Path(config) / 'config' / 'kinematics.yaml'))
        self.assertEqual(pipeline['move_group']['planning_plugin'], 'ompl_interface/OMPLPlanner')
        return config, urdf, srdf

    def test_arm_only(self):
        """손 없는 실물 모델에는 팔 7개만 있고 API에서 ready를 읽을 수 있어야 한다."""
        self.check_configuration(False)

    def test_with_franka_hand(self):
        """손 형상/mimic은 존재하되 실물 팔 ros2_control에는 손가락을 추가하지 않는다."""
        self.check_configuration(True)

    def test_official_effort_controller_preserved(self):
        """실물 effort/update rate/gain YAML이 저장소에 받은 공식 파일 그대로인지 확인한다."""
        config = real_launch.robot_configuration(ROBOT_IP, False)[0]
        installed = Path(config) / 'config' / 'fr3_ros_controllers.yaml'
        official = (REPOSITORY / 'ros2_ws' / 'src' / 'vendor' / 'franka_ros2' /
                    'franka_fr3_moveit_config' / 'config' / 'fr3_ros_controllers.yaml')
        self.assertEqual(installed.read_bytes(), official.read_bytes())
        # ROS 파라미터 YAML의 반복 '/**' 블록을 일반 dict loader가 덮어쓰지 않게 읽는다.
        # 공식 파일의 controller_manager 블록과 팔 controller 블록을 함께 검사한다.
        configuration = {}
        for prefix, parameters in yaml.compose(installed.read_text(encoding='utf-8')).value:
            self.assertEqual(prefix.value, '/**')
            configuration.update(yaml.safe_load(yaml.serialize(parameters)))
        self.assertEqual(configuration['controller_manager']['ros__parameters']['update_rate'], 1000)
        controller = configuration['fr3_arm_controller']['ros__parameters']
        self.assertEqual(controller['command_interfaces'], ['effort'])
        self.assertEqual(controller['state_interfaces'], ['position', 'velocity'])
        self.assertEqual(controller['joints'], ARM_NAMES)
        self.assertEqual(set(controller['gains']), set(ARM_NAMES))
        self.assertTrue(all(gain['p'] > 0 and gain['d'] > 0 for gain in controller['gains'].values()))

    def test_invalid_ip_rejected(self):
        """미지정/hostname/IPv6/loopback/multicast 주소로 실물 설정을 만들지 않는다."""
        for address in ('', 'dont-care', 'robot.franka.de', '::1', '127.0.0.1',
                        '0.0.0.0', '224.0.0.1', '192.0.2.999'):
            with self.subTest(address=address), self.assertRaises(ValueError):
                real_launch.robot_configuration(address, False)

    def test_backend_and_robot_ip_guards(self):
        """가상/실물 모델 혼용, 대상 IP 불일치와 손 없는 모델의 손 요청을 거부한다."""
        for with_gripper in (False, True):
            with self.subTest(with_gripper=with_gripper):
                _, urdf, srdf = self.check_configuration(with_gripper)
                with self.assertRaises(ValueError):
                    robot_api.parse_model(urdf, srdf, backend='fake')
                for wrong_ip in (None, '', 'dont-care', '192.0.2.3'):
                    with self.subTest(wrong_ip=wrong_ip), self.assertRaises(ValueError):
                        robot_api.parse_model(urdf, srdf, backend='real', robot_ip=wrong_ip)
                bringup = Path(get_package_share_directory('franka_bringup'))
                fake_urdf = xacro.process_file(
                    str(bringup / 'urdf' / 'franka_arm.urdf.xacro'), mappings={
                        'robot_type': 'fr3', 'hand': str(with_gripper).lower(),
                        'ee_id': 'franka_hand' if with_gripper else 'none',
                        'robot_ip': 'dont-care', 'use_fake_hardware': 'true',
                        'fake_sensor_commands': 'false',
                    }).toxml()
                robot_api.parse_model(fake_urdf, srdf, gripper_mode=with_gripper, backend='fake')
                with self.assertRaises(ValueError):
                    robot_api.parse_model(fake_urdf, srdf, backend='real', robot_ip=ROBOT_IP)
                if not with_gripper:
                    with self.assertRaises(ValueError):
                        robot_api.parse_model(urdf, srdf, gripper_mode=True, backend='real', robot_ip=ROBOT_IP)


def main():
    """소켓/ROS 초기화/노드 생성이 발생하면 즉시 실패하도록 막고 검사만 실행한다."""
    forbidden = AssertionError('Configuration checks must not initialize ROS or connect to a robot.')
    with patch.object(socket, 'socket', side_effect=forbidden), \
            patch.object(Context, 'init', side_effect=forbidden), \
            patch.object(Node, '__init__', side_effect=forbidden), \
            patch.object(real_launch, 'Node', side_effect=forbidden), \
            patch.object(real_launch, 'create_nodes', side_effect=forbidden):
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(RealConfigurationTests)
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    print('Configuration only: no ROS nodes, FCI connection or motion. Physical operation: Not Tested.')
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
