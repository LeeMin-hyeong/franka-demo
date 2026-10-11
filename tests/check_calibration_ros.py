#!/usr/bin/env python3
"""ROS Humble의 합성 RGB-D/TF로 좌표 발행과 입력 거부를 검증한다.

입력은 메모리의 2x2 영상과 정적 TF다. 카메라나 Robot/FCI를 생성하지 않는다.
프로젝트 install을 source한 뒤 python3 tests/check_calibration_ros.py로 실행한다.
"""

import importlib.util
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster
from check_handeye import fixture, render_board


root = Path(__file__).resolve().parents[1]
path = root / 'ros2_ws/src/fr3_moveit_demo/examples/click_camera_point.py'
spec = importlib.util.spec_from_file_location('click_camera_point', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
record_spec = importlib.util.spec_from_file_location('calibration_record', root / 'calibration/record.py')
record_module = importlib.util.module_from_spec(record_spec)
record_spec.loader.exec_module(record_module)


def main():
    """센서 DDS 연결 → 점 검증 → 프레임/시간/깊이 거부 검사. 실패는 AssertionError다."""
    rclpy.init()
    source = Node('fr3_calibration_fixture')
    node = None
    try:
        mount_camera = np.eye(4)
        mount_camera[:3, 3] = [.1, .2, .3]
        args = NS(matrix=mount_camera, base_frame='fr3_link0',
                  mount_frame='calibration_test_mount', camera_frame='test_color_optical_frame',
                  color_topic='/fr3_calibration_test/color', depth_topic='/fr3_calibration_test/depth',
                  info_topic='/fr3_calibration_test/info', pixel=[1, 1])
        node = module.ClickCameraPoint(args)
        pub = [source.create_publisher(typ, topic, qos_profile_sensor_data)
               for typ, topic in [(Image, args.color_topic), (Image, args.depth_topic),
                                  (CameraInfo, args.info_topic)]]
        received = []
        subscription = source.create_subscription(PointStamped, '/fr3/calibration/clicked_point',
                                                  received.append, 10)
        broadcaster = StaticTransformBroadcaster(source)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = args.base_frame, args.mount_frame
        tf.header.stamp = source.get_clock().now().to_msg()
        tf.transform.rotation.w = 1.0
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (1., 2., 3.)
        broadcaster.sendTransform(tf)
        bridge = CvBridge()

        def images(encoding='16UC1'):
            color = bridge.cv2_to_imgmsg(np.zeros((2, 2, 3), np.uint8), 'bgr8')
            data = np.full((2, 2), 1000, np.uint16) if encoding == '16UC1' else np.ones((2, 2), np.float32)
            depth = bridge.cv2_to_imgmsg(data, encoding)
            info = CameraInfo()
            info.width, info.height = 2, 2
            info.k = [100., 0., 0., 0., 100., 0., 0., 0., 1.]
            info.d = [0.] * 5
            info.distortion_model = 'plumb_bob'
            for msg in (color, depth, info):
                msg.header.frame_id = args.camera_frame
                msg.header.stamp = source.get_clock().now().to_msg()
            return color, depth, info

        deadline = time.monotonic() + 8
        while not received and time.monotonic() < deadline:
            for publisher, message in zip(pub, images()):
                publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=.02)
            if node.images is not None:
                node.project(1, 1)
            rclpy.spin_once(source, timeout_sec=.02)
        assert received, '합성 RGB-D 동기화/TF/PointStamped 수신 실패'
        point = received[-1]
        assert point.header.frame_id == args.base_frame
        np.testing.assert_allclose([point.point.x, point.point.y, point.point.z], [1.11, 2.21, 4.3])
        print('PASS: sensor QoS, RGB-D synchronization, image-time TF chain, PointStamped in base frame')

        for encoding in ('16UC1', '32FC1'):
            sample = images(encoding)
            node.on_images(*sample)
            assert node.images is not None
            node.completed = False
            node.project(1, 1)
            assert node.completed
        print('PASS: mm and m depth encodings')

        bad = images()
        bad[1].header.frame_id = 'wrong_optical_frame'
        node.on_images(*bad)
        assert node.images is None
        bad = images()
        bad[2].width = 3
        node.on_images(*bad)
        assert node.images is None
        bad = images()
        for msg in bad:
            msg.header.stamp.sec -= 2
        node.on_images(*bad)
        assert node.images is None
        print('PASS: wrong frames, CameraInfo resolution, stale timestamps rejected')

        node.on_images(*images())
        node.completed = False
        node.images[1][:] = 0
        node.project(1, 1)
        assert not node.completed
        node.on_images(*images())
        node.args.mount_frame = 'nonexistent_mount'
        node.completed = False
        node.project(1, 1)
        assert not node.completed
        print('PASS: zero depth and unavailable TF produce no coordinate result')

        node.args.mount_frame = 'calibration_test_mount'
        with tempfile.TemporaryDirectory(prefix='calibration-ros-', dir=root) as directory:
            folder = Path(directory)
            assert folder.resolve().parent == root
            calibration = folder / 'T_gripper2camera.npy'
            np.save(calibration, mount_camera)
            metadata = dict(base_frame=args.base_frame, mount_frame=args.mount_frame,
                            camera_frame=args.camera_frame, translation_unit='m',
                            transform_convention='T_mount_camera', image_size=[2, 2],
                            camera_matrix=[[200., 0, 0], [0, 200., 0], [0, 0, 1]], distortion=[0.] * 5)
            calibration.with_suffix('.json').write_text(json.dumps(metadata))
            args.calibration, args.translation_unit = str(calibration), 'm'
            node.camera_calibration = module.load_camera_metadata(args)
            node.on_images(*images())
            node.completed = False
            node.project(1, 1)
            deadline = time.monotonic() + 2
            expected = [1.105, 2.205, 4.3]
            def has_expected_point():
                return bool(received) and np.allclose(
                    [received[-1].point.x, received[-1].point.y, received[-1].point.z], expected)
            while not has_expected_point() and time.monotonic() < deadline:
                rclpy.spin_once(source, timeout_sec=.02)
            assert has_expected_point(), '보정 K/D를 사용한 새 좌표 수신 실패'
            np.testing.assert_allclose([received[-1].point.x, received[-1].point.y,
                                       received[-1].point.z], [1.105, 2.205, 4.3])
            metadata['mount_frame'] = 'wrong_mount'
            calibration.with_suffix('.json').write_text(json.dumps(metadata))
            try:
                module.load_camera_metadata(args)
                raise AssertionError('서로 다른 보정 프레임을 허용했습니다.')
            except ValueError:
                pass
            node.camera_calibration['image_size'] = (3, 2)
            node.on_images(*images())
            assert node.images is None
            print('PASS: paired calibration intrinsics used, frame/resolution mismatch rejected')

            record_args = NS(**vars(args))
            record_args.output, record_args.board = folder / 'recording', [10, 7]
            recorder = record_module.Recorder(record_args)
            try:
                _, _, boards = fixture()
                matrix = np.array([[950., 0, 480], [0, 950, 360], [0, 0, 1]])
                rgb = render_board(boards[0], matrix)
                info = CameraInfo()
                info.width, info.height = 960, 720
                deadline = time.monotonic() + 5
                while not recorder.data['samples'] and time.monotonic() < deadline:
                    color = bridge.cv2_to_imgmsg(rgb, 'bgr8')
                    for message in (color, info):
                        message.header.frame_id = args.camera_frame
                        message.header.stamp = source.get_clock().now().to_msg()
                    pub[0].publish(color)
                    pub[2].publish(info)
                    rclpy.spin_once(recorder, timeout_sec=.02)
                    if recorder.image is not None:
                        recorder.capture()
                assert len(recorder.data['samples']) == 1, '촬영/영상시각 TF 저장 실패'
                saved = json.loads((record_args.output / 'calibrate_data.json').read_text())
                assert saved['translation_unit'] == 'm'
                np.testing.assert_allclose(np.array(saved['samples'][0]['T_base_mount'])[:3, 3], [1, 2, 3])
                assert (record_args.output / saved['samples'][0]['file_name']).is_file()
                assert not recorder.capture(), '같은 영상 중복 저장'
                recorder.args.mount_frame = 'missing_record_mount'
                recorder.image[1].nanosec += 1
                assert not recorder.capture(), 'TF 없는 영상 저장'
                assert len(recorder.data['samples']) == 1
                print('PASS: synchronized RGB/TF recorder, m pose/PNG/JSON, duplicate/missing TF rejection')
            finally:
                recorder.destroy_node()
    finally:
        if node is not None:
            node.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
