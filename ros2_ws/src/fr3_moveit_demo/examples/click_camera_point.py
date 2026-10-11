#!/usr/bin/env python3
"""MacGyvBot click_base_point.py를 FR3용으로 연결한 좌표 검증 예제.

입력: 새 보정 .npy 파일, 기준 프레임/단위, RealSense의 동기화 RGB/정렬 깊이/CameraInfo.
출력: 클릭한 점의 camera/base XYZ(m)와 /fr3/calibration/clicked_point(PointStamped).
흐름: 파일 검사 → ROS 구독 → 같은 시각의 TF → 투영/보정 행렬 곱 → 좌표 확인.
Robot API, 이동 액션, libfranka를 호출하지 않는다. 행렬을 새로 추정하는 프로그램이 아니다.
원본의 투영/행렬 계산을 calibration.py로 분리하고 영상 동기화와 단위 검사를 추가했다.
"""

import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from fr3_moveit_demo.calibration import (
    load_calibration, pixel_to_camera_point, transform_message_to_matrix,
    transform_point_to_base,
)


WINDOW = 'FR3 camera/base point (no robot motion)'


def load_camera_metadata(args):
    """solve.py의 동명 JSON을 검사한다. 없으면 CameraInfo 사용, 불일치/손상은 ValueError다."""
    path = Path(args.calibration).expanduser().with_suffix('.json')
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding='utf-8'))
    expected = dict(base_frame=args.base_frame, mount_frame=args.mount_frame,
                    camera_frame=args.camera_frame, translation_unit=args.translation_unit,
                    transform_convention='T_mount_camera')
    if any(data.get(key) != value for key, value in expected.items()):
        raise ValueError('보정 JSON의 프레임/단위/변환 방향이 CLI와 다릅니다.')
    matrix, distortion = np.asarray(data['camera_matrix'], dtype=float), np.asarray(data['distortion'], dtype=float)
    size = data['image_size']
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all() or matrix[0, 0] <= 0
            or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0, 0, 1])
            or distortion.ndim != 1 or len(distortion) not in (4, 5, 8, 12, 14)
            or not np.isfinite(distortion).all()
            or not isinstance(size, list) or len(size) != 2
            or any(not isinstance(n, int) or n <= 0 for n in size)):
        raise ValueError('보정 JSON의 카메라 내부 파라미터/해상도가 잘못되었습니다.')
    return dict(matrix=matrix, distortion=distortion, image_size=tuple(size))


class ClickCameraPoint(Node):
    """동기화한 영상의 점만 출력하는 노드. 모든 콜백과 GUI는 한 스레드에서 수행한다."""

    def __init__(self, args):
        """CLI 설정을 적용한다. 잘못된 보정 파일은 노드 생성 전에 main에서 거부한다."""
        super().__init__('fr3_click_camera_point')
        self.args = args
        self.mount_camera = args.matrix
        self.camera_calibration = getattr(args, 'camera_calibration', None)
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointStamped, '/fr3/calibration/clicked_point', 10)
        self.images = None
        self.received = 0.0
        self.completed = False
        self.warn_time = 0.0
        # 같은 시각의 RGB/깊이/내부 파라미터를 묶는다. 센서 QoS와 맞춘다.
        self.subscribers = [
            message_filters.Subscriber(self, typ, topic, qos_profile=qos_profile_sensor_data)
            for typ, topic in [(Image, args.color_topic), (Image, args.depth_topic),
                               (CameraInfo, args.info_topic)]
        ]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subscribers, 10, 0.05)
        self.sync.registerCallback(self.on_images)
        if args.pixel is None:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(WINDOW, self.on_click)
        self.get_logger().info('좌표 검증만 수행합니다. 로봇 이동은 요청하지 않습니다.')

    def warn(self, message):
        """무효 입력은 발행하지 않고 최대 1초에 한 번 원인을 출력한다."""
        if time.monotonic() - self.warn_time >= 1.0:
            self.get_logger().warning(str(message))
            self.warn_time = time.monotonic()

    def on_images(self, color, depth, info):
        """동기화 시 호출된다. 정렬/프레임/크기/시각을 확인하고 최신 묶음만 보관한다."""
        try:
            if any(msg.header.frame_id != self.args.camera_frame for msg in (color, depth, info)):
                raise ValueError('RGB/정렬 깊이/CameraInfo의 optical frame이 --camera-frame과 다릅니다.')
            if (color.width, color.height) != (depth.width, depth.height):
                raise ValueError('RGB와 깊이 해상도가 다릅니다. align_depth.enable:=true를 확인하세요.')
            if (info.width, info.height) != (color.width, color.height):
                raise ValueError('CameraInfo 해상도가 RGB와 다릅니다.')
            if self.camera_calibration and (color.width, color.height) != self.camera_calibration['image_size']:
                raise ValueError('현재 RGB 해상도가 보정 촬영 해상도와 다릅니다.')
            if depth.encoding not in ('16UC1', '32FC1'):
                raise ValueError('깊이 encoding은 16UC1(mm) 또는 32FC1(m)이어야 합니다.')
            now = self.get_clock().now().nanoseconds
            if any(not 0 <= now - Time.from_msg(msg.header.stamp).nanoseconds <= 500_000_000
                   for msg in (color, depth, info)):
                raise ValueError('영상 시각이 현재 ROS 시각에서 0.5초 이내가 아닙니다.')
            rgb = self.bridge.imgmsg_to_cv2(color, 'bgr8')
            z = self.bridge.imgmsg_to_cv2(depth, 'passthrough')
            self.images = (rgb, z, color.header.stamp, info, depth.encoding)
            self.received = time.monotonic()
        except (ValueError, cv2.error, RuntimeError, CvBridgeError) as exc:
            self.images = None
            self.warn(exc)

    def project(self, u, v):
        """픽셀을 측정 시각의 베이스 좌표로 변환한다. 깊이/TF 실패는 발행 없이 반환한다."""
        if self.images is None or time.monotonic() - self.received > 0.5:
            self.warn('최신 RGB/깊이/CameraInfo를 기다립니다.')
            return
        _, depth, stamp, info, encoding = self.images
        try:
            if not 0 <= self.get_clock().now().nanoseconds - Time.from_msg(stamp).nanoseconds <= 500_000_000:
                raise ValueError('영상이 오래되었습니다. 다음 영상을 기다리세요.')
            # 원본 calibrateCamera로 추정한 K/D와 외부 보정은 함께 사용한다.
            # JSON이 없는 기존 파일은 드라이버의 CameraInfo를 사용한다.
            matrix = (self.camera_calibration['matrix'] if self.camera_calibration
                      else np.asarray(info.k, dtype=float).reshape(3, 3))
            intrinsics = dict(zip(('fx', 'fy', 'ppx', 'ppy'),
                                  (matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2])))
            point = pixel_to_camera_point(u, v, depth, intrinsics,
                                          depth_scale=0.001 if encoding == '16UC1' else 1.0)
            if point is None:
                raise ValueError('영상 밖 픽셀이거나 유효 깊이가 없습니다.')
            # MacGyvBot의 핀홀 계산에 RGB 왜곡 보정을 추가한다. 깊이 값은 정렬 픽셀에서 읽는다.
            distortion = (self.camera_calibration['distortion'] if self.camera_calibration
                          else np.asarray(info.d, dtype=float))
            if not np.isfinite(distortion).all():
                raise ValueError('CameraInfo 왜곡 계수가 유한하지 않습니다.')
            if np.any(distortion != 0):
                if not self.camera_calibration and info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
                    raise ValueError('지원하지 않는 RGB 왜곡 모델입니다.')
                ray = cv2.undistortPoints(np.array([[[u, v]]], dtype=float),
                                          matrix, distortion)[0, 0]
                point = (ray[0] * point[2], ray[1] * point[2], point[2])
            if self.args.mount_frame == self.args.base_frame:
                base_camera = self.mount_camera  # 외부 고정 카메라: 직접 측정한 T_base_camera.
            else:
                # latest TF가 아닌 영상 시각의 T_base_mount를 사용한다.
                tf = self.buffer.lookup_transform(self.args.base_frame, self.args.mount_frame,
                                                   Time.from_msg(stamp))
                base_camera = transform_message_to_matrix(tf.transform) @ self.mount_camera
            xyz = transform_point_to_base(point, base_camera)
            message = PointStamped()
            message.header.frame_id = self.args.base_frame
            message.header.stamp = stamp
            message.point.x, message.point.y, message.point.z = map(float, xyz)
            self.publisher.publish(message)
            self.get_logger().info(f'pixel=({u},{v}), camera={np.round(point, 4)} m, '
                                   f'{self.args.base_frame}={np.round(xyz, 4)} m')
            self.completed = True
            return message
        except (ValueError, TransformException, cv2.error) as exc:
            self.warn(exc)

    def on_click(self, event, x, y, flags, userdata):
        """왼쪽 클릭 시 좌표를 확인한다. 로봇 동작에는 연결하지 않는다."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.project(int(x), int(y))

    def spin_ui_once(self):
        """영상 창 갱신 또는 --pixel의 단일 검증을 수행한다. ESC/검증 완료는 False다."""
        if self.args.pixel is not None:
            self.project(*self.args.pixel)
            return not self.completed
        if self.images is not None:
            cv2.imshow(WINDOW, self.images[0])
        return cv2.waitKey(1) != 27


def main():
    """CLI → 입력 검사 → ROS/GUI 반복 → 자원 정리. 오류 시 1, Ctrl+C는 정상 종료한다."""
    parser = argparse.ArgumentParser(description='새 FR3 보정 파일로 카메라 좌표만 검증합니다.')
    parser.add_argument('--calibration', required=True, help='T_mount_camera.npy 경로')
    parser.add_argument('--translation-unit', required=True, choices=['m', 'mm'])
    parser.add_argument('--mount-frame', required=True, help='보정 기준 링크. 외부 고정은 base와 같게 지정')
    parser.add_argument('--base-frame', default='fr3_link0')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--color-topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--depth-topic', default='/camera/camera/aligned_depth_to_color/image_raw')
    parser.add_argument('--info-topic', default='/camera/camera/color/camera_info')
    parser.add_argument('--pixel', type=int, nargs=2, metavar=('U', 'V'), help='GUI 없이 한 점 확인 후 종료')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    try:
        for name in ('base_frame', 'mount_frame', 'camera_frame'):
            if not getattr(args, name).strip() or getattr(args, name).startswith('/'):
                raise ValueError('프레임 이름은 비어 있거나 /로 시작할 수 없습니다.')
        args.matrix = load_calibration(args.calibration, args.translation_unit)
        args.camera_calibration = load_camera_metadata(args)
        rclpy.init(args=sys.argv)
        node = None
        try:
            node = ClickCameraPoint(args)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
                if not node.spin_ui_once():
                    break
        finally:
            if args.pixel is None:
                cv2.destroyAllWindows()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except (ValueError, KeyError, TypeError, OSError, RuntimeError, cv2.error) as exc:
        print(f'좌표 검증 실패: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
