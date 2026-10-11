#!/usr/bin/env python3
"""Calibration_Tutorial/data_recording.py의 수동 촬영을 FR3 ROS TF에 연결한다.

입력: RGB/CameraInfo와 영상 시각의 T_base_mount(m). q를 누를 때 한 쌍을 저장한다.
출력: 새 폴더의 PNG와 calibrate_data.json. solve.py가 이 자료를 읽는다.
두산 ZYZ/DSR API 대신 ROS TF를 사용한다. 이동 목표, FCI, 그리퍼 명령은 보내지 않는다.
로봇과 보드가 정지한 뒤 촬영해야 한다. 자동으로 보정 자세를 만들지 않는다.
"""

import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import message_filters
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from fr3_moveit_demo.calibration import transform_message_to_matrix
from fr3_moveit_demo.handeye import checkerboard_points, find_corners


WINDOW = 'FR3 hand-eye recording: q=save, ESC=exit (no robot motion)'


class Recorder(Node):
    """수동 촬영 노드. pose와 영상은 같은 시각을 사용하고 오래된 입력은 저장하지 않는다."""

    def __init__(self, args):
        """새 출력 폴더를 생성하고 구독한다. 기존 폴더는 FileExistsError로 거부한다."""
        super().__init__('fr3_calibration_recorder')
        self.args = args
        args.output.mkdir(parents=True, exist_ok=False)
        self.data = dict(translation_unit='m', pose_convention='T_base_mount',
                         base_frame=args.base_frame, mount_frame=args.mount_frame,
                         camera_frame=args.camera_frame, samples=[])
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.image = None
        self.received = 0.0
        self.warn_time = 0.0
        self.subscribers = [message_filters.Subscriber(self, typ, topic,
                                                       qos_profile=qos_profile_sensor_data)
                            for typ, topic in ((Image, args.color_topic), (CameraInfo, args.info_topic))]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subscribers, 10, .05)
        self.sync.registerCallback(self.on_image)
        self.get_logger().info('자동 이동 없음. 보드를 고정하고 로봇 정지 후 q로 저장, ESC로 종료합니다.')

    def warn(self, message):
        """입력 거부 이유를 1초 간격으로 출력한다."""
        if time.monotonic() - self.warn_time >= 1:
            self.get_logger().warning(str(message))
            self.warn_time = time.monotonic()

    def on_image(self, color, info):
        """RGB/CameraInfo 동기화 콜백. 프레임/해상도/시각 검사 후 최신 영상만 보관한다."""
        try:
            if color.header.frame_id != self.args.camera_frame or info.header.frame_id != self.args.camera_frame:
                raise ValueError('RGB/CameraInfo의 optical frame이 --camera-frame과 다릅니다.')
            if (info.width, info.height) != (color.width, color.height):
                raise ValueError('RGB와 CameraInfo 해상도가 다릅니다.')
            now = self.get_clock().now().nanoseconds
            if any(not 0 <= now - Time.from_msg(msg.header.stamp).nanoseconds <= 500_000_000
                   for msg in (color, info)):
                raise ValueError('0.5초 이내의 영상이 필요합니다.')
            image = self.bridge.imgmsg_to_cv2(color, 'bgr8')
            if self.data['samples'] and [color.width, color.height] != self.data['image_size']:
                raise ValueError('촬영 중 해상도를 변경할 수 없습니다.')
            self.image = (image, color.header.stamp)
            self.received = time.monotonic()
        except (ValueError, RuntimeError, cv2.error, CvBridgeError) as exc:
            self.image = None
            self.warn(exc)

    def capture(self):
        """q 입력 시 PNG/TF 한 쌍을 저장한다. 코너/TF/최신 시각 실패는 False이며 저장하지 않는다."""
        if self.image is None or time.monotonic() - self.received > .5:
            self.warn('최신 영상이 없습니다.')
            return False
        image, stamp = self.image
        try:
            if not 0 <= self.get_clock().now().nanoseconds - Time.from_msg(stamp).nanoseconds <= 500_000_000:
                raise ValueError('영상이 오래되었습니다.')
            if find_corners(image, tuple(self.args.board)) is None:
                raise ValueError('10x7 내부 코너 전체가 보이도록 조정하세요. --board 설정도 확인하세요.')
            transform = self.buffer.lookup_transform(self.args.base_frame, self.args.mount_frame,
                                                     Time.from_msg(stamp))
            pose = transform_message_to_matrix(transform.transform)
            # 같은 자세를 반복 저장해도 손눈 보정의 회전축 다양성은 늘어나지 않는다.
            if self.data['samples'] and self.data['samples'][-1]['stamp'] == [stamp.sec, stamp.nanosec]:
                raise ValueError('같은 영상은 이미 저장했습니다.')
            filename = f'sample_{len(self.data["samples"]) + 1:04d}.png'
            target = self.args.output / filename
            ok, encoded = cv2.imencode('.png', image)
            if not ok:
                raise ValueError('PNG 변환 실패')
            with target.open('xb') as stream:
                stream.write(encoded.tobytes())
            self.data['image_size'] = [image.shape[1], image.shape[0]]
            self.data['samples'].append(dict(file_name=filename, stamp=[stamp.sec, stamp.nanosec],
                                             T_base_mount=pose.tolist()))
            # 임시 JSON을 같은 폴더에 저장한 뒤 교체해 기록 중단 시 기존 자료를 보존한다.
            temporary = self.args.output / 'calibrate_data.tmp'
            temporary.write_text(json.dumps(self.data, indent=2, ensure_ascii=False), encoding='utf-8')
            temporary.replace(self.args.output / 'calibrate_data.json')
            self.get_logger().info(f'SAVED {filename} ({len(self.data["samples"])} pairs)')
            return True
        except (ValueError, TransformException) as exc:
            self.warn(exc)
            return False


def main():
    """CLI 검사 → 영상 표시 → q 촬영 → ROS/GUI 정리. 파일 오류는 종료 코드 1이다."""
    parser = argparse.ArgumentParser(description='팔 장착 카메라의 수동 촬영. 로봇을 이동하지 않습니다.')
    parser.add_argument('--output', type=Path, required=True, help='새 데이터 폴더. 기존 폴더 거부')
    parser.add_argument('--mount-frame', required=True, help='예: fr3_link8. 보정과 검증에서 동일하게 사용')
    parser.add_argument('--base-frame', default='fr3_link0')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--color-topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--info-topic', default='/camera/camera/color/camera_info')
    parser.add_argument('--board', type=int, nargs=2, default=[10, 7], metavar=('COLS', 'ROWS'))
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    try:
        checkerboard_points(tuple(args.board), 25)
        if args.base_frame == args.mount_frame:
            raise ValueError('팔 장착 보정의 mount는 움직이는 로봇 링크여야 합니다.')
        for name in ('base_frame', 'mount_frame', 'camera_frame'):
            if not getattr(args, name).strip() or getattr(args, name).startswith('/'):
                raise ValueError('프레임 이름은 비어 있거나 /로 시작할 수 없습니다.')
        args.output = args.output.expanduser().resolve()
        rclpy.init(args=sys.argv)
        node = None
        try:
            node = Recorder(args)
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.01)
                if node.image is not None:
                    cv2.imshow(WINDOW, node.image[0])
                key = cv2.waitKey(1) & 0xff
                if key == 27:
                    break
                if key == ord('q'):
                    node.capture()
        finally:
            cv2.destroyAllWindows()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except (ValueError, OSError, RuntimeError, cv2.error) as exc:
        print(f'촬영 실패: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
