#!/usr/bin/env python3
"""CV 창에서 직육면체 상단 중심을 선택해 집기/원위치 내려놓기를 수행한다.

입력: 새 손눈 보정 파일, 물체 XYZ 크기[m], 작업대 베이스 Z[m], 힘[N].
출력: 보정된 베이스 XYZ, 단계별 측정 상태, 선택한 보고서 JSON.
카메라/GUI는 주 스레드, 단일 Robot 작업은 별도 스레드에서 수행한다.
ClickCameraPoint의 시각/깊이/TF 검사를 재사용하고 PickAndReturn에 작업 순서를 맡긴다.
기본은 좌표 확인이며 --execute를 명시해야 이동한다. 자동 물체 검출은 하지 않는다.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args

from click_camera_point import ClickCameraPoint, load_camera_metadata, WINDOW
from fr3_moveit_demo.calibration import load_calibration, transform_message_to_matrix
from fr3_moveit_demo.pick_place import PickAndReturn, validate_selection, CancellableRobot


class SelectObject(ClickCameraPoint):
    """같은 보정/동기화 검사를 사용한다. 작업이 시작되면 추가 클릭을 받지 않는다."""

    def __init__(self, args):
        super().__init__(args)
        self.selected = None
        self.busy = False
        self.get_logger().info('이 창은 --execute일 때만 선택한 물체의 집기 작업을 실행합니다.')

    def on_click(self, event, x, y, flags, userdata):
        """유효한 상단 중심만 작업 큐에 저장한다. ROS 콜백 안에서는 이동하지 않는다."""
        if event != cv2.EVENT_LBUTTONDOWN or self.busy:
            return
        point = self.project(int(x), int(y))
        if point is None:
            return
        top = [point.point.x, point.point.y, point.point.z]
        try:
            validate_selection(top, self.args.size, self.args.table_z,
                               self.args.clearance, self.args.lift, self.args.force)
        except ValueError as error:
            self.warn(error)
            return
        self.selected = top
        self.get_logger().info(f'SELECTED top={top} m. execute={self.args.execute}')
        if self.args.execute:
            self.busy = True


def run_cycle(args, top):
    """작업 스레드에서 Robot을 생성/정리한다. 오류 후 자동 복귀나 해제를 하지 않는다."""
    with CancellableRobot(args=[], backend=args.backend, robot_ip=args.robot_ip) as robot:
        robot.cancel_event = args.cancel
        task = PickAndReturn(robot, duration=args.duration)
        try:
            report = task.run(top, args.size, args.table_z, args.clearance, args.lift, args.force, args.hold)
            if args.report:
                with Path(args.report).expanduser().open('x', encoding='utf-8') as output:
                    json.dump(report, output, indent=2)
            print('PICK AND RETURN COMPLETE: block released; initial arm joints restored.')
            return report
        finally:
            task.close()


def demo_configuration(args, node):
    """가상 카메라를 정의한다. 측정 보정이 아닌 알려진 행렬이며 real에는 사용하지 않는다."""
    import time
    from rclpy.time import Time
    from tf2_ros import TransformException
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        try:
            tf = node.buffer.lookup_transform(args.base_frame, args.mount_frame, Time())
        except TransformException:
            continue
        base_mount = transform_message_to_matrix(tf.transform)
        base_camera = np.diag([1., -1., -1., 1.])
        base_camera[:3, 3] = [.45, 0., .65]
        args.matrix = np.linalg.inv(base_mount) @ base_camera
        node.mount_camera = args.matrix
        return
    raise TimeoutError('Fake robot mount TF unavailable.')


def demo_images(node):
    """합성 RGB/정렬 깊이/CameraInfo를 기존 콜백에 넣는다. 물체 바깥 깊이는 무효다."""
    from sensor_msgs.msg import CameraInfo
    rgb = np.full((480, 640, 3), 235, np.uint8)
    cv2.rectangle(rgb, (295, 220), (345, 260), (70, 150, 70), -1)
    cv2.putText(rgb, 'FAKE: click TOP CENTER of the block', (25, 70),
                cv2.FONT_HERSHEY_SIMPLEX, .7, (30, 30, 30), 2)
    depth = np.zeros((480, 640), np.float32)
    depth[220:261, 295:346] = .4
    color = node.bridge.cv2_to_imgmsg(rgb, 'bgr8')
    aligned = node.bridge.cv2_to_imgmsg(depth, '32FC1')
    info = CameraInfo(width=640, height=480, k=[500., 0., 320., 0., 500., 240., 0., 0., 1.],
                      d=[0.] * 5, distortion_model='plumb_bob')
    from rclpy.duration import Duration
    # 합성 센서의 50 ms 지연을 넣어 같은 시각의 TF가 구독 버퍼에 도착할 시간을 준다.
    stamp = (node.get_clock().now() - Duration(nanoseconds=50_000_000)).to_msg()
    for message in (color, aligned, info):
        message.header.stamp = stamp
        message.header.frame_id = node.args.camera_frame
    node.on_images(color, aligned, info)


def main():
    """CLI 검사 → CV 선택 → 한 번의 작업 → 보고서/정리. ESC는 진행 작업의 중단도 요청한다."""
    parser = argparse.ArgumentParser(description='클릭한 직육면체를 들었다 원위치에 놓고 시작 자세로 복귀')
    parser.add_argument('--demo', action='store_true', help='Fake 서버 + 합성 RGB-D/알려진 보정 행렬')
    parser.add_argument('--backend', choices=['fake', 'real'], default='fake')
    parser.add_argument('--robot-ip')
    parser.add_argument('--execute', action='store_true', help='한 번의 선택으로 전체 작업 실행')
    parser.add_argument('--calibration')
    parser.add_argument('--translation-unit', choices=['m', 'mm'], default='mm')
    parser.add_argument('--mount-frame', default='fr3_link8')
    parser.add_argument('--base-frame', default='fr3_link0')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--color-topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--depth-topic', default='/camera/camera/aligned_depth_to_color/image_raw')
    parser.add_argument('--info-topic', default='/camera/camera/color/camera_info')
    parser.add_argument('--size', nargs=3, type=float, metavar=('X', 'Y', 'Z'), help='직육면체 길이/집는 폭/높이 [m]')
    parser.add_argument('--table-z', type=float, help='실측한 작업대 상단의 베이스 Z [m]')
    parser.add_argument('--force', type=float, help='명시적 실물 집기 힘 [N]')
    parser.add_argument('--clearance', type=float, default=.10)
    parser.add_argument('--lift', type=float, default=.05)
    parser.add_argument('--hold', type=float, default=1.)
    parser.add_argument('--duration', type=float, default=5.)
    parser.add_argument('--report', help='새 JSON 보고서 경로. 기존 파일 거부')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    args.pixel = None
    node, future = None, None
    try:
        if args.base_frame != 'fr3_link0' or args.mount_frame != 'fr3_link8':
            raise ValueError('This first task uses fr3_link0 and a camera calibrated to fr3_link8.')
        if args.report and Path(args.report).expanduser().exists():
            raise ValueError('Report exists. Choose a new path before any movement.')
        if args.demo:
            if args.backend != 'fake' or args.robot_ip or args.calibration:
                raise ValueError('--demo only accepts Fake Hardware and synthetic calibration.')
            args.size = args.size or [.04, .03, .05]
            args.table_z = .20 if args.table_z is None else args.table_z
            args.force = 20. if args.force is None else args.force
            args.matrix, args.camera_calibration = np.eye(4), None
        else:
            if args.backend != 'real' or not args.robot_ip or not args.calibration:
                raise ValueError('Physical camera mode requires --backend real --robot-ip and --calibration.')
            if args.size is None or args.table_z is None or args.force is None:
                raise ValueError('Measure --size/--table-z and explicitly choose --force before selection.')
            if args.execute:
                # 같은 실물 PC 조건을 직접 Python 진입점에서도 확인한다.
                import subprocess
                root = Path(__file__).resolve().parents[4]
                subprocess.run(['bash', '-c', 'source "$1/scripts/common.sh"; demo_require_real_pc',
                                'fr3-pick', str(root)], check=True)
            args.matrix = load_calibration(args.calibration, args.translation_unit)
            args.camera_calibration = load_camera_metadata(args)
            if args.camera_calibration is None:
                raise ValueError('Use the new FR3 calibration with matching JSON metadata for this task.')
        # 클릭 전에 설정값을 확인하고 상단 좌표는 실제 깊이로 다시 검사한다.
        validate_selection([.45, 0., args.table_z + args.size[2]], args.size,
                           args.table_z, args.clearance, args.lift, args.force)
        rclpy.init(args=[])
        node = SelectObject(args)
        if args.demo:
            demo_configuration(args, node)
        with ThreadPoolExecutor(max_workers=1) as pool:
            try:
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=.01)
                    if args.demo and not node.busy:
                        demo_images(node)
                    if node.images is not None:
                        image = copy.copy(node.images[0])
                        text = 'RUNNING: clicks locked; ESC requests stop' if node.busy else 'CLICK TOP CENTER (XYZ in metres)'
                        cv2.putText(image, text, (20, 440), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 180), 2)
                        cv2.imshow(WINDOW, image)
                    if cv2.waitKey(1) == 27:
                        if future is not None and not future.done():
                            # Robot을 소유한 작업 스레드에 중단을 요청한다.
                            args.cancel.set()
                            return 130
                        break
                    if node.selected is not None and args.execute and future is None:
                        import threading
                        args.cancel = threading.Event()
                        future = pool.submit(run_cycle, args, node.selected)
                    if future is not None and future.done():
                        future.result()
                        return 0
            finally:
                if future is not None and not future.done():
                    args.cancel.set()
    except KeyboardInterrupt:
        if future is not None and not future.done():
            args.cancel.set()
        print('집기 작업 중단 요청. 실제 유지/정지 상태를 확인하세요. 손을 자동으로 열지 않습니다.', file=sys.stderr)
        return 130
    except Exception as error:
        print(f'Pick task stopped: {error}. Inspect robot and scene; no automatic release/retry.', file=sys.stderr)
        return 1
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
