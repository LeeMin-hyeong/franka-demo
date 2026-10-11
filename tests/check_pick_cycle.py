#!/usr/bin/env python3
"""실행 중인 Fake 서버에서 합성 RGB-D/보정/TF → 집기 작업 → 원위치 복귀를 검사한다.

scripts/test_pick.sh가 별도 ROS 도메인의 서버를 준비한다. 실제 카메라, FCI, 접촉은 없다.
CLI/GUI와 같은 ClickCameraPoint 투영과 PickAndReturn 작업 코드를 사용한다.
"""

import importlib.util
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import numpy as np
import rclpy

from fr3_moveit_demo.pick_place import PickAndReturn, CancellableRobot, BASE

root = Path(__file__).resolve().parents[1]
examples = root / 'ros2_ws/src/fr3_moveit_demo/examples'
sys.path.insert(0, str(examples))
spec = importlib.util.spec_from_file_location('pick_example', examples / 'pick_and_return.py')
example = importlib.util.module_from_spec(spec)
spec.loader.exec_module(example)


def main():
    """카메라 선택을 합성하고 모든 단계의 측정/Scene을 확인한다. 실패는 AssertionError다."""
    gui = os.environ.get('FR3_PICK_GUI') == '1'
    args = NS(base_frame=BASE, mount_frame='fr3_link8', camera_frame='pick_test_color_optical_frame',
              color_topic='/fr3_pick_test/color', depth_topic='/fr3_pick_test/depth',
              info_topic='/fr3_pick_test/info', pixel=None if gui else [320, 240],
              execute=True, size=[.04, .03, .05], table_z=.20, clearance=.10, lift=.05, force=20.)
    args.matrix, args.camera_calibration = np.eye(4), None
    rclpy.init(args=[])
    node = example.SelectObject(args)
    example.demo_configuration(args, node)
    # NPY 저장/읽기로 실제 보정 파일과 같은 단위 규약을 검사한다.
    import tempfile
    from fr3_moveit_demo.calibration import load_calibration
    with tempfile.TemporaryDirectory(prefix='fr3-pick-') as folder:
        path = Path(folder) / 'synthetic_T_mount_camera.npy'
        np.save(path, args.matrix)
        args.matrix = load_calibration(path, 'm')
        node.mount_camera = args.matrix
    try:
        deadline = time.monotonic() + 10
        while node.selected is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            example.demo_images(node)
            if gui:
                example.cv2.imshow(example.WINDOW, node.images[0])
                example.cv2.waitKey(10)
            node.on_click(example.cv2.EVENT_LBUTTONDOWN, 320, 240, 0, None)
        assert node.selected is not None, 'Synthetic calibrated camera point was not received.'
        top = node.selected
        np.testing.assert_allclose(top, [.45, 0., .25], atol=1e-6)
        print(f'CALIBRATED SELECT PASS: {top} m (synthetic calibration, image-time robot TF)')
    finally:
        node.destroy_node()
        if gui:
            example.cv2.destroyAllWindows()
        rclpy.shutdown()
    with CancellableRobot(args=[], backend='fake') as robot:
        task = PickAndReturn(robot, duration=1.)
        try:
            report = task.run(top, [.04, .03, .05], .20, clearance=.10, lift=.05, force=20., hold=.1)
            stages = {stage['stage']: stage for stage in report['stages']}
            assert list(stages) == ['initial', 'approach', 'at_object', 'grasped', 'lifted', 'lowered', 'released', 'returned']
            assert not stages['initial']['attached'] and not stages['released']['attached']
            assert stages['grasped']['attached'] and stages['lifted']['attached'] and stages['lowered']['attached']
            assert abs(stages['lifted']['tcp'][2] - stages['grasped']['tcp'][2] - .05) < .003
            np.testing.assert_allclose(stages['lowered']['tcp'], report['object_center'], atol=.003)
            assert abs(stages['grasped']['width'] - .03) < .0002
            assert abs(stages['released']['width'] - .05) < .0002
            assert report['return_tcp_error_m'] < .003
            np.testing.assert_allclose(report['placed_scene_center'], report['object_center'], atol=.003)
            assert not report['physical_grasp_verified']
            output = root / 'ros2_ws/pick-report.json'
            output.write_text(json.dumps(report, indent=2), encoding='utf-8')
            print('PICK CYCLE PASS: measured TCP lift/lower, attach/detach, finger width and original arm joints.')
            print(f'Report: {output}. Physical object/contact/calibration accuracy NOT TESTED.')
        finally:
            task.close()


if __name__ == '__main__':
    main()
