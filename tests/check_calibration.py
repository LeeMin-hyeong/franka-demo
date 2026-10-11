#!/usr/bin/env python3
"""로봇/ROS 없이 단위와 광학 좌표, 장착/베이스 변환의 방향을 검증한다.

합성 값만 사용한다. 잘못된 보정의 거부를 검사하지만 실제 캘리브레이션 정확도는
검증하지 않는다. 실행: python3 tests/check_calibration.py.
"""

import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np


root = Path(__file__).resolve().parents[1]
path = root / 'ros2_ws/src/fr3_moveit_demo/fr3_moveit_demo/calibration.py'
spec = importlib.util.spec_from_file_location('calibration', path)
cal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cal)


class CalibrationTests(unittest.TestCase):
    def test_depth_units_and_optical_axes(self):
        k = dict(fx=500, fy=500, ppx=0, ppy=0)
        mm = np.full((2, 2), 1000, dtype=np.uint16)
        m = np.full((2, 2), 1.0, dtype=np.float32)
        np.testing.assert_allclose(cal.pixel_to_camera_point(1, 1, mm, k), [0.002, 0.002, 1])
        np.testing.assert_allclose(cal.pixel_to_camera_point(1, 1, m, k, 1), [0.002, 0.002, 1])
        for bad in [0, -1, np.nan, np.inf]:
            self.assertIsNone(cal.pixel_to_camera_point(0, 0, np.array([[bad]]), k))
        self.assertIsNone(cal.pixel_to_camera_point(-1, 0, mm, k))
        with self.assertRaises(ValueError):
            cal.pixel_to_camera_point(0, 0, mm, dict(k, fx=0))

    def test_mount_chain_direction(self):
        # 카메라 x=1 → mount의 y=1 → base의 y=3. 역행렬을 곱하면 이 값과 다르다.
        mount_camera = np.array([[0, -1, 0, .1], [1, 0, 0, 0],
                                 [0, 0, 1, .2], [0, 0, 0, 1]], dtype=float)
        base_mount = np.eye(4)
        base_mount[:3, 3] = [1, 2, 3]
        np.testing.assert_allclose(cal.transform_point_to_base([1, 0, 0], base_mount @ mount_camera),
                                   [1.1, 3, 3.2])
        np.testing.assert_allclose(cal.transform_point_to_base([1, 0, 0], mount_camera), [.1, 1, .2])

    def test_file_translation_units(self):
        # 테스트 파일은 저장소 안의 임시 디렉터리에만 만들고 정리한다.
        with tempfile.TemporaryDirectory(dir=root) as tmp:
            assert Path(tmp).resolve().parent == root.resolve()
            filename = Path(tmp) / 'new_fr3.npy'
            transform = np.eye(4)
            transform[:3, 3] = [100, -200, 300]
            np.save(filename, transform)
            np.testing.assert_allclose(cal.load_calibration(filename, 'mm')[:3, 3], [.1, -.2, .3])
            np.testing.assert_allclose(cal.load_calibration(filename, 'm')[:3, 3], [100, -200, 300])
            with self.assertRaises(ValueError):
                cal.load_calibration(filename, 'auto')
            np.save(filename, np.array([{'unsafe_pickle': True}], dtype=object))
            with self.assertRaises(ValueError):
                cal.load_calibration(filename, 'm')

    def test_reject_invalid_transforms(self):
        bad = [np.zeros((4, 4)), np.eye(3), np.full((4, 4), np.nan)]
        reflected = np.eye(4)
        reflected[0, 0] = -1
        scaled = np.eye(4)
        scaled[0, 0] = 2
        bad.extend([reflected, scaled])
        for matrix in bad:
            with self.assertRaises(ValueError):
                cal.validate_transform(matrix)

    def test_tf_quaternion(self):
        tf = NS(rotation=NS(x=0, y=0, z=np.sqrt(.5), w=np.sqrt(.5)),
                translation=NS(x=1, y=2, z=3))
        matrix = cal.transform_message_to_matrix(tf)
        np.testing.assert_allclose(cal.transform_point_to_base([1, 0, 0], matrix), [1, 3, 3])
        tf.rotation = NS(x=0, y=0, z=0, w=0)
        with self.assertRaises(ValueError):
            cal.transform_message_to_matrix(tf)


if __name__ == '__main__':
    unittest.main()
