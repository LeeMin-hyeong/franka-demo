#!/usr/bin/env python3
"""합성 체커보드/로봇 pose로 원본 PARK 재사용과 CLI 저장을 검증한다. FCI는 호출하지 않는다."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np

from fr3_moveit_demo.handeye import checkerboard_points, consistency_errors, solve_handeye


ROOT = Path(__file__).resolve().parents[1]


def transform(rotation, translation):
    """합성 Rodrigues 회전(rad)과 이동으로 SE(3) 정답을 만든다."""
    result = np.eye(4)
    result[:3, :3] = cv2.Rodrigues(np.asarray(rotation, dtype=float))[0]
    result[:3, 3] = translation
    return result


def fixture():
    """고정 표적과 알려진 카메라 장착 변환에서 여러 시점의 pose를 역산한다. 실물 목표가 아니다."""
    rng = np.random.default_rng(42)
    mount_camera = transform([.03, -.1, .04], [35, -25, 75])
    base_board = transform([-.2, .3, .5], [500, 50, 400])
    bases, boards = [], []
    for _ in range(20):
        board = transform(rng.uniform([-.5, -.5, -.2], [.5, .5, .2]), [0, 0, 0])
        # 표적 중심이 영상 중앙 근처에 오도록 이동한다.
        board[:3, 3] = [-20, 10, rng.uniform(750, 1000)] - board[:3, :3] @ [112.5, 75, 0]
        base = base_board @ np.linalg.inv(board) @ np.linalg.inv(mount_camera)
        base[:3, 3] /= 1000
        bases.append(base)
        boards.append(board)
    return mount_camera, bases, boards


def render_board(pose, matrix):
    """11x8 칸을 직접 투영해 10x7 내부 코너의 BGR 영상을 만든다. 텍스처/실물 영상은 사용하지 않는다."""
    scale = 3
    image = np.full((720 * scale, 960 * scale, 3), 255, np.uint8)
    rvec = cv2.Rodrigues(pose[:3, :3])[0]
    for y in range(8):
        for x in range(11):
            if (x + y) % 2:
                continue
            corners = np.array([[x-1, y-1, 0], [x, y-1, 0], [x, y, 0], [x-1, y, 0]], dtype=float) * 25
            pixels = cv2.projectPoints(corners, rvec, pose[:3, 3], matrix, np.zeros(5))[0]
            cv2.fillConvexPoly(image, np.rint(pixels.reshape(-1, 2) * scale).astype(np.int32), (0, 0, 0))
    return cv2.resize(image, (960, 720), interpolation=cv2.INTER_AREA)


class HandeyeTests(unittest.TestCase):
    def test_square_size_and_invalid_input(self):
        points = checkerboard_points((10, 7), 30)
        np.testing.assert_allclose(points[-1], [270, 180, 0])
        for size, square in [((1, 7), 25), ((10, 7), 0), ((10, 7), float('nan'))]:
            with self.assertRaises(ValueError):
                checkerboard_points(size, square)

    def test_park_direction_and_units(self):
        expected, bases, boards = fixture()
        result = solve_handeye(bases, boards)
        np.testing.assert_allclose(result, expected, atol=1e-8)
        position, rotation = consistency_errors(bases, boards, result)
        self.assertLess(position, 1e-7)
        self.assertLess(rotation, 1e-7)

    def test_degenerate_and_missing_poses(self):
        _, bases, boards = fixture()
        for bad in (bases[:3], bases[:-1], [np.eye(4)] * len(boards)):
            with self.assertRaises(ValueError):
                solve_handeye(bad, boards)
        single_axis = [transform([0, 0, i*.1], [i*.01, 0, 0]) for i in range(len(boards))]
        with self.assertRaises(ValueError):
            solve_handeye(single_axis, boards)

    def test_image_to_saved_calibration_cli(self):
        expected, bases, boards = fixture()
        matrix = np.array([[950., 0, 480], [0, 950, 360], [0, 0, 1]])
        with tempfile.TemporaryDirectory(prefix='handeye-test-', dir=ROOT) as directory:
            data_root = Path(directory)
            self.assertEqual(data_root.resolve().parent, ROOT)
            samples = []
            for i, (base, board) in enumerate(zip(bases, boards)):
                filename = f'sample_{i:04d}.png'
                self.assertTrue(cv2.imwrite(str(data_root / filename), render_board(board, matrix)))
                samples.append(dict(file_name=filename, T_base_mount=base.tolist()))
            data = dict(translation_unit='m', pose_convention='T_base_mount', base_frame='fr3_link0',
                        mount_frame='fr3_link8', camera_frame='camera_color_optical_frame', samples=samples)
            (data_root / 'calibrate_data.json').write_text(json.dumps(data), encoding='utf-8')
            command = [sys.executable, str(ROOT / 'calibration/solve.py'), '--data', directory]
            run = subprocess.run(command, capture_output=True, text=True, timeout=90)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            result = np.load(data_root / 'T_gripper2camera.npy', allow_pickle=False)
            np.testing.assert_allclose(result[:3, 3], expected[:3, 3], atol=5)
            np.testing.assert_allclose(result[:3, :3], expected[:3, :3], atol=.015)
            metadata = json.loads((data_root / 'T_gripper2camera.json').read_text())
            self.assertEqual(metadata['translation_unit'], 'mm')
            self.assertEqual(metadata['transform_convention'], 'T_mount_camera')
            self.assertEqual(len(metadata['valid_images']), 20)
            self.assertLess(metadata['camera_rms_px'], 1)
            self.assertLess(metadata['board_position_rms_mm'], 5)
            second = subprocess.run(command, capture_output=True, text=True, timeout=90)
            self.assertEqual(second.returncode, 1)
            self.assertIn('덮어쓰지', second.stderr)


if __name__ == '__main__':
    unittest.main()
