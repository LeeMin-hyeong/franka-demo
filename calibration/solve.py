#!/usr/bin/env python3
"""Calibration_Tutorial의 체커보드→PnP→PARK 손눈 보정을 FR3 기록 파일에 실행한다.

입력: record.py의 JSON/이미지, 코너 수와 칸 크기(mm).
출력: 기존 이름 T_gripper2camera.npy(mm)와 프레임/내부 파라미터/잔차 JSON.
원본의 두산 ZYZ 입력과 자동 pick 코드는 사용하지 않는다. 모든 계산은 오프라인이다.
"""

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

from fr3_moveit_demo.handeye import (
    calibrate_camera_from_chessboard, consistency_errors, find_checkerboard_pose, solve_handeye,
)


def main():
    """입력을 검사하고 원본 계산 순서로 저장한다. 기존 결과 덮어쓰기/검출 부족은 실패다."""
    parser = argparse.ArgumentParser(description='팔 장착 FR3 카메라의 오프라인 손눈 보정')
    parser.add_argument('--data', type=Path, required=True, help='record.py 출력 폴더')
    parser.add_argument('--board', nargs=2, type=int, default=[10, 7], metavar=('COLS', 'ROWS'))
    parser.add_argument('--square-mm', type=float, default=25.0)
    parser.add_argument('--output', type=Path, help='결과 .npy. 기본: 데이터 폴더/T_gripper2camera.npy')
    args = parser.parse_args()
    try:
        data_root = args.data.expanduser().resolve()
        data = json.loads((data_root / 'calibrate_data.json').read_text(encoding='utf-8'))
        if data.get('translation_unit') != 'm' or data.get('pose_convention') != 'T_base_mount':
            raise ValueError('FR3 record.py 형식(T_base_mount, m)이 필요합니다. 두산 Euler pose를 넣지 마세요.')
        for key in ('base_frame', 'mount_frame', 'camera_frame'):
            if not isinstance(data.get(key), str) or not data[key]:
                raise ValueError('기록 파일에 기준 프레임이 없습니다.')
        samples = data['samples']
        images, poses = [], []
        for sample in samples:
            filename = (data_root / sample['file_name']).resolve()
            if filename.parent != data_root:
                raise ValueError('이미지는 데이터 폴더 안에 있어야 합니다.')
            image = cv2.imread(str(filename))
            if image is None:
                raise ValueError(f'이미지를 읽지 못했습니다: {filename.name}')
            images.append(image)
            poses.append(np.asarray(sample['T_base_mount'], dtype=float))
        matrix, distortion, rms = calibrate_camera_from_chessboard(images, tuple(args.board), args.square_mm)
        valid_base, valid_board, errors, names = [], [], [], []
        for image, pose, sample in zip(images, poses, samples):
            result = find_checkerboard_pose(image, tuple(args.board), args.square_mm, matrix, distortion)
            if result is None:
                continue
            rotation, translation, error = result
            board = np.eye(4)
            board[:3, :3], board[:3, 3] = rotation, translation.flatten()
            valid_base.append(pose)
            valid_board.append(board)
            errors.append(error)
            names.append(sample['file_name'])
        result = solve_handeye(valid_base, valid_board)
        position_rms, rotation_rms = consistency_errors(valid_base, valid_board, result)
        output = (args.output or data_root / 'T_gripper2camera.npy').expanduser()
        if output.suffix != '.npy':
            raise ValueError('출력 확장자는 .npy여야 합니다.')
        metadata = output.with_suffix('.json')
        if output.exists() or metadata.exists():
            raise ValueError('기존 결과는 덮어쓰지 않습니다. 새 --output 파일명을 지정하세요.')
        output.parent.mkdir(parents=True, exist_ok=True)
        report = {key: data[key] for key in ('base_frame', 'mount_frame', 'camera_frame')}
        report.update(translation_unit='mm', transform_convention='T_mount_camera', method='CALIB_HAND_EYE_PARK',
                      camera_matrix=matrix.tolist(), distortion=distortion.flatten().tolist(),
                      image_size=[images[0].shape[1], images[0].shape[0]], valid_images=names,
                      camera_rms_px=rms, pnp_rms_px=errors,
                      board_position_rms_mm=position_rms, board_rotation_rms_deg=rotation_rms)
        # 배타적으로 생성해 기존 보정 결과를 덮어쓰지 않는다.
        with output.open('xb') as stream:
            np.save(stream, result)
        with metadata.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2, ensure_ascii=False)
        print(f'SAVED {output}\nT_mount_camera (translation mm):\n{result}')
        print(f'camera RMS={rms:.3f} px, board consistency={position_rms:.3f} mm / {rotation_rms:.3f} deg')
        print('학습 자료의 잔차입니다. 실제 위치를 별도로 측정해 검증하세요. 로봇 이동은 수행하지 않았습니다.')
    except (ValueError, KeyError, TypeError, OSError, cv2.error) as exc:
        print(f'보정 실패: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
