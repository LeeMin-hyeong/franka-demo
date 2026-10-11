"""Calibration_Tutorial.zip/handeye_calibration.py의 손눈 보정 계산을 재사용한다.

입력: 정지한 체커보드 이미지, 촬영 시각의 FR3 T_base_mount(m), 내부 코너/칸 크기(mm).
출력: 카메라→장착 링크 보정 T_mount_camera(mm), 내부 파라미터와 검증 지표.
원본의 findChessboardCorners/cornerSubPix/calibrateCamera/solvePnP/PARK를 유지한다.
두산 ZYZ Euler 대신 ROS TF 행렬을 받으며 원본의 고정 25 mm는 square_size로 수정했다.
이 모듈은 ROS/FCI를 import하거나 로봇을 이동하지 않는다.
"""

import cv2
import numpy as np

from .calibration import validate_transform


def checkerboard_points(board_size, square_size):
    """내부 코너 (열, 행), 칸 크기 mm로 체커보드의 3D 점을 반환한다. 잘못된 값은 ValueError다."""
    if (len(board_size) != 2 or any(not isinstance(n, int) or n < 2 for n in board_size)
            or not np.isfinite(square_size) or square_size <= 0):
        raise ValueError('내부 코너는 각 축 2개 이상, 칸 크기는 양수 mm여야 합니다.')
    points = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    points[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_size
    return points


def find_corners(image, board_size):
    """원본의 체커보드 검출과 sub-pixel 보정. 검출 실패는 None이다."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, board_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
        + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, .001))


def find_checkerboard_pose(image, board_size, square_size, camera_matrix, dist_coeffs):
    """원본 solvePnP로 T_camera_board를 구한다. 회전/이동(mm)/오차(px), 검출 실패는 None이다.

    PnP 결과는 board→camera 변환이다. 손눈 보정 입력에 역행렬로 뒤집지 않는다.
    """
    corners = find_corners(image, board_size)
    if corners is None:
        return None
    points = checkerboard_points(board_size, square_size)
    success, rvec, tvec = cv2.solvePnP(points, corners, camera_matrix, dist_coeffs)
    if not success or tvec[2, 0] <= 0:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    projected, _ = cv2.projectPoints(points, rvec, tvec, camera_matrix, dist_coeffs)
    error = float(np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=2))))
    return rotation, tvec, error


def calibrate_camera_from_chessboard(images, board_size, square_size):
    """원본 calibrateCamera로 K/D를 추정한다. 입력은 동일 해상도 BGR 영상, 출력 K/D/RMS(px)다.

    코너를 찾은 영상이 6개 미만이거나 비유한 결과면 ValueError다. 실제 정확도는 별도 검증한다.
    """
    points = checkerboard_points(board_size, square_size)
    if len({image.shape[:2] for image in images}) != 1:
        raise ValueError('모든 이미지의 해상도가 같아야 합니다.')
    valid = [(image, find_corners(image, board_size)) for image in images]
    valid = [(image, corners) for image, corners in valid if corners is not None]
    if len(valid) < 6:
        raise ValueError('체커보드를 검출한 이미지가 최소 6개 필요합니다. 15~25개를 권장합니다.')
    sizes = {image.shape[:2] for image, _ in valid}
    if len(sizes) != 1:
        raise ValueError('모든 이미지의 해상도가 같아야 합니다.')
    rms, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
        [points] * len(valid), [corners for _, corners in valid], valid[0][0].shape[1::-1], None, None)
    if not np.isfinite(rms) or not np.isfinite(camera_matrix).all() or not np.isfinite(distortion).all():
        raise ValueError('카메라 내부 보정 결과가 유한하지 않습니다.')
    return camera_matrix, distortion, float(rms)


def solve_handeye(base_mount_m, camera_board_mm):
    """원본 PARK 계산으로 T_mount_camera(mm)를 반환한다. 배열 길이/자세 다양성 오류는 ValueError다.

    입력 TF는 mount→base, PnP는 board→camera다. ROS 이동 성분만 m→mm로 바꾼다.
    같은 축의 회전만 있으면 손눈 변환을 결정하기 어려우므로 서로 다른 회전축을 검사한다.
    """
    if len(base_mount_m) != len(camera_board_mm) or len(base_mount_m) < 6:
        raise ValueError('같은 개수의 로봇/표적 pose가 최소 6쌍 필요합니다.')
    base = [validate_transform(t).copy() for t in base_mount_m]
    board = [validate_transform(t) for t in camera_board_mm]
    axes = [cv2.Rodrigues(base[0][:3, :3].T @ t[:3, :3])[0].flatten() for t in base[1:]]
    singular = np.linalg.svd(np.array(axes), compute_uv=False)
    if singular[1] < 1e-3:
        raise ValueError('로봇 자세에 서로 다른 회전축이 필요합니다. 같은 축/평행 이동만으로 보정하지 마세요.')
    rotation, translation = cv2.calibrateHandEye(
        [t[:3, :3] for t in base], [t[:3, 3].reshape(3, 1) * 1000 for t in base],
        [t[:3, :3] for t in board], [t[:3, 3].reshape(3, 1) for t in board],
        method=cv2.CALIB_HAND_EYE_PARK)
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rotation, translation.flatten()
    return validate_transform(result)


def consistency_errors(base_mount_m, camera_board_mm, mount_camera_mm):
    """고정 체커보드의 베이스 pose 일관성을 이동 RMS(mm)/회전 RMS(deg)로 반환한다.

    학습에 사용한 데이터의 일관성이며 독립 검증 정확도가 아니다. 위치 기준은 평균,
    회전 기준은 첫 표적 pose다. 자동 합격 판정이나 로봇 이동 허용에 사용하지 않는다.
    """
    poses = []
    for base, board in zip(base_mount_m, camera_board_mm):
        base_mm = base.copy()
        base_mm[:3, 3] *= 1000
        poses.append(base_mm @ mount_camera_mm @ board)
    positions = np.array([t[:3, 3] for t in poses])
    angles = [np.linalg.norm(cv2.Rodrigues(poses[0][:3, :3].T @ t[:3, :3])[0]) for t in poses]
    return (float(np.sqrt(np.mean(np.sum((positions - positions.mean(axis=0))**2, axis=1)))),
            float(np.degrees(np.sqrt(np.mean(np.array(angles)**2)))))
