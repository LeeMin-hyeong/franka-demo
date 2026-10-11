"""MacGyvBot의 깊이 투영과 좌표 변환 계산을 FR3에서 재사용한다.

원본: MacGyvBot/macgyvbot의 depth_projection.py, click_base_point.py.
입력: RGB에 정렬된 깊이, 카메라 내부 파라미터, 새 장착 환경의 4x4 보정 행렬.
출력: 카메라 또는 로봇 베이스 기준 3D 좌표(m). ROS/FCI 연결과 이동은 하지 않는다.
기존 T_gripper2camera.npy 소비 코드와 같은 행렬 방향은 T_mount_camera이다.
즉 카메라의 점을 장착 기준 프레임으로 변환한다. 파일 이름만 보고 역행렬을 취하지 않는다.
FR3에는 MacGyvBot의 측정값을 복사하지 않고 별도로 측정한 파일을 사용한다.
"""

from pathlib import Path

import numpy as np


def validate_transform(matrix):
    """유한한 SE(3) 행렬을 반환한다. 이동 단위는 호출자가 통일하며 형식/회전 오류는 ValueError다."""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('보정 행렬은 유한한 4x4 배열이어야 합니다.')
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5, rtol=0)):
        raise ValueError('회전은 SO(3), 마지막 행은 [0, 0, 0, 1]이어야 합니다.')
    return matrix


def load_calibration(path, translation_unit):
    """T_mount_camera.npy를 읽어 m로 통일한다. 단위는 m/mm를 반드시 명시한다.

    MacGyvBot 소비 코드는 이동 성분을 1000으로 나누었다. 이 규칙을 mm일 때만
    적용해 m로 저장한 새 보정 파일이 다시 축소되는 것을 막는다. pickle은 허용하지 않는다.
    파일 누락/읽기 실패는 해당 파일 예외, 단위/행렬 오류는 ValueError다.
    """
    if translation_unit not in ('m', 'mm'):
        raise ValueError('이동 성분 단위는 m 또는 mm로 명시하세요.')
    matrix = np.array(np.load(Path(path).expanduser(), allow_pickle=False), dtype=float)
    validate_transform(matrix)
    if translation_unit == 'mm':
        matrix[:3, 3] /= 1000.0
    return matrix


def pixel_to_camera_point(u, v, depth_image, intrinsics, depth_scale=0.001):
    """MacGyvBot의 핀홀 투영 계산. 유효 깊이는 XYZ(m), 무효 깊이는 None을 반환한다.

    u/v는 정수 픽셀, intrinsics는 fx/fy/ppx/ppy, depth_scale은 원시 깊이→m 계수다.
    색 영상과 깊이는 정렬되어 있어야 한다. 왜곡된 RGB는 호출자가 별도로 보정한다.
    배열/내부 파라미터/단위 오류는 ValueError이며 영상 밖 픽셀은 None이다.
    """
    depth_image = np.asarray(depth_image)
    if depth_image.ndim != 2 or not np.isfinite(depth_scale) or depth_scale <= 0:
        raise ValueError('깊이는 2차원 배열이며 depth_scale은 유한한 양수여야 합니다.')
    values = [intrinsics[name] for name in ('fx', 'fy', 'ppx', 'ppy')]
    if not np.isfinite(values).all() or values[0] <= 0 or values[1] <= 0:
        raise ValueError('카메라 내부 파라미터와 양수 초점거리가 필요합니다.')
    if not isinstance(u, (int, np.integer)) or not isinstance(v, (int, np.integer)):
        raise ValueError('픽셀 좌표는 정수여야 합니다.')
    height, width = depth_image.shape
    if not (0 <= u < width and 0 <= v < height):
        return None
    z_raw = float(depth_image[v, u])
    if not np.isfinite(z_raw) or z_raw <= 0:
        return None

    # MacGyvBot에서 사용한 계산식을 유지한다. z는 광학 축 방향 깊이다.
    z_m = z_raw * depth_scale
    cam_x = (u - intrinsics['ppx']) * z_m / intrinsics['fx']
    cam_y = (v - intrinsics['ppy']) * z_m / intrinsics['fy']
    if not np.isfinite([cam_x, cam_y, z_m]).all():
        return None
    return float(cam_x), float(cam_y), float(z_m)


def transform_point_to_base(cam_xyz, base_to_camera):
    """MacGyvBot의 동차좌표 곱으로 카메라 XYZ(m)를 베이스 XYZ(m)로 변환한다.

    base_to_camera는 T_base_camera, 즉 camera→base 변환이다. 입력 오류는 ValueError다.
    카메라가 팔에 있으면 T_base_mount @ T_mount_camera로 만들고,
    외부 고정 카메라라면 측정한 T_base_camera를 직접 사용한다.
    """
    validate_transform(base_to_camera)
    point = np.asarray(cam_xyz, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('카메라 좌표는 유한한 XYZ(m)여야 합니다.')
    coord = np.append(point, 1.0)
    return (base_to_camera @ coord)[:3]


def transform_message_to_matrix(transform):
    """ROS Transform의 quaternion/이동(m)을 SE(3)로 바꾼다. 영 quaternion은 거부한다.

    MacGyvBot click_base_point.py의 정규화 공식을 재사용한다. ROS import는 필요 없다.
    """
    q = np.array([transform.rotation.x, transform.rotation.y,
                  transform.rotation.z, transform.rotation.w], dtype=float)
    norm = float(np.dot(q, q))
    if not np.isfinite(q).all() or norm <= 1e-12:
        raise ValueError('유효한 TF quaternion이 필요합니다.')
    x, y, z, w = q * np.sqrt(2.0 / norm)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - y*y - z*z, x*y - w*z, x*z + w*y],
        [x*y + w*z, 1 - x*x - z*z, y*z - w*x],
        [x*z - w*y, y*z + w*x, 1 - x*x - y*y],
    ]
    matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    return validate_transform(matrix)
