"""很小的坐标计算工具。

头部相机没有发布 TF，所以这里直接使用 Wanda 2026-08-09 的手眼标定结果。
"""

import math

T_LINK_HEAD1_CAMERA = [
    [
        [0.009332943656, -0.289703491070, 0.957070939599, 0.096377898213],
        [-0.999939471431, -0.008280843843, 0.007244383974, 0.011213356032],
        [0.005826631670, -0.957080620892, -0.289763240382, 0.082311965994],
        [0.0, 0.0, 0.0, 1.0],
    ]
][0]


def _transform(rotation=None, translation=None):
    result = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    if rotation is not None:
        for row in range(3):
            for column in range(3):
                result[row][column] = rotation[row][column]
    if translation is not None:
        for row in range(3):
            result[row][3] = translation[row]
    return result


def _rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _rotation_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _matrix_multiply(left, right):
    return [
        [sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4)]
        for row in range(4)
    ]


def _matrix_point(matrix, point):
    return [sum(matrix[row][column] * point[column] for column in range(4)) for row in range(4)]


def camera_point_to_base(point_camera, body_m, head_yaw, head_pitch):
    """把相机光学坐标中的 XYZ 换成 base_link 中的 XYZ。"""
    tiny_base_yaw = -0.00001283
    base_to_body = _transform(
        _rotation_z(tiny_base_yaw), (0.0, -0.000025184, 0.985 + body_m)
    )
    body_to_head0 = _transform(_rotation_z(head_yaw), (-0.00999, 0.0, 0.2215))
    head0_to_head1 = _transform(_rotation_y(head_pitch), (0.0, 0.0, 0.05345))
    base_to_camera = _matrix_multiply(base_to_body, body_to_head0)
    base_to_camera = _matrix_multiply(base_to_camera, head0_to_head1)
    base_to_camera = _matrix_multiply(base_to_camera, T_LINK_HEAD1_CAMERA)
    return _matrix_point(base_to_camera, [*point_camera, 1.0])[:3]


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def rotate_by_quaternion(point, quaternion):
    """用 ROS 顺序 xyzw 的四元数旋转一个三维点。"""
    x, y, z, w = quaternion
    px, py, pz = (float(value) for value in point)

    # q * p * q^-1 展开的写法，避免额外数学库。
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return [
        px + w * tx + (y * tz - z * ty),
        py + w * ty + (z * tx - x * tz),
        pz + w * tz + (x * ty - y * tx),
    ]


def apply_ros_transform(point, transform):
    t = transform.translation
    q = transform.rotation
    rotated = rotate_by_quaternion(point, (q.x, q.y, q.z, q.w))
    return [rotated[0] + t.x, rotated[1] + t.y, rotated[2] + t.z]


def fruit_from_text(text):
    """从“请帮我拿香蕉”中找出 banana。"""
    from config import FRUITS

    lower = str(text).lower()
    for name, info in FRUITS.items():
        if any(word.lower() in lower for word in info["words"]):
            return name
    return None
