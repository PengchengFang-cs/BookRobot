"""ROS-free cart-platform geometry and five equal placement slots."""

from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np

from book_geometry import CameraIntrinsics, decode_bbox_rle
from book_rpc import SceneMask


class CartGeometryError(ValueError):
    pass


@dataclass(frozen=True)
class CartPlatformGeometry:
    center: tuple[float, float, float]
    forward_axis: tuple[float, float, float]
    lateral_axis_right_to_left: tuple[float, float, float]
    normal: tuple[float, float, float]
    depth_extent_m: float
    lateral_extent_m: float
    slot_centers: tuple[tuple[float, float, float], ...]
    slot_pixels: tuple[tuple[int, int], ...]
    confidence: float


def _fail(code):
    raise CartGeometryError(code)


def reconstruct_cart_platform(
    *,
    observation,
    depth_m,
    intrinsics,
    camera_to_base: Callable[[tuple[float, float, float]], Sequence[float]],
    slot_count=5,
    minimum_depth_m=0.20,
    maximum_depth_m=3.0,
    minimum_depth_points=128,
):
    """Fit the segmented loading plane and divide it right-to-left."""

    if not isinstance(observation, SceneMask):
        _fail("cart_mask_invalid")
    if observation.semantic_class != "cart_platform":
        _fail("cart_platform_mask_required")
    if not isinstance(intrinsics, CameraIntrinsics):
        _fail("camera_intrinsics_invalid")
    if not callable(camera_to_base):
        _fail("camera_transform_invalid")
    slot_count = int(slot_count)
    if slot_count < 1:
        _fail("cart_slot_count_invalid")

    depth = np.asarray(depth_m, dtype=float)
    expected_shape = (observation.image_height, observation.image_width)
    if depth.shape != expected_shape:
        _fail("depth_shape_mismatch")
    mask = decode_bbox_rle(
        image_shape=expected_shape,
        bbox=observation.bbox,
        counts=observation.rle_counts,
    )
    valid = (
        mask
        & np.isfinite(depth)
        & (depth >= float(minimum_depth_m))
        & (depth <= float(maximum_depth_m))
    )
    rows, columns = np.nonzero(valid)
    if rows.size < int(minimum_depth_points):
        _fail("insufficient_cart_platform_depth")

    z = depth[rows, columns]
    camera_points = np.column_stack((
        (columns - intrinsics.cx) * z / intrinsics.fx,
        (rows - intrinsics.cy) * z / intrinsics.fy,
        z,
    ))
    try:
        base_points = np.asarray(
            [camera_to_base(tuple(point)) for point in camera_points],
            dtype=float,
        )
    except Exception as error:
        raise CartGeometryError("camera_transform_failed") from error
    finite = np.all(np.isfinite(base_points), axis=1)
    base_points = base_points[finite]
    rows = rows[finite]
    columns = columns[finite]
    if base_points.shape[0] < int(minimum_depth_points):
        _fail("insufficient_cart_platform_depth")

    center = np.median(base_points, axis=0)
    centered = base_points - center
    _u, singular_values, axes = np.linalg.svd(centered, full_matrices=False)
    if singular_values.size != 3 or singular_values[1] <= 1e-6:
        _fail("cart_platform_axes_ambiguous")
    plane_axes = [axes[0].copy(), axes[1].copy()]
    lateral_index = int(abs(plane_axes[1][1]) > abs(plane_axes[0][1]))
    lateral_axis = plane_axes[lateral_index]
    forward_axis = plane_axes[1 - lateral_index]
    if lateral_axis[1] < 0.0:
        lateral_axis = -lateral_axis
    if forward_axis[0] < 0.0:
        forward_axis = -forward_axis
    normal = np.cross(forward_axis, lateral_axis)
    normal /= np.linalg.norm(normal)
    if normal[2] < 0.0:
        normal = -normal

    forward_projection = centered @ forward_axis
    lateral_projection = centered @ lateral_axis
    forward_min, forward_max = np.percentile(forward_projection, (2.0, 98.0))
    lateral_min, lateral_max = np.percentile(lateral_projection, (2.0, 98.0))
    forward_mid = (float(forward_min) + float(forward_max)) / 2.0
    lateral_extent = float(lateral_max) - float(lateral_min)
    depth_extent = float(forward_max) - float(forward_min)
    if lateral_extent <= 0.05 or depth_extent <= 0.05:
        _fail("cart_platform_too_small")

    slot_centers = []
    slot_pixels = []
    for index in range(slot_count):
        lateral_offset = float(lateral_min) + (
            (index + 0.5) * lateral_extent / slot_count
        )
        point = center + forward_axis * forward_mid + lateral_axis * lateral_offset
        point_tuple = tuple(float(value) for value in point)
        slot_centers.append(point_tuple)
        nearest = int(np.argmin(np.sum((base_points - point) ** 2, axis=1)))
        slot_pixels.append((int(columns[nearest]), int(rows[nearest])))

    mask_area = max(1, int(mask.sum()))
    valid_fraction = min(1.0, float(base_points.shape[0]) / mask_area)
    return CartPlatformGeometry(
        center=tuple(float(value) for value in center),
        forward_axis=tuple(float(value) for value in forward_axis),
        lateral_axis_right_to_left=tuple(float(value) for value in lateral_axis),
        normal=tuple(float(value) for value in normal),
        depth_extent_m=depth_extent,
        lateral_extent_m=lateral_extent,
        slot_centers=tuple(slot_centers),
        slot_pixels=tuple(slot_pixels),
        confidence=float(observation.confidence) * valid_fraction,
    )
