"""ROS-free table-book mask and suction-point geometry."""

from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np


class BookGeometryError(ValueError):
    """Geometry failure with a stable, log-friendly code."""


@dataclass(frozen=True)
class BookMask:
    confidence: float
    bbox: tuple[int, int, int, int]
    rle_counts: tuple[int, ...]
    image_width: int
    image_height: int


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class BookGeometry:
    suction_point: tuple[float, float, float]
    long_axis: tuple[float, float, float]
    short_axis_right_to_left: tuple[float, float, float]
    long_extent_m: float
    short_extent_m: float
    confidence: float
    long_inset_m: float
    right_inset_m: float


def _fail(code):
    raise BookGeometryError(code)


def decode_bbox_rle(*, image_shape, bbox, counts):
    """Decode ``bbox_rle_row_major_v1`` into a full-image boolean mask."""

    if len(image_shape) != 2 or len(bbox) != 4:
        _fail("mask_shape_invalid")
    image_height, image_width = image_shape
    x, y, width, height = bbox
    values = (image_height, image_width, x, y, width, height)
    if any(type(value) is not int for value in values):
        _fail("mask_shape_invalid")
    if image_height <= 0 or image_width <= 0 or width <= 0 or height <= 0:
        _fail("mask_shape_invalid")
    if x < 0 or y < 0 or x + width > image_width or y + height > image_height:
        _fail("mask_bbox_out_of_bounds")
    if not isinstance(counts, Sequence) or not counts:
        _fail("mask_rle_invalid")

    normalized = []
    for index, count in enumerate(counts):
        if type(count) is not int or count < 0 or (index > 0 and count == 0):
            _fail("mask_rle_invalid")
        normalized.append(count)
    if sum(normalized) != width * height:
        _fail("mask_rle_size_mismatch")

    local = np.zeros(width * height, dtype=bool)
    cursor = 0
    foreground = False
    for count in normalized:
        if foreground:
            local[cursor : cursor + count] = True
        cursor += count
        foreground = not foreground

    result = np.zeros((image_height, image_width), dtype=bool)
    result[y : y + height, x : x + width] = local.reshape((height, width))
    return result


def _validate_intrinsics(intrinsics):
    values = np.asarray(
        [intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy], dtype=float
    )
    if not np.all(np.isfinite(values)) or intrinsics.fx <= 0 or intrinsics.fy <= 0:
        _fail("camera_intrinsics_invalid")


def reconstruct_table_book(
    *,
    observation,
    depth_m,
    intrinsics,
    camera_to_base: Callable[[tuple[float, float, float]], Sequence[float]],
    long_inset_m=0.13,
    right_inset_m=0.10,
    minimum_edge_clearance_m=0.015,
    minimum_depth_m=0.20,
    maximum_depth_m=2.50,
    minimum_depth_points=64,
    cover_band_m=0.02,
):
    """Reconstruct one flat book and its fixed robot-view suction point."""

    if not isinstance(observation, BookMask):
        _fail("book_mask_invalid")
    if not isinstance(intrinsics, CameraIntrinsics):
        _fail("camera_intrinsics_invalid")
    _validate_intrinsics(intrinsics)
    if not callable(camera_to_base):
        _fail("camera_transform_invalid")

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
        _fail("insufficient_book_depth")

    z = depth[rows, columns]
    camera_points = np.column_stack(
        (
            (columns - intrinsics.cx) * z / intrinsics.fx,
            (rows - intrinsics.cy) * z / intrinsics.fy,
            z,
        )
    )
    try:
        base_points = np.asarray(
            [camera_to_base(tuple(point)) for point in camera_points], dtype=float
        )
    except Exception as error:
        raise BookGeometryError("camera_transform_failed") from error
    if base_points.shape != (rows.size, 3):
        _fail("camera_transform_invalid")
    finite = np.all(np.isfinite(base_points), axis=1)
    base_points = base_points[finite]
    if base_points.shape[0] < int(minimum_depth_points):
        _fail("insufficient_book_depth")

    # A flat book's cover is the upper band in base_link Z. The 80th
    # percentile prevents a few high depth outliers from defining the band.
    cover_level = float(np.percentile(base_points[:, 2], 80.0))
    cover_points = base_points[base_points[:, 2] >= cover_level - cover_band_m]
    if cover_points.shape[0] < int(minimum_depth_points):
        _fail("insufficient_cover_depth")

    xy = cover_points[:, :2]
    center = np.mean(xy, axis=0)
    centered = xy - center
    covariance = centered.T @ centered / float(centered.shape[0])
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or eigenvalues[-1] <= 1e-10:
        _fail("book_axes_ambiguous")
    long_axis = eigenvectors[:, -1]

    long_projection = centered @ long_axis
    long_min = float(np.min(long_projection))
    long_max = float(np.max(long_projection))
    endpoint_min = center + long_axis * long_min
    endpoint_max = center + long_axis * long_max
    if np.linalg.norm(endpoint_max) < np.linalg.norm(endpoint_min):
        long_axis = -long_axis
        long_projection = centered @ long_axis
        long_min = float(np.min(long_projection))
        long_max = float(np.max(long_projection))

    short_axis = np.asarray((-long_axis[1], long_axis[0]), dtype=float)
    if short_axis[1] < 0:
        short_axis = -short_axis
    short_projection = centered @ short_axis
    short_min = float(np.min(short_projection))
    short_max = float(np.max(short_projection))

    long_extent = long_max - long_min
    short_extent = short_max - short_min
    clearance = float(minimum_edge_clearance_m)
    if (
        long_inset_m < clearance
        or right_inset_m < clearance
        or long_extent - long_inset_m < clearance
        or short_extent - right_inset_m < clearance
    ):
        _fail("book_too_small_for_insets")

    suction_xy = (
        center
        + long_axis * (long_min + float(long_inset_m))
        + short_axis * (short_min + float(right_inset_m))
    )
    suction_z = float(np.median(cover_points[:, 2]))
    mask_area = max(1, int(mask.sum()))
    valid_fraction = min(1.0, float(base_points.shape[0]) / mask_area)
    confidence = float(observation.confidence) * valid_fraction

    return BookGeometry(
        suction_point=(float(suction_xy[0]), float(suction_xy[1]), suction_z),
        long_axis=(float(long_axis[0]), float(long_axis[1]), 0.0),
        short_axis_right_to_left=(
            float(short_axis[0]),
            float(short_axis[1]),
            0.0,
        ),
        long_extent_m=float(long_extent),
        short_extent_m=float(short_extent),
        confidence=confidence,
        long_inset_m=float(long_inset_m),
        right_inset_m=float(right_inset_m),
    )
