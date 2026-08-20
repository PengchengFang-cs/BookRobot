"""ROS-free cart geometry used by coarse and fine cart alignment."""

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
    front_edge: tuple[float, float, float]
    left_edge: tuple[float, float, float]
    right_edge: tuple[float, float, float]
    outline_pixels: tuple[tuple[int, int], ...]
    confidence: float


@dataclass(frozen=True)
class CartBodyTarget:
    """A robust RGB-D representative point used only for coarse base docking."""

    center: tuple[float, float, float]
    center_pixel: tuple[int, int]
    confidence: float


def reconstruct_cart_body_target(
    *,
    observation,
    depth_m,
    intrinsics,
    camera_to_base: Callable[[tuple[float, float, float]], Sequence[float]],
    minimum_depth_m=0.20,
    maximum_depth_m=3.0,
    minimum_depth_points=32,
):
    """Find a stable 3-D point near the center of a segmented cart body."""

    if not isinstance(observation, SceneMask):
        _fail("cart_mask_invalid")
    if observation.semantic_class != "cart_body":
        _fail("cart_body_mask_required")
    if not isinstance(intrinsics, CameraIntrinsics):
        _fail("camera_intrinsics_invalid")
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
    x, y, width, height = observation.bbox
    central = np.zeros(expected_shape, dtype=bool)
    central[
        y + int(height * 0.30):y + max(1, int(height * 0.70)),
        x + int(width * 0.30):x + max(1, int(width * 0.70)),
    ] = True
    valid = (
        mask
        & central
        & np.isfinite(depth)
        & (depth >= float(minimum_depth_m))
        & (depth <= float(maximum_depth_m))
    )
    rows, columns = np.nonzero(valid)
    if rows.size < int(minimum_depth_points):
        _fail("insufficient_cart_body_center_depth")

    # Median depth rejects isolated pixels on the handle, shelf edges and holes.
    median_depth = float(np.median(depth[rows, columns]))
    deviations = np.abs(depth[rows, columns] - median_depth)
    # Keep the entire central surface when many pixels have identical depth;
    # selecting an arbitrary fixed count would bias the median toward one side.
    cutoff = max(0.03, float(np.percentile(deviations, 50.0)))
    keep = deviations <= cutoff
    rows = rows[keep]
    columns = columns[keep]
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
    base_points = base_points[np.all(np.isfinite(base_points), axis=1)]
    if base_points.shape[0] < int(minimum_depth_points):
        _fail("insufficient_cart_body_center_depth")

    center = np.median(base_points, axis=0)
    center_pixel = (int(np.median(columns)), int(np.median(rows)))
    return CartBodyTarget(
        center=tuple(float(value) for value in center),
        center_pixel=center_pixel,
        confidence=float(observation.confidence),
    )


def select_leftmost_cart_platform(platforms):
    """Choose the cart-side plane when a table is also labelled as a platform."""

    candidates = tuple(platform for platform in platforms if platform is not None)
    if not candidates:
        return None
    return max(candidates, key=lambda platform: float(platform.center[1]))


def _fail(code):
    raise CartGeometryError(code)


def _masked_base_points(
    *, observation, depth_m, intrinsics, camera_to_base,
    minimum_depth_m, maximum_depth_m,
):
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
    return base_points[finite], rows[finite], columns[finite], mask


def _nearest_pixel(point, base_points, rows, columns):
    nearest = int(np.argmin(np.sum((base_points - point) ** 2, axis=1)))
    return int(columns[nearest]), int(rows[nearest])


def reconstruct_cart_top_platform(
    *,
    observation,
    depth_m,
    intrinsics,
    camera_to_base: Callable[[tuple[float, float, float]], Sequence[float]],
    slot_offsets_from_left_m=(0.17, 0.24, 0.31, 0.38, 0.45),
    minimum_depth_m=0.20,
    maximum_depth_m=3.0,
    height_bin_m=0.012,
    plane_tolerance_m=0.012,
    minimum_depth_points=128,
    minimum_lateral_extent_m=0.42,
    minimum_forward_extent_m=0.12,
):
    """Extract the highest broad horizontal shelf inside a whole-cart mask."""

    if not isinstance(observation, SceneMask):
        _fail("cart_mask_invalid")
    if observation.semantic_class != "cart_body":
        _fail("cart_body_mask_required")
    if not isinstance(intrinsics, CameraIntrinsics):
        _fail("camera_intrinsics_invalid")
    if not callable(camera_to_base):
        _fail("camera_transform_invalid")

    slot_offsets = tuple(float(value) for value in slot_offsets_from_left_m)
    if not slot_offsets or any(
        not np.isfinite(value) or value <= 0.0 for value in slot_offsets
    ):
        _fail("cart_slot_offsets_invalid")

    base_points, rows, columns, mask = _masked_base_points(
        observation=observation,
        depth_m=depth_m,
        intrinsics=intrinsics,
        camera_to_base=camera_to_base,
        minimum_depth_m=minimum_depth_m,
        maximum_depth_m=maximum_depth_m,
    )
    if base_points.shape[0] < int(minimum_depth_points):
        _fail("insufficient_cart_body_depth")

    heights = base_points[:, 2]
    low = float(np.min(heights))
    bins = np.floor((heights - low) / float(height_bin_m)).astype(int)
    populated_bins = np.nonzero(np.bincount(bins))[0]
    chosen = None
    # Inspect from top to bottom. Posts spread over many height bins; a shelf
    # contributes a broad, dense horizontal band.
    for bin_index in populated_bins[::-1]:
        height_center = low + (float(bin_index) + 0.5) * float(height_bin_m)
        near = np.abs(heights - height_center) <= float(plane_tolerance_m)
        if int(np.count_nonzero(near)) < int(minimum_depth_points):
            continue
        candidate = base_points[near]
        center_xy = np.median(candidate[:, :2], axis=0)
        centered_xy = candidate[:, :2] - center_xy
        _u, singular, axes_xy = np.linalg.svd(centered_xy, full_matrices=False)
        if singular.size != 2 or singular[1] <= 1e-6:
            continue
        axes = [axes_xy[0].copy(), axes_xy[1].copy()]
        lateral_index = int(abs(axes[1][1]) > abs(axes[0][1]))
        lateral_xy = axes[lateral_index]
        forward_xy = axes[1 - lateral_index]
        if lateral_xy[1] < 0.0:
            lateral_xy = -lateral_xy
        if forward_xy[0] < 0.0:
            forward_xy = -forward_xy
        lateral_projection = centered_xy @ lateral_xy
        forward_projection = centered_xy @ forward_xy
        lateral_min, lateral_max = np.percentile(lateral_projection, (2.0, 98.0))
        forward_min, forward_max = np.percentile(forward_projection, (2.0, 98.0))
        lateral_extent = float(lateral_max - lateral_min)
        forward_extent = float(forward_max - forward_min)
        if (
            lateral_extent < float(minimum_lateral_extent_m)
            or forward_extent < float(minimum_forward_extent_m)
        ):
            continue
        chosen = (
            near, candidate, center_xy, forward_xy, lateral_xy,
            float(forward_min), float(forward_max),
            float(lateral_min), float(lateral_max),
            forward_extent, lateral_extent,
        )
        break

    if chosen is None:
        _fail("cart_top_platform_not_found")

    (
        near, plane_points, center_xy, forward_xy, lateral_xy,
        forward_min, forward_max, lateral_min, lateral_max,
        forward_extent, lateral_extent,
    ) = chosen
    plane_rows = rows[near]
    plane_columns = columns[near]
    plane_z = float(np.median(plane_points[:, 2]))
    center = np.array((center_xy[0], center_xy[1], plane_z), dtype=float)
    forward_axis = np.array((forward_xy[0], forward_xy[1], 0.0), dtype=float)
    lateral_axis = np.array((lateral_xy[0], lateral_xy[1], 0.0), dtype=float)
    normal = np.array((0.0, 0.0, 1.0), dtype=float)
    forward_mid = (forward_min + forward_max) / 2.0
    lateral_mid = (lateral_min + lateral_max) / 2.0

    front_edge = center + forward_axis * forward_min + lateral_axis * lateral_mid
    left_edge = center + forward_axis * forward_mid + lateral_axis * lateral_max
    right_edge = center + forward_axis * forward_mid + lateral_axis * lateral_min
    corners = (
        center + forward_axis * forward_min + lateral_axis * lateral_max,
        center + forward_axis * forward_max + lateral_axis * lateral_max,
        center + forward_axis * forward_max + lateral_axis * lateral_min,
        center + forward_axis * forward_min + lateral_axis * lateral_min,
    )

    slot_centers = []
    slot_pixels = []
    for offset in slot_offsets:
        point = center + forward_axis * forward_mid + lateral_axis * (
            lateral_max - offset
        )
        slot_centers.append(tuple(float(value) for value in point))
        slot_pixels.append(_nearest_pixel(
            point, plane_points, plane_rows, plane_columns
        ))

    outline_pixels = tuple(
        _nearest_pixel(point, plane_points, plane_rows, plane_columns)
        for point in corners
    )
    valid_fraction = min(1.0, float(plane_points.shape[0]) / max(1, int(mask.sum())))
    return CartPlatformGeometry(
        center=tuple(float(value) for value in center),
        forward_axis=tuple(float(value) for value in forward_axis),
        lateral_axis_right_to_left=tuple(float(value) for value in lateral_axis),
        normal=tuple(float(value) for value in normal),
        depth_extent_m=forward_extent,
        lateral_extent_m=lateral_extent,
        slot_centers=tuple(slot_centers),
        slot_pixels=tuple(slot_pixels),
        front_edge=tuple(float(value) for value in front_edge),
        left_edge=tuple(float(value) for value in left_edge),
        right_edge=tuple(float(value) for value in right_edge),
        outline_pixels=outline_pixels,
        confidence=float(observation.confidence) * valid_fraction,
    )


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
        front_edge=tuple(float(value) for value in (
            center + forward_axis * float(forward_min)
        )),
        left_edge=tuple(float(value) for value in (
            center + lateral_axis * float(lateral_max)
        )),
        right_edge=tuple(float(value) for value in (
            center + lateral_axis * float(lateral_min)
        )),
        outline_pixels=(),
        confidence=float(observation.confidence) * valid_fraction,
    )
