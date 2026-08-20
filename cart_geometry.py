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


def detect_cart_black_marker_pixels(
    *,
    color_bgr,
    observation,
    gray_threshold=160,
):
    """Return back-left/back-right/front-left/front-right marker centers."""

    if not isinstance(observation, SceneMask):
        _fail("cart_mask_invalid")
    if observation.semantic_class != "cart_body":
        _fail("cart_body_mask_required")
    color = np.asarray(color_bgr)
    expected_shape = (observation.image_height, observation.image_width)
    if color.shape != expected_shape + (3,):
        _fail("cart_color_shape_mismatch")
    if type(gray_threshold) is not int or not 0 <= gray_threshold <= 255:
        _fail("cart_marker_threshold_invalid")

    try:
        import cv2
    except Exception as error:
        raise CartGeometryError("opencv_unavailable") from error

    x, y, width, height = observation.bbox
    bbox_area = float(width * height)
    search = decode_bbox_rle(
        image_shape=expected_shape,
        bbox=observation.bbox,
        counts=observation.rle_counts,
    )
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    dark = ((gray < gray_threshold) & search).astype(np.uint8)
    count, _labels, stats, centers = cv2.connectedComponentsWithStats(
        dark, connectivity=8
    )

    minimum_area = max(8, int(round(bbox_area * 0.00008)))
    maximum_area = max(minimum_area, int(round(bbox_area * 0.00080)))
    candidates = []
    for label in range(1, count):
        left, top, component_width, component_height, area = (
            int(value) for value in stats[label]
        )
        if not minimum_area <= area <= maximum_area:
            continue
        if not (
            max(4, int(round(width * 0.010)))
            <= component_width
            <= max(8, int(round(width * 0.070)))
        ):
            continue
        if not (
            max(2, int(round(height * 0.004)))
            <= component_height
            <= max(5, int(round(height * 0.040)))
        ):
            continue
        aspect_ratio = component_width / component_height
        if not 1.2 <= aspect_ratio <= 6.0:
            continue
        center_column, center_row = (float(value) for value in centers[label])
        candidates.append((
            center_column,
            center_row,
            left,
            top,
            component_width,
            component_height,
        ))

    if len(candidates) != 4:
        _fail(f"cart_marker_count_invalid:{len(candidates)}")
    candidates.sort(key=lambda item: (item[1], item[0]))
    back = sorted(candidates[:2], key=lambda item: item[0])
    front = sorted(candidates[2:], key=lambda item: item[0])

    def marker(candidate):
        center_column, center_row, left, top, marker_width, marker_height = candidate
        return {
            "center": (int(round(center_column)), int(round(center_row))),
            "bbox": (left, top, marker_width, marker_height),
        }

    return tuple(marker(item) for item in (back[0], back[1], front[0], front[1]))


def reconstruct_cart_marker_platform(
    *,
    observation,
    color_bgr,
    depth_m,
    intrinsics,
    camera_to_base: Callable[[tuple[float, float, float]], Sequence[float]],
    slot_offsets_from_left_m=(0.17, 0.24, 0.31, 0.38, 0.45),
    platform_width_m=0.75,
    platform_depth_m=0.29,
    gray_threshold=160,
    minimum_depth_m=0.20,
    maximum_depth_m=3.0,
):
    """Build the sloped loading plane from four black edge markers."""

    if not isinstance(intrinsics, CameraIntrinsics):
        _fail("camera_intrinsics_invalid")
    if not callable(camera_to_base):
        _fail("camera_transform_invalid")
    dimensions = np.asarray((platform_width_m, platform_depth_m), dtype=float)
    if not np.all(np.isfinite(dimensions)) or np.any(dimensions <= 0.0):
        _fail("cart_marker_dimensions_invalid")
    slot_offsets = tuple(float(value) for value in slot_offsets_from_left_m)
    if not slot_offsets or any(
        not np.isfinite(value)
        or value <= 0.0
        or value >= float(platform_width_m)
        for value in slot_offsets
    ):
        _fail("cart_slot_offsets_invalid")

    markers = detect_cart_black_marker_pixels(
        color_bgr=color_bgr,
        observation=observation,
        gray_threshold=gray_threshold,
    )
    depth = np.asarray(depth_m, dtype=float)
    expected_shape = (observation.image_height, observation.image_width)
    if depth.shape != expected_shape:
        _fail("depth_shape_mismatch")

    marker_points = []
    for marker in markers:
        left, top, width, height = marker["bbox"]
        padding = 2
        row_start = max(0, top - padding)
        row_stop = min(expected_shape[0], top + height + padding)
        column_start = max(0, left - padding)
        column_stop = min(expected_shape[1], left + width + padding)
        patch = depth[row_start:row_stop, column_start:column_stop]
        valid = (
            np.isfinite(patch)
            & (patch >= float(minimum_depth_m))
            & (patch <= float(maximum_depth_m))
        )
        values = patch[valid]
        if values.size < 4:
            _fail("cart_marker_depth_insufficient")
        value = float(np.median(values))
        column, row = marker["center"]
        camera_point = (
            (float(column) - intrinsics.cx) * value / intrinsics.fx,
            (float(row) - intrinsics.cy) * value / intrinsics.fy,
            value,
        )
        try:
            point = np.asarray(camera_to_base(camera_point), dtype=float)
        except Exception as error:
            raise CartGeometryError("camera_transform_failed") from error
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            _fail("cart_marker_point_invalid")
        marker_points.append(point)

    back_left, back_right, front_left, front_right = marker_points
    raw_lateral = 0.5 * (
        (back_left - back_right) + (front_left - front_right)
    )
    raw_forward = 0.5 * (
        (back_left - front_left) + (back_right - front_right)
    )
    forward_norm = float(np.linalg.norm(raw_forward))
    if forward_norm <= 1e-6:
        _fail("cart_marker_forward_axis_invalid")
    forward_axis = raw_forward / forward_norm
    lateral_axis = raw_lateral - forward_axis * float(
        np.dot(raw_lateral, forward_axis)
    )
    lateral_norm = float(np.linalg.norm(lateral_axis))
    if lateral_norm <= 1e-6:
        _fail("cart_marker_lateral_axis_invalid")
    lateral_axis /= lateral_norm
    normal = np.cross(forward_axis, lateral_axis)
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm <= 1e-6:
        _fail("cart_marker_plane_invalid")
    normal /= normal_norm
    if normal[2] < 0.0:
        lateral_axis = -lateral_axis
        normal = -normal
    if forward_axis[0] < 0.0:
        forward_axis = -forward_axis
        normal = -normal
    if lateral_axis[1] < 0.0:
        lateral_axis = -lateral_axis
        normal = -normal

    center = np.mean(np.asarray(marker_points), axis=0)
    half_width = float(platform_width_m) / 2.0
    half_depth = float(platform_depth_m) / 2.0
    front_edge = center - forward_axis * half_depth
    left_edge = center + lateral_axis * half_width
    right_edge = center - lateral_axis * half_width
    corners = (
        center - forward_axis * half_depth + lateral_axis * half_width,
        center + forward_axis * half_depth + lateral_axis * half_width,
        center + forward_axis * half_depth - lateral_axis * half_width,
        center - forward_axis * half_depth - lateral_axis * half_width,
    )
    slot_centers = tuple(
        tuple(float(value) for value in (
            center + lateral_axis * (half_width - offset)
        ))
        for offset in slot_offsets
    )

    base_points, rows, columns, _mask = _masked_base_points(
        observation=observation,
        depth_m=depth,
        intrinsics=intrinsics,
        camera_to_base=camera_to_base,
        minimum_depth_m=minimum_depth_m,
        maximum_depth_m=maximum_depth_m,
    )
    slot_pixels = tuple(
        _nearest_pixel(np.asarray(point), base_points, rows, columns)
        for point in slot_centers
    )
    outline_pixels = tuple(
        _nearest_pixel(point, base_points, rows, columns) for point in corners
    )
    return CartPlatformGeometry(
        center=tuple(float(value) for value in center),
        forward_axis=tuple(float(value) for value in forward_axis),
        lateral_axis_right_to_left=tuple(float(value) for value in lateral_axis),
        normal=tuple(float(value) for value in normal),
        depth_extent_m=float(platform_depth_m),
        lateral_extent_m=float(platform_width_m),
        slot_centers=slot_centers,
        slot_pixels=slot_pixels,
        front_edge=tuple(float(value) for value in front_edge),
        left_edge=tuple(float(value) for value in left_edge),
        right_edge=tuple(float(value) for value in right_edge),
        outline_pixels=outline_pixels,
        confidence=float(observation.confidence),
    )


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
    minimum_level_separation_m=0.10,
    minimum_depth_points=128,
    minimum_lateral_extent_m=0.42,
    minimum_forward_extent_m=0.12,
    minimum_plane_fill_fraction=0.25,
):
    """Extract the first usable shelf below the cart's top cover."""

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
    candidates = []
    # Inspect from top to bottom. Posts spread over many height bins; broad
    # horizontal surfaces form dense bands.  The first unique surface is the
    # cart's solid top cover; the next one is the upper loading shelf.
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
        # A height slice through vertical side/back walls can have the same
        # outer dimensions as a shelf while containing only a hollow U-shaped
        # outline.  A real horizontal board occupies the interior area too.
        grid_m = 0.02
        lateral_cells = np.floor(
            (lateral_projection - float(lateral_min)) / grid_m
        ).astype(int)
        forward_cells = np.floor(
            (forward_projection - float(forward_min)) / grid_m
        ).astype(int)
        inside = (
            (lateral_projection >= lateral_min)
            & (lateral_projection <= lateral_max)
            & (forward_projection >= forward_min)
            & (forward_projection <= forward_max)
        )
        occupied = len(set(zip(
            lateral_cells[inside].tolist(),
            forward_cells[inside].tolist(),
        )))
        grid_width = max(1, int(np.ceil(lateral_extent / grid_m)))
        grid_depth = max(1, int(np.ceil(forward_extent / grid_m)))
        fill_fraction = occupied / float(grid_width * grid_depth)
        if fill_fraction < float(minimum_plane_fill_fraction):
            continue
        candidate_geometry = (
            near, candidate, center_xy, forward_xy, lateral_xy,
            float(forward_min), float(forward_max),
            float(lateral_min), float(lateral_max),
            forward_extent, lateral_extent,
        )
        candidate_height = float(np.median(candidate[:, 2]))
        if any(
            abs(candidate_height - existing[0]) < float(minimum_level_separation_m)
            for existing in candidates
        ):
            continue
        candidates.append((candidate_height, candidate_geometry))

    if len(candidates) < 2:
        _fail("cart_top_platform_not_found")
    _height, chosen = candidates[1]

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
