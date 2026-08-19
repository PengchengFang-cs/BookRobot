"""DataReplay image/depth calibration for the recorded suction contact."""

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from book_geometry import CameraIntrinsics, decode_bbox_rle


@dataclass(frozen=True)
class ProjectedReplayContact:
    reference_contact_base_m: tuple[float, float, float]
    sample_count: int
    axis_spread_m: tuple[float, float, float]


@dataclass(frozen=True)
class ReplayPickReference:
    asset_id: str
    reference_frame_index: int
    contact_frame_index: int
    recorded_book_rule_point_base_m: tuple[float, float, float]
    recorded_contact_point_base_m: tuple[float, float, float]
    recorded_contact_pixel: tuple[float, float]
    recorded_book_suction_point_base_m: tuple[float, float, float]
    recorded_book_long_axis_base: tuple[float, float, float]
    recorded_book_long_extent_m: float
    recorded_book_short_extent_m: float
    hdf5_sha256: str


def scale_intrinsics(intrinsics, *, source_size, target_size):
    """Scale camera intrinsics for a separately resized width and height."""

    source_width, source_height = (float(value) for value in source_size)
    target_width, target_height = (float(value) for value in target_size)
    if min(source_width, source_height, target_width, target_height) <= 0:
        raise ValueError("camera_size_invalid")
    scale_x = target_width / source_width
    scale_y = target_height / source_height
    return CameraIntrinsics(
        fx=float(intrinsics.fx) * scale_x,
        fy=float(intrinsics.fy) * scale_y,
        cx=(float(intrinsics.cx) + 0.5) * scale_x - 0.5,
        cy=(float(intrinsics.cy) + 0.5) * scale_y - 0.5,
    )


def _largest_new_blue_component(*, baseline_rgb, contact_rgb, roi_xywh):
    baseline = np.asarray(baseline_rgb, dtype=np.uint8)
    contact = np.asarray(contact_rgb, dtype=np.uint8)
    if baseline.shape != contact.shape or baseline.ndim != 3 or baseline.shape[2] != 3:
        raise ValueError("recorded_rgb_shape_mismatch")
    x, y, width, height = (int(value) for value in roi_xywh)
    image_height, image_width = contact.shape[:2]
    if (
        width <= 0
        or height <= 0
        or x < 0
        or y < 0
        or x + width > image_width
        or y + height > image_height
    ):
        raise ValueError("suction_roi_invalid")

    roi = contact[y : y + height, x : x + width]
    baseline_roi = baseline[y : y + height, x : x + width]
    rgb = roi.astype(float) / 255.0
    maximum = np.max(rgb, axis=2)
    minimum = np.min(rgb, axis=2)
    delta = maximum - minimum
    hue_degrees = np.zeros(maximum.shape, dtype=float)
    nonzero = delta > 0
    red_max = nonzero & (rgb[:, :, 0] == maximum)
    green_max = nonzero & (rgb[:, :, 1] == maximum)
    blue_max = nonzero & (rgb[:, :, 2] == maximum)
    hue_degrees[red_max] = (
        60.0
        * ((rgb[:, :, 1][red_max] - rgb[:, :, 2][red_max]) / delta[red_max])
    ) % 360.0
    hue_degrees[green_max] = 60.0 * (
        (rgb[:, :, 2][green_max] - rgb[:, :, 0][green_max]) / delta[green_max]
        + 2.0
    )
    hue_degrees[blue_max] = 60.0 * (
        (rgb[:, :, 0][blue_max] - rgb[:, :, 1][blue_max]) / delta[blue_max]
        + 4.0
    )
    saturation = np.zeros(maximum.shape, dtype=float)
    positive = maximum > 0
    saturation[positive] = delta[positive] / maximum[positive]
    blue = (
        (hue_degrees >= 180.0)
        & (hue_degrees <= 280.0)
        & (saturation >= 70.0 / 255.0)
        & (maximum >= 35.0 / 255.0)
    )
    changed = np.max(
        np.abs(roi.astype(np.int16) - baseline_roi.astype(np.int16)), axis=2
    ) >= 20
    mask = changed & blue
    visited = np.zeros(mask.shape, dtype=bool)
    components = []
    for row, column in zip(*np.nonzero(mask)):
        if visited[row, column]:
            continue
        stack = [(int(row), int(column))]
        visited[row, column] = True
        pixels = []
        while stack:
            current_row, current_column = stack.pop()
            pixels.append((current_row, current_column))
            for delta_row in (-1, 0, 1):
                for delta_column in (-1, 0, 1):
                    next_row = current_row + delta_row
                    next_column = current_column + delta_column
                    if (
                        0 <= next_row < mask.shape[0]
                        and 0 <= next_column < mask.shape[1]
                        and mask[next_row, next_column]
                        and not visited[next_row, next_column]
                    ):
                        visited[next_row, next_column] = True
                        stack.append((next_row, next_column))
        components.append(pixels)
    if not components:
        raise ValueError("blue_suction_not_found")
    component = max(components, key=len)
    full_component = np.zeros(contact.shape[:2], dtype=bool)
    rows, columns = np.asarray(component, dtype=int).T
    full_component[y + rows, x + columns] = True
    return full_component


def find_blue_suction_center(*, baseline_rgb, contact_rgb, roi_xywh):
    """Return the centroid of the largest newly visible blue ROI component."""

    component = _largest_new_blue_component(
        baseline_rgb=baseline_rgb,
        contact_rgb=contact_rgb,
        roi_xywh=roi_xywh,
    )
    rows, columns = np.nonzero(component)
    return (float(np.mean(columns)), float(np.mean(rows)))


def find_blue_suction_contact_tip(*, baseline_rgb, contact_rgb, roi_xywh):
    """Return the midpoint of the blue suction's farthest image-row edge."""

    component = _largest_new_blue_component(
        baseline_rgb=baseline_rgb,
        contact_rgb=contact_rgb,
        roi_xywh=roi_xywh,
    )
    rows, columns = np.nonzero(component)
    tip_row = int(np.min(rows))
    tip_columns = columns[rows == tip_row]
    return (float(np.median(tip_columns)), float(tip_row))


def find_blue_book_contact_pixel(
    *, baseline_rgb, contact_rgb, roi_xywh, book_mask
):
    """Return the recorded book pixel nearest the blue suction's contact end."""

    component = _largest_new_blue_component(
        baseline_rgb=baseline_rgb,
        contact_rgb=contact_rgb,
        roi_xywh=roi_xywh,
    )
    book = np.asarray(book_mask, dtype=bool)
    if book.shape != component.shape or not np.any(book):
        raise ValueError("recorded_book_mask_invalid")
    blue_rows, blue_columns = np.nonzero(component)
    book_rows, book_columns = np.nonzero(book)
    minimum_distance_squared = None
    closest_book_pixels = []
    for blue_row, blue_column in zip(blue_rows, blue_columns):
        distances = (
            (book_rows - blue_row) * (book_rows - blue_row)
            + (book_columns - blue_column) * (book_columns - blue_column)
        )
        local_minimum = int(np.min(distances))
        if (
            minimum_distance_squared is None
            or local_minimum < minimum_distance_squared
        ):
            minimum_distance_squared = local_minimum
            closest_book_pixels = []
        if local_minimum == minimum_distance_squared:
            for index in np.flatnonzero(distances == local_minimum):
                closest_book_pixels.append(
                    (int(book_rows[index]), int(book_columns[index]))
                )
    unique = np.asarray(sorted(set(closest_book_pixels)), dtype=float)
    return (float(np.mean(unique[:, 1])), float(np.mean(unique[:, 0])))


def project_recorded_contact(
    *,
    center_px,
    depths_mm,
    intrinsics,
    torso_head_states,
    camera_to_base,
):
    """Project one recorded contact pixel through several early depth frames."""

    if len(depths_mm) != len(torso_head_states):
        raise ValueError("recorded_state_count_mismatch")
    u, v = (float(value) for value in center_px)
    if not math.isfinite(u) or not math.isfinite(v):
        raise ValueError("recorded_contact_pixel_invalid")
    projected = []
    for depth_mm, state in zip(depths_mm, torso_head_states):
        depth = np.asarray(depth_mm)
        if depth.ndim != 2:
            raise ValueError("recorded_depth_shape_invalid")
        column = int(round(u))
        row = int(round(v))
        y0, y1 = max(0, row - 2), min(depth.shape[0], row + 3)
        x0, x1 = max(0, column - 2), min(depth.shape[1], column + 3)
        patch = depth[y0:y1, x0:x1].astype(float)
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            continue
        z = float(np.median(valid)) / 1000.0
        camera_point = (
            (u - float(intrinsics.cx)) * z / float(intrinsics.fx),
            (v - float(intrinsics.cy)) * z / float(intrinsics.fy),
            z,
        )
        base_point = np.asarray(camera_to_base(camera_point, *state), dtype=float)
        if base_point.shape != (3,) or not np.all(np.isfinite(base_point)):
            raise ValueError("recorded_contact_transform_invalid")
        projected.append(base_point)
    if not projected:
        raise ValueError("recorded_contact_depth_missing")
    points = np.asarray(projected, dtype=float)
    median = np.median(points, axis=0)
    spread = np.ptp(points, axis=0)
    return ProjectedReplayContact(
        reference_contact_base_m=tuple(float(value) for value in median),
        sample_count=int(points.shape[0]),
        axis_spread_m=tuple(float(value) for value in spread),
    )


def project_recorded_contact_to_cover(
    *,
    center_px,
    intrinsics,
    torso_head_states,
    cover_z_base_m,
    camera_to_base,
):
    """Intersect a recorded image ray with the horizontal book-cover plane."""

    u, v = (float(value) for value in center_px)
    cover_z = float(cover_z_base_m)
    ray_camera = (
        (u - float(intrinsics.cx)) / float(intrinsics.fx),
        (v - float(intrinsics.cy)) / float(intrinsics.fy),
        1.0,
    )
    projected = []
    for state in torso_head_states:
        origin = np.asarray(camera_to_base((0.0, 0.0, 0.0), *state), dtype=float)
        ray_point = np.asarray(camera_to_base(ray_camera, *state), dtype=float)
        direction = ray_point - origin
        if (
            origin.shape != (3,)
            or direction.shape != (3,)
            or not np.all(np.isfinite(origin))
            or not np.all(np.isfinite(direction))
            or abs(float(direction[2])) <= 1e-12
        ):
            raise ValueError("recorded_cover_projection_invalid")
        scale = (cover_z - float(origin[2])) / float(direction[2])
        point = origin + direction * scale
        if scale <= 0.0 or not np.all(np.isfinite(point)):
            raise ValueError("recorded_cover_projection_invalid")
        projected.append(point)
    if not projected:
        raise ValueError("recorded_state_count_mismatch")
    points = np.asarray(projected, dtype=float)
    median = np.median(points, axis=0)
    spread = np.ptp(points, axis=0)
    return ProjectedReplayContact(
        reference_contact_base_m=tuple(float(value) for value in median),
        sample_count=int(points.shape[0]),
        axis_spread_m=tuple(float(value) for value in spread),
    )


def calibrate_replay_pick_reference(
    *,
    h5_path,
    asset_id,
    reference_frame_index,
    contact_frame_index,
    suction_roi_xywh,
    source_intrinsics,
    source_size,
    detect_recorded_book,
    camera_to_base,
):
    """Detect the recorded book suction point at the replay start frame."""

    import h5py

    frame_index = int(reference_frame_index)
    contact_index = int(contact_frame_index)
    with h5py.File(h5_path, "r") as recording:
        images = recording["observation/image"]
        depths = recording["observations/depth_head_rgbd"]
        heads = recording["observations/qpos_head"]
        torsos = recording["observations/qpos_torso"]
        intrinsics = scale_intrinsics(
            source_intrinsics,
            source_size=source_size,
            target_size=(int(images.shape[2]), int(images.shape[1])),
        )
        if not (0 <= frame_index < len(images)) or not (
            0 <= contact_index < len(images)
        ):
            raise ValueError("recorded_frame_index_invalid")
        detected_books = detect_recorded_book(
            images[frame_index],
            depths[frame_index],
            intrinsics,
            float(torsos[frame_index, 0]),
            (
                float(heads[frame_index, 0]),
                float(heads[frame_index, 1]),
            ),
        )
        if hasattr(detected_books, "geometry"):
            detected_books = (detected_books,)
        else:
            detected_books = tuple(detected_books)
        if not detected_books:
            raise ValueError("recorded_book_not_detected")
        contact_pixel = find_blue_suction_contact_tip(
            baseline_rgb=images[frame_index],
            contact_rgb=images[contact_index],
            roi_xywh=suction_roi_xywh,
        )
        contact_state = (
            float(torsos[contact_index, 0]),
            float(heads[contact_index, 0]),
            float(heads[contact_index, 1]),
        )
        candidates = []
        for detected_book in detected_books:
            candidate_geometry = detected_book.geometry
            candidate_rule = tuple(
                float(value) for value in candidate_geometry.suction_point
            )
            if len(candidate_rule) != 3 or not all(
                math.isfinite(value) for value in candidate_rule
            ):
                continue
            candidate_contact = project_recorded_contact_to_cover(
                center_px=contact_pixel,
                intrinsics=intrinsics,
                torso_head_states=[contact_state],
                cover_z_base_m=candidate_rule[2],
                camera_to_base=camera_to_base,
            ).reference_contact_base_m
            candidates.append(
                (
                    abs(candidate_rule[1] - candidate_contact[1]),
                    detected_book,
                    candidate_rule,
                    candidate_contact,
                )
            )
        if not candidates:
            raise ValueError("recorded_book_suction_point_invalid")
        _, detected_book, rule_point, contact_point = min(
            candidates, key=lambda candidate: candidate[0]
        )
        geometry = detected_book.geometry
        long_axis = tuple(float(value) for value in geometry.long_axis)
        if len(long_axis) != 3 or not all(math.isfinite(value) for value in long_axis):
            raise ValueError("recorded_book_long_axis_invalid")
        point = (rule_point[0], contact_point[1], rule_point[2])
    digest = hashlib.sha256()
    with Path(h5_path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return ReplayPickReference(
        asset_id=str(asset_id),
        reference_frame_index=frame_index,
        contact_frame_index=contact_index,
        recorded_book_rule_point_base_m=rule_point,
        recorded_contact_point_base_m=contact_point,
        recorded_contact_pixel=tuple(float(value) for value in contact_pixel),
        recorded_book_suction_point_base_m=point,
        recorded_book_long_axis_base=long_axis,
        recorded_book_long_extent_m=float(geometry.long_extent_m),
        recorded_book_short_extent_m=float(geometry.short_extent_m),
        hdf5_sha256=digest.hexdigest(),
    )


def save_replay_pick_reference(path, reference):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = asdict(reference)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def load_replay_pick_reference(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return ReplayPickReference(
        asset_id=str(payload["asset_id"]),
        reference_frame_index=int(payload["reference_frame_index"]),
        contact_frame_index=int(payload["contact_frame_index"]),
        recorded_book_rule_point_base_m=tuple(
            float(value) for value in payload["recorded_book_rule_point_base_m"]
        ),
        recorded_contact_point_base_m=tuple(
            float(value) for value in payload["recorded_contact_point_base_m"]
        ),
        recorded_contact_pixel=tuple(
            float(value) for value in payload["recorded_contact_pixel"]
        ),
        recorded_book_suction_point_base_m=tuple(
            float(value)
            for value in payload["recorded_book_suction_point_base_m"]
        ),
        recorded_book_long_axis_base=tuple(
            float(value) for value in payload["recorded_book_long_axis_base"]
        ),
        recorded_book_long_extent_m=float(
            payload["recorded_book_long_extent_m"]
        ),
        recorded_book_short_extent_m=float(
            payload["recorded_book_short_extent_m"]
        ),
        hdf5_sha256=str(payload["hdf5_sha256"]),
    )
