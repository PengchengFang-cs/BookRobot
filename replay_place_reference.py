"""Offline Place-2.4 cart geometry reference derived from recorded RGB-D."""

from dataclasses import asdict, dataclass
import json
from pathlib import Path

import h5py
import numpy as np

from book_geometry import CameraIntrinsics, decode_bbox_rle
from cart_geometry import reconstruct_cart_top_platform
from geometry import camera_point_to_base
from replay_pick_reference import scale_intrinsics


@dataclass(frozen=True)
class ReplayPlaceReference:
    asset_id: str
    reference_frame_index: int
    placed_frame_index: int
    recorded_torso_m: float
    recorded_head_rad: tuple[float, float]
    platform_front_edge_base_m: tuple[float, float, float]
    platform_left_edge_base_m: tuple[float, float, float]
    platform_right_edge_base_m: tuple[float, float, float]
    platform_forward_axis_base: tuple[float, float, float]
    platform_lateral_axis_base: tuple[float, float, float]
    platform_width_m: float
    platform_depth_m: float
    recorded_book_offset_from_left_m: float


def _mask_center_base(observation, depth_m, intrinsics, transform):
    mask = decode_bbox_rle(
        image_shape=(observation.image_height, observation.image_width),
        bbox=observation.bbox,
        counts=observation.rle_counts,
    )
    depth = np.asarray(depth_m, dtype=float)
    valid = mask & np.isfinite(depth) & (depth >= 0.2) & (depth <= 3.0)
    rows, columns = np.nonzero(valid)
    if rows.size < 32:
        raise ValueError("recorded_book_depth_insufficient")
    values = depth[rows, columns]
    points_camera = np.column_stack((
        (columns - intrinsics.cx) * values / intrinsics.fx,
        (rows - intrinsics.cy) * values / intrinsics.fy,
        values,
    ))
    points_base = np.asarray(
        [transform(tuple(point)) for point in points_camera], dtype=float
    )
    points_base = points_base[np.all(np.isfinite(points_base), axis=1)]
    if points_base.shape[0] < 32:
        raise ValueError("recorded_book_depth_insufficient")
    return np.median(points_base, axis=0)


def calibrate_replay_place_reference(
    *,
    h5_path,
    asset_id,
    source_intrinsics,
    source_size,
    detect_cart,
    detect_book=None,
    reference_frame_index=0,
    placed_frame_index=-1,
):
    """Read two HDF frames and derive the cart-relative Place reference."""

    with h5py.File(h5_path, "r") as recording:
        images = recording["observation/image"]
        depths = recording["observations/depth_head_rgbd"]
        torsos = recording["observations/qpos_torso"]
        heads = recording["observations/qpos_head"]
        frame_count = int(images.shape[0])
        placed_index = int(placed_frame_index)
        if placed_index < 0:
            placed_index += frame_count
        if not 0 <= int(reference_frame_index) < frame_count:
            raise ValueError("place_reference_frame_invalid")
        if not 0 <= placed_index < frame_count:
            raise ValueError("place_placed_frame_invalid")
        frames = {}
        for index in (int(reference_frame_index), placed_index):
            frames[index] = (
                np.asarray(images[index]),
                np.asarray(depths[index], dtype=float) / 1000.0,
                float(torsos[index][0]),
                tuple(float(value) for value in heads[index]),
            )

    height, width = frames[int(reference_frame_index)][0].shape[:2]
    intrinsics = scale_intrinsics(
        source_intrinsics,
        source_size=source_size,
        target_size=(width, height),
    )

    platforms = {}
    observations_by_frame = {}
    for index, (rgb, depth, torso, head) in frames.items():
        observations = tuple(detect_cart(np.ascontiguousarray(rgb[:, :, ::-1])))
        observations_by_frame[index] = observations
        transform = lambda point, torso=torso, head=head: camera_point_to_base(
            point, torso, head[0], head[1]
        )
        candidates = []
        for observation in observations:
            if observation.semantic_class != "cart_body":
                continue
            try:
                candidates.append(reconstruct_cart_top_platform(
                    observation=observation,
                    depth_m=depth,
                    intrinsics=intrinsics,
                    camera_to_base=transform,
                ))
            except ValueError:
                continue
        if not candidates:
            raise ValueError(f"recorded_cart_platform_not_found_frame_{index}")
        platforms[index] = max(candidates, key=lambda item: item.confidence)

    final_rgb, final_depth, final_torso, final_head = frames[placed_index]
    final_transform = lambda point: camera_point_to_base(
        point, final_torso, final_head[0], final_head[1]
    )
    books = [
        observation
        for observation in observations_by_frame[placed_index]
        if observation.semantic_class == "book"
    ]
    if not books and detect_book is not None:
        books = list(detect_book(np.ascontiguousarray(final_rgb[:, :, ::-1])))
    if not books:
        raise ValueError("recorded_placed_book_not_found")
    final_platform = platforms[placed_index]
    book_centers = [
        _mask_center_base(book, final_depth, intrinsics, final_transform)
        for book in books
    ]
    # The recording contains one placed book. If the service returns another
    # book outside the cart, choose the center nearest the loading plane.
    book_center = min(
        book_centers,
        key=lambda point: abs(float(point[2]) - float(final_platform.center[2])),
    )
    left = np.asarray(final_platform.left_edge, dtype=float)
    lateral = np.asarray(final_platform.lateral_axis_right_to_left, dtype=float)
    offset_from_left = float(np.dot(left - book_center, lateral))

    reference_platform = platforms[int(reference_frame_index)]
    _reference_rgb, _depth, reference_torso, reference_head = frames[
        int(reference_frame_index)
    ]
    return ReplayPlaceReference(
        asset_id=str(asset_id),
        reference_frame_index=int(reference_frame_index),
        placed_frame_index=placed_index,
        recorded_torso_m=reference_torso,
        recorded_head_rad=reference_head,
        platform_front_edge_base_m=reference_platform.front_edge,
        platform_left_edge_base_m=reference_platform.left_edge,
        platform_right_edge_base_m=reference_platform.right_edge,
        platform_forward_axis_base=reference_platform.forward_axis,
        platform_lateral_axis_base=reference_platform.lateral_axis_right_to_left,
        platform_width_m=float(reference_platform.lateral_extent_m),
        platform_depth_m=float(reference_platform.depth_extent_m),
        recorded_book_offset_from_left_m=offset_from_left,
    )


def save_replay_place_reference(path, reference):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(asdict(reference), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def load_replay_place_reference(path):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    tuple_fields = (
        "recorded_head_rad",
        "platform_front_edge_base_m",
        "platform_left_edge_base_m",
        "platform_right_edge_base_m",
        "platform_forward_axis_base",
        "platform_lateral_axis_base",
    )
    for name in tuple_fields:
        payload[name] = tuple(float(value) for value in payload[name])
    for name in (
        "recorded_torso_m",
        "platform_width_m",
        "platform_depth_m",
        "recorded_book_offset_from_left_m",
    ):
        payload[name] = float(payload[name])
    payload["reference_frame_index"] = int(payload["reference_frame_index"])
    payload["placed_frame_index"] = int(payload["placed_frame_index"])
    return ReplayPlaceReference(**payload)
