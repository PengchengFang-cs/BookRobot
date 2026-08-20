#!/usr/bin/env python3
"""Capture one live cart RGB-D frame and export it with replay frame zero."""

import argparse
from pathlib import Path
import sys
import time

import h5py
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from book_geometry import CameraIntrinsics, decode_bbox_rle
from book_navigation import load_navnav_runtime
from book_rpc import BookVisionClient
from geometry import camera_point_to_base
from replay_pick_reference import scale_intrinsics


SOURCE_INTRINSICS = CameraIntrinsics(
    fx=1036.581787109375,
    fy=1036.768798828125,
    cx=957.6090087890625,
    cy=533.7483520507812,
)
SOURCE_SIZE = (1920, 1080)


def arguments(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--h5", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--raw-output", required=True)
    parser.add_argument("--reference-frame", type=int, default=0)
    return parser.parse_args(argv)


def _cart_body(observations):
    bodies = [row for row in observations if row.semantic_class == "cart_body"]
    if not bodies:
        raise RuntimeError("cart_body_not_detected")
    return max(bodies, key=lambda row: row.confidence)


def _camera_to_base_matrix(body_m, head_yaw, head_pitch):
    origin = np.asarray(
        camera_point_to_base((0.0, 0.0, 0.0), body_m, head_yaw, head_pitch),
        dtype=float,
    )
    columns = []
    for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
        columns.append(
            np.asarray(
                camera_point_to_base(axis, body_m, head_yaw, head_pitch),
                dtype=float,
            )
            - origin
        )
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.column_stack(columns)
    matrix[:3, 3] = origin
    return matrix


def _masked_cloud(color_bgr, depth_m, intrinsics, observation, camera_to_base):
    mask = decode_bbox_rle(
        image_shape=depth_m.shape,
        bbox=observation.bbox,
        counts=observation.rle_counts,
    )
    valid = (
        mask
        & np.isfinite(depth_m)
        & (depth_m >= 0.20)
        & (depth_m <= 3.0)
    )
    rows, columns = np.nonzero(valid)
    depth = depth_m[rows, columns]
    camera_points = np.column_stack((
        (columns - intrinsics.cx) * depth / intrinsics.fx,
        (rows - intrinsics.cy) * depth / intrinsics.fy,
        depth,
    ))
    base_points = (
        camera_points @ camera_to_base[:3, :3].T
        + camera_to_base[:3, 3]
    )
    colors_rgb = color_bgr[rows, columns, ::-1].astype(np.float32) / 255.0
    return base_points.astype(np.float32), colors_rgb, mask.astype(np.uint8)


def _detect(client, image_bgr, label):
    captured_at_ns = time.time_ns()
    return client.detect_cart(
        np.ascontiguousarray(image_bgr),
        captured_at_ns=captured_at_ns,
        base_motion_epoch=f"cart-registration-{label}-{captured_at_ns}",
        head_motion_epoch=f"cart-registration-{label}-{captured_at_ns}",
    )


def main(argv=None):
    args = arguments(argv)

    import cv2
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener
    from vision import Vision

    rclpy.init()
    runtime = load_navnav_runtime()
    adapter = runtime.WandaRos2Adapter()
    try:
        adapter.preflight()
        adapter.execute_command(
            runtime.MappedMotionCommand(
                runtime.WandaCommandKind.TORSO_POSITION, 0.20, "Z"
            ),
            precision_mode=True,
        )
    finally:
        adapter.stop()
        adapter.destroy_node()

    node = Node("fpc_cart_registration_capture")
    tf_buffer = Buffer()
    _tf_listener = TransformListener(tf_buffer, node)
    vision = Vision(node, tf_buffer)
    try:
        vision.set_head_pose(
            yaw_rad=0.0003601923480223146,
            pitch_rad=0.24972170937534868,
        )
        captured = vision.capture_cart_frame(scan_angle_deg=0, timeout_s=5.0)
        if captured is None:
            raise RuntimeError("live_rgbd_capture_failed")
        current_color, current_depth = vision._snapshot_arrays(captured.snapshot)
        current_intrinsics = CameraIntrinsics(
            fx=float(captured.snapshot.info.k[0]),
            fy=float(captured.snapshot.info.k[4]),
            cx=float(captured.snapshot.info.k[2]),
            cy=float(captured.snapshot.info.k[5]),
        )
        current_body = float(captured.joints["body_joint"])
        current_head = (
            float(captured.joints["joint_head0"]),
            float(captured.joints["joint_head1"]),
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    with h5py.File(args.h5, "r") as recording:
        index = int(args.reference_frame)
        reference_rgb = np.asarray(recording["observation/image"][index])
        reference_color = np.ascontiguousarray(reference_rgb[:, :, ::-1])
        reference_depth = (
            np.asarray(recording["observations/depth_head_rgbd"][index], dtype=float)
            / 1000.0
        )
        reference_body = float(recording["observations/qpos_torso"][index][0])
        reference_head = tuple(
            float(value) for value in recording["observations/qpos_head"][index]
        )

    reference_intrinsics = scale_intrinsics(
        SOURCE_INTRINSICS,
        source_size=SOURCE_SIZE,
        target_size=(reference_color.shape[1], reference_color.shape[0]),
    )

    client = BookVisionClient.from_config()
    try:
        current_observation = _cart_body(_detect(client, current_color, "live"))
        reference_observation = _cart_body(
            _detect(client, reference_color, "replay-frame-zero")
        )
    finally:
        client.close()

    current_camera_to_base = _camera_to_base_matrix(
        current_body, current_head[0], current_head[1]
    )
    reference_camera_to_base = _camera_to_base_matrix(
        reference_body, reference_head[0], reference_head[1]
    )
    current_points, current_colors, current_mask = _masked_cloud(
        current_color,
        current_depth,
        current_intrinsics,
        current_observation,
        current_camera_to_base,
    )
    reference_points, reference_colors, reference_mask = _masked_cloud(
        reference_color,
        reference_depth,
        reference_intrinsics,
        reference_observation,
        reference_camera_to_base,
    )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        current_color_bgr=current_color,
        current_depth_m=current_depth.astype(np.float32),
        current_intrinsics=np.asarray((
            current_intrinsics.fx,
            current_intrinsics.fy,
            current_intrinsics.cx,
            current_intrinsics.cy,
        )),
        current_camera_to_base=current_camera_to_base,
        current_points_base=current_points,
        current_colors_rgb=current_colors,
        current_mask=current_mask,
        reference_color_bgr=reference_color,
        reference_depth_m=reference_depth.astype(np.float32),
        reference_intrinsics=np.asarray((
            reference_intrinsics.fx,
            reference_intrinsics.fy,
            reference_intrinsics.cx,
            reference_intrinsics.cy,
        )),
        reference_camera_to_base=reference_camera_to_base,
        reference_points_base=reference_points,
        reference_colors_rgb=reference_colors,
        reference_mask=reference_mask,
    )
    raw_output = Path(args.raw_output)
    raw_output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(raw_output), current_color)
    print(f"pair={output}")
    print(f"raw={raw_output}")
    print(f"current_points={current_points.shape[0]}")
    print(f"reference_points={reference_points.shape[0]}")


if __name__ == "__main__":
    main()
