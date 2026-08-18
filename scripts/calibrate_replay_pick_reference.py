#!/usr/bin/env python3
"""Calibrate one Stage-1 replay contact from read-only recorded RGB-D."""

import argparse
from pathlib import Path
import sys
import time

import h5py
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from book_frame import detect_book_frame
from book_geometry import CameraIntrinsics
from book_rpc import BookVisionClient
from geometry import camera_point_to_base
from replay_pick_reference import (
    calibrate_replay_pick_reference,
    save_replay_pick_reference,
)


SOURCE_INTRINSICS = CameraIntrinsics(
    fx=1036.581787109375,
    fy=1036.768798828125,
    cx=957.6090087890625,
    cy=533.7483520507812,
)
SOURCE_SIZE = (1920, 1080)


def _csv_ints(value, *, count=None):
    result = tuple(int(part.strip()) for part in str(value).split(","))
    if count is not None and len(result) != count:
        raise argparse.ArgumentTypeError(f"expected {count} comma-separated integers")
    return result


def arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Detect the recorded book suction point at replay frame 0; no motion."
    )
    parser.add_argument("--h5", required=True, help="read-only DataReplay HDF5")
    parser.add_argument("--asset-id", default="S1_TABLE_PICK_BOOK")
    parser.add_argument("--reference-frame", type=int, default=0)
    parser.add_argument(
        "--output",
        default=str(ROOT / "config" / "stage1_pick_reference.json"),
    )
    parser.add_argument(
        "--overlay",
        default=str(ROOT / "logs" / "stage1_pick_reference_overlay.jpg"),
    )
    return parser.parse_args(argv)


def _detector(client):
    def detect(rgb, depth_mm, intrinsics, torso, head):
        captured_at_ns = time.time_ns()
        books = detect_book_frame(
            book_client=client,
            color_bgr=np.ascontiguousarray(np.asarray(rgb)[:, :, ::-1]),
            depth_m=np.asarray(depth_mm, dtype=float) / 1000.0,
            intrinsics=intrinsics,
            camera_to_base=lambda point: camera_point_to_base(
                point, torso, head[0], head[1]
            ),
            captured_at_ns=captured_at_ns,
            base_motion_epoch="stage1-replay-calibration-base",
            head_motion_epoch="stage1-replay-calibration-head",
        )
        if not books:
            raise RuntimeError("recorded_book_not_detected")
        return books[0]

    return detect


def _write_overlay(h5_path, reference, output_path):
    import cv2

    with h5py.File(h5_path, "r") as recording:
        images = recording["observation/image"]
        overlay = np.asarray(images[reference.reference_frame_index])[:, :, ::-1].copy()
    cv2.putText(
        overlay,
        f"DataReplay reference frame {reference.reference_frame_index}",
        (5, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    xyz = reference.recorded_book_suction_point_base_m
    cv2.putText(
        overlay,
        f"base=({xyz[0]:.4f},{xyz[1]:.4f},{xyz[2]:.4f})m",
        (5, overlay.shape[0] - 7),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), overlay):
        raise RuntimeError(f"overlay_write_failed: {path}")


def main(argv=None):
    args = arguments(argv)
    client = BookVisionClient.from_config()
    try:
        reference = calibrate_replay_pick_reference(
            h5_path=args.h5,
            asset_id=args.asset_id,
            reference_frame_index=args.reference_frame,
            source_intrinsics=SOURCE_INTRINSICS,
            source_size=SOURCE_SIZE,
            detect_recorded_book=_detector(client),
            camera_to_base=camera_point_to_base,
        )
    finally:
        client.close()
    save_replay_pick_reference(args.output, reference)
    _write_overlay(args.h5, reference, args.overlay)
    print(f"reference={args.output}")
    print(f"overlay={args.overlay}")
    print(
        "recorded_book_suction_point_base_m="
        f"{reference.recorded_book_suction_point_base_m}"
    )


if __name__ == "__main__":
    main()
