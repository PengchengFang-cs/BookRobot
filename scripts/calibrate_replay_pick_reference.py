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
        description="Read a Pick HDF5 and calibrate its blue suction contact; no motion."
    )
    parser.add_argument("--h5", required=True, help="read-only DataReplay HDF5")
    parser.add_argument("--asset-id", default="S1_TABLE_PICK_BOOK")
    parser.add_argument("--contact-frame", type=int, default=300)
    parser.add_argument("--early-frames", default="0,10,20,40,80")
    parser.add_argument("--suction-roi", default="145,170,45,54")
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
        early = np.asarray(images[reference.early_frame_indices[0]])[:, :, ::-1].copy()
        contact = np.asarray(images[reference.contact_frame_index])[:, :, ::-1].copy()
    center = tuple(int(round(value)) for value in reference.suction_center_px)
    for image, label in ((early, "early projection"), (contact, "blue suction")):
        cv2.circle(image, center, 5, (0, 0, 255), 2)
        cv2.putText(
            image,
            label,
            (5, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    overlay = np.concatenate((early, contact), axis=1)
    xyz = reference.reference_contact_base_m
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
    early_frames = _csv_ints(args.early_frames)
    suction_roi = _csv_ints(args.suction_roi, count=4)
    client = BookVisionClient.from_config()
    try:
        reference = calibrate_replay_pick_reference(
            h5_path=args.h5,
            asset_id=args.asset_id,
            contact_frame_index=args.contact_frame,
            early_frame_indices=early_frames,
            source_intrinsics=SOURCE_INTRINSICS,
            source_size=SOURCE_SIZE,
            suction_roi_xywh=suction_roi,
            detect_recorded_book=_detector(client),
            camera_to_base=camera_point_to_base,
        )
    finally:
        client.close()
    save_replay_pick_reference(args.output, reference)
    _write_overlay(args.h5, reference, args.overlay)
    print(f"reference={args.output}")
    print(f"overlay={args.overlay}")
    print(f"suction_center_px={reference.suction_center_px}")
    print(f"reference_contact_base_m={reference.reference_contact_base_m}")
    print(
        "book_insets_m="
        f"({reference.long_inset_m}, {reference.right_inset_m})"
    )
    print(f"axis_spread_m={reference.axis_spread_m}")


if __name__ == "__main__":
    main()
