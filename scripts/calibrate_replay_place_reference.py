#!/usr/bin/env python3
"""Create the Place-2.4 cart reference from read-only recorded RGB-D."""

import argparse
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from book_geometry import CameraIntrinsics
from book_rpc import BookVisionClient
from replay_place_reference import (
    calibrate_replay_place_reference,
    save_replay_place_reference,
)


SOURCE_INTRINSICS = CameraIntrinsics(
    fx=1036.581787109375,
    fy=1036.768798828125,
    cx=957.6090087890625,
    cy=533.7483520507812,
)
SOURCE_SIZE = (1920, 1080)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description="Calibrate Place 2.4; no motion")
    parser.add_argument("--h5", required=True)
    parser.add_argument("--asset-id", default="S1_CART_PLACE_BOOK")
    parser.add_argument("--reference-frame", type=int, default=0)
    parser.add_argument("--placed-frame", type=int, default=-1)
    parser.add_argument("--output", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = arguments(argv)
    client = BookVisionClient.from_config()

    def detect(image_bgr):
        last_error = None
        for attempt in range(3):
            captured_at_ns = time.time_ns()
            try:
                return client.detect_cart(
                    image_bgr,
                    captured_at_ns=captured_at_ns,
                    base_motion_epoch=f"place-calibration-base-{captured_at_ns}",
                    head_motion_epoch=f"place-calibration-head-{captured_at_ns}",
                )
            except Exception as error:
                last_error = error
                if attempt < 2:
                    time.sleep(0.2 * (attempt + 1))
        raise last_error

    try:
        reference = calibrate_replay_place_reference(
            h5_path=args.h5,
            asset_id=args.asset_id,
            source_intrinsics=SOURCE_INTRINSICS,
            source_size=SOURCE_SIZE,
            detect_cart=detect,
            reference_frame_index=args.reference_frame,
            placed_frame_index=args.placed_frame,
        )
    finally:
        client.close()
    save_replay_place_reference(args.output, reference)
    print(Path(args.output).read_text(encoding="utf-8"), end="")


if __name__ == "__main__":
    main()
