import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import h5py
import numpy as np

from book_geometry import CameraIntrinsics
from replay_place_reference import (
    ReplayPlaceReference,
    calibrate_replay_place_reference,
    load_replay_place_reference,
    save_replay_place_reference,
)


def _platform(*, left, center, confidence=0.9):
    return SimpleNamespace(
        center=center,
        front_edge=(center[0] - 0.1, center[1], center[2]),
        left_edge=left,
        right_edge=(left[0], left[1] - 0.75, left[2]),
        forward_axis=(1.0, 0.0, 0.0),
        lateral_axis_right_to_left=(0.0, 1.0, 0.0),
        lateral_extent_m=0.75,
        depth_extent_m=0.20,
        confidence=confidence,
    )


class ReplayPlaceReferenceTests(unittest.TestCase):
    def test_calibrates_frame_zero_platform_and_recorded_book_offset(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "place.h5"
            with h5py.File(path, "w") as recording:
                recording.create_dataset(
                    "observation/image",
                    data=np.zeros((2, 12, 16, 3), dtype=np.uint8),
                )
                recording.create_dataset(
                    "observations/depth_head_rgbd",
                    data=np.full((2, 12, 16), 1000, dtype=np.uint16),
                )
                recording.create_dataset(
                    "observations/qpos_torso",
                    data=np.asarray([[0.2], [0.2]]),
                )
                recording.create_dataset(
                    "observations/qpos_head",
                    data=np.asarray([[0.01, 0.25], [0.02, 0.26]]),
                )

            observation = SimpleNamespace(semantic_class="cart_body")
            book = SimpleNamespace(semantic_class="book")
            reference_platform = _platform(
                left=(1.0, 0.40, 0.9), center=(1.0, 0.025, 0.9)
            )
            final_platform = _platform(
                left=(1.2, 0.50, 0.9), center=(1.2, 0.125, 0.9)
            )
            platforms = iter((reference_platform, final_platform))
            with (
                patch(
                    "replay_place_reference.reconstruct_cart_top_platform",
                    side_effect=lambda **_kwargs: next(platforms),
                ),
                patch(
                    "replay_place_reference._mask_center_base",
                    return_value=np.asarray((1.2, 0.33, 0.9)),
                ),
            ):
                result = calibrate_replay_place_reference(
                    h5_path=path,
                    asset_id="S1_CART_PLACE_BOOK",
                    source_intrinsics=CameraIntrinsics(10.0, 10.0, 8.0, 6.0),
                    source_size=(16, 12),
                    detect_cart=lambda _image: (observation, book),
                    reference_frame_index=0,
                    placed_frame_index=1,
                )

        self.assertEqual(result.reference_frame_index, 0)
        self.assertEqual(result.placed_frame_index, 1)
        self.assertAlmostEqual(result.recorded_torso_m, 0.2)
        self.assertEqual(result.recorded_head_rad, (0.01, 0.25))
        self.assertEqual(result.platform_front_edge_base_m, (0.9, 0.025, 0.9))
        self.assertAlmostEqual(result.recorded_book_offset_from_left_m, 0.17)

    def test_json_round_trip_restores_numeric_tuples(self):
        reference = ReplayPlaceReference(
            asset_id="S1_CART_PLACE_BOOK",
            reference_frame_index=0,
            placed_frame_index=387,
            recorded_torso_m=0.2,
            recorded_head_rad=(0.0, 0.25),
            platform_front_edge_base_m=(0.8, 0.0, 0.9),
            platform_left_edge_base_m=(0.9, 0.4, 0.9),
            platform_right_edge_base_m=(0.9, -0.35, 0.9),
            platform_forward_axis_base=(1.0, 0.0, 0.0),
            platform_lateral_axis_base=(0.0, 1.0, 0.0),
            platform_width_m=0.75,
            platform_depth_m=0.2,
            recorded_book_offset_from_left_m=0.17,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            save_replay_place_reference(path, reference)
            loaded = load_replay_place_reference(path)

        self.assertEqual(loaded, reference)


if __name__ == "__main__":
    unittest.main()
