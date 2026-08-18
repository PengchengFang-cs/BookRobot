import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from book_geometry import BookGeometry, CameraIntrinsics
from replay_pick_reference import (
    ReplayPickReference,
    calibrate_replay_pick_reference,
    find_blue_suction_center,
    load_replay_pick_reference,
    project_recorded_contact,
    save_replay_pick_reference,
    scale_intrinsics,
)


class ReplayPickReferenceTests(unittest.TestCase):
    def test_calibrates_reference_from_synthetic_hdf5(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pick.h5"
            images = np.zeros((4, 20, 20, 3), dtype=np.uint8)
            images[3, 10:14, 10:14] = (0, 0, 255)
            depths = np.full((4, 20, 20), 1000, dtype=np.uint16)
            with h5py.File(path, "w") as recording:
                recording.create_dataset("observation/image", data=images)
                recording.create_dataset(
                    "observations/depth_head_rgbd", data=depths
                )
                recording.create_dataset(
                    "observations/qpos_head", data=np.zeros((4, 2))
                )
                recording.create_dataset(
                    "observations/qpos_torso", data=np.zeros((4, 1))
                )

            geometry = BookGeometry(
                suction_point=(0.20, 0.20, 1.0),
                long_axis=(1.0, 0.0, 0.0),
                short_axis_right_to_left=(0.0, 1.0, 0.0),
                long_extent_m=0.30,
                short_extent_m=0.20,
                confidence=0.9,
                long_inset_m=0.13,
                right_inset_m=0.10,
            )

            result = calibrate_replay_pick_reference(
                h5_path=path,
                asset_id="S1_TABLE_PICK_BOOK",
                contact_frame_index=3,
                early_frame_indices=(0, 1, 2),
                source_intrinsics=CameraIntrinsics(10.0, 10.0, 10.0, 10.0),
                source_size=(20, 20),
                suction_roi_xywh=(8, 8, 8, 8),
                detect_recorded_book=lambda *_args: geometry,
                camera_to_base=lambda point, *_state: point,
            )

        self.assertEqual(result.contact_frame_index, 3)
        self.assertEqual(result.early_frame_indices, (0, 1, 2))
        self.assertEqual(result.suction_center_px, (11.5, 11.5))
        np.testing.assert_allclose(
            result.reference_contact_base_m, (0.15, 0.15, 1.0), atol=1e-12
        )
        self.assertAlmostEqual(result.long_inset_m, 0.08)
        self.assertAlmostEqual(result.right_inset_m, 0.05)
        self.assertEqual(result.sample_count, 3)
        self.assertEqual(result.axis_spread_m, (0.0, 0.0, 0.0))

    def test_scales_x_and_y_intrinsics_independently(self):
        scaled = scale_intrinsics(
            CameraIntrinsics(1000.0, 900.0, 960.0, 540.0),
            source_size=(1920, 1080),
            target_size=(224, 224),
        )

        self.assertAlmostEqual(scaled.fx, 1000.0 * 224.0 / 1920.0)
        self.assertAlmostEqual(scaled.fy, 900.0 * 224.0 / 1080.0)
        self.assertAlmostEqual(scaled.cx, 960.0 * 224.0 / 1920.0)
        self.assertAlmostEqual(scaled.cy, 540.0 * 224.0 / 1080.0)

    def test_finds_largest_new_blue_component_inside_roi(self):
        baseline = np.zeros((40, 60, 3), dtype=np.uint8)
        contact = baseline.copy()
        contact[4:8, 4:8] = (0, 0, 255)
        contact[20:30, 35:45] = (0, 0, 255)

        center = find_blue_suction_center(
            baseline_rgb=baseline,
            contact_rgb=contact,
            roi_xywh=(30, 15, 20, 20),
        )

        self.assertEqual(center, (39.5, 24.5))

    def test_rejects_contact_frame_without_new_blue_component(self):
        image = np.zeros((20, 20, 3), dtype=np.uint8)

        with self.assertRaisesRegex(ValueError, "blue_suction_not_found"):
            find_blue_suction_center(
                baseline_rgb=image,
                contact_rgb=image.copy(),
                roi_xywh=(0, 0, 20, 20),
            )

    def test_projects_early_depth_frames_and_uses_axis_median(self):
        depths = []
        for millimetres in (900, 1000, 1100):
            depth = np.zeros((5, 5), dtype=np.uint16)
            depth[:, :] = millimetres
            depths.append(depth)

        result = project_recorded_contact(
            center_px=(2.0, 1.0),
            depths_mm=depths,
            intrinsics=CameraIntrinsics(2.0, 2.0, 0.0, 0.0),
            torso_head_states=[(0.2, 0.0, 0.25)] * 3,
            camera_to_base=lambda point, *_state: point,
        )

        self.assertEqual(result.sample_count, 3)
        np.testing.assert_allclose(
            result.reference_contact_base_m,
            (1.0, 0.5, 1.0),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            result.axis_spread_m,
            (0.2, 0.1, 0.2),
            atol=1e-12,
        )

    def test_rejects_projection_when_all_depth_patches_are_empty(self):
        with self.assertRaisesRegex(ValueError, "recorded_contact_depth_missing"):
            project_recorded_contact(
                center_px=(2.0, 2.0),
                depths_mm=[np.zeros((5, 5), dtype=np.uint16)],
                intrinsics=CameraIntrinsics(2.0, 2.0, 0.0, 0.0),
                torso_head_states=[(0.2, 0.0, 0.25)],
                camera_to_base=lambda point, *_state: point,
            )

    def test_reference_json_round_trip_preserves_numeric_types(self):
        reference = ReplayPickReference(
            asset_id="S1_TABLE_PICK_BOOK",
            contact_frame_index=300,
            early_frame_indices=(0, 10, 20, 40, 80),
            suction_center_px=(164.17, 196.70),
            reference_contact_base_m=(0.715, -0.391, 0.713),
            long_inset_m=0.075,
            right_inset_m=0.042,
            sample_count=5,
            axis_spread_m=(0.001, 0.002, 0.003),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            save_replay_pick_reference(path, reference)

            loaded = load_replay_pick_reference(path)

        self.assertEqual(loaded, reference)


if __name__ == "__main__":
    unittest.main()
