import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import h5py
import numpy as np

from book_geometry import BookGeometry, BookMask, CameraIntrinsics
from replay_pick_reference import (
    ReplayPickReference,
    calibrate_replay_pick_reference,
    find_blue_book_contact_pixel,
    find_blue_suction_contact_tip,
    find_blue_suction_center,
    load_replay_pick_reference,
    project_recorded_contact,
    project_recorded_contact_to_cover,
    save_replay_pick_reference,
    scale_intrinsics,
)


class ReplayPickReferenceTests(unittest.TestCase):
    def test_calibrates_reference_from_synthetic_hdf5(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pick.h5"
            images = np.zeros((4, 20, 20, 3), dtype=np.uint8)
            images[3, 10:14, 14:18] = (0, 0, 255)
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
                suction_point=(0.35, 0.25, 1.0),
                long_axis=(1.0, 0.0, 0.0),
                short_axis_right_to_left=(0.0, 1.0, 0.0),
                long_extent_m=0.30,
                short_extent_m=0.20,
                confidence=0.9,
                long_inset_m=0.13,
                right_inset_m=0.10,
            )
            observation = BookMask(
                confidence=0.9,
                bbox=(4, 4, 10, 10),
                rle_counts=(0, 100),
                image_width=20,
                image_height=20,
            )

            result = calibrate_replay_pick_reference(
                h5_path=path,
                asset_id="S1_TABLE_PICK_BOOK",
                reference_frame_index=0,
                contact_frame_index=3,
                suction_roi_xywh=(10, 8, 10, 8),
                source_intrinsics=CameraIntrinsics(10.0, 10.0, 10.0, 10.0),
                source_size=(20, 20),
                detect_recorded_book=lambda *_args: (
                    SimpleNamespace(
                        observation=observation,
                        geometry=geometry,
                    ),
                ),
                camera_to_base=lambda point, *_state: point,
            )

        self.assertEqual(result.reference_frame_index, 0)
        self.assertEqual(result.contact_frame_index, 3)
        self.assertEqual(result.asset_id, "S1_TABLE_PICK_BOOK")
        self.assertEqual(len(result.hdf5_sha256), 64)
        np.testing.assert_allclose(
            result.recorded_book_rule_point_base_m,
            (0.35, 0.25, 1.0),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            result.recorded_contact_point_base_m,
            (0.55, 0.0, 1.0),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            result.recorded_book_suction_point_base_m,
            (0.35, 0.0, 1.0),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            result.recorded_contact_pixel,
            (15.5, 10.0),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            result.recorded_book_long_axis_base,
            (1.0, 0.0, 0.0),
            atol=1e-12,
        )
        self.assertAlmostEqual(result.recorded_book_long_extent_m, 0.30)
        self.assertAlmostEqual(result.recorded_book_short_extent_m, 0.20)

    def test_scales_x_and_y_intrinsics_independently(self):
        scaled = scale_intrinsics(
            CameraIntrinsics(1000.0, 900.0, 960.0, 540.0),
            source_size=(1920, 1080),
            target_size=(224, 224),
        )

        self.assertAlmostEqual(scaled.fx, 1000.0 * 224.0 / 1920.0)
        self.assertAlmostEqual(scaled.fy, 900.0 * 224.0 / 1080.0)
        self.assertAlmostEqual(
            scaled.cx,
            (960.0 + 0.5) * 224.0 / 1920.0 - 0.5,
        )
        self.assertAlmostEqual(
            scaled.cy,
            (540.0 + 0.5) * 224.0 / 1080.0 - 0.5,
        )

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

    def test_finds_robot_side_blue_tip_at_component_top_edge(self):
        baseline = np.zeros((40, 60, 3), dtype=np.uint8)
        contact = baseline.copy()
        contact[10:20, 30:36] = (0, 0, 255)
        contact[10, 29] = (0, 0, 255)

        tip = find_blue_suction_contact_tip(
            baseline_rgb=baseline,
            contact_rgb=contact,
            roi_xywh=(20, 5, 30, 25),
        )

        self.assertEqual(tip, (32.0, 10.0))

    def test_finds_book_surface_point_nearest_blue_suction_tip(self):
        baseline = np.zeros((20, 20, 3), dtype=np.uint8)
        contact = baseline.copy()
        contact[10:14, 14:18] = (0, 0, 255)
        book_mask = np.zeros((20, 20), dtype=bool)
        book_mask[4:14, 4:14] = True

        point = find_blue_book_contact_pixel(
            baseline_rgb=baseline,
            contact_rgb=contact,
            roi_xywh=(10, 8, 10, 8),
            book_mask=book_mask,
        )

        self.assertEqual(point, (13.0, 11.5))

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

    def test_projects_contact_ray_to_recorded_cover_plane(self):
        result = project_recorded_contact_to_cover(
            center_px=(2.0, 1.0),
            intrinsics=CameraIntrinsics(2.0, 2.0, 0.0, 0.0),
            torso_head_states=[(0.2, 0.0, 0.25)] * 3,
            cover_z_base_m=2.0,
            camera_to_base=lambda point, *_state: point,
        )

        self.assertEqual(result.sample_count, 3)
        np.testing.assert_allclose(
            result.reference_contact_base_m,
            (2.0, 1.0, 2.0),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            result.axis_spread_m,
            (0.0, 0.0, 0.0),
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
            reference_frame_index=0,
            contact_frame_index=158,
            recorded_book_rule_point_base_m=(0.935, -0.304, 0.754),
            recorded_contact_point_base_m=(0.68, -0.281, 0.754),
            recorded_contact_pixel=(118.5, 177.0),
            recorded_book_suction_point_base_m=(0.935, -0.281, 0.754),
            recorded_book_long_axis_base=(1.0, 0.0, 0.0),
            recorded_book_long_extent_m=0.30,
            recorded_book_short_extent_m=0.20,
            hdf5_sha256="a" * 64,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            save_replay_pick_reference(path, reference)

            loaded = load_replay_pick_reference(path)

        self.assertEqual(loaded, reference)


if __name__ == "__main__":
    unittest.main()
