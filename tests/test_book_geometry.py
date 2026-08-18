import unittest

import numpy as np

from book_geometry import (
    BookGeometry,
    BookGeometryError,
    BookMask,
    CameraIntrinsics,
    contact_point_for_insets,
    decode_bbox_rle,
    insets_for_contact_point,
    reconstruct_table_book,
)


class BookGeometryTests(unittest.TestCase):
    @staticmethod
    def _geometry():
        return BookGeometry(
            suction_point=(0.38, 0.0, 1.0),
            long_axis=(1.0, 0.0, 0.0),
            short_axis_right_to_left=(0.0, 1.0, 0.0),
            long_extent_m=0.30,
            short_extent_m=0.20,
            confidence=0.9,
            long_inset_m=0.13,
            right_inset_m=0.10,
        )

    def test_decodes_bbox_local_rle_into_full_image(self):
        mask = decode_bbox_rle(
            image_shape=(120, 160),
            bbox=(40, 30, 80, 60),
            counts=(0, 4800),
        )

        self.assertEqual(mask.shape, (120, 160))
        self.assertEqual(int(mask.sum()), 4800)
        self.assertTrue(mask[30, 40])
        self.assertTrue(mask[89, 119])
        self.assertFalse(mask[29, 40])

    def test_rejects_rle_whose_runs_do_not_fill_bbox(self):
        with self.assertRaisesRegex(BookGeometryError, "mask_rle_size_mismatch"):
            decode_bbox_rle(
                image_shape=(20, 20),
                bbox=(2, 3, 5, 4),
                counts=(0, 19),
            )

    def test_reconstructs_fixed_inset_suction_point(self):
        depth = np.zeros((240, 320), dtype=np.float64)
        depth[80:160, 100:220] = 1.0
        observation = BookMask(
            confidence=0.9,
            bbox=(100, 80, 120, 80),
            rle_counts=(0, 120 * 80),
            image_width=320,
            image_height=240,
        )

        result = reconstruct_table_book(
            observation=observation,
            depth_m=depth,
            intrinsics=CameraIntrinsics(fx=400.0, fy=400.0, cx=0.0, cy=120.0),
            camera_to_base=lambda point: point,
            long_inset_m=0.13,
            right_inset_m=0.10,
        )

        # The synthetic book spans x=0.25..0.5475 m and y=-0.10..0.0975 m.
        self.assertAlmostEqual(result.suction_point[0], 0.38, places=3)
        self.assertAlmostEqual(result.suction_point[1], 0.0, places=3)
        self.assertAlmostEqual(result.suction_point[2], 1.0, places=6)
        self.assertAlmostEqual(result.long_extent_m, 0.2975, places=3)
        self.assertAlmostEqual(result.short_extent_m, 0.1975, places=3)
        self.assertAlmostEqual(result.long_inset_m, 0.13)
        self.assertAlmostEqual(result.right_inset_m, 0.10)

    def test_recovers_and_reapplies_contact_insets(self):
        geometry = self._geometry()
        contact = (0.31, -0.08, geometry.suction_point[2])

        long_inset, right_inset = insets_for_contact_point(geometry, contact)
        rebuilt = contact_point_for_insets(
            geometry,
            long_inset_m=long_inset,
            right_inset_m=right_inset,
        )

        self.assertAlmostEqual(long_inset, 0.06)
        self.assertAlmostEqual(right_inset, 0.02)
        np.testing.assert_allclose(rebuilt, contact, atol=1e-12)

    def test_asset_contact_rotates_with_current_book_axes(self):
        geometry = BookGeometry(
            suction_point=(0.0, 0.0, 0.75),
            long_axis=(0.0, 1.0, 0.0),
            short_axis_right_to_left=(-1.0, 0.0, 0.0),
            long_extent_m=0.30,
            short_extent_m=0.20,
            confidence=0.9,
            long_inset_m=0.13,
            right_inset_m=0.10,
        )

        point = contact_point_for_insets(
            geometry,
            long_inset_m=0.07,
            right_inset_m=0.04,
        )

        np.testing.assert_allclose(point, (0.06, -0.06, 0.75), atol=1e-12)

    def test_default_insets_return_existing_suction_point(self):
        geometry = self._geometry()

        point = contact_point_for_insets(
            geometry,
            long_inset_m=geometry.long_inset_m,
            right_inset_m=geometry.right_inset_m,
        )

        self.assertEqual(point, geometry.suction_point)

    def test_rejects_contact_insets_outside_reconstructed_book(self):
        with self.assertRaisesRegex(BookGeometryError, "contact_insets_out_of_bounds"):
            contact_point_for_insets(
                self._geometry(),
                long_inset_m=0.31,
                right_inset_m=0.05,
            )

    def test_rejects_insufficient_valid_depth(self):
        observation = BookMask(
            confidence=0.8,
            bbox=(10, 10, 40, 40),
            rle_counts=(0, 1600),
            image_width=80,
            image_height=80,
        )
        with self.assertRaisesRegex(BookGeometryError, "insufficient_book_depth"):
            reconstruct_table_book(
                observation=observation,
                depth_m=np.zeros((80, 80), dtype=np.float64),
                intrinsics=CameraIntrinsics(400.0, 400.0, 0.0, 0.0),
                camera_to_base=lambda point: point,
            )

    def test_rejects_book_too_small_for_fixed_inset(self):
        depth = np.zeros((80, 80), dtype=np.float64)
        depth[10:50, 10:50] = 1.0
        observation = BookMask(
            confidence=0.8,
            bbox=(10, 10, 40, 40),
            rle_counts=(0, 1600),
            image_width=80,
            image_height=80,
        )
        with self.assertRaisesRegex(BookGeometryError, "book_too_small_for_insets"):
            reconstruct_table_book(
                observation=observation,
                depth_m=depth,
                intrinsics=CameraIntrinsics(400.0, 400.0, 0.0, 0.0),
                camera_to_base=lambda point: point,
            )


if __name__ == "__main__":
    unittest.main()
