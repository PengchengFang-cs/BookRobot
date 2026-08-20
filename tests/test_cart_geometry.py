import unittest

import numpy as np

from book_geometry import CameraIntrinsics
from book_rpc import SceneMask
from cart_geometry import (
    reconstruct_cart_body_target,
    reconstruct_cart_platform,
    reconstruct_cart_top_platform,
    select_leftmost_cart_platform,
)


class CartGeometryTests(unittest.TestCase):
    def test_rejects_hollow_wall_slice_above_loading_shelf(self):
        observation = SceneMask(
            semantic_class="cart_body",
            scene_profile_id="cart_loading_v1",
            confidence=0.9,
            bbox=(0, 0, 300, 200),
            rle_counts=(0, 60_000),
            image_width=300,
            image_height=200,
        )
        depth = np.zeros((200, 300), dtype=float)
        # Top cover and loading shelf are filled. The middle candidate is only
        # a U-shaped wall slice and must not count as a physical level.
        depth[5:45, 20:280] = 1.00
        depth[60:64, 20:280] = 0.90
        depth[60:125, 20:24] = 0.90
        depth[60:125, 276:280] = 0.90
        depth[130:195, 20:280] = 0.78

        result = reconstruct_cart_top_platform(
            observation=observation,
            depth_m=depth,
            intrinsics=CameraIntrinsics(300.0, 300.0, 150.0, 100.0),
            camera_to_base=lambda point: (point[1], point[0], point[2]),
        )

        self.assertAlmostEqual(result.center[2], 0.78, places=6)

    def test_skips_top_cover_and_uses_upper_loading_shelf_spacing(self):
        observation = SceneMask(
            semantic_class="cart_body",
            scene_profile_id="cart_loading_v1",
            confidence=0.9,
            bbox=(20, 20, 240, 160),
            rle_counts=(0, 38_400),
            image_width=300,
            image_height=200,
        )
        depth = np.zeros((200, 300), dtype=float)
        depth[25:70, 30:270] = 1.00
        # The same top cover may produce another height band from tilt/noise;
        # it must not be mistaken for a separate loading level.
        depth[70:110, 30:270] = 0.96
        depth[110:175, 30:270] = 0.82

        result = reconstruct_cart_top_platform(
            observation=observation,
            depth_m=depth,
            intrinsics=CameraIntrinsics(300.0, 300.0, 150.0, 100.0),
            camera_to_base=lambda point: (point[1], point[0], point[2]),
        )

        self.assertAlmostEqual(result.center[2], 0.82, places=6)
        self.assertGreater(result.lateral_extent_m, 0.55)
        self.assertGreater(result.depth_extent_m, 0.14)
        self.assertEqual(len(result.slot_centers), 5)
        left = np.asarray(result.left_edge)
        lateral = np.asarray(result.lateral_axis_right_to_left)
        offsets = [
            float(np.dot(left - np.asarray(point), lateral))
            for point in result.slot_centers
        ]
        np.testing.assert_allclose(
            offsets, (0.17, 0.24, 0.31, 0.38, 0.45), atol=0.01
        )
        self.assertEqual(len(result.outline_pixels), 4)

    def test_cart_body_target_uses_mask_center_depth_not_bbox_background(self):
        observation = SceneMask(
            semantic_class="cart_body",
            scene_profile_id="cart_loading_v1",
            confidence=0.8,
            bbox=(20, 20, 100, 100),
            rle_counts=(0, 10_000),
            image_width=160,
            image_height=140,
        )
        depth = np.zeros((140, 160), dtype=float)
        depth[20:120, 20:120] = 1.2
        depth[45, 45] = 2.8

        result = reconstruct_cart_body_target(
            observation=observation,
            depth_m=depth,
            intrinsics=CameraIntrinsics(400.0, 400.0, 80.0, 70.0),
            camera_to_base=lambda point: (point[2], -point[0], -point[1]),
        )

        self.assertAlmostEqual(result.center[0], 1.2)
        self.assertAlmostEqual(result.center[1], 0.0315)
        self.assertAlmostEqual(result.center[2], 0.0015)
        self.assertEqual(result.center_pixel, (69, 69))
        self.assertEqual(result.confidence, 0.8)

    def test_selects_left_cart_plane_instead_of_higher_confidence_table(self):
        table = type("Platform", (), {
            "center": (1.0, -0.2, 0.7),
            "confidence": 0.90,
        })()
        cart = type("Platform", (), {
            "center": (1.2, 0.8, 0.7),
            "confidence": 0.49,
        })()

        self.assertIs(select_leftmost_cart_platform((table, cart)), cart)

    def test_divides_platform_into_five_right_to_left_slots(self):
        observation = SceneMask(
            semantic_class="cart_platform",
            scene_profile_id="cart_loading_v1",
            confidence=0.9,
            bbox=(20, 30, 200, 100),
            rle_counts=(0, 20_000),
            image_width=240,
            image_height=160,
        )
        depth = np.zeros((160, 240), dtype=float)
        depth[30:130, 20:220] = 1.0

        result = reconstruct_cart_platform(
            observation=observation,
            depth_m=depth,
            intrinsics=CameraIntrinsics(400.0, 400.0, 120.0, 80.0),
            camera_to_base=lambda point: (point[1], point[0], point[2]),
        )

        self.assertEqual(len(result.slot_centers), 5)
        self.assertGreater(result.lateral_extent_m, 0.45)
        self.assertGreater(result.depth_extent_m, 0.20)
        self.assertTrue(all(
            result.slot_centers[index][1] < result.slot_centers[index + 1][1]
            for index in range(4)
        ))
        self.assertTrue(all(abs(point[2] - 1.0) < 1e-9 for point in result.slot_centers))


if __name__ == "__main__":
    unittest.main()
