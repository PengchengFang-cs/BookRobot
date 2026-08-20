import unittest

import numpy as np

from book_geometry import CameraIntrinsics
from book_rpc import SceneMask
from cart_geometry import (
    reconstruct_cart_body_target,
    reconstruct_cart_platform,
    select_leftmost_cart_platform,
)


class CartGeometryTests(unittest.TestCase):
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
