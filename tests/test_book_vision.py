import unittest

import numpy as np

from book_frame import detect_book_frame
from book_geometry import BookMask, CameraIntrinsics


class _FakeBookClient:
    def __init__(self, masks):
        self.masks = masks
        self.calls = []

    def detect(self, image, **identity):
        self.calls.append((image.copy(), identity))
        return self.masks


class BookVisionIntegrationTests(unittest.TestCase):
    def observation(self):
        return BookMask(
            confidence=0.9,
            bbox=(100, 80, 120, 80),
            rle_counts=(0, 120 * 80),
            image_width=320,
            image_height=240,
        )

    def test_reconstructs_client_mask_with_local_depth(self):
        client = _FakeBookClient((self.observation(),))
        color = np.zeros((240, 320, 3), dtype=np.uint8)
        depth = np.zeros((240, 320), dtype=np.float64)
        depth[80:160, 100:220] = 1.0

        result = detect_book_frame(
            book_client=client,
            color_bgr=color,
            depth_m=depth,
            intrinsics=CameraIntrinsics(400.0, 400.0, 0.0, 120.0),
            camera_to_base=lambda point: point,
            captured_at_ns=123,
            base_motion_epoch="base-1",
            head_motion_epoch="head-1",
        )

        self.assertAlmostEqual(result.suction_point[0], 0.38, places=3)
        self.assertAlmostEqual(result.suction_point[1], 0.0, places=3)
        self.assertEqual(client.calls[0][1]["captured_at_ns"], 123)
        self.assertTrue(np.array_equal(client.calls[0][0], color))

    def test_returns_none_when_service_finds_no_book(self):
        client = _FakeBookClient(())
        result = detect_book_frame(
            book_client=client,
            color_bgr=np.zeros((10, 10, 3), dtype=np.uint8),
            depth_m=np.ones((10, 10), dtype=np.float64),
            intrinsics=CameraIntrinsics(100.0, 100.0, 5.0, 5.0),
            camera_to_base=lambda point: point,
            captured_at_ns=1,
            base_motion_epoch="base-1",
            head_motion_epoch="head-1",
        )
        self.assertIsNone(result)

    def test_skips_invalid_high_confidence_mask(self):
        invalid = BookMask(
            confidence=0.99,
            bbox=(0, 0, 20, 20),
            rle_counts=(0, 400),
            image_width=320,
            image_height=240,
        )
        client = _FakeBookClient((invalid, self.observation()))
        depth = np.zeros((240, 320), dtype=np.float64)
        depth[80:160, 100:220] = 1.0

        result = detect_book_frame(
            book_client=client,
            color_bgr=np.zeros((240, 320, 3), dtype=np.uint8),
            depth_m=depth,
            intrinsics=CameraIntrinsics(400.0, 400.0, 0.0, 120.0),
            camera_to_base=lambda point: point,
            captured_at_ns=1,
            base_motion_epoch="base-1",
            head_motion_epoch="head-1",
        )

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.confidence, 0.9)

    def test_reports_selected_observation_for_debug_overlay(self):
        observation = self.observation()
        client = _FakeBookClient((observation,))
        depth = np.zeros((240, 320), dtype=np.float64)
        depth[80:160, 100:220] = 1.0
        selected = []

        detect_book_frame(
            book_client=client,
            color_bgr=np.zeros((240, 320, 3), dtype=np.uint8),
            depth_m=depth,
            intrinsics=CameraIntrinsics(400.0, 400.0, 0.0, 120.0),
            camera_to_base=lambda point: point,
            captured_at_ns=1,
            base_motion_epoch="base-1",
            head_motion_epoch="head-1",
            on_result=lambda mask, geometry: selected.append((mask, geometry)),
        )

        self.assertIs(selected[0][0], observation)
        self.assertAlmostEqual(selected[0][1].suction_point[0], 0.38, places=3)


if __name__ == "__main__":
    unittest.main()
