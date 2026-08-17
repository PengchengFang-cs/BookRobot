import unittest
from types import SimpleNamespace

import numpy as np

from book_rpc import BookVisionClient, BookVisionSettings


class _Repeated(list):
    def add(self):
        value = SimpleNamespace(color=SimpleNamespace())
        self.append(value)
        return value


class _InferRequest:
    def __init__(self):
        self.header = SimpleNamespace()
        self.captures = _Repeated()
        self.task = ""
        self.expected_source = ""
        self.expected_worker_id = ""
        self.expected_model_version = ""


class _FakePb2:
    InferRequest = _InferRequest


def _scene_row(*, semantic_class="book", profile="table_books_v1", confidence=0.8):
    return SimpleNamespace(
        semantic_class=semantic_class,
        scene_profile_id=profile,
        confidence=confidence,
        image_width=64,
        image_height=48,
        bbox_px=SimpleNamespace(x=10, y=4, width=30, height=40),
        mask=SimpleNamespace(counts=(0, 1200)),
    )


class BookRpcTests(unittest.TestCase):
    def settings(self):
        return BookVisionSettings(
            endpoint="127.0.0.1:7443",
            server_name="planning-server.bookbot.internal",
            ca_certificate_path="/not/read/in/unit/test/ca.pem",
            client_certificate_path="/not/read/in/unit/test/client.pem",
            client_private_key_path="/not/read/in/unit/test/client.key",
            expected_source="ruan-unified-vision",
            expected_worker_id="ruan-5090-worker-0",
            expected_model_version="model-r9",
            config_hash="a" * 64,
            calibration_version="wanda-head-rgbd-v1",
            timeout_s=2.5,
        )

    def test_builds_fixed_profile_request_and_decodes_book_mask(self):
        calls = []

        def rpc(request, timeout):
            calls.append((request, timeout))
            return SimpleNamespace(
                task="scene_table_books_segmentation",
                scene_instances=[_scene_row(confidence=0.91)],
            )

        client = BookVisionClient(self.settings(), pb2_module=_FakePb2, rpc=rpc)
        image = np.arange(48 * 64 * 3, dtype=np.uint8).reshape((48, 64, 3))

        result = client.detect(
            image,
            captured_at_ns=123456,
            base_motion_epoch="base-static-1",
            head_motion_epoch="head-static-1",
        )

        request, timeout = calls[0]
        self.assertEqual(timeout, 2.5)
        self.assertEqual(request.task, "scene_table_books_segmentation")
        self.assertEqual(request.header.expected_output_frame, "image")
        self.assertEqual(request.header.calibration_version, "wanda-head-rgbd-v1")
        self.assertEqual(request.captures[0].camera_id, "head_rgbd")
        self.assertEqual(request.captures[0].color.encoding, "bgr8")
        self.assertEqual(request.captures[0].color.payload, image.tobytes())
        self.assertEqual(request.captures[0].base_motion_epoch, "base-static-1")
        self.assertEqual(result[0].bbox, (10, 4, 30, 40))
        self.assertEqual(result[0].rle_counts, (0, 1200))
        self.assertAlmostEqual(result[0].confidence, 0.91)

    def test_ignores_non_book_and_wrong_profile_rows(self):
        def rpc(request, timeout):
            return SimpleNamespace(
                task="scene_table_books_segmentation",
                scene_instances=[
                    _scene_row(semantic_class="cart_body", confidence=0.99),
                    _scene_row(profile="cart_loading_v1", confidence=0.98),
                    _scene_row(confidence=0.70),
                    _scene_row(confidence=0.90),
                ],
            )

        client = BookVisionClient(self.settings(), pb2_module=_FakePb2, rpc=rpc)
        result = client.detect(
            np.zeros((48, 64, 3), dtype=np.uint8),
            captured_at_ns=1,
            base_motion_epoch="base-1",
            head_motion_epoch="head-1",
        )

        self.assertEqual([item.confidence for item in result], [0.90, 0.70])


if __name__ == "__main__":
    unittest.main()
