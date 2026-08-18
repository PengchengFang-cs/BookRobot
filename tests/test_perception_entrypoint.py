import json
import unittest
from types import SimpleNamespace

from test_book_perception import format_result


class PerceptionEntrypointTests(unittest.TestCase):
    def book(self, confidence, bbox, point):
        return SimpleNamespace(
            observation=SimpleNamespace(confidence=confidence, bbox=bbox),
            suction_point=point,
        )

    def test_formats_multiple_base_link_books(self):
        payload = json.loads(
            format_result(
                [
                    self.book(0.9, (10, 20, 30, 40), (0.41, -0.07, 0.82)),
                    self.book(0.7, (50, 60, 70, 80), (0.52, 0.08, 0.79)),
                ]
            )
        )

        self.assertEqual(payload["frame_id"], "base_link")
        self.assertEqual(payload["book_count"], 2)
        self.assertEqual(payload["books"][0]["confidence"], 0.9)
        self.assertEqual(payload["books"][0]["bbox_xywh"], [10, 20, 30, 40])
        self.assertEqual(payload["books"][0]["suction_point_m"], [0.41, -0.07, 0.82])
        self.assertEqual(payload["books"][1]["suction_point_m"], [0.52, 0.08, 0.79])
        self.assertTrue(payload["ok"])

    def test_formats_missing_book(self):
        payload = json.loads(format_result([]))

        self.assertFalse(payload["ok"])
        self.assertEqual(payload["book_count"], 0)
        self.assertEqual(payload["books"], [])


if __name__ == "__main__":
    unittest.main()
