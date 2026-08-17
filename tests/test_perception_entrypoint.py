import json
import unittest

from test_book_perception import format_result


class PerceptionEntrypointTests(unittest.TestCase):
    def test_formats_base_link_suction_point(self):
        payload = json.loads(format_result((0.41, -0.07, 0.82)))

        self.assertEqual(payload["frame_id"], "base_link")
        self.assertEqual(payload["suction_point_m"], [0.41, -0.07, 0.82])
        self.assertTrue(payload["ok"])

    def test_formats_missing_book(self):
        payload = json.loads(format_result(None))

        self.assertFalse(payload["ok"])
        self.assertIsNone(payload["suction_point_m"])


if __name__ == "__main__":
    unittest.main()
