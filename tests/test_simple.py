import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry import camera_point_to_base, fruit_from_text, quaternion_from_yaw


class SimpleTests(unittest.TestCase):
    def test_chinese_fruit_word(self):
        self.assertEqual(fruit_from_text("请帮我拿一根香蕉"), "banana")
        self.assertEqual(fruit_from_text("我想要苹果"), "apple")

    def test_yaw_quaternion(self):
        q = quaternion_from_yaw(math.pi)
        self.assertAlmostEqual(q[2], 1.0)
        self.assertAlmostEqual(q[3], 0.0, places=7)

    def test_camera_transform_is_finite(self):
        point = camera_point_to_base((0.0, 0.0, 1.0), 0.05, 0.0, 0.0)
        self.assertEqual(len(point), 3)
        self.assertTrue(all(math.isfinite(value) for value in point))


if __name__ == "__main__":
    unittest.main()
