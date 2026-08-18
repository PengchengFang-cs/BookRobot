import math
import unittest
from types import SimpleNamespace

from book_alignment import (
    STAGE1_PICK_REFERENCE_BASE_M,
    select_alignment_book,
)


class BookAlignmentTests(unittest.TestCase):
    def test_selects_book_nearest_recorded_pick_point(self):
        near = SimpleNamespace(suction_point=(1.01, -0.33, 0.76))
        far = SimpleNamespace(suction_point=(1.08, 0.31, 0.77))

        selected = select_alignment_book([far, near])

        self.assertIs(selected.book, near)
        self.assertEqual(selected.reference_m, STAGE1_PICK_REFERENCE_BASE_M)
        self.assertEqual(selected.observed_m, near.suction_point)
        for actual, observed, reference in zip(
            selected.residual_m,
            near.suction_point,
            STAGE1_PICK_REFERENCE_BASE_M,
        ):
            self.assertAlmostEqual(actual, observed - reference)

    def test_rejects_empty_book_list(self):
        with self.assertRaisesRegex(RuntimeError, "没有检测到可对位的书本"):
            select_alignment_book([])

    def test_rejects_non_finite_suction_point(self):
        book = SimpleNamespace(suction_point=(1.0, math.nan, 0.75))

        with self.assertRaisesRegex(ValueError, "书本吸取点必须是三个有限数值"):
            select_alignment_book([book])


if __name__ == "__main__":
    unittest.main()
