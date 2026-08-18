import math
import unittest
from types import SimpleNamespace

from book_alignment import (
    STAGE1_PICK_REFERENCE_BASE_M,
    select_alignment_book,
)
from config import APPROACH_DISTANCE_M


class BookAlignmentTests(unittest.TestCase):
    def test_stage1_reference_uses_the_existing_fruittest_standoff(self):
        self.assertEqual(STAGE1_PICK_REFERENCE_BASE_M[0], APPROACH_DISTANCE_M)
        self.assertAlmostEqual(STAGE1_PICK_REFERENCE_BASE_M[1], -0.31509978336130007)

    def test_selects_book_nearest_pick_alignment_point(self):
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
        self.assertAlmostEqual(
            selected.z_offset_m,
            near.suction_point[2] - STAGE1_PICK_REFERENCE_BASE_M[2],
        )

    def test_selection_uses_xy_distance_and_ignores_z_distance(self):
        reference = STAGE1_PICK_REFERENCE_BASE_M
        xy_near = SimpleNamespace(
            suction_point=(reference[0] + 0.001, reference[1], reference[2] + 0.02)
        )
        xyz_near = SimpleNamespace(
            suction_point=(reference[0] + 0.015, reference[1], reference[2])
        )

        selected = select_alignment_book([xyz_near, xy_near])

        self.assertIs(selected.book, xy_near)
        self.assertAlmostEqual(selected.z_offset_m, 0.02)

    def test_preserves_zero_and_negative_z_offsets(self):
        reference = STAGE1_PICK_REFERENCE_BASE_M
        zero = SimpleNamespace(suction_point=reference)
        negative = SimpleNamespace(
            suction_point=(reference[0] + 0.01, reference[1], reference[2] - 0.01)
        )

        self.assertEqual(select_alignment_book([zero]).z_offset_m, 0.0)
        self.assertAlmostEqual(select_alignment_book([negative]).z_offset_m, -0.01)

    def test_rejects_selected_z_offset_outside_thirty_millimetres(self):
        reference = STAGE1_PICK_REFERENCE_BASE_M
        book = SimpleNamespace(
            suction_point=(reference[0], reference[1], reference[2] + 0.031)
        )

        with self.assertRaisesRegex(RuntimeError, "Z 偏移.*0.030"):
            select_alignment_book([book])

    def test_rejects_empty_book_list(self):
        with self.assertRaisesRegex(RuntimeError, "没有检测到可对位的书本"):
            select_alignment_book([])

    def test_rejects_non_finite_suction_point(self):
        book = SimpleNamespace(suction_point=(1.0, math.nan, 0.75))

        with self.assertRaisesRegex(ValueError, "书本吸取点必须是三个有限数值"):
            select_alignment_book([book])


if __name__ == "__main__":
    unittest.main()
