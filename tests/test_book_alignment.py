import math
import unittest
from types import SimpleNamespace

import numpy as np

from book_alignment import (
    COARSE_APPROACH_REFERENCE_X_M,
    build_replay_alignment_target,
    predict_book_after_base_motion,
    reassociate_book,
    select_coarse_book,
)
from book_geometry import BookGeometry
from config import APPROACH_DISTANCE_M
from replay_pick_reference import ReplayPickReference


def _geometry(*, suction_point, long_axis=(1.0, 0.0, 0.0)):
    short_axis = (-long_axis[1], long_axis[0], 0.0)
    return BookGeometry(
        suction_point=suction_point,
        long_axis=long_axis,
        short_axis_right_to_left=short_axis,
        long_extent_m=0.30,
        short_extent_m=0.20,
        confidence=0.9,
        long_inset_m=0.13,
        right_inset_m=0.10,
    )


def _book(point, *, long_axis=(1.0, 0.0, 0.0), long_extent=0.30, short_extent=0.20):
    geometry = BookGeometry(
        suction_point=point,
        long_axis=long_axis,
        short_axis_right_to_left=(-long_axis[1], long_axis[0], 0.0),
        long_extent_m=long_extent,
        short_extent_m=short_extent,
        confidence=0.9,
        long_inset_m=0.13,
        right_inset_m=0.10,
    )
    return SimpleNamespace(suction_point=point, geometry=geometry)


def _reference(**changes):
    values = dict(
        asset_id="S1_TABLE_PICK_BOOK",
        reference_frame_index=0,
        recorded_book_suction_point_base_m=(0.715, -0.391, 0.713),
        recorded_book_long_axis_base=(1.0, 0.0, 0.0),
        recorded_book_long_extent_m=0.30,
        recorded_book_short_extent_m=0.20,
        hdf5_sha256="a" * 64,
    )
    values.update(changes)
    return ReplayPickReference(**values)


class BookAlignmentTests(unittest.TestCase):
    def test_coarse_reference_uses_fruittest_working_distance_only(self):
        self.assertEqual(COARSE_APPROACH_REFERENCE_X_M, APPROACH_DISTANCE_M)
        near = _book((0.60, -0.32, 0.76))
        far = _book((0.80, 0.20, 0.76))

        selected = select_coarse_book([far, near])

        self.assertIs(selected.book, near)
        self.assertEqual(selected.reference_m[0], APPROACH_DISTANCE_M)

    def test_precise_target_uses_replay_contact_not_point_four_eight(self):
        reference = _reference()
        book = _book((0.80, -0.30, 0.72))

        target = build_replay_alignment_target(
            book=book,
            replay_reference=reference,
        )

        expected_observed = book.suction_point
        self.assertEqual(
            target.reference_m,
            reference.recorded_book_suction_point_base_m,
        )
        self.assertNotEqual(target.reference_m[0], APPROACH_DISTANCE_M)
        self.assertEqual(target.observed_m, expected_observed)
        np.testing.assert_allclose(
            target.residual_m,
            np.asarray(expected_observed)
            - np.asarray(reference.recorded_book_suction_point_base_m),
        )
        self.assertAlmostEqual(
            target.z_offset_m,
            expected_observed[2]
            - reference.recorded_book_suction_point_base_m[2],
        )

    def test_predicts_same_book_in_new_base_with_inverse_se2_motion(self):
        predicted = predict_book_after_base_motion(
            (1.0, 0.2, 0.75),
            odom_dx_m=0.20,
            odom_dy_m=-0.10,
            imu_dyaw_rad=math.pi / 2.0,
        )

        np.testing.assert_allclose(predicted, (0.30, -0.80, 0.75), atol=1e-12)

    def test_reassociates_nearest_predicted_book_not_nearest_replay_reference(self):
        reference = _reference(
            recorded_book_suction_point_base_m=(0.70, -0.39, 0.72),
        )
        same_book = _book((0.91, -0.10, 0.72))
        distractor = _book((0.70, -0.39, 0.72))

        selected = reassociate_book(
            [distractor, same_book],
            predicted_point_m=(0.90, -0.11, 0.72),
            replay_reference=reference,
        )

        self.assertIs(selected, same_book)

    def test_rejects_precise_z_offset_outside_thirty_millimetres(self):
        reference = _reference(
            recorded_book_suction_point_base_m=(0.715, -0.391, 0.70)
        )
        book = _book((0.80, -0.30, 0.731))

        with self.assertRaisesRegex(RuntimeError, "Z 偏移.*0.030"):
            build_replay_alignment_target(
                book=book,
                replay_reference=reference,
            )

    def test_rejects_empty_coarse_or_reassociation_candidates(self):
        with self.assertRaisesRegex(RuntimeError, "没有检测到可对位的书本"):
            select_coarse_book([])
        with self.assertRaisesRegex(RuntimeError, "没有检测到可重关联的书本"):
            reassociate_book(
                [],
                predicted_point_m=(0.7, -0.3, 0.7),
                replay_reference=_reference(),
            )

    def test_reassociation_rejects_candidate_far_from_prediction(self):
        with self.assertRaisesRegex(RuntimeError, "目标书重关联失败"):
            reassociate_book(
                [_book((1.20, 0.30, 0.72))],
                predicted_point_m=(0.70, -0.30, 0.72),
                replay_reference=_reference(),
            )

    def test_reassociation_rejects_wrong_size_or_yaw(self):
        angle = math.radians(10.0)
        wrong_yaw = _book(
            (0.71, -0.39, 0.72),
            long_axis=(math.cos(angle), math.sin(angle), 0.0),
        )
        wrong_size = _book(
            (0.71, -0.39, 0.72),
            long_extent=0.40,
        )

        with self.assertRaisesRegex(RuntimeError, "目标书重关联失败"):
            reassociate_book(
                [wrong_yaw, wrong_size],
                predicted_point_m=(0.71, -0.39, 0.72),
                replay_reference=_reference(),
            )

    def test_reassociation_treats_a_pca_long_axis_as_undirected(self):
        same_book_flipped_axis = _book(
            (0.71, -0.39, 0.72), long_axis=(-1.0, 0.0, 0.0)
        )

        selected = reassociate_book(
            [same_book_flipped_axis],
            predicted_point_m=(0.71, -0.39, 0.72),
            replay_reference=_reference(),
        )

        self.assertIs(selected, same_book_flipped_axis)


if __name__ == "__main__":
    unittest.main()
