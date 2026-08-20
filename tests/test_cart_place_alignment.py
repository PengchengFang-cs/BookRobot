import math
import unittest
from types import SimpleNamespace

from cart_place_alignment import (
    build_cart_place_alignment_target,
    median_cart_place_alignment_target,
)
from replay_place_reference import ReplayPlaceReference


def _reference():
    return ReplayPlaceReference(
        asset_id="S1_CART_PLACE_BOOK",
        reference_frame_index=0,
        placed_frame_index=387,
        recorded_torso_m=0.2,
        recorded_head_rad=(0.0, 0.25),
        platform_front_edge_base_m=(0.77, -0.25, 0.54),
        platform_left_edge_base_m=(0.95, 0.15, 0.54),
        platform_right_edge_base_m=(0.95, -0.60, 0.54),
        platform_forward_axis_base=(1.0, 0.0, 0.0),
        platform_lateral_axis_base=(0.0, 1.0, 0.0),
        platform_width_m=0.75,
        platform_depth_m=0.37,
        recorded_book_offset_from_left_m=0.17,
    )


def _platform(*, front=(0.77, -0.25, 0.54), yaw=0.0):
    forward = (math.cos(yaw), math.sin(yaw), 0.0)
    lateral = (-math.sin(yaw), math.cos(yaw), 0.0)
    slots = tuple(
        (front[0] + lateral[0] * (0.40 - offset),
         front[1] + lateral[1] * (0.40 - offset), 0.54)
        for offset in (0.17, 0.24, 0.31, 0.38, 0.45)
    )
    return SimpleNamespace(
        front_edge=front,
        forward_axis=forward,
        lateral_axis_right_to_left=lateral,
        slot_centers=slots,
    )


class CartPlaceAlignmentTests(unittest.TestCase):
    def test_slot_one_matches_recorded_left_offset(self):
        target = build_cart_place_alignment_target(
            platform=_platform(), replay_reference=_reference(), slot_index=1
        )

        self.assertAlmostEqual(target.residual_m[0], 0.0)
        self.assertAlmostEqual(target.residual_m[1], 0.0)
        self.assertAlmostEqual(target.yaw_error_rad, 0.0)

    def test_later_slots_shift_robot_left_by_seven_centimetres_each(self):
        target = build_cart_place_alignment_target(
            platform=_platform(), replay_reference=_reference(), slot_index=3
        )

        self.assertAlmostEqual(target.residual_m[0], 0.0)
        self.assertAlmostEqual(target.residual_m[1], -0.14)

    def test_front_edge_and_yaw_are_independent_components(self):
        target = build_cart_place_alignment_target(
            platform=_platform(front=(0.82, -0.23, 0.54), yaw=0.1),
            replay_reference=_reference(),
            slot_index=1,
        )

        self.assertGreater(target.residual_m[0], 0.02)
        self.assertAlmostEqual(target.yaw_error_rad, 0.1)

    def test_medians_three_measurements(self):
        targets = [
            build_cart_place_alignment_target(
                platform=_platform(front=(0.77 + dx, -0.25 + dy, 0.54)),
                replay_reference=_reference(),
                slot_index=1,
            )
            for dx, dy in ((0.02, 0.01), (0.01, 0.00), (0.40, -0.30))
        ]

        stable = median_cart_place_alignment_target(targets)

        self.assertAlmostEqual(stable.residual_m[0], 0.02)
        self.assertAlmostEqual(stable.residual_m[1], 0.00)


if __name__ == "__main__":
    unittest.main()
