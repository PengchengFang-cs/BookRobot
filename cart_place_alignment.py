"""Place-2.4 platform geometry alignment, independent of ROS."""

from dataclasses import dataclass, replace
import math
from statistics import median

import numpy as np


@dataclass(frozen=True)
class CartPlaceAlignmentTarget:
    slot_index: int
    reference_anchor_m: tuple[float, float, float]
    observed_anchor_m: tuple[float, float, float]
    residual_m: tuple[float, float, float]
    yaw_error_rad: float


def _yaw(axis):
    return math.atan2(float(axis[1]), float(axis[0]))


def _normalize_yaw(value):
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


def _front_slot_anchor(front_edge, lateral_axis, slot_center):
    front = np.asarray(front_edge, dtype=float)
    lateral = np.asarray(lateral_axis, dtype=float)
    slot = np.asarray(slot_center, dtype=float)
    return front + lateral * float(np.dot(slot - front, lateral))


def build_cart_place_alignment_target(
    *,
    platform,
    replay_reference,
    slot_index,
):
    """Match live shelf front distance and selected slot to replay frame zero."""

    slot_index = int(slot_index)
    if slot_index < 1 or slot_index > len(platform.slot_centers):
        raise ValueError("cart_place_slot_invalid")
    reference_left = np.asarray(
        replay_reference.platform_left_edge_base_m, dtype=float
    )
    reference_lateral = np.asarray(
        replay_reference.platform_lateral_axis_base, dtype=float
    )
    reference_slot = (
        reference_left
        - reference_lateral
        * float(replay_reference.recorded_book_offset_from_left_m)
    )
    reference_anchor = _front_slot_anchor(
        replay_reference.platform_front_edge_base_m,
        reference_lateral,
        reference_slot,
    )
    observed_anchor = _front_slot_anchor(
        platform.front_edge,
        platform.lateral_axis_right_to_left,
        platform.slot_centers[slot_index - 1],
    )
    residual = observed_anchor - reference_anchor
    yaw_error = _normalize_yaw(
        _yaw(platform.forward_axis)
        - _yaw(replay_reference.platform_forward_axis_base)
    )
    return CartPlaceAlignmentTarget(
        slot_index=slot_index,
        reference_anchor_m=tuple(float(value) for value in reference_anchor),
        observed_anchor_m=tuple(float(value) for value in observed_anchor),
        residual_m=tuple(float(value) for value in residual),
        yaw_error_rad=float(yaw_error),
    )


def median_cart_place_alignment_target(targets):
    targets = tuple(targets)
    if not targets:
        raise ValueError("cart_place_targets_empty")
    slot_index = targets[0].slot_index
    if any(target.slot_index != slot_index for target in targets):
        raise ValueError("cart_place_slot_mismatch")
    residual = tuple(
        float(median(target.residual_m[axis] for target in targets))
        for axis in range(3)
    )
    yaw_error = float(median(target.yaw_error_rad for target in targets))
    representative = min(
        targets,
        key=lambda target: sum(
            (target.residual_m[axis] - residual[axis]) ** 2
            for axis in range(3)
        ) + (target.yaw_error_rad - yaw_error) ** 2,
    )
    reference = representative.reference_anchor_m
    observed = tuple(reference[axis] + residual[axis] for axis in range(3))
    return replace(
        representative,
        observed_anchor_m=observed,
        residual_m=residual,
        yaw_error_rad=yaw_error,
    )
