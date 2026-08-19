"""Choose the book that best matches the recorded Stage 1 pick pose."""

from dataclasses import dataclass
import math

from config import APPROACH_DISTANCE_M


COARSE_APPROACH_REFERENCE_X_M = APPROACH_DISTANCE_M
COARSE_APPROACH_REFERENCE_BASE_M = (
    COARSE_APPROACH_REFERENCE_X_M,
    -0.31509978336130007,
    0.7552452105314827,
)
# Temporary compatibility name for the one-stage mission while it is migrated.
STAGE1_PICK_REFERENCE_BASE_M = COARSE_APPROACH_REFERENCE_BASE_M
STAGE1_MAX_Z_OFFSET_M = 0.03
REASSOCIATE_MAX_DISTANCE_M = 0.12
BOOK_EXTENT_TOLERANCE_M = 0.05
BOOK_LONG_AXIS_TOLERANCE_RAD = math.radians(3.0)


@dataclass(frozen=True)
class BookAlignmentTarget:
    book: object
    reference_m: tuple[float, float, float]
    observed_m: tuple[float, float, float]
    residual_m: tuple[float, float, float]
    z_offset_m: float


def _point3(value):
    try:
        point = tuple(float(component) for component in value)
    except (TypeError, ValueError):
        raise ValueError("书本吸取点必须是三个有限数值") from None
    if len(point) != 3 or not all(math.isfinite(component) for component in point):
        raise ValueError("书本吸取点必须是三个有限数值")
    return point


def _book_matches_reference(book, replay_reference):
    geometry = book.geometry
    axis = _point3(geometry.long_axis)
    reference_axis = _point3(replay_reference.recorded_book_long_axis_base)
    axis_xy = math.hypot(axis[0], axis[1])
    reference_xy = math.hypot(reference_axis[0], reference_axis[1])
    if axis_xy <= 1e-9 or reference_xy <= 1e-9:
        return False
    dot = (
        axis[0] * reference_axis[0] + axis[1] * reference_axis[1]
    ) / (axis_xy * reference_xy)
    yaw_error = math.acos(max(-1.0, min(1.0, dot)))
    return (
        yaw_error <= BOOK_LONG_AXIS_TOLERANCE_RAD
        and abs(
            float(geometry.long_extent_m)
            - float(replay_reference.recorded_book_long_extent_m)
        )
        <= BOOK_EXTENT_TOLERANCE_M
        and abs(
            float(geometry.short_extent_m)
            - float(replay_reference.recorded_book_short_extent_m)
        )
        <= BOOK_EXTENT_TOLERANCE_M
    )


def select_coarse_book(books):
    """Choose a book for the approximate 0.48 m working-range approach."""

    if not books:
        raise RuntimeError("没有检测到可对位的书本")
    reference = COARSE_APPROACH_REFERENCE_BASE_M
    candidates = []
    for book in books:
        point = _point3(book.suction_point)
        residual = tuple(point[i] - reference[i] for i in range(3))
        distance_squared = residual[0] * residual[0] + residual[1] * residual[1]
        candidates.append((distance_squared, book, point, residual))
    _, book, point, residual = min(candidates, key=lambda row: row[0])
    return BookAlignmentTarget(book, reference, point, residual, residual[2])


def build_replay_alignment_target(*, book, replay_reference):
    """Build the precise target from one asset's recorded contact geometry."""

    if not _book_matches_reference(book, replay_reference):
        raise RuntimeError("书本方向或尺寸与 DataReplay 第0帧不一致")
    observed = _point3(book.suction_point)
    reference = _point3(replay_reference.recorded_book_suction_point_base_m)
    residual = tuple(observed[index] - reference[index] for index in range(3))
    z_offset_m = residual[2]
    if abs(z_offset_m) > STAGE1_MAX_Z_OFFSET_M:
        raise RuntimeError(
            f"选中书本的 Z 偏移 {z_offset_m:.4f} m 超过 "
            f"{STAGE1_MAX_Z_OFFSET_M:.3f} m"
        )
    return BookAlignmentTarget(book, reference, observed, residual, z_offset_m)


def select_replay_book(books, *, replay_reference):
    """Choose the current book whose suction point is nearest the replay pose."""

    if not books:
        raise RuntimeError("没有检测到可精确对位的书本")
    reference = _point3(replay_reference.recorded_book_suction_point_base_m)
    candidates = []
    for book in books:
        if not _book_matches_reference(book, replay_reference):
            continue
        point = _point3(book.suction_point)
        dx = point[0] - reference[0]
        dy = point[1] - reference[1]
        candidates.append((dx * dx + dy * dy, book))
    if not candidates:
        raise RuntimeError("没有方向和尺寸匹配的 DataReplay 目标书")
    return min(candidates, key=lambda row: row[0])[1]


def predict_book_after_base_motion(
    point,
    *,
    odom_dx_m,
    odom_dy_m,
    imu_dyaw_rad,
):
    """Express a pre-motion base point in the post-motion base frame."""

    x, y, z = _point3(point)
    translated_x = x - float(odom_dx_m)
    translated_y = y - float(odom_dy_m)
    angle = -float(imu_dyaw_rad)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        cosine * translated_x - sine * translated_y,
        sine * translated_x + cosine * translated_y,
        z,
    )


def reassociate_book(books, *, predicted_point_m, replay_reference):
    """Find the current observation nearest the odom-predicted same book."""

    if not books:
        raise RuntimeError("没有检测到可重关联的书本")
    predicted = _point3(predicted_point_m)
    candidates = []
    for book in books:
        if not _book_matches_reference(book, replay_reference):
            continue
        point = _point3(book.suction_point)
        dx = point[0] - predicted[0]
        dy = point[1] - predicted[1]
        distance_squared = dx * dx + dy * dy
        if distance_squared <= REASSOCIATE_MAX_DISTANCE_M**2:
            candidates.append((distance_squared, book))
    if not candidates:
        raise RuntimeError("目标书重关联失败：预测距离、方向或尺寸不匹配")
    return min(candidates, key=lambda row: row[0])[1]


def select_alignment_book(books):
    """Compatibility wrapper for the former one-stage mission."""

    return select_coarse_book(books)
