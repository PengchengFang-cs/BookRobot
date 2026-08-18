"""Choose the book that best matches the recorded Stage 1 pick pose."""

from dataclasses import dataclass
import math

from config import APPROACH_DISTANCE_M


STAGE1_PICK_REFERENCE_BASE_M = (
    APPROACH_DISTANCE_M,
    -0.31509978336130007,
    0.7552452105314827,
)
STAGE1_MAX_Z_OFFSET_M = 0.03


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


def select_alignment_book(books):
    """Return the book whose suction point is nearest the replay reference."""

    if not books:
        raise RuntimeError("没有检测到可对位的书本")
    reference = STAGE1_PICK_REFERENCE_BASE_M
    candidates = []
    for book in books:
        point = _point3(book.suction_point)
        residual = tuple(point[i] - reference[i] for i in range(3))
        distance_squared = residual[0] * residual[0] + residual[1] * residual[1]
        candidates.append((distance_squared, book, point, residual))
    _, book, point, residual = min(candidates, key=lambda row: row[0])
    z_offset_m = residual[2]
    if abs(z_offset_m) > STAGE1_MAX_Z_OFFSET_M:
        raise RuntimeError(
            f"选中书本的 Z 偏移 {z_offset_m:.4f} m 超过 "
            f"{STAGE1_MAX_Z_OFFSET_M:.3f} m"
        )
    return BookAlignmentTarget(book, reference, point, residual, z_offset_m)
