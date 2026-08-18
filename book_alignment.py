"""Choose the book that best matches the recorded Stage 1 pick pose."""

from dataclasses import dataclass
import math


STAGE1_PICK_REFERENCE_BASE_M = (
    0.9116001170551475,
    -0.31509978336130007,
    0.7552452105314827,
)


@dataclass(frozen=True)
class BookAlignmentTarget:
    book: object
    reference_m: tuple[float, float, float]
    observed_m: tuple[float, float, float]
    residual_m: tuple[float, float, float]


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
        distance_squared = sum(value * value for value in residual)
        candidates.append((distance_squared, book, point, residual))
    _, book, point, residual = min(candidates, key=lambda row: row[0])
    return BookAlignmentTarget(book, reference, point, residual)
