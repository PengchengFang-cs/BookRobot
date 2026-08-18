"""One-frame book perception shared by ROS and offline tests."""

from dataclasses import dataclass

from book_geometry import BookGeometryError, reconstruct_table_book
from config import (
    BOOK_LONG_INSET_M,
    BOOK_MAX_RESULTS,
    BOOK_MIN_EDGE_CLEARANCE_M,
    BOOK_MIN_CONFIDENCE,
    BOOK_RIGHT_INSET_M,
)


@dataclass(frozen=True)
class DetectedBook:
    observation: object
    geometry: object


def detect_book_frame(
    *,
    book_client,
    color_bgr,
    depth_m,
    intrinsics,
    camera_to_base,
    captured_at_ns,
    base_motion_epoch,
    head_motion_epoch,
    long_inset_m=BOOK_LONG_INSET_M,
    right_inset_m=BOOK_RIGHT_INSET_M,
    minimum_edge_clearance_m=BOOK_MIN_EDGE_CLEARANCE_M,
    minimum_confidence=BOOK_MIN_CONFIDENCE,
    maximum_results=BOOK_MAX_RESULTS,
    on_results=None,
):
    """Return up to five confidence-ranked masks with usable RGB-D geometry."""

    observations = sorted(
        book_client.detect(
            color_bgr,
            captured_at_ns=captured_at_ns,
            base_motion_epoch=base_motion_epoch,
            head_motion_epoch=head_motion_epoch,
        ),
        key=lambda item: item.confidence,
        reverse=True,
    )
    results = []
    for observation in observations:
        if observation.confidence < minimum_confidence:
            continue
        try:
            geometry = reconstruct_table_book(
                observation=observation,
                depth_m=depth_m,
                intrinsics=intrinsics,
                camera_to_base=camera_to_base,
                long_inset_m=long_inset_m,
                right_inset_m=right_inset_m,
                minimum_edge_clearance_m=minimum_edge_clearance_m,
            )
            results.append(DetectedBook(observation=observation, geometry=geometry))
            if len(results) == maximum_results:
                break
        except BookGeometryError:
            continue
    selected = tuple(results)
    if on_results is not None:
        on_results(selected)
    return selected
