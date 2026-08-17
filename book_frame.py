"""One-frame book perception shared by ROS and offline tests."""

from book_geometry import BookGeometryError, reconstruct_table_book
from config import (
    BOOK_LONG_INSET_M,
    BOOK_MIN_EDGE_CLEARANCE_M,
    BOOK_RIGHT_INSET_M,
)


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
    on_result=None,
):
    """Return the highest-confidence mask with usable local RGB-D geometry."""

    observations = book_client.detect(
        color_bgr,
        captured_at_ns=captured_at_ns,
        base_motion_epoch=base_motion_epoch,
        head_motion_epoch=head_motion_epoch,
    )
    for observation in observations:
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
            if on_result is not None:
                on_result(observation, geometry)
            return geometry
        except BookGeometryError:
            continue
    return None
