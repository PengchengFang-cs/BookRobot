"""Stage 2 followed immediately by Stage 3.

This file follows the Stage 1 mission.py style: the main task is written as a
straight sequence of perception, navigation, alignment and DataReplay calls.
The interfaces that are not present in the current repository are kept as
explicit gaps instead of being replaced with a new framework.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
import sys
import time
import uuid

from place_ocr import load_stage1_cart_book_labels

REPLAY_ROOT = Path(
    "/home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/"
    "20260819_libraryrobot_datareplay"
)

BOOK_PICK_X_TOLERANCE_M = 0.020
BOOK_PICK_Y_TOLERANCE_M = 0.010
SHELF_PLACE_X_TOLERANCE_M = 0.030
SHELF_PLACE_Y_TOLERANCE_M = 0.020
SHELF_PLACE_YAW_TOLERANCE_RAD = math.radians(5.0)
MAXIMUM_ALIGNMENT_CORRECTIONS = 3
DEFAULT_BOOK_WIDTH_M = 0.05
VERTICAL_SPINE_GRASP_HEIGHT_M = 0.13
SHELF_SCAN_HEAD_ANGLES_DEG = (-45, -30, -15, 0, 15, 30, 45)

BOOK_LABEL_RE = re.compile(r"^A[0-9]{2}-[0-9]{4}$")
BOOK_LABEL_TWO_LINE_RE = re.compile(
    r"^(A[0-9]{2})[ \t]*\r?\n[ \t]*([0-9]{4})$"
)


@dataclass(frozen=True)
class ReplayAsset:
    name: str
    path: Path
    frame_count: int
    operation: str
    level: int | None = None


@dataclass(frozen=True)
class ShelfObservationPose:
    torso_m: float
    head_pitch_rad: float


@dataclass(frozen=True)
class BookLabel:
    text: str
    level: int
    number: int


@dataclass(frozen=True)
class TrackedBook:
    tracking_key: object
    suction_point_m: tuple[float, float, float]
    label: BookLabel | None = None
    actual_level: int | None = None


@dataclass(frozen=True)
class CartBookTrackingKey:
    order_from_right: int
    label_text: str


@dataclass(frozen=True)
class ShelfBookObservation:
    tracking_key: object
    actual_level: int
    label: BookLabel
    center_m: tuple[float, float, float]
    confidence: float
    lower_number_edge_m: tuple[float, float, float] | None = None
    higher_number_edge_m: tuple[float, float, float] | None = None
    spine_direction_m: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class ShelfTarget:
    level: int
    number: int
    center_m: tuple[float, float, float]
    left_m: tuple[float, float, float]
    right_m: tuple[float, float, float]
    yaw_rad: float


@dataclass(frozen=True)
class MisplacedBook:
    book: ShelfBookObservation
    reason: str


@dataclass(frozen=True)
class ShelfAlignmentCommand:
    reference_anchor_m: tuple[float, float, float]
    observed_anchor_m: tuple[float, float, float]
    residual_m: tuple[float, float, float]
    yaw_error_rad: float


ASSETS = {
    "DR5.1": ReplayAsset(
        "DR5.1",
        REPLAY_ROOT / "pi05_wanda_dr5.1_20260821_223711.h5",
        557,
        "place",
        5,
    ),
    "DR6.4": ReplayAsset(
        "DR6.4",
        REPLAY_ROOT / "pi05_wanda_dr6.4_20260821_235247.h5",
        527,
        "place",
        4,
    ),
    "DR7.1": ReplayAsset(
        "DR7.1",
        REPLAY_ROOT / "pi05_wanda_dr7.1_20260821_234944.h5",
        505,
        "place",
        3,
    ),
    "DR8.1": ReplayAsset(
        "DR8.1",
        REPLAY_ROOT / "pi05_wanda_dr8.1_20260821_235504.h5",
        427,
        "pick",
        4,
    ),
    "DR9.2": ReplayAsset(
        "DR9.2",
        REPLAY_ROOT / "pi05_wanda_dr9.2_20260822_000416.h5",
        366,
        "pick",
        3,
    ),
    "DR10.1": ReplayAsset(
        "DR10.1",
        REPLAY_ROOT / "pi05_wanda_dr10.1_20260822_000941.h5",
        387,
        "pick",
        5,
    ),
    "DR11.2": ReplayAsset(
        "DR11.2",
        REPLAY_ROOT / "pi05_wanda_dr11.2_20260822_002132.h5",
        373,
        "pick",
        None,
    ),
}

# Pending before formal asset-contract verification: record an independent
# SHA-256, start/end state and start anchor for every Stage 2/3 HDF5.

STAGE2_PLACE_ASSET_BY_LEVEL = {3: "DR7.1", 4: "DR6.4", 5: "DR5.1"}
STAGE3_PICK_ASSET_BY_LEVEL = {3: "DR9.2", 4: "DR8.1", 5: "DR10.1"}

SHELF_OBSERVATION_POSE_BY_LEVEL = {
    5: ShelfObservationPose(torso_m=0.28, head_pitch_rad=-0.174),
    4: ShelfObservationPose(torso_m=0.00, head_pitch_rad=-0.174),
    3: ShelfObservationPose(torso_m=0.00, head_pitch_rad=+0.108),
}

# Fill these seven asset-specific frames after reviewing the matching videos.
# A value from DR1.2 or DR2.4 must not be copied into these entries.
D01_EVENT_FRAME_BY_ASSET = {
    "DR5.1": None,
    "DR6.4": 344,
    "DR7.1": None,
    "DR8.1": None,
    "DR9.2": None,
    "DR10.1": None,
    "DR11.2": 0,
}

# These points must be calibrated from each asset's frame-zero image/depth and
# its exact semantic point. Visual Z is logged but never offsets replay torso.
PICK_REFERENCE_POINT_M_BY_ASSET = {
    "DR8.1": None,
    "DR9.2": None,
    "DR10.1": None,
    "DR11.2": (0.792389, -0.343970, 1.036510),
}
PLACE_REFERENCE_POINT_M_BY_ASSET = {
    "DR5.1": None,
    "DR6.4": None,
    "DR7.1": None,
}
PLACE_REFERENCE_YAW_RAD_BY_ASSET = {
    "DR5.1": None,
    "DR6.4": None,
    "DR7.1": None,
}

# The cart-book OCR and vertical-spine path is ready for the one-book entry.
# Shelf perception and Stage 2/3 coarse routes remain explicit gaps.
OCR_TRANSPORT_READY = True
STAGE23_PERCEPTION_READY = False
STAGE23_COARSE_NAVIGATION_READY = False


def pending_stage23_configuration():
    pending = []
    for asset_name, frame_index in D01_EVENT_FRAME_BY_ASSET.items():
        if frame_index is None:
            pending.append(f"{asset_name} D01事件帧")
        elif (
            type(frame_index) is not int
            or frame_index < 0
            or frame_index >= ASSETS[asset_name].frame_count
        ):
            pending.append(
                f"{asset_name} D01事件帧无效；必须是0到"
                f"{ASSETS[asset_name].frame_count - 1}之间的整数"
            )
    for asset_name, point in PICK_REFERENCE_POINT_M_BY_ASSET.items():
        if point is None:
            pending.append(f"{asset_name} Pick第0帧视觉参考点")
        else:
            try:
                _point3(point)
            except (TypeError, ValueError):
                pending.append(f"{asset_name} Pick第0帧视觉参考点无效")
    for asset_name, point in PLACE_REFERENCE_POINT_M_BY_ASSET.items():
        if point is None:
            pending.append(f"{asset_name} Place第0帧视觉参考点")
        else:
            try:
                _point3(point)
            except (TypeError, ValueError):
                pending.append(f"{asset_name} Place第0帧视觉参考点无效")
    for asset_name, yaw in PLACE_REFERENCE_YAW_RAD_BY_ASSET.items():
        if yaw is None:
            pending.append(f"{asset_name} Place第0帧yaw参考")
        else:
            try:
                yaw_value = float(yaw)
            except (TypeError, ValueError):
                yaw_value = math.nan
            if not math.isfinite(yaw_value):
                pending.append(f"{asset_name} Place第0帧yaw参考无效")
    if not OCR_TRANSPORT_READY:
        pending.append("Wanda到RTX 5090的OCR传输接口")
    if not STAGE23_PERCEPTION_READY:
        pending.append("竖直书脊、书架层级和书位框感知接口")
    if not STAGE23_COARSE_NAVIGATION_READY:
        pending.append("Stage 2/3推车与书架之间的粗导航接口")
    return tuple(pending)


def require_stage23_configuration():
    pending = pending_stage23_configuration()
    if pending:
        raise RuntimeError(
            "Stage 2/3尚有未补接口；未初始化ROS，也不会产生机器人运动：\n- "
            + "\n- ".join(pending)
        )


def require_stage2_pick_one_configuration():
    frame_index = D01_EVENT_FRAME_BY_ASSET["DR11.2"]
    if (
        type(frame_index) is not int
        or frame_index < 0
        or frame_index >= ASSETS["DR11.2"].frame_count
    ):
        raise RuntimeError("DR11.2吸盘开启帧无效")
    try:
        _point3(PICK_REFERENCE_POINT_M_BY_ASSET["DR11.2"])
    except (TypeError, ValueError) as error:
        raise RuntimeError("DR11.2第0帧视觉参考点无效") from error
    if not OCR_TRANSPORT_READY:
        raise RuntimeError("Wanda到RTX 5090的OCR传输接口未就绪")


def parse_book_label(text):
    raw = str(text).strip().upper()
    two_line = BOOK_LABEL_TWO_LINE_RE.fullmatch(raw)
    normalized = (
        f"{two_line.group(1)}-{two_line.group(2)}"
        if two_line is not None
        else raw
    )
    if BOOK_LABEL_RE.fullmatch(normalized) is None:
        raise RuntimeError("OCR无法识别")
    level = int(normalized[1:3])
    if level not in (3, 4, 5):
        raise RuntimeError("识别结果不是第三、第四或第五层")
    return BookLabel(
        text=normalized,
        level=level,
        number=int(normalized[4:8]),
    )


def validate_book_label(value):
    if isinstance(value, BookLabel):
        parsed = parse_book_label(value.text)
        if parsed.level != value.level or parsed.number != value.number:
            raise RuntimeError("OCR标签文字与解析字段不一致")
        return parsed
    return parse_book_label(value)


def ocr_box_center(polygon_px):
    """Use the left/right extent center requested for one OCR detection box."""

    points = tuple(tuple(float(value) for value in point) for point in polygon_px)
    if len(points) != 4 or any(len(point) != 2 for point in points):
        raise ValueError("OCR检测框必须包含四个二维点")
    if not all(math.isfinite(value) for point in points for value in point):
        raise ValueError("OCR检测框坐标必须是有限数值")
    left = min(point[0] for point in points)
    right = max(point[0] for point in points)
    top = min(point[1] for point in points)
    bottom = max(point[1] for point in points)
    return ((left + right) / 2.0, (top + bottom) / 2.0)


def _point3(value):
    point = tuple(float(component) for component in value)
    if len(point) != 3 or not all(math.isfinite(component) for component in point):
        raise ValueError("三维点必须包含三个有限数值")
    return point


def _unit_xy(value):
    x, y = float(value[0]), float(value[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("书架编号增长方向无效")
    length = math.hypot(x, y)
    if length <= 1e-9:
        raise ValueError("书架编号增长方向无效")
    return (x / length, y / length, 0.0)


def infer_shelf_target(
    *,
    level,
    number,
    books,
    shelf_number_axis,
    shelf_yaw_rad,
):
    """Infer one empty numbered slot from OCR-associated shelf books."""

    level = int(level)
    number = int(number)
    if level not in (3, 4, 5):
        raise ValueError("目标层必须是第三、第四或第五层")
    if number < 0 or number > 9999:
        raise ValueError("书本编号必须是四位非负整数")
    axis = _unit_xy(shelf_number_axis)
    yaw = float(shelf_yaw_rad)
    if not math.isfinite(yaw):
        raise ValueError("书架yaw必须是有限数值")

    books = tuple(books)
    for book in books:
        validate_book_label(book.label)

    actual_level_books = tuple(
        book for book in books if int(book.actual_level) == level
    )
    if any(
        int(book.label.level) == level and book.label.number == number
        for book in actual_level_books
    ):
        raise RuntimeError("目标编号书位已经被占用")

    same_level = tuple(
        book
        for book in actual_level_books
        if int(book.label.level) == level
    )
    if not same_level:
        raise RuntimeError("目标层没有可用于推算书位的OCR书本")

    numbers = tuple(book.label.number for book in same_level)
    if len(set(numbers)) != len(numbers):
        raise RuntimeError("目标层存在重复OCR编号，不能推算书位")

    lower = max(
        (book for book in same_level if book.label.number < number),
        key=lambda book: book.label.number,
        default=None,
    )
    higher = min(
        (book for book in same_level if book.label.number > number),
        key=lambda book: book.label.number,
        default=None,
    )
    if (
        lower is not None
        and higher is not None
        and lower.label.number == number - 1
        and higher.label.number == number + 1
    ):
        if (
            lower.higher_number_edge_m is None
            or higher.lower_number_edge_m is None
        ):
            raise RuntimeError("相邻编号书缺少空位内侧边缘")
        left = _point3(lower.higher_number_edge_m)
        right = _point3(higher.lower_number_edge_m)
        gap_width = sum(
            (right[index] - left[index]) * axis[index]
            for index in range(3)
        )
        if not math.isfinite(gap_width) or gap_width <= 0.0:
            raise RuntimeError("相邻书空位边缘方向无效")
        center = tuple((left[index] + right[index]) / 2.0 for index in range(3))
        return ShelfTarget(
            level=level,
            number=number,
            center_m=_point3(center),
            left_m=left,
            right_m=right,
            yaw_rad=yaw,
        )

    if len(same_level) >= 2:
        pairs = [
            (first, second)
            for first_index, first in enumerate(same_level)
            for second in same_level[first_index + 1 :]
            if first.label.number != second.label.number
        ]
        if not pairs:
            raise RuntimeError("目标层OCR编号不足以推算书位")
        first, second = max(
            pairs,
            key=lambda pair: abs(pair[1].label.number - pair[0].label.number),
        )
        first_point = _point3(first.center_m)
        second_point = _point3(second.center_m)
        number_delta = second.label.number - first.label.number
        measured_delta = sum(
            (second_point[index] - first_point[index]) * axis[index]
            for index in range(3)
        )
        spacing_m = measured_delta / float(number_delta)
        if not math.isfinite(spacing_m) or spacing_m <= 0.0:
            raise RuntimeError("书架编号轴方向与OCR书本排列不一致")
        center = tuple(
            first_point[index]
            + axis[index] * spacing_m * (number - first.label.number)
            for index in range(3)
        )
    else:
        reference = same_level[0]
        reference_point = _point3(reference.center_m)
        center = tuple(
            reference_point[index]
            + axis[index]
            * DEFAULT_BOOK_WIDTH_M
            * (number - reference.label.number)
            for index in range(3)
        )

    half_width = DEFAULT_BOOK_WIDTH_M / 2.0
    left = tuple(center[index] - axis[index] * half_width for index in range(3))
    right = tuple(center[index] + axis[index] * half_width for index in range(3))
    return ShelfTarget(
        level=level,
        number=number,
        center_m=_point3(center),
        left_m=_point3(left),
        right_m=_point3(right),
        yaw_rad=yaw,
    )


@dataclass(frozen=True)
class _VerticalCartBook:
    observation: object
    mask: object
    suction_point_m: tuple[float, float, float]


@dataclass(frozen=True)
class _VerticalCartFrame:
    color_bgr: object
    books_right_to_left: tuple[_VerticalCartBook, ...]
    base_motion_epoch: str
    head_motion_epoch: str


def build_ocr_client():
    """Construct one reusable OCR client over the existing 7443 mTLS channel."""

    from book_rpc import BookVisionClient

    return BookVisionClient.from_config()


def _vertical_spine_suction_point(
    *,
    mask,
    depth_m,
    intrinsics,
    camera_to_base,
):
    import numpy as np

    depth = np.asarray(depth_m, dtype=float)
    spine = np.asarray(mask, dtype=bool)
    if depth.shape != spine.shape or not np.any(spine):
        raise RuntimeError("竖直书脊深度不可用")

    row_points = []
    for row in np.flatnonzero(np.any(spine, axis=1)):
        columns = np.flatnonzero(spine[row])
        if columns.size == 0:
            continue
        u = (float(columns[0]) + float(columns[-1])) / 2.0
        column = int(round(u))
        y0, y1 = max(0, int(row) - 2), min(depth.shape[0], int(row) + 3)
        x0, x1 = max(0, column - 2), min(depth.shape[1], column + 3)
        patch_depth = depth[y0:y1, x0:x1]
        patch_mask = spine[y0:y1, x0:x1]
        valid = patch_depth[
            patch_mask
            & np.isfinite(patch_depth)
            & (patch_depth >= 0.20)
            & (patch_depth <= 2.50)
        ]
        if valid.size == 0:
            continue
        distance_m = float(np.median(valid))
        camera_point = (
            (u - float(intrinsics.cx)) * distance_m / float(intrinsics.fx),
            (float(row) - float(intrinsics.cy))
            * distance_m
            / float(intrinsics.fy),
            distance_m,
        )
        base_point = _point3(camera_to_base(camera_point))
        row_points.append((u, float(row), base_point))

    if not row_points:
        raise RuntimeError("竖直书脊深度不可用")
    bottom = min(row_points, key=lambda item: item[2][2])
    target_z = bottom[2][2] + VERTICAL_SPINE_GRASP_HEIGHT_M
    return min(row_points, key=lambda item: abs(item[2][2] - target_z))[2]


def _capture_vertical_cart_frame(vision):
    import numpy as np

    from book_geometry import CameraIntrinsics, decode_bbox_rle
    from geometry import camera_point_to_base

    capture = vision.capture_book_frame(scan_angle_deg=0)
    if capture is None:
        raise RuntimeError("没有取得新的同步RGB-D")
    color, depth = vision._snapshot_arrays(capture.snapshot)
    if color is None:
        raise RuntimeError("没有取得新的同步RGB-D")

    snapshot = capture.snapshot
    joints = capture.joints
    captured_at_ns = int(snapshot.captured_at_ns)
    base_motion_epoch = f"stage2-cart-book-base-{captured_at_ns}"
    head_motion_epoch = f"stage2-cart-book-head-{captured_at_ns}"
    observations = vision.book_client.detect(
        color,
        captured_at_ns=time.time_ns(),
        base_motion_epoch=base_motion_epoch,
        head_motion_epoch=head_motion_epoch,
    )
    intrinsics = CameraIntrinsics(
        fx=float(snapshot.info.k[0]),
        fy=float(snapshot.info.k[4]),
        cx=float(snapshot.info.k[2]),
        cy=float(snapshot.info.k[5]),
    )
    body = float(joints["body_joint"])
    head_yaw = float(joints["joint_head0"])
    head_pitch = float(joints["joint_head1"])
    books = []
    for observation in observations:
        mask = decode_bbox_rle(
            image_shape=depth.shape,
            bbox=observation.bbox,
            counts=observation.rle_counts,
        )
        point = _vertical_spine_suction_point(
            mask=mask,
            depth_m=depth,
            intrinsics=intrinsics,
            camera_to_base=lambda camera_point: camera_point_to_base(
                camera_point,
                body,
                head_yaw,
                head_pitch,
            ),
        )
        books.append(
            _VerticalCartBook(
                observation=observation,
                mask=np.asarray(mask, dtype=bool),
                suction_point_m=point,
            )
        )
    if not books:
        raise RuntimeError("未检测到推车竖直书本")
    books.sort(
        key=lambda book: (
            float(book.observation.bbox[0])
            + float(book.observation.bbox[2]) / 2.0
        ),
        reverse=True,
    )
    return _VerticalCartFrame(
        color_bgr=np.ascontiguousarray(color),
        books_right_to_left=tuple(books),
        base_motion_epoch=base_motion_epoch,
        head_motion_epoch=head_motion_epoch,
    )


def _ocr_label_for_book(frame, book, ocr_client):
    labels = ocr_client.detect_labels(
        frame.color_bgr,
        captured_at_ns=time.time_ns(),
        base_motion_epoch=frame.base_motion_epoch,
        head_motion_epoch=frame.head_motion_epoch,
    )
    height, width = book.mask.shape
    associated = []
    for label in labels:
        center_u, center_v = ocr_box_center(label.polygon_px)
        column = int(round(center_u))
        row = int(round(center_v))
        if not (0 <= column < width and 0 <= row < height):
            continue
        if not bool(book.mask[row, column]):
            continue
        try:
            parsed = validate_book_label(label.text)
        except RuntimeError:
            continue
        associated.append((float(label.confidence), parsed))
    if not associated:
        raise RuntimeError("OCR无法识别")
    return max(associated, key=lambda item: item[0])[1]


def observe_rightmost_cart_book(vision, ocr_client, *, known_label=None):
    """Select the robot-view rightmost spine and attach its Stage-1 label."""

    frame = _capture_vertical_cart_frame(vision)
    selected = frame.books_right_to_left[0]
    label = (
        _ocr_label_for_book(frame, selected, ocr_client)
        if known_label is None
        else validate_book_label(known_label)
    )
    return TrackedBook(
        tracking_key=CartBookTrackingKey(
            order_from_right=0,
            label_text=label.text,
        ),
        suction_point_m=selected.suction_point_m,
        label=label,
    )


def observe_tracked_book(vision, tracking_key, *, actual_level=None):
    """Reobserve the same cart book by its stable right-to-left order."""

    if not isinstance(tracking_key, CartBookTrackingKey):
        raise RuntimeError("推车书本跟踪标识无效")
    frame = _capture_vertical_cart_frame(vision)
    index = int(tracking_key.order_from_right)
    if index < 0 or index >= len(frame.books_right_to_left):
        raise RuntimeError("无法重关联同一本推车书")
    selected = frame.books_right_to_left[index]
    return TrackedBook(
        tracking_key=tracking_key,
        suction_point_m=selected.suction_point_m,
        label=parse_book_label(tracking_key.label_text),
        actual_level=actual_level,
    )


def observe_shelf_target(vision, ocr_client, label):
    """Pending: scan labels, infer the numbered empty frame and return target."""

    raise NotImplementedError("待补书架OCR目标框感知接口")


def scan_stage3_shelf(vision, ocr_client):
    """Pending: scan levels 3/4/5 at all seven requested head angles."""

    raise NotImplementedError("待补Stage 3整架OCR扫描接口")


def select_unique_misplaced_book(shelf_scan):
    """Pending: return exactly one level- or number-misplaced book."""

    raise NotImplementedError("待补唯一错放书判定接口")


def navigate_initial_to_cart(vision):
    """Pending: cart on the robot's right, turn-drive-face coarse route."""

    raise NotImplementedError("待补Stage 2首次靠近推车粗导航")


def navigate_cart_to_shelf(vision, ocr_client, label):
    """Pending: retreat 20 cm, scan and approach the shelf to about 45 cm."""

    raise NotImplementedError("待补推车到书架粗导航")


def navigate_shelf_to_cart(vision):
    """Pending: retreat 45 cm, scan right and return to the cart."""

    raise NotImplementedError("待补书架到推车粗导航")


def retreat_after_stage2_third_place(vision):
    """Pending: retreat exactly 20 cm after the third Stage 2 Place."""

    raise NotImplementedError("待补Stage 2第三本放书后的20 cm后退")


def navigate_to_stage3_scan_pose(vision):
    """Pending: face the shelf with chassis outer edge at 60 cm."""

    raise NotImplementedError("待补Stage 3书架前60 cm定位")


def navigate_to_misplaced_book(vision, misplaced):
    """Pending: approach the misplaced book's shelf Pick working range."""

    raise NotImplementedError("待补Stage 3错放书抓取粗导航")


def navigate_to_correct_shelf_target(vision, ocr_client, label):
    """Pending: keep holding the book and approach its correct empty slot."""

    raise NotImplementedError("待补Stage 3抓取位置到正确书位粗导航")


def return_to_stage3_scan_pose(vision):
    """Pending: finish facing the shelf at the same 60 cm scan distance."""

    raise NotImplementedError("待补Stage 3结束位置导航")


def _build_replay_runtime(asset, *, joint_positions, spin_feedback):
    from book_pick_replay import (
        LegacyV3PickRuntime,
        PICK_ASSET_ID,
        V3_CONFIG_PATH,
        V3_ROOT,
    )
    from book_place_replay import LegacyV3PlaceRuntime, PLACE_ASSET_ID

    root_text = str(V3_ROOT)
    if root_text not in sys.path:
        sys.path.insert(0, root_text)

    from v3_pipeline.config import build_adapter, load_config
    from v3_pipeline.contracts import LoadState, RunContext
    from v3_pipeline.plan import build_stage1

    from book_navigation import load_navnav_runtime

    config = load_config(str(V3_CONFIG_PATH))
    replay_adapter = build_adapter(config)
    blocks = {block.step_id: block for block in build_stage1()}
    event_frame = int(D01_EVENT_FRAME_BY_ASSET[asset.name])
    if asset.operation == "pick":
        return LegacyV3PickRuntime(
            replay_adapter=replay_adapter,
            context=RunContext(
                run_id="fpc-stage23-pick-" + uuid.uuid4().hex,
                current_stage=1,
                load_state=LoadState.EMPTY_READY,
            ),
            pick_block=blocks["S1-B1-02"],
            check_block=blocks["S1-B1-03"],
            nav_runtime=load_navnav_runtime(),
            joint_positions=joint_positions,
            spin_feedback=spin_feedback,
            asset_id=PICK_ASSET_ID,
            asset_path=asset.path,
            frame_count=asset.frame_count,
            d01_events=[{
                "frame_index": event_frame,
                "command": "right_suction_start",
            }],
            asset_version=f"stage23-{asset.name.lower()}-20260822",
            operation_label=f"Stage 2/3 {asset.name} Pick",
        )

    runtime = LegacyV3PlaceRuntime(
        replay_adapter=replay_adapter,
        context=RunContext(
            run_id="fpc-stage23-place-" + uuid.uuid4().hex,
            current_stage=1,
            load_state=LoadState.LOADED_CARRY,
        ),
        pick_block=blocks["S1-B1-05"],
        check_block=blocks["S1-B1-06"],
        nav_runtime=load_navnav_runtime(),
        joint_positions=joint_positions,
        spin_feedback=spin_feedback,
        asset_id=PLACE_ASSET_ID,
        asset_path=asset.path,
        frame_count=asset.frame_count,
        d01_events=[{
            "frame_index": event_frame,
            "command": "right_suction_stop",
        }],
        asset_version=f"stage23-{asset.name.lower()}-20260822",
        operation_label=f"Stage 2/3 {asset.name} Place",
    )
    runtime.entry["speed"] = 1.0
    return runtime


def build_stage2_pick_one_replayer(*, joint_positions, spin_feedback):
    from book_pick_replay import Stage1BookPickReplayer

    asset = ASSETS["DR11.2"]
    return Stage1BookPickReplayer(
        runtime=_build_replay_runtime(
            asset,
            joint_positions=joint_positions,
            spin_feedback=spin_feedback,
        ),
        keep_runtime_open=True,
    )


def build_stage23_replayers(*, joint_positions, spin_feedback):
    from book_pick_replay import Stage1BookPickReplayer
    from book_place_replay import Stage1BookPlaceReplayer

    pick_replayers = {}
    place_replayers = {}
    try:
        for asset_name in ("DR11.2", "DR9.2", "DR8.1", "DR10.1"):
            asset = ASSETS[asset_name]
            pick_replayers[asset_name] = Stage1BookPickReplayer(
                runtime=_build_replay_runtime(
                    asset,
                    joint_positions=joint_positions,
                    spin_feedback=spin_feedback,
                ),
                keep_runtime_open=True,
            )
        for asset_name in ("DR5.1", "DR6.4", "DR7.1"):
            asset = ASSETS[asset_name]
            place_replayers[asset_name] = Stage1BookPlaceReplayer(
                runtime=_build_replay_runtime(
                    asset,
                    joint_positions=joint_positions,
                    spin_feedback=spin_feedback,
                ),
                keep_runtime_open=True,
            )
    except BaseException:
        for replayer in tuple(pick_replayers.values()) + tuple(
            place_replayers.values()
        ):
            try:
                replayer.close()
            except Exception:
                pass
        raise
    return pick_replayers, place_replayers


def preload_stage23_replayers(pick_replayers, place_replayers, say=print):
    for asset_name in ("DR11.2", "DR5.1", "DR6.4", "DR7.1"):
        say(f"加载 {asset_name}")
        if asset_name in pick_replayers:
            pick_replayers[asset_name].preload()
        else:
            place_replayers[asset_name].preload()
    for asset_name in ("DR9.2", "DR8.1", "DR10.1"):
        say(f"加载 {asset_name}")
        pick_replayers[asset_name].preload()


def close_stage23_replayers(pick_replayers, place_replayers):
    first_error = None
    for replayer in tuple(pick_replayers.values()) + tuple(place_replayers.values()):
        try:
            replayer.close()
        except Exception as error:
            if first_error is None:
                first_error = error
    if first_error is not None:
        raise first_error


def _pick_within_tolerance(residual):
    return (
        abs(residual[0]) <= BOOK_PICK_X_TOLERANCE_M
        and abs(residual[1]) <= BOOK_PICK_Y_TOLERANCE_M
    )


def align_tracked_book(
    vision,
    navigator,
    tracked_book,
    reference_point_m,
    *,
    say=print,
):
    reference = _point3(reference_point_m)
    current = tracked_book
    final_residual = None
    for correction_index in range(MAXIMUM_ALIGNMENT_CORRECTIONS + 1):
        observed = _point3(current.suction_point_m)
        final_residual = tuple(observed[index] - reference[index] for index in range(3))
        say(
            f"Pick精定位 {correction_index}/{MAXIMUM_ALIGNMENT_CORRECTIONS}: "
            f"dx={final_residual[0]:.3f} m, "
            f"dy={final_residual[1]:.3f} m, "
            f"dz={final_residual[2]:.3f} m"
        )
        if _pick_within_tolerance(final_residual):
            return final_residual
        if correction_index == MAXIMUM_ALIGNMENT_CORRECTIONS:
            break
        navigator.align(reference=reference, observed=observed)
        current = observe_tracked_book(
            vision,
            current.tracking_key,
            actual_level=current.actual_level,
        )
    raise RuntimeError(
        "Pick最终对位未达标，不执行DataReplay: "
        f"dx={final_residual[0]:.3f} m, dy={final_residual[1]:.3f} m"
    )


def set_shelf_observation_pose(vision, navigator, level, say=print):
    pose = SHELF_OBSERVATION_POSE_BY_LEVEL[int(level)]
    torso_actual = navigator.set_observation_torso(pose.torso_m)
    vision.set_head_pose(yaw_rad=0.0, pitch_rad=pose.head_pitch_rad)
    say(
        f"第{level}层视觉观察姿态: torso={torso_actual:.3f} m, "
        f"head_pitch={pose.head_pitch_rad:.3f} rad"
    )


def _normalize_yaw(value):
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


def align_shelf_target(
    vision,
    ocr_client,
    navigator,
    label,
    asset_name,
    *,
    say=print,
):
    reference = _point3(PLACE_REFERENCE_POINT_M_BY_ASSET[asset_name])
    reference_yaw = float(PLACE_REFERENCE_YAW_RAD_BY_ASSET[asset_name])
    correction_count = 0
    target = observe_shelf_target(vision, ocr_client, label)

    while True:
        observed = _point3(target.center_m)
        residual = tuple(observed[index] - reference[index] for index in range(3))
        yaw_error = _normalize_yaw(float(target.yaw_rad) - reference_yaw)
        say(
            f"{asset_name} yaw对位 {correction_count}/"
            f"{MAXIMUM_ALIGNMENT_CORRECTIONS}: "
            f"yaw={math.degrees(yaw_error):.2f}°"
        )
        if abs(yaw_error) <= SHELF_PLACE_YAW_TOLERANCE_RAD:
            break
        if correction_count == MAXIMUM_ALIGNMENT_CORRECTIONS:
            raise RuntimeError(
                "Place yaw对位未达标，不执行DataReplay: "
                f"yaw={math.degrees(yaw_error):.2f}°"
            )
        navigator.align(ShelfAlignmentCommand(
            reference_anchor_m=(0.0, 0.0, reference[2]),
            observed_anchor_m=(0.0, 0.0, reference[2]),
            residual_m=(0.0, 0.0, residual[2]),
            yaw_error_rad=yaw_error,
        ))
        correction_count += 1
        target = observe_shelf_target(vision, ocr_client, label)

    while True:
        observed = _point3(target.center_m)
        residual = tuple(observed[index] - reference[index] for index in range(3))
        say(
            f"{asset_name} 左右对位 {correction_count}/"
            f"{MAXIMUM_ALIGNMENT_CORRECTIONS}: "
            f"左右={residual[1]:.3f} m"
        )
        if abs(residual[1]) <= SHELF_PLACE_Y_TOLERANCE_M:
            break
        if correction_count == MAXIMUM_ALIGNMENT_CORRECTIONS:
            raise RuntimeError(
                "Place左右对位未达标，不执行DataReplay: "
                f"左右={residual[1]:.3f} m"
            )
        navigator.align(ShelfAlignmentCommand(
            reference_anchor_m=(0.0, reference[1], reference[2]),
            observed_anchor_m=(0.0, observed[1], reference[2]),
            residual_m=(0.0, residual[1], residual[2]),
            yaw_error_rad=0.0,
        ))
        correction_count += 1
        target = observe_shelf_target(vision, ocr_client, label)

    set_shelf_observation_pose(vision, navigator, label.level, say=say)
    target = observe_shelf_target(vision, ocr_client, label)

    while True:
        observed = _point3(target.center_m)
        residual = tuple(observed[index] - reference[index] for index in range(3))
        final = ShelfAlignmentCommand(
            reference_anchor_m=reference,
            observed_anchor_m=observed,
            residual_m=residual,
            yaw_error_rad=_normalize_yaw(float(target.yaw_rad) - reference_yaw),
        )
        say(
            f"{asset_name} 前后对位 {correction_count}/"
            f"{MAXIMUM_ALIGNMENT_CORRECTIONS}: "
            f"前后={residual[0]:.3f} m, Z记录={residual[2]:.3f} m"
        )
        if abs(residual[0]) <= SHELF_PLACE_X_TOLERANCE_M:
            return final
        if correction_count == MAXIMUM_ALIGNMENT_CORRECTIONS:
            raise RuntimeError(
                "Place前后对位未达标，不执行DataReplay: "
                f"前后={residual[0]:.3f} m"
            )
        navigator.align(ShelfAlignmentCommand(
            reference_anchor_m=(reference[0], 0.0, reference[2]),
            observed_anchor_m=(observed[0], 0.0, reference[2]),
            residual_m=(residual[0], 0.0, residual[2]),
            yaw_error_rad=0.0,
        ))
        correction_count += 1
        target = observe_shelf_target(vision, ocr_client, label)


def run_stage2_pick_one(
    vision,
    ocr_client,
    pick_navigator,
    cart_pick_replayer,
    *,
    say=print,
):
    say("恢复DR11.2第0帧全身姿态，底盘保持静止")
    cart_pick_replayer.prepare()
    known_label = load_stage1_cart_book_labels()[-1]
    selected = observe_rightmost_cart_book(
        vision,
        ocr_client,
        known_label=known_label,
    )
    label = validate_book_label(selected.label)
    say(f"最右侧书使用Stage 1记录标签={label.text}")
    align_tracked_book(
        vision,
        pick_navigator,
        selected,
        PICK_REFERENCE_POINT_M_BY_ASSET["DR11.2"],
        say=say,
    )
    say("启动DR11.2；回放第0帧立即开启右吸盘")
    result = cart_pick_replayer.pick(0.0, check_holding=False)
    say(
        f"DR11.2完成并停止: frames={result.frames_sent}, "
        "右吸盘保持开启"
    )
    return result


def run_stage2(
    vision,
    ocr_client,
    pick_navigator,
    place_navigator,
    pick_replayers,
    place_replayers,
    *,
    skip_initial_coarse=False,
    say=print,
):
    if not skip_initial_coarse:
        say("Stage 2：检测推车并执行首次粗导航")
        navigate_initial_to_cart(vision)

    cart_pick_replayer = pick_replayers["DR11.2"]
    labels_right_to_left = tuple(reversed(load_stage1_cart_book_labels()))
    for book_index, known_label in enumerate(labels_right_to_left, start=1):
        say(f"Stage 2 第 {book_index}/{len(labels_right_to_left)} 本")
        say("恢复DR11.2第0帧全身姿态，底盘保持静止")
        cart_pick_replayer.prepare()
        selected = observe_rightmost_cart_book(
            vision,
            ocr_client,
            known_label=known_label,
        )
        if selected.label is None:
            raise RuntimeError("OCR无法识别")
        label = validate_book_label(selected.label)
        place_asset_name = STAGE2_PLACE_ASSET_BY_LEVEL[label.level]
        say(f"识别到 {label.text}，放书使用 {place_asset_name}")
        align_tracked_book(
            vision,
            pick_navigator,
            selected,
            PICK_REFERENCE_POINT_M_BY_ASSET["DR11.2"],
            say=say,
        )
        pick_result = cart_pick_replayer.pick(0.0, check_holding=False)
        say(
            f"DR11.2完成: frames={pick_result.frames_sent}, "
            "D01 holding=未检查"
        )

        navigate_cart_to_shelf(vision, ocr_client, label)
        align_shelf_target(
            vision,
            ocr_client,
            place_navigator,
            label,
            place_asset_name,
            say=say,
        )
        place_result = place_replayers[place_asset_name].place()
        say(
            f"{place_asset_name}完成: frames={place_result.frames_sent}, "
            f"D01 released={place_result.d01_released}"
        )

        if book_index < len(labels_right_to_left):
            say("后退离开书架，重新检测cart_body并返回推车")
            navigate_shelf_to_cart(vision)
        else:
            say("Stage 2最后一本放书完成：向后离开书架20 cm")
            retreat_after_stage2_third_place(vision)


def run_stage3(
    vision,
    ocr_client,
    pick_navigator,
    place_navigator,
    pick_replayers,
    place_replayers,
    *,
    say=print,
):
    say("Stage 3：移动到书架前60 cm扫描位置")
    navigate_to_stage3_scan_pose(vision)
    shelf_scan = scan_stage3_shelf(vision, ocr_client)
    misplaced = select_unique_misplaced_book(shelf_scan)
    actual_level = int(misplaced.book.actual_level)
    if actual_level not in STAGE3_PICK_ASSET_BY_LEVEL:
        raise RuntimeError("错放书实际层级不是第三、第四或第五层")
    target_label = validate_book_label(misplaced.book.label)
    pick_asset_name = STAGE3_PICK_ASSET_BY_LEVEL[actual_level]
    place_asset_name = STAGE2_PLACE_ASSET_BY_LEVEL[target_label.level]
    say(
        f"唯一错放书={target_label.text}, 实际第{actual_level}层, "
        f"原因={misplaced.reason}, Pick={pick_asset_name}, "
        f"Place={place_asset_name}"
    )

    navigate_to_misplaced_book(vision, misplaced)
    shelf_pick_replayer = pick_replayers[pick_asset_name]
    say(f"恢复{pick_asset_name}第0帧全身姿态")
    shelf_pick_replayer.prepare()
    tracked = observe_tracked_book(
        vision,
        misplaced.book.tracking_key,
        actual_level=actual_level,
    )
    align_tracked_book(
        vision,
        pick_navigator,
        tracked,
        PICK_REFERENCE_POINT_M_BY_ASSET[pick_asset_name],
        say=say,
    )
    pick_result = shelf_pick_replayer.pick(0.0, check_holding=False)
    say(
        f"{pick_asset_name}完成: frames={pick_result.frames_sent}, "
        "D01 holding=未检查"
    )

    navigate_to_correct_shelf_target(vision, ocr_client, target_label)
    set_shelf_observation_pose(
        vision,
        place_navigator,
        target_label.level,
        say=say,
    )
    align_shelf_target(
        vision,
        ocr_client,
        place_navigator,
        target_label,
        place_asset_name,
        say=say,
    )
    place_result = place_replayers[place_asset_name].place()
    say(
        f"{place_asset_name}完成: frames={place_result.frames_sent}, "
        f"D01 released={place_result.d01_released}"
    )
    return_to_stage3_scan_pose(vision)
    say("Stage 3完成：机器人面向书架，底盘外沿距离书架60 cm")


def run_stage2_stage3(
    vision,
    *,
    book_align_mode,
    pick_replayers,
    place_replayers,
    skip_stage2_initial_coarse=False,
    say=print,
):
    from book_navigation import BookAlignmentNavigator, CartPlaceDockingNavigator

    ocr_client = build_ocr_client()
    pick_navigator = BookAlignmentNavigator(mode=book_align_mode)
    place_navigator = CartPlaceDockingNavigator()
    run_stage2(
        vision,
        ocr_client,
        pick_navigator,
        place_navigator,
        pick_replayers,
        place_replayers,
        skip_initial_coarse=skip_stage2_initial_coarse,
        say=say,
    )
    run_stage3(
        vision,
        ocr_client,
        pick_navigator,
        place_navigator,
        pick_replayers,
        place_replayers,
        say=say,
    )
