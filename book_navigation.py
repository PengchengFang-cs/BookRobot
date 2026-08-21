"""Thin adapter around the robot's deployed navnav_final alignment commands."""

from dataclasses import dataclass
import importlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace


NAVNAV_ROOT = Path("/home/unix_ai/navnav_final")
NAVNAV_MODULE = "runtime.wanda_nav_whrc"
VECTOR_FINAL_YAW_TOLERANCE_RAD = math.radians(0.15)
VECTOR_DRIVE_OVERSHOOT_COMPENSATION_M = 0.010
CART_TURN_CLEARANCE_RETREAT_M = 0.20
CART_ROUTE_MAX_SEGMENT_M = 0.20
CART_SCAN_STEP_RAD = math.radians(15.0)
CART_SCAN_STEPS = 6
CART_SCAN_CAPTURE_ANGLES_DEG = (30, 45, 60)
TABLE_RETURN_SCAN_ANGLES_DEG = (30, 45, 60, 75)
COARSE_TRANSLATION_SPEED_MPS = 0.5


@dataclass(frozen=True)
class BookAlignmentExecution:
    command_count: int
    mode: str
    odom_dx_m: float
    odom_dy_m: float
    imu_dyaw_rad: float


@dataclass(frozen=True)
class CartVisualNavigationExecution:
    command_count: int
    mode: str
    odom_dx_m: float
    odom_dy_m: float
    imu_dyaw_rad: float
    selected_scan_yaw_rad: float
    slot_index: int
    cart_target_origin_m: tuple[float, float, float]
    slot_center_origin_m: tuple[float, float, float]
    platform_near_x_origin_m: float
    selected_scan_debug_image: str | None


@dataclass(frozen=True)
class TableReturnNavigationExecution:
    command_count: int
    mode: str
    odom_dx_m: float
    odom_dy_m: float
    imu_dyaw_rad: float
    selected_book_point_m: tuple[float, float, float]
    table_leg_m: float


def _normalize_yaw(value):
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


def _build_vector_commands(runtime, reference, observed, epsilon=1e-9):
    dx = float(observed[0]) - float(reference[0])
    dy = float(observed[1]) - float(reference[1])
    distance = math.hypot(dx, dy)
    drive_distance = max(0.0, distance - VECTOR_DRIVE_OVERSHOOT_COMPENSATION_M)
    if drive_distance <= epsilon:
        return ()
    forward_turn = _normalize_yaw(math.atan2(dy, dx))
    reverse_turn = _normalize_yaw(forward_turn + math.pi)
    if abs(reverse_turn) < abs(forward_turn):
        turn = reverse_turn
        drive_kind = runtime.WandaCommandKind.DRIVE_BACKWARD
    else:
        turn = forward_turn
        drive_kind = runtime.WandaCommandKind.DRIVE_FORWARD
    commands = []
    if abs(turn) > epsilon:
        commands.append(
            runtime.MappedMotionCommand(runtime.WandaCommandKind.SPIN, turn, "XY")
        )
    commands.append(runtime.MappedMotionCommand(drive_kind, drive_distance, "XY"))
    if abs(turn) > epsilon:
        commands.append(
            runtime.MappedMotionCommand(runtime.WandaCommandKind.SPIN, -turn, "XY")
        )
    return tuple(commands)


def load_navnav_runtime(root=NAVNAV_ROOT):
    root = Path(root)
    if not root.is_dir():
        raise RuntimeError(f"找不到机器人导航运行时: {root}")
    root_text = str(root)
    inserted = root_text not in sys.path
    if inserted:
        sys.path.insert(0, root_text)
    try:
        package = importlib.import_module(NAVNAV_MODULE)
        adapter_module = importlib.import_module(
            f"{NAVNAV_MODULE}.wanda_ros2_adapter"
        )
        return SimpleNamespace(
            BasePoint3D=package.BasePoint3D,
            build_base_alignment_commands=package.build_base_alignment_commands,
            MappedMotionCommand=package.MappedMotionCommand,
            WandaCommandKind=package.WandaCommandKind,
            WandaRos2Adapter=adapter_module.WandaRos2Adapter,
        )
    finally:
        if inserted:
            sys.path.remove(root_text)


class BookAlignmentNavigator:
    """Execute one X/Y base alignment while preserving yaw and torso height."""

    def __init__(self, runtime=None, mode="legacy"):
        if mode not in ("legacy", "vector"):
            raise ValueError("alignment mode must be legacy or vector")
        self.runtime = runtime
        self.mode = mode

    def align(self, *, reference, observed):
        if self.runtime is None:
            self.runtime = load_navnav_runtime()
        adapter = self.runtime.WandaRos2Adapter()
        try:
            adapter.preflight()
            starting_absolute_yaw = (
                adapter.current_absolute_imu_yaw()
                if self.mode == "vector"
                else None
            )
            adapter.capture_task_origin()
            torso = adapter.current_torso_position()
            planned_observed = (
                float(observed[0]),
                float(observed[1]),
                float(reference[2]),
            )
            if self.mode == "legacy":
                commands = self.runtime.build_base_alignment_commands(
                    self.runtime.BasePoint3D(*reference),
                    self.runtime.BasePoint3D(*planned_observed),
                    torso,
                )
            else:
                commands = _build_vector_commands(
                    self.runtime,
                    reference,
                    planned_observed,
                )
            for command in commands:
                adapter.execute_command(command, precision_mode=True)
            if self.mode == "vector" and commands:
                adapter.correct_absolute_imu_yaw(
                    target_yaw_rad=starting_absolute_yaw,
                    tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
                )
            pose = adapter.current_task_pose()
            return BookAlignmentExecution(
                command_count=len(commands),
                mode=self.mode,
                odom_dx_m=float(pose.x),
                odom_dy_m=float(pose.y),
                imu_dyaw_rad=float(pose.yaw),
            )
        finally:
            adapter.stop()


class CartPlaceDockingNavigator:
    """Apply one complete replay-platform SE(2) correction from one image."""

    def __init__(self, runtime=None):
        self.runtime = runtime

    def set_observation_torso(self, torso_m):
        """Restore the recorded Place torso height before cart perception."""

        if self.runtime is None:
            self.runtime = load_navnav_runtime()
        adapter = self.runtime.WandaRos2Adapter()
        try:
            adapter.preflight()
            command = self.runtime.MappedMotionCommand(
                self.runtime.WandaCommandKind.TORSO_POSITION,
                float(torso_m),
                "Z",
            )
            adapter.execute_command(command, precision_mode=True)
            return float(adapter.current_torso_position())
        finally:
            adapter.stop()

    def align(self, target):
        if self.runtime is None:
            self.runtime = load_navnav_runtime()
        adapter = self.runtime.WandaRos2Adapter()
        try:
            adapter.preflight()
            starting_yaw = adapter.current_absolute_imu_yaw()
            adapter.capture_task_origin()
            yaw_error = float(target.yaw_error_rad)
            cosine = math.cos(yaw_error)
            sine = math.sin(yaw_error)
            reference_x = float(target.reference_anchor_m[0])
            reference_y = float(target.reference_anchor_m[1])
            rotated_reference = (
                cosine * reference_x - sine * reference_y,
                sine * reference_x + cosine * reference_y,
                float(target.reference_anchor_m[2]),
            )
            translation = (
                float(target.observed_anchor_m[0]) - rotated_reference[0],
                float(target.observed_anchor_m[1]) - rotated_reference[1],
                rotated_reference[2],
            )
            commands = _build_vector_commands(
                self.runtime,
                (0.0, 0.0, translation[2]),
                translation,
            )
            for command in commands:
                adapter.execute_command(command, precision_mode=True)
            yaw_command_count = 0
            if abs(yaw_error) > 1e-9:
                adapter.correct_absolute_imu_yaw(
                    target_yaw_rad=starting_yaw + yaw_error,
                    tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
                )
                yaw_command_count = 1
            pose = adapter.current_task_pose()
            return BookAlignmentExecution(
                command_count=len(commands) + yaw_command_count,
                mode="cart-place-se2",
                odom_dx_m=float(pose.x),
                odom_dy_m=float(pose.y),
                imu_dyaw_rad=float(pose.yaw),
            )
        finally:
            adapter.stop()


class Stage1CartNavigator:
    """Run the reviewed Pick-endpoint to cart coarse transition."""

    def __init__(
        self,
        runtime=None,
        *,
        vision=None,
        book_index=1,
        scan_only=False,
    ):
        self.runtime = runtime
        self.vision = vision
        self.book_index = int(book_index)
        self.scan_only = bool(scan_only)
        if self.book_index < 1 or self.book_index > 5:
            raise ValueError("book_index must be between 1 and 5")

    def navigate(self):
        if self.vision is None:
            raise RuntimeError("cart navigation requires live cart vision")
        if self.runtime is None:
            self.runtime = load_navnav_runtime()
        adapter = self.runtime.WandaRos2Adapter()
        adapter._linear_speed = COARSE_TRANSLATION_SPEED_MPS
        adapter._backup_speed = COARSE_TRANSLATION_SPEED_MPS
        return self._navigate_with_cart_scan(adapter)

    def _navigate_with_cart_scan(self, adapter):
        commands_sent = 0
        candidates = []
        captures = []
        try:
            adapter.preflight()
            starting_yaw = adapter.current_absolute_imu_yaw()
            adapter.capture_task_origin()
            if not self.scan_only:
                retreat = self.runtime.MappedMotionCommand(
                    self.runtime.WandaCommandKind.DRIVE_BACKWARD,
                    CART_TURN_CLEARANCE_RETREAT_M,
                    "XY",
                )
                adapter.execute_command(retreat, precision_mode=False)
                commands_sent += 1

            for step in range(1, CART_SCAN_STEPS + 1):
                adapter.refresh_feedback()
                turn = self.runtime.MappedMotionCommand(
                    self.runtime.WandaCommandKind.SPIN,
                    CART_SCAN_STEP_RAD,
                    "Y",
                )
                adapter.execute_command(turn, precision_mode=True)
                commands_sent += 1
                pose = adapter.current_task_pose()
                scan_angle_deg = step * 15
                if scan_angle_deg in CART_SCAN_CAPTURE_ANGLES_DEG:
                    capture = self.vision.capture_cart_frame(
                        scan_angle_deg=scan_angle_deg,
                    )
                    if capture is not None:
                        captures.append((pose, capture))

            carts = self.vision.detect_cart_frames_queued(
                capture for _pose, capture in captures
            )
            for (capture_pose, _capture), cart in zip(captures, carts):
                if cart is None or cart.body_target is None:
                    continue
                cart_target = transform_cart_body_target_to_origin(
                    cart.body_target,
                    capture_pose,
                )
                platform = (
                    transform_cart_platform_to_origin(cart.platform, capture_pose)
                    if cart.platform is not None
                    else None
                )
                candidates.append(
                    (
                        cart_target.confidence,
                        float(capture_pose.yaw),
                        cart_target,
                        platform,
                        cart.debug_image,
                    )
                )

            pose = adapter.current_task_pose()
            if not candidates:
                raise RuntimeError("旋转90度期间没有获得可用的小推车整体深度点")
            (
                _confidence,
                selected_scan_yaw,
                cart_target,
                selected_platform,
                selected_debug_image,
            ) = max(
                candidates,
                key=lambda row: row[0],
            )
            cart_target_point = cart_target.center
            if selected_platform is not None:
                platform = selected_platform
                slot_center = platform.slot_centers[self.book_index - 1]
                platform_near_x = _platform_near_x(platform)
            else:
                slot_center = cart_target_point
                platform_near_x = float(cart_target_point[0])

            if self.scan_only:
                return CartVisualNavigationExecution(
                    command_count=commands_sent,
                    mode="cart-scan-only",
                    odom_dx_m=float(pose.x),
                    odom_dy_m=float(pose.y),
                    imu_dyaw_rad=float(pose.yaw),
                    selected_scan_yaw_rad=selected_scan_yaw,
                    slot_index=self.book_index,
                    cart_target_origin_m=cart_target_point,
                    slot_center_origin_m=slot_center,
                    platform_near_x_origin_m=platform_near_x,
                    selected_scan_debug_image=selected_debug_image,
                )

            # X/Y stay in the scan-origin frame, whose axes are fixed when the
            # robot faces the books: X is forward/back and Y is left/right.
            # The chassis is already turned left by 90 degrees, so a straight
            # drive now executes the required Y displacement.
            lateral_delta = float(cart_target_point[1]) - float(pose.y)
            lateral_kind = (
                self.runtime.WandaCommandKind.DRIVE_FORWARD
                if lateral_delta >= 0.0
                else self.runtime.WandaCommandKind.DRIVE_BACKWARD
            )
            print(
                "[导航] 推车粗定位（书本坐标系）: "
                f"target_x={cart_target_point[0]:.3f} m, "
                f"target_y={cart_target_point[1]:.3f} m, "
                f"move_y={lateral_delta:.3f} m"
            )
            for command in _distance_commands(
                self.runtime,
                lateral_kind,
                abs(lateral_delta),
            ):
                adapter.execute_command(command, precision_mode=False)
                commands_sent += 1

            return_turn = self.runtime.MappedMotionCommand(
                self.runtime.WandaCommandKind.SPIN,
                -math.pi / 2.0,
                "Y",
            )
            adapter.execute_command(return_turn, precision_mode=True)
            commands_sent += 1

            # The route started by backing exactly 20 cm away from the Pick
            # replay endpoint. After completing the Y leg and restoring yaw,
            # move forward by the same 20 cm. This finishes coarse navigation;
            # precise docking belongs to shelf/DataReplay geometry, not the
            # cart-body center used above to estimate the long Y leg.
            forward_delta = CART_TURN_CLEARANCE_RETREAT_M
            print(
                "[导航] 推车粗定位（书本坐标系）: "
                f"restore_x={forward_delta:.3f} m; 粗导航完成"
            )
            for command in _distance_commands(
                self.runtime,
                self.runtime.WandaCommandKind.DRIVE_FORWARD,
                forward_delta,
            ):
                adapter.execute_command(command, precision_mode=False)
                commands_sent += 1

            adapter.correct_absolute_imu_yaw(
                target_yaw_rad=starting_yaw,
                tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
            )

            pose = adapter.current_task_pose()
            return CartVisualNavigationExecution(
                command_count=commands_sent,
                mode="cart-visual-manhattan-coarse",
                odom_dx_m=float(pose.x),
                odom_dy_m=float(pose.y),
                imu_dyaw_rad=float(pose.yaw),
                selected_scan_yaw_rad=selected_scan_yaw,
                slot_index=self.book_index,
                cart_target_origin_m=cart_target_point,
                slot_center_origin_m=slot_center,
                platform_near_x_origin_m=platform_near_x,
                selected_scan_debug_image=selected_debug_image,
            )
        finally:
            adapter.stop()


class Stage1TableReturnNavigator:
    """Return from the cart to the table using the inverse right-angle route."""

    def __init__(self, runtime=None, *, vision=None, retreat_before_turn=True):
        self.runtime = runtime
        self.vision = vision
        self.retreat_before_turn = bool(retreat_before_turn)

    def navigate(self):
        if self.vision is None:
            raise RuntimeError("table return requires live book vision")
        books = tuple(
            self.vision.scan_books_to_robot_right(TABLE_RETURN_SCAN_ANGLES_DEG)
        )
        if not books:
            raise RuntimeError("30/45/60/75度扫描没有找到书本")
        selected = max(
            books,
            key=lambda book: math.hypot(
                float(book.suction_point[0]),
                float(book.suction_point[1]),
            ),
        )
        point = tuple(float(value) for value in selected.suction_point)
        from book_alignment import COARSE_APPROACH_REFERENCE_BASE_M

        table_leg = float(COARSE_APPROACH_REFERENCE_BASE_M[1]) - point[1]
        if self.runtime is None:
            self.runtime = load_navnav_runtime()
        adapter = self.runtime.WandaRos2Adapter()
        adapter._linear_speed = COARSE_TRANSLATION_SPEED_MPS
        adapter._backup_speed = COARSE_TRANSLATION_SPEED_MPS
        commands_sent = 0
        try:
            adapter.preflight()
            starting_yaw = adapter.current_absolute_imu_yaw()
            adapter.capture_task_origin()
            route = []
            if self.retreat_before_turn:
                route.append(
                    self.runtime.MappedMotionCommand(
                        self.runtime.WandaCommandKind.DRIVE_BACKWARD,
                        CART_TURN_CLEARANCE_RETREAT_M,
                        "XY",
                    )
                )
            route.append(
                self.runtime.MappedMotionCommand(
                    self.runtime.WandaCommandKind.SPIN, -math.pi / 2.0, "Y"
                )
            )
            for command in route:
                adapter.execute_command(
                    command,
                    precision_mode=(command.kind is self.runtime.WandaCommandKind.SPIN),
                )
                commands_sent += 1
            kind = (
                self.runtime.WandaCommandKind.DRIVE_FORWARD
                if table_leg >= 0.0
                else self.runtime.WandaCommandKind.DRIVE_BACKWARD
            )
            for command in _distance_commands(self.runtime, kind, abs(table_leg)):
                adapter.execute_command(command, precision_mode=False)
                commands_sent += 1
            finish = [
                self.runtime.MappedMotionCommand(
                    self.runtime.WandaCommandKind.SPIN, math.pi / 2.0, "Y"
                )
            ]
            if self.retreat_before_turn:
                finish.append(
                    self.runtime.MappedMotionCommand(
                        self.runtime.WandaCommandKind.DRIVE_FORWARD,
                        CART_TURN_CLEARANCE_RETREAT_M,
                        "XY",
                    )
                )
            for command in finish:
                adapter.execute_command(
                    command,
                    precision_mode=(command.kind is self.runtime.WandaCommandKind.SPIN),
                )
                commands_sent += 1
            adapter.correct_absolute_imu_yaw(
                target_yaw_rad=starting_yaw,
                tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
            )
            pose = adapter.current_task_pose()
            return TableReturnNavigationExecution(
                command_count=commands_sent,
                mode="table-visual-manhattan-coarse",
                odom_dx_m=float(pose.x),
                odom_dy_m=float(pose.y),
                imu_dyaw_rad=float(pose.yaw),
                selected_book_point_m=point,
                table_leg_m=table_leg,
            )
        finally:
            adapter.stop()


def _rotate_xy(value, yaw):
    cosine = math.cos(float(yaw))
    sine = math.sin(float(yaw))
    return (
        cosine * float(value[0]) - sine * float(value[1]),
        sine * float(value[0]) + cosine * float(value[1]),
    )


def transform_cart_platform_to_origin(platform, pose):
    """Express one capture-time base_link platform in the scan origin frame."""

    def point(value):
        x, y = _rotate_xy(value, pose.yaw)
        return (
            float(pose.x) + x,
            float(pose.y) + y,
            float(value[2]),
        )

    def axis(value):
        x, y = _rotate_xy(value, pose.yaw)
        return (x, y, float(value[2]))

    return SimpleNamespace(
        center=point(platform.center),
        forward_axis=axis(platform.forward_axis),
        lateral_axis_right_to_left=axis(platform.lateral_axis_right_to_left),
        normal=axis(platform.normal),
        depth_extent_m=float(platform.depth_extent_m),
        lateral_extent_m=float(platform.lateral_extent_m),
        slot_centers=tuple(point(value) for value in platform.slot_centers),
        confidence=float(platform.confidence),
    )


def transform_cart_body_target_to_origin(target, pose):
    """Express one capture-time cart-body point in the scan origin frame."""

    x, y = _rotate_xy(target.center, pose.yaw)
    return SimpleNamespace(
        center=(
            float(pose.x) + x,
            float(pose.y) + y,
            float(target.center[2]),
        ),
        confidence=float(target.confidence),
    )


def _platform_near_x(platform):
    half_x_extent = 0.5 * (
        abs(float(platform.forward_axis[0])) * float(platform.depth_extent_m)
        + abs(float(platform.lateral_axis_right_to_left[0]))
        * float(platform.lateral_extent_m)
    )
    return float(platform.center[0]) - half_x_extent


def _distance_commands(runtime, kind, distance_m):
    if float(distance_m) <= 1e-9:
        return ()
    count = max(1, math.ceil(float(distance_m) / CART_ROUTE_MAX_SEGMENT_M))
    segment = float(distance_m) / count
    return tuple(
        runtime.MappedMotionCommand(kind, segment, "XY")
        for _ in range(count)
    )
