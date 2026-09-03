"""Thin adapter around the robot's deployed navnav_final alignment commands."""

from dataclasses import dataclass
import importlib
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace


NAVNAV_ROOT = Path("/home/unix_ai/navnav_final")
NAVNAV_MODULE = "runtime.wanda_nav_whrc"
VECTOR_FINAL_YAW_TOLERANCE_RAD = math.radians(0.15)
VECTOR_DRIVE_OVERSHOOT_COMPENSATION_M = 0.010
PICK_INITIAL_VECTOR_DRIVE_OVERSHOOT_COMPENSATION_M = 0.030
ALIGNMENT_ROTATION_SPEED_RAD_S = 0.24
CART_TURN_CLEARANCE_RETREAT_M = 0.20
CART_SCAN_CAPTURE_ANGLES_DEG = (45, 60)
TABLE_RETURN_SCAN_ANGLES_DEG = (45, 60)
COARSE_TRANSLATION_SPEED_MPS = 0.5
CART_COARSE_TRANSLATION_SPEED_MPS = 0.15
TABLE_COARSE_DRIVE_COMPENSATION_M = 0.06
COARSE_ROTATION_SPEED_RAD_S = 0.36
CART_SCAN_ROTATION_SPEED_RAD_S = COARSE_ROTATION_SPEED_RAD_S


def _drive_coarse_direct_with_odom(
    adapter,
    distance_m,
    *,
    speed_mps=CART_COARSE_TRANSLATION_SPEED_MPS,
):
    import rclpy
    from geometry_msgs.msg import Twist

    signed_distance = float(distance_m)
    target_distance = abs(signed_distance)
    if target_distance <= 1e-9:
        return
    direction = 1.0 if signed_distance > 0.0 else -1.0
    start = adapter.current_task_pose()
    heading_cosine = math.cos(start.yaw)
    heading_sine = math.sin(start.yaw)
    command = Twist()
    command.linear.x = direction * float(speed_mps)
    try:
        while rclpy.ok(context=adapter.context):
            pose = adapter.latest_task_pose()
            travelled = direction * (
                (pose.x - start.x) * heading_cosine
                + (pose.y - start.y) * heading_sine
            )
            if travelled >= target_distance:
                return
            adapter._zero_velocity_publisher.publish(command)
            rclpy.spin_once(adapter, timeout_sec=0.02)
    finally:
        adapter._publish_zero_velocity()


def _spin_coarse(adapter, angle_rad):
    start_yaw = adapter.current_absolute_imu_yaw()
    target_yaw = math.remainder(start_yaw + float(angle_rad), 2.0 * math.pi)
    adapter.correct_absolute_imu_yaw(
        target_yaw_rad=target_yaw,
        maximum_speed_rad_s=COARSE_ROTATION_SPEED_RAD_S,
    )


def _spin_alignment(adapter, angle_rad):
    start_yaw = adapter.current_absolute_imu_yaw()
    target_yaw = math.remainder(start_yaw + float(angle_rad), 2.0 * math.pi)
    adapter.correct_absolute_imu_yaw(
        target_yaw_rad=target_yaw,
        maximum_speed_rad_s=ALIGNMENT_ROTATION_SPEED_RAD_S,
    )


def _scan_cart_while_turning(adapter, vision):
    import rclpy
    from geometry_msgs.msg import Twist

    start = adapter.current_task_pose()
    target_yaw = start.yaw + math.pi / 2.0
    capture_angles = iter(CART_SCAN_CAPTURE_ANGLES_DEG)
    next_capture_angle = next(capture_angles, None)
    captures = []
    command = Twist()
    stable_since = None
    try:
        while rclpy.ok(context=adapter.context):
            pose = adapter.latest_task_pose()
            turned_rad = math.remainder(pose.yaw - start.yaw, 2.0 * math.pi)
            turned_deg = math.degrees(turned_rad)
            if (
                next_capture_angle is not None
                and turned_deg >= next_capture_angle
            ):
                capture = vision.capture_cart_frame(
                    scan_angle_deg=next_capture_angle,
                    keep_moving=lambda: adapter._zero_velocity_publisher.publish(
                        command
                    ),
                )
                if capture is not None:
                    adapter._zero_velocity_publisher.publish(command)
                    rclpy.spin_once(adapter, timeout_sec=0.02)
                    pose = adapter.latest_task_pose()
                    captures.append((pose, capture))
                next_capture_angle = next(capture_angles, None)
            yaw_error = math.remainder(target_yaw - pose.yaw, 2.0 * math.pi)
            if abs(yaw_error) <= math.radians(1.0):
                command.angular.z = 0.0
                now = time.monotonic()
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= 0.5:
                    return tuple(captures)
            else:
                stable_since = None
                command.angular.z = math.copysign(
                    min(
                        CART_SCAN_ROTATION_SPEED_RAD_S,
                        max(0.04, abs(yaw_error) * 0.8),
                    ),
                    yaw_error,
                )
            adapter._zero_velocity_publisher.publish(command)
            rclpy.spin_once(adapter, timeout_sec=0.02)
    finally:
        adapter._publish_zero_velocity()


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


def _build_vector_commands(
    runtime,
    reference,
    observed,
    *,
    overshoot_compensation_m=VECTOR_DRIVE_OVERSHOOT_COMPENSATION_M,
    epsilon=1e-9,
):
    dx = float(observed[0]) - float(reference[0])
    dy = float(observed[1]) - float(reference[1])
    distance = math.hypot(dx, dy)
    drive_distance = max(0.0, distance - float(overshoot_compensation_m))
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
        self._vector_correction_count = 0

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
                compensation_m = (
                    PICK_INITIAL_VECTOR_DRIVE_OVERSHOOT_COMPENSATION_M
                    if self._vector_correction_count == 0
                    else VECTOR_DRIVE_OVERSHOOT_COMPENSATION_M
                )
                commands = _build_vector_commands(
                    self.runtime,
                    reference,
                    planned_observed,
                    overshoot_compensation_m=compensation_m,
                )
            spin_count = 0
            for command in commands:
                if (
                    self.mode == "vector"
                    and command.kind is self.runtime.WandaCommandKind.SPIN
                ):
                    spin_count += 1
                    if spin_count == 2:
                        adapter.correct_absolute_imu_yaw(
                            target_yaw_rad=starting_absolute_yaw,
                            tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
                            maximum_speed_rad_s=ALIGNMENT_ROTATION_SPEED_RAD_S,
                        )
                    else:
                        _spin_alignment(adapter, command.value)
                else:
                    adapter.execute_command(command, precision_mode=True)
            if self.mode == "vector" and commands and spin_count == 0:
                adapter.correct_absolute_imu_yaw(
                    target_yaw_rad=starting_absolute_yaw,
                    tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
                    maximum_speed_rad_s=ALIGNMENT_ROTATION_SPEED_RAD_S,
                )
            pose = adapter.current_task_pose()
            if self.mode == "vector":
                self._vector_correction_count += 1
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

    def move_forward(self, distance_m):
        """Move a short signed distance while looking for cart markers."""

        if self.runtime is None:
            self.runtime = load_navnav_runtime()
        distance_m = float(distance_m)
        kind = (
            self.runtime.WandaCommandKind.DRIVE_FORWARD
            if distance_m >= 0.0
            else self.runtime.WandaCommandKind.DRIVE_BACKWARD
        )
        adapter = self.runtime.WandaRos2Adapter()
        try:
            adapter.preflight()
            adapter.capture_task_origin()
            adapter.execute_command(
                self.runtime.MappedMotionCommand(kind, abs(distance_m), "XY"),
                precision_mode=True,
            )
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
                if command.kind is self.runtime.WandaCommandKind.SPIN:
                    _spin_alignment(adapter, command.value)
                else:
                    adapter.execute_command(command, precision_mode=True)
            yaw_command_count = 0
            if abs(yaw_error) > 1e-9:
                adapter.correct_absolute_imu_yaw(
                    target_yaw_rad=starting_yaw + yaw_error,
                    tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
                    maximum_speed_rad_s=ALIGNMENT_ROTATION_SPEED_RAD_S,
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
        adapter._linear_speed = CART_COARSE_TRANSLATION_SPEED_MPS
        adapter._backup_speed = CART_COARSE_TRANSLATION_SPEED_MPS
        return self._navigate_with_cart_scan(adapter)

    def _navigate_with_cart_scan(self, adapter):
        commands_sent = 0
        candidates = []
        try:
            adapter.preflight()
            starting_yaw = adapter.current_absolute_imu_yaw()
            adapter.capture_task_origin()
            if not self.scan_only:
                adapter.execute_command(
                    self.runtime.MappedMotionCommand(
                        self.runtime.WandaCommandKind.DRIVE_BACKWARD,
                        CART_TURN_CLEARANCE_RETREAT_M,
                        "XY",
                    ),
                    precision_mode=False,
                )
                commands_sent += 1
            captures = _scan_cart_while_turning(adapter, self.vision)
            commands_sent += 1

            for capture_pose, capture in captures:
                carts = self.vision.detect_cart_frames_queued((capture,))
                cart = carts[0] if carts else None
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
                break

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
            print(
                "[导航] 推车粗定位（书本坐标系）: "
                f"target_x={cart_target_point[0]:.3f} m, "
                f"target_y={cart_target_point[1]:.3f} m, "
                f"move_y={lateral_delta:.3f} m"
            )
            if abs(lateral_delta) > 1e-9:
                _drive_coarse_direct_with_odom(adapter, lateral_delta)
                commands_sent += 1

            _spin_coarse(adapter, -math.pi / 2.0)
            commands_sent += 1

            adapter.execute_command(
                self.runtime.MappedMotionCommand(
                    self.runtime.WandaCommandKind.DRIVE_FORWARD,
                    CART_TURN_CLEARANCE_RETREAT_M,
                    "XY",
                ),
                precision_mode=False,
            )
            commands_sent += 1

            adapter.correct_absolute_imu_yaw(
                target_yaw_rad=starting_yaw,
                tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
                maximum_speed_rad_s=COARSE_ROTATION_SPEED_RAD_S,
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

    def __init__(self, runtime=None, *, vision=None):
        self.runtime = runtime
        self.vision = vision

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
        table_drive_distance = math.copysign(
            max(0.0, abs(table_leg) - TABLE_COARSE_DRIVE_COMPENSATION_M),
            table_leg,
        )
        print(
            "[导航] 书桌粗定位: "
            f"选中书本点=({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) m, "
            f"目标Y={COARSE_APPROACH_REFERENCE_BASE_M[1]:.3f} m, "
            f"计算直行距离={table_leg:.3f} m, "
            f"减去6cm后执行={table_drive_distance:.3f} m"
        )
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
            _spin_coarse(adapter, -math.pi / 2.0)
            commands_sent += 1
            if abs(table_drive_distance) > 1e-9:
                _drive_coarse_direct_with_odom(
                    adapter,
                    table_drive_distance,
                    speed_mps=COARSE_TRANSLATION_SPEED_MPS,
                )
                commands_sent += 1
            _spin_coarse(adapter, math.pi / 2.0)
            commands_sent += 1
            adapter.correct_absolute_imu_yaw(
                target_yaw_rad=starting_yaw,
                tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
                maximum_speed_rad_s=COARSE_ROTATION_SPEED_RAD_S,
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
