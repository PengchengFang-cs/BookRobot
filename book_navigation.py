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
RETURN_TABLE_MAP_POSE = (2.857213, -2.520253, math.radians(-1.920977))
CART_FRONT_MAP_POSE = (3.411691, -1.647823, math.radians(-1.558860))
CART_TURN_CLEARANCE_RETREAT_M = 0.20
CART_ROUTE_MAX_SEGMENT_M = 0.20
CART_SCAN_STEP_RAD = math.radians(15.0)
CART_SCAN_STEPS = 6
CART_COARSE_FRONT_CLEARANCE_M = 0.80


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


def cart_transition_body_delta(
    table_pose=RETURN_TABLE_MAP_POSE,
    cart_pose=CART_FRONT_MAP_POSE,
):
    """Express the reviewed table-to-cart map displacement in table body axes."""

    world_dx = float(cart_pose[0]) - float(table_pose[0])
    world_dy = float(cart_pose[1]) - float(table_pose[1])
    yaw = float(table_pose[2])
    return (
        math.cos(yaw) * world_dx + math.sin(yaw) * world_dy,
        -math.sin(yaw) * world_dx + math.cos(yaw) * world_dy,
    )


class Stage1CartMapNavigator:
    """Follow an axis-aligned table-to-cart route with turn clearance."""

    def __init__(
        self,
        runtime=None,
        *,
        resume_final_forward_segments=None,
        vision=None,
        book_index=1,
        scan_only=False,
    ):
        self.runtime = runtime
        self.resume_final_forward_segments = resume_final_forward_segments
        self.vision = vision
        self.book_index = int(book_index)
        self.scan_only = bool(scan_only)
        if self.book_index < 1 or self.book_index > 5:
            raise ValueError("book_index must be between 1 and 5")

    def navigate(self):
        if self.runtime is None:
            self.runtime = load_navnav_runtime()
        adapter = self.runtime.WandaRos2Adapter()
        if self.vision is not None and self.resume_final_forward_segments is None:
            return self._navigate_with_cart_scan(adapter)
        dx_m, dy_m = cart_transition_body_delta()
        if self.resume_final_forward_segments is None:
            commands = build_cart_manhattan_commands(self.runtime, dx_m, dy_m)
            mode = "map-manhattan"
        else:
            commands = build_cart_final_forward_commands(
                self.runtime,
                dx_m,
                self.resume_final_forward_segments,
            )
            mode = "map-manhattan-resume"
        try:
            adapter.preflight()
            starting_yaw = adapter.current_absolute_imu_yaw()
            adapter.capture_task_origin()
            for command in commands:
                adapter.execute_command(command, precision_mode=True)
            adapter.correct_absolute_imu_yaw(
                target_yaw_rad=starting_yaw,
                tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
            )
            pose = adapter.current_task_pose()
            return BookAlignmentExecution(
                command_count=len(commands),
                mode=mode,
                odom_dx_m=float(pose.x),
                odom_dy_m=float(pose.y),
                imu_dyaw_rad=float(pose.yaw),
            )
        finally:
            adapter.stop()

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
                adapter.execute_command(retreat, precision_mode=True)
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
                capture = self.vision.capture_cart_frame(
                    scan_angle_deg=step * 15,
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

            # At the 90-degree scan endpoint the robot faces the cart.  Express
            # its RGB-D target in the current body frame, remove lateral error
            # with a right-angle leg, then restore the cart-facing yaw.
            target_in_body = _origin_point_to_body(cart_target_point, pose)
            lateral_delta = float(target_in_body[1])
            lateral_commands = _lateral_alignment_commands(
                self.runtime,
                lateral_delta,
            )
            for command in lateral_commands:
                adapter.execute_command(command, precision_mode=True)
                commands_sent += 1

            pose = adapter.current_task_pose()
            target_in_body = _origin_point_to_body(cart_target_point, pose)
            forward_delta = (
                float(target_in_body[0]) - CART_COARSE_FRONT_CLEARANCE_M
            )
            forward_kind = (
                self.runtime.WandaCommandKind.DRIVE_FORWARD
                if forward_delta >= 0.0
                else self.runtime.WandaCommandKind.DRIVE_BACKWARD
            )
            for command in _distance_commands(
                self.runtime,
                forward_kind,
                abs(forward_delta),
            ):
                adapter.execute_command(command, precision_mode=True)
                commands_sent += 1

            adapter.correct_absolute_imu_yaw(
                target_yaw_rad=_normalize_yaw(starting_yaw + math.pi / 2.0),
                tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
            )

            # Rotation and long grid legs can accumulate tyre-slip error.  Once
            # the robot faces the cart, take a new RGB-D observation and reuse
            # the same vector X/Y correction used before book pickup.
            fine_cart = self.vision.find_cart(require_body_target=True)
            if fine_cart is None or fine_cart.body_target is None:
                raise RuntimeError("正面朝向小推车后没有获得可用的微调深度点")
            fine_observed = fine_cart.body_target.center
            fine_reference = (
                CART_COARSE_FRONT_CLEARANCE_M,
                0.0,
                float(fine_observed[2]),
            )
            fine_commands = _build_vector_commands(
                self.runtime,
                fine_reference,
                fine_observed,
            )
            cart_facing_yaw = adapter.current_absolute_imu_yaw()
            for command in fine_commands:
                adapter.execute_command(command, precision_mode=True)
                commands_sent += 1
            if fine_commands:
                adapter.correct_absolute_imu_yaw(
                    target_yaw_rad=cart_facing_yaw,
                    tolerance_rad=VECTOR_FINAL_YAW_TOLERANCE_RAD,
                )
            pose = adapter.current_task_pose()
            return CartVisualNavigationExecution(
                command_count=commands_sent,
                mode="cart-visual-manhattan-fine",
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


def _origin_point_to_body(point, pose):
    dx = float(point[0]) - float(pose.x)
    dy = float(point[1]) - float(pose.y)
    x, y = _rotate_xy((dx, dy), -float(pose.yaw))
    return (x, y, float(point[2]))


def _lateral_alignment_commands(runtime, lateral_m):
    if abs(float(lateral_m)) <= 1e-9:
        return ()
    turn = math.copysign(math.pi / 2.0, float(lateral_m))
    commands = [
        runtime.MappedMotionCommand(runtime.WandaCommandKind.SPIN, turn, "Y")
    ]
    commands.extend(_distance_commands(
        runtime,
        runtime.WandaCommandKind.DRIVE_FORWARD,
        abs(float(lateral_m)),
    ))
    commands.append(
        runtime.MappedMotionCommand(runtime.WandaCommandKind.SPIN, -turn, "Y")
    )
    return tuple(commands)


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


def build_cart_manhattan_commands(runtime, dx_m, dy_m):
    """Back away, move laterally on a 90-degree leg, then finish forward."""

    commands = list(_distance_commands(
        runtime,
        runtime.WandaCommandKind.DRIVE_BACKWARD,
        CART_TURN_CLEARANCE_RETREAT_M,
    ))
    lateral_turn = math.copysign(math.pi / 2.0, float(dy_m))
    commands.append(
        runtime.MappedMotionCommand(runtime.WandaCommandKind.SPIN, lateral_turn, "Y")
    )
    commands.extend(_distance_commands(
        runtime,
        runtime.WandaCommandKind.DRIVE_FORWARD,
        abs(float(dy_m)),
    ))
    commands.append(
        runtime.MappedMotionCommand(runtime.WandaCommandKind.SPIN, -lateral_turn, "Y")
    )
    forward_m = float(dx_m) + CART_TURN_CLEARANCE_RETREAT_M
    forward_kind = (
        runtime.WandaCommandKind.DRIVE_FORWARD
        if forward_m >= 0.0
        else runtime.WandaCommandKind.DRIVE_BACKWARD
    )
    commands.extend(_distance_commands(runtime, forward_kind, abs(forward_m)))
    return tuple(commands)


def build_cart_final_forward_commands(runtime, dx_m, segment_count):
    """Return the requested trailing commands from the final forward leg."""

    all_commands = _distance_commands(
        runtime,
        runtime.WandaCommandKind.DRIVE_FORWARD,
        float(dx_m) + CART_TURN_CLEARANCE_RETREAT_M,
    )
    segment_count = int(segment_count)
    if segment_count < 1 or segment_count > len(all_commands):
        raise ValueError(
            f"remaining final segments must be between 1 and {len(all_commands)}"
        )
    return all_commands[-segment_count:]
