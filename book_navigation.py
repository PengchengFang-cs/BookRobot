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


@dataclass(frozen=True)
class BookAlignmentExecution:
    command_count: int
    mode: str
    odom_dx_m: float
    odom_dy_m: float
    imu_dyaw_rad: float


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

    def __init__(self, runtime=None, *, resume_final_forward_segments=None):
        self.runtime = runtime
        self.resume_final_forward_segments = resume_final_forward_segments

    def navigate(self):
        if self.runtime is None:
            self.runtime = load_navnav_runtime()
        adapter = self.runtime.WandaRos2Adapter()
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


def _distance_commands(runtime, kind, distance_m):
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
