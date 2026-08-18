"""Thin adapter around the robot's deployed navnav_final alignment commands."""

from dataclasses import dataclass
import importlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace


NAVNAV_ROOT = Path("/home/unix_ai/navnav_final")
NAVNAV_MODULE = "runtime.wanda_nav_whrc"
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
    if distance <= epsilon:
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
    commands.append(runtime.MappedMotionCommand(drive_kind, distance, "XY"))
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
