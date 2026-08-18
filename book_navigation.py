"""Thin adapter around the robot's deployed navnav_final alignment commands."""

from dataclasses import dataclass
import importlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace


NAVNAV_ROOT = Path("/home/unix_ai/navnav_final")
NAVNAV_MODULE = "runtime.wanda_nav_whrc"
TORSO_MIN_M = 0.0
TORSO_MAX_M = 0.28
STAGE1_REPLAY_TORSO_M = 0.20
EFFECTIVE_Z_TOLERANCE_M = 0.003


@dataclass(frozen=True)
class BookAlignmentExecution:
    command_count: int
    replay_torso_m: float
    target_torso_m: float
    actual_torso_m: float
    effective_z_residual_m: float


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
            WandaRos2Adapter=adapter_module.WandaRos2Adapter,
        )
    finally:
        if inserted:
            sys.path.remove(root_text)


class BookAlignmentNavigator:
    """Execute one Y/Z/X alignment plan while preserving the current yaw."""

    def __init__(self, runtime=None):
        self.runtime = runtime

    def align(self, *, reference, observed):
        if self.runtime is None:
            self.runtime = load_navnav_runtime()
        adapter = self.runtime.WandaRos2Adapter(
            torso_tolerance_m=EFFECTIVE_Z_TOLERANCE_M
        )
        try:
            adapter.preflight()
            torso = adapter.current_torso_position()
            book_z_delta = float(observed[2]) - float(reference[2])
            torso_target = STAGE1_REPLAY_TORSO_M + book_z_delta
            if not math.isfinite(torso_target) or not (
                TORSO_MIN_M <= torso_target <= TORSO_MAX_M
            ):
                raise RuntimeError(
                    f"DataReplay 升降目标 {torso_target:.3f} m 超出机器人行程 "
                    f"[{TORSO_MIN_M:.3f}, {TORSO_MAX_M:.3f}] m"
                )
            planned_observed = (
                float(observed[0]),
                float(observed[1]),
                float(reference[2]) + torso_target - float(torso),
            )
            commands = self.runtime.build_base_alignment_commands(
                self.runtime.BasePoint3D(*reference),
                self.runtime.BasePoint3D(*planned_observed),
                torso,
            )
            for command in commands:
                adapter.execute_command(command, precision_mode=True)
            actual_torso = float(adapter.current_torso_position())
            effective_z_residual = book_z_delta - (
                actual_torso - STAGE1_REPLAY_TORSO_M
            )
            if abs(effective_z_residual) > EFFECTIVE_Z_TOLERANCE_M:
                raise RuntimeError(
                    f"高度对位后的有效 Z 残差 {effective_z_residual:.4f} m "
                    f"超过 {EFFECTIVE_Z_TOLERANCE_M:.4f} m"
                )
            return BookAlignmentExecution(
                command_count=len(commands),
                replay_torso_m=STAGE1_REPLAY_TORSO_M,
                target_torso_m=torso_target,
                actual_torso_m=actual_torso,
                effective_z_residual_m=effective_z_residual,
            )
        finally:
            adapter.stop()
