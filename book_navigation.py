"""Thin adapter around the robot's deployed navnav_final alignment commands."""

import importlib
from pathlib import Path
import sys


NAVNAV_ROOT = Path("/home/unix_ai/navnav_final")
NAVNAV_MODULE = "runtime.wanda_nav_whrc"


def load_navnav_runtime(root=NAVNAV_ROOT):
    root = Path(root)
    if not root.is_dir():
        raise RuntimeError(f"找不到机器人导航运行时: {root}")
    root_text = str(root)
    inserted = root_text not in sys.path
    if inserted:
        sys.path.insert(0, root_text)
    try:
        return importlib.import_module(NAVNAV_MODULE)
    finally:
        if inserted:
            sys.path.remove(root_text)


class BookAlignmentNavigator:
    """Execute one Y/Z/X alignment plan while preserving the current yaw."""

    def __init__(self, runtime=None):
        self.runtime = runtime or load_navnav_runtime()

    def align(self, *, reference, observed):
        adapter = self.runtime.WandaRos2Adapter()
        try:
            adapter.preflight()
            commands = self.runtime.build_base_alignment_commands(
                self.runtime.BasePoint3D(*reference),
                self.runtime.BasePoint3D(*observed),
                adapter.current_torso_position(),
            )
            for command in commands:
                adapter.execute_command(command, precision_mode=True)
            return len(commands)
        finally:
            adapter.stop()
