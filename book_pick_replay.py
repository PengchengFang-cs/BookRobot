"""Stage-1 book Pick replay with one constant visual torso offset."""

from copy import copy
from dataclasses import dataclass
import math
from numbers import Real
from pathlib import Path
import sys
import time
import uuid

import numpy as np


TORSO_MIN_M = 0.0
TORSO_MAX_M = 0.3
V3_ROOT = Path("/home/unix_ai/WHRC")
V3_CONFIG_PATH = V3_ROOT / "v3_pipeline/config/v3_robot_stage1.json"
PICK_ASSET_ID = "S1_TABLE_PICK_BOOK"
PICK_FRAME_COUNT = 607


def _finite_offset(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError("Z offset must be a numeric value in metres")
    offset = float(value)
    if not math.isfinite(offset):
        raise ValueError("Z offset must be finite")
    return offset


def shift_pick_episode_torso(episode, offset_m):
    """Return a shallow episode copy with a private, shifted torso trajectory."""

    offset = _finite_offset(offset_m)
    actions = dict(getattr(episode, "actions", {}))
    if "target_qpos_torso" not in actions:
        raise ValueError("Pick episode has no target_qpos_torso")
    torso = np.asarray(actions["target_qpos_torso"], dtype=float)
    frame_count = int(getattr(episode, "num_frames", -1))
    if torso.shape != (frame_count, 1):
        raise ValueError("Pick torso target frame count or shape is invalid")
    shifted_torso = np.array(torso, dtype=float, copy=True) + offset
    if not np.isfinite(shifted_torso).all():
        raise ValueError("shifted torso trajectory is non-finite")
    if np.any(shifted_torso < TORSO_MIN_M) or np.any(shifted_torso > TORSO_MAX_M):
        raise ValueError("shifted torso trajectory exceeds [0.0, 0.3] m")

    actions["target_qpos_torso"] = shifted_torso
    observations = dict(getattr(episode, "observations", {}))
    if "target_qpos_torso" in observations:
        observations["target_qpos_torso"] = np.array(shifted_torso, copy=True)

    shifted = copy(episode)
    shifted.actions = actions
    shifted.observations = observations
    return shifted


@dataclass(frozen=True)
class BookPickReplayResult:
    frames_sent: int
    z_offset_m: float
    torso_target_m: float
    torso_actual_m: float
    d01_holding: bool


@dataclass(frozen=True)
class ReplayPublishEvidence:
    frames_sent: int


def _success(result):
    status = getattr(result, "status", None)
    return getattr(status, "value", status) == "SUCCESS"


class LegacyV3PickRuntime:
    """Narrow bridge to the reviewed V3 Pick publisher and D01 schedule."""

    def __init__(
        self,
        *,
        replay_adapter,
        context,
        pick_block,
        check_block,
        nav_runtime,
        monotonic_clock=time.monotonic,
    ):
        self.replay_adapter = replay_adapter
        self.context = context
        self.pick_block = pick_block
        self.check_block = check_block
        self.nav_runtime = nav_runtime
        self.monotonic_clock = monotonic_clock
        self.entry = replay_adapter.assets[PICK_ASSET_ID]
        self.module = None
        self._closed = False

    def load_episode(self):
        module = self.replay_adapter._load_module()
        episode = module.HDF5EpisodeLoader.load(str(self.entry["file"]))
        if int(getattr(episode, "num_frames", -1)) != PICK_FRAME_COUNT:
            raise RuntimeError(
                f"Stage-1 Pick asset must contain exactly {PICK_FRAME_COUNT} frames"
            )
        validation = module.DataValidator.validate(episode)
        if getattr(validation, "valid", False) is not True:
            if self.entry.get("contract_validation_only") is not True:
                raise RuntimeError("Stage-1 Pick HDF5 validation failed")
            errors = self.replay_adapter._contract_validation_errors(
                module, episode, self.entry
            )
            if errors:
                raise RuntimeError(
                    "Stage-1 Pick contract validation failed: " + "; ".join(errors)
                )
        contract_error = self.replay_adapter._episode_contract_error(
            episode, self.entry
        )
        if contract_error:
            raise RuntimeError(contract_error)
        self.module = module
        return episode

    def preposition_torso(self, target_m):
        adapter = self.nav_runtime.WandaRos2Adapter()
        try:
            adapter.preflight()
            command = self.nav_runtime.MappedMotionCommand(
                self.nav_runtime.WandaCommandKind.TORSO_POSITION,
                float(target_m),
                "Z",
            )
            adapter.execute_command(command, precision_mode=True)
            return float(adapter.current_torso_position())
        finally:
            try:
                adapter.stop()
            finally:
                adapter.destroy_node()

    def replay_pick(self, episode):
        if self.entry.get("allow_base_motion") is not False:
            raise RuntimeError("Stage-1 Pick asset unexpectedly allows base motion")
        if self.module is None:
            raise RuntimeError("Stage-1 Pick episode must be loaded before replay")
        before = int(self.replay_adapter.frames_sent)
        deadline = self.monotonic_clock() + float(self.entry["timeout_seconds"])
        result = self.replay_adapter._run_episode(
            self.module,
            episode,
            self.entry,
            PICK_ASSET_ID,
            self.context,
            deadline,
            before,
        )
        if not _success(result):
            raise RuntimeError(
                "Stage-1 Pick DataReplay failed: "
                + str(getattr(result, "detail", "unknown error"))
            )
        frames = int(getattr(result, "data", {}).get(
            "frames_sent", self.replay_adapter.frames_sent - before
        ))
        actual_frames = int(self.replay_adapter.frames_sent) - before
        if frames != PICK_FRAME_COUNT or actual_frames != PICK_FRAME_COUNT:
            raise RuntimeError(
                f"Stage-1 Pick must publish all {PICK_FRAME_COUNT} frames; "
                f"reported={frames}, actual={actual_frames}"
            )
        return ReplayPublishEvidence(frames_sent=frames)

    def confirm_holding(self):
        result = self.replay_adapter.delegate.check(
            self.check_block, self.context
        )
        if not _success(result):
            raise RuntimeError(
                "Stage-1 Pick D01 check failed: "
                + str(getattr(result, "detail", "unknown error"))
            )
        return True

    def close(self):
        if not self._closed:
            self.replay_adapter.stop_command_stream("FPC table Pick finished")
            self._closed = True


def load_legacy_v3_pick_runtime(
    *, root=V3_ROOT, config_path=V3_CONFIG_PATH, nav_runtime=None
):
    """Load only the old V3 components needed for one reviewed table Pick."""

    root = Path(root)
    config_path = Path(config_path)
    root_text = str(root)
    if root_text not in sys.path:
        sys.path.insert(0, root_text)

    from v3_pipeline.config import build_adapter, load_config
    from v3_pipeline.contracts import LoadState, RunContext
    from v3_pipeline.plan import build_stage1

    if nav_runtime is None:
        from book_navigation import load_navnav_runtime

        nav_runtime = load_navnav_runtime()

    config = load_config(str(config_path))
    replay_adapter = build_adapter(config)
    blocks = {block.step_id: block for block in build_stage1()}
    return LegacyV3PickRuntime(
        replay_adapter=replay_adapter,
        context=RunContext(
            run_id="fpc-book-pick-" + uuid.uuid4().hex,
            current_stage=1,
            load_state=LoadState.EMPTY_READY,
        ),
        pick_block=blocks["S1-B1-02"],
        check_block=blocks["S1-B1-03"],
        nav_runtime=nav_runtime,
    )


class Stage1BookPickReplayer:
    """Apply visual Z once, replay the Pick episode, and leave the book held."""

    def __init__(self, runtime=None):
        self.runtime = runtime

    def pick(self, z_offset_m):
        offset = _finite_offset(z_offset_m)
        if self.runtime is None:
            self.runtime = load_legacy_v3_pick_runtime()
        try:
            episode = self.runtime.load_episode()
            shifted = shift_pick_episode_torso(episode, offset)
            torso_target = float(shifted.actions["target_qpos_torso"][0, 0])
            torso_actual = float(self.runtime.preposition_torso(torso_target))
            replay = self.runtime.replay_pick(shifted)
            holding = bool(self.runtime.confirm_holding())
            if not holding:
                raise RuntimeError("DataReplay 已完成，但右吸盘没有吸住书本")
            return BookPickReplayResult(
                frames_sent=int(replay.frames_sent),
                z_offset_m=offset,
                torso_target_m=torso_target,
                torso_actual_m=torso_actual,
                d01_holding=holding,
            )
        finally:
            self.runtime.close()
