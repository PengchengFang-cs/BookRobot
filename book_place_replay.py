"""Stage-1 cart Place using the complete recorded DataReplay 2.4 episode."""

from dataclasses import dataclass
from pathlib import Path
import sys
import uuid

from book_pick_replay import (
    EXACT_ACTION_SHAPES,
    LegacyV3PickRuntime,
    V3_CONFIG_PATH,
    V3_ROOT,
)


PLACE_ASSET_ID = "S1_CART_PLACE_BOOK"
PLACE_ASSET_PATH = Path(
    "/home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/"
    "20260819_libraryrobot_datareplay/"
    "pi05_wanda_dr2.4_20260819_202457.h5"
)
PLACE_FRAME_COUNT = 388
PLACE_D01_STOP_FRAME_INDEX = 190


@dataclass(frozen=True)
class BookPlaceReplayResult:
    frames_sent: int
    torso_target_m: float
    torso_actual_m: float
    d01_released: bool


class LegacyV3PlaceRuntime(LegacyV3PickRuntime):
    def replay_pick(self, episode, *, restore_frame_zero=True):
        delegate = self.replay_adapter.delegate
        original = delegate.replay_start_check
        result_class = self.replay_adapter.__class__._publish_frames.__globals__.get(
            "BlockResult"
        )
        if result_class is None:
            result_class = getattr(self.replay_adapter, "_fpc_block_result", None)
        if result_class is None:
            raise RuntimeError("DataReplay result contract is unavailable")

        def accept_loaded_start(**_kwargs):
            return result_class.success("Place starts with suction already running")

        delegate.replay_start_check = accept_loaded_start
        try:
            return super().replay_pick(
                episode,
                restore_frame_zero=restore_frame_zero,
            )
        finally:
            delegate.replay_start_check = original

    def confirm_released(self):
        result = self.replay_adapter.delegate.check(
            self.check_block, self.context
        )
        success = getattr(getattr(result, "status", None), "value", None) == "SUCCESS"
        if not success:
            raise RuntimeError(
                "Stage-1 Place D01 check failed: "
                + str(getattr(result, "detail", "unknown error"))
            )
        self._d01_state = "released"
        return True


class Stage1BookPlaceReplayer:
    """Restore Place frame zero, replay 2.4, and leave the book in the cart."""

    def __init__(self, runtime=None, *, joint_positions=None, spin_feedback=None):
        self.runtime = runtime
        self.joint_positions = joint_positions
        self.spin_feedback = spin_feedback

    def place(self):
        if self.runtime is None:
            self.runtime = load_legacy_v3_place_runtime(
                joint_positions=self.joint_positions,
                spin_feedback=self.spin_feedback,
            )
        try:
            episode = self.runtime.load_episode()
            torso_target = float(episode.actions["target_qpos_torso"][0, 0])
            replay = self.runtime.replay_pick(episode)
            torso_actual = float(self.runtime.frame_zero_torso_actual_m)
            released = bool(self.runtime.confirm_released())
            if not released:
                raise RuntimeError("DataReplay 已完成，但右吸盘仍然吸着书本")
            return BookPlaceReplayResult(
                frames_sent=int(replay.frames_sent),
                torso_target_m=torso_target,
                torso_actual_m=torso_actual,
                d01_released=released,
            )
        finally:
            self.runtime.close()


def load_legacy_v3_place_runtime(
    *,
    root=V3_ROOT,
    config_path=V3_CONFIG_PATH,
    nav_runtime=None,
    joint_positions=None,
    spin_feedback=None,
):
    """Load only the old V3 pieces needed for one cart Place replay."""

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
    runtime = LegacyV3PlaceRuntime(
        replay_adapter=replay_adapter,
        context=RunContext(
            run_id="fpc-book-place-" + uuid.uuid4().hex,
            current_stage=1,
            load_state=LoadState.LOADED_CARRY,
        ),
        pick_block=blocks["S1-B1-05"],
        check_block=blocks["S1-B1-06"],
        nav_runtime=nav_runtime,
        joint_positions=joint_positions,
        spin_feedback=spin_feedback,
        asset_id=PLACE_ASSET_ID,
        asset_path=PLACE_ASSET_PATH,
        frame_count=PLACE_FRAME_COUNT,
        d01_events=[{
            "frame_index": PLACE_D01_STOP_FRAME_INDEX,
            "command": "right_suction_stop",
        }],
        asset_version="stage1-dr2.4-20260819",
        operation_label="Stage-1 Place",
    )
    required = list(runtime.entry.get("required_action_channels", ()))
    for channel in EXACT_ACTION_SHAPES:
        if channel not in required:
            required.append(channel)
    runtime.entry["required_action_channels"] = required
    runtime.entry["speed"] = 1.0
    runtime.entry["allow_base_motion"] = True
    return runtime
