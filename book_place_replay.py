"""Stage-1 cart Place using the complete recorded DataReplay 2.4 episode."""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import os
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
PLACE_CAPTURE_PRE_BUFFER_COUNT = 12
PLACE_CAPTURE_OFFSETS = tuple(range(-12, -4))


@dataclass(frozen=True)
class PlaceCaptureFrame:
    release_offset: int
    captured_at_ns: int
    path: str


class _PlaceFrameCapture:
    """Save consecutive camera frames before the Place release trigger."""

    def __init__(self, completed_callback=None):
        record_dir = os.environ.get("FPC_EXPERIMENT_RECORD_DIR")
        if not record_dir:
            raise RuntimeError("FPC_EXPERIMENT_RECORD_DIR is required for Place capture")
        self.record_dir = Path(record_dir)
        self.bridge = None
        self.subscription = None
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.futures = []
        self.completed_callback = completed_callback
        self.completed_future = None
        self.pre_release_frames = deque(maxlen=PLACE_CAPTURE_PRE_BUFFER_COUNT)
        self.selected_pre_release_frames = None
        self.release_marked = False

    def start(self, node):
        from cv_bridge import CvBridge
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image

        from config import COLOR_TOPIC

        self.record_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = node.create_subscription(
            Image,
            COLOR_TOPIC,
            self._receive_color,
            qos,
        )

    def _receive_color(self, message):
        stamp = message.header.stamp
        captured = (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
            message,
        )
        if not self.release_marked:
            self.pre_release_frames.append(captured)

    def capture(self, frame_index):
        if frame_index != PLACE_D01_STOP_FRAME_INDEX or self.release_marked:
            return
        self.selected_pre_release_frames = tuple(self.pre_release_frames)
        self.release_marked = True
        selected = tuple(zip(
            PLACE_CAPTURE_OFFSETS,
            self.selected_pre_release_frames[:len(PLACE_CAPTURE_OFFSETS)],
        ))
        captures = []
        for offset, (stamp_ns, message) in selected:
            output_path = self.record_dir / (
                f"place_release_minus_{abs(offset):02d}.jpg"
            )
            captures.append(PlaceCaptureFrame(
                release_offset=offset,
                captured_at_ns=stamp_ns,
                path=str(output_path),
            ))
            self.futures.append((
                offset,
                stamp_ns,
                output_path,
                self.executor.submit(self._write, message, output_path),
            ))
        if self.completed_callback is not None:
            self.completed_future = self.executor.submit(
                self.completed_callback,
                tuple(captures),
            )

    def _write(self, message, output_path):
        import cv2

        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if not cv2.imwrite(str(output_path), image):
            raise RuntimeError(f"failed to write {output_path}")
        return image.shape

    def finish(self, node):
        if not self.release_marked:
            raise RuntimeError("Place release frame was not reached")
        pre_release = self.selected_pre_release_frames or ()
        if len(pre_release) != PLACE_CAPTURE_PRE_BUFFER_COUNT:
            raise RuntimeError(
                "Place pre-release camera frames incomplete: "
                f"expected={PLACE_CAPTURE_PRE_BUFFER_COUNT}, actual={len(pre_release)}"
            )

        self.close(node)
        for offset, stamp_ns, output_path, future in self.futures:
            shape = future.result()
            print(
                f"PLACE_CAPTURE release_offset={offset:+d} "
                f"camera_stamp_ns={stamp_ns} path={output_path} shape={shape}",
                flush=True,
            )
        if self.completed_future is not None:
            self.completed_future.result()

    def close(self, node):
        if self.subscription is not None:
            node.destroy_subscription(self.subscription)
            self.subscription = None
        if self.executor is not None:
            self.executor.shutdown(wait=True)
            self.executor = None


@dataclass(frozen=True)
class BookPlaceReplayResult:
    frames_sent: int
    torso_target_m: float
    torso_actual_m: float
    d01_released: bool


class LegacyV3PlaceRuntime(LegacyV3PickRuntime):
    def _publish_recorded_frames(
        self, module, node, episode, entry, context, deadline, before
    ):
        capture = _PlaceFrameCapture(
            completed_callback=getattr(self, "place_capture_callback", None)
        )
        capture.start(node)
        self.frame_callback = capture.capture
        try:
            result = super()._publish_recorded_frames(
                module, node, episode, entry, context, deadline, before
            )
            capture.finish(node)
            return result
        finally:
            self.frame_callback = None
            capture.close(node)

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

    def __init__(
        self,
        runtime=None,
        *,
        joint_positions=None,
        spin_feedback=None,
        keep_runtime_open=False,
        capture_handler=None,
    ):
        self.runtime = runtime
        self.joint_positions = joint_positions
        self.spin_feedback = spin_feedback
        self.keep_runtime_open = bool(keep_runtime_open)
        self.capture_handler = capture_handler
        self._prepared_episode = None

    def initialize(self):
        if self.runtime is None:
            self.runtime = load_legacy_v3_place_runtime(
                joint_positions=self.joint_positions,
                spin_feedback=self.spin_feedback,
            )
            self.runtime.initialize_module()

    def preload(self):
        if self._prepared_episode is not None:
            return
        self.initialize()
        self._prepared_episode = self.runtime.load_episode()

    def place(self, *, book_index=None):
        self.preload()
        self.runtime.place_capture_callback = (
            None
            if self.capture_handler is None
            else lambda captures: self.capture_handler(book_index, captures)
        )
        try:
            episode = self._prepared_episode
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
            self.runtime.place_capture_callback = None
            if not self.keep_runtime_open:
                self.runtime.close()

    def close(self):
        if self.runtime is not None:
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
