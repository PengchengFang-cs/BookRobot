"""Stage-1 book Pick replay with one constant visual torso offset."""

from copy import copy
from dataclasses import dataclass
import math
from numbers import Real
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
import uuid

import numpy as np


TORSO_MIN_M = 0.0
TORSO_MAX_M = 0.3
V3_ROOT = Path("/home/unix_ai/WHRC")
V3_CONFIG_PATH = V3_ROOT / "v3_pipeline/config/v3_robot_stage1.json"
PICK_ASSET_ID = "S1_TABLE_PICK_BOOK"
PICK_ASSET_PATH = Path(
    "/home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/"
    "20260819_libraryrobot_datareplay/"
    "pi05_wanda_dr1.2_20260819_195231.h5"
)
PICK_FRAME_COUNT = 333
PICK_D01_FRAME_INDEX = 20
EXACT_ACTION_SHAPES = {
    "target_qpos_arms": (16,),
    "target_qpos_head": (2,),
    "target_qpos_torso": (1,),
    "target_base_vel": (2,),
    "target_qpos_left_gripper": (1,),
    "target_qpos_right_dexhand": (2,),
}
ARM_PREROLL_STEP_RAD = 0.01
HEAD_PREROLL_STEP_RAD = 0.01
TORSO_PREROLL_STEP_M = 0.003
PREROLL_PERIOD_S = 0.05
CONTROLLER_DISCOVERY_TIMEOUT_S = 5.0
FRAME_ZERO_ARM_TOLERANCE_RAD = 0.03
FRAME_ZERO_HEAD_TOLERANCE_RAD = 0.02
FRAME_ZERO_TORSO_TOLERANCE_M = 0.005
FRAME_ZERO_SETTLE_SAMPLES = 60


def _joint_vector(joints, names):
    values = np.asarray([joints[name] for name in names], dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("frame-zero feedback contains non-finite joint values")
    return values


def build_frame_zero_preroll(joints, episode):
    """Build a base-stationary interpolation ending at recorded frame zero."""

    arm_names = [f"joint_la{index}" for index in range(8)] + [
        f"joint_ra{index}" for index in range(8)
    ]
    current_arms = _joint_vector(joints, arm_names)
    current_head = _joint_vector(joints, ("joint_head0", "joint_head1"))
    current_torso = _joint_vector(joints, ("body_joint",))
    target_arms = np.asarray(episode.actions["target_qpos_arms"][0], dtype=float)
    target_head = np.asarray(episode.actions["target_qpos_head"][0], dtype=float)
    target_torso = np.asarray(
        episode.actions["target_qpos_torso"][0], dtype=float
    )
    steps = max(
        1,
        math.ceil(float(np.max(np.abs(target_arms - current_arms))) / ARM_PREROLL_STEP_RAD),
        math.ceil(float(np.max(np.abs(target_head - current_head))) / HEAD_PREROLL_STEP_RAD),
        math.ceil(float(np.max(np.abs(target_torso - current_torso))) / TORSO_PREROLL_STEP_M),
    )

    def interpolate(start, target):
        fractions = np.arange(1, steps + 1, dtype=float).reshape((-1, 1)) / steps
        return start + fractions * (target - start)

    actions = {
        "target_qpos_arms": interpolate(current_arms, target_arms),
        "target_qpos_head": interpolate(current_head, target_head),
        "target_qpos_torso": interpolate(current_torso, target_torso),
        "target_base_vel": np.zeros((steps, 2), dtype=float),
    }
    return SimpleNamespace(
        num_frames=steps,
        timestamps=np.arange(steps, dtype=float) * PREROLL_PERIOD_S,
        actions=actions,
        observations={},
    )


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


def _stop_service_without_sudo(name):
    print(f"  Stopping {name} ...")
    subprocess.run(
        ["systemctl", "stop", name],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(1)


def _start_service_without_sudo(name, timeout=20.0):
    print(f"  Starting {name} ...")
    subprocess.run(
        ["systemctl", "start", name],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        active = subprocess.run(
            ["systemctl", "is-active", "--quiet", name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if active.returncode == 0:
            time.sleep(5)
            print(f"  {name} ready.")
            return
        time.sleep(1)
    raise RuntimeError(f"{name} did not become active within {timeout:.1f} s")


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
        wait_function=time.sleep,
        joint_positions=None,
        spin_feedback=None,
    ):
        self.replay_adapter = replay_adapter
        self.context = context
        self.pick_block = pick_block
        self.check_block = check_block
        self.nav_runtime = nav_runtime
        self.monotonic_clock = monotonic_clock
        self.wait_function = wait_function
        self.joint_positions = joint_positions
        self.spin_feedback = spin_feedback or (lambda: None)
        self.entry = dict(replay_adapter.assets[PICK_ASSET_ID])
        self.entry["file"] = str(PICK_ASSET_PATH)
        self.entry["version"] = "stage1-dr1.2-20260819"
        self.entry["d01_events"] = [
            {
                "frame_index": PICK_D01_FRAME_INDEX,
                "command": "right_suction_start",
            }
        ]
        self.entry["allow_base_motion"] = True
        required = list(self.entry.get("required_action_channels", ()))
        for channel in EXACT_ACTION_SHAPES:
            if channel not in required:
                required.append(channel)
        self.entry["required_action_channels"] = required
        self.module = None
        self._closed = False
        self._d01_state = "unattached"
        self.frame_zero_torso_actual_m = None

    @property
    def d01_state(self):
        return self._d01_state

    def load_episode(self):
        module = self.replay_adapter._load_module()
        module._stop_service = _stop_service_without_sudo
        module._start_service_and_wait = _start_service_without_sudo
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
        actions = getattr(episode, "actions", {})
        for channel, trailing_shape in EXACT_ACTION_SHAPES.items():
            value = actions.get(channel)
            expected = (PICK_FRAME_COUNT, *trailing_shape)
            if value is None or np.asarray(value).shape != expected:
                actual = None if value is None else np.asarray(value).shape
                raise RuntimeError(
                    f"{channel} must have exact shape {expected}, got {actual}"
                )
            if not np.isfinite(np.asarray(value, dtype=float)).all():
                raise RuntimeError(f"{channel} contains non-finite values")
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

    def preposition_frame_zero(self, episode):
        """Move the lift to the shifted recording baseline before raw replay."""

        target = float(episode.actions["target_qpos_torso"][0, 0])
        return self.preposition_torso(target)

    def _wait_for_controller_subscribers(self, node, deadline):
        publishers = {
            "arms": node.pub_arms,
            "left_gripper": node.pub_gripper,
            "head": node.pub_head,
            "torso": node.pub_torso,
            "base": node.pub_base,
        }
        discovery_deadline = min(
            float(deadline),
            self.monotonic_clock() + CONTROLLER_DISCOVERY_TIMEOUT_S,
        )
        while self.monotonic_clock() < discovery_deadline:
            missing = [
                name
                for name, publisher in publishers.items()
                if publisher.get_subscription_count() < 1
            ]
            if not missing:
                return
            self.spin_feedback()
            self.wait_function(0.05)
        raise RuntimeError(
            "DataReplay controllers have no subscribers: " + ",".join(missing)
        )

    @staticmethod
    def _feedback_error(joints, episode):
        arm_names = [f"joint_la{index}" for index in range(8)] + [
            f"joint_ra{index}" for index in range(8)
        ]
        actual_arms = _joint_vector(joints, arm_names)
        actual_head = _joint_vector(joints, ("joint_head0", "joint_head1"))
        actual_torso = _joint_vector(joints, ("body_joint",))
        return (
            float(np.max(np.abs(
                actual_arms - episode.actions["target_qpos_arms"][0]
            ))),
            float(np.max(np.abs(
                actual_head - episode.actions["target_qpos_head"][0]
            ))),
            float(np.max(np.abs(
                actual_torso - episode.actions["target_qpos_torso"][0]
            ))),
        )

    def _publish_exact_frames(
        self,
        module,
        node,
        episode,
        entry,
        context,
        deadline,
        before,
    ):
        if self.joint_positions is None:
            raise RuntimeError("frame-zero joint feedback provider is unavailable")
        self._wait_for_controller_subscribers(node, deadline)
        self.spin_feedback()
        preroll = build_frame_zero_preroll(self.joint_positions(), episode)
        original_episode = node.episode
        try:
            node.episode = preroll
            for frame in range(preroll.num_frames):
                if self.monotonic_clock() >= deadline:
                    raise RuntimeError("timeout while restoring frame-zero posture")
                node._publish_frame(frame)
                if hasattr(module.rclpy, "spin_once"):
                    module.rclpy.spin_once(node, timeout_sec=0.0)
                self.spin_feedback()
                if frame + 1 < preroll.num_frames:
                    self.wait_function(PREROLL_PERIOD_S)
        finally:
            node.episode = original_episode

        final_joints = None
        arm_error = head_error = torso_error = math.inf
        for _attempt in range(FRAME_ZERO_SETTLE_SAMPLES):
            if self.monotonic_clock() >= deadline:
                break
            self.spin_feedback()
            final_joints = self.joint_positions()
            arm_error, head_error, torso_error = self._feedback_error(
                final_joints, episode
            )
            if (
                arm_error <= FRAME_ZERO_ARM_TOLERANCE_RAD
                and head_error <= FRAME_ZERO_HEAD_TOLERANCE_RAD
                and torso_error <= FRAME_ZERO_TORSO_TOLERANCE_M
            ):
                break
            self.wait_function(PREROLL_PERIOD_S)
        else:
            final_joints = None
        if final_joints is None or (
            arm_error > FRAME_ZERO_ARM_TOLERANCE_RAD
            or head_error > FRAME_ZERO_HEAD_TOLERANCE_RAD
            or torso_error > FRAME_ZERO_TORSO_TOLERANCE_M
        ):
            raise RuntimeError(
                "frame-zero posture restore failed: "
                f"arms={arm_error:.4f} rad, head={head_error:.4f} rad, "
                f"torso={torso_error:.4f} m"
            )
        self.frame_zero_torso_actual_m = float(final_joints["body_joint"])

        return self._publish_recorded_frames(
            module,
            node,
            episode,
            entry,
            context,
            deadline,
            before,
        )

    def _publish_recorded_frames(
        self, module, node, episode, entry, context, deadline, before
    ):
        """Publish each original frame, then execute events bound to that frame."""

        adapter = self.replay_adapter
        result_class = getattr(adapter, "_fpc_block_result", None)
        if result_class is None:
            result_class = adapter.__class__._publish_frames.__globals__["BlockResult"]

        def fail(detail):
            return result_class.fail(
                detail, frames_sent=adapter._frames_sent - before
            )

        timestamps = episode.timestamps
        speed = float(entry.get("speed", 1.0))
        pending = [dict(event) for event in entry["d01_events"]]
        scheduled = set()
        event_states = {}
        event_lock = threading.Lock()
        origin = float(timestamps[0])

        def run_event(index, event, started):
            started.set()
            try:
                acknowledgement = adapter.delegate.execute_d01_event(
                    event=event, context=context
                )
                data = getattr(acknowledgement, "data", {})
                if (
                    not _success(acknowledgement)
                    or data.get("fresh_ack") is not True
                    or data.get("fault") is not False
                    or data.get("acknowledged_command") != event["command"]
                ):
                    state = "D01 event failed/faulted/stale"
                else:
                    state = "success"
            except Exception as exc:
                state = f"D01 callback exception: {exc}"
            with event_lock:
                event_states[index] = state

        event_threads = {}

        def fail_after_join(detail):
            for worker in event_threads.values():
                remaining = max(0.0, deadline - adapter._clock())
                worker.join(timeout=remaining)
            return fail(detail)

        for frame in range(int(episode.num_frames)):
            frame_started = adapter._clock()
            if adapter._abort.is_set():
                return fail_after_join("DataReplay aborted")
            with event_lock:
                failed_events = [
                    state for state in event_states.values() if state != "success"
                ]
            if failed_events:
                adapter._abort.set()
                return fail_after_join(failed_events[0])
            if adapter._clock() >= deadline:
                return fail_after_join("DataReplay timeout")

            relative = float(timestamps[frame]) - origin
            node._publish_frame(frame)
            adapter._frames_sent += 1
            if hasattr(module.rclpy, "spin_once"):
                module.rclpy.spin_once(node, timeout_sec=0.0)

            for index, event in enumerate(pending):
                due = (
                    event.get("frame_index") == frame
                    or (
                        "time_seconds" in event
                        and float(event["time_seconds"]) <= relative
                    )
                )
                if index in scheduled or not due:
                    continue
                if event["command"] == "right_suction_start":
                    self._d01_state = "holding_or_unknown"
                scheduled.add(index)
                started = threading.Event()
                worker = threading.Thread(
                    target=run_event,
                    args=(index, event, started),
                    name=f"fpc-d01-frame-event-{index}",
                    daemon=True,
                )
                event_threads[index] = worker
                worker.start()
                if not started.wait(timeout=0.01):
                    adapter._abort.set()
                    return fail_after_join("D01 event thread did not start promptly")

            if frame + 1 < int(episode.num_frames):
                interval = max(
                    0.0, float(timestamps[frame + 1] - timestamps[frame])
                ) / speed
                remaining = max(0.0, interval - (adapter._clock() - frame_started))
                wait_error = adapter._bounded_wait(
                    remaining, deadline, "frame pacing"
                )
                if wait_error:
                    return fail_after_join(wait_error)

        if len(scheduled) != len(pending):
            return fail_after_join("not all D01 events executed")
        for index, worker in event_threads.items():
            remaining = max(0.0, deadline - adapter._clock())
            worker.join(timeout=remaining)
            if worker.is_alive():
                adapter._abort.set()
                return fail("D01 event did not finish before replay deadline")
            with event_lock:
                state = event_states.get(index)
            if state != "success":
                adapter._abort.set()
                return fail(state or "D01 event returned no result")
        return result_class.success(
            "DataReplay episode published",
            frames_sent=adapter._frames_sent - before,
            episode_id=str(getattr(episode, "episode_id", "unknown")),
        )

    def replay_pick(self, episode):
        if self.entry.get("allow_base_motion") is not True:
            raise RuntimeError("Stage-1 Pick must replay its recorded base motion")
        if self.module is None:
            raise RuntimeError("Stage-1 Pick episode must be loaded before replay")
        before = int(self.replay_adapter.frames_sent)
        deadline = self.monotonic_clock() + float(self.entry["timeout_seconds"])
        delegate = self.replay_adapter.delegate
        original_publish = self.replay_adapter._publish_frames
        original_tracker = getattr(delegate, "track_replay_completion", None)
        result_class = self.replay_adapter.__class__._publish_frames.__globals__.get(
            "BlockResult"
        )
        if result_class is None:
            result_class = getattr(self.replay_adapter, "_fpc_block_result", None)
        if result_class is None:
            raise RuntimeError("DataReplay result contract is unavailable")

        def completion_tracker(*, done_event, context):
            while not done_event.wait(0.05):
                if self.monotonic_clock() >= deadline:
                    return result_class.fail(
                        "FPC replay completion tracker timed out"
                    )
            return result_class.success(
                "FPC tracked raw replay completion without moving the base",
                navigation_event=None,
            )

        def exact_publish(module, node, replay_episode, entry, context, end, start):
            return self._publish_exact_frames(
                module,
                node,
                replay_episode,
                entry,
                context,
                end,
                start,
            )

        self.replay_adapter._publish_frames = exact_publish
        delegate.track_replay_completion = completion_tracker
        try:
            result = self.replay_adapter._run_episode(
                self.module,
                episode,
                self.entry,
                PICK_ASSET_ID,
                self.context,
                deadline,
                before,
            )
        finally:
            self.replay_adapter._publish_frames = original_publish
            if original_tracker is None:
                delattr(delegate, "track_replay_completion")
            else:
                delegate.track_replay_completion = original_tracker
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
        self._d01_state = "holding"
        return True

    def cleanup_failed_pick(self):
        # A replay failure after suction starts leaves attachment state unknown.
        # Never drop a possibly held book as a side effect of error handling.
        return None

    def close(self):
        if not self._closed:
            self.replay_adapter.stop_command_stream("FPC table Pick finished")
            self._closed = True


def load_legacy_v3_pick_runtime(
    *,
    root=V3_ROOT,
    config_path=V3_CONFIG_PATH,
    nav_runtime=None,
    joint_positions=None,
    spin_feedback=None,
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
        joint_positions=joint_positions,
        spin_feedback=spin_feedback,
    )


class Stage1BookPickReplayer:
    """Apply visual Z once, replay the Pick episode, and leave the book held."""

    def __init__(self, runtime=None, *, joint_positions=None, spin_feedback=None):
        self.runtime = runtime
        self.joint_positions = joint_positions
        self.spin_feedback = spin_feedback

    def pick(self, z_offset_m):
        offset = _finite_offset(z_offset_m)
        if self.runtime is None:
            self.runtime = load_legacy_v3_pick_runtime(
                joint_positions=self.joint_positions,
                spin_feedback=self.spin_feedback,
            )
        replay_started = False
        try:
            episode = self.runtime.load_episode()
            shifted = shift_pick_episode_torso(episode, offset)
            torso_target = float(shifted.actions["target_qpos_torso"][0, 0])
            replay_started = True
            replay = self.runtime.replay_pick(shifted)
            torso_actual = float(self.runtime.frame_zero_torso_actual_m)
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
        except Exception:
            if replay_started:
                try:
                    self.runtime.cleanup_failed_pick()
                except Exception as cleanup_error:
                    print(f"[D01] Pick 失败后的关闭命令也失败: {cleanup_error}")
            raise
        finally:
            self.runtime.close()
