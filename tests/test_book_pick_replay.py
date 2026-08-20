import unittest
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from book_pick_replay import (
    LegacyV3PickRuntime,
    Stage1BookPickReplayer,
    build_frame_zero_preroll,
    shift_pick_episode_torso,
)


def _episode(values):
    torso = np.asarray(values, dtype=float).reshape((-1, 1))
    frames = len(torso)
    return SimpleNamespace(
        num_frames=frames,
        timestamps=np.arange(frames, dtype=float) * 0.05,
        actions={
            "target_qpos_arms": np.zeros((frames, 16)),
            "target_qpos_head": np.zeros((frames, 2)),
            "target_qpos_torso": torso,
            "target_base_vel": np.zeros((frames, 2)),
            "target_qpos_left_gripper": np.zeros((frames, 1)),
            "target_qpos_right_dexhand": np.zeros((frames, 2)),
        },
        observations={
            "target_qpos_torso": torso,
            "qpos_arms": np.zeros((frames, 16)),
            "qpos_head": np.zeros((frames, 2)),
            "qpos_torso": torso.copy(),
        },
    )


class ShiftPickEpisodeTests(unittest.TestCase):
    def test_applies_constant_positive_offset_without_mutating_source(self):
        source = _episode([0.20, 0.21, 0.22])
        original = source.actions["target_qpos_torso"].copy()

        shifted = shift_pick_episode_torso(source, 0.012)

        np.testing.assert_allclose(
            shifted.actions["target_qpos_torso"].reshape(-1),
            [0.212, 0.222, 0.232],
        )
        np.testing.assert_allclose(source.actions["target_qpos_torso"], original)
        self.assertIsNot(shifted, source)
        self.assertIsNot(shifted.actions, source.actions)
        self.assertIsNot(
            shifted.actions["target_qpos_torso"],
            source.actions["target_qpos_torso"],
        )

    def test_keeps_observation_target_consistent_for_zero_and_negative_offsets(self):
        for offset in (0.0, -0.015):
            with self.subTest(offset=offset):
                source = _episode([0.20, 0.22])
                shifted = shift_pick_episode_torso(source, offset)
                expected = np.asarray([0.20 + offset, 0.22 + offset])
                np.testing.assert_allclose(
                    shifted.actions["target_qpos_torso"].reshape(-1), expected
                )
                np.testing.assert_allclose(
                    shifted.observations["target_qpos_torso"].reshape(-1), expected
                )

    def test_rejects_non_finite_boolean_and_physical_range_overflow(self):
        for offset in (True, float("nan"), float("inf"), 0.11, -0.21):
            with self.subTest(offset=offset):
                with self.assertRaises((TypeError, ValueError)):
                    shift_pick_episode_torso(_episode([0.20]), offset)

    def test_requires_one_torso_target_per_frame(self):
        source = _episode([0.20, 0.21])
        source.num_frames = 3

        with self.assertRaisesRegex(ValueError, "frame count"):
            shift_pick_episode_torso(source, 0.01)


class FrameZeroPostureTests(unittest.TestCase):
    def test_builds_bounded_preroll_that_ends_at_frame_zero(self):
        episode = _episode([0.20, 0.20])
        episode.actions["target_qpos_arms"][0] = np.linspace(0.0, 0.15, 16)
        episode.actions["target_qpos_head"][0] = (0.02, 0.25)
        joints = {
            **{f"joint_la{i}": 0.0 for i in range(8)},
            **{f"joint_ra{i}": 0.0 for i in range(8)},
            "joint_head0": 0.0,
            "joint_head1": 0.20,
            "body_joint": 0.20,
        }

        preroll = build_frame_zero_preroll(joints, episode)

        self.assertGreater(preroll.num_frames, 1)
        np.testing.assert_allclose(
            preroll.actions["target_qpos_arms"][-1],
            episode.actions["target_qpos_arms"][0],
        )
        np.testing.assert_allclose(
            preroll.actions["target_qpos_head"][-1],
            episode.actions["target_qpos_head"][0],
        )
        np.testing.assert_allclose(
            preroll.actions["target_qpos_torso"][-1],
            episode.actions["target_qpos_torso"][0],
        )
        self.assertLessEqual(
            np.max(np.abs(np.diff(preroll.actions["target_qpos_arms"], axis=0))),
            0.0100001,
        )
        self.assertTrue(np.all(preroll.actions["target_base_vel"] == 0.0))


class RealPickAssetContractTests(unittest.TestCase):
    ASSET = Path(
        "/home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/"
        "20260819_libraryrobot_datareplay/"
        "pi05_wanda_dr1.2_20260819_195231.h5"
    )

    @unittest.skipUnless(ASSET.is_file(), "read-only robot Pick asset unavailable")
    def test_real_pick_has_all_333_recorded_control_channels(self):
        import h5py

        with h5py.File(self.ASSET, "r") as handle:
            actions = handle["observations"]
            expected = {
                "target_qpos_arms": (333, 16),
                "target_qpos_head": (333, 2),
                "target_qpos_torso": (333, 1),
                "target_base_vel": (333, 2),
                "target_qpos_left_gripper": (333, 1),
                "target_qpos_right_dexhand": (333, 2),
            }
            for channel, shape in expected.items():
                self.assertEqual(actions[channel].shape, shape)
            self.assertGreater(
                float(np.max(np.abs(actions["target_base_vel"][:]))), 0.0
            )

class _Runtime:
    def __init__(self, *, holding=True, fail_replay=False):
        self.source = _episode([0.20, 0.21, 0.22])
        self.holding = holding
        self.fail_replay = fail_replay
        self.calls = []
        self.closed = False
        self.cleaned_up = False
        self.replayed_episode = None
        self.prepared_episode = None
        self.frame_zero_torso_actual_m = None

    def load_episode(self):
        self.calls.append("load")
        return self.source

    def replay_pick(self, episode):
        self.calls.append("replay")
        self.replayed_episode = episode
        self.frame_zero_torso_actual_m = (
            float(episode.actions["target_qpos_torso"][0, 0]) - 0.001
        )
        if self.fail_replay:
            raise RuntimeError("replay failed")
        return SimpleNamespace(frames_sent=episode.num_frames)

    def prepare_frame_zero(self, episode):
        self.calls.append("prepare")
        self.prepared_episode = episode

    def confirm_holding(self):
        self.calls.append("holding")
        return self.holding

    def cleanup_failed_pick(self):
        self.calls.append("cleanup")
        self.cleaned_up = True

    def close(self):
        self.calls.append("close")
        self.closed = True


class Stage1BookPickReplayerTests(unittest.TestCase):
    def test_new_loop_restores_recorded_frame_zero_before_vision(self):
        runtime = _Runtime()

        Stage1BookPickReplayer(runtime=runtime).prepare()

        self.assertEqual(runtime.calls, ["load", "prepare"])
        self.assertIs(runtime.prepared_episode, runtime.source)

    def test_prepositions_shifted_frame_zero_replays_once_and_confirms_holding(self):
        runtime = _Runtime()

        result = Stage1BookPickReplayer(runtime=runtime).pick(0.012)

        self.assertEqual(runtime.calls[0], "load")
        self.assertEqual(runtime.calls[1:], ["replay", "holding", "close"])
        self.assertEqual(result.frames_sent, 3)
        self.assertAlmostEqual(result.z_offset_m, 0.012)
        self.assertAlmostEqual(result.torso_target_m, 0.212)
        self.assertAlmostEqual(result.torso_actual_m, 0.211)
        self.assertTrue(result.d01_holding)
        np.testing.assert_allclose(
            runtime.replayed_episode.actions["target_qpos_torso"].reshape(-1),
            [0.212, 0.222, 0.232],
        )

    def test_rejects_missing_attachment_after_replay(self):
        runtime = _Runtime(holding=False)

        with self.assertRaisesRegex(RuntimeError, "没有吸住"):
            Stage1BookPickReplayer(runtime=runtime).pick(0.0)

        self.assertTrue(runtime.cleaned_up)
        self.assertEqual(runtime.calls[-2:], ["cleanup", "close"])
        self.assertTrue(runtime.closed)

    def test_closes_runtime_when_replay_fails(self):
        runtime = _Runtime(fail_replay=True)

        with self.assertRaisesRegex(RuntimeError, "replay failed"):
            Stage1BookPickReplayer(runtime=runtime).pick(0.0)

        self.assertTrue(runtime.cleaned_up)
        self.assertEqual(runtime.calls[-2:], ["cleanup", "close"])
        self.assertTrue(runtime.closed)


class _Status:
    def __init__(self, value):
        self.value = value


class _ReplayAdapter:
    def __init__(self, episode, reported_frames=None):
        self.episode = episode
        self.reported_frames = reported_frames
        self.assets = {
            "S1_TABLE_PICK_BOOK": {
                "file": "/readonly/pick.h5",
                "timeout_seconds": 140.0,
                "allow_base_motion": False,
                "contract_validation_only": True,
                "required_action_channels": [
                    "target_qpos_arms",
                    "target_qpos_head",
                    "target_qpos_torso",
                ],
                "d01_events": [
                    {"frame_index": 300, "command": "right_suction_start"}
                ],
            }
        }
        self.frames_sent = 0
        self._fpc_block_result = _BlockResult
        self.run_calls = []
        self.closed = False
        self.d01_events = []
        self.navigation_starts = 0
        self.original_track_calls = 0
        navigation = SimpleNamespace(start_task=self._start_navigation)
        self.delegate = SimpleNamespace(
            check=self._check,
            execute_d01_event=self._execute_d01_event,
            _navigation=navigation,
            _navigation_started=False,
            track_replay_completion=self._track_replay_completion,
        )

    def _start_navigation(self):
        self.navigation_starts += 1
        return SimpleNamespace(event_type="TASK_STARTED")

    def _track_replay_completion(self, *, done_event, context):
        self.original_track_calls += 1
        return SimpleNamespace(
            status=_Status("SUCCESS"), data={"navigation_event": "legacy"}
        )

    def _load_module(self):
        return SimpleNamespace(
            HDF5EpisodeLoader=SimpleNamespace(load=lambda _path: self.episode),
            DataValidator=SimpleNamespace(
                validate=lambda _episode: SimpleNamespace(valid=True)
            ),
        )

    def _episode_contract_error(self, _episode, _entry):
        return ""

    def _contract_validation_errors(self, _module, _episode, _entry):
        return []

    def _publish_frames(self, *_args):
        raise AssertionError("fake _run_episode does not publish frames")

    def _run_episode(
        self, _module, episode, entry, asset_id, context, _deadline, before
    ):
        self.run_calls.append((episode, entry, asset_id, context, before))
        if entry["allow_base_motion"]:
            done = threading.Event()
            done.set()
            self.track_result = self.delegate.track_replay_completion(
                done_event=done, context=context
            )
        frames = (
            episode.num_frames
            if self.reported_frames is None
            else self.reported_frames
        )
        self.frames_sent += frames
        return SimpleNamespace(
            status=_Status("SUCCESS"),
            detail="published",
            data={"frames_sent": frames},
        )

    def _check(self, block, context):
        self.check_call = (block, context)
        return SimpleNamespace(status=_Status("SUCCESS"), detail="holding")

    def _execute_d01_event(self, *, event, context):
        self.d01_events.append((event, context))
        return SimpleNamespace(status=_Status("SUCCESS"), detail="stopped")

    def stop_command_stream(self, _reason):
        self.closed = True


class _TorsoAdapter:
    def __init__(self):
        self.commands = []
        self.stopped = False
        self.destroyed = False

    def preflight(self):
        pass

    def execute_command(self, command, *, precision_mode):
        self.commands.append((command, precision_mode))

    def current_torso_position(self):
        return 0.211

    def stop(self):
        self.stopped = True

    def destroy_node(self):
        self.destroyed = True


class _NavRuntime:
    class WandaCommandKind:
        TORSO_POSITION = "torso"

    def __init__(self):
        self.adapter = _TorsoAdapter()

    def WandaRos2Adapter(self):
        return self.adapter

    @staticmethod
    def MappedMotionCommand(kind, value, axis):
        return SimpleNamespace(kind=kind, value=value, axis=axis)


class _Publisher:
    def get_subscription_count(self):
        return 1


class _ReplayNode:
    def __init__(self, episode, events):
        self.episode = episode
        self.events = events
        self.pub_arms = _Publisher()
        self.pub_gripper = _Publisher()
        self.pub_dexhand = _Publisher()
        self.pub_head = _Publisher()
        self.pub_torso = _Publisher()
        self.pub_base = _Publisher()

    def _publish_frame(self, frame):
        self.events.append(
            (
                "preroll",
                frame,
                tuple(self.episode.actions["target_base_vel"][frame]),
            )
        )


class _BlockResult:
    @classmethod
    def success(cls, detail, **data):
        return SimpleNamespace(
            status=_Status("SUCCESS"), detail=detail, data=data
        )

    @classmethod
    def fail(cls, detail, **data):
        return SimpleNamespace(status=_Status("FAIL"), detail=detail, data=data)


class _ExactPublishAdapter:
    def __init__(self, events, *, d01_started=None, release_d01=None):
        self._clock = lambda: 10.0
        self._abort = threading.Event()
        self._frames_sent = 0
        self._fpc_block_result = _BlockResult
        self.events = events
        self.d01_started = d01_started
        self.release_d01 = release_d01
        self.delegate = SimpleNamespace(execute_d01_event=self._d01)

    def _d01(self, *, event, context):
        self.events.append(("d01_start", event["command"]))
        if self.d01_started is not None:
            self.d01_started.set()
        if self.release_d01 is not None:
            self.release_d01.wait(timeout=1.0)
        self.events.append(("d01_done", event["command"]))
        return SimpleNamespace(
            status=_Status("SUCCESS"),
            data={
                "fresh_ack": True,
                "fault": False,
                "acknowledged_command": event["command"],
            },
        )

    def _bounded_wait(self, _duration, _deadline, _label):
        return ""


class _RecordedNode:
    def __init__(self, episode, events):
        self.episode = episode
        self.events = events

    def _publish_frame(self, frame):
        self.events.append(("publish", frame))


class LegacyV3PickRuntimeTests(unittest.TestCase):
    def _runtime(self, frame_count=333, reported_frames=None):
        episode = _episode([0.212] * frame_count)
        replay_adapter = _ReplayAdapter(episode, reported_frames=reported_frames)
        context = SimpleNamespace(load_state=SimpleNamespace(value="EMPTY_READY"))
        pick_block = SimpleNamespace(step_id="S1-B1-02")
        check_block = SimpleNamespace(step_id="S1-B1-03")
        nav_runtime = _NavRuntime()
        runtime = LegacyV3PickRuntime(
            replay_adapter=replay_adapter,
            context=context,
            pick_block=pick_block,
            check_block=check_block,
            nav_runtime=nav_runtime,
            monotonic_clock=lambda: 10.0,
        )
        return runtime, replay_adapter, nav_runtime, episode

    def test_loads_reviewed_pick_episode_and_prepositions_frame_zero(self):
        runtime, _replay, nav, episode = self._runtime()

        loaded = runtime.load_episode()
        actual = runtime.preposition_torso(0.212)

        self.assertIs(loaded, episode)
        command, precision = nav.adapter.commands[0]
        self.assertEqual(command.kind, nav.WandaCommandKind.TORSO_POSITION)
        self.assertAlmostEqual(command.value, 0.212)
        self.assertEqual(command.axis, "Z")
        self.assertTrue(precision)
        self.assertAlmostEqual(actual, 0.211)
        self.assertTrue(nav.adapter.stopped)
        self.assertTrue(nav.adapter.destroyed)

    def test_missing_replaced_right_dexhand_subscriber_does_not_block_replay(self):
        times = iter((0.0, 0.0, 6.0))
        runtime, _replay, _nav, episode = self._runtime()
        runtime.monotonic_clock = lambda: next(times)
        node = _ReplayNode(episode, [])
        node.pub_dexhand = SimpleNamespace(get_subscription_count=lambda: 0)

        runtime._wait_for_controller_subscribers(node, deadline=10.0)

    def test_loaded_replay_uses_direct_systemctl_without_sudo(self):
        runtime, _replay, _nav, _episode_value = self._runtime()
        runtime.load_episode()
        completed = SimpleNamespace(returncode=0)

        with patch("book_pick_replay.subprocess.run", return_value=completed) as run:
            with patch("book_pick_replay.time.sleep"):
                runtime.module._stop_service("manipulation.service")
                runtime.module._start_service_and_wait(
                    "manipulation.service", timeout=1.0
                )

        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(
            commands[0], ["systemctl", "stop", "manipulation.service"]
        )
        self.assertEqual(
            commands[1], ["systemctl", "start", "manipulation.service"]
        )
        self.assertNotIn("sudo", [part for command in commands for part in command])

    def test_rejects_pick_asset_that_is_not_exactly_333_frames(self):
        runtime, _replay, _nav, _episode_value = self._runtime(frame_count=332)

        with self.assertRaisesRegex(RuntimeError, "333"):
            runtime.load_episode()

    def test_replays_new_pick_with_recorded_base_and_frame_20_d01_event(self):
        runtime, replay, _nav, episode = self._runtime()
        runtime.load_episode()

        evidence = runtime.replay_pick(episode)

        self.assertEqual(evidence.frames_sent, 333)
        self.assertEqual(len(replay.run_calls), 1)
        sent_episode, entry, asset_id, _context, before = replay.run_calls[0]
        self.assertIs(sent_episode, episode)
        self.assertEqual(asset_id, "S1_TABLE_PICK_BOOK")
        self.assertEqual(
            entry["file"],
            "/home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/"
            "20260819_libraryrobot_datareplay/"
            "pi05_wanda_dr1.2_20260819_195231.h5",
        )
        self.assertTrue(entry["allow_base_motion"])
        self.assertIn("target_base_vel", entry["required_action_channels"])
        self.assertEqual(
            entry["d01_events"],
            [{"frame_index": 20, "command": "right_suction_start"}],
        )
        self.assertEqual(before, 0)
        self.assertEqual(replay.navigation_starts, 0)
        self.assertFalse(replay.delegate._navigation_started)
        self.assertEqual(replay.original_track_calls, 0)
        self.assertEqual(replay.track_result.status.value, "SUCCESS")

    def test_exact_publish_prepositions_all_frame_zero_joints_before_recording(self):
        runtime, _replay, _nav, episode = self._runtime()
        target_arms = np.linspace(0.0, 0.15, 16)
        episode.actions["target_qpos_arms"][0] = target_arms
        episode.actions["target_qpos_head"][0] = (0.02, 0.25)
        feedback = [
            {
                **{f"joint_la{i}": 0.0 for i in range(8)},
                **{f"joint_ra{i}": 0.0 for i in range(8)},
                "joint_head0": 0.0,
                "joint_head1": 0.20,
                "body_joint": 0.212,
            },
            {
                **{f"joint_la{i}": 0.0 for i in range(8)},
                **{f"joint_ra{i}": 0.0 for i in range(8)},
                "joint_head0": 0.0,
                "joint_head1": 0.20,
                "body_joint": 0.212,
            },
            {
                **{
                    f"joint_la{i}": float(target_arms[i])
                    for i in range(8)
                },
                **{
                    f"joint_ra{i}": float(target_arms[i + 8])
                    for i in range(8)
                },
                "joint_head0": 0.02,
                "joint_head1": 0.25,
                "body_joint": 0.212,
            },
        ]
        runtime.joint_positions = lambda: feedback.pop(0)
        runtime.spin_feedback = lambda: True
        events = []
        node = _ReplayNode(episode, events)
        module = SimpleNamespace(
            rclpy=SimpleNamespace(spin_once=lambda *_args, **_kwargs: None)
        )

        def original(_module, original_node, original_episode, entry, *_args):
            self.assertIs(original_node.episode, original_episode)
            events.append(("recording", entry["d01_events"]))
            return "published"

        runtime._publish_recorded_frames = (
            lambda module, original_node, original_episode, entry, *args: original(
                module, original_node, original_episode, entry, *args
            )
        )

        result = runtime._publish_exact_frames(
            module,
            node,
            episode,
            runtime.entry,
            runtime.context,
            20.0,
            0,
        )

        self.assertEqual(result, "published")
        self.assertEqual(events[-1][0], "recording")
        self.assertTrue(all(event[2] == (0.0, 0.0) for event in events[:-1]))
        self.assertEqual(
            events[-1][1],
            [{"frame_index": 20, "command": "right_suction_start"}],
        )

    def test_stale_frame_zero_feedback_never_enters_recorded_frames(self):
        runtime, _replay, _nav, episode = self._runtime()
        target_arms = episode.actions["target_qpos_arms"][0]
        target_head = episode.actions["target_qpos_head"][0]
        cached_joints = {
            **{
                f"joint_la{i}": float(target_arms[i])
                for i in range(8)
            },
            **{
                f"joint_ra{i}": float(target_arms[i + 8])
                for i in range(8)
            },
            "joint_head0": float(target_head[0]),
            "joint_head1": float(target_head[1]),
            "body_joint": float(episode.actions["target_qpos_torso"][0, 0]),
        }
        runtime.joint_positions = lambda: dict(cached_joints)
        runtime.spin_feedback = lambda: False
        runtime._publish_recorded_frames = (
            lambda *_args, **_kwargs: self.fail(
                "stale feedback must not enter recorded frames"
            )
        )
        node = _ReplayNode(episode, [])
        module = SimpleNamespace(
            rclpy=SimpleNamespace(spin_once=lambda *_args, **_kwargs: None)
        )

        with self.assertRaisesRegex(RuntimeError, "fresh body_joint"):
            runtime._publish_exact_frames(
                module,
                node,
                episode,
                runtime.entry,
                runtime.context,
                20.0,
                0,
            )

    def test_recorded_frame_is_published_before_its_d01_event_without_delay(self):
        runtime, _replay, _nav, _episode_value = self._runtime()
        episode = _episode([0.20, 0.20])
        events = []
        adapter = _ExactPublishAdapter(events)
        runtime.replay_adapter = adapter
        node = _RecordedNode(episode, events)
        module = SimpleNamespace(
            rclpy=SimpleNamespace(spin_once=lambda *_args, **_kwargs: None)
        )
        entry = {
            "speed": 1.0,
            "d01_events": [
                {"frame_index": 0, "command": "right_suction_start"}
            ],
        }

        result = runtime._publish_recorded_frames(
            module, node, episode, entry, runtime.context, 20.0, 0
        )

        self.assertEqual(
            events,
            [
                ("publish", 0),
                ("d01_start", "right_suction_start"),
                ("d01_done", "right_suction_start"),
                ("publish", 1),
            ],
        )
        self.assertEqual(result.status.value, "SUCCESS")
        self.assertEqual(result.data["frames_sent"], 2)
        self.assertEqual(runtime.d01_state, "holding_or_unknown")

    def test_early_d01_start_only_waits_for_command_ack(self):
        runtime, _replay, _nav, _episode_value = self._runtime()
        episode = _episode([0.20, 0.20])
        events = []
        adapter = _ExactPublishAdapter(events)
        adapter.delegate.d01_host = "127.0.0.1"
        runtime.replay_adapter = adapter
        runtime._start_d01_without_waiting = lambda: events.append(
            ("d01_command_ack", "right_suction_start")
        )
        node = _RecordedNode(episode, events)
        module = SimpleNamespace(
            rclpy=SimpleNamespace(spin_once=lambda *_args, **_kwargs: None)
        )
        entry = {
            "speed": 1.0,
            "d01_events": [
                {"frame_index": 0, "command": "right_suction_start"}
            ],
        }

        result = runtime._publish_recorded_frames(
            module, node, episode, entry, runtime.context, 20.0, 0
        )

        self.assertEqual(
            events,
            [
                ("publish", 0),
                ("d01_command_ack", "right_suction_start"),
                ("publish", 1),
            ],
        )
        self.assertEqual(result.status.value, "SUCCESS")

    def test_d01_holding_wait_does_not_delay_the_next_recorded_frame(self):
        runtime, _replay, _nav, _episode_value = self._runtime()
        episode = _episode([0.20, 0.20])
        events = []
        started = threading.Event()
        release = threading.Event()
        adapter = _ExactPublishAdapter(
            events, d01_started=started, release_d01=release
        )
        runtime.replay_adapter = adapter
        node = _RecordedNode(episode, events)
        module = SimpleNamespace(
            rclpy=SimpleNamespace(spin_once=lambda *_args, **_kwargs: None)
        )
        entry = {
            "speed": 1.0,
            "d01_events": [
                {"frame_index": 0, "command": "right_suction_start"}
            ],
        }
        outcome = []
        worker = threading.Thread(
            target=lambda: outcome.append(
                runtime._publish_recorded_frames(
                    module, node, episode, entry, runtime.context, 20.0, 0
                )
            )
        )
        worker.start()
        self.assertTrue(started.wait(timeout=0.5))
        try:
            deadline = time.monotonic() + 0.5
            while (
                ("publish", 1) not in events
                and time.monotonic() < deadline
            ):
                time.sleep(0.001)
            self.assertIn(("publish", 1), events)
            self.assertNotIn(("d01_done", "right_suction_start"), events)
        finally:
            release.set()
            worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(outcome[0].status.value, "SUCCESS")

    def test_rejects_missing_or_malformed_exact_replay_channels(self):
        for channel, shape in (
            ("target_qpos_arms", (333, 15)),
            ("target_qpos_head", (333, 1)),
            ("target_qpos_torso", (333, 2)),
            ("target_base_vel", (333, 1)),
        ):
            with self.subTest(channel=channel):
                runtime, _replay, _nav, episode = self._runtime()
                episode.actions[channel] = np.zeros(shape)
                with self.assertRaisesRegex(RuntimeError, channel):
                    runtime.load_episode()

    def test_nonzero_recorded_base_trajectory_cannot_be_disabled(self):
        runtime, replay, _nav, episode = self._runtime()
        episode.actions["target_base_vel"][10, 0] = 0.1
        runtime.load_episode()

        self.assertTrue(runtime.entry["allow_base_motion"])
        self.assertIn("target_base_vel", runtime.entry["required_action_channels"])

    def test_rejects_success_result_that_did_not_publish_all_333_frames(self):
        runtime, _replay, _nav, episode = self._runtime(reported_frames=332)
        runtime.load_episode()

        with self.assertRaisesRegex(RuntimeError, "333"):
            runtime.replay_pick(episode)

    def test_confirms_pick_check_and_closes_command_stream(self):
        runtime, replay, _nav, _episode_value = self._runtime()

        self.assertTrue(runtime.confirm_holding())
        self.assertEqual(replay.check_call[0].step_id, "S1-B1-03")
        runtime.close()
        self.assertTrue(replay.closed)

    def test_failed_pick_cleanup_never_stops_a_holding_or_unknown_book(self):
        runtime, replay, _nav, _episode_value = self._runtime()
        runtime._d01_state = "holding_or_unknown"

        runtime.cleanup_failed_pick()

        self.assertEqual(replay.d01_events, [])

if __name__ == "__main__":
    unittest.main()
