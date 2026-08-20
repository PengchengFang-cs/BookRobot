import unittest
from types import SimpleNamespace

import numpy as np

from book_place_replay import (
    PLACE_ASSET_PATH,
    PLACE_D01_STOP_FRAME_INDEX,
    PLACE_FRAME_COUNT,
    Stage1BookPlaceReplayer,
)
from book_pick_replay import LegacyV3PickRuntime


class _Runtime:
    def __init__(self, *, released=True):
        self.episode = SimpleNamespace(
            num_frames=PLACE_FRAME_COUNT,
            actions={"target_qpos_torso": np.full((PLACE_FRAME_COUNT, 1), 0.2)},
        )
        self.released = released
        self.calls = []
        self.frame_zero_torso_actual_m = 0.2

    def load_episode(self):
        self.calls.append("load")
        return self.episode

    def replay_pick(self, episode):
        self.calls.append("replay")
        self.asserted_episode = episode
        return SimpleNamespace(frames_sent=PLACE_FRAME_COUNT)

    def confirm_released(self):
        self.calls.append("released")
        return self.released

    def close(self):
        self.calls.append("close")


class _CommandClient:
    calls = []

    def __init__(self, host, port, timeout_s):
        self.args = (host, port, timeout_s)

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return None

    def command(self, action, **fields):
        self.calls.append((self.args, action, fields))
        return {"ok": True, "action": action}


SuctionClient = _CommandClient


class _Delegate:
    d01_host = "127.0.0.1"
    d01_port = 8765
    d01_side = "right"

    def execute_d01_event(self, **_kwargs):
        raise AssertionError("direct event callback must not be used")


class BookPlaceReplayTests(unittest.TestCase):
    def setUp(self):
        _CommandClient.calls.clear()

    def test_uses_new_dr24_asset_and_frame_190_stop(self):
        self.assertIn("pi05_wanda_dr2.4_20260819_202457.h5", str(PLACE_ASSET_PATH))
        self.assertEqual(PLACE_FRAME_COUNT, 388)
        self.assertEqual(PLACE_D01_STOP_FRAME_INDEX, 190)

    def test_replays_original_torso_and_confirms_release(self):
        runtime = _Runtime()

        result = Stage1BookPlaceReplayer(runtime=runtime).place()

        self.assertEqual(runtime.calls, ["load", "replay", "released", "close"])
        self.assertIs(runtime.asserted_episode, runtime.episode)
        self.assertEqual(result.frames_sent, 388)
        self.assertEqual(result.torso_target_m, 0.2)
        self.assertTrue(result.d01_released)

    def test_stop_event_disables_suction_without_release_puff(self):
        runtime = LegacyV3PickRuntime.__new__(LegacyV3PickRuntime)
        runtime.replay_adapter = SimpleNamespace(delegate=_Delegate())

        runtime._stop_d01_without_release()

        self.assertEqual(len(_CommandClient.calls), 1)
        _connection, action, fields = _CommandClient.calls[0]
        self.assertEqual(action, "stop")
        self.assertEqual(fields, {
            "side": "right",
            "release_vacuum": False,
            "disable": True,
        })


if __name__ == "__main__":
    unittest.main()
