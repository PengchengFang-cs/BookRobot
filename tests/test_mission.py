import unittest
from types import SimpleNamespace

from mission import run_book_alignment_once, run_book_pick_once, run_one_fruit


class _Vision:
    def __init__(self, results):
        self.results = iter(results)

    def find(self, target, frame):
        return next(self.results)


class _Navigation:
    def __init__(self):
        self.approach_inputs = []
        self.goals = []
        self.home_calls = 0

    def approach_pose(self, point):
        self.approach_inputs.append(point)
        return ("approach", point)

    def go(self, goal):
        self.goals.append(goal)

    def spin(self, angle):
        pass

    def go_home(self):
        self.home_calls += 1


class _Arm:
    def __init__(self):
        self.picks = []

    def pick(self, point):
        self.picks.append(point)

    def drop(self):
        pass


class _AlignmentNavigator:
    def __init__(self):
        self.calls = []

    def align(self, *, reference, observed):
        self.calls.append((reference, observed))
        return SimpleNamespace(
            command_count=2,
            mode="vector",
            odom_dx_m=-0.012,
            odom_dy_m=-0.024,
            imu_dyaw_rad=0.002,
        )


class _PickReplayer:
    def __init__(self):
        self.offsets = []

    def pick(self, offset):
        self.offsets.append(offset)
        return SimpleNamespace(
            frames_sent=607,
            z_offset_m=offset,
            torso_target_m=0.212,
            torso_actual_m=0.211,
            d01_holding=True,
        )


class MissionMultiBookTests(unittest.TestCase):
    def test_legacy_single_pick_uses_first_book_from_list(self):
        map_book = SimpleNamespace(suction_point=(1.0, 0.2, 0.8))
        base_book = SimpleNamespace(suction_point=(0.5, 0.1, 0.8))
        vision = _Vision([[map_book], [base_book]])
        navigation = _Navigation()
        arm = _Arm()

        result = run_one_fruit("book", vision, navigation, arm, say=lambda _: None)

        self.assertTrue(result)
        self.assertEqual(navigation.approach_inputs, [map_book.suction_point])
        self.assertEqual(arm.picks, [base_book.suction_point])

    def test_empty_lists_mean_no_book(self):
        vision = _Vision([[], [], [], []])
        navigation = _Navigation()
        arm = _Arm()

        result = run_one_fruit("book", vision, navigation, arm, say=lambda _: None)

        self.assertFalse(result)
        self.assertEqual(arm.picks, [])
        self.assertEqual(navigation.home_calls, 1)


class BookAlignmentMissionTests(unittest.TestCase):
    def test_detects_aligns_and_measures_again(self):
        initial_far = SimpleNamespace(suction_point=(1.08, 0.31, 0.77))
        initial_near = SimpleNamespace(suction_point=(1.01, -0.33, 0.76))
        final_near = SimpleNamespace(suction_point=(0.915, -0.317, 0.756))
        vision = _Vision([[initial_far, initial_near], [final_near]])
        vision.frames = []
        original_find = vision.find

        def record_find(target, frame):
            vision.frames.append(frame)
            return original_find(target, frame)

        vision.find = record_find
        navigator = _AlignmentNavigator()
        messages = []

        result = run_book_alignment_once(vision, navigator, say=messages.append)

        self.assertEqual(vision.frames, ["base_link", "base_link"])
        self.assertIs(result.initial.book, initial_near)
        self.assertIs(result.final.book, final_near)
        self.assertEqual(result.command_count, 2)
        self.assertEqual(result.navigation_mode, "vector")
        self.assertTrue(result.xy_within_tolerance)
        self.assertAlmostEqual(
            result.z_offset_m,
            initial_near.suction_point[2] - result.initial.reference_m[2],
        )
        self.assertNotAlmostEqual(
            result.z_offset_m,
            final_near.suction_point[2] - result.final.reference_m[2],
        )
        self.assertEqual(
            navigator.calls,
            [(result.initial.reference_m, result.initial.observed_m)],
        )
        self.assertTrue(any("初始偏差" in message for message in messages))
        self.assertTrue(any("最终偏差" in message for message in messages))
        self.assertTrue(any("固定Z偏移=0.005 m" in message for message in messages))
        self.assertTrue(any("odom dx=-0.012 m" in message for message in messages))
        self.assertTrue(any("dy=-0.024 m" in message for message in messages))
        self.assertTrue(any("yaw=0.002 rad" in message for message in messages))
        self.assertTrue(any("XY验收=达标" in message for message in messages))

    def test_rejects_empty_initial_detection(self):
        vision = _Vision([[]])

        with self.assertRaisesRegex(RuntimeError, "没有检测到可对位的书本"):
            run_book_alignment_once(
                vision,
                _AlignmentNavigator(),
                say=lambda _message: None,
            )

    def test_book_pick_aligns_then_replays_once_even_when_xy_report_is_not_ten_mm(self):
        initial = SimpleNamespace(suction_point=(0.920, -0.292, 0.767))
        final = SimpleNamespace(suction_point=(0.904, -0.360, 0.766))
        vision = _Vision([[initial], [final]])
        navigator = _AlignmentNavigator()
        replayer = _PickReplayer()
        messages = []

        result = run_book_pick_once(
            vision, navigator, replayer, say=messages.append
        )

        self.assertFalse(result.alignment.xy_within_tolerance)
        self.assertEqual(replayer.offsets, [result.alignment.z_offset_m])
        self.assertEqual(result.replay.frames_sent, 607)
        self.assertTrue(result.replay.d01_holding)
        self.assertTrue(any("607" in message for message in messages))
        self.assertTrue(any("D01 holding=True" in message for message in messages))


if __name__ == "__main__":
    unittest.main()
