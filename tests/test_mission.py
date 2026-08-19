import unittest
from types import SimpleNamespace

from book_geometry import BookGeometry
from mission import (
    run_book_alignment_from_current_once,
    run_book_alignment_once,
    run_book_pick_once,
    run_one_fruit,
)
from replay_pick_reference import ReplayPickReference


class _Vision:
    def __init__(self, results):
        self.results = iter(results)
        self.frames = []

    def find(self, target, frame):
        self.frames.append(frame)
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
    def __init__(self, results):
        self.calls = []
        self.results = iter(results)

    def align(self, *, reference, observed):
        self.calls.append((reference, observed))
        return next(self.results)


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


def _nav_result(dx, dy, yaw=0.0):
    return SimpleNamespace(
        command_count=2,
        mode="vector",
        odom_dx_m=dx,
        odom_dy_m=dy,
        imu_dyaw_rad=yaw,
    )


def _book(point):
    geometry = BookGeometry(
        suction_point=point,
        long_axis=(1.0, 0.0, 0.0),
        short_axis_right_to_left=(0.0, 1.0, 0.0),
        long_extent_m=0.30,
        short_extent_m=0.20,
        confidence=0.9,
        long_inset_m=0.13,
        right_inset_m=0.10,
    )
    return SimpleNamespace(suction_point=point, geometry=geometry)


def _reference():
    return ReplayPickReference(
        asset_id="S1_TABLE_PICK_BOOK",
        reference_frame_index=0,
        recorded_book_suction_point_base_m=(0.715, -0.391, 0.713),
        recorded_book_long_axis_base=(1.0, 0.0, 0.0),
        recorded_book_long_extent_m=0.30,
        recorded_book_short_extent_m=0.20,
        hdf5_sha256="a" * 64,
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
    def _scenario(self):
        initial_target = _book((0.90, -0.20, 0.723))
        initial_other = _book((0.88, 0.25, 0.724))
        after_coarse_target = _book((0.70, -0.20, 0.723))
        replay_near_distractor = _book((0.715, -0.391, 0.723))
        final_target = _book((0.716, -0.390, 0.725))
        final_other = _book((0.72, 0.05, 0.724))
        vision = _Vision(
            [
                [initial_other, initial_target],
                [replay_near_distractor, after_coarse_target],
                [final_other, final_target],
            ]
        )
        navigator = _AlignmentNavigator(
            [
                _nav_result(0.20, 0.0),
                _nav_result(-0.015, 0.191),
            ]
        )
        return vision, navigator, initial_target, after_coarse_target, final_target

    def test_runs_coarse_then_precise_and_keeps_same_book(self):
        vision, navigator, initial, after_coarse, final = self._scenario()
        messages = []

        result = run_book_alignment_once(
            vision,
            navigator,
            _reference(),
            say=messages.append,
        )

        self.assertEqual(vision.frames, ["base_link", "base_link", "base_link"])
        self.assertEqual(len(navigator.calls), 2)
        self.assertIs(result.coarse.book, initial)
        self.assertIs(result.precise.book, after_coarse)
        self.assertIs(result.final.book, final)
        self.assertAlmostEqual(navigator.calls[0][0][0], 0.48)
        self.assertEqual(
            navigator.calls[1][0],
            _reference().recorded_book_suction_point_base_m,
        )
        self.assertAlmostEqual(result.z_offset_m, result.final.z_offset_m)
        self.assertAlmostEqual(result.z_offset_m, 0.012)
        self.assertTrue(result.xy_within_tolerance)
        self.assertTrue(any("0.48 m 粗定位" in message for message in messages))
        self.assertTrue(any("DataReplay 精确偏差" in message for message in messages))
        self.assertTrue(any("最终 DataReplay 残差" in message for message in messages))

    def test_explicit_coarse_pick_receives_final_stage_z_offset(self):
        vision, navigator, _initial, _after_coarse, _final = self._scenario()
        replayer = _PickReplayer()

        result = run_book_pick_once(
            vision,
            navigator,
            replayer,
            _reference(),
            say=lambda _message: None,
            coarse=True,
        )

        self.assertEqual(replayer.offsets, [result.alignment.final.z_offset_m])
        self.assertEqual(result.replay.frames_sent, 607)
        self.assertTrue(result.replay.d01_holding)

    def test_pick_defaults_to_current_position_without_coarse_alignment(self):
        target = _book((0.785, -0.380, 0.723))
        final_target = _book((0.716, -0.390, 0.723))
        vision = _Vision([[target], [final_target]])
        navigator = _AlignmentNavigator([_nav_result(0.070, 0.010)])
        replayer = _PickReplayer()

        result = run_book_pick_once(
            vision,
            navigator,
            replayer,
            _reference(),
            say=lambda _message: None,
        )

        self.assertIsNone(result.alignment.coarse)
        self.assertEqual(len(navigator.calls), 1)
        self.assertEqual(replayer.offsets, [result.alignment.final.z_offset_m])

    def test_pick_does_not_replay_when_final_x_exceeds_twenty_millimetres(self):
        target = _book((0.785, -0.380, 0.723))
        final_target = _book((0.736, -0.390, 0.723))
        vision = _Vision([[target], [final_target]])
        navigator = _AlignmentNavigator([_nav_result(0.070, 0.010)])
        replayer = _PickReplayer()

        with self.assertRaisesRegex(RuntimeError, "最终对位未达标"):
            run_book_pick_once(
                vision,
                navigator,
                replayer,
                _reference(),
                say=lambda _message: None,
            )

        self.assertEqual(replayer.offsets, [])

    def test_pick_does_not_replay_when_final_y_exceeds_ten_millimetres(self):
        target = _book((0.785, -0.380, 0.723))
        final_target = _book((0.715, -0.379, 0.723))
        vision = _Vision([[target], [final_target]])
        navigator = _AlignmentNavigator([_nav_result(0.070, 0.010)])
        replayer = _PickReplayer()

        with self.assertRaisesRegex(RuntimeError, "最终对位未达标"):
            run_book_pick_once(
                vision,
                navigator,
                replayer,
                _reference(),
                say=lambda _message: None,
            )

        self.assertEqual(replayer.offsets, [])

    def test_rejects_empty_initial_detection_before_navigation(self):
        vision = _Vision([[]])
        navigator = _AlignmentNavigator([])

        with self.assertRaisesRegex(RuntimeError, "没有检测到可对位的书本"):
            run_book_alignment_once(
                vision,
                navigator,
                _reference(),
                say=lambda _message: None,
            )

        self.assertEqual(navigator.calls, [])

    def test_current_position_mode_skips_coarse_and_runs_only_precise_alignment(self):
        target = _book((0.785, -0.380, 0.723))
        other = _book((0.900, 0.250, 0.724))
        final_target = _book((0.716, -0.390, 0.723))
        vision = _Vision([[other, target], [final_target]])
        navigator = _AlignmentNavigator([_nav_result(0.070, 0.010)])

        result = run_book_alignment_from_current_once(
            vision,
            navigator,
            _reference(),
            say=lambda _message: None,
        )

        self.assertEqual(vision.frames, ["base_link", "base_link"])
        self.assertEqual(len(navigator.calls), 1)
        self.assertEqual(
            navigator.calls[0][0],
            _reference().recorded_book_suction_point_base_m,
        )
        self.assertIs(result.precise.book, target)
        self.assertIs(result.final.book, final_target)
        self.assertIsNone(result.coarse)
        self.assertIsNone(result.coarse_navigation)
        self.assertAlmostEqual(result.z_offset_m, result.final.z_offset_m)

    def test_pick_can_start_from_current_position_without_coarse_alignment(self):
        target = _book((0.785, -0.380, 0.723))
        final_target = _book((0.716, -0.390, 0.723))
        vision = _Vision([[target], [final_target]])
        navigator = _AlignmentNavigator([_nav_result(0.070, 0.010)])
        replayer = _PickReplayer()

        result = run_book_pick_once(
            vision,
            navigator,
            replayer,
            _reference(),
            say=lambda _message: None,
            coarse=False,
        )

        self.assertEqual(len(navigator.calls), 1)
        self.assertEqual(replayer.offsets, [result.alignment.final.z_offset_m])


if __name__ == "__main__":
    unittest.main()
