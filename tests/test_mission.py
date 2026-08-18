import unittest
from types import SimpleNamespace

from mission import run_one_fruit


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


if __name__ == "__main__":
    unittest.main()
