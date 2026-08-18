import unittest
from types import SimpleNamespace

from book_navigation import BookAlignmentNavigator


class _Adapter:
    def __init__(self, runtime):
        self.runtime = runtime
        self.preflight_calls = 0
        self.executed = []
        self.stopped = False

    def preflight(self):
        self.preflight_calls += 1

    def current_torso_position(self):
        return 0.20

    def execute_command(self, command, *, precision_mode):
        self.executed.append((command, precision_mode))

    def stop(self):
        self.stopped = True


class _Runtime:
    def __init__(self, commands):
        self.commands = commands
        self.builder_calls = []
        self.adapter = None

    def BasePoint3D(self, x, y, z):
        return (x, y, z)

    def WandaRos2Adapter(self):
        self.adapter = _Adapter(self)
        return self.adapter

    def build_base_alignment_commands(self, reference, observed, torso):
        self.builder_calls.append(
            SimpleNamespace(reference=reference, observed=observed, torso=torso)
        )
        return self.commands


class BookNavigationTests(unittest.TestCase):
    def test_executes_navnav_alignment_plan_in_order(self):
        runtime = _Runtime(["y", "z", "x"])
        navigator = BookAlignmentNavigator(runtime=runtime)

        count = navigator.align(
            reference=(0.91, -0.31, 0.75),
            observed=(1.01, -0.33, 0.76),
        )

        self.assertEqual(count, 3)
        self.assertEqual(runtime.adapter.preflight_calls, 1)
        call = runtime.builder_calls[0]
        self.assertEqual(call.reference, (0.91, -0.31, 0.75))
        self.assertEqual(call.observed, (1.01, -0.33, 0.76))
        self.assertEqual(call.torso, 0.20)
        self.assertEqual(
            runtime.adapter.executed,
            [("y", True), ("z", True), ("x", True)],
        )
        self.assertTrue(runtime.adapter.stopped)

    def test_stops_adapter_when_a_command_fails(self):
        runtime = _Runtime(["x"])
        navigator = BookAlignmentNavigator(runtime=runtime)
        original_execute = _Adapter.execute_command

        def fail(_self, _command, *, precision_mode):
            raise RuntimeError("motion failed")

        _Adapter.execute_command = fail
        try:
            with self.assertRaisesRegex(RuntimeError, "motion failed"):
                navigator.align(reference=(0.9, -0.3, 0.75), observed=(1, 0, 0.8))
            self.assertTrue(runtime.adapter.stopped)
        finally:
            _Adapter.execute_command = original_execute


if __name__ == "__main__":
    unittest.main()
