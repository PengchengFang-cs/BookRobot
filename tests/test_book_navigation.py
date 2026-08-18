import unittest
from types import SimpleNamespace
from tempfile import TemporaryDirectory
from unittest.mock import patch

from book_navigation import BookAlignmentNavigator, load_navnav_runtime


class _Adapter:
    def __init__(self, runtime):
        self.runtime = runtime
        self.preflight_calls = 0
        self.executed = []
        self.stopped = False

    def preflight(self):
        self.preflight_calls += 1

    def current_torso_position(self):
        if self.runtime.torso_readings:
            return self.runtime.torso_readings.pop(0)
        return self.runtime.torso

    def execute_command(self, command, *, precision_mode):
        self.executed.append((command, precision_mode))

    def stop(self):
        self.stopped = True


class _Runtime:
    def __init__(self, commands, torso=0.20, torso_readings=()):
        self.commands = commands
        self.torso = torso
        self.torso_readings = list(torso_readings)
        self.builder_calls = []
        self.adapter = None

    def BasePoint3D(self, x, y, z):
        return (x, y, z)

    def WandaRos2Adapter(self, **kwargs):
        self.adapter_kwargs = kwargs
        self.adapter = _Adapter(self)
        return self.adapter

    def build_base_alignment_commands(self, reference, observed, torso):
        self.builder_calls.append(
            SimpleNamespace(reference=reference, observed=observed, torso=torso)
        )
        return self.commands


class BookNavigationTests(unittest.TestCase):
    @patch("book_navigation.load_navnav_runtime")
    def test_real_runtime_is_loaded_only_when_alignment_starts(self, load_runtime):
        runtime = _Runtime([])
        load_runtime.return_value = runtime

        navigator = BookAlignmentNavigator()

        load_runtime.assert_not_called()
        navigator.align(reference=(0.9, -0.3, 0.75), observed=(1.0, -0.3, 0.75))
        load_runtime.assert_called_once_with()

    @patch("book_navigation.importlib.import_module")
    def test_loader_gets_adapter_from_its_real_submodule(self, import_module):
        package = SimpleNamespace(
            BasePoint3D="point",
            build_base_alignment_commands="builder",
        )
        adapter_class = object()
        adapter_module = SimpleNamespace(WandaRos2Adapter=adapter_class)
        import_module.side_effect = [package, adapter_module]

        with TemporaryDirectory() as root:
            runtime = load_navnav_runtime(root)

        self.assertEqual(
            import_module.call_args_list[1].args[0],
            "runtime.wanda_nav_whrc.wanda_ros2_adapter",
        )
        self.assertIs(runtime.WandaRos2Adapter, adapter_class)
        self.assertEqual(runtime.BasePoint3D, "point")
        self.assertEqual(runtime.build_base_alignment_commands, "builder")

    def test_executes_navnav_alignment_plan_in_order(self):
        runtime = _Runtime(["y", "z", "x"], torso_readings=(0.20, 0.21))
        navigator = BookAlignmentNavigator(runtime=runtime)

        result = navigator.align(
            reference=(0.91, -0.31, 0.75),
            observed=(1.01, -0.33, 0.76),
        )

        self.assertEqual(result.command_count, 3)
        self.assertEqual(runtime.adapter_kwargs, {"torso_tolerance_m": 0.003})
        self.assertAlmostEqual(result.target_torso_m, 0.21)
        self.assertAlmostEqual(result.actual_torso_m, 0.21)
        self.assertAlmostEqual(result.effective_z_residual_m, 0.0)
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

    def test_uses_replay_start_height_instead_of_current_height(self):
        runtime = _Runtime([], torso_readings=(0.28, 0.208))
        navigator = BookAlignmentNavigator(runtime=runtime)

        result = navigator.align(
            reference=(0.91, -0.31, 0.75),
            observed=(1.01, -0.33, 0.758),
        )

        call = runtime.builder_calls[0]
        self.assertAlmostEqual(call.observed[2], 0.678)
        self.assertEqual(call.torso, 0.28)
        self.assertAlmostEqual(result.target_torso_m, 0.208)
        self.assertAlmostEqual(result.effective_z_residual_m, 0.0)

    def test_negative_book_z_delta_is_applied_to_replay_height(self):
        runtime = _Runtime([], torso_readings=(0.28, 0.19))
        navigator = BookAlignmentNavigator(runtime=runtime)

        result = navigator.align(
            reference=(0.91, -0.31, 0.75),
            observed=(1.01, -0.33, 0.74),
        )

        self.assertAlmostEqual(result.target_torso_m, 0.19)
        self.assertAlmostEqual(runtime.builder_calls[0].observed[2], 0.66)

    def test_rejects_replay_based_target_outside_robot_travel(self):
        runtime = _Runtime([], torso_readings=(0.28,))
        navigator = BookAlignmentNavigator(runtime=runtime)

        with self.assertRaisesRegex(RuntimeError, "升降目标.*超出"):
            navigator.align(
                reference=(0.91, -0.31, 0.75),
                observed=(1.01, -0.33, 0.84),
            )

        self.assertEqual(runtime.builder_calls, [])
        self.assertTrue(runtime.adapter.stopped)

    def test_accepts_effective_z_residual_within_three_millimetres(self):
        runtime = _Runtime([], torso_readings=(0.28, 0.2109))
        navigator = BookAlignmentNavigator(runtime=runtime)

        result = navigator.align(
            reference=(0.91, -0.31, 0.75),
            observed=(1.01, -0.33, 0.758),
        )

        self.assertAlmostEqual(result.effective_z_residual_m, -0.0029)

    def test_rejects_effective_z_residual_over_three_millimetres(self):
        runtime = _Runtime([], torso_readings=(0.28, 0.212))
        navigator = BookAlignmentNavigator(runtime=runtime)

        with self.assertRaisesRegex(RuntimeError, "有效 Z 残差"):
            navigator.align(
                reference=(0.91, -0.31, 0.75),
                observed=(1.01, -0.33, 0.758),
            )

        self.assertTrue(runtime.adapter.stopped)


if __name__ == "__main__":
    unittest.main()
