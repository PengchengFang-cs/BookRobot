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
        self.origin_calls = 0

    def preflight(self):
        self.preflight_calls += 1

    def current_torso_position(self):
        if self.runtime.torso_readings:
            return self.runtime.torso_readings.pop(0)
        return self.runtime.torso

    def capture_task_origin(self):
        self.origin_calls += 1
        return SimpleNamespace(x=0.0, y=0.0, yaw=0.0)

    def current_task_pose(self):
        return self.runtime.final_pose

    def execute_command(self, command, *, precision_mode):
        self.executed.append((command, precision_mode))

    def stop(self):
        self.stopped = True


class _Runtime:
    class WandaCommandKind:
        SPIN = "spin"
        DRIVE_FORWARD = "forward"
        DRIVE_BACKWARD = "backward"

    def __init__(
        self,
        commands,
        torso=0.20,
        torso_readings=(),
        final_pose=(0.01, -0.02, 0.003),
    ):
        self.commands = commands
        self.torso = torso
        self.torso_readings = list(torso_readings)
        self.builder_calls = []
        self.adapter = None
        self.final_pose = SimpleNamespace(
            x=final_pose[0], y=final_pose[1], yaw=final_pose[2]
        )

    @staticmethod
    def MappedMotionCommand(kind, value, axis):
        return SimpleNamespace(kind=kind, value=value, axis=axis)

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
            MappedMotionCommand="command",
            WandaCommandKind="kind",
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
        self.assertEqual(runtime.MappedMotionCommand, "command")
        self.assertEqual(runtime.WandaCommandKind, "kind")

    def test_executes_navnav_alignment_plan_in_order(self):
        runtime = _Runtime(["y", "x"], torso_readings=(0.28,))
        navigator = BookAlignmentNavigator(runtime=runtime)

        result = navigator.align(
            reference=(0.91, -0.31, 0.75),
            observed=(1.01, -0.33, 0.76),
        )

        self.assertEqual(result.command_count, 2)
        self.assertEqual(result.mode, "legacy")
        self.assertAlmostEqual(result.odom_dx_m, 0.01)
        self.assertAlmostEqual(result.odom_dy_m, -0.02)
        self.assertAlmostEqual(result.imu_dyaw_rad, 0.003)
        self.assertEqual(runtime.adapter_kwargs, {})
        self.assertEqual(runtime.adapter.preflight_calls, 1)
        self.assertEqual(runtime.adapter.origin_calls, 1)
        call = runtime.builder_calls[0]
        self.assertEqual(call.reference, (0.91, -0.31, 0.75))
        self.assertEqual(call.observed, (1.01, -0.33, 0.75))
        self.assertEqual(call.torso, 0.28)
        self.assertEqual(
            runtime.adapter.executed,
            [("y", True), ("x", True)],
        )
        self.assertTrue(runtime.adapter.stopped)

    def test_vector_mode_uses_shorter_reverse_heading(self):
        runtime = _Runtime([])
        navigator = BookAlignmentNavigator(runtime=runtime, mode="vector")

        result = navigator.align(
            reference=(0.9116, -0.3151, 0.755),
            observed=(0.8996, -0.3401, 0.766),
        )

        commands = [row[0] for row in runtime.adapter.executed]
        self.assertEqual(result.mode, "vector")
        self.assertEqual(runtime.builder_calls, [])
        self.assertEqual([command.kind for command in commands], [
            runtime.WandaCommandKind.SPIN,
            runtime.WandaCommandKind.DRIVE_BACKWARD,
            runtime.WandaCommandKind.SPIN,
        ])
        self.assertAlmostEqual(commands[1].value, (0.012**2 + 0.025**2) ** 0.5)
        self.assertLess(abs(commands[0].value), 3.141592653589793 / 2)
        self.assertAlmostEqual(commands[2].value, -commands[0].value)

    def test_vector_mode_uses_forward_for_small_forward_bearing(self):
        runtime = _Runtime([])
        navigator = BookAlignmentNavigator(runtime=runtime, mode="vector")

        navigator.align(
            reference=(0.9, -0.3, 0.75),
            observed=(1.0, -0.28, 0.80),
        )

        commands = [row[0] for row in runtime.adapter.executed]
        self.assertEqual(commands[1].kind, runtime.WandaCommandKind.DRIVE_FORWARD)
        self.assertAlmostEqual(commands[1].value, (0.1**2 + 0.02**2) ** 0.5)

    def test_vector_mode_emits_no_commands_at_xy_target(self):
        runtime = _Runtime([])
        navigator = BookAlignmentNavigator(runtime=runtime, mode="vector")

        result = navigator.align(
            reference=(0.9, -0.3, 0.75),
            observed=(0.9, -0.3, 0.82),
        )

        self.assertEqual(result.command_count, 0)
        self.assertEqual(runtime.adapter.executed, [])

    def test_rejects_unknown_alignment_mode(self):
        with self.assertRaisesRegex(ValueError, "legacy or vector"):
            BookAlignmentNavigator(runtime=_Runtime([]), mode="sideways")

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

    def test_visual_z_never_reaches_navigation_builder(self):
        runtime = _Runtime([], torso_readings=(0.28,))
        navigator = BookAlignmentNavigator(runtime=runtime)

        result = navigator.align(
            reference=(0.91, -0.31, 0.75),
            observed=(1.01, -0.33, 0.758),
        )

        call = runtime.builder_calls[0]
        self.assertEqual(call.observed[2], call.reference[2])
        self.assertEqual(call.torso, 0.28)
        self.assertEqual(result.command_count, 0)


if __name__ == "__main__":
    unittest.main()
