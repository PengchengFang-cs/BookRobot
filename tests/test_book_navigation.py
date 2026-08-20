import math
import unittest
from types import SimpleNamespace
from tempfile import TemporaryDirectory
from unittest.mock import patch

from book_navigation import (
    BookAlignmentNavigator,
    CART_ROUTE_MAX_SEGMENT_M,
    CART_TURN_CLEARANCE_RETREAT_M,
    Stage1CartMapNavigator,
    build_cart_final_forward_commands,
    build_cart_manhattan_commands,
    cart_transition_body_delta,
    load_navnav_runtime,
)


class _Adapter:
    def __init__(self, runtime):
        self.runtime = runtime
        self.preflight_calls = 0
        self.executed = []
        self.stopped = False
        self.origin_calls = 0
        self.yaw_corrections = []

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

    def current_absolute_imu_yaw(self):
        return self.runtime.absolute_yaw

    def correct_absolute_imu_yaw(self, **kwargs):
        self.yaw_corrections.append(kwargs)
        return kwargs["target_yaw_rad"]

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
        absolute_yaw=0.42,
    ):
        self.commands = commands
        self.torso = torso
        self.torso_readings = list(torso_readings)
        self.builder_calls = []
        self.adapter = None
        self.final_pose = SimpleNamespace(
            x=final_pose[0], y=final_pose[1], yaw=final_pose[2]
        )
        self.absolute_yaw = absolute_yaw

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
        self.assertAlmostEqual(
            commands[1].value,
            (0.012**2 + 0.025**2) ** 0.5 - 0.010,
        )
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
        self.assertAlmostEqual(
            commands[1].value,
            (0.1**2 + 0.02**2) ** 0.5 - 0.010,
        )

    def test_vector_mode_does_not_move_inside_ten_millimetres(self):
        runtime = _Runtime([])
        navigator = BookAlignmentNavigator(runtime=runtime, mode="vector")

        result = navigator.align(
            reference=(0.9, -0.3, 0.75),
            observed=(0.906, -0.292, 0.80),
        )

        self.assertEqual(result.command_count, 0)
        self.assertEqual(runtime.adapter.executed, [])

    def test_vector_mode_restores_the_captured_absolute_imu_yaw(self):
        runtime = _Runtime([], absolute_yaw=0.42)
        navigator = BookAlignmentNavigator(runtime=runtime, mode="vector")

        navigator.align(
            reference=(0.9, -0.3, 0.75),
            observed=(1.0, -0.28, 0.80),
        )

        self.assertEqual(len(runtime.adapter.yaw_corrections), 1)
        correction = runtime.adapter.yaw_corrections[0]
        self.assertAlmostEqual(correction["target_yaw_rad"], 0.42)
        self.assertAlmostEqual(correction["tolerance_rad"], math.radians(0.15))

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

    def test_cart_transition_uses_reviewed_map_delta_in_table_body_axes(self):
        dx_m, dy_m = cart_transition_body_delta()

        self.assertAlmostEqual(dx_m, 0.52493, places=4)
        self.assertAlmostEqual(dy_m, 0.89052, places=4)
        self.assertAlmostEqual(math.hypot(dx_m, dy_m), 1.03372, places=4)

    def test_cart_route_backs_up_then_uses_two_right_angle_legs(self):
        runtime = _Runtime([])
        dx_m, dy_m = cart_transition_body_delta()

        commands = build_cart_manhattan_commands(runtime, dx_m, dy_m)

        self.assertEqual(commands[0].kind, runtime.WandaCommandKind.DRIVE_BACKWARD)
        self.assertAlmostEqual(commands[0].value, CART_TURN_CLEARANCE_RETREAT_M)
        spins = [command for command in commands if command.kind == "spin"]
        self.assertEqual([command.value for command in spins], [math.pi / 2, -math.pi / 2])
        translations = [command for command in commands if command.kind != "spin"]
        self.assertTrue(all(command.value <= CART_ROUTE_MAX_SEGMENT_M for command in translations))
        self.assertAlmostEqual(
            sum(command.value for command in translations[1:6]),
            dy_m,
        )
        self.assertAlmostEqual(
            sum(command.value for command in translations[6:]),
            dx_m + CART_TURN_CLEARANCE_RETREAT_M,
        )

    def test_cart_map_navigator_executes_manhattan_commands_and_restores_yaw(self):
        runtime = _Runtime([], absolute_yaw=0.31)
        expected_commands = build_cart_manhattan_commands(
            runtime, *cart_transition_body_delta()
        )

        result = Stage1CartMapNavigator(runtime).navigate()

        self.assertEqual(result.mode, "map-manhattan")
        self.assertEqual(result.command_count, len(expected_commands))
        self.assertEqual(
            [(row.kind, row.value) for row, _precision in runtime.adapter.executed],
            [(row.kind, row.value) for row in expected_commands],
        )
        self.assertTrue(all(precision for _row, precision in runtime.adapter.executed))
        self.assertEqual(runtime.adapter.yaw_corrections[0]["target_yaw_rad"], 0.31)
        self.assertTrue(runtime.adapter.stopped)

    def test_cart_resume_executes_only_requested_final_forward_segments(self):
        runtime = _Runtime([], absolute_yaw=0.31)
        dx_m, _dy_m = cart_transition_body_delta()
        expected = build_cart_final_forward_commands(runtime, dx_m, 2)

        result = Stage1CartMapNavigator(
            runtime,
            resume_final_forward_segments=2,
        ).navigate()

        self.assertEqual(result.mode, "map-manhattan-resume")
        self.assertEqual(result.command_count, 2)
        self.assertEqual(
            [(row.kind, row.value) for row, _precision in runtime.adapter.executed],
            [(row.kind, row.value) for row in expected],
        )
        self.assertAlmostEqual(sum(row.value for row in expected), 0.36246, places=4)

    def test_cart_resume_rejects_an_impossible_segment_count(self):
        runtime = _Runtime([])
        dx_m, _dy_m = cart_transition_body_delta()

        with self.assertRaisesRegex(ValueError, "between 1 and 4"):
            build_cart_final_forward_commands(runtime, dx_m, 5)


if __name__ == "__main__":
    unittest.main()
