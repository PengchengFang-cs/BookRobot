import math
import unittest
from types import SimpleNamespace
from tempfile import TemporaryDirectory
from unittest.mock import patch

from book_navigation import (
    BookAlignmentNavigator,
    CartPlaceDockingNavigator,
    CART_ROUTE_MAX_SEGMENT_M,
    CART_SCAN_STEP_RAD,
    CART_SCAN_STEPS,
    CART_TURN_CLEARANCE_RETREAT_M,
    Stage1CartNavigator,
    Stage1TableReturnNavigator,
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
        self.refresh_calls = 0

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
        if self.runtime.pose_readings:
            x, y, yaw = self.runtime.pose_readings.pop(0)
            return SimpleNamespace(x=x, y=y, yaw=yaw)
        return self.runtime.final_pose

    def current_absolute_imu_yaw(self):
        return self.runtime.absolute_yaw

    def refresh_feedback(self):
        self.refresh_calls += 1

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
        pose_readings=(),
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
        self.pose_readings = list(pose_readings)

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
    def test_table_return_scans_rightmost_book_and_runs_inverse_grid_route(self):
        runtime = _Runtime([], absolute_yaw=0.2)
        books = (
            SimpleNamespace(suction_point=(0.8, -1.20, 0.7)),
            SimpleNamespace(suction_point=(0.9, -1.50, 0.7)),
        )
        vision = SimpleNamespace(
            scan_books_to_robot_right=lambda angles: books,
        )

        result = Stage1TableReturnNavigator(runtime, vision=vision).navigate()

        commands = [command for command, _precision in runtime.adapter.executed]
        self.assertEqual(result.selected_book_point_m, (0.9, -1.5, 0.7))
        self.assertAlmostEqual(result.table_leg_m, 1.1849002166387)
        self.assertEqual(commands[0].kind, "backward")
        self.assertAlmostEqual(commands[0].value, 0.2)
        self.assertEqual(commands[1].kind, "spin")
        self.assertAlmostEqual(commands[1].value, -math.pi / 2.0)
        self.assertTrue(all(command.kind == "forward" for command in commands[2:-2]))
        self.assertEqual(commands[-2].kind, "spin")
        self.assertAlmostEqual(commands[-2].value, math.pi / 2.0)
        self.assertEqual(commands[-1].kind, "forward")
        self.assertAlmostEqual(commands[-1].value, 0.2)
        self.assertAlmostEqual(
            runtime.adapter.yaw_corrections[-1]["target_yaw_rad"], 0.2
        )
        self.assertTrue(runtime.adapter.stopped)

    def test_cart_place_corrects_platform_yaw_before_xy(self):
        runtime = _Runtime([], absolute_yaw=0.3)
        target = SimpleNamespace(
            yaw_error_rad=math.radians(2.0),
            reference_anchor_m=(0.77, -0.08, 0.54),
            observed_anchor_m=(0.80, -0.06, 0.54),
        )

        result = CartPlaceDockingNavigator(runtime).align(target)

        self.assertEqual(result.mode, "cart-place-yaw")
        self.assertEqual(runtime.adapter.executed, [])
        self.assertAlmostEqual(
            runtime.adapter.yaw_corrections[0]["target_yaw_rad"],
            0.3 + math.radians(2.0),
        )
        self.assertTrue(runtime.adapter.stopped)

    def test_cart_place_uses_vector_xy_after_yaw_matches(self):
        runtime = _Runtime([], absolute_yaw=0.3)
        target = SimpleNamespace(
            yaw_error_rad=math.radians(0.2),
            reference_anchor_m=(0.77, -0.08, 0.54),
            observed_anchor_m=(0.82, -0.06, 0.54),
        )

        result = CartPlaceDockingNavigator(runtime).align(target)

        self.assertEqual(result.mode, "cart-place-xy")
        self.assertGreater(len(runtime.adapter.executed), 0)
        self.assertAlmostEqual(
            runtime.adapter.yaw_corrections[-1]["target_yaw_rad"], 0.3
        )
        self.assertTrue(runtime.adapter.stopped)

    @staticmethod
    def _batch_vision(carts, events=None):
        events = [] if events is None else events

        def capture_cart_frame(*, scan_angle_deg):
            events.append(("capture", scan_angle_deg))
            return SimpleNamespace(scan_angle_deg=scan_angle_deg)

        def detect_cart_frames_queued(frames):
            frames = tuple(frames)
            events.append(("detect_queue", len(frames)))
            return tuple(carts)

        return SimpleNamespace(
            capture_cart_frame=capture_cart_frame,
            detect_cart_frames_queued=detect_cart_frames_queued,
        )

    @staticmethod
    def _cart_seen_from_pose(pose, *, confidence, origin_center=(1.8, 0.5, 0.7)):
        x, y, yaw = pose
        cosine = math.cos(-yaw)
        sine = math.sin(-yaw)

        def from_origin(point):
            dx = point[0] - x
            dy = point[1] - y
            return (
                cosine * dx - sine * dy,
                sine * dx + cosine * dy,
                point[2],
            )

        def axis_from_origin(axis):
            return (
                cosine * axis[0] - sine * axis[1],
                sine * axis[0] + cosine * axis[1],
                axis[2],
            )

        slots = tuple((1.8, 0.5 + 0.1 * index, 0.7) for index in range(5))
        platform = SimpleNamespace(
            center=from_origin(origin_center),
            forward_axis=axis_from_origin((1.0, 0.0, 0.0)),
            lateral_axis_right_to_left=axis_from_origin((0.0, 1.0, 0.0)),
            normal=(0.0, 0.0, 1.0),
            depth_extent_m=0.4,
            lateral_extent_m=0.5,
            slot_centers=tuple(from_origin(slot) for slot in slots),
            confidence=confidence,
        )
        angle_deg = int(round(math.degrees(yaw)))
        return SimpleNamespace(
            body_target=SimpleNamespace(
                center=from_origin(origin_center),
                confidence=confidence,
            ),
            platform=platform,
            debug_image=f"/tmp/cart_scan_{angle_deg:03d}.jpg",
        )

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

    def test_cart_scan_only_stays_in_place_and_samples_every_fifteen_degrees(self):
        scan_poses = tuple(
            (-0.2, 0.0, CART_SCAN_STEP_RAD * index)
            for index in range(1, CART_SCAN_STEPS + 1)
        )
        runtime = _Runtime(
            [],
            pose_readings=scan_poses + (scan_poses[-1],),
        )
        carts = [
            self._cart_seen_from_pose(
                pose,
                confidence=0.9 if index == 3 else 0.5,
            )
            for index, pose in enumerate(scan_poses, 1)
        ]
        events = []
        vision = self._batch_vision(carts, events)
        result = Stage1CartNavigator(
            runtime,
            vision=vision,
            book_index=2,
            scan_only=True,
        ).navigate()

        commands = [row for row, _precision in runtime.adapter.executed]
        self.assertEqual(result.mode, "cart-scan-only")
        self.assertEqual(result.command_count, CART_SCAN_STEPS)
        self.assertEqual([row.kind for row in commands], ["spin"] * 6)
        self.assertEqual(runtime.adapter.refresh_calls, CART_SCAN_STEPS)
        self.assertTrue(all(
            math.isclose(row.value, CART_SCAN_STEP_RAD) for row in commands
        ))
        self.assertAlmostEqual(result.selected_scan_yaw_rad, math.radians(45.0))
        self.assertEqual(result.selected_scan_debug_image, "/tmp/cart_scan_045.jpg")
        self.assertEqual(result.slot_index, 2)
        for actual, expected in zip(
            result.slot_center_origin_m,
            (1.8, 0.6, 0.7),
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(
            events,
            [("capture", angle) for angle in (15, 30, 45, 60, 75, 90)]
            + [("detect_queue", 6)],
        )
        self.assertEqual(runtime.adapter.yaw_corrections, [])
        self.assertTrue(runtime.adapter.stopped)

    def test_cart_visual_navigation_stops_after_reviewed_coarse_route(self):
        scan_poses = tuple(
            (-0.2, 0.0, CART_SCAN_STEP_RAD * index)
            for index in range(1, CART_SCAN_STEPS + 1)
        )
        lateral_pose = (-0.2, 0.5, 0.0)
        final_pose = (0.0, 0.5, 0.0)
        runtime = _Runtime(
            [],
            absolute_yaw=0.12,
            pose_readings=(
                scan_poses
                + (scan_poses[-1], lateral_pose, final_pose)
            ),
        )
        carts = [
            self._cart_seen_from_pose(
                pose,
                confidence=0.95 if index == 4 else 0.6,
            )
            for index, pose in enumerate(scan_poses, 1)
        ]
        vision = self._batch_vision(carts)

        result = Stage1CartNavigator(
            runtime,
            vision=vision,
            book_index=1,
        ).navigate()

        commands = [row for row, _precision in runtime.adapter.executed]
        self.assertEqual(result.mode, "cart-visual-manhattan-coarse")
        for actual, expected in zip(
            result.cart_target_origin_m,
            (1.8, 0.5, 0.7),
        ):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(
            result.slot_center_origin_m,
            (1.8, 0.5, 0.7),
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(result.platform_near_x_origin_m, 1.6)
        self.assertAlmostEqual(result.selected_scan_yaw_rad, math.radians(60.0))
        self.assertEqual(commands[0].kind, "backward")
        self.assertEqual([row.kind for row in commands[1:7]], ["spin"] * 6)
        self.assertEqual([row.kind for row in commands[7:10]], ["forward"] * 3)
        self.assertAlmostEqual(sum(row.value for row in commands[7:10]), 0.5)
        self.assertEqual(commands[10].kind, "spin")
        self.assertAlmostEqual(commands[10].value, -math.pi / 2.0)
        self.assertEqual(commands[11].kind, "forward")
        self.assertAlmostEqual(commands[11].value, 0.2)
        self.assertAlmostEqual(
            runtime.adapter.yaw_corrections[0]["target_yaw_rad"],
            0.12,
        )
        self.assertEqual(len(runtime.adapter.yaw_corrections), 1)
        self.assertTrue(runtime.adapter.stopped)

    def test_cart_scan_stops_after_ninety_degrees_when_nothing_is_detected(self):
        scan_poses = tuple(
            (-0.2, 0.0, CART_SCAN_STEP_RAD * index)
            for index in range(1, CART_SCAN_STEPS + 1)
        )
        runtime = _Runtime([], pose_readings=scan_poses + (scan_poses[-1],))
        vision = self._batch_vision([None] * CART_SCAN_STEPS)

        with self.assertRaisesRegex(RuntimeError, "没有获得可用的小推车整体深度点"):
            Stage1CartNavigator(runtime, vision=vision, scan_only=True).navigate()

        commands = [row for row, _precision in runtime.adapter.executed]
        self.assertEqual(len(commands), CART_SCAN_STEPS)
        self.assertEqual([row.kind for row in commands], ["spin"] * 6)
        self.assertTrue(runtime.adapter.stopped)


if __name__ == "__main__":
    unittest.main()
