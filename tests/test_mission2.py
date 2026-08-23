import math
import sys
import unittest
from contextlib import ExitStack
from unittest.mock import patch

import mission2
import mission_main2


def shelf_book(
    number,
    *,
    actual_level=3,
    label_level=3,
    center=(0.0, 0.0, 1.0),
    lower_edge=None,
    higher_edge=None,
):
    return mission2.ShelfBookObservation(
        tracking_key=f"book-{actual_level}-{label_level}-{number}",
        actual_level=actual_level,
        label=mission2.BookLabel(
            text=f"A{label_level:02d}-{number:04d}",
            level=label_level,
            number=number,
        ),
        center_m=center,
        confidence=0.9,
        lower_number_edge_m=lower_edge,
        higher_number_edge_m=higher_edge,
        spine_direction_m=(0.0, 0.0, 1.0),
    )


class Stage23PureLogicTests(unittest.TestCase):
    def test_parses_canonical_and_physical_two_line_labels(self):
        canonical = mission2.parse_book_label("A03-0015")
        two_line = mission2.parse_book_label("A03\n0015")

        self.assertEqual(canonical, two_line)
        self.assertEqual(canonical.level, 3)
        self.assertEqual(canonical.number, 15)

    def test_rejects_wrong_prefix_or_level(self):
        with self.assertRaisesRegex(RuntimeError, "OCR无法识别"):
            mission2.parse_book_label("B03-0015")
        with self.assertRaisesRegex(RuntimeError, "第三、第四或第五层"):
            mission2.parse_book_label("A02-0015")

    def test_rejects_inconsistent_parsed_label(self):
        with self.assertRaisesRegex(RuntimeError, "文字与解析字段不一致"):
            mission2.validate_book_label(
                mission2.BookLabel(text="A03-0015", level=4, number=15)
            )

    def test_ocr_box_center_rejects_non_finite_coordinates(self):
        self.assertEqual(
            mission2.ocr_box_center(((2, 4), (8, 3), (7, 10), (1, 9))),
            (4.5, 6.5),
        )
        with self.assertRaisesRegex(ValueError, "有限数值"):
            mission2.ocr_box_center(
                ((0, 0), (math.nan, 0), (1, 1), (0, 1))
            )

    def test_adjacent_books_use_actual_inner_edges(self):
        target = mission2.infer_shelf_target(
            level=3,
            number=15,
            books=(
                shelf_book(
                    14,
                    center=(0.0, 0.00, 1.0),
                    higher_edge=(0.0, 0.03, 1.0),
                ),
                shelf_book(
                    16,
                    center=(0.0, 0.10, 1.0),
                    lower_edge=(0.0, 0.07, 1.0),
                ),
            ),
            shelf_number_axis=(0.0, 1.0, 0.0),
            shelf_yaw_rad=0.0,
        )

        self.assertEqual(target.left_m, (0.0, 0.03, 1.0))
        self.assertEqual(target.right_m, (0.0, 0.07, 1.0))
        self.assertEqual(target.center_m, (0.0, 0.05, 1.0))

    def test_target_number_already_present_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "已经被占用"):
            mission2.infer_shelf_target(
                level=3,
                number=15,
                books=(shelf_book(15), shelf_book(16, center=(0.0, 0.05, 1.0))),
                shelf_number_axis=(0.0, 1.0, 0.0),
                shelf_yaw_rad=0.0,
            )

    def test_wrong_level_label_does_not_calibrate_target_level_spacing(self):
        target = mission2.infer_shelf_target(
            level=3,
            number=13,
            books=(
                shelf_book(11, label_level=4, center=(0.0, 0.00, 1.0)),
                shelf_book(12, label_level=3, center=(0.0, 0.10, 1.0)),
            ),
            shelf_number_axis=(0.0, 1.0, 0.0),
            shelf_yaw_rad=0.0,
        )

        self.assertEqual(target.center_m[0], 0.0)
        self.assertAlmostEqual(target.center_m[1], 0.15)
        self.assertEqual(target.center_m[2], 1.0)

    def test_configuration_gate_rejects_invalid_values(self):
        frames = {name: 0 for name in mission2.D01_EVENT_FRAME_BY_ASSET}
        pick_points = {
            name: (1.0, 2.0, 3.0)
            for name in mission2.PICK_REFERENCE_POINT_M_BY_ASSET
        }
        place_points = {
            name: (1.0, 2.0, 3.0)
            for name in mission2.PLACE_REFERENCE_POINT_M_BY_ASSET
        }
        yaws = {name: 0.0 for name in mission2.PLACE_REFERENCE_YAW_RAD_BY_ASSET}
        with ExitStack() as stack:
            stack.enter_context(
                patch.dict(mission2.D01_EVENT_FRAME_BY_ASSET, frames, clear=True)
            )
            stack.enter_context(patch.dict(
                mission2.PICK_REFERENCE_POINT_M_BY_ASSET,
                pick_points,
                clear=True,
            ))
            stack.enter_context(patch.dict(
                mission2.PLACE_REFERENCE_POINT_M_BY_ASSET,
                place_points,
                clear=True,
            ))
            stack.enter_context(patch.dict(
                mission2.PLACE_REFERENCE_YAW_RAD_BY_ASSET,
                yaws,
                clear=True,
            ))
            stack.enter_context(patch.object(mission2, "OCR_TRANSPORT_READY", True))
            stack.enter_context(
                patch.object(mission2, "STAGE23_PERCEPTION_READY", True)
            )
            stack.enter_context(
                patch.object(mission2, "STAGE23_COARSE_NAVIGATION_READY", True)
            )
            self.assertEqual(mission2.pending_stage23_configuration(), ())

            mission2.D01_EVENT_FRAME_BY_ASSET["DR5.1"] = True
            mission2.PICK_REFERENCE_POINT_M_BY_ASSET["DR8.1"] = (
                math.nan,
                2.0,
                3.0,
            )
            mission2.PLACE_REFERENCE_YAW_RAD_BY_ASSET["DR5.1"] = math.inf
            pending = mission2.pending_stage23_configuration()

        self.assertTrue(any("DR5.1 D01事件帧无效" in item for item in pending))
        self.assertTrue(any("DR8.1 Pick第0帧视觉参考点无效" in item for item in pending))
        self.assertTrue(any("DR5.1 Place第0帧yaw参考无效" in item for item in pending))


class Stage23EntryTests(unittest.TestCase):
    def test_keyboard_interrupt_returns_130(self):
        with patch.object(sys, "argv", ["mission_main2.py"]):
            with patch.object(
                mission_main2,
                "run_real",
                side_effect=KeyboardInterrupt,
            ):
                with self.assertRaises(SystemExit) as raised:
                    mission_main2.main()

        self.assertEqual(raised.exception.code, 130)

    def test_run_script_routes_stage23_to_its_entrypoint(self):
        from pathlib import Path

        text = (Path(__file__).resolve().parents[1] / "run.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('if [[ " $* " == *" --stage23 "* ]]', text)
        self.assertIn('ENTRYPOINT="$DIR/mission_main2.py"', text)
        self.assertIn('if [[ "$argument" != "--stage23" ]]', text)

    def test_default_configuration_stops_before_ros(self):
        with patch.dict(sys.modules, {"rclpy": None}):
            with self.assertRaisesRegex(RuntimeError, "未初始化ROS"):
                mission2.require_stage23_configuration()


if __name__ == "__main__":
    unittest.main()
