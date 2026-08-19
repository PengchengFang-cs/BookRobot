import sys
import unittest
from unittest.mock import patch

from main import arguments


class MainArgumentTests(unittest.TestCase):
    def test_accepts_book_alignment_mode(self):
        with patch.object(sys, "argv", ["main.py", "--book-align"]):
            args = arguments()

        self.assertTrue(args.book_align)
        self.assertEqual(args.book_align_mode, "legacy")

    def test_accepts_vector_book_alignment_mode(self):
        with patch.object(
            sys,
            "argv",
            ["main.py", "--book-align", "--book-align-mode", "vector"],
        ):
            args = arguments()

        self.assertEqual(args.book_align_mode, "vector")

    def test_accepts_book_pick_with_vector_alignment(self):
        with patch.object(
            sys,
            "argv",
            ["main.py", "--book-pick", "--book-align-mode", "vector"],
        ):
            args = arguments()

        self.assertTrue(args.book_pick)
        self.assertEqual(args.book_align_mode, "vector")

    def test_book_pick_defaults_to_no_coarse_positioning(self):
        with patch.object(sys, "argv", ["main.py", "--book-pick"]):
            args = arguments()

        self.assertFalse(args.book_coarse)

    def test_accepts_explicit_coarse_positioning_for_pick(self):
        with patch.object(
            sys,
            "argv",
            [
                "main.py",
                "--book-pick",
                "--book-coarse",
                "--book-align-mode",
                "vector",
            ],
        ):
            args = arguments()

        self.assertTrue(args.book_pick)
        self.assertTrue(args.book_coarse)

    def test_check_and_book_pick_are_mutually_exclusive(self):
        with patch.object(
            sys,
            "argv",
            ["main.py", "--check", "--book-pick"],
        ):
            with self.assertRaises(SystemExit):
                arguments()

    def test_book_alignment_branch_precedes_legacy_navigation_setup(self):
        from pathlib import Path

        text = (Path(__file__).resolve().parents[1] / "main.py").read_text(
            encoding="utf-8"
        )
        branch = text.index("if args.book_align or args.book_pick:")
        legacy = text.index("from navigation import Navigation", branch)
        self.assertLess(branch, legacy)
        self.assertIn("BookAlignmentNavigator(mode=args.book_align_mode)", text)
        self.assertIn(
            "Stage1BookPickReplayer(\n"
            "                        joint_positions=lambda: dict(vision.joints),",
            text,
        )
        self.assertIn(
            "spin_feedback=lambda: rclpy.spin_once(node, timeout_sec=0.05)",
            text,
        )

    def test_book_modes_load_and_pass_the_replay_reference(self):
        from pathlib import Path

        from config import REPLAY_PICK_REFERENCE_PATH

        self.assertEqual(
            REPLAY_PICK_REFERENCE_PATH,
            Path(__file__).resolve().parents[1]
            / "config"
            / "stage1_pick_reference.json",
        )
        text = (Path(__file__).resolve().parents[1] / "main.py").read_text(
            encoding="utf-8"
        )
        branch = text[text.index("if args.book_align or args.book_pick:") :]
        self.assertIn("load_replay_pick_reference", branch)
        self.assertIn("REPLAY_PICK_REFERENCE_PATH", branch)
        self.assertIn("replay_reference", branch)


if __name__ == "__main__":
    unittest.main()
