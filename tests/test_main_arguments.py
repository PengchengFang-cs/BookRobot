import sys
import unittest
from unittest.mock import patch

from main import arguments


class MainArgumentTests(unittest.TestCase):
    def test_accepts_book_alignment_mode(self):
        with patch.object(sys, "argv", ["main.py", "--book-align"]):
            args = arguments()

        self.assertTrue(args.book_align)

    def test_book_alignment_branch_precedes_legacy_navigation_setup(self):
        from pathlib import Path

        text = (Path(__file__).resolve().parents[1] / "main.py").read_text(
            encoding="utf-8"
        )
        branch = text.index("if args.book_align:")
        legacy = text.index("from navigation import Navigation", branch)
        self.assertLess(branch, legacy)


if __name__ == "__main__":
    unittest.main()
