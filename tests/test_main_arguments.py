import sys
import unittest
from unittest.mock import patch

from main import arguments


class MainArgumentTests(unittest.TestCase):
    def test_accepts_book_alignment_mode(self):
        with patch.object(sys, "argv", ["main.py", "--book-align"]):
            args = arguments()

        self.assertTrue(args.book_align)


if __name__ == "__main__":
    unittest.main()
