from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "release_base.sh"


class ReleaseBaseScriptTests(unittest.TestCase):
    def test_captures_controller_list_before_grep(self):
        text = SCRIPT.read_text(encoding="utf-8")

        self.assertIn("controllers=$(ros2 control list_controllers)", text)
        self.assertNotIn("ros2 control list_controllers | grep", text)
        self.assertEqual(text.count("ros2 control list_controllers"), 1)


if __name__ == "__main__":
    unittest.main()
