import subprocess
import sys
import unittest
from pathlib import Path


class ReplayCalibrationCliTests(unittest.TestCase):
    def test_help_exposes_read_only_asset_inputs(self):
        script = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "calibrate_replay_pick_reference.py"
        )

        result = subprocess.run(
            [sys.executable, str(script), "--help"],
            text=True,
            capture_output=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--h5", result.stdout)
        self.assertIn("--reference-frame", result.stdout)
        self.assertIn("--output", result.stdout)
        self.assertIn("--overlay", result.stdout)


if __name__ == "__main__":
    unittest.main()
