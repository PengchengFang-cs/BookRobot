from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class BookAlignLauncherTests(unittest.TestCase):
    def test_perception_wait_covers_live_rgbd_startup(self):
        from config import VISION_TIMEOUT_S, VISION_WARMUP_S

        self.assertEqual(VISION_TIMEOUT_S, 20.0)
        self.assertEqual(VISION_WARMUP_S, 5.0)

    def test_book_alignment_loads_vision_environment_before_moveit_setup(self):
        run_text = (ROOT / "run.sh").read_text(encoding="utf-8")

        branch = run_text.index('" --book-align "')
        moveit_install = run_text.index("install_moveit_user.sh")
        self.assertLess(branch, moveit_install)
        self.assertIn('source "$DIR/scripts/book_vision_env.sh"', run_text)

    def test_book_alignment_does_not_wait_for_a_new_release_button_edge(self):
        main_text = (ROOT / "main.py").read_text(encoding="utf-8")
        branch_start = main_text.index("if args.book_align:")
        branch_end = main_text.index("\n            return", branch_start)
        branch = main_text[branch_start:branch_end]

        self.assertNotIn("wait_for_release", branch)

    def test_book_alignment_does_not_release_an_already_enabled_base(self):
        run_text = (ROOT / "run.sh").read_text(encoding="utf-8")
        branch_start = run_text.index('if [[ " $* " == *" --book-align "* ]]')
        branch_end = run_text.index("\nfi", branch_start)
        branch = run_text[branch_start:branch_end]

        self.assertNotIn("release_base.sh", branch)

    def test_vision_environment_matches_running_7443_service(self):
        text = (ROOT / "scripts" / "book_vision_env.sh").read_text(
            encoding="utf-8"
        )

        for required in (
            "127.0.0.1:7443",
            "planning-server.bookbot.internal",
            "ruan/visiond",
            "onsite-gpu0-vision",
            "grounding-dino-base-swinb+sam2.1-hiera-large+ppocrv6-20260814-r3",
            "bb6af3d199e3c41051a4844ae122ebf1e0a7bd63496f33f22f78e9ee0599371f",
            "wanda-head-rgbd-live-20260815",
            "/home/unix_ai/.local/share/bookbot/vision-rpc/runtime/sysroot",
            "/home/unix_ai/.local/share/bookbot/vision-rpc/certs",
            "/home/unix_ai/.local/share/bookbot/vision-rpc/private/wanda-vision-client.key",
        ):
            self.assertIn(required, text)
        self.assertNotIn("BEGIN PRIVATE KEY", text)

    def test_large_rgbd_frames_use_reliable_qos(self):
        text = (ROOT / "vision.py").read_text(encoding="utf-8")

        self.assertIn("ReliabilityPolicy.RELIABLE", text)
        self.assertIn("Image, COLOR_TOPIC, self._color, rgbd_qos", text)
        self.assertIn("Image, DEPTH_TOPIC, self._depth, rgbd_qos", text)


if __name__ == "__main__":
    unittest.main()
