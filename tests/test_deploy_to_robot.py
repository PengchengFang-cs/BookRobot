from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "deploy_to_robot.sh"
)


class DeployToRobotContractTests(unittest.TestCase):
    def test_deploys_committed_main_directly_without_github_or_delete(self):
        text = SCRIPT.read_text(encoding="utf-8")

        for required in (
            'repo_root="$(cd "$(dirname "$0")/.." && pwd)"',
            "/home/unix_ai/fpc",
            "git branch --show-current",
            "git status --porcelain",
            "git archive HEAD",
            "ssh tsinghuaBot",
            "rsync --archive",
            ".deployed-commit",
        ):
            self.assertIn(required, text)
        for forbidden in (
            "/home/cvailab/fpc",
            "--untracked-files=no",
            "git push",
            "git pull",
            "rsync --delete",
            "--delete",
        ):
            self.assertNotIn(forbidden, text)


if __name__ == "__main__":
    unittest.main()
