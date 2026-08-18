# FPC Single-Repository Deployment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `/home/cvailab/fpc` self-contained for development records and direct deployment to `/home/unix_ai/fpc`, without requiring Git or GitHub on the robot.

**Architecture:** Store public connection topology and deployment policy in the main repository. Export only committed files with `git archive`, transfer them directly over SSH/rsync without deletion, and mark the deployed commit on Wanda. Keep `fpc-ops` as untouched historical reference.

**Tech Stack:** Bash, Git, SSH, rsync, shell integration tests.

---

### Task 1: Define the direct-deployment contract in a failing test

**Files:**
- Create: `tests/test_deploy_to_robot.py`
- Create: `scripts/deploy_to_robot.sh`

- [ ] **Step 1: Write a failing static contract test**

Read `scripts/deploy_to_robot.sh` as text and assert it contains the fixed local and robot roots, requires `main` and a clean working tree, uses `git archive HEAD`, `ssh tsinghuaBot`, `rsync --archive`, and `.deployed-commit`. Assert it does not contain `git push`, `git pull`, `rsync --delete`, or `--delete`.

- [ ] **Step 2: Run the test and observe failure**

Run: `python3 -m unittest tests.test_deploy_to_robot -v`

Expected: failure because `scripts/deploy_to_robot.sh` does not exist.

- [ ] **Step 3: Implement the deployment script**

Create an executable script that verifies local `main` and cleanliness, exports `HEAD` into a `mktemp` directory, creates `/home/unix_ai/fpc`, runs `rsync --archive`, writes the commit marker through SSH, and prints `deployed=<sha>`.

- [ ] **Step 4: Run the focused test and syntax check**

Run: `python3 -m unittest tests.test_deploy_to_robot -v && bash -n scripts/deploy_to_robot.sh`

Expected: all checks pass.

### Task 2: Record the consolidated workspace and infrastructure

**Files:**
- Modify: `AGENTS.md`
- Create: `docs/INFRASTRUCTURE.md`
- Modify: `docs/README.md`
- Modify: `docs/PROJECT_MAINLINE.md`
- Modify: `docs/CURRENT_STATUS.md`

- [ ] **Step 1: Update the workspace policy**

List `/home/cvailab/fpc` and `/home/cvailab/fpc-ops` as writable 5090 project directories. State that Wanda `/home/unix_ai/fpc` is a plain runtime copy, so the local Git tree must be clean but the robot directory has no fast-forward requirement. Retain the prohibitions on credentials, writes to other robot paths, force push, and unauthorized hardware motion.

- [ ] **Step 2: Add the infrastructure record**

Document `tsinghua5090` (`10.10.0.213`, `cvailab`, port 22), `tsinghuaBot` (`192.168.137.212`, `unix_ai`, port 22), project paths, the direct 5090→Wanda deployment direction, GitHub's backup-only role, and the no-password/no-key boundary.

- [ ] **Step 3: Correct the project documentation**

Link `INFRASTRUCTURE.md` from `docs/README.md`, replace the old robot fast-forward statement in `PROJECT_MAINLINE.md`, and record in `CURRENT_STATUS.md` that the one-shot alignment implementation is locally complete while direct deployment and the real attempt are next.

- [ ] **Step 4: Verify documentation consistency**

Run `rg` checks for the two writable 5090 paths, the plain robot runtime copy, both Host aliases, backup-only GitHub role, and the direct deployment script. Run `git diff --check`.

### Task 3: Verify and commit the repository change

**Files:**
- All files from Tasks 1 and 2

- [ ] **Step 1: Run the complete local test suite**

Run: `python3 -m unittest discover -s tests -v`

Expected: all existing tests and the new deployment contract test pass.

- [ ] **Step 2: Commit the implementation**

```bash
git add AGENTS.md scripts/deploy_to_robot.sh tests/test_deploy_to_robot.py \
  docs/INFRASTRUCTURE.md docs/README.md docs/PROJECT_MAINLINE.md \
  docs/CURRENT_STATUS.md
git commit -m "feat: deploy directly from fpc to robot"
```

### Task 4: Deploy and verify without motion

**Files:**
- Runtime copy: `/home/unix_ai/fpc`

- [ ] **Step 1: Execute the direct deployment script**

Run: `/home/cvailab/fpc/scripts/deploy_to_robot.sh`

Expected: the script prints the deployed local commit and does not access GitHub.

- [ ] **Step 2: Verify the robot runtime without motion**

On Wanda, run `bash -n run.sh`, compile the new Python files, run the pure alignment/navigation tests, and run `python3 main.py --help`. Verify `.deployed-commit` equals local `HEAD`.

- [ ] **Step 3: Update the status record**

Record the deployed commit and no-motion verification result in `docs/CURRENT_STATUS.md` and `docs/WORKLOG.md`, commit, redeploy once with the same script, and verify the marker again.

### Task 5: Run the already-authorized alignment attempt

**Files:**
- Runtime evidence under `/home/unix_ai/fpc/logs`

- [ ] **Step 1: Execute one alignment**

Run on Wanda: `cd /home/unix_ai/fpc && ./run.sh --book-align`.

Expected: detect, select the nearest book, execute only Y→Z→X alignment, detect again, report residual, and exit without arm, DataReplay, or suction.

- [ ] **Step 2: Return the result**

Copy the final debug image to local ignored `logs/`, report the initial and final residuals, and record the observed result in the project worklog.
