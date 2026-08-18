# Optional Vector XY Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in combined XY vector maneuver with odom/IMU evidence while preserving legacy alignment as the default.

**Architecture:** `book_navigation.py` owns mode selection and vector command construction using runtime command types. Both modes capture a task-frame odom/IMU origin and return the final relative pose. CLI wiring selects the mode; mission reporting exposes physical and visual evidence without automatic retries.

**Tech Stack:** Python 3, `unittest`, existing `navnav_final` ROS 2 adapter.

---

### Task 1: Specify vector command behavior

**Files:**
- Modify: `tests/test_book_navigation.py`
- Modify: `book_navigation.py`

- [ ] Add failing tests for default legacy mode, vector forward and reverse branches, zero vector, reference-Z preservation, task-origin capture, and returned odom `x/y/yaw`.
- [ ] Run `python3 -m unittest tests.test_book_navigation -v` and confirm RED.
- [ ] Extend runtime loading with `MappedMotionCommand` and `WandaCommandKind`; add a pure vector command builder that normalizes yaw and chooses the smaller forward/reverse turn.
- [ ] Add `mode="legacy"` to `BookAlignmentNavigator`; capture task origin before commands, read current task pose afterward, and return command count, mode and odom evidence.
- [ ] Re-run the focused test and confirm GREEN.

### Task 2: Wire CLI and mission evidence

**Files:**
- Modify: `tests/test_main_arguments.py`
- Modify: `tests/test_mission.py`
- Modify: `main.py`
- Modify: `mission.py`

- [ ] Add failing tests that default to `legacy`, accept `vector`, pass it into `BookAlignmentNavigator`, and report odom/IMU evidence plus whether final X/Y are within `10 mm`.
- [ ] Run the two focused modules and confirm RED.
- [ ] Add `--book-align-mode` choices and pass the selected value. Extend mission output with mode, odom `dx/dy/yaw`, and a single final `达标/未达标` result without another movement.
- [ ] Run focused tests and `python3 -m unittest discover -s tests -v`.

### Task 3: Deploy and test

**Files:**
- Modify: `docs/CURRENT_STATUS.md`
- Modify: `docs/WORKLOG.md`
- Modify: `docs/ROBOT_ISSUES.md`

- [ ] Record the stationary perception evidence and optional-mode boundary.
- [ ] Compile modified modules, run full tests, run `git diff --check`, and commit on `main`.
- [ ] Deploy directly with `scripts/deploy_to_robot.sh`; run robot-side compile and unit tests without motion.
- [ ] After a fresh user confirmation that the area is clear, record pre/post odom, run exactly one `--book-align --book-align-mode vector`, and report final visual and physical residuals. Do not run torso, arm, DataReplay, or D01.
