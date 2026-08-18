# XY Navigation and Z Handoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove visual Z from FPC navigation and expose one fixed Stage 1 Z offset for the legacy Pipeline.

**Architecture:** `book_alignment.py` selects by XY and owns the finite ±30 mm Z handoff value. `book_navigation.py` passes reference Z for both points so `navnav_final` emits only base commands. `mission.py` preserves and reports the initial offset without implementing Pipeline token or replay behavior.

**Tech Stack:** Python 3, `unittest`, existing `navnav_final` adapter.

---

### Task 1: Specify XY-only selection and one-shot Z output

**Files:**
- Modify: `tests/test_book_alignment.py`

- [ ] Write failing tests proving selection uses only squared X/Y distance, `z_offset_m` equals initial `observed_z-reference_z`, `0.0` is retained, and offsets outside ±`0.03 m` are rejected.
- [ ] Run `python3 -m unittest tests.test_book_alignment -v` and confirm RED because selection currently includes Z and has no `z_offset_m` contract.
- [ ] Modify `book_alignment.py` to add `STAGE1_MAX_Z_OFFSET_M = 0.03`, store `z_offset_m` on `BookAlignmentTarget`, sort with `dx*dx + dy*dy`, and reject the selected target when the offset is non-finite or out of range.
- [ ] Re-run the focused test and confirm GREEN.

### Task 2: Remove torso movement from navigation

**Files:**
- Modify: `tests/test_book_navigation.py`
- Modify: `book_navigation.py`

- [ ] Replace replay-height tests with failing tests that assert the builder receives `observed.z == reference.z`, the adapter receives no custom torso tolerance, only the initial torso reading is needed, and the result reports only command count.
- [ ] Run `python3 -m unittest tests.test_book_navigation -v` and confirm RED because the current navigator still computes and executes a torso target.
- [ ] Remove `BookAlignmentExecution` torso fields and replay-height constants. Pass `(observed_x, observed_y, reference_z)` to the existing builder and return an immutable result containing only `command_count`.
- [ ] Re-run the focused test and confirm GREEN.

### Task 3: Preserve the initial offset through the mission result

**Files:**
- Modify: `tests/test_mission.py`
- Modify: `mission.py`

- [ ] Write a failing test that gives initial and final detections different Z values, then asserts `BookAlignmentRun.z_offset_m` and the reported offset use the initial detection only.
- [ ] Run `python3 -m unittest tests.test_mission -v` and confirm RED because the current mission reports torso feedback instead of a Pipeline handoff.
- [ ] Replace `height_alignment` with `z_offset_m`, report `固定Z偏移`, and keep the raw final residual report unchanged.
- [ ] Run the three focused modules and then `python3 -m unittest discover -s tests -v`.

### Task 4: Record and deploy without hardware motion

**Files:**
- Modify: `docs/CURRENT_STATUS.md`
- Modify: `docs/WORKLOG.md`
- Modify: `docs/ROBOT_ISSUES.md`

- [ ] Mark the prior navigation-stage torso fix as superseded by XY/Z separation, while retaining its historical record.
- [ ] Run `python3 -m py_compile book_alignment.py book_navigation.py mission.py`, `git diff --check`, and the complete test suite; commit the intended files on `main`.
- [ ] Use `scripts/deploy_to_robot.sh` to update `/home/unix_ai/fpc` directly from the committed local tree; do not use GitHub.
- [ ] On Wanda, verify the deployed commit, compile the three modules, and run the alignment/navigation/mission unit tests. Do not invoke `--book-align`, DataReplay, D01, torso, arm, or base commands.
