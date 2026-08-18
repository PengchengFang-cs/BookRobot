# DataReplay Height Baseline Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Compute and verify book-pick torso height from the Stage 1 DataReplay recording baseline instead of the robot's arbitrary current torso height.

**Architecture:** Keep `navnav_final` read-only and continue using its Y→Z→X command builder. The local adapter converts the absolute replay-based torso target into the builder's relative Z input, verifies final `body_joint` feedback within 3 mm, and returns structured evidence for the mission report.

**Tech Stack:** Python 3, `unittest`, existing `navnav_final` ROS 2 adapter.

---

### Task 1: Specify replay-based height behavior

**Files:**
- Modify: `tests/test_book_navigation.py`

- [ ] **Step 1: Replace the old clamp test with failing baseline tests**

Add tests that start at `0.28 m`, use a `+0.008 m` book Z delta, and assert that the builder receives a virtual Z displacement producing `0.208 m`. Add cases for negative delta, out-of-range targets, accepted `3 mm` feedback, rejected feedback beyond `3 mm`, and adapter stop on rejection.

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
python3 -m unittest tests.test_book_navigation -v
```

Expected: failures because `BookAlignmentNavigator` still calculates from the current torso, clamps out-of-range values, returns an integer, and does not verify final torso feedback.

### Task 2: Implement the absolute replay baseline

**Files:**
- Modify: `book_navigation.py`

- [ ] **Step 1: Add the result contract and constants**

Add an immutable `BookAlignmentExecution` with `command_count`, `replay_torso_m`, `target_torso_m`, `actual_torso_m`, and `effective_z_residual_m`. Define the Stage 1 replay torso baseline as `0.20 m` and tolerance as `0.003 m`.

- [ ] **Step 2: Calculate the absolute target and virtual builder input**

Implement:

```python
book_z_delta = float(observed[2]) - float(reference[2])
target_torso = replay_torso + book_z_delta
virtual_observed_z = float(reference[2]) + target_torso - current_torso
```

Reject targets outside `[0.0, 0.28] m`; do not clamp them.

- [ ] **Step 3: Verify final torso feedback**

After executing commands, read `body_joint` again and calculate:

```python
effective_z_residual = book_z_delta - (actual_torso - replay_torso)
```

Raise a clear error when its absolute value exceeds `0.003 m`; otherwise return `BookAlignmentExecution`.

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run:

```bash
python3 -m unittest tests.test_book_navigation -v
```

Expected: all book-navigation tests pass.

### Task 3: Report compensated Z evidence

**Files:**
- Modify: `mission.py`
- Modify: `tests/test_mission.py`

- [ ] **Step 1: Write a failing mission-report test**

Make the fake navigator return `BookAlignmentExecution`-shaped data and assert that the report includes target torso, actual torso, and effective Z residual while preserving the raw final `dx/dy/dz` report.

- [ ] **Step 2: Run the mission tests and verify RED**

Run:

```bash
python3 -m unittest tests.test_mission -v
```

Expected: failure because the mission currently expects an integer and does not report height evidence.

- [ ] **Step 3: Update the mission result and output**

Store the structured navigation result in `BookAlignmentRun`, keep `command_count` available to callers, and print the target, actual, and effective Z residual in metres.

- [ ] **Step 4: Run focused and full tests**

Run:

```bash
python3 -m unittest tests.test_book_navigation tests.test_mission -v
python3 -m unittest discover -s tests -v
```

Expected: all tests pass.

### Task 4: Record, deploy, and verify without motion

**Files:**
- Modify: `docs/WORKLOG.md`
- Modify: `docs/CURRENT_STATUS.md`
- Modify: `docs/ROBOT_ISSUES.md`

- [ ] **Step 1: Record the root cause and new formula**

Document that the former `current + delta` formula violated the replay-start precondition, and record the new `0.20 + delta` formula and `3 mm` effective-residual criterion.

- [ ] **Step 2: Run static verification and commit**

Run:

```bash
python3 -m py_compile book_navigation.py mission.py
git diff --check
git status --short
```

Commit only the intended local project files.

- [ ] **Step 3: Deploy through the existing robot deployment path**

Run the repository's existing deployment script after confirming both local and robot Git trees permit a fast-forward deployment. Do not use GitHub as a transport.

- [ ] **Step 4: Perform non-motion verification on the robot**

Verify the deployed commit, import the modified modules, and run applicable unit tests. Do not invoke the live alignment entrypoint, DataReplay, D01 start, or any hardware-motion command.
