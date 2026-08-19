# Exact Stage 1 DataReplay Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the complete Stage-1 Pick recording, including base motion, while making coarse positioning opt-in and restoring the robot to the HDF5 frame-0 position and posture before frame 0 is published.

**Architecture:** Keep perception and book geometry in FPC, but replace the unsafe two-stage mission assumptions with one frame-0 docking contract. Extend the FPC replay bridge so the real Pick asset is validated against its actual channels, the robot is smoothly prepositioned from feedback, and the legacy publisher emits the original base/arm/head/torso timeline with corrected D01 event ordering.

**Tech Stack:** Python 3, `unittest`, NumPy, h5py, ROS 2 runtime adapters loaded lazily on Wanda, existing DINO/SAM RGB-D geometry.

---

### Task 1: Lock the mission and CLI contracts with failing tests

**Files:**
- Modify: `tests/test_mission.py`
- Modify: `tests/test_main_arguments.py`
- Modify: `mission.py`
- Modify: `main.py`

- [ ] **Step 1: Write failing mission tests**

Add tests proving that default Pick uses only frame-0 docking, optional coarse positioning is called only when explicitly requested, X `>0.020 m` or Y `>0.010 m` prevents any replayer call, and final Z is the accepted replay offset.

```python
with self.assertRaisesRegex(RuntimeError, "最终对位未达标"):
    run_book_pick_once(...)
self.assertEqual(replayer.offsets, [])
```

- [ ] **Step 2: Write failing CLI tests**

Add parser tests for `--book-coarse` defaulting to false and a mutually exclusive error for `--check --book-pick`.

- [ ] **Step 3: Run focused tests and verify RED**

Run:

```bash
python3 -m unittest tests.test_mission tests.test_main_arguments -v
```

Expected: failures because default mission still executes old coarse docking, failed XY still calls replay, and CLI flags are not mutually exclusive.

- [ ] **Step 4: Implement the minimal mission and parser changes**

Use one default frame-0 alignment path. Rename the explicit opt-in argument to `coarse=False`; reject failed final residual before constructing or calling a replayer.

```python
if not alignment.xy_within_tolerance:
    raise RuntimeError("最终对位未达标，不启动 DataReplay")
```

- [ ] **Step 5: Run focused tests and verify GREEN**

Run the same unittest command. Expected: all focused tests pass.

- [ ] **Step 6: Commit**

```bash
git add mission.py main.py tests/test_mission.py tests/test_main_arguments.py
git commit -m "fix: gate stage1 replay on frame zero docking"
```

### Task 2: Bind replay references to geometry and repair recorded intrinsics

**Files:**
- Modify: `replay_pick_reference.py`
- Modify: `scripts/calibrate_replay_pick_reference.py`
- Modify: `config/stage1_pick_reference.json`
- Modify: `tests/test_replay_pick_reference.py`
- Modify: `tests/test_book_alignment.py`
- Modify: `book_alignment.py`

- [ ] **Step 1: Write failing reference and alignment tests**

Require OpenCV center-aware principal point scaling, persisted long axis/extents/asset identity, runtime asset validation, a maximum reassociation distance, extent agreement, and long-axis yaw agreement.

```python
self.assertAlmostEqual(scaled.cx, (source.cx + 0.5) * sx - 0.5)
with self.assertRaisesRegex(RuntimeError, "目标书重关联失败"):
    reassociate_book([distractor], predicted_point_m=predicted, ...)
```

- [ ] **Step 2: Run focused tests and verify RED**

```bash
python3 -m unittest tests.test_replay_pick_reference tests.test_book_alignment -v
```

Expected: missing reference fields and rejection gates; principal point assertion differs from current scaling.

- [ ] **Step 3: Implement immutable reference geometry and matching gates**

Extend `ReplayPickReference` with `recorded_book_long_axis_base`, `recorded_book_long_extent_m`, `recorded_book_short_extent_m`, and `hdf5_sha256`. Normalize undirected long-axis angle using `abs(dot)` and reject candidates outside the configured distance, extent, or yaw limits.

- [ ] **Step 4: Recalculate the real JSON from the read-only asset**

Run the existing calibration script against the real HDF5 with no motion; save only the generated JSON and debug overlay in FPC. Verify the JSON names frame 0 and the real Pick asset.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run the same focused unittest command. Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add replay_pick_reference.py scripts/calibrate_replay_pick_reference.py config/stage1_pick_reference.json book_alignment.py tests/test_replay_pick_reference.py tests/test_book_alignment.py
git commit -m "fix: bind frame zero docking geometry"
```

### Task 3: Validate and restore every frame-0 robot channel

**Files:**
- Modify: `book_pick_replay.py`
- Modify: `tests/test_book_pick_replay.py`

- [ ] **Step 1: Write failing tests for the real channel contract**

Tests must reject a nonzero `target_base_vel` when base publishing is disabled, require arms/head/torso/base action shapes, and require feedback-backed preposition before `replay_pick()`.

```python
with self.assertRaisesRegex(RuntimeError, "non-zero base trajectory is disabled"):
    runtime.validate_episode_contract(episode)
```

- [ ] **Step 2: Write failing smooth-preposition tests**

Inject current arm/head/torso feedback and assert bounded interpolation reaches the shifted HDF5 frame-0 targets before formal replay. Base remains zero throughout preposition.

- [ ] **Step 3: Run focused tests and verify RED**

```bash
python3 -m unittest tests.test_book_pick_replay -v
```

Expected: missing channel validation and posture-preposition APIs.

- [ ] **Step 4: Implement the local frame-0 prepositioner**

Build a private in-memory pre-roll from fresh feedback to HDF5 frame 0, publish bounded arm/head/torso samples through a narrow injected runtime, and verify final feedback. Do not count pre-roll samples as the 607 DataReplay frames and do not mutate HDF5 arrays.

- [ ] **Step 5: Enable exact Pick base replay**

Use an entry copy with `allow_base_motion=True` and `target_base_vel` in required channels. Reject any attempt to turn it off while the actual asset contains nonzero base commands.

- [ ] **Step 6: Run focused tests and verify GREEN**

Run the same focused unittest command. Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add book_pick_replay.py tests/test_book_pick_replay.py
git commit -m "fix: restore exact stage1 replay channels"
```

### Task 4: Correct replay delivery, D01 ordering, and failure state

**Files:**
- Modify: `book_pick_replay.py`
- Modify: `tests/test_book_pick_replay.py`

- [ ] **Step 1: Write failing ordering and cleanup tests**

Assert controller discovery occurs before frame 0, frame 300 publish precedes `right_suction_start`, all 607 frames retain original timing and base velocities, and holding/unknown failure never issues suction stop.

```python
self.assertLess(events.index(("publish", 300)), events.index(("d01", "start")))
```

- [ ] **Step 2: Run focused tests and verify RED**

```bash
python3 -m unittest tests.test_book_pick_replay -v
```

Expected: legacy `_run_episode` executes D01 before frame publish and cleanup is unconditional.

- [ ] **Step 3: Implement an FPC exact replay orchestrator**

Wrap the legacy `RobotReplayer` with FPC-controlled event ordering: construct the node, wait for required subscriptions, publish each original frame in timestamp order, execute frame-index events after that frame, stop base in `finally`, and restore services. Track D01 state so cleanup distinguishes unattached from holding/unknown.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run the focused tests. Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add book_pick_replay.py tests/test_book_pick_replay.py
git commit -m "fix: preserve stage1 replay event order"
```

### Task 5: Complete regression coverage and project records

**Files:**
- Modify: `README.md`
- Modify: `docs/CURRENT_STATUS.md`
- Modify: `docs/ROBOT_ISSUES.md`
- Modify: `docs/WORKLOG.md`
- Modify: relevant tests if integration gaps are found

- [ ] **Step 1: Add the real-asset offline contract test**

The test may skip when the read-only robot asset is unavailable, but on the RTX environment it must open the real HDF5 and assert nonzero base trajectory, 607 frames, exact action shapes, and the enabled FPC base policy.

- [ ] **Step 2: Run the complete test suite**

```bash
python3 -m unittest discover -s tests -v
```

Expected: zero failures.

- [ ] **Step 3: Update current and historical documentation**

Record the confirmed skipped-base root cause, exact replay repair, default-no-coarse CLI, X/Y tolerances, frame-0 posture contract, D01 ordering, offline-only status, and that no deployment or hardware test occurred.

- [ ] **Step 4: Run static verification**

```bash
python3 -m py_compile main.py mission.py book_alignment.py book_navigation.py book_pick_replay.py replay_pick_reference.py scripts/calibrate_replay_pick_reference.py
git diff --check
```

Expected: exit status 0.

- [ ] **Step 5: Commit**

```bash
git add README.md docs/CURRENT_STATUS.md docs/ROBOT_ISSUES.md docs/WORKLOG.md tests
git commit -m "docs: record exact stage1 replay repair"
```

### Task 6: Independent review and final verification

**Files:**
- Review all changes from `a2d3956` to final HEAD

- [ ] **Step 1: Request a read-only code review**

Review requirements: exact base/arm/head/torso replay, optional coarse default false, frame-0 position and posture restore, X/Y gates, target identity, D01 ordering, CLI exclusivity, and no writes outside FPC.

- [ ] **Step 2: Address every Critical and Important finding with TDD**

For each valid issue, add a failing regression test, observe RED, make the minimal fix, and observe GREEN.

- [ ] **Step 3: Run fresh final verification**

```bash
python3 -m unittest discover -s tests -v
python3 -m py_compile main.py mission.py book_alignment.py book_navigation.py book_pick_replay.py replay_pick_reference.py scripts/calibrate_replay_pick_reference.py
git diff --check
git status --short
```

Expected: all tests pass, compilation and diff checks exit 0, and the working tree is clean.
