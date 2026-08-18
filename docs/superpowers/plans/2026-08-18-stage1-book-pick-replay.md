# Stage 1 Book Pick Replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add and run one FPC-controlled Stage 1 book pick using the existing perception/alignment result, a constant in-memory torso offset, the reviewed Pick DataReplay asset, and its frame-300 D01 event.

**Architecture:** Keep FPC responsible for perception and XYZ handoff. Add a narrow replay adapter that loads the read-only V3 Pick asset, copies and shifts only torso targets, prepositions the torso with navnav_final, then delegates paced publishing and D01 scheduling to the existing V3 runtime.

**Tech Stack:** Python 3, unittest, NumPy, ROS 2 Humble, navnav_final, V3 DataReplay, D01 TCP client.

---

### Task 1: Define the replay transformation

**Files:**
- Create: `book_pick_replay.py`
- Create: `tests/test_book_pick_replay.py`

- [ ] Write failing tests for positive, negative and zero torso offsets, preservation of the original episode/actions array, all-frame constant shift, and physical-range rejection.
- [ ] Run `python3 -m unittest tests.test_book_pick_replay -v` and verify failures are caused by the missing module/API.
- [ ] Implement `shift_pick_episode_torso(episode, offset_m)` with copied mappings and NumPy arrays; require finite numeric offset and every shifted target in `[0.0, 0.3]`.
- [ ] Re-run the focused tests and verify they pass.

### Task 2: Add the real Pick replay adapter

**Files:**
- Modify: `book_pick_replay.py`
- Modify: `tests/test_book_pick_replay.py`

- [ ] Write failing tests for one torso preposition at shifted frame 0, exactly one S1 Pick replay call, base replay disabled, forwarding of the frame-300 D01 event, final holding confirmation, and cleanup on error.
- [ ] Run the focused tests and verify the expected failures.
- [ ] Implement `Stage1BookPickReplayer.pick(z_offset_m)` with injectable loaders/runtime for tests. In production load `/home/unix_ai/WHRC/v3_pipeline/config/v3_robot_stage1.json`, `S1_TABLE_PICK_BOOK`, and the existing V3 DataReplay/site adapter; do not enable SHA verification.
- [ ] Use navnav_final `TORSO_POSITION` for the shifted frame-0 target and return structured frames/torso/D01 evidence.
- [ ] Re-run the focused tests and verify they pass.

### Task 3: Wire mission and command entrypoint

**Files:**
- Modify: `mission.py`
- Modify: `main.py`
- Modify: `run.sh`
- Modify: `tests/test_mission.py`
- Modify: `tests/test_main_arguments.py`
- Modify: `tests/test_book_align_launcher.py`

- [ ] Write failing tests that `run_book_pick_once` performs alignment then one replay with the unchanged Z offset, and that `--book-pick` selects the book runtime before legacy MoveIt setup.
- [ ] Run the focused tests and verify the expected failures.
- [ ] Implement the minimal orchestration and CLI branch; preserve existing `--book-align` and legacy FruitTest behavior.
- [ ] Re-run focused tests, then run `python3 -m unittest discover -s tests -v`.

### Task 4: Deploy and execute one live Pick

**Files:**
- Modify after evidence: `docs/CURRENT_STATUS.md`
- Modify after evidence: `docs/WORKLOG.md`
- Modify after evidence: `docs/ROBOT_ISSUES.md` only if a new or changed fault is observed

- [ ] Commit implementation on `main` and deploy with `./scripts/deploy_to_robot.sh`.
- [ ] On Wanda, compile the changed Python files and run focused non-motion tests.
- [ ] Read D01 idle state and current torso/odom once, then execute exactly `./run.sh --book-pick --book-align-mode vector` under the user's standing live-motion authorization.
- [ ] Record detection, alignment, torso target/feedback, replay frame count, D01 holding result, final stopped base state, and the debug image.
- [ ] Update and commit project records, deploy the record commit, and report the observed outcome without claiming success unless fresh evidence proves it.
