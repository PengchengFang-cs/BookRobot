# Fresh Torso Feedback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent Stage 1 frame-zero verification from rejecting a torso that physically reached its target because the cached `body_joint` value predates the pre-roll.

**Architecture:** `Vision` separates body feedback from the general joint callback and increments a body-feedback sequence after storing each valid body message. A small method spins ROS callbacks until that sequence advances or a bounded timeout expires; `main.py` supplies this method to the existing replayer feedback hook.

**Tech Stack:** Python 3, ROS 2 `rclpy`, `unittest`.

---

### Task 1: Reproduce stale torso feedback

**Files:**
- Modify: `tests/test_book_vision.py`

- [ ] **Step 1: Write failing tests**

Add focused tests which construct a `Vision` instance without ROS initialization, feed a body message through `_body_joints`, and prove `spin_until_fresh_body` keeps invoking a scripted callback until `body_feedback_sequence` advances. Add a second test with a scripted monotonic clock proving the method returns `False` on timeout.

- [ ] **Step 2: Verify RED**

Run:

```bash
python3 -m unittest tests.test_book_vision -v
```

Expected: failure because `_body_joints`, `body_feedback_sequence`, and `spin_until_fresh_body` do not exist.

### Task 2: Implement fresh torso feedback

**Files:**
- Modify: `vision.py`
- Modify: `main.py`
- Modify: `tests/test_main_arguments.py`

- [ ] **Step 1: Add the minimum implementation**

Initialize `body_feedback_sequence = 0`. Subscribe `BODY_JOINT_STATES_TOPIC` to `_body_joints`; that callback stores joints through `_joints` and increments the sequence. Add:

```python
def spin_until_fresh_body(self, spin_once, *, timeout_s=0.05, clock=time.monotonic):
    initial = self.body_feedback_sequence
    deadline = clock() + timeout_s
    while self.body_feedback_sequence == initial:
        remaining = deadline - clock()
        if remaining <= 0.0:
            return False
        spin_once(remaining)
    return True
```

Wire the Stage 1 replayer with:

```python
spin_feedback=lambda: vision.spin_until_fresh_body(
    lambda timeout_s: rclpy.spin_once(node, timeout_sec=timeout_s)
)
```

Update the source-wiring assertion in `tests/test_main_arguments.py` to require the new hook.

- [ ] **Step 2: Verify GREEN**

Run:

```bash
python3 -m unittest tests.test_book_vision tests.test_main_arguments -v
```

Expected: all tests pass.

- [ ] **Step 3: Run related regression tests**

Run:

```bash
python3 -m unittest tests.test_book_pick_replay tests.test_mission tests.test_sensor_sync -v
python3 -m unittest discover -s tests -v
git diff --check
```

Expected: all tests pass and `git diff --check` emits no output.

- [ ] **Step 4: Commit**

```bash
git add vision.py main.py tests/test_book_vision.py tests/test_main_arguments.py
git commit -m "fix: require fresh torso feedback before replay"
```

### Task 3: Deploy and resume the observed Pick

**Files:**
- Deploy committed `main` with `scripts/deploy_to_robot.sh`

- [ ] **Step 1: Deploy**

Run:

```bash
scripts/deploy_to_robot.sh
```

Expected: the robot `.deployed-commit` equals local `HEAD`.

- [ ] **Step 2: Verify without motion**

Compile the deployed modules and run the focused pure tests on the robot. Confirm controller and manipulation services are active and the right D01 is idle.

- [ ] **Step 3: Resume the authorized test**

Run without `--book-coarse`:

```bash
cd /home/unix_ai/fpc && ./run.sh --book-pick --book-align-mode vector
```

Expected: current-pose visual docking, fresh frame-zero torso acceptance, 333 recorded frames, frame-20 D01 start, and holding evidence.
