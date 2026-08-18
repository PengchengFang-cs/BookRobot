# Book Navigation Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a one-shot command that selects the detected book nearest the recorded Stage 1 pick point, uses the deployed `navnav_final` Y→Z→X commands to align the robot, and reports the post-motion residual without running the arm, DataReplay, or suction.

**Architecture:** Keep selection and residual arithmetic in a ROS-free `book_alignment.py`. Put the dynamic import and command execution for the robot-owned `/home/unix_ai/navnav_final` runtime in `book_navigation.py`. Add a focused orchestration function in `mission.py` and expose it through `main.py --book-align`; existing FruitTest behavior remains available but is not used by this entry point.

**Tech Stack:** Python 3, `unittest`, ROS 2/rclpy on Wanda, deployed `runtime.wanda_nav_whrc` navigation package.

---

### Task 1: Pure book target selection and residuals

**Files:**
- Create: `book_alignment.py`
- Create: `tests/test_book_alignment.py`

- [ ] **Step 1: Write failing selection tests**

Create tests that build simple objects with `suction_point`, verify that the point nearest `STAGE1_PICK_REFERENCE_BASE_M` is selected, verify the returned residual is `observed - reference`, and verify an empty list raises `RuntimeError("没有检测到可对位的书本")`.

```python
def test_selects_book_nearest_recorded_pick_point(self):
    near = SimpleNamespace(suction_point=(1.01, -0.33, 0.76))
    far = SimpleNamespace(suction_point=(1.08, 0.31, 0.77))
    selected = select_alignment_book([far, near])
    self.assertIs(selected.book, near)
    self.assertAlmostEqual(selected.residual_m[0], 1.01 - 0.9116001170551475)
```

- [ ] **Step 2: Run the new test and verify failure**

Run: `python3 -m unittest tests.test_book_alignment -v`

Expected: import failure because `book_alignment.py` does not exist.

- [ ] **Step 3: Implement the pure alignment model**

Add the recorded reference, immutable result type, finite three-vector normalization, distance-based selection, and residual calculation.

```python
STAGE1_PICK_REFERENCE_BASE_M = (
    0.9116001170551475,
    -0.31509978336130007,
    0.7552452105314827,
)

@dataclass(frozen=True)
class BookAlignmentTarget:
    book: object
    reference_m: tuple[float, float, float]
    observed_m: tuple[float, float, float]
    residual_m: tuple[float, float, float]

def select_alignment_book(books):
    if not books:
        raise RuntimeError("没有检测到可对位的书本")
    # Normalize points, choose the minimum squared 3-D distance, and return
    # observed-reference in x/y/z order.
```

- [ ] **Step 4: Run the focused tests**

Run: `python3 -m unittest tests.test_book_alignment -v`

Expected: all tests pass.

- [ ] **Step 5: Commit Task 1**

```bash
git add book_alignment.py tests/test_book_alignment.py
git commit -m "feat: select book for recorded pick alignment"
```

### Task 2: Adapter for deployed navnav_final alignment commands

**Files:**
- Create: `book_navigation.py`
- Create: `tests/test_book_navigation.py`

- [ ] **Step 1: Write failing adapter tests**

Inject a fake runtime module containing `BasePoint3D`, `build_base_alignment_commands`, and a fake `WandaRos2Adapter`. Verify that `align()` passes the fixed reference and observed point to the builder, uses the adapter's current torso value, executes returned commands in order with `precision_mode=True`, and always calls `stop()` when finished.

```python
def test_executes_navnav_alignment_plan_in_order(self):
    runtime = fake_runtime(commands=["y", "z", "x"], torso=0.20)
    navigator = BookAlignmentNavigator(runtime=runtime)
    navigator.align(reference=(0.91, -0.31, 0.75), observed=(1.01, -0.33, 0.76))
    self.assertEqual(runtime.builder_calls[0].torso, 0.20)
    self.assertEqual(runtime.adapter.executed, [
        ("y", True), ("z", True), ("x", True),
    ])
    self.assertTrue(runtime.adapter.stopped)
```

- [ ] **Step 2: Run the adapter tests and verify failure**

Run: `python3 -m unittest tests.test_book_navigation -v`

Expected: import failure because `book_navigation.py` does not exist.

- [ ] **Step 3: Implement the thin runtime adapter**

Implement a loader for `/home/unix_ai/navnav_final` and the one-shot navigator. Do not start `IntegratedWandaNavigation`, because its task-start path corrects absolute IMU yaw to zero; this alignment must preserve the robot's current yaw. Import and reuse the deployed `build_base_alignment_commands` and `WandaRos2Adapter` directly.

```python
class BookAlignmentNavigator:
    def __init__(self, runtime=None):
        self.runtime = runtime or load_navnav_runtime()

    def align(self, *, reference, observed):
        adapter = self.runtime.WandaRos2Adapter()
        try:
            adapter.preflight()
            commands = self.runtime.build_base_alignment_commands(
                self.runtime.BasePoint3D(*reference),
                self.runtime.BasePoint3D(*observed),
                adapter.current_torso_position(),
            )
            for command in commands:
                adapter.execute_command(command, precision_mode=True)
            return len(commands)
        finally:
            adapter.stop()
```

- [ ] **Step 4: Run adapter tests**

Run: `python3 -m unittest tests.test_book_navigation -v`

Expected: all tests pass without importing ROS.

- [ ] **Step 5: Commit Task 2**

```bash
git add book_navigation.py tests/test_book_navigation.py
git commit -m "feat: adapt navnav book alignment commands"
```

### Task 3: One-shot mission and CLI entry

**Files:**
- Modify: `mission.py`
- Modify: `main.py`
- Modify: `run.sh`
- Modify: `tests/test_mission.py`
- Create: `tests/test_main_arguments.py`

- [ ] **Step 1: Write failing mission tests**

Add a fake vision sequence and navigator. Verify the mission detects in `base_link`, chooses the nearest book, calls the navigator exactly once, detects again, reports the final residual, and never needs an arm object. Also verify the first empty detection raises the explicit no-book error.

```python
result = run_book_alignment_once(vision, navigator, say=messages.append)
self.assertEqual(vision.frames, ["base_link", "base_link"])
self.assertEqual(navigator.calls, [(result.initial.reference_m, result.initial.observed_m)])
self.assertIs(result.final.book, post_motion_near_book)
```

- [ ] **Step 2: Write a failing CLI parsing test**

Patch `sys.argv` with `['main.py', '--book-align']`, call `arguments()`, and assert `args.book_align` is true.

- [ ] **Step 3: Run focused mission and CLI tests and verify failure**

Run: `python3 -m unittest tests.test_mission tests.test_main_arguments -v`

Expected: failures because `run_book_alignment_once` and `--book-align` do not exist.

- [ ] **Step 4: Implement the one-shot orchestration**

Add result data and this flow to `mission.py`: first `vision.find("book", frame="base_link")`, `select_alignment_book`, navigator `align`, second detection, second selection, and formatted initial/final residual messages. The function returns both selections for logging and testing.

- [ ] **Step 5: Add `--book-align` to the real entry point**

In `main.py`, construct `BookAlignmentNavigator` only for this option, run the one-shot mission, print the current `vision.debug_path`, and return before importing or constructing `Arm`. Keep the existing base-release interaction because the chassis must be released before motion.

```python
if args.book_align:
    from book_navigation import BookAlignmentNavigator
    voice.say("请打开遥控器并按一下机身释放键")
    navigation.wait_for_release()
    run_book_alignment_once(vision, BookAlignmentNavigator(), voice.say)
    print(f"[视觉] 调试图: {vision.debug_path}")
    return
```

Update `run.sh` so `--book-align` performs the existing ROS environment setup and base release but directly launches `main.py`, without installing, preparing, or starting MoveIt. The branch must appear after environment setup and base release, and before `prepare_moveit.py`.

```bash
if [[ " $* " == *" --book-align "* ]]; then
  exec python3 "$DIR/main.py" "$@"
fi
```

- [ ] **Step 6: Run focused tests**

Run: `python3 -m unittest tests.test_book_alignment tests.test_book_navigation tests.test_mission tests.test_main_arguments -v`

Expected: all focused tests pass.

- [ ] **Step 7: Run the complete local test suite**

Run: `python3 -m unittest discover -s tests -v`

Expected: all tests pass; no robot motion occurs on the 5090.

- [ ] **Step 8: Commit Task 3**

```bash
git add mission.py main.py run.sh tests/test_mission.py tests/test_main_arguments.py
git commit -m "feat: add one-shot book navigation alignment"
```

### Task 4: Deploy without running motion

**Files:**
- Modify on Wanda only through fast-forward deployment: `/home/unix_ai/fpc`

- [ ] **Step 1: Verify both Git worktrees are clean and related**

Run local `git status --short` and read-only remote `git -C /home/unix_ai/fpc status --short`. Stop if either produces output. Confirm the robot's current commit is an ancestor of local `main`.

- [ ] **Step 2: Push local commits directly over SSH**

Use the established 5090-to-Wanda Git deployment path to fast-forward `/home/unix_ai/fpc`; do not use GitHub and do not run the new CLI.

- [ ] **Step 3: Verify robot-side import and tests without motion**

Run the focused pure tests and `python3 main.py --help` inside `/home/unix_ai/fpc`. Expected: tests pass and help lists `--book-align`; no ROS action or actuator command is sent.

- [ ] **Step 4: Record the deployment commit**

Update project status/worklog with the implementation commit and the fact that the motion test remains pending explicit onsite authorization. Commit the documentation update locally and fast-forward it to Wanda using the same direct SSH deployment path.

### Task 5: Execute one real alignment after onsite authorization

**Files:**
- Runtime evidence only under `/home/unix_ai/fpc/logs`; do not add logs or images to Git.

- [ ] **Step 1: Obtain the single motion authorization**

Confirm with the user that the robot's immediate floor area is clear, the table will not be struck by the small expected correction, and they authorize this one Y→Z→X alignment attempt.

- [ ] **Step 2: Run the one-shot command on Wanda**

Run: `cd /home/unix_ai/fpc && ./run.sh --book-align`

Expected: detect books, print selected point and initial residual, execute only navnav_final Y/Z/X alignment commands, detect again, print final residual and debug-image path, then exit without arm/DataReplay/D01 actions.

- [ ] **Step 3: Return evidence to the user**

Copy the post-motion debug image to the 5090 project logs for viewing and report initial versus final x/y/z residuals and the runtime's completion/error message. Do not claim navigation success unless the final observation supports it.
