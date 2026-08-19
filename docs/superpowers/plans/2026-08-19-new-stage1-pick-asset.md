# New Stage 1 Pick Asset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the old Stage-1 Pick replay with the new 333-frame `dr1.2` recording, start D01 at frame 20, and calibrate docking with frame-0 rule X/Z plus recorded contact Y.

**Architecture:** Keep the read-only HDF5 in DataCollector and override the legacy V3 asset entry inside the FPC replay bridge. Extend the existing replay-reference calibration module so it preserves frame-0 13 cm/10 cm geometry but substitutes only base Y from a blue-suction contact frame. The runtime continues consuming one `recorded_book_suction_point_base_m`, so mission and navigation code do not need a second alignment path.

**Tech Stack:** Python 3, NumPy, h5py, OpenCV, `unittest`, existing DINO/SAM book RPC, legacy ROS 2 replay adapter.

---

### Task 1: Switch the exact replay asset contract

**Files:**
- Modify: `book_pick_replay.py:18-28, 200-252, 570-605`
- Modify: `tests/test_book_pick_replay.py:120-145, 440-710`

- [ ] **Step 1: Write failing tests for the new asset and sidecar event**

Change the real-asset test path and expected shapes to 333 frames. Change runtime fixtures to default to 333 and assert that the FPC entry overrides the legacy file and event:

```python
NEW_PICK_ASSET = Path(
    "/home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/"
    "20260819_libraryrobot_datareplay/pi05_wanda_dr1.2_20260819_195231.h5"
)

self.assertEqual(Path(entry["file"]), NEW_PICK_ASSET)
self.assertEqual(
    entry["d01_events"],
    [{"frame_index": 20, "command": "right_suction_start"}],
)
self.assertEqual(evidence.frames_sent, 333)
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
python3 -m unittest tests.test_book_pick_replay -v
```

Expected: failures still report the old 607-frame contract, old HDF5 path, or frame-300 D01 event.

- [ ] **Step 3: Implement the new immutable asset constants and entry override**

Define the new contract in `book_pick_replay.py`:

```python
PICK_ASSET_PATH = Path(
    "/home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/"
    "20260819_libraryrobot_datareplay/pi05_wanda_dr1.2_20260819_195231.h5"
)
PICK_FRAME_COUNT = 333
PICK_D01_FRAME_INDEX = 20
```

After copying the legacy entry, replace only the asset-specific values:

```python
self.entry["file"] = str(PICK_ASSET_PATH)
self.entry["version"] = "stage1-dr1.2-20260819"
self.entry["d01_events"] = [
    {
        "frame_index": PICK_D01_FRAME_INDEX,
        "command": "right_suction_start",
    }
]
```

Retain `allow_base_motion=True`, all six required channels, original timestamps, asynchronous D01 execution, and the existing frame-zero pre-roll.

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run:

```bash
python3 -m unittest tests.test_book_pick_replay -v
```

Expected: all replay tests pass; when the read-only robot asset is reachable, the real contract test confirms six `(333, ...)` arrays and nonzero base velocity.

- [ ] **Step 5: Commit the replay contract**

```bash
git add book_pick_replay.py tests/test_book_pick_replay.py
git commit -m "feat: switch stage1 pick to new replay"
```

### Task 2: Calibrate frame-0 X/Z with contact-frame Y

**Files:**
- Modify: `replay_pick_reference.py:14-30, 104-180, 276-365`
- Modify: `scripts/calibrate_replay_pick_reference.py:42-145`
- Modify: `tests/test_replay_pick_reference.py:15-230`

- [ ] **Step 1: Write failing unit tests for the hybrid reference**

Extend the reference fields and construct a synthetic recording whose frame-0 rule point is `(0.90, -0.30, 0.75)` while the contact ray projects to Y `-0.27`:

```python
self.assertEqual(result.reference_frame_index, 0)
self.assertEqual(result.contact_frame_index, 158)
self.assertEqual(result.recorded_book_rule_point_base_m, (0.90, -0.30, 0.75))
self.assertAlmostEqual(result.recorded_contact_point_base_m[1], -0.27)
self.assertEqual(
    result.recorded_book_suction_point_base_m,
    (0.90, -0.27, 0.75),
)
```

Also update the JSON round-trip test so the contact frame, rule point, contact point and final hybrid point survive serialization.

- [ ] **Step 2: Run the reference tests and verify RED**

Run:

```bash
python3 -m unittest tests.test_replay_pick_reference -v
```

Expected: `ReplayPickReference` lacks contact metadata and calibration still returns frame-0 XYZ unchanged.

- [ ] **Step 3: Implement the hybrid reference function**

Add explicit fields:

```python
@dataclass(frozen=True)
class ReplayPickReference:
    asset_id: str
    reference_frame_index: int
    contact_frame_index: int
    recorded_book_rule_point_base_m: tuple[float, float, float]
    recorded_contact_point_base_m: tuple[float, float, float]
    recorded_book_suction_point_base_m: tuple[float, float, float]
    recorded_book_long_axis_base: tuple[float, float, float]
    recorded_book_long_extent_m: float
    recorded_book_short_extent_m: float
    hdf5_sha256: str
```

Make calibration accept `contact_frame_index` and `suction_roi_xywh`. Detect frame 0 normally, decode the selected contact-frame book mask, find the nearest book pixel to the new blue component, project that ray to the frame-0 cover height, and combine axes:

```python
contact = project_recorded_contact_to_cover(
    center_px=contact_px,
    intrinsics=intrinsics,
    torso_head_states=[contact_state],
    cover_z_base_m=rule_point[2],
    camera_to_base=camera_to_base,
).reference_contact_base_m
hybrid = (rule_point[0], contact[1], rule_point[2])
```

The CLI gains `--contact-frame` (default `158`) and `--suction-roi` arguments. Its overlay uses the contact image and marks both the detected contact pixel and the final hybrid XYZ.

- [ ] **Step 4: Run the reference and CLI tests and verify GREEN**

Run:

```bash
python3 -m unittest tests.test_replay_pick_reference tests.test_replay_calibration_cli -v
```

Expected: all tests pass and help output includes `--contact-frame` and `--suction-roi`.

- [ ] **Step 5: Commit hybrid calibration support**

```bash
git add replay_pick_reference.py scripts/calibrate_replay_pick_reference.py tests/test_replay_pick_reference.py tests/test_replay_calibration_cli.py
git commit -m "feat: calibrate replay lateral contact"
```

### Task 3: Generate the real reference, verify, document, and deploy

**Files:**
- Modify: `config/stage1_pick_reference.json`
- Modify: `README.md`
- Modify: `docs/CURRENT_STATUS.md`
- Modify: `docs/WORKLOG.md`
- Test: `tests/test_mission.py`

- [ ] **Step 1: Run the read-only real calibration**

Use frames 156–160 to choose the clearest contact image, then run the CLI with that fixed frame and an ROI surrounding the blue right suction assembly:

```bash
python3 scripts/calibrate_replay_pick_reference.py \
  --h5 /home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/20260819_libraryrobot_datareplay/pi05_wanda_dr1.2_20260819_195231.h5 \
  --reference-frame 0 \
  --contact-frame 158 \
  --suction-roi 80,80,100,144 \
  --output config/stage1_pick_reference.json \
  --overlay logs/stage1_pick_reference_overlay.jpg
```

Expected: JSON reports reference frame 0, contact frame 158, hybrid X/Z equal to the rule point, and hybrid Y equal to the projected contact Y. The overlay marks the blue suction tip at the book contact.

- [ ] **Step 2: Update fixtures and run the full offline suite**

Update `ReplayPickReference(...)` fixtures in mission/alignment tests with the new metadata, then run:

```bash
python3 -m unittest discover -s tests -v
git diff --check
```

Expected: all tests pass; no whitespace errors.

- [ ] **Step 3: Update operator-facing records**

Replace old README examples and current-status statements with the exact new asset, 333 frames, frame-20 D01 event, and hybrid alignment rule. Add a worklog entry stating that calibration and deployment are non-motion operations and that no Pick has yet been run with the new asset.

- [ ] **Step 4: Commit the generated reference and records**

```bash
git add config/stage1_pick_reference.json README.md docs/CURRENT_STATUS.md docs/WORKLOG.md tests
git commit -m "docs: activate new stage1 pick reference"
```

- [ ] **Step 5: Verify clean main and deploy without motion**

Run:

```bash
git status --short --branch
scripts/deploy_to_robot.sh
```

Expected: local `main` is clean before deployment; deployment copies the committed runtime to `/home/unix_ai/fpc`; it does not invoke `run.sh --book-pick`, navigation, arm, torso, base, or D01 commands.
