# DataReplay Image-Based Precise Docking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the incorrect final `x=0.48 m` alignment with a two-stage flow that uses `0.48 m` only for coarse approach and derives the final XYZ target from the recorded DataReplay book image and blue suction-center location.

**Architecture:** Add a ROS-free asset calibration module that extracts the blue suction center at the D01 frame, projects it through early unobstructed recorded depth, and stores one small asset-specific reference JSON. Extend the existing book geometry so that this recorded contact location can be transferred onto the currently detected book, then orchestrate coarse approach, odom-based same-book reassociation, precise XY alignment, and the existing fixed Z replay handoff as distinct phases.

**Tech Stack:** Python 3, dataclasses, NumPy, OpenCV, h5py, unittest, existing RGB-D/DINO-SAM client, ROS 2 Humble, navnav_final, V3 DataReplay.

---

### Task 1: Add the pure replay-reference calibration core

**Files:**
- Create: `replay_pick_reference.py`
- Create: `tests/test_replay_pick_reference.py`

- [ ] **Step 1: Write failing tests for reference data and image/depth helpers**

  Add synthetic tests that require these exact public interfaces:

  ```python
  from replay_pick_reference import (
      ReplayPickReference,
      find_blue_suction_center,
      scale_intrinsics,
      project_recorded_contact,
  )

  def test_scales_x_and_y_intrinsics_independently(self):
      scaled = scale_intrinsics(
          CameraIntrinsics(1000.0, 900.0, 960.0, 540.0),
          source_size=(1920, 1080),
          target_size=(224, 224),
      )
      self.assertAlmostEqual(scaled.fx, 1000.0 * 224.0 / 1920.0)
      self.assertAlmostEqual(scaled.fy, 900.0 * 224.0 / 1080.0)

  def test_finds_largest_new_blue_component_inside_roi(self):
      baseline = np.zeros((40, 60, 3), dtype=np.uint8)
      contact = baseline.copy()
      contact[20:30, 35:45] = (0, 0, 255)  # RGB blue
      self.assertEqual(
          find_blue_suction_center(
              baseline_rgb=baseline,
              contact_rgb=contact,
              roi_xywh=(30, 15, 20, 20),
          ),
          (39.5, 24.5),
      )

  def test_projects_multiple_early_depth_frames_and_uses_axis_median(self):
      result = project_recorded_contact(
          center_px=(2.0, 1.0),
          depths_mm=[depth_a, depth_b, depth_c],
          intrinsics=CameraIntrinsics(2.0, 2.0, 0.0, 0.0),
          torso_head_states=[(0.2, 0.0, 0.25)] * 3,
          camera_to_base=lambda point, *_state: point,
      )
      self.assertEqual(result.sample_count, 3)
      self.assertEqual(result.reference_contact_base_m, (1.0, 0.5, 1.0))
  ```

  Also cover JSON round-trip, malformed/missing blue components, invalid depth at all samples, and non-finite projected points.

- [ ] **Step 2: Run the new test module and verify it fails because the module is missing**

  Run:

  ```bash
  python3 -m unittest tests.test_replay_pick_reference -v
  ```

  Expected: `ModuleNotFoundError: No module named 'replay_pick_reference'`.

- [ ] **Step 3: Implement the minimal pure module**

  Define immutable values with these fields:

  ```python
  @dataclass(frozen=True)
  class ProjectedReplayContact:
      reference_contact_base_m: tuple[float, float, float]
      sample_count: int
      axis_spread_m: tuple[float, float, float]

  @dataclass(frozen=True)
  class ReplayPickReference:
      asset_id: str
      contact_frame_index: int
      early_frame_indices: tuple[int, ...]
      suction_center_px: tuple[float, float]
      reference_contact_base_m: tuple[float, float, float]
      long_inset_m: float
      right_inset_m: float
      sample_count: int
  ```

  `find_blue_suction_center(...)` must convert RGB to HSV, restrict processing to `roi_xywh`, require color change from the early frame, choose the largest connected component, and return its centroid in full-image coordinates. `project_recorded_contact(...)` must take a 5×5 patch around the centroid, use the median positive depth for each early frame, back-project with the scaled intrinsics, call the injected transform with that frame's torso/head state, and return per-axis medians and peak-to-peak spreads. Add `save_replay_pick_reference(path, reference)` and `load_replay_pick_reference(path)` using ordinary JSON with numeric arrays.

- [ ] **Step 4: Run the tests and verify they pass**

  Run:

  ```bash
  python3 -m unittest tests.test_replay_pick_reference -v
  ```

  Expected: all tests pass.

- [ ] **Step 5: Commit the calibration core**

  ```bash
  git add replay_pick_reference.py tests/test_replay_pick_reference.py
  git commit -m "feat: add replay contact calibration core"
  ```

### Task 2: Transfer an asset-specific contact point onto a detected book

**Files:**
- Modify: `book_geometry.py`
- Modify: `tests/test_book_geometry.py`

- [ ] **Step 1: Write failing tests for local book coordinates**

  Extend `tests/test_book_geometry.py` to require:

  ```python
  from book_geometry import contact_point_for_insets, insets_for_contact_point

  def test_recovers_and_reapplies_contact_insets(self):
      contact = (0.31, -0.08, geometry.suction_point[2])
      long_inset, right_inset = insets_for_contact_point(geometry, contact)
      rebuilt = contact_point_for_insets(
          geometry,
          long_inset_m=long_inset,
          right_inset_m=right_inset,
      )
      np.testing.assert_allclose(rebuilt, contact, atol=1e-9)

  def test_asset_contact_rotates_with_current_book_axes(self):
      point = contact_point_for_insets(
          rotated_geometry,
          long_inset_m=0.07,
          right_inset_m=0.04,
      )
      self.assertEqual(point[2], rotated_geometry.suction_point[2])
  ```

  Include one case proving the original default `0.13/0.10` reconstruction result is unchanged.

- [ ] **Step 2: Run the focused tests and verify the two new imports fail**

  ```bash
  python3 -m unittest tests.test_book_geometry -v
  ```

  Expected: import failure for `contact_point_for_insets`/`insets_for_contact_point`.

- [ ] **Step 3: Implement the two geometry functions without changing reconstruction defaults**

  Recover the robot-near/right corner from the existing geometry:

  ```python
  corner_xy = (
      np.asarray(geometry.suction_point[:2])
      - np.asarray(geometry.long_axis[:2]) * geometry.long_inset_m
      - np.asarray(geometry.short_axis_right_to_left[:2]) * geometry.right_inset_m
  )
  ```

  Reapply supplied insets along the same axes and use the current book cover Z. The inverse function must project `contact_xy-corner_xy` onto the two unit axes. Reject only non-finite inputs and inset values outside the reconstructed book extents.

- [ ] **Step 4: Run geometry tests and the existing perception tests**

  ```bash
  python3 -m unittest tests.test_book_geometry tests.test_book_frame tests.test_perception_entrypoint -v
  ```

  Expected: all tests pass and the legacy default suction point remains unchanged.

- [ ] **Step 5: Commit the geometry extension**

  ```bash
  git add book_geometry.py tests/test_book_geometry.py
  git commit -m "feat: map replay contact onto book geometry"
  ```

### Task 3: Build the HDF5 asset calibration command

**Files:**
- Modify: `replay_pick_reference.py`
- Create: `scripts/calibrate_replay_pick_reference.py`
- Modify: `tests/test_replay_pick_reference.py`
- Create: `config/stage1_pick_reference.json`

- [ ] **Step 1: Write a failing synthetic HDF5 calibration test**

  Create a temporary HDF5 with datasets matching the real asset:

  ```text
  observation/image
  observations/depth_head_rgbd
  observations/qpos_head
  observations/qpos_torso
  ```

  Inject a detector returning one known `BookGeometry`, place a blue rectangle in frame 300, and assert `calibrate_replay_pick_reference(...)` emits the correct `reference_contact_base_m`, `long_inset_m`, `right_inset_m`, contact frame, early indices, and sample count.

- [ ] **Step 2: Run the test and verify the high-level calibrator is missing**

  ```bash
  python3 -m unittest tests.test_replay_pick_reference -v
  ```

  Expected: failure because `calibrate_replay_pick_reference` is not defined.

- [ ] **Step 3: Implement asset loading and the non-motion CLI**

  Add this callable boundary:

  ```python
  def calibrate_replay_pick_reference(
      *, h5_path, asset_id, contact_frame_index, early_frame_indices,
      source_intrinsics, source_size, suction_roi_xywh,
      detect_recorded_book, camera_to_base,
  ) -> ReplayPickReference:
      with h5py.File(h5_path, "r") as recording:
          images = recording["observation/image"]
          depths = recording["observations/depth_head_rgbd"]
          heads = recording["observations/qpos_head"]
          torsos = recording["observations/qpos_torso"]
          center = find_blue_suction_center(
              baseline_rgb=images[early_frame_indices[0]],
              contact_rgb=images[contact_frame_index],
              roi_xywh=suction_roi_xywh,
          )
          intrinsics = scale_intrinsics(
              source_intrinsics,
              source_size=source_size,
              target_size=(images.shape[2], images.shape[1]),
          )
          projected = project_recorded_contact(
              center_px=center,
              depths_mm=[depths[index] for index in early_frame_indices],
              intrinsics=intrinsics,
              torso_head_states=[
                  (float(torsos[index, 0]), *map(float, heads[index]))
                  for index in early_frame_indices
              ],
              camera_to_base=camera_to_base,
          )
          book = detect_recorded_book(
              images[early_frame_indices[0]],
              depths[early_frame_indices[0]],
              intrinsics,
              float(torsos[early_frame_indices[0], 0]),
              tuple(map(float, heads[early_frame_indices[0]])),
          )
      long_inset, right_inset = insets_for_contact_point(
          book,
          projected.reference_contact_base_m,
      )
      return ReplayPickReference(
          asset_id=asset_id,
          contact_frame_index=contact_frame_index,
          early_frame_indices=tuple(early_frame_indices),
          suction_center_px=center,
          reference_contact_base_m=projected.reference_contact_base_m,
          long_inset_m=long_inset,
          right_inset_m=right_inset,
          sample_count=projected.sample_count,
      )
  ```

  The CLI defaults must be explicit:

  ```text
  --asset-id S1_TABLE_PICK_BOOK
  --contact-frame 300
  --early-frames 0,10,20,40,80
  --suction-roi 145,170,45,54
  --output config/stage1_pick_reference.json
  --overlay logs/stage1_pick_reference_overlay.jpg
  ```

  It must use the known 1920×1080 CameraInfo values from the current camera, scale them to the HDF5 dimensions, call the existing DINO/SAM book client on an early frame, reconstruct the book with existing depth geometry, derive local insets from the projected contact point, write the JSON, and draw the contact center/reference onto a two-panel early/contact overlay. It must not import ROS motion adapters or issue any command.

- [ ] **Step 4: Run synthetic tests and compile the CLI**

  ```bash
  python3 -m unittest tests.test_replay_pick_reference -v
  python3 -m py_compile replay_pick_reference.py scripts/calibrate_replay_pick_reference.py
  ```

  Expected: all tests pass and compilation exits zero.

- [ ] **Step 5: Commit the command before running it on the robot**

  ```bash
  git add replay_pick_reference.py scripts/calibrate_replay_pick_reference.py tests/test_replay_pick_reference.py
  git commit -m "feat: calibrate replay pick reference from hdf5"
  ```

  Do not add the output JSON yet; it is produced from the real read-only asset in Task 7.

### Task 4: Separate coarse and precise alignment models

**Files:**
- Modify: `book_alignment.py`
- Modify: `tests/test_book_alignment.py`

- [ ] **Step 1: Replace the old fixed-reference tests with failing two-stage tests**

  Require these interfaces:

  ```python
  coarse = select_coarse_book(books)
  precise = build_replay_alignment_target(
      book=coarse.book,
      replay_reference=reference,
  )
  predicted = predict_book_after_base_motion(
      coarse.observed_m,
      odom_dx_m=0.20,
      odom_dy_m=-0.01,
      imu_dyaw_rad=0.02,
  )
  same = reassociate_book(books_after_motion, predicted_point_m=predicted)
  ```

  Assert that:

  - coarse reference X is `APPROACH_DISTANCE_M`;
  - precise reference is the JSON's `reference_contact_base_m`, never `0.48`;
  - precise observed point uses `contact_point_for_insets(...)`;
  - Z offset equals precise observed Z minus replay reference Z;
  - SE(2) prediction uses `R(-yaw) * (p_old - translation)`;
  - reassociation selects the candidate nearest the predicted same-book point rather than the candidate nearest the replay reference.

- [ ] **Step 2: Run alignment tests and verify the new APIs fail**

  ```bash
  python3 -m unittest tests.test_book_alignment -v
  ```

  Expected: import/API failures for the new two-stage functions.

- [ ] **Step 3: Implement immutable coarse and precise targets**

  Preserve `BookAlignmentTarget` for the precise result, but remove `APPROACH_DISTANCE_M` from `STAGE1_PICK_REFERENCE_BASE_M`. Add a separate `COARSE_APPROACH_REFERENCE_X_M = APPROACH_DISTANCE_M`. `build_replay_alignment_target(...)` must use the current book's full `geometry`, the replay local insets, and replay reference point. Keep the existing ±30 mm Z contract for the replay offset, now applied only after the precise calculation.

  `predict_book_after_base_motion(...)` must preserve Z and apply the inverse planar robot motion. `reassociate_book(...)` must calculate candidate contact points with the replay insets before choosing the closest candidate.

- [ ] **Step 4: Run alignment and navigation unit tests**

  ```bash
  python3 -m unittest tests.test_book_alignment tests.test_book_navigation -v
  ```

  Expected: all tests pass; `book_navigation.py` still receives Z equal to reference Z and emits only X/Y motion.

- [ ] **Step 5: Commit the alignment split**

  ```bash
  git add book_alignment.py tests/test_book_alignment.py
  git commit -m "feat: separate coarse and replay alignment"
  ```

### Task 5: Orchestrate coarse approach, same-book lock, and precise alignment

**Files:**
- Modify: `mission.py`
- Modify: `tests/test_mission.py`

- [ ] **Step 1: Write failing orchestration tests**

  Replace the one-move expectation with a scripted three-capture test:

  ```text
  capture 1 → choose and lock the target
  navigator.align(coarse reference, coarse observed)
  capture 2 → reassociate the same target using odom prediction
  navigator.align(replay reference, current replay-contact point)
  capture 3 → reassociate and report final replay residual
  replayer.pick(precise z_offset)
  ```

  Assert exactly two navigation calls, three vision calls, no switch to a distractor nearer the replay reference, and that `replayer.pick(...)` receives the precise-stage Z offset rather than the initial generic suction-point Z. Add an align-only test proving `run_book_alignment_once` stops after final reporting and never constructs a replayer.

- [ ] **Step 2: Run mission tests and verify they fail on the old one-stage call order**

  ```bash
  python3 -m unittest tests.test_mission -v
  ```

  Expected: failures showing only two captures/one navigation call in the current implementation.

- [ ] **Step 3: Implement the two-stage mission flow**

  Change signatures to accept an already loaded replay reference:

  ```python
  def run_book_alignment_once(vision, navigator, replay_reference, say=print):
      initial_books = vision.find("book", frame="base_link")
      coarse = select_coarse_book(initial_books)
      coarse_navigation = navigator.align(
          reference=coarse.reference_m,
          observed=coarse.observed_m,
      )
      initial_contact = contact_point_for_insets(
          coarse.book.geometry,
          long_inset_m=replay_reference.long_inset_m,
          right_inset_m=replay_reference.right_inset_m,
      )
      predicted = predict_book_after_base_motion(
          initial_contact,
          odom_dx_m=coarse_navigation.odom_dx_m,
          odom_dy_m=coarse_navigation.odom_dy_m,
          imu_dyaw_rad=coarse_navigation.imu_dyaw_rad,
      )
      after_coarse = reassociate_book(
          vision.find("book", frame="base_link"),
          predicted_point_m=predicted,
          replay_reference=replay_reference,
      )
      precise = build_replay_alignment_target(
          book=after_coarse,
          replay_reference=replay_reference,
      )
      precise_navigation = navigator.align(
          reference=precise.reference_m,
          observed=precise.observed_m,
      )
      final_prediction = predict_book_after_base_motion(
          precise.observed_m,
          odom_dx_m=precise_navigation.odom_dx_m,
          odom_dy_m=precise_navigation.odom_dy_m,
          imu_dyaw_rad=precise_navigation.imu_dyaw_rad,
      )
      final_book = reassociate_book(
          vision.find("book", frame="base_link"),
          predicted_point_m=final_prediction,
          replay_reference=replay_reference,
      )
      final = build_replay_alignment_target(
          book=final_book,
          replay_reference=replay_reference,
      )
      return BookAlignmentRun(
          coarse=coarse,
          precise=precise,
          final=final,
          coarse_navigation=coarse_navigation,
          precise_navigation=precise_navigation,
          z_offset_m=precise.z_offset_m,
      )

  def run_book_pick_once(vision, navigator, replayer, replay_reference, say=print):
      alignment = run_book_alignment_once(
          vision,
          navigator,
          replay_reference,
          say=say,
      )
      replay = replayer.pick(alignment.z_offset_m)
      return BookPickRun(alignment=alignment, replay=replay)
  ```

  Return a result containing separate `coarse`, `precise`, and `final` measurements plus both navigation execution records. Log the three meanings distinctly: `0.48 m 粗定位`, `DataReplay 精确偏差`, and `最终 DataReplay 残差`. Preserve the precise stage's Z offset unchanged through the Pick call.

- [ ] **Step 4: Run mission, alignment, navigation, and replay tests**

  ```bash
  python3 -m unittest \
    tests.test_mission \
    tests.test_book_alignment \
    tests.test_book_navigation \
    tests.test_book_pick_replay -v
  ```

  Expected: all tests pass.

- [ ] **Step 5: Commit the mission rewrite**

  ```bash
  git add mission.py tests/test_mission.py
  git commit -m "feat: run coarse then replay-image docking"
  ```

### Task 6: Load the asset reference at the CLI boundary

**Files:**
- Modify: `config.py`
- Modify: `main.py`
- Modify: `README.md`
- Modify: `tests/test_main_modes.py`
- Modify: `tests/test_run_launcher.py`

- [ ] **Step 1: Write failing CLI tests**

  Require `--book-align` and `--book-pick` to load `REPLAY_PICK_REFERENCE_PATH`, pass the resulting `ReplayPickReference` into mission functions, and keep the existing book-vision environment path. Assert ordinary fruit execution remains unchanged.

- [ ] **Step 2: Run CLI tests and verify the missing reference argument/path failures**

  ```bash
  python3 -m unittest tests.test_main_modes tests.test_run_launcher -v
  ```

  Expected: failures because the current entrypoint neither loads nor passes a replay reference.

- [ ] **Step 3: Implement reference loading and document the commands**

  Add:

  ```python
  REPLAY_PICK_REFERENCE_PATH = Path(__file__).resolve().parent / "config" / "stage1_pick_reference.json"
  ```

  Load it lazily only for `--book-align`/`--book-pick`. Update README to state that `0.48 m` is coarse only, the JSON is the final DataReplay reference, and the calibration command is read-only/non-motion.

- [ ] **Step 4: Run CLI tests and the full local suite**

  ```bash
  python3 -m unittest tests.test_main_modes tests.test_run_launcher -v
  python3 -m unittest discover -s tests -v
  ```

  Expected: all tests pass.

- [ ] **Step 5: Commit CLI integration**

  ```bash
  git add config.py main.py README.md tests/test_main_modes.py tests/test_run_launcher.py
  git commit -m "feat: load replay docking reference"
  ```

### Task 7: Generate and review the real Stage 1 reference without motion

**Files:**
- Create: `config/stage1_pick_reference.json`
- Modify: `docs/WORKLOG.md`
- Modify: `docs/CURRENT_STATUS.md`

- [ ] **Step 1: Verify clean committed local main and deploy the non-motion calibration code**

  Run:

  ```bash
  git status --short
  ./scripts/deploy_to_robot.sh
  ```

  Expected: clean status before deployment; deployment reports the committed main revision. Do not run `--book-align` or `--book-pick`.

- [ ] **Step 2: Run the calibration command against the read-only real asset**

  On Wanda, run only:

  ```bash
  cd /home/unix_ai/fpc
  source scripts/book_vision_env.sh
  python3 scripts/calibrate_replay_pick_reference.py \
    --h5 /home/unix_ai/DataCollector/DataReplay_v3/v3_assets/takes/pi05_wanda_new1.2_20260816_034411.h5 \
    --output /home/unix_ai/fpc/config/stage1_pick_reference.json \
    --overlay /home/unix_ai/fpc/logs/stage1_pick_reference_overlay.jpg
  ```

  Expected: one JSON reference and one overlay; no ROS motion/controller/D01 call. The computed contact should be near the preliminary `(0.7155, -0.3913, 0.7128) m`, and sample count should be 5.

- [ ] **Step 3: Copy only the small JSON back, inspect the overlay, and commit the reference**

  Copy the JSON into local `config/stage1_pick_reference.json`; copy the overlay only into ignored local `logs/`. Verify the blue marker is centered on the user-identified blue suction part and the early-frame projection lies on the intended book contact area. Record actual pixel, XYZ, local insets, sample count, and axis spreads in `docs/WORKLOG.md` and make `CURRENT_STATUS.md` say that real motion has not yet been run.

- [ ] **Step 4: Run all non-motion verification**

  ```bash
  python3 -m unittest discover -s tests -v
  python3 -m py_compile \
    replay_pick_reference.py book_geometry.py book_alignment.py \
    book_navigation.py mission.py main.py \
    scripts/calibrate_replay_pick_reference.py
  git diff --check
  ```

  Expected: all tests and compilation pass; diff check is clean.

- [ ] **Step 5: Commit the real reference and records**

  ```bash
  git add config/stage1_pick_reference.json docs/WORKLOG.md docs/CURRENT_STATUS.md
  git commit -m "config: record stage1 replay pick reference"
  ```

### Task 8: Deploy the finished non-motion build and stop before hardware testing

**Files:**
- No new source files

- [ ] **Step 1: Verify the final local main is clean**

  ```bash
  git status --short --branch
  git log -1 --oneline
  ```

  Expected: clean `main` with the reference commit at HEAD.

- [ ] **Step 2: Deploy with the repository script**

  ```bash
  ./scripts/deploy_to_robot.sh
  ```

  Expected: committed files copied to `/home/unix_ai/fpc`; GitHub is not used.

- [ ] **Step 3: Run robot-side non-motion tests only**

  ```bash
  ssh tsinghuaBot 'cd /home/unix_ai/fpc && python3 -m unittest \
    tests.test_replay_pick_reference \
    tests.test_book_geometry \
    tests.test_book_alignment \
    tests.test_book_navigation \
    tests.test_mission \
    tests.test_book_pick_replay -v'
  ```

  Expected: all selected tests pass. Do not invoke `run.sh --book-align`, `run.sh --book-pick`, navigation adapters, torso, arm, or D01.

- [ ] **Step 4: Report the exact reference and request one new physical authorization**

  Report the calibrated contact pixel, `base_link` XYZ, book-local insets, test count, deployed commit, and overlay path. The next hardware action must be a separately authorized coarse-only motion; it must not combine coarse motion, precise motion, and Pick in one first test.
