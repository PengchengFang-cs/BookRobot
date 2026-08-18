# FruitTest Multi-Book Perception Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Return zero to five geometry-valid books at confidence `>=0.25` and render all returned masks in one debug image.

**Architecture:** Keep the RPC client responsible for decoding every 5090 instance. Make `book_frame.py` filter, sort, reconstruct and cap the usable results, then let `vision.py` transform each suction point and render the selected collection once.

**Tech Stack:** Python 3, unittest, NumPy, OpenCV, ROS 2/rclpy on Wanda.

---

### Task 1: Multi-book frame selection

**Files:**
- Modify: `config.py`
- Modify: `book_frame.py`
- Test: `tests/test_book_vision.py`

- [ ] **Step 1: Write failing selection tests**

Add tests that pass unsorted synthetic masks through `detect_book_frame(...)` and assert: confidence `<0.25` is absent, confidence `==0.25` is retained, four valid masks return four results, six valid masks return the five highest confidences, and an invalid high-confidence mask does not consume a result slot.

- [ ] **Step 2: Verify RED**

Run `python3 -m unittest tests.test_book_vision -v`; expect failures because the function currently returns one `BookGeometry` rather than a result collection.

- [ ] **Step 3: Implement minimal selection behavior**

Add `BOOK_MIN_CONFIDENCE = 0.25` and `BOOK_MAX_RESULTS = 5`. Introduce a frozen `DetectedBook` containing `observation` and `geometry`. Sort candidates by confidence, skip candidates below the threshold, reconstruct each remaining candidate, collect successful results until five, invoke a collection callback once, and return a tuple (empty when nothing is usable).

- [ ] **Step 4: Verify GREEN**

Run `python3 -m unittest tests.test_book_vision -v`; expect all selection tests to pass.

### Task 2: Multi-book Vision output and overlay

**Files:**
- Modify: `vision.py`
- Modify: `test_book_perception.py`
- Test: `tests/test_perception_entrypoint.py`

- [ ] **Step 1: Write failing output tests**

Update entrypoint tests to call `format_result(...)` with zero, two and five structured book results. Assert `ok`, `book_count`, descending confidence, bbox values and every `suction_point_m` in `base_link`.

- [ ] **Step 2: Verify RED**

Run `python3 -m unittest tests.test_perception_entrypoint -v`; expect failure because the formatter only accepts one point.

- [ ] **Step 3: Implement list output and one-pass overlay**

Add a `LocatedBook` result carrying observation, geometry, frame and transformed suction point. Make `Vision.find(...)` return a list of these results. Render every selected mask, bbox, index, confidence and suction point onto one copied color frame, then write `logs/last_detection.jpg` once. Make `format_result(...)` emit `{"ok": bool(books), "frame_id": "base_link", "book_count": len(books), "books": [...]}` and return exit code 2 only for an empty list.

- [ ] **Step 4: Verify GREEN and regression suite**

Run `python3 -m unittest discover -s tests -v`; expect all tests to pass.

### Task 3: Deploy and capture

**Files:**
- Update on Wanda only within `/home/unix_ai/fpc`
- Produce: `/home/unix_ai/fpc/logs/last_detection.jpg`

- [ ] **Step 1: Check deployment preconditions**

Confirm the local worktree contains only the intended committed changes and the robot project files match the previously deployed revision. Do not pull from GitHub and do not modify reference paths.

- [ ] **Step 2: Deploy from 5090 to Wanda**

Copy the changed project files from `/home/cvailab/fpc` to `/home/unix_ai/fpc` over SSH without deletion or force operations.

- [ ] **Step 3: Run one perception-only capture**

Run the existing temporary environment setup with `SensorSynchronizer(joint_buffer_size=512)` and the current listener identities. Do not instantiate navigation or arm controllers. Expect JSON with `book_count` from 0 through 5 and one row per returned book.

- [ ] **Step 4: Return the image**

Copy `/home/unix_ai/fpc/logs/last_detection.jpg` to `/home/cvailab/fpc/logs/last_detection-multi-book-20260818.jpg`, inspect it, and provide the local image link to the user.
