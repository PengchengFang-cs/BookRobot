# FruitTest Book Perception Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace FruitTest HSV fruit detection with a 5090-produced book mask and Wanda-local RGB-D suction-point reconstruction while leaving navigation and arm behavior unchanged.

**Architecture:** `book_geometry.py` is a ROS-free numerical core that decodes bbox-local RLE masks and reconstructs one table-book suction point. `book_rpc.py` is the only gRPC/protobuf boundary and returns small `BookMask` values. `vision.py` keeps the existing ROS subscriptions and `find()` interface, delegates segmentation and geometry to those modules, and continues to transform the resulting point into `map` when requested.

**Tech Stack:** Python 3, NumPy, OpenCV, ROS 2/rclpy, gRPC/protobuf at runtime, unittest.

---

## File map

- Create `book_geometry.py`: mask/result data classes, RLE decoding, depth projection, cover PCA, fixed-offset suction point.
- Create `book_rpc.py`: configurable mTLS `VisionService/Infer` client for `scene_table_books_segmentation`.
- Modify `config.py`: endpoint, certificate/protobuf paths, identity fields, geometry thresholds and fixed insets.
- Modify `vision.py`: replace HSV detector with the injected/default book client and local geometry call.
- Create `test_book_perception.py`: perception-only ROS entrypoint that never constructs navigation or arm objects.
- Create `tests/test_book_geometry.py`: ROS-free numerical tests.
- Create `tests/test_book_rpc.py`: fake protobuf/channel tests; no network.
- Create `tests/test_book_vision.py`: stub client and synthetic RGB-D integration test.
- Modify `README.md`: document perception-only test configuration and note unchanged motion distances.

### Task 1: ROS-free mask and table-book geometry

**Files:**
- Create: `book_geometry.py`
- Create: `tests/test_book_geometry.py`

- [ ] **Step 1: Write failing RLE and synthetic-book tests**

Create tests that construct a tight rectangular bbox mask, encode it as alternating bbox-local row-major counts, and assert:

```python
mask = decode_bbox_rle(
    image_shape=(120, 160),
    bbox=(40, 30, 80, 60),
    counts=(0, 4800),
)
self.assertEqual(mask.shape, (120, 160))
self.assertEqual(int(mask.sum()), 4800)

result = reconstruct_table_book(
    observation=BookMask(
        confidence=0.9,
        bbox=(40, 30, 80, 60),
        rle_counts=(0, 4800),
        image_width=160,
        image_height=120,
    ),
    depth_m=synthetic_depth,
    intrinsics=CameraIntrinsics(fx=120.0, fy=120.0, cx=80.0, cy=60.0),
    camera_to_base=lambda point: point,
    long_inset_m=0.13,
    right_inset_m=0.10,
)
self.assertAlmostEqual(result.long_inset_m, 0.13)
self.assertAlmostEqual(result.right_inset_m, 0.10)
```

Also assert that malformed RLE, insufficient valid depth and a book too small for the two insets raise `BookGeometryError` with stable codes.

- [ ] **Step 2: Run tests and verify failure**

Run: `python3 -m unittest tests.test_book_geometry -v`

Expected: import failure because `book_geometry.py` does not exist.

- [ ] **Step 3: Implement the numerical core**

Define immutable values:

```python
@dataclass(frozen=True)
class BookMask:
    confidence: float
    bbox: tuple[int, int, int, int]
    rle_counts: tuple[int, ...]
    image_width: int
    image_height: int

@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

@dataclass(frozen=True)
class BookGeometry:
    suction_point: tuple[float, float, float]
    long_axis: tuple[float, float, float]
    short_axis_right_to_left: tuple[float, float, float]
    long_extent_m: float
    short_extent_m: float
    confidence: float
    long_inset_m: float
    right_inset_m: float
```

`decode_bbox_rle()` must validate image bounds, alternating runs and exact bbox area. `reconstruct_table_book()` must convert valid mask depth pixels to camera XYZ, transform them to `base_link`, retain the top-cover band, run 2-D PCA in base XY, orient the long axis from the nearer endpoint to the farther endpoint, orient the short axis from robot-right (lower base Y) to robot-left, and compute:

```python
suction_xy = (
    near_edge_center
    + long_axis_xy * long_inset_m
    + short_axis_right_to_left_xy * right_inset_m
)
```

The cover Z is the median of the retained cover points. Reject results when either fixed inset would leave less than `0.015 m` clearance at the opposite edge.

- [ ] **Step 4: Run geometry tests**

Run: `python3 -m unittest tests.test_book_geometry -v`

Expected: all geometry tests pass.

- [ ] **Step 5: Commit geometry core**

```bash
git add book_geometry.py tests/test_book_geometry.py
git commit -m "feat: reconstruct table book suction point"
```

### Task 2: 5090 scene-mask gRPC boundary

**Files:**
- Create: `book_rpc.py`
- Create: `tests/test_book_rpc.py`
- Modify: `config.py`

- [ ] **Step 1: Write failing codec/client tests**

Use fake protobuf message classes and a fake unary channel. Assert that `BookVisionClient.detect()`:

```python
self.assertEqual(request.task, "scene_table_books_segmentation")
self.assertEqual(request.header.expected_output_frame, "image")
self.assertEqual(request.captures[0].camera_id, "head_rgbd")
self.assertEqual(request.captures[0].color.encoding, "bgr8")
self.assertEqual(request.captures[0].color.payload, image.tobytes())
self.assertEqual(result[0].bbox, (10, 20, 30, 40))
self.assertEqual(result[0].rle_counts, (0, 1200))
```

Assert the RPC method is `/bookbot.vision.v2.VisionService/Infer`, the configured timeout is used, and non-book scene rows are ignored.

- [ ] **Step 2: Run tests and verify failure**

Run: `python3 -m unittest tests.test_book_rpc -v`

Expected: import failure because `book_rpc.py` does not exist.

- [ ] **Step 3: Add visual-service configuration**

Add constants to `config.py` with environment overrides:

```python
BOOK_VISION_ENDPOINT = os.getenv("BOOK_VISION_ENDPOINT", "127.0.0.1:7443")
BOOK_VISION_SERVER_NAME = os.getenv(
    "BOOK_VISION_SERVER_NAME", "planning-server.bookbot.internal"
)
BOOK_VISION_PROTO_DIR = os.getenv(
    "BOOK_VISION_PROTO_DIR", "/var/lib/bookbot/task3/release_inputs/vision_generated"
)
BOOK_VISION_CA = os.getenv("BOOK_VISION_CA", "/var/lib/bookbot/vision-mtls/ca.pem")
BOOK_VISION_CERT = os.getenv(
    "BOOK_VISION_CERT", "/var/lib/bookbot/vision-mtls/wanda-client.pem"
)
BOOK_VISION_KEY = os.getenv(
    "BOOK_VISION_KEY", "/var/lib/bookbot/vision-mtls/wanda-client.key"
)
BOOK_VISION_TIMEOUT_S = 3.0
BOOK_LONG_INSET_M = 0.13
BOOK_RIGHT_INSET_M = 0.10
BOOK_MIN_EDGE_CLEARANCE_M = 0.015
```

Add the exact environment-backed identity fields required by the reviewed service contract:

```python
BOOK_VISION_SOURCE = os.getenv("BOOK_VISION_SOURCE", "ruan-unified-vision")
BOOK_VISION_WORKER_ID = os.getenv("BOOK_VISION_WORKER_ID", "ruan-5090-worker-0")
BOOK_VISION_MODEL_VERSION = os.getenv("BOOK_VISION_MODEL_VERSION", "")
BOOK_VISION_CONFIG_HASH = os.getenv("BOOK_VISION_CONFIG_HASH", "")
BOOK_VISION_CALIBRATION_VERSION = os.getenv(
    "BOOK_VISION_CALIBRATION_VERSION", ""
)
```

The last three values must be set from the activated 5090/Wanda deployment. Do not store certificate bytes, keys or tokens in Git.

- [ ] **Step 4: Implement the client**

`BookVisionClient` dynamically imports `vision_v2_pb2` only from `BOOK_VISION_PROTO_DIR`, builds a secure gRPC channel from the three certificate files, and creates a unary call with:

```python
channel.unary_unary(
    "/bookbot.vision.v2.VisionService/Infer",
    request_serializer=lambda message: message.SerializeToString(),
    response_deserializer=pb2.InferResponse.FromString,
)
```

`detect(image_bgr, captured_at_ns, base_motion_epoch, head_motion_epoch)` sends one exact `bgr8` capture and returns sorted `BookMask` values for `semantic_class == "book"` and `scene_profile_id == "table_books_v1"`. No depth or robot-control field crosses this module.

- [ ] **Step 5: Run client tests**

Run: `python3 -m unittest tests.test_book_rpc -v`

Expected: all tests pass without opening a socket.

- [ ] **Step 6: Commit RPC boundary**

```bash
git add book_rpc.py config.py tests/test_book_rpc.py
git commit -m "feat: call 5090 book mask service"
```

### Task 3: Integrate the new perception path into FruitTest

**Files:**
- Modify: `vision.py`
- Create: `tests/test_book_vision.py`

- [ ] **Step 1: Write failing integration tests**

Construct `Vision` without ROS subscriptions through a small pure helper `_detect_book_frame(color, depth, info, joints)`. Inject a fake client returning one `BookMask`; assert the helper returns the synthetic `BookGeometry`. Add a second test where the client returns no masks and assert `None`.

- [ ] **Step 2: Run test and verify failure**

Run: `python3 -m unittest tests.test_book_vision -v`

Expected: failure because `_detect_book_frame` and client injection are absent.

- [ ] **Step 3: Replace HSV detection in `Vision`**

Change construction to:

```python
def __init__(self, node, tf_buffer, book_client=None):
    self.node = node
    self.tf_buffer = tf_buffer
    self.bridge = CvBridge()
    self.book_client = book_client or BookVisionClient.from_config()
```

Replace `_detect_once(fruit)` with `_detect_book_frame(color_bgr, depth_m, intrinsics, joints, captured_at_ns)`. It must obtain masks from the client, try them in descending confidence order, call `reconstruct_table_book()` with the current aligned depth, camera intrinsics and `camera_point_to_base`, save `logs/last_detection.jpg`, and return the first valid `BookGeometry`. Keep `find(target, frame="base_link")` and the existing `map <- base_link` TF transform; return only `geometry.suction_point` so navigation/arm interfaces remain unchanged.

- [ ] **Step 4: Run integration and existing unit tests**

Run:

```bash
python3 -m unittest tests.test_book_vision -v
python3 -m unittest discover -s tests -v
```

Expected: all tests pass; no ROS graph, network connection or motion command is created by the tests.

- [ ] **Step 5: Commit Vision integration**

```bash
git add vision.py tests/test_book_vision.py
git commit -m "feat: use book masks in FruitTest vision"
```

### Task 4: Add a perception-only test entrypoint and documentation

**Files:**
- Create: `test_book_perception.py`
- Modify: `README.md`

- [ ] **Step 1: Implement the perception-only entrypoint**

The script initializes only ROS, TF and `Vision`, then prints one JSON result:

```python
point = vision.find("book", frame="base_link")
print(json.dumps({"frame_id": "base_link", "suction_point_m": point}))
```

It must not import or construct `Navigation`, `Arm`, or `Mission`, so running it cannot dispatch motion through this repository.

- [ ] **Step 2: Document configuration and perception-only testing**

Update `README.md` with the endpoint `127.0.0.1:7443`, required environment-backed identity/certificate/protobuf paths, debug image location, and a command that exercises only perception. State explicitly that this code revision has not validated navigation or suction motion.

- [ ] **Step 3: Run full offline verification**

Run:

```bash
python3 -m unittest discover -s tests -v
python3 -m py_compile book_geometry.py book_rpc.py vision.py test_book_perception.py config.py
bash -n run.sh overlay_env.sh release_base.sh install_moveit_user.sh
git diff --check
```

Expected: all unit tests pass, compilation and shell syntax checks exit zero, and `git diff --check` prints nothing.

- [ ] **Step 4: Commit the test entrypoint and documentation**

```bash
git add test_book_perception.py README.md
git commit -m "docs: add book perception test entrypoint"
```
