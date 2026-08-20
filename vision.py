"""视觉积木：5090 分割书本，机器人本地用深度计算吸取点。"""

from dataclasses import dataclass
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray

from book_frame import detect_book_frame
from book_geometry import CameraIntrinsics, decode_bbox_rle
from book_rpc import BookVisionClient
from cart_geometry import (
    CartGeometryError,
    reconstruct_cart_body_target,
    reconstruct_cart_top_platform,
    select_leftmost_cart_platform,
)
from config import (
    BODY_JOINT_STATES_TOPIC,
    CAMERA_INFO_TOPIC,
    COLOR_TOPIC,
    DEPTH_TOPIC,
    JOINT_STATES_TOPIC,
    MERGED_JOINT_STATES_TOPIC,
    VISION_TIMEOUT_S,
    VISION_WARMUP_S,
)
from geometry import apply_ros_transform, camera_point_to_base
from sensor_sync import (
    SensorSynchronizer,
    collect_successful_results,
    spin_until_counter_advances,
)


@dataclass(frozen=True)
class LocatedBook:
    observation: object
    geometry: object
    frame_id: str
    suction_point: tuple[float, float, float]


@dataclass(frozen=True)
class LocatedCart:
    observations: tuple[object, ...]
    body_target: object
    platform: object
    frame_id: str
    captured_at_ns: int
    debug_image: str | None = None


@dataclass(frozen=True)
class CapturedCartFrame:
    snapshot: object
    joints: dict
    scan_angle_deg: int


class Vision:
    def __init__(self, node, tf_buffer, book_client=None):
        self.node = node
        self.tf_buffer = tf_buffer
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.info = None
        self.color_time = 0.0
        self.depth_time = 0.0
        self.joints = {}
        self.body_feedback_sequence = 0
        self.sensor_sync = SensorSynchronizer()
        self.last_capture_ns = -1
        self.book_client = book_client or BookVisionClient.from_config()

        sensor_qos = rclpy.qos.qos_profile_sensor_data
        rgbd_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(Image, COLOR_TOPIC, self._color, rgbd_qos)
        node.create_subscription(Image, DEPTH_TOPIC, self._depth, rgbd_qos)
        node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._info, rgbd_qos)
        node.create_subscription(JointState, JOINT_STATES_TOPIC, self._joints, sensor_qos)
        node.create_subscription(
            JointState,
            BODY_JOINT_STATES_TOPIC,
            self._body_joints,
            sensor_qos,
        )

        # MoveIt 需要看到 body_joint 和双臂在同一条 JointState 消息中。
        self.merged_pub = node.create_publisher(
            JointState, MERGED_JOINT_STATES_TOPIC, 10
        )
        self.head_command_pub = node.create_publisher(
            Float64MultiArray,
            "/head_forward_position_controller/commands",
            10,
        )
        node.create_timer(0.05, self._publish_merged_joints)
        self.debug_path = Path(__file__).resolve().parent / "logs" / "last_detection.jpg"
        self.debug_path.parent.mkdir(parents=True, exist_ok=True)
        self.debug_path.unlink(missing_ok=True)

    def _color(self, message):
        self.color = message
        self.color_time = time.monotonic()
        self.sensor_sync.add_color(
            message,
            arrived_at_s=self.color_time,
            received_at_ns=self.node.get_clock().now().nanoseconds,
        )

    def _depth(self, message):
        self.depth = message
        self.depth_time = time.monotonic()
        self.sensor_sync.add_depth(
            message,
            arrived_at_s=self.depth_time,
            received_at_ns=self.node.get_clock().now().nanoseconds,
        )

    def _info(self, message):
        self.info = message
        self.sensor_sync.add_info(
            message,
            arrived_at_s=time.monotonic(),
            received_at_ns=self.node.get_clock().now().nanoseconds,
        )

    def _joints(self, message):
        self.sensor_sync.add_joints(message)
        for name, value in zip(message.name, message.position):
            if np.isfinite(value):
                self.joints[name] = float(value)

    def _body_joints(self, message):
        self._joints(message)
        if any(
            name == "body_joint" and np.isfinite(value)
            for name, value in zip(message.name, message.position)
        ):
            self.body_feedback_sequence += 1

    def spin_until_fresh_body(self, spin_once, *, timeout_s=0.05):
        return spin_until_counter_advances(
            lambda: self.body_feedback_sequence,
            spin_once,
            timeout_s=timeout_s,
        )

    def _publish_merged_joints(self):
        if not self.joints:
            return
        from config import RIGHT_ARM_JOINTS

        wanted = {"body_joint", *RIGHT_ARM_JOINTS}
        names = sorted(name for name in self.joints if name in wanted)
        if "body_joint" not in names or len(names) != 9:
            return
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.name = names
        message.position = [self.joints[name] for name in message.name]
        self.merged_pub.publish(message)

    def right_arm_positions(self):
        from config import RIGHT_ARM_JOINTS

        if not all(name in self.joints for name in RIGHT_ARM_JOINTS):
            return None
        return {name: self.joints[name] for name in RIGHT_ARM_JOINTS}

    def set_head_pose(self, *, yaw_rad, pitch_rad=0.25, timeout_s=3.0):
        """Move the head to one scan pose and wait until feedback reaches it."""

        message = Float64MultiArray()
        message.data = [float(yaw_rad), float(pitch_rad)]
        deadline = time.monotonic() + float(timeout_s)
        while rclpy.ok() and time.monotonic() < deadline:
            self.head_command_pub.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.10)
            if (
                abs(self.joints.get("joint_head0", 99.0) - float(yaw_rad)) <= 0.02
                and abs(self.joints.get("joint_head1", 99.0) - float(pitch_rad)) <= 0.02
            ):
                return True
        raise RuntimeError("头部没有到达书桌扫描角度")

    def scan_books_to_robot_right(self, angles_deg=(30, 45, 60, 75)):
        """Scan toward the table without rotating the chassis."""

        books = []
        try:
            for angle_deg in angles_deg:
                self.set_head_pose(yaw_rad=-np.deg2rad(float(angle_deg)))
                detected = self.find("book", frame="base_link")
                if detected:
                    books.extend(detected)
        finally:
            self.set_head_pose(yaw_rad=0.0)
        return tuple(books)

    def _save_debug_overlay(self, color, books):
        overlay = color.copy()
        colors = (
            (0, 220, 0),
            (255, 120, 0),
            (0, 180, 255),
            (220, 0, 220),
            (255, 200, 0),
        )
        for index, book in enumerate(books):
            observation = book.observation
            mask = decode_bbox_rle(
                image_shape=color.shape[:2],
                bbox=observation.bbox,
                counts=observation.rle_counts,
            )
            overlay[mask] = colors[index]
        color[:] = cv2.addWeighted(color, 0.65, overlay, 0.35, 0.0)
        for index, book in enumerate(books):
            observation = book.observation
            geometry = book.geometry
            draw_color = colors[index]
            x, y, width, height = observation.bbox
            cv2.rectangle(
                color, (x, y), (x + width, y + height), draw_color, 3
            )
            point = geometry.suction_point
            cv2.putText(
                color,
                (
                    f"#{index + 1} conf={observation.confidence:.2f} "
                    f"p=({point[0]:.2f},{point[1]:.2f},{point[2]:.2f})m"
                ),
                (x, max(30, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                draw_color,
                2,
            )
        cv2.imwrite(str(self.debug_path), color)

    def _detect_once(self, snapshot, joints):
        color, depth = self._snapshot_arrays(snapshot)
        if color is None:
            return None

        body = joints["body_joint"]
        head_yaw = joints["joint_head0"]
        head_pitch = joints["joint_head1"]
        captured_at_ns = snapshot.captured_at_ns

        books = detect_book_frame(
            book_client=self.book_client,
            color_bgr=color,
            depth_m=depth,
            intrinsics=CameraIntrinsics(
                fx=float(snapshot.info.k[0]),
                fy=float(snapshot.info.k[4]),
                cx=float(snapshot.info.k[2]),
                cy=float(snapshot.info.k[5]),
            ),
            camera_to_base=lambda point: camera_point_to_base(
                point, body, head_yaw, head_pitch
            ),
            captured_at_ns=captured_at_ns,
            base_motion_epoch=f"fruittest-base-capture-{captured_at_ns}",
            head_motion_epoch=f"fruittest-head-capture-{captured_at_ns}",
        )
        if not books:
            return None
        self._save_debug_overlay(color, books)
        return books, captured_at_ns

    def _snapshot_arrays(self, snapshot):
        color = self.bridge.imgmsg_to_cv2(snapshot.color, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(snapshot.depth, desired_encoding="passthrough")
        if color.shape[:2] != depth.shape[:2]:
            return None, None
        depth = depth.astype(float)
        if snapshot.depth.encoding.upper() == "16UC1":
            depth /= 1000.0
        return color, depth

    def _detect_cart_once(
        self,
        snapshot,
        joints,
        *,
        rpc_captured_at_ns=None,
        debug_path=None,
    ):
        color, depth = self._snapshot_arrays(snapshot)
        if color is None:
            return None
        body = joints["body_joint"]
        head_yaw = joints["joint_head0"]
        head_pitch = joints["joint_head1"]
        captured_at_ns = snapshot.captured_at_ns
        rpc_captured_at_ns = (
            time.time_ns()
            if rpc_captured_at_ns is None
            else int(rpc_captured_at_ns)
        )
        observations = self.book_client.detect_cart(
            color,
            captured_at_ns=rpc_captured_at_ns,
            base_motion_epoch=f"fruittest-base-capture-{captured_at_ns}",
            head_motion_epoch=f"fruittest-head-capture-{captured_at_ns}",
        )
        if not observations:
            return None
        platform_candidates = []
        body_candidates = []
        for observation in observations:
            try:
                geometry_args = dict(
                    observation=observation,
                    depth_m=depth,
                    intrinsics=CameraIntrinsics(
                        fx=float(snapshot.info.k[0]),
                        fy=float(snapshot.info.k[4]),
                        cx=float(snapshot.info.k[2]),
                        cy=float(snapshot.info.k[5]),
                    ),
                    camera_to_base=lambda point: camera_point_to_base(
                        point, body, head_yaw, head_pitch
                    ),
                )
                if observation.semantic_class == "cart_body":
                    body_candidates.append(reconstruct_cart_body_target(
                        **geometry_args
                    ))
                    platform_candidates.append(reconstruct_cart_top_platform(
                        **geometry_args
                    ))
            except CartGeometryError as error:
                print(f"[视觉] 小推车几何不可用: {error}")
        body_target = (
            max(body_candidates, key=lambda candidate: candidate.confidence)
            if body_candidates
            else None
        )
        platform = select_leftmost_cart_platform(platform_candidates)
        debug_path = self._save_cart_debug_overlay(
            color,
            observations,
            body_target,
            platform,
            debug_path=debug_path,
        )
        return LocatedCart(
            observations=tuple(observations),
            body_target=body_target,
            platform=platform,
            frame_id="base_link",
            captured_at_ns=captured_at_ns,
            debug_image=str(debug_path),
        )

    def _save_cart_debug_overlay(
        self,
        color,
        observations,
        body_target,
        platform,
        *,
        debug_path=None,
    ):
        overlay = color.copy()
        for observation in observations:
            mask = decode_bbox_rle(
                image_shape=color.shape[:2],
                bbox=observation.bbox,
                counts=observation.rle_counts,
            )
            draw_color = (
                (0, 220, 255)
                if observation.semantic_class == "cart_platform"
                else (255, 120, 0)
            )
            overlay[mask] = draw_color
            x, y, width, height = observation.bbox
            cv2.rectangle(color, (x, y), (x + width, y + height), draw_color, 3)
            cv2.putText(
                color,
                f"{observation.semantic_class} {observation.confidence:.2f}",
                (x, max(30, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                draw_color,
                2,
            )
        color[:] = cv2.addWeighted(color, 0.65, overlay, 0.35, 0.0)
        if body_target is not None:
            cv2.circle(color, body_target.center_pixel, 12, (0, 0, 255), -1)
            cv2.putText(
                color,
                "cart navigation target",
                (body_target.center_pixel[0] + 14, body_target.center_pixel[1] - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
            )
        if platform is not None:
            if platform.outline_pixels:
                cv2.polylines(
                    color,
                    [np.asarray(platform.outline_pixels, dtype=np.int32)],
                    True,
                    (0, 255, 0),
                    3,
                )
            for index, pixel in enumerate(platform.slot_pixels, 1):
                radius = 14 if index == 1 else 9
                draw_color = (0, 0, 255) if index == 1 else (255, 0, 255)
                cv2.circle(color, pixel, radius, draw_color, -1)
                cv2.putText(
                    color,
                    f"slot {index}",
                    (pixel[0] + 10, pixel[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    draw_color,
                    2,
                )
        debug_path = self.debug_path if debug_path is None else Path(debug_path)
        cv2.imwrite(str(debug_path), color)
        return debug_path

    def capture_cart_frame(self, *, scan_angle_deg, timeout_s=3.0):
        """Capture one synchronized RGB-D frame without calling the 5090."""

        started = time.monotonic()
        deadline = started + float(timeout_s)
        needed_joints = ("body_joint", "joint_head0", "joint_head1")
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.10)
            selected = self.sensor_sync.select(
                arrived_after_s=started,
                captured_after_ns=self.last_capture_ns,
                required_joints=needed_joints,
            )
            if selected is None:
                continue
            snapshot, joints = selected
            self.last_capture_ns = snapshot.captured_at_ns
            raw_color = self.bridge.imgmsg_to_cv2(
                snapshot.color,
                desired_encoding="bgr8",
            )
            raw_path = self.debug_path.with_name(
                f"cart_raw_{int(scan_angle_deg):03d}.jpg"
            )
            cv2.imwrite(str(raw_path), raw_color)
            return CapturedCartFrame(
                snapshot=snapshot,
                joints=dict(joints),
                scan_angle_deg=int(scan_angle_deg),
            )
        return None

    def detect_cart_frames_queued(self, frames):
        """Process the captured scan frames in order after rotation has stopped."""

        frames = tuple(frame for frame in frames if frame is not None)
        if not frames:
            return ()

        def detect(frame):
            debug_path = self.debug_path.with_name(
                f"cart_scan_{frame.scan_angle_deg:03d}.jpg"
            )
            last_error = None
            for attempt in range(1, 4):
                try:
                    return self._detect_cart_once(
                        frame.snapshot,
                        frame.joints,
                        rpc_captured_at_ns=time.time_ns(),
                        debug_path=debug_path,
                    )
                except Exception as error:
                    last_error = error
                    if attempt < 3:
                        time.sleep(0.2 * attempt)
            cause = getattr(last_error, "__cause__", None)
            detail = f"; cause={cause}" if cause is not None else ""
            self.node.get_logger().warning(
                f"小推车并行检测帧三次失败: {last_error}{detail}"
            )
            print(
                "[视觉] 小推车并行检测帧三次失败: "
                f"{type(last_error).__name__}: {last_error}{detail}"
            )
            return None

        return tuple(detect(frame) for frame in frames)

    def find_cart(self):
        """Return one cart-loading observation without any robot motion."""

        started = time.monotonic()
        deadline = started + VISION_TIMEOUT_S
        needed_joints = ("body_joint", "joint_head0", "joint_head1")
        warmup_deadline = min(deadline, started + VISION_WARMUP_S)
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.10)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.10)
            selected = self.sensor_sync.select(
                arrived_after_s=started,
                captured_after_ns=self.last_capture_ns,
                required_joints=needed_joints,
            )
            if selected is None:
                continue
            snapshot, joints = selected
            self.last_capture_ns = snapshot.captured_at_ns
            try:
                result = self._detect_cart_once(snapshot, joints)
                if result is not None:
                    return result
                print("[视觉] 当前帧没有检测到可用的小推车整体深度点")
            except Exception as error:
                self.node.get_logger().warning(f"小推车检测帧不能用: {error}")
                print(f"[视觉] 小推车检测帧不能用: {type(error).__name__}: {error}")
        return None

    def find_cart_samples(self, *, successful_samples=1, maximum_attempts=3):
        """Return immediately on the first successful cart measurement."""

        def report(attempt, success_count, found):
            if found:
                print(
                    f"[视觉] 推车稳定采样 {success_count}/{successful_samples} "
                    f"(尝试 {attempt}/{maximum_attempts})"
                )
            else:
                print(
                    f"[视觉] 推车第 {attempt}/{maximum_attempts} 次没有结果，继续"
                )

        def find_usable_platform():
            result = self.find_cart()
            return result if result is not None and result.platform is not None else None

        return collect_successful_results(
            find_usable_platform,
            successful_samples=successful_samples,
            maximum_attempts=maximum_attempts,
            on_attempt=report,
        )

    def find(self, target="book", frame="map"):
        if target != "book":
            raise ValueError(f"当前视觉只支持书本，不支持: {target}")
        started = time.monotonic()
        deadline = started + VISION_TIMEOUT_S
        needed_joints = ("body_joint", "joint_head0", "joint_head1")
        warmup_deadline = min(deadline, started + VISION_WARMUP_S)
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.10)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.10)
            selected = self.sensor_sync.select(
                arrived_after_s=started,
                captured_after_ns=self.last_capture_ns,
                required_joints=needed_joints,
            )
            if selected is None:
                continue
            snapshot, joints = selected
            # Consume before the RPC so a no-book result waits for a new frame
            # instead of repeatedly sending the identical capture.
            self.last_capture_ns = snapshot.captured_at_ns
            try:
                detected = self._detect_once(snapshot, joints)
                if detected is None:
                    print("[视觉] 当前帧没有可用的书本几何")
                    continue
                books, captured_at_ns = detected
                transform = None
                if frame == "map":
                    tf = self.tf_buffer.lookup_transform(
                        "map",
                        "base_link",
                        Time(nanoseconds=captured_at_ns),
                    )
                    transform = tf.transform
                elif frame != "base_link":
                    raise ValueError(f"不支持的坐标系: {frame}")
                results = []
                for index, book in enumerate(books, 1):
                    point = book.geometry.suction_point
                    if transform is not None:
                        point = apply_ros_transform(point, transform)
                    located = LocatedBook(
                        observation=book.observation,
                        geometry=book.geometry,
                        frame_id=frame,
                        suction_point=tuple(float(value) for value in point),
                    )
                    results.append(located)
                    print(
                        f"[视觉] book #{index} suction @ {frame}: "
                        f"x={point[0]:.3f}, y={point[1]:.3f}, z={point[2]:.3f}"
                    )
                return results
            except Exception as error:
                self.node.get_logger().warning(f"这一帧不能用: {error}")
                print(f"[视觉] 这一帧不能用: {type(error).__name__}: {error}")
        print(
            "[视觉] 检测超时，缓存数量: "
            f"color={len(self.sensor_sync.colors)}, "
            f"depth={len(self.sensor_sync.depths)}, "
            f"info={len(self.sensor_sync.infos)}"
        )
        return []

    def find_samples(
        self,
        target="book",
        frame="map",
        *,
        successful_samples=1,
        maximum_attempts=3,
    ):
        """Collect distinct successful detections, retrying missed frames."""

        def report(attempt, success_count, found):
            if found:
                print(
                    f"[视觉] 稳定采样 {success_count}/{successful_samples} "
                    f"(尝试 {attempt}/{maximum_attempts})"
                )
            else:
                print(
                    f"[视觉] 第 {attempt}/{maximum_attempts} 次没有检测结果，继续"
                )

        return collect_successful_results(
            lambda: self.find(target, frame=frame),
            successful_samples=successful_samples,
            maximum_attempts=maximum_attempts,
            on_attempt=report,
        )
