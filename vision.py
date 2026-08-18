"""视觉积木：5090 分割书本，机器人本地用深度计算吸取点。"""

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState

from book_frame import detect_book_frame
from book_geometry import CameraIntrinsics, decode_bbox_rle
from book_rpc import BookVisionClient
from config import (
    BODY_JOINT_STATES_TOPIC,
    CAMERA_INFO_TOPIC,
    COLOR_TOPIC,
    DEPTH_TOPIC,
    JOINT_STATES_TOPIC,
    MERGED_JOINT_STATES_TOPIC,
    VISION_TIMEOUT_S,
)
from geometry import apply_ros_transform, camera_point_to_base
from sensor_sync import SensorSynchronizer


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
        self.sensor_sync = SensorSynchronizer()
        self.last_capture_ns = -1
        self.book_client = book_client or BookVisionClient.from_config()

        sensor_qos = rclpy.qos.qos_profile_sensor_data
        node.create_subscription(Image, COLOR_TOPIC, self._color, sensor_qos)
        node.create_subscription(Image, DEPTH_TOPIC, self._depth, sensor_qos)
        node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._info, sensor_qos)
        node.create_subscription(JointState, JOINT_STATES_TOPIC, self._joints, sensor_qos)
        node.create_subscription(
            JointState, BODY_JOINT_STATES_TOPIC, self._joints, sensor_qos
        )

        # MoveIt 需要看到 body_joint 和双臂在同一条 JointState 消息中。
        self.merged_pub = node.create_publisher(
            JointState, MERGED_JOINT_STATES_TOPIC, 10
        )
        node.create_timer(0.05, self._publish_merged_joints)
        self.debug_path = Path(__file__).resolve().parent / "logs" / "last_detection.jpg"
        self.debug_path.parent.mkdir(parents=True, exist_ok=True)
        self.debug_path.unlink(missing_ok=True)

    def _color(self, message):
        self.color = message
        self.color_time = time.monotonic()
        self.sensor_sync.add_color(message, arrived_at_s=self.color_time)

    def _depth(self, message):
        self.depth = message
        self.depth_time = time.monotonic()
        self.sensor_sync.add_depth(message, arrived_at_s=self.depth_time)

    def _info(self, message):
        self.info = message
        self.sensor_sync.add_info(message, arrived_at_s=time.monotonic())

    def _joints(self, message):
        self.sensor_sync.add_joints(message)
        for name, value in zip(message.name, message.position):
            if np.isfinite(value):
                self.joints[name] = float(value)

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

    def _save_debug_overlay(self, color, observation, geometry):
        mask = decode_bbox_rle(
            image_shape=color.shape[:2],
            bbox=observation.bbox,
            counts=observation.rle_counts,
        )
        overlay = color.copy()
        overlay[mask] = (0, 220, 0)
        color[:] = cv2.addWeighted(color, 0.65, overlay, 0.35, 0.0)
        x, y, width, height = observation.bbox
        cv2.rectangle(color, (x, y), (x + width, y + height), (0, 255, 0), 3)
        point = geometry.suction_point
        cv2.putText(
            color,
            f"book suction=({point[0]:.3f},{point[1]:.3f},{point[2]:.3f})m",
            (x, max(30, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
        )
        cv2.imwrite(str(self.debug_path), color)

    def _detect_once(self, snapshot, joints):
        color = self.bridge.imgmsg_to_cv2(snapshot.color, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(snapshot.depth, desired_encoding="passthrough")
        if color.shape[:2] != depth.shape[:2]:
            return None
        depth = depth.astype(float)
        if snapshot.depth.encoding.upper() == "16UC1":
            depth /= 1000.0

        body = joints["body_joint"]
        head_yaw = joints["joint_head0"]
        head_pitch = joints["joint_head1"]
        captured_at_ns = snapshot.captured_at_ns

        geometry = detect_book_frame(
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
            on_result=lambda observation, geometry: self._save_debug_overlay(
                color, observation, geometry
            ),
        )
        if geometry is None:
            return None
        return geometry, captured_at_ns

    def find(self, target="book", frame="map"):
        if target != "book":
            raise ValueError(f"当前视觉只支持书本，不支持: {target}")
        started = time.monotonic()
        deadline = started + VISION_TIMEOUT_S
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
            # Consume before the RPC so a no-book result waits for a new frame
            # instead of repeatedly sending the identical capture.
            self.last_capture_ns = snapshot.captured_at_ns
            try:
                detected = self._detect_once(snapshot, joints)
                if detected is None:
                    continue
                geometry, captured_at_ns = detected
                point_base = geometry.suction_point
                if frame == "base_link":
                    result = point_base
                elif frame == "map":
                    tf = self.tf_buffer.lookup_transform(
                        "map",
                        "base_link",
                        Time(nanoseconds=captured_at_ns),
                    )
                    result = apply_ros_transform(point_base, tf.transform)
                else:
                    raise ValueError(f"不支持的坐标系: {frame}")
                print(
                    f"[视觉] book suction @ {frame}: "
                    f"x={result[0]:.3f}, y={result[1]:.3f}, z={result[2]:.3f}"
                )
                return result
            except Exception as error:
                self.node.get_logger().warning(f"这一帧不能用: {error}")
        return None
