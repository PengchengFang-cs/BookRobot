"""导航积木：记住原点、转身、直走、回原点。"""

import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.msg import DynamicInterfaceGroupValues
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DriveOnHeading, Spin
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

from config import (
    APPROACH_DISTANCE_M,
    BASE_BUTTON_TOPIC,
    BASE_MODE_TOPIC,
    NAV_SPEED_M_S,
    NAV_TIMEOUT_S,
)
from geometry import apply_ros_transform, quaternion_from_yaw


class Navigation:
    def __init__(self, node):
        self.node = node
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.drive_client = ActionClient(
            node, DriveOnHeading, "/drive_on_heading"
        )
        # main.py 的接口检查只需要 wait_for_server，保留这个直观名字。
        self.nav_client = self.drive_client
        self.spin_client = ActionClient(node, Spin, "/spin")
        self.mode_pub = node.create_publisher(
            Float64MultiArray, BASE_MODE_TOPIC, 10
        )
        self.release_pressed = False
        self.release_seen = False
        self.release_until = time.monotonic() + 0.5
        node.create_subscription(
            DynamicInterfaceGroupValues,
            BASE_BUTTON_TOPIC,
            self._buttons,
            10,
        )
        node.create_timer(0.10, self._publish_base_mode)
        self.home = None

    def _buttons(self, message):
        """机身释放键缺少原厂桥接，这里直接接到公开模式接口。"""
        pressed = False
        for group, values in zip(
            message.interface_groups, message.interface_values
        ):
            if group != "button_sensor":
                continue
            for name, value in zip(values.interface_names, values.values):
                if name == "release_button_status":
                    pressed = value > 0.5
        if pressed and not self.release_pressed:
            self.release_seen = True
            self.release_until = time.monotonic() + 0.5
            print("[底盘] 检测到机身释放键")
        self.release_pressed = pressed

    def _publish_base_mode(self):
        message = Float64MultiArray()
        if time.monotonic() < self.release_until:
            message.data = [5.0]  # RELEASE
        else:
            message.data = [0.0]  # 左右轮速度模式
        self.mode_pub.publish(message)

    def wait_for_release(self):
        """等现场按一下释放键；这是唯一需要人的底盘上电步骤。"""
        print("[底盘] 请打开遥控器，并按一下机身释放键……")
        while rclpy.ok() and not self.release_seen:
            rclpy.spin_once(self.node, timeout_sec=0.10)

    def _wait_future(self, future, timeout):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("等待 ROS 结果超时")
        return future.result()

    def _run_action(self, client, goal, name, timeout):
        if not client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError(f"Nav2 的 {name} 没有启动")
        handle = self._wait_future(client.send_goal_async(goal), 8.0)
        if not handle.accepted:
            raise RuntimeError(f"Nav2 拒绝了{name}目标")

        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=3.0)
            raise RuntimeError(f"{name}等待结果超时")
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"{name}没有成功，状态={result.status}")

    def current_pose(self):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform("map", "base_link", Time())
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.pose.position.x = tf.transform.translation.x
                pose.pose.position.y = tf.transform.translation.y
                pose.pose.position.z = 0.0
                pose.pose.orientation = tf.transform.rotation
                return pose
            except Exception:
                rclpy.spin_once(self.node, timeout_sec=0.10)
        raise RuntimeError("找不到 map -> base_link，请确认 slam.service 正在运行")

    def remember_home(self):
        self.home = self.current_pose()
        print(
            f"[导航] 记住原点 x={self.home.pose.position.x:.3f}, "
            f"y={self.home.pose.position.y:.3f}"
        )

    def approach_pose(self, target_map):
        here = self.current_pose().pose.position
        dx = float(target_map[0]) - here.x
        dy = float(target_map[1]) - here.y
        distance = math.hypot(dx, dy)
        if distance < 0.05:
            raise RuntimeError("水果坐标离机器人中心太近")

        travel = max(0.0, distance - APPROACH_DISTANCE_M)
        ux, uy = dx / distance, dy / distance
        yaw = math.atan2(dy, dx)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = here.x + travel * ux
        goal.pose.position.y = here.y + travel * uy
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        return goal

    def go(self, pose):
        """空场里最简单的走法：面向目标，直走，再对准目标朝向。"""
        here = self.current_pose()
        dx = pose.pose.position.x - here.pose.position.x
        dy = pose.pose.position.y - here.pose.position.y
        distance = math.hypot(dx, dy)

        if distance >= 0.03:
            heading = math.atan2(dy, dx)
            first_turn = _short_angle(heading - _pose_yaw(here))
            if abs(first_turn) >= 0.02:
                self.spin(first_turn)
            self.drive(distance)

        final_turn = _short_angle(_pose_yaw(pose) - _pose_yaw(self.current_pose()))
        if abs(final_turn) >= 0.02:
            self.spin(final_turn)

        arrived = self.current_pose()
        error = math.hypot(
            pose.pose.position.x - arrived.pose.position.x,
            pose.pose.position.y - arrived.pose.position.y,
        )
        if error > 0.15:
            raise RuntimeError(f"直行结束后离目标还差 {error:.3f} m")
        print(f"[导航] 到达目标，位置误差 {error:.3f} m")

    def drive(self, distance):
        """沿机器人正前方直走。"""
        goal = DriveOnHeading.Goal()
        goal.target.x = float(distance)
        goal.speed = NAV_SPEED_M_S
        goal.time_allowance.sec = int(NAV_TIMEOUT_S)
        self._run_action(
            self.drive_client, goal, "/drive_on_heading", NAV_TIMEOUT_S + 5.0
        )

    def spin(self, angle):
        goal = Spin.Goal()
        goal.target_yaw = float(_short_angle(angle))
        goal.time_allowance.sec = 20
        self._run_action(self.spin_client, goal, "/spin", 25.0)

    def go_home(self):
        if self.home is None:
            raise RuntimeError("还没有记住原点")
        self.home.header.stamp = self.node.get_clock().now().to_msg()
        self.go(self.home)

    def check_command(self):
        """发送零转角 action，验证 Nav2 命令链但不让底盘位移。"""
        before = self.current_pose()
        self.spin(0.0)
        after = self.current_pose()
        moved = math.hypot(
            after.pose.position.x - before.pose.position.x,
            after.pose.position.y - before.pose.position.y,
        )
        turned = abs(_short_angle(_pose_yaw(after) - _pose_yaw(before)))
        if moved > 0.03 or turned > 0.03:
            raise RuntimeError(
                f"底盘本应保持不动，却移动 {moved:.3f} m、"
                f"旋转 {turned:.3f} rad"
            )
        print("[检查] Nav2 零转角真实 action 成功")

    def check_motion(self):
        """向前走 40 cm 再回原点，验证底盘确实会执行导航轨迹。"""
        start = self.current_pose()
        tf = self.tf_buffer.lookup_transform("map", "base_link", Time())
        target = apply_ros_transform((0.40, 0.0, 0.0), tf.transform)

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = target[0]
        goal.pose.position.y = target[1]
        goal.pose.orientation = start.pose.orientation
        self.go(goal)

        reached = self.current_pose()
        travelled = math.hypot(
            reached.pose.position.x - start.pose.position.x,
            reached.pose.position.y - start.pose.position.y,
        )
        if travelled < 0.25:
            raise RuntimeError(f"底盘只走了 {travelled:.3f} m")

        self.go_home()
        returned = self.current_pose()
        home_error = math.hypot(
            returned.pose.position.x - start.pose.position.x,
            returned.pose.position.y - start.pose.position.y,
        )
        if home_error > 0.12:
            raise RuntimeError(f"底盘返回原点还差 {home_error:.3f} m")
        print(
            f"[检查] 底盘真实往返成功，前进 {travelled:.3f} m，"
            f"返航误差 {home_error:.3f} m"
        )

    def map_point_to_base(self, point_map):
        tf = self.tf_buffer.lookup_transform("base_link", "map", Time())
        return apply_ros_transform(np.asarray(point_map, dtype=float), tf.transform)


def _pose_yaw(pose):
    q = pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _short_angle(angle):
    """把任意角度变成 -pi 到 pi 之间的最短转角。"""
    return math.atan2(math.sin(angle), math.cos(angle))
