"""不连接机器人也能看的积木演示。"""

import math


class FakeVision:
    POSITIONS = {
        "banana": [1.0, 0.0, 0.82],
        "apple": [0.0, 1.0, 0.82],
        "orange": [-1.0, 0.0, 0.82],
        "pear": [0.0, -1.0, 0.82],
    }

    def find(self, fruit, frame="map"):
        point = self.POSITIONS.get(fruit)
        print(f"[视觉] 在 {frame} 中找到 {fruit}: {point}")
        return point


class FakeNavigation:
    def spin(self, angle):
        print(f"[导航] 原地转 {math.degrees(angle):.0f} 度")

    def approach_pose(self, target):
        print(f"[导航] 计算水果前方的接近点: {target}")
        return target

    def go(self, pose):
        print(f"[导航] 前往 {pose}")

    def go_home(self):
        print("[导航] 返回原点")

    def map_point_to_base(self, point):
        return point


class FakeArm:
    def pick(self, point):
        print(f"[机械臂] MoveIt: 预抓取 -> 抓取 {point} -> 抬起 -> 收回")

    def drop(self):
        print("[夹爪] 在原点打开")

    def restore_controller(self):
        pass
