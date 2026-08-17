"""机械臂积木：MoveIt 只负责规划，轨迹控制器负责真实执行。"""

import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from unix_msgs.msg import GripperCommand

from config import (
    ARM_SPEED,
    ARM_TIMEOUT_S,
    GRASP_UP_M,
    GRIPPER_CLOSE,
    GRIPPER_EFFORT,
    GRIPPER_MODE,
    GRIPPER_OPEN,
    GRIPPER_TOPIC,
    GRIPPER_VELOCITY,
    GRIPPER_WAIT_S,
    LEFT_TRAJECTORY_CONTROLLER,
    LIFT_UP_M,
    MOVE_GROUP,
    MOVEIT_PLAN_SERVICE,
    ORIENTATION_TOLERANCE_RAD,
    PLAN_TIME_S,
    PREGRASP_UP_M,
    RAW_ARM_CONTROLLER,
    RIGHT_ARM_JOINTS,
    RIGHT_TRAJECTORY_ACTION,
    RIGHT_TRAJECTORY_CONTROLLER,
    TOOL_LINK,
    TOOL_ORIENTATION_XYZW,
)


class Arm:
    def __init__(self, node, joint_source):
        self.node = node
        self.joint_source = joint_source
        self.stow_joints = None
        self.controllers_ready = False
        self.plan_client = node.create_client(GetMotionPlan, MOVEIT_PLAN_SERVICE)
        self.switch_client = node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.trajectory_client = ActionClient(
            node, FollowJointTrajectory, RIGHT_TRAJECTORY_ACTION
        )
        self.gripper_pub = node.create_publisher(GripperCommand, GRIPPER_TOPIC, 10)

    def _wait_future(self, future, timeout):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("等待机械臂 ROS 结果超时")
        return future.result()

    def remember_stow(self):
        for _ in range(50):
            joints = self.joint_source()
            if joints is not None:
                self.stow_joints = dict(joints)
                print("[机械臂] 已记住启动时的收纳姿态")
                return
            rclpy.spin_once(self.node, timeout_sec=0.10)
        raise RuntimeError("没有收到右臂 joint_states")

    def _switch(self, activate, deactivate):
        if not self.switch_client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("找不到 controller_manager/switch_controller")
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        # 重启 FruitTest 时轨迹控制器可能已经是 active；BEST_EFFORT 会把
        # “已经切好了”当成成功，随后真实轨迹 action 仍会验证右臂是否可用。
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = Duration(sec=8)
        response = self._wait_future(self.switch_client.call_async(request), 12.0)
        if not response.ok:
            raise RuntimeError("机械臂控制器切换失败")

    def prepare(self):
        if self.controllers_ready:
            return
        if self.stow_joints is None:
            self.remember_stow()
        self._switch(
            (LEFT_TRAJECTORY_CONTROLLER, RIGHT_TRAJECTORY_CONTROLLER),
            (RAW_ARM_CONTROLLER,),
        )
        if not self.trajectory_client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("右臂 FollowJointTrajectory 没有启动")
        self.controllers_ready = True
        print("[机械臂] 轨迹控制器已准备好")

    def restore_controller(self):
        """需要回到原机器人遥控模式时可手工调用。"""
        if not self.controllers_ready:
            return
        self._switch(
            (RAW_ARM_CONTROLLER,),
            (LEFT_TRAJECTORY_CONTROLLER, RIGHT_TRAJECTORY_CONTROLLER),
        )
        self.controllers_ready = False

    def _base_request(self, constraints):
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name = MOVE_GROUP
        if hasattr(plan, "pipeline_id"):
            plan.pipeline_id = "ompl"
        plan.planner_id = "RRTConnectkConfigDefault"
        plan.num_planning_attempts = 4
        plan.allowed_planning_time = PLAN_TIME_S
        plan.max_velocity_scaling_factor = ARM_SPEED
        plan.max_acceleration_scaling_factor = ARM_SPEED
        plan.start_state.is_diff = True
        plan.workspace_parameters.header.frame_id = "base_link"
        plan.workspace_parameters.min_corner.x = -2.0
        plan.workspace_parameters.min_corner.y = -2.0
        plan.workspace_parameters.min_corner.z = 0.0
        plan.workspace_parameters.max_corner.x = 2.0
        plan.workspace_parameters.max_corner.y = 2.0
        plan.workspace_parameters.max_corner.z = 2.5
        plan.goal_constraints = [constraints]
        return request

    def _pose_constraints(self, position):
        constraints = Constraints()
        constraints.name = "fruit_pose"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.025, 0.025, 0.025]
        box_pose = Pose()
        box_pose.position.x = float(position[0])
        box_pose.position.y = float(position[1])
        box_pose.position.z = float(position[2])
        box_pose.orientation.w = 1.0

        volume = BoundingVolume()
        volume.primitives = [box]
        volume.primitive_poses = [box_pose]
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = TOOL_LINK
        position_constraint.constraint_region = volume
        position_constraint.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = "base_link"
        orientation.link_name = TOOL_LINK
        orientation.orientation.x = TOOL_ORIENTATION_XYZW[0]
        orientation.orientation.y = TOOL_ORIENTATION_XYZW[1]
        orientation.orientation.z = TOOL_ORIENTATION_XYZW[2]
        orientation.orientation.w = TOOL_ORIENTATION_XYZW[3]
        orientation.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        orientation.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        orientation.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        orientation.weight = 0.5

        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation]
        return constraints

    def _joint_constraints(self, positions):
        constraints = Constraints()
        constraints.name = "stow"
        for name in RIGHT_ARM_JOINTS:
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(positions[name])
            joint.tolerance_above = 0.015
            joint.tolerance_below = 0.015
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def _plan(self, constraints):
        if not self.plan_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("MoveIt /plan_kinematic_path 没有启动")
        response = self._wait_future(
            self.plan_client.call_async(self._base_request(constraints)),
            PLAN_TIME_S + 8.0,
        )
        answer = response.motion_plan_response
        if answer.error_code.val != 1:
            raise RuntimeError(f"MoveIt 规划失败，错误码={answer.error_code.val}")
        trajectory = answer.trajectory.joint_trajectory
        if not trajectory.points:
            raise RuntimeError("MoveIt 返回了空轨迹")
        if set(trajectory.joint_names) != set(RIGHT_ARM_JOINTS):
            raise RuntimeError(
                "MoveIt 轨迹不是纯右臂: " + ", ".join(trajectory.joint_names)
            )
        return trajectory

    def _execute(self, trajectory):
        trajectory.header.stamp.sec = 0
        trajectory.header.stamp.nanosec = 0
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(sec=5)
        handle = self._wait_future(
            self.trajectory_client.send_goal_async(goal), timeout=8.0
        )
        if not handle.accepted:
            raise RuntimeError("右臂轨迹控制器拒绝了 MoveIt 轨迹")
        result = self._wait_future(handle.get_result_async(), ARM_TIMEOUT_S)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"机械臂轨迹执行失败，状态={result.status}")
        # 让新的 joint_states 进入 MoveIt，再规划下一小段。
        for _ in range(4):
            rclpy.spin_once(self.node, timeout_sec=0.08)

    def move_to_point(self, point):
        self._execute(self._plan(self._pose_constraints(point)))

    def move_to_stow(self):
        self._execute(self._plan(self._joint_constraints(self.stow_joints)))

    def check_plan(self):
        """只规划一次典型的预抓取、抓取和抬起，不执行。"""
        points = (
            ("预抓取", (0.45, -0.05, 1.00)),
            ("抓取", (0.45, -0.05, 0.845)),
            ("抬起", (0.45, -0.05, 1.045)),
        )
        for name, point in points:
            trajectory = self._plan(self._pose_constraints(point))
            last_time = trajectory.points[-1].time_from_start
            seconds = last_time.sec + last_time.nanosec / 1_000_000_000
            print(
                f"[检查] MoveIt {name}规划成功，8 个右臂关节，"
                f"{len(trajectory.points)} 点，预计 {seconds:.1f} 秒"
            )

    def check_controller(self):
        """发送“保持当前位置”的轨迹，验证真实控制链但不让右臂位移。"""
        before = self.joint_source()
        if before is None:
            raise RuntimeError("没有右臂当前位置")

        self.prepare()
        trajectory = JointTrajectory()
        trajectory.joint_names = list(RIGHT_ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [before[name] for name in RIGHT_ARM_JOINTS]
        point.velocities = [0.0] * len(RIGHT_ARM_JOINTS)
        point.time_from_start = Duration(sec=2)
        trajectory.points = [point]
        self._execute(trajectory)

        after = self.joint_source()
        moved = max(abs(after[name] - before[name]) for name in RIGHT_ARM_JOINTS)
        if moved > 0.02:
            raise RuntimeError(f"右臂本应保持不动，却变化了 {moved:.3f} rad")
        print(f"[检查] 右臂真实轨迹控制链成功，最大变化 {moved:.4f} rad")

    def check_motion(self):
        """让一个关节小幅往返，再让夹爪开合一次。"""
        start = dict(self.stow_joints)
        target = dict(start)
        target["joint_ra3"] += 0.05

        self.prepare()
        self._execute(self._plan(self._joint_constraints(target)))
        reached = self.joint_source()
        moved = abs(reached["joint_ra3"] - start["joint_ra3"])
        if moved < 0.02:
            raise RuntimeError(f"右臂只变化了 {moved:.3f} rad")

        self.move_to_stow()
        returned = self.joint_source()
        return_error = max(
            abs(returned[name] - start[name]) for name in RIGHT_ARM_JOINTS
        )
        if return_error > 0.03:
            raise RuntimeError(f"右臂回收误差 {return_error:.3f} rad")

        self.gripper(GRIPPER_OPEN)
        self.gripper(GRIPPER_CLOSE)
        self.gripper(GRIPPER_OPEN)
        print(
            f"[检查] 右臂真实往返和夹爪开合成功，"
            f"关节变化 {moved:.3f} rad，回收误差 {return_error:.3f} rad"
        )

    def gripper(self, value):
        message = GripperCommand()
        message.mode = GRIPPER_MODE
        message.command = JointState()
        message.command.header.stamp = self.node.get_clock().now().to_msg()
        message.command.name = ["joint_rf"]
        message.command.position = [float(value)]
        message.command.velocity = [GRIPPER_VELOCITY]
        message.command.effort = [GRIPPER_EFFORT]
        end = time.monotonic() + GRIPPER_WAIT_S
        while time.monotonic() < end:
            self.gripper_pub.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.10)

    def pick(self, target_base):
        self.prepare()
        target = np.asarray(target_base, dtype=float)
        pregrasp = target + np.array((0.0, 0.0, PREGRASP_UP_M))
        grasp = target + np.array((0.0, 0.0, GRASP_UP_M))
        lift = grasp + np.array((0.0, 0.0, LIFT_UP_M))

        self.gripper(GRIPPER_OPEN)
        self.move_to_point(pregrasp)
        self.move_to_point(grasp)
        self.gripper(GRIPPER_CLOSE)
        self.move_to_point(lift)
        self.move_to_stow()

    def drop(self):
        # 在启动时的右手位置松开；原点处在右手下方放一个小篮子即可。
        self.gripper(GRIPPER_OPEN)
