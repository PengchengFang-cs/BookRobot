#!/usr/bin/env python3
"""从机器人已安装的 URDF 生成一个最小 MoveIt 参数文件。"""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from config import RIGHT_ARM_JOINTS, TOOL_FROM_WRIST_XYZ


ROOT = Path(__file__).resolve().parent
GENERATED = ROOT / "generated"
ROBOT_XACRO = Path(
    "/home/unix_ai/work/controller/install/mh2_description/share/"
    "mh2_description/description/urdf/mh2_description.urdf.xacro"
)


def make_urdf():
    xml = subprocess.check_output(("xacro", str(ROBOT_XACRO)), text=True)
    robot = ET.fromstring(xml)

    # 只留下底座、升降轴和右臂。其它机构与这次规划没有关系。
    needed_links = {
        "base_footprint",
        "base_link",
        "body_link",
        *(f"link_ra{i}" for i in range(8)),
    }
    for joint in list(robot.findall("joint")):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        if parent not in needed_links or child not in needed_links:
            robot.remove(joint)
    for link in list(robot.findall("link")):
        if link.attrib["name"] not in needed_links:
            robot.remove(link)

    # 这是 5m x 5m 空场的冒烟测试：去掉网格和碰撞体，让规划更直观。
    for link in robot.findall("link"):
        for tag in ("visual", "collision", "inertial"):
            for child in list(link.findall(tag)):
                link.remove(child)

    tool = ET.SubElement(robot, "link", {"name": "fruit_tool"})
    del tool  # 只需要一个空 link。
    joint = ET.SubElement(
        robot, "joint", {"name": "fruit_tool_joint", "type": "fixed"}
    )
    xyz = " ".join(str(value) for value in TOOL_FROM_WRIST_XYZ)
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": "link_ra7"})
    ET.SubElement(joint, "child", {"link": "fruit_tool"})

    path = GENERATED / "robot.urdf"
    ET.ElementTree(robot).write(path, encoding="unicode", xml_declaration=True)
    return path.read_text()


def make_params(robot_description):
    srdf = (ROOT / "robot.srdf").read_text()
    joint_limits = {
        name: {
            "has_velocity_limits": True,
            "max_velocity": 0.45,
            "has_acceleration_limits": True,
            "max_acceleration": 0.70,
        }
        for name in RIGHT_ARM_JOINTS
    }
    params = {
        "/fruit_move_group": {
            "ros__parameters": {
                "robot_description": robot_description,
                "robot_description_semantic": srdf,
                "robot_description_kinematics": {
                    "right_arm": {
                        "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                        "kinematics_solver_search_resolution": 0.01,
                        "kinematics_solver_timeout": 0.15,
                        "kinematics_solver_attempts": 5,
                    }
                },
                "robot_description_planning": {"joint_limits": joint_limits},
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
                "ompl": {
                    "planning_plugin": "ompl_interface/OMPLPlanner",
                    "request_adapters": (
                        "default_planner_request_adapters/ResolveConstraintFrames "
                        "default_planner_request_adapters/FixWorkspaceBounds "
                        "default_planner_request_adapters/FixStartStateBounds "
                        "default_planner_request_adapters/FixStartStateCollision "
                        "default_planner_request_adapters/AddTimeOptimalParameterization"
                    ),
                    "start_state_max_bounds_error": 0.15,
                    "planner_configs": {
                        "RRTConnectkConfigDefault": {
                            "type": "geometric::RRTConnect",
                            "range": 0.0,
                        }
                    },
                    "right_arm": {
                        "planner_configs": ["RRTConnectkConfigDefault"]
                    },
                },
                "allow_trajectory_execution": False,
                "moveit_manage_controllers": False,
                "moveit_controller_manager": (
                    "moveit_simple_controller_manager/MoveItSimpleControllerManager"
                ),
                "moveit_simple_controller_manager": {
                    "controller_names": [
                        "right_arm_joint_trajectory_position_controller"
                    ],
                    "right_arm_joint_trajectory_position_controller": {
                        "type": "FollowJointTrajectory",
                        "action_ns": "follow_joint_trajectory",
                        "default": True,
                        "joints": list(RIGHT_ARM_JOINTS),
                    },
                },
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": False,
                "publish_geometry_updates": False,
                "publish_state_updates": False,
                "publish_transforms_updates": False,
            }
        }
    }
    with (GENERATED / "moveit_params.yaml").open("w") as stream:
        yaml.safe_dump(params, stream, sort_keys=False, allow_unicode=True)


def main():
    GENERATED.mkdir(exist_ok=True)
    make_params(make_urdf())
    print("[准备] MoveIt 最小模型已生成")


if __name__ == "__main__":
    main()
