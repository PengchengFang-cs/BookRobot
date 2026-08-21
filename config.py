"""FruitTest 的所有可调参数。

第一次演示时，如果水果颜色、支架高度或夹爪位置不合适，只改这个文件。
"""

import math
import os
from pathlib import Path


# 说出这些词时，程序会把它们换成统一的英文名字。
FRUITS = {
    "banana": {
        "words": ("香蕉", "banana"),
        "hsv": (((18, 80, 80), (38, 255, 255)),),
    },
    "apple": {
        "words": ("苹果", "apple"),
        # 红色横跨 HSV 的 0 度，所以需要两段。
        "hsv": (
            ((0, 90, 70), (12, 255, 255)),
            ((168, 90, 70), (180, 255, 255)),
        ),
    },
    "orange": {
        "words": ("橙子", "橘子", "orange"),
        "hsv": (((5, 100, 80), (22, 255, 255)),),
    },
    "pear": {
        "words": ("梨", "梨子", "pear"),
        "hsv": (((35, 60, 55), (88, 255, 255)),),
    },
}


# 真机 ROS 接口。
COLOR_TOPIC = "/head_rgbd/color/image_raw"
DEPTH_TOPIC = "/head_rgbd/depth/image_raw"
CAMERA_INFO_TOPIC = "/head_rgbd/color/camera_info"
JOINT_STATES_TOPIC = "/joint_states"
BODY_JOINT_STATES_TOPIC = "/lifting_controller/joint_states"
MERGED_JOINT_STATES_TOPIC = "/fruit_test/joint_states"
VOICE_TOPIC = "/aiui_pkg/audio_topic"
TEXT_COMMAND_TOPIC = "/fruit_test/command"
BASE_BUTTON_TOPIC = "/movebase_gpio_controller/gpio_states"
BASE_MODE_TOPIC = "/fruit_movebase_mode_controller/commands"

# AIUI 当前只发布“正在识别/正在播放”等状态，没有发布识别文字。
# 所以再用机器人已经装好的 Whisper 和默认麦克风阵列听一次。
WHISPER_MODEL = "base"
WHISPER_RECORD_SECONDS = 4.0
WHISPER_CHANNELS = 1
WHISPER_MIN_PEAK = 600
WHISPER_WAV = "/tmp/fruit_test_voice.wav"


# 视觉：鲜艳模型比真实水果容易识别，这正适合冒烟测试。
MIN_COLOR_AREA_PX = 900
# 水果模型不会占满画面。这个限制可以排除绿色柜门、整面墙等大背景。
MAX_COLOR_AREA_FRACTION = 0.08
MAX_COLOR_BOX_FRACTION = 0.35
DEPTH_PATCH_PX = 15
MIN_DEPTH_M = 0.20
MAX_DEPTH_M = 2.50
VISION_TIMEOUT_S = 20.0

# RTX 5090 上的固定桌面书本分割服务。机器人通过本机 SSH 转发访问它；
# 模型只返回二维 bbox/mask，深度和三维坐标始终留在机器人本机。
BOOK_VISION_ENDPOINT = os.getenv("BOOK_VISION_ENDPOINT", "127.0.0.1:7443")
BOOK_VISION_SERVER_NAME = os.getenv(
    "BOOK_VISION_SERVER_NAME", "planning-server.bookbot.internal"
)
BOOK_VISION_PROTO_DIR = os.getenv(
    "BOOK_VISION_PROTO_DIR",
    "/var/lib/bookbot/task3/release_inputs/vision_generated",
)
BOOK_VISION_CA = os.getenv(
    "BOOK_VISION_CA", "/var/lib/bookbot/vision-mtls/ca.pem"
)
BOOK_VISION_CERT = os.getenv(
    "BOOK_VISION_CERT", "/var/lib/bookbot/vision-mtls/wanda-client.pem"
)
BOOK_VISION_KEY = os.getenv(
    "BOOK_VISION_KEY", "/var/lib/bookbot/vision-mtls/wanda-client.key"
)
BOOK_VISION_SOURCE = os.getenv("BOOK_VISION_SOURCE", "ruan-unified-vision")
BOOK_VISION_WORKER_ID = os.getenv(
    "BOOK_VISION_WORKER_ID", "ruan-5090-worker-0"
)
BOOK_VISION_MODEL_VERSION = os.getenv("BOOK_VISION_MODEL_VERSION", "")
BOOK_VISION_CONFIG_HASH = os.getenv("BOOK_VISION_CONFIG_HASH", "")
BOOK_VISION_CALIBRATION_VERSION = os.getenv(
    "BOOK_VISION_CALIBRATION_VERSION", ""
)
BOOK_VISION_TIMEOUT_S = 10.0

# 第一阶段平放书本的固定吸取点。
BOOK_LONG_INSET_M = 0.13
BOOK_RIGHT_INSET_M = 0.10
BOOK_MIN_EDGE_CLEARANCE_M = 0.015
BOOK_MIN_CONFIDENCE = 0.25
BOOK_MAX_RESULTS = 5

# 每个 Pick DataReplay 资产从录制图像标定出的真实吸盘接触参考。
REPLAY_PICK_REFERENCE_PATH = (
    Path(__file__).resolve().parent / "config" / "stage1_pick_reference.json"
)
REPLAY_PLACE_REFERENCE_PATH = (
    Path(__file__).resolve().parent / "config" / "stage1_place_reference.json"
)


# 导航：水果约在一米外，机器人停在水果前方，让机械臂还留有空间。
APPROACH_DISTANCE_M = 0.48
SCAN_ANGLE_RAD = math.pi / 2.0
SCAN_COUNT = 4
NAV_TIMEOUT_S = 60.0
NAV_SPEED_M_S = 0.15


# MoveIt / 右臂。
MOVE_GROUP = "right_arm"
MOVEIT_PLAN_SERVICE = "/plan_kinematic_path"
RIGHT_TRAJECTORY_ACTION = (
    "/right_arm_joint_trajectory_position_controller/follow_joint_trajectory"
)
RAW_ARM_CONTROLLER = "dual_arm_forward_position_controller"
LEFT_TRAJECTORY_CONTROLLER = "left_arm_joint_trajectory_position_controller"
RIGHT_TRAJECTORY_CONTROLLER = "right_arm_joint_trajectory_position_controller"
RIGHT_ARM_JOINTS = tuple(f"joint_ra{i}" for i in range(8))
TOOL_LINK = "fruit_tool"

# fruit_tool 在两指之间。单位是米，方向来自机器人的 link_ra7 坐标系。
TOOL_FROM_WRIST_XYZ = (-0.0044, -0.0038, -0.135)

# 顶部抓取：fruit_tool 的坐标轴与 base_link 对齐。
TOOL_ORIENTATION_XYZW = (0.0, 0.0, 0.0, 1.0)
ORIENTATION_TOLERANCE_RAD = 0.40
PREGRASP_UP_M = 0.18
GRASP_UP_M = 0.025
LIFT_UP_M = 0.20
PLAN_TIME_S = 6.0
ARM_TIMEOUT_S = 90.0
ARM_SPEED = 0.16


# 夹爪消息的范围是 0~1：0 打开，1 闭合。
GRIPPER_TOPIC = "/right_gripper/gripper_command"
GRIPPER_MODE = "parallel"
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 1.0
GRIPPER_VELOCITY = 0.35
GRIPPER_EFFORT = 0.65
# 夹爪速度设为 0.35，从全开到全闭大约需要 1 / 0.35 ≈ 2.9 秒。
GRIPPER_WAIT_S = 3.2
