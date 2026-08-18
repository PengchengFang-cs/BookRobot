"""把感知、移动和动作积木按任务顺序接起来。"""

from dataclasses import dataclass

from book_alignment import select_alignment_book
from config import SCAN_ANGLE_RAD, SCAN_COUNT


BOOK_ALIGNMENT_XY_TOLERANCE_M = 0.010


@dataclass(frozen=True)
class BookAlignmentRun:
    initial: object
    final: object
    command_count: int
    z_offset_m: float
    navigation_mode: str
    odom_dx_m: float
    odom_dy_m: float
    imu_dyaw_rad: float
    xy_within_tolerance: bool


@dataclass(frozen=True)
class BookPickRun:
    alignment: BookAlignmentRun
    replay: object


def _format_residual(label, target):
    dx, dy, dz = target.residual_m
    return f"{label}: dx={dx:.3f} m, dy={dy:.3f} m, dz={dz:.3f} m"


def run_book_alignment_once(vision, navigator, say=print):
    """Align one detected book to the recorded DataReplay pick point."""

    initial = select_alignment_book(vision.find("book", frame="base_link"))
    say(
        "选择吸取点 "
        f"x={initial.observed_m[0]:.3f}, "
        f"y={initial.observed_m[1]:.3f}, "
        f"z={initial.observed_m[2]:.3f}"
    )
    say(_format_residual("初始偏差", initial))
    navigation = navigator.align(
        reference=initial.reference_m,
        observed=initial.observed_m,
    )
    say(
        f"导航对位动作完成，模式={navigation.mode}，"
        f"共执行 {navigation.command_count} 条 X/Y 命令"
    )
    say(
        "运动反馈: "
        f"odom dx={navigation.odom_dx_m:.3f} m, "
        f"dy={navigation.odom_dy_m:.3f} m, "
        f"yaw={navigation.imu_dyaw_rad:.3f} rad"
    )
    say(f"Pipeline 固定Z偏移={initial.z_offset_m:.3f} m")

    final = select_alignment_book(vision.find("book", frame="base_link"))
    say(_format_residual("最终偏差", final))
    xy_within_tolerance = (
        abs(final.residual_m[0]) <= BOOK_ALIGNMENT_XY_TOLERANCE_M
        and abs(final.residual_m[1]) <= BOOK_ALIGNMENT_XY_TOLERANCE_M
    )
    say(f"XY验收={'达标' if xy_within_tolerance else '未达标'}")
    return BookAlignmentRun(
        initial,
        final,
        navigation.command_count,
        initial.z_offset_m,
        navigation.mode,
        navigation.odom_dx_m,
        navigation.odom_dy_m,
        navigation.imu_dyaw_rad,
        xy_within_tolerance,
    )


def run_book_pick_once(vision, navigator, replayer, say=print):
    """Align one book, apply the fixed Z handoff, and replay one Pick."""

    alignment = run_book_alignment_once(vision, navigator, say=say)
    say("开始按固定 Z 偏移执行 Stage-1 Pick DataReplay")
    replay = replayer.pick(alignment.z_offset_m)
    say(
        f"Pick 回放完成: frames={replay.frames_sent}, "
        f"torso target={replay.torso_target_m:.3f} m, "
        f"actual={replay.torso_actual_m:.3f} m, "
        f"D01 holding={replay.d01_holding}"
    )
    return BookPickRun(alignment=alignment, replay=replay)


def run_one_fruit(fruit, vision, navigation, arm, say=print):
    say(f"开始找 {fruit}")

    target_map = None
    for direction in range(SCAN_COUNT):
        books = vision.find(fruit, frame="map")
        if books:
            target_map = books[0].suction_point
            break
        if direction + 1 < SCAN_COUNT:
            say("这一面没有，转九十度继续看")
            navigation.spin(SCAN_ANGLE_RAD)

    if target_map is None:
        say("四个方向都没有找到")
        navigation.go_home()
        return False

    say("看到了，现在走到水果前面")
    navigation.go(navigation.approach_pose(target_map))

    # 走动后再看一次；如果太近看不到，就用刚才记住的地图坐标。
    books = vision.find(fruit, frame="base_link")
    if books:
        target_base = books[0].suction_point
    else:
        target_base = navigation.map_point_to_base(target_map)

    say("开始用 MoveIt 规划抓取")
    arm.pick(target_base)
    say("拿到了，回到原点")
    navigation.go_home()
    arm.drop()
    say("任务完成，可以再说一种水果")
    return True
