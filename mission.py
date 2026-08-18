"""把感知、移动和动作积木按任务顺序接起来。"""

from dataclasses import dataclass

from book_alignment import select_alignment_book
from config import SCAN_ANGLE_RAD, SCAN_COUNT


@dataclass(frozen=True)
class BookAlignmentRun:
    initial: object
    final: object
    command_count: int
    height_alignment: object


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
    height_alignment = navigator.align(
        reference=initial.reference_m,
        observed=initial.observed_m,
    )
    say(
        f"导航对位动作完成，共执行 {height_alignment.command_count} 条 Y/Z/X 命令"
    )
    say(
        "高度补偿: "
        f"目标升降={height_alignment.target_torso_m:.3f} m, "
        f"实际升降={height_alignment.actual_torso_m:.3f} m, "
        f"有效Z残差={height_alignment.effective_z_residual_m:.3f} m"
    )

    final = select_alignment_book(vision.find("book", frame="base_link"))
    say(_format_residual("最终偏差", final))
    return BookAlignmentRun(
        initial,
        final,
        height_alignment.command_count,
        height_alignment,
    )


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
