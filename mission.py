"""把感知、移动和动作积木按任务顺序接起来。"""

from dataclasses import dataclass

from book_alignment import (
    build_replay_alignment_target,
    predict_book_after_base_motion,
    reassociate_book,
    select_coarse_book,
    select_replay_book,
)
from config import SCAN_ANGLE_RAD, SCAN_COUNT


BOOK_ALIGNMENT_X_TOLERANCE_M = 0.020
BOOK_ALIGNMENT_Y_TOLERANCE_M = 0.010


@dataclass(frozen=True)
class BookAlignmentRun:
    coarse: object
    precise: object
    final: object
    coarse_navigation: object
    precise_navigation: object
    z_offset_m: float
    xy_within_tolerance: bool


@dataclass(frozen=True)
class BookPickRun:
    alignment: BookAlignmentRun
    replay: object


def _format_residual(label, target):
    dx, dy, dz = target.residual_m
    return f"{label}: dx={dx:.3f} m, dy={dy:.3f} m, dz={dz:.3f} m"


def _format_navigation(label, navigation):
    return (
        f"{label}: 模式={navigation.mode}, 命令={navigation.command_count}, "
        f"odom dx={navigation.odom_dx_m:.3f} m, "
        f"dy={navigation.odom_dy_m:.3f} m, "
        f"yaw={navigation.imu_dyaw_rad:.3f} rad"
    )


def run_book_alignment_once(vision, navigator, replay_reference, say=print):
    """Run coarse visual approach followed by replay-image precise docking."""

    coarse = select_coarse_book(vision.find("book", frame="base_link"))
    say(
        "0.48 m 粗定位: "
        f"observed=({coarse.observed_m[0]:.3f}, "
        f"{coarse.observed_m[1]:.3f}, {coarse.observed_m[2]:.3f}) m"
    )
    coarse_navigation = navigator.align(
        reference=coarse.reference_m,
        observed=coarse.observed_m,
    )
    say(_format_navigation("粗定位运动反馈", coarse_navigation))

    initial_contact = coarse.book.suction_point
    predicted_after_coarse = predict_book_after_base_motion(
        initial_contact,
        odom_dx_m=coarse_navigation.odom_dx_m,
        odom_dy_m=coarse_navigation.odom_dy_m,
        imu_dyaw_rad=coarse_navigation.imu_dyaw_rad,
    )
    after_coarse_book = reassociate_book(
        vision.find("book", frame="base_link"),
        predicted_point_m=predicted_after_coarse,
        replay_reference=replay_reference,
    )
    precise = build_replay_alignment_target(
        book=after_coarse_book,
        replay_reference=replay_reference,
    )
    say(_format_residual("DataReplay 精确偏差", precise))
    precise_navigation = navigator.align(
        reference=precise.reference_m,
        observed=precise.observed_m,
    )
    say(_format_navigation("精确对位运动反馈", precise_navigation))

    predicted_final = predict_book_after_base_motion(
        precise.observed_m,
        odom_dx_m=precise_navigation.odom_dx_m,
        odom_dy_m=precise_navigation.odom_dy_m,
        imu_dyaw_rad=precise_navigation.imu_dyaw_rad,
    )
    final_book = reassociate_book(
        vision.find("book", frame="base_link"),
        predicted_point_m=predicted_final,
        replay_reference=replay_reference,
    )
    final = build_replay_alignment_target(
        book=final_book,
        replay_reference=replay_reference,
    )
    say(_format_residual("最终 DataReplay 残差", final))
    xy_within_tolerance = (
        abs(final.residual_m[0]) <= BOOK_ALIGNMENT_X_TOLERANCE_M
        and abs(final.residual_m[1]) <= BOOK_ALIGNMENT_Y_TOLERANCE_M
    )
    say(f"XY验收={'达标' if xy_within_tolerance else '未达标'}")
    return BookAlignmentRun(
        coarse=coarse,
        precise=precise,
        final=final,
        coarse_navigation=coarse_navigation,
        precise_navigation=precise_navigation,
        z_offset_m=precise.z_offset_m,
        xy_within_tolerance=xy_within_tolerance,
    )


def run_book_alignment_from_current_once(
    vision,
    navigator,
    replay_reference,
    say=print,
):
    """Skip coarse approach and precisely dock from the current base pose."""

    current_book = select_replay_book(
        vision.find("book", frame="base_link"),
        replay_reference=replay_reference,
    )
    precise = build_replay_alignment_target(
        book=current_book,
        replay_reference=replay_reference,
    )
    say(_format_residual("当前位置 DataReplay 精确偏差", precise))
    precise_navigation = navigator.align(
        reference=precise.reference_m,
        observed=precise.observed_m,
    )
    say(_format_navigation("精确对位运动反馈", precise_navigation))

    predicted_final = predict_book_after_base_motion(
        precise.observed_m,
        odom_dx_m=precise_navigation.odom_dx_m,
        odom_dy_m=precise_navigation.odom_dy_m,
        imu_dyaw_rad=precise_navigation.imu_dyaw_rad,
    )
    final_book = reassociate_book(
        vision.find("book", frame="base_link"),
        predicted_point_m=predicted_final,
        replay_reference=replay_reference,
    )
    final = build_replay_alignment_target(
        book=final_book,
        replay_reference=replay_reference,
    )
    say(_format_residual("最终 DataReplay 残差", final))
    xy_within_tolerance = (
        abs(final.residual_m[0]) <= BOOK_ALIGNMENT_X_TOLERANCE_M
        and abs(final.residual_m[1]) <= BOOK_ALIGNMENT_Y_TOLERANCE_M
    )
    say(f"XY验收={'达标' if xy_within_tolerance else '未达标'}")
    return BookAlignmentRun(
        coarse=None,
        precise=precise,
        final=final,
        coarse_navigation=None,
        precise_navigation=precise_navigation,
        z_offset_m=precise.z_offset_m,
        xy_within_tolerance=xy_within_tolerance,
    )


def run_book_pick_once(
    vision,
    navigator,
    replayer,
    replay_reference,
    say=print,
    coarse=False,
):
    """Align one book, apply the fixed Z handoff, and replay one Pick."""

    align = (
        run_book_alignment_once
        if coarse
        else run_book_alignment_from_current_once
    )
    alignment = align(vision, navigator, replay_reference, say=say)
    if not alignment.xy_within_tolerance:
        dx, dy, _ = alignment.final.residual_m
        raise RuntimeError(
            "最终对位未达标，不启动 DataReplay: "
            f"dx={dx:.3f} m (允许 ±{BOOK_ALIGNMENT_X_TOLERANCE_M:.3f}), "
            f"dy={dy:.3f} m (允许 ±{BOOK_ALIGNMENT_Y_TOLERANCE_M:.3f})"
        )
    say("开始按固定 Z 偏移执行 Stage-1 Pick DataReplay")
    replay = replayer.pick(alignment.final.z_offset_m)
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
