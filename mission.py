"""把感知、移动和动作积木按任务顺序接起来。"""

from dataclasses import dataclass
from statistics import median

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
BOOK_VISION_SUCCESSFUL_SAMPLES = 3
BOOK_VISION_MAXIMUM_ATTEMPTS = 5
BOOK_ALIGNMENT_MAXIMUM_CORRECTIONS = 3


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


@dataclass(frozen=True)
class BookPlaceRun:
    navigation: object
    replay: object


@dataclass(frozen=True)
class BookPickPlaceRun:
    pick: BookPickRun
    place: BookPlaceRun


@dataclass(frozen=True)
class StableLocatedBook:
    observation: object
    geometry: object
    frame_id: str
    suction_point: tuple[float, float, float]


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


def _xy_within_tolerance(target):
    return (
        abs(target.residual_m[0]) <= BOOK_ALIGNMENT_X_TOLERANCE_M
        and abs(target.residual_m[1]) <= BOOK_ALIGNMENT_Y_TOLERANCE_M
    )


def _stable_book_measurement(vision, selector, *, frame="base_link", say=print):
    """Select the same logical book in several frames and median its point."""

    find_samples = getattr(vision, "find_samples", None)
    if callable(find_samples):
        frames = find_samples(
            "book",
            frame=frame,
            successful_samples=BOOK_VISION_SUCCESSFUL_SAMPLES,
            maximum_attempts=BOOK_VISION_MAXIMUM_ATTEMPTS,
        )
        if len(frames) < BOOK_VISION_SUCCESSFUL_SAMPLES:
            raise RuntimeError(
                "多帧视觉没有获得足够的书本结果: "
                f"{len(frames)}/{BOOK_VISION_SUCCESSFUL_SAMPLES}"
            )
    else:
        # Small test doubles and old callers keep the original one-frame API.
        frames = (vision.find("book", frame=frame),)

    selected = [selector(books) for books in frames if books]
    if not selected:
        raise RuntimeError("没有检测到可对位的书本")
    if len(selected) == 1:
        return selected[0]
    point = tuple(
        float(median(book.suction_point[axis] for book in selected))
        for axis in range(3)
    )
    representative = min(
        selected,
        key=lambda book: sum(
            (float(book.suction_point[axis]) - point[axis]) ** 2
            for axis in range(3)
        ),
    )
    say(
        "多帧书本位置: "
        f"samples={len(selected)}, x={point[0]:.3f}, "
        f"y={point[1]:.3f}, z={point[2]:.3f} m"
    )
    return StableLocatedBook(
        observation=getattr(representative, "observation", None),
        geometry=representative.geometry,
        frame_id=getattr(representative, "frame_id", frame),
        suction_point=point,
    )


def _run_precise_alignment_loop(
    vision,
    navigator,
    replay_reference,
    first_book,
    *,
    say=print,
):
    book = first_book
    first_target = None
    last_navigation = None
    for correction_index in range(BOOK_ALIGNMENT_MAXIMUM_CORRECTIONS + 1):
        target = build_replay_alignment_target(
            book=book,
            replay_reference=replay_reference,
        )
        if first_target is None:
            first_target = target
        say(
            _format_residual(
                f"DataReplay 细校准 {correction_index}/{BOOK_ALIGNMENT_MAXIMUM_CORRECTIONS}",
                target,
            )
        )
        if _xy_within_tolerance(target):
            say("XY验收=达标")
            return first_target, target, last_navigation, True
        if correction_index == BOOK_ALIGNMENT_MAXIMUM_CORRECTIONS:
            say("XY验收=未达标")
            return first_target, target, last_navigation, False

        last_navigation = navigator.align(
            reference=target.reference_m,
            observed=target.observed_m,
        )
        say(
            _format_navigation(
                f"第 {correction_index + 1} 次细校准运动反馈",
                last_navigation,
            )
        )
        predicted = predict_book_after_base_motion(
            target.observed_m,
            odom_dx_m=last_navigation.odom_dx_m,
            odom_dy_m=last_navigation.odom_dy_m,
            imu_dyaw_rad=last_navigation.imu_dyaw_rad,
        )
        book = _stable_book_measurement(
            vision,
            lambda books, predicted=predicted: reassociate_book(
                books,
                predicted_point_m=predicted,
                replay_reference=replay_reference,
            ),
            say=say,
        )


def run_book_alignment_once(vision, navigator, replay_reference, say=print):
    """Run coarse visual approach followed by replay-image precise docking."""

    coarse_book = _stable_book_measurement(
        vision,
        lambda books: select_coarse_book(books).book,
        say=say,
    )
    coarse = select_coarse_book((coarse_book,))
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
    after_coarse_book = _stable_book_measurement(
        vision,
        lambda books: reassociate_book(
            books,
            predicted_point_m=predicted_after_coarse,
            replay_reference=replay_reference,
        ),
        say=say,
    )
    precise, final, precise_navigation, xy_within_tolerance = (
        _run_precise_alignment_loop(
            vision,
            navigator,
            replay_reference,
            after_coarse_book,
            say=say,
        )
    )
    return BookAlignmentRun(
        coarse=coarse,
        precise=precise,
        final=final,
        coarse_navigation=coarse_navigation,
        precise_navigation=precise_navigation,
        z_offset_m=final.z_offset_m,
        xy_within_tolerance=xy_within_tolerance,
    )


def run_book_alignment_from_current_once(
    vision,
    navigator,
    replay_reference,
    say=print,
):
    """Skip coarse approach and precisely dock from the current base pose."""

    current_book = _stable_book_measurement(
        vision,
        lambda books: select_replay_book(
            books,
            replay_reference=replay_reference,
        ),
        say=say,
    )
    precise, final, precise_navigation, xy_within_tolerance = (
        _run_precise_alignment_loop(
            vision,
            navigator,
            replay_reference,
            current_book,
            say=say,
        )
    )
    return BookAlignmentRun(
        coarse=None,
        precise=precise,
        final=final,
        coarse_navigation=None,
        precise_navigation=precise_navigation,
        z_offset_m=final.z_offset_m,
        xy_within_tolerance=xy_within_tolerance,
    )


def run_book_pick_once(
    vision,
    navigator,
    replayer,
    replay_reference,
    say=print,
    coarse=False,
    press_m=0.0,
):
    """Align one book, apply the fixed Z handoff, and replay one Pick."""

    say("新一轮抓取：先自动恢复 DataReplay 第0帧观察姿态")
    replayer.prepare()
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
    press_m = float(press_m)
    say(
        "开始执行 Stage-1 Pick："
        f"DataReplay 原始高度，额外下压={press_m * 1000.0:.1f} mm"
    )
    replay = replayer.pick(-press_m)
    say(
        f"Pick 回放完成: frames={replay.frames_sent}, "
        f"torso target={replay.torso_target_m:.3f} m, "
        f"actual={replay.torso_actual_m:.3f} m, "
        f"D01 holding={replay.d01_holding if replay.d01_holding is not None else '未检查'}"
    )
    return BookPickRun(alignment=alignment, replay=replay)


def run_book_place_once(cart_navigator, replayer, say=print):
    """Navigate from the reviewed table point and replay one cart Place."""

    say("保持右吸盘开启，扫描小推车并按直角路线前往对应槽位")
    navigation = cart_navigator.navigate()
    say(_format_navigation("小推车地图导航反馈", navigation))
    say("恢复 DataReplay 2.4 第0帧姿态并开始原速 Place")
    replay = replayer.place()
    say(
        f"Place 回放完成: frames={replay.frames_sent}, "
        f"torso target={replay.torso_target_m:.3f} m, "
        f"actual={replay.torso_actual_m:.3f} m, "
        f"D01 released={replay.d01_released}"
    )
    return BookPlaceRun(navigation=navigation, replay=replay)


def run_book_pick_place_once(
    vision,
    pick_navigator,
    pick_replayer,
    replay_reference,
    cart_navigator,
    place_replayer,
    say=print,
    coarse=False,
    press_m=0.0,
):
    """Run one complete table Pick followed immediately by one cart Place."""

    pick = run_book_pick_once(
        vision,
        pick_navigator,
        pick_replayer,
        replay_reference,
        say=say,
        coarse=coarse,
        press_m=press_m,
    )
    place = run_book_place_once(cart_navigator, place_replayer, say=say)
    return BookPickPlaceRun(pick=pick, place=place)


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
