"""把感知、移动和动作积木按任务顺序接起来。"""

from dataclasses import dataclass
import math
from statistics import median

from experiment_timing import timed_phase
from cart_place_alignment import (
    build_cart_place_alignment_target,
    median_cart_place_alignment_target,
)

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
BOOK_VISION_SUCCESSFUL_SAMPLES = 1
BOOK_VISION_MAXIMUM_ATTEMPTS = 1
BOOK_ALIGNMENT_MAXIMUM_CORRECTIONS = 3
CART_PLACE_X_TOLERANCE_M = 0.030
CART_PLACE_Y_TOLERANCE_M = 0.020
CART_PLACE_YAW_TOLERANCE_RAD = 5.0 * 3.141592653589793 / 180.0
CART_PLACE_MAXIMUM_CORRECTIONS = 3
CART_VISION_SUCCESSFUL_SAMPLES = 1
CART_VISION_MAXIMUM_ATTEMPTS = 1
CART_PLACE_RETRY_FORWARD_M = 0.10
CART_PLACE_POSITION_ATTEMPTS = 3


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
class CartPlaceAlignmentRun:
    first: object
    final: object
    last_navigation: object
    within_tolerance: bool


@dataclass(frozen=True)
class BookPlaceRun:
    alignment: CartPlaceAlignmentRun
    replay: object


@dataclass(frozen=True)
class BookPickPlaceRun:
    pick: BookPickRun
    coarse_cart_navigation: object
    place: BookPlaceRun


@dataclass(frozen=True)
class StableLocatedBook:
    observation: object
    geometry: object
    frame_id: str
    suction_point: tuple[float, float, float]


def _format_residual(label, target):
    dx, dy, dz = target.residual_m
    return (
        f"{label}: "
        f"目标=({target.reference_m[0]:.3f}, {target.reference_m[1]:.3f}, "
        f"{target.reference_m[2]:.3f}) m, "
        f"检测=({target.observed_m[0]:.3f}, {target.observed_m[1]:.3f}, "
        f"{target.observed_m[2]:.3f}) m, "
        f"误差 dx={dx:.3f} m, dy={dy:.3f} m, dz={dz:.3f} m"
    )


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
    """Use the first successful frame; missed frames may be retried."""

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
                "本轮没有获得可用的书本图像: "
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

        with timed_phase(
            "pick_alignment_motion",
            correction_index=correction_index + 1,
        ) as timing:
            last_navigation = navigator.align(
                reference=target.reference_m,
                observed=target.observed_m,
            )
            timing.update(
                command_count=last_navigation.command_count,
                odom_dx_m=last_navigation.odom_dx_m,
                odom_dy_m=last_navigation.odom_dy_m,
                imu_dyaw_rad=last_navigation.imu_dyaw_rad,
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
        with timed_phase(
            "pick_book_vision",
            observation="after_correction",
            correction_index=correction_index + 1,
        ):
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

    with timed_phase("pick_book_vision", observation="coarse"):
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
    with timed_phase("pick_alignment_motion", observation="coarse") as timing:
        coarse_navigation = navigator.align(
            reference=coarse.reference_m,
            observed=coarse.observed_m,
        )
        timing.update(
            command_count=coarse_navigation.command_count,
            odom_dx_m=coarse_navigation.odom_dx_m,
            odom_dy_m=coarse_navigation.odom_dy_m,
            imu_dyaw_rad=coarse_navigation.imu_dyaw_rad,
        )
    say(_format_navigation("粗定位运动反馈", coarse_navigation))

    initial_contact = coarse.book.suction_point
    predicted_after_coarse = predict_book_after_base_motion(
        initial_contact,
        odom_dx_m=coarse_navigation.odom_dx_m,
        odom_dy_m=coarse_navigation.odom_dy_m,
        imu_dyaw_rad=coarse_navigation.imu_dyaw_rad,
    )
    with timed_phase("pick_book_vision", observation="after_coarse"):
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

    with timed_phase("pick_book_vision", observation="initial"):
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
    with timed_phase("pick_frame_zero_prepare"):
        replayer.prepare()
    align = (
        run_book_alignment_once
        if coarse
        else run_book_alignment_from_current_once
    )
    with timed_phase("pick_alignment") as timing:
        alignment = align(vision, navigator, replay_reference, say=say)
        timing.update(
            xy_within_tolerance=alignment.xy_within_tolerance,
            final_dx_m=alignment.final.residual_m[0],
            final_dy_m=alignment.final.residual_m[1],
            final_dz_m=alignment.final.residual_m[2],
        )
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
    with timed_phase("pick_replay", speed=1.0) as timing:
        replay = replayer.pick(-press_m)
        timing.update(
            frames_sent=replay.frames_sent,
            torso_target_m=replay.torso_target_m,
            torso_actual_m=replay.torso_actual_m,
            d01_holding=replay.d01_holding,
        )
    say(
        f"Pick 回放完成: frames={replay.frames_sent}, "
        f"torso target={replay.torso_target_m:.3f} m, "
        f"actual={replay.torso_actual_m:.3f} m, "
        f"D01 holding={replay.d01_holding if replay.d01_holding is not None else '未检查'}"
    )
    return BookPickRun(alignment=alignment, replay=replay)


def _stable_cart_place_target(
    vision,
    replay_reference,
    slot_index,
    preferred_body_target_m=None,
):
    find_samples = getattr(vision, "find_cart_samples", None)
    if callable(find_samples):
        carts = find_samples(
            successful_samples=CART_VISION_SUCCESSFUL_SAMPLES,
            maximum_attempts=CART_VISION_MAXIMUM_ATTEMPTS,
            preferred_body_target_m=preferred_body_target_m,
        )
        if len(carts) < CART_VISION_SUCCESSFUL_SAMPLES:
            raise RuntimeError(
                "本轮没有获得可用的小推车托板图像: "
                f"{len(carts)}/{CART_VISION_SUCCESSFUL_SAMPLES}"
            )
    else:
        carts = (
            vision.find_cart(preferred_body_target_m=preferred_body_target_m),
        )
    targets = []
    for cart in carts:
        if cart is None or cart.platform is None:
            continue
        cart_bodies = [
            observation
            for observation in cart.observations
            if observation.semantic_class == "cart_body"
        ]
        if not cart_bodies:
            continue
        cart_body = max(cart_bodies, key=lambda observation: observation.confidence)
        targets.append(build_cart_place_alignment_target(
            platform=cart.platform,
            replay_reference=replay_reference,
            slot_index=slot_index,
            observed_cart_bbox_width_ratio=(
                float(cart_body.bbox[2]) / float(cart_body.image_width)
            ),
        ))
    if not targets:
        raise RuntimeError("没有检测到可用于 Place 对位的小推车托板")
    return median_cart_place_alignment_target(targets)


def _cart_place_within_tolerance(target):
    return (
        abs(target.residual_m[0]) <= CART_PLACE_X_TOLERANCE_M
        and abs(target.residual_m[1]) <= CART_PLACE_Y_TOLERANCE_M
        and abs(target.yaw_error_rad) <= CART_PLACE_YAW_TOLERANCE_RAD
    )


def _cart_place_target_with_position_retries(
    vision,
    navigator,
    replay_reference,
    slot_index,
    preferred_body_target_m=None,
    say=print,
):
    moved_forward_m = 0.0
    for attempt in range(CART_PLACE_POSITION_ATTEMPTS):
        try:
            return _stable_cart_place_target(
                vision,
                replay_reference,
                slot_index,
                preferred_body_target_m=preferred_body_target_m,
            )
        except RuntimeError:
            if attempt + 1 == CART_PLACE_POSITION_ATTEMPTS:
                if moved_forward_m:
                    say(
                        "小推车平台连续三次检测失败，"
                        f"自动后退 {moved_forward_m:.2f} m 后报错"
                    )
                    navigator.move_forward(-moved_forward_m)
                raise
            say(
                "小推车平台检测失败，"
                f"前进 {CART_PLACE_RETRY_FORWARD_M:.2f} m 后重新检测"
            )
            navigator.move_forward(CART_PLACE_RETRY_FORWARD_M)
            moved_forward_m += CART_PLACE_RETRY_FORWARD_M


def run_cart_place_alignment_once(
    vision,
    navigator,
    replay_reference,
    *,
    slot_index,
    preferred_body_target_m=None,
    say=print,
):
    """Repeatedly match live shelf geometry to Place-2.4 frame zero."""

    with timed_phase("place_observation_pose") as timing:
        torso_actual_m = navigator.set_observation_torso(
            replay_reference.recorded_torso_m
        )
        vision.set_head_pose(
            yaw_rad=replay_reference.recorded_head_rad[0],
            pitch_rad=replay_reference.recorded_head_rad[1],
        )
        timing.update(
            torso_actual_m=torso_actual_m,
            head_yaw_rad=replay_reference.recorded_head_rad[0],
            head_pitch_rad=replay_reference.recorded_head_rad[1],
        )
    say(
        "Place 感知姿态已恢复: "
        f"torso={torso_actual_m:.3f} m, "
        f"head_pitch={replay_reference.recorded_head_rad[1]:.3f} rad"
    )

    first = None
    last_navigation = None
    for correction_index in range(CART_PLACE_MAXIMUM_CORRECTIONS + 1):
        with timed_phase(
            "place_cart_vision",
            correction_index=correction_index,
            slot_index=slot_index,
        ):
            target = _cart_place_target_with_position_retries(
                vision,
                navigator,
                replay_reference,
                slot_index,
                preferred_body_target_m=preferred_body_target_m,
                say=say,
            )
        if first is None:
            first = target
        say(
            "Place 2.4 细校准 "
            f"{correction_index}/{CART_PLACE_MAXIMUM_CORRECTIONS}: "
            f"目标=({target.reference_anchor_m[0]:.3f}, "
            f"{target.reference_anchor_m[1]:.3f}, "
            f"{target.reference_anchor_m[2]:.3f}) m, "
            f"检测=({target.observed_anchor_m[0]:.3f}, "
            f"{target.observed_anchor_m[1]:.3f}, "
            f"{target.observed_anchor_m[2]:.3f}) m, "
            f"前后={target.residual_m[0]:.3f} m, "
            f"左右={target.residual_m[1]:.3f} m, "
            f"yaw={target.yaw_error_rad * 180.0 / 3.141592653589793:.2f}°"
        )
        if _cart_place_within_tolerance(target):
            return CartPlaceAlignmentRun(first, target, last_navigation, True)
        if correction_index == CART_PLACE_MAXIMUM_CORRECTIONS:
            return CartPlaceAlignmentRun(first, target, last_navigation, False)
        with timed_phase(
            "place_alignment_motion",
            correction_index=correction_index + 1,
            slot_index=slot_index,
        ) as timing:
            last_navigation = navigator.align(target)
            timing.update(
                command_count=last_navigation.command_count,
                odom_dx_m=last_navigation.odom_dx_m,
                odom_dy_m=last_navigation.odom_dy_m,
                imu_dyaw_rad=last_navigation.imu_dyaw_rad,
            )
        if preferred_body_target_m is not None:
            delta_x = (
                preferred_body_target_m[0] - last_navigation.odom_dx_m
            )
            delta_y = (
                preferred_body_target_m[1] - last_navigation.odom_dy_m
            )
            cosine = math.cos(-last_navigation.imu_dyaw_rad)
            sine = math.sin(-last_navigation.imu_dyaw_rad)
            preferred_body_target_m = (
                cosine * delta_x - sine * delta_y,
                sine * delta_x + cosine * delta_y,
                preferred_body_target_m[2],
            )
        say(_format_navigation("小推车细校准运动反馈", last_navigation))
    raise AssertionError("unreachable")


def run_book_place_once(
    vision,
    navigator,
    replayer,
    replay_reference,
    *,
    slot_index,
    preferred_body_target_m=None,
    say=print,
):
    with timed_phase("place_alignment", slot_index=slot_index) as timing:
        alignment = run_cart_place_alignment_once(
            vision,
            navigator,
            replay_reference,
            slot_index=slot_index,
            preferred_body_target_m=preferred_body_target_m,
            say=say,
        )
        timing.update(
            within_tolerance=alignment.within_tolerance,
            final_dx_m=alignment.final.residual_m[0],
            final_dy_m=alignment.final.residual_m[1],
            final_yaw_rad=alignment.final.yaw_error_rad,
        )
    if not alignment.within_tolerance:
        target = alignment.final
        raise RuntimeError(
            "Place 最终对位未达标，不启动 DataReplay: "
            f"前后={target.residual_m[0]:.3f} m, "
            f"左右={target.residual_m[1]:.3f} m, "
            f"yaw={target.yaw_error_rad * 180.0 / 3.141592653589793:.2f}°"
        )
    say("恢复 Place 2.4 第0帧全身姿态并以1.0倍速回放")
    with timed_phase("place_replay", speed=1.0, slot_index=slot_index) as timing:
        replay = replayer.place()
        timing.update(
            frames_sent=replay.frames_sent,
            torso_target_m=replay.torso_target_m,
            torso_actual_m=replay.torso_actual_m,
            d01_released=replay.d01_released,
        )
    say(
        f"Place 回放完成: frames={replay.frames_sent}, "
        f"torso target={replay.torso_target_m:.3f} m, "
        f"actual={replay.torso_actual_m:.3f} m, "
        f"D01 released={replay.d01_released}"
    )
    return BookPlaceRun(alignment=alignment, replay=replay)


def run_book_pick_place_once(
    vision,
    book_navigator,
    pick_replayer,
    pick_reference,
    cart_navigator,
    place_navigator,
    place_replayer,
    place_reference,
    *,
    slot_index,
    coarse=False,
    press_m=0.0,
    say=print,
):
    pick = run_book_pick_once(
        vision,
        book_navigator,
        pick_replayer,
        pick_reference,
        say=say,
        coarse=coarse,
        press_m=press_m,
    )
    say("Pick 完成：后退20cm并沿直角路线粗导航到小推车")
    with timed_phase("cart_coarse_navigation", slot_index=slot_index) as timing:
        coarse_cart_navigation = cart_navigator.navigate()
        timing.update(
            command_count=coarse_cart_navigation.command_count,
            odom_dx_m=coarse_cart_navigation.odom_dx_m,
            odom_dy_m=coarse_cart_navigation.odom_dy_m,
            imu_dyaw_rad=coarse_cart_navigation.imu_dyaw_rad,
            selected_scan_yaw_rad=(
                coarse_cart_navigation.selected_scan_yaw_rad
            ),
        )
    target_x = (
        coarse_cart_navigation.cart_target_origin_m[0]
        - coarse_cart_navigation.odom_dx_m
    )
    target_y = (
        coarse_cart_navigation.cart_target_origin_m[1]
        - coarse_cart_navigation.odom_dy_m
    )
    cosine = math.cos(-coarse_cart_navigation.imu_dyaw_rad)
    sine = math.sin(-coarse_cart_navigation.imu_dyaw_rad)
    preferred_body_target_m = (
        cosine * target_x - sine * target_y,
        sine * target_x + cosine * target_y,
        coarse_cart_navigation.cart_target_origin_m[2],
    )
    place = run_book_place_once(
        vision,
        place_navigator,
        place_replayer,
        place_reference,
        slot_index=slot_index,
        preferred_body_target_m=preferred_body_target_m,
        say=say,
    )
    return BookPickPlaceRun(pick, coarse_cart_navigation, place)


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
