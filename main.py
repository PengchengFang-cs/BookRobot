#!/usr/bin/env python3
"""FruitTest 入口。主流程故意保持成小朋友也能读懂的样子。"""

import argparse
import json
import sys

from geometry import fruit_from_text
from mission import (
    run_book_alignment_from_current_once,
    run_book_alignment_once,
    run_book_pick_place_once,
    run_book_pick_once,
    run_book_place_once,
    run_one_fruit,
)


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fruit", help="跳过语音，直接测试一种水果")
    parser.add_argument("--once", action="store_true", help="只做一次后退出")
    parser.add_argument("--fake", action="store_true", help="不连接机器人")
    operation = parser.add_mutually_exclusive_group()
    operation.add_argument("--check", action="store_true", help="只检查接口，不运动")
    operation.add_argument(
        "--command-check",
        action="store_true",
        help="发送保持当前位置的导航和右臂命令",
    )
    operation.add_argument(
        "--motion-check",
        action="store_true",
        help="执行小幅底盘、右臂和夹爪往返",
    )
    operation.add_argument(
        "--base-motion-check",
        action="store_true",
        help="只让底盘往返 40 cm",
    )
    operation.add_argument(
        "--arm-motion-check",
        action="store_true",
        help="只让右臂小幅往返并开合夹爪",
    )
    operation.add_argument(
        "--pick-motion-check",
        action="store_true",
        help="在空中执行一次完整抓取动作",
    )
    operation.add_argument(
        "--book-align",
        action="store_true",
        help="检测书本并对位到 DataReplay 固定抓取点，不抓取",
    )
    operation.add_argument(
        "--book-pick",
        action="store_true",
        help="检测、对位并执行一次 Stage-1 DataReplay 吸书",
    )
    operation.add_argument(
        "--book-place",
        action="store_true",
        help="从当前推车粗定位位置多轮对位并执行一次Place 2.4",
    )
    operation.add_argument(
        "--book-pick-place",
        action="store_true",
        help="连续执行抓书、推车粗导航、精定位和Place 2.4",
    )
    operation.add_argument(
        "--stage1-loop",
        action="store_true",
        help="连续完成五本书的抓取、放置与返回书桌粗导航",
    )
    operation.add_argument(
        "--cart-perception",
        action="store_true",
        help="只检测小推车顶面并输出五个槽位，不产生运动",
    )
    operation.add_argument(
        "--cart-scan-navigation",
        action="store_true",
        help="不拿书、不平移：原地每15度扫描一次，左转到90度后停止",
    )
    operation.add_argument(
        "--cart-approach-navigation",
        action="store_true",
        help="不拿书、不放书：扫描推车并执行当前直角粗导航",
    )
    parser.add_argument(
        "--book-align-mode",
        choices=("legacy", "vector"),
        default="legacy",
        help="书本底盘对位方式；默认保留原 legacy，vector 合并 X/Y 位移",
    )
    parser.add_argument(
        "--book-coarse",
        action="store_true",
        help="显式启用抓书前的远距离粗定位；默认直接恢复回放第0帧位置",
    )
    parser.add_argument(
        "--book-pick-press-mm",
        type=float,
        default=0.0,
        help="薄书 Pick 相对录像额外下压毫米数；默认0",
    )
    parser.add_argument(
        "--book-index",
        type=int,
        choices=range(1, 6),
        default=1,
        help="当前书本对应的小推车槽位，1为机器人视角最右侧",
    )
    return parser.parse_args()


def run_fake(args):
    from fake import FakeArm, FakeNavigation, FakeVision

    fruit = fruit_from_text(args.fruit or "香蕉") or args.fruit or "banana"
    run_one_fruit(fruit, FakeVision(), FakeNavigation(), FakeArm())


def run_real(args):
    import rclpy
    from rclpy.node import Node

    from vision import Vision

    rclpy.init()
    node = Node("fruit_test")
    arm = None

    try:
        if (
            args.book_align
            or args.book_pick
            or args.book_place
            or args.book_pick_place
            or args.stage1_loop
            or args.cart_perception
            or args.cart_scan_navigation
            or args.cart_approach_navigation
        ):
            from book_navigation import (
                BookAlignmentNavigator,
                CartPlaceDockingNavigator,
                Stage1CartNavigator,
                Stage1TableReturnNavigator,
            )
            from config import (
                REPLAY_PICK_REFERENCE_PATH,
                REPLAY_PLACE_REFERENCE_PATH,
            )
            from replay_pick_reference import load_replay_pick_reference
            from replay_place_reference import load_replay_place_reference
            from tf2_ros import Buffer, TransformListener

            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, node)
            vision = Vision(node, tf_buffer)

            if args.cart_perception:
                cart = vision.find_cart()
                platform = None if cart is None else cart.platform
                payload = {
                    "ok": cart is not None,
                    "classes": [] if cart is None else [
                        row.semantic_class for row in cart.observations
                    ],
                    "frame_id": None if cart is None else cart.frame_id,
                    "cart_target_m": (
                        None if cart is None or cart.body_target is None
                        else cart.body_target.center
                    ),
                    "platform_center_m": (
                        None if platform is None else platform.center
                    ),
                    "platform_lateral_extent_m": (
                        None if platform is None else platform.lateral_extent_m
                    ),
                    "platform_depth_extent_m": (
                        None if platform is None else platform.depth_extent_m
                    ),
                    "platform_front_edge_m": (
                        None if platform is None else platform.front_edge
                    ),
                    "platform_left_edge_m": (
                        None if platform is None else platform.left_edge
                    ),
                    "platform_right_edge_m": (
                        None if platform is None else platform.right_edge
                    ),
                    "slot_centers_m": (
                        [] if platform is None else platform.slot_centers
                    ),
                    "debug_image": str(vision.debug_path),
                }
                print(json.dumps(payload, ensure_ascii=False))
                return

            if args.cart_scan_navigation or args.cart_approach_navigation:
                navigation = Stage1CartNavigator(
                    vision=vision,
                    book_index=args.book_index,
                    scan_only=args.cart_scan_navigation,
                ).navigate()
                print(json.dumps({
                    "ok": True,
                    "mode": navigation.mode,
                    "commands": navigation.command_count,
                    "selected_scan_yaw_deg": (
                        navigation.selected_scan_yaw_rad * 180.0 / 3.141592653589793
                    ),
                    "slot_index": navigation.slot_index,
                    "cart_target_origin_m": navigation.cart_target_origin_m,
                    "slot_center_origin_m": navigation.slot_center_origin_m,
                    "platform_near_x_origin_m": navigation.platform_near_x_origin_m,
                    "debug_image": navigation.selected_scan_debug_image,
                }, ensure_ascii=False))
                return

            say = lambda text: print(f"[机器人] {text}")
            feedback = dict(
                joint_positions=lambda: dict(vision.joints),
                spin_feedback=lambda: vision.spin_until_fresh_body(
                    lambda timeout_s: rclpy.spin_once(
                        node, timeout_sec=timeout_s
                    ),
                    timeout_s=0.5,
                ),
            )
            if args.stage1_loop:
                from book_pick_replay import Stage1BookPickReplayer
                from book_place_replay import Stage1BookPlaceReplayer

                pick_reference = load_replay_pick_reference(
                    REPLAY_PICK_REFERENCE_PATH
                )
                place_reference = load_replay_place_reference(
                    REPLAY_PLACE_REFERENCE_PATH
                )
                for book_index in range(1, 4):
                    say(f"Stage 1 第 {book_index}/3 本")
                    run_book_pick_place_once(
                        vision,
                        BookAlignmentNavigator(mode=args.book_align_mode),
                        Stage1BookPickReplayer(**feedback),
                        pick_reference,
                        Stage1CartNavigator(
                            vision=vision,
                            book_index=book_index,
                        ),
                        CartPlaceDockingNavigator(),
                        Stage1BookPlaceReplayer(**feedback),
                        place_reference,
                        slot_index=book_index,
                        coarse=(args.book_coarse and book_index == 1),
                        press_m=args.book_pick_press_mm / 1000.0,
                        say=say,
                    )
                    if book_index < 3:
                        say("扫描书本并沿直角路线返回下一轮抓书位置")
                        Stage1TableReturnNavigator(vision=vision).navigate()
            elif args.book_place:
                from book_place_replay import Stage1BookPlaceReplayer

                run_book_place_once(
                    vision,
                    CartPlaceDockingNavigator(),
                    Stage1BookPlaceReplayer(**feedback),
                    load_replay_place_reference(REPLAY_PLACE_REFERENCE_PATH),
                    slot_index=args.book_index,
                    say=say,
                )
            elif args.book_pick_place:
                from book_pick_replay import Stage1BookPickReplayer
                from book_place_replay import Stage1BookPlaceReplayer

                run_book_pick_place_once(
                    vision,
                    BookAlignmentNavigator(mode=args.book_align_mode),
                    Stage1BookPickReplayer(**feedback),
                    load_replay_pick_reference(REPLAY_PICK_REFERENCE_PATH),
                    Stage1CartNavigator(
                        vision=vision,
                        book_index=args.book_index,
                    ),
                    CartPlaceDockingNavigator(),
                    Stage1BookPlaceReplayer(**feedback),
                    load_replay_place_reference(REPLAY_PLACE_REFERENCE_PATH),
                    slot_index=args.book_index,
                    coarse=args.book_coarse,
                    press_m=args.book_pick_press_mm / 1000.0,
                    say=say,
                )
            elif args.book_pick:
                from book_pick_replay import Stage1BookPickReplayer

                navigator = BookAlignmentNavigator(mode=args.book_align_mode)
                replay_reference = load_replay_pick_reference(
                    REPLAY_PICK_REFERENCE_PATH
                )
                pick_replayer = Stage1BookPickReplayer(**feedback)
                run_book_pick_once(
                    vision,
                    navigator,
                    pick_replayer,
                    replay_reference,
                    say,
                    coarse=args.book_coarse,
                    press_m=args.book_pick_press_mm / 1000.0,
                )
            else:
                navigator = BookAlignmentNavigator(mode=args.book_align_mode)
                replay_reference = load_replay_pick_reference(
                    REPLAY_PICK_REFERENCE_PATH
                )
                align = (
                    run_book_alignment_once
                    if args.book_coarse
                    else run_book_alignment_from_current_once
                )
                align(
                    vision,
                    navigator,
                    replay_reference,
                    say,
                )
            print(f"[视觉] 调试图: {vision.debug_path}")
            return

        from navigation import Navigation
        from voice import Voice

        navigation = Navigation(node)
        vision = Vision(node, navigation.tf_buffer)
        voice = Voice(node)

        from arm import Arm

        arm = Arm(node, vision.right_arm_positions)
        navigation.remember_home()
        arm.remember_stow()

        if args.check:
            end = node.get_clock().now().nanoseconds + 5_000_000_000
            while node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.10)
                if vision.color and vision.depth and vision.info:
                    break
            if not (vision.color and vision.depth and vision.info):
                raise RuntimeError("五秒内没有收到 RGB-D")
            if not arm.plan_client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError("MoveIt 规划服务没有启动")
            if not navigation.drive_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError("Nav2 直行没有启动")
            if not navigation.spin_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError("Nav2 原地旋转没有启动")
            arm.check_plan()
            print("[检查通过] RGB-D、Nav2、关节状态、MoveIt 位姿规划都正常；没有产生运动。")
            return

        if args.command_check:
            navigation.check_command()
            arm.check_controller()
            print("[检查通过] Nav2 和右臂都接受了真实命令，并保持在当前位置。")
            return

        if args.base_motion_check:
            navigation.check_motion()
            print("[检查通过] 底盘完成了 40 cm 真实往返。")
            return

        if args.arm_motion_check:
            arm.check_motion()
            print("[检查通过] MoveIt 轨迹控制和夹爪完成了小幅真实运动。")
            return

        if args.pick_motion_check:
            arm.pick((0.45, -0.05, 0.82))
            arm.drop()
            print("[检查通过] 完整预抓取、下探、夹取、抬起和回收动作成功。")
            return

        if args.motion_check:
            navigation.check_motion()
            arm.check_motion()
            print("[检查通过] 底盘、MoveIt 轨迹控制和夹爪都完成了小幅真实运动。")
            return

        manual = None
        if args.fruit:
            manual = fruit_from_text(args.fruit) or args.fruit.lower()

        if manual is None:
            voice.prepare()
        voice.say("请打开遥控器并按一下机身释放键")
        navigation.wait_for_release()
        voice.say("水果冒烟测试已经准备好，请说一种水果")
        while rclpy.ok():
            fruit = manual or voice.wait_for_fruit()
            manual = None
            try:
                run_one_fruit(fruit, vision, navigation, arm, voice.say)
            except Exception as error:
                voice.say(f"这次没有完成，原因是 {error}")
                print(f"[错误] {error}", file=sys.stderr)
                try:
                    navigation.go_home()
                except Exception as home_error:
                    print(f"[错误] 返回原点也失败: {home_error}", file=sys.stderr)
                if args.once:
                    raise
            if args.once:
                break
    finally:
        if arm is not None:
            try:
                arm.restore_controller()
            except Exception as error:
                print(f"[提示] 双臂控制器没有自动恢复: {error}", file=sys.stderr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    args = arguments()
    try:
        if args.fake:
            run_fake(args)
        else:
            run_real(args)
    except KeyboardInterrupt:
        print("\n[退出] FruitTest 已停止")


if __name__ == "__main__":
    main()
