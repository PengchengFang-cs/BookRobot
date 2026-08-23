#!/usr/bin/env python3
"""Stage 2 then Stage 3 entry point, following the Stage 1 main.py layout."""

import argparse
import sys

from mission2 import (
    build_stage23_replayers,
    close_stage23_replayers,
    preload_stage23_replayers,
    require_stage23_configuration,
    run_stage2_stage3,
)


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--skip-stage2-initial-coarse",
        action="store_true",
        help="Stage 2从已经正对推车的DR11.2精定位区域开始",
    )
    parser.add_argument(
        "--book-align-mode",
        choices=("legacy", "vector"),
        default="legacy",
        help="书本底盘对位方式；与Stage 1保持一致",
    )
    return parser.parse_args()


def run_real(args):
    # All missing references and interfaces are rejected before rclpy.init().
    require_stage23_configuration()

    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    from vision import Vision

    rclpy.init()
    node = None
    pick_replayers = {}
    place_replayers = {}
    try:
        node = Node("library_stage2_stage3")
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        vision = Vision(node, tf_buffer)
        feedback = dict(
            joint_positions=lambda: dict(vision.joints),
            spin_feedback=lambda: vision.spin_until_fresh_body(
                lambda timeout_s: rclpy.spin_once(node, timeout_sec=timeout_s),
                timeout_s=0.5,
            ),
        )
        pick_replayers, place_replayers = build_stage23_replayers(**feedback)
        preload_stage23_replayers(
            pick_replayers,
            place_replayers,
            say=lambda text: print(f"[机器人] {text}"),
        )
        run_stage2_stage3(
            vision,
            book_align_mode=args.book_align_mode,
            pick_replayers=pick_replayers,
            place_replayers=place_replayers,
            skip_stage2_initial_coarse=args.skip_stage2_initial_coarse,
            say=lambda text: print(f"[机器人] {text}"),
        )
    finally:
        active_error = sys.exc_info()[0] is not None
        cleanup_errors = []
        try:
            close_stage23_replayers(pick_replayers, place_replayers)
        except Exception as error:
            cleanup_errors.append(error)
        if node is not None:
            try:
                node.destroy_node()
            except Exception as error:
                cleanup_errors.append(error)
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as error:
                cleanup_errors.append(error)
        if cleanup_errors:
            if active_error:
                for error in cleanup_errors:
                    print(f"[清理错误] {error}", file=sys.stderr)
            else:
                raise cleanup_errors[0]


def main():
    args = arguments()
    try:
        run_real(args)
    except KeyboardInterrupt:
        print("\n[退出] Stage 2/3任务已停止")
        raise SystemExit(130)
    except Exception as error:
        print(f"[错误] {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
