#!/usr/bin/env python3
"""Capture one book suction point without constructing motion controllers."""

import json


def format_result(point):
    payload = {
        "ok": point is not None,
        "frame_id": "base_link",
        "suction_point_m": None if point is None else [float(value) for value in point],
    }
    return json.dumps(payload, ensure_ascii=False, allow_nan=False, sort_keys=True)


def main():
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    from vision import Vision

    rclpy.init()
    node = Node("book_perception_test")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    vision = Vision(node, tf_buffer)
    try:
        point = vision.find("book", frame="base_link")
        print(format_result(point))
        return 0 if point is not None else 2
    finally:
        # Retain the listener until all subscriptions are stopped.
        del tf_listener
        vision.book_client.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
