#!/usr/bin/env python3
"""Capture up to five book suction points without motion controllers."""

import json


def format_result(books):
    payload = {
        "ok": bool(books),
        "frame_id": "base_link",
        "book_count": len(books),
        "books": [
            {
                "confidence": float(book.observation.confidence),
                "bbox_xywh": [int(value) for value in book.observation.bbox],
                "suction_point_m": [float(value) for value in book.suction_point],
            }
            for book in books
        ],
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
        books = vision.find("book", frame="base_link")
        print(format_result(books))
        return 0 if books else 2
    finally:
        # Retain the listener until all subscriptions are stopped.
        del tf_listener
        vision.book_client.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
