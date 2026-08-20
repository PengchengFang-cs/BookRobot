import unittest
from types import SimpleNamespace

import sensor_sync
from sensor_sync import (
    SensorSynchronizer,
    TimedJointValue,
    TimedMessage,
    select_joint_positions,
    select_rgbd_snapshot,
)


def message(stamp_ns, *, frame="head_rgbd_color_optical_frame", width=1920, height=1080):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
            frame_id=frame,
        ),
        width=width,
        height=height,
    )


class SensorSyncTests(unittest.TestCase):
    def test_spins_until_feedback_counter_advances(self):
        waiter = getattr(sensor_sync, "spin_until_counter_advances", None)
        self.assertIsNotNone(waiter)
        state = {"counter": 4, "spins": 0, "now": 0.0}

        def spin_once(_timeout_s):
            state["spins"] += 1
            state["now"] += 0.01
            if state["spins"] == 3:
                state["counter"] += 1

        result = waiter(
            lambda: state["counter"],
            spin_once,
            timeout_s=0.05,
            clock=lambda: state["now"],
        )

        self.assertTrue(result)
        self.assertEqual(state["spins"], 3)

    def test_feedback_counter_wait_is_bounded(self):
        waiter = getattr(sensor_sync, "spin_until_counter_advances", None)
        self.assertIsNotNone(waiter)
        state = {"now": 0.0, "spins": 0}

        def spin_once(timeout_s):
            state["spins"] += 1
            state["now"] += timeout_s

        result = waiter(
            lambda: 7,
            spin_once,
            timeout_s=0.05,
            clock=lambda: state["now"],
        )

        self.assertFalse(result)
        self.assertEqual(state["spins"], 1)

    def test_zero_camera_stamp_uses_ros_receive_time(self):
        sync = SensorSynchronizer(maximum_rgbd_skew_ns=5_000_000)
        received_ns = 7_000_000_000
        sync.add_color(
            message(0),
            arrived_at_s=50.0,
            received_at_ns=received_ns,
        )
        sync.add_depth(
            message(0),
            arrived_at_s=50.001,
            received_at_ns=received_ns + 1_000_000,
        )
        sync.add_info(
            message(0),
            arrived_at_s=50.002,
            received_at_ns=received_ns + 2_000_000,
        )
        joints = message(received_ns + 1_000_000)
        joints.name = ["body_joint", "joint_head0", "joint_head1"]
        joints.position = [0.2, 0.0, -0.3]
        sync.add_joints(joints)

        selected = sync.select(
            arrived_after_s=49.0,
            captured_after_ns=0,
            required_joints=("body_joint", "joint_head0", "joint_head1"),
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected[0].captured_at_ns, received_ns)

    def test_default_joint_history_covers_delayed_camera_delivery(self):
        synchronizer = SensorSynchronizer()

        self.assertEqual(synchronizer.joints["body_joint"].maxlen, 512)

    def test_default_accepts_measured_joint_timestamp_offset(self):
        color_stamp = 6_000_000_000
        sync = SensorSynchronizer()
        sync.add_color(message(color_stamp), arrived_at_s=40.0)
        sync.add_depth(message(color_stamp + 80_000_000), arrived_at_s=40.1)
        sync.add_info(message(color_stamp), arrived_at_s=40.0)
        joints = message(color_stamp + 190_000_000)
        joints.name = ["body_joint", "joint_head0", "joint_head1"]
        joints.position = [0.2, 0.0, 0.25]
        sync.add_joints(joints)

        selected = sync.select(
            arrived_after_s=39.0,
            captured_after_ns=0,
            required_joints=("body_joint", "joint_head0", "joint_head1"),
        )

        self.assertIsNotNone(selected)

    def sample(self, stamp_ns, arrived_at_s, **message_values):
        return TimedMessage(
            stamp_ns=stamp_ns,
            arrived_at_s=arrived_at_s,
            message=message(stamp_ns, **message_values),
        )

    def test_selects_same_capture_rgbd_and_camera_info(self):
        stamp = 1_000_000_000
        snapshot = select_rgbd_snapshot(
            color_samples=(self.sample(stamp, 10.0),),
            depth_samples=(self.sample(stamp + 1_000_000, 10.01),),
            info_samples=(self.sample(stamp, 9.99),),
            arrived_after_s=9.5,
            captured_after_ns=0,
            maximum_skew_ns=5_000_000,
        )

        self.assertIsNotNone(snapshot)
        self.assertEqual(snapshot.captured_at_ns, stamp)
        self.assertEqual(snapshot.color.header.frame_id, snapshot.depth.header.frame_id)

    def test_rejects_old_or_unsynchronized_frames(self):
        stamp = 2_000_000_000
        snapshot = select_rgbd_snapshot(
            color_samples=(self.sample(stamp, 10.0),),
            depth_samples=(self.sample(stamp + 30_000_000, 10.0),),
            info_samples=(self.sample(stamp, 10.0),),
            arrived_after_s=9.0,
            captured_after_ns=stamp,
            maximum_skew_ns=5_000_000,
        )

        self.assertIsNone(snapshot)

    def test_rejects_different_optical_frames_or_dimensions(self):
        stamp = 3_000_000_000
        wrong_frame = select_rgbd_snapshot(
            color_samples=(self.sample(stamp, 10.0),),
            depth_samples=(self.sample(stamp, 10.0, frame="depth_optical_frame"),),
            info_samples=(self.sample(stamp, 10.0),),
            arrived_after_s=9.0,
        )
        wrong_size = select_rgbd_snapshot(
            color_samples=(self.sample(stamp, 10.0),),
            depth_samples=(self.sample(stamp, 10.0, width=640, height=400),),
            info_samples=(self.sample(stamp, 10.0),),
            arrived_after_s=9.0,
        )

        self.assertIsNone(wrong_frame)
        self.assertIsNone(wrong_size)

    def test_selects_joint_values_nearest_to_capture(self):
        history = {
            "body_joint": (
                TimedJointValue(900_000_000, 0.1),
                TimedJointValue(1_002_000_000, 0.2),
            ),
            "joint_head0": (TimedJointValue(999_000_000, 0.3),),
            "joint_head1": (TimedJointValue(1_001_000_000, -0.2),),
        }

        result = select_joint_positions(
            history,
            names=("body_joint", "joint_head0", "joint_head1"),
            captured_at_ns=1_000_000_000,
            maximum_skew_ns=5_000_000,
        )

        self.assertEqual(
            result,
            {"body_joint": 0.2, "joint_head0": 0.3, "joint_head1": -0.2},
        )

    def test_returns_none_without_capture_time_joint_values(self):
        history = {
            "body_joint": (TimedJointValue(800_000_000, 0.1),),
            "joint_head0": (TimedJointValue(800_000_000, 0.0),),
            "joint_head1": (TimedJointValue(800_000_000, 0.0),),
        }

        result = select_joint_positions(
            history,
            names=("body_joint", "joint_head0", "joint_head1"),
            captured_at_ns=1_000_000_000,
            maximum_skew_ns=5_000_000,
        )

        self.assertIsNone(result)

    def test_synchronizer_builds_one_unused_snapshot_with_capture_time_joints(self):
        stamp = 4_000_000_000
        sync = SensorSynchronizer(maximum_rgbd_skew_ns=5_000_000)
        sync.add_color(message(stamp), arrived_at_s=20.0)
        sync.add_depth(message(stamp + 1_000_000), arrived_at_s=20.01)
        sync.add_info(message(stamp), arrived_at_s=20.0)
        joints = message(stamp)
        joints.name = ["body_joint", "joint_head0", "joint_head1"]
        joints.position = [0.2, 0.3, -0.2]
        sync.add_joints(joints)

        selected = sync.select(
            arrived_after_s=19.0,
            captured_after_ns=0,
            required_joints=("body_joint", "joint_head0", "joint_head1"),
        )
        reused = sync.select(
            arrived_after_s=19.0,
            captured_after_ns=stamp,
            required_joints=("body_joint", "joint_head0", "joint_head1"),
        )

        snapshot, positions = selected
        self.assertEqual(snapshot.captured_at_ns, stamp)
        self.assertEqual(positions["body_joint"], 0.2)
        self.assertIsNone(reused)

    def test_default_matches_wanda_color_depth_timestamp_offset(self):
        color_stamp = 5_000_000_000
        sync = SensorSynchronizer()
        sync.add_color(message(color_stamp), arrived_at_s=30.0)
        sync.add_depth(message(color_stamp + 76_000_000), arrived_at_s=30.08)
        sync.add_info(message(color_stamp), arrived_at_s=30.0)
        joints = message(color_stamp + 20_000_000)
        joints.name = ["body_joint", "joint_head0", "joint_head1"]
        joints.position = [0.2, 0.0, 0.25]
        sync.add_joints(joints)

        selected = sync.select(
            arrived_after_s=29.0,
            captured_after_ns=0,
            required_joints=("body_joint", "joint_head0", "joint_head1"),
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected[0].captured_at_ns, color_stamp)


if __name__ == "__main__":
    unittest.main()
