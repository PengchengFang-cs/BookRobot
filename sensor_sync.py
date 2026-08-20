"""ROS-independent selection of one RGB-D frame and capture-time joints."""

from collections import defaultdict, deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TimedMessage:
    stamp_ns: int
    arrived_at_s: float
    message: object


@dataclass(frozen=True)
class TimedJointValue:
    stamp_ns: int
    value: float


@dataclass(frozen=True)
class RgbdSnapshot:
    captured_at_ns: int
    color: object
    depth: object
    info: object


def spin_until_counter_advances(
    counter,
    spin_once,
    *,
    timeout_s=0.05,
    clock=None,
):
    """Process callbacks until one new feedback sample arrives or time expires."""

    if clock is None:
        import time

        clock = time.monotonic
    initial = int(counter())
    deadline = float(clock()) + float(timeout_s)
    while int(counter()) == initial:
        remaining = deadline - float(clock())
        if remaining <= 0.0:
            return False
        spin_once(remaining)
    return True


def collect_successful_results(
    find_once,
    *,
    successful_samples=3,
    maximum_attempts=5,
    on_attempt=None,
):
    """Retry a detector and return distinct non-empty results."""

    successful_samples = int(successful_samples)
    maximum_attempts = int(maximum_attempts)
    if successful_samples < 1 or maximum_attempts < successful_samples:
        raise ValueError("视觉采样次数无效")
    samples = []
    for attempt in range(1, maximum_attempts + 1):
        result = find_once()
        if result:
            samples.append(result)
        if on_attempt is not None:
            on_attempt(attempt, len(samples), bool(result))
        if len(samples) == successful_samples:
            break
    return tuple(samples)


def message_stamp_ns(message):
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _nearest(samples, stamp_ns, maximum_skew_ns):
    if not samples:
        return None
    candidate = min(samples, key=lambda sample: abs(sample.stamp_ns - stamp_ns))
    if abs(candidate.stamp_ns - stamp_ns) > maximum_skew_ns:
        return None
    return candidate


def _frame_id(message):
    return str(message.header.frame_id)


def _dimensions(message):
    return int(message.width), int(message.height)


def select_rgbd_snapshot(
    *,
    color_samples,
    depth_samples,
    info_samples,
    arrived_after_s,
    captured_after_ns=-1,
    maximum_skew_ns=100_000_000,
):
    """Choose the newest unused color frame with matching depth and intrinsics."""

    for color_sample in reversed(tuple(color_samples)):
        stamp_ns = color_sample.stamp_ns
        if (
            color_sample.arrived_at_s < arrived_after_s
            or stamp_ns <= captured_after_ns
        ):
            continue
        depth_sample = _nearest(depth_samples, stamp_ns, maximum_skew_ns)
        info_sample = _nearest(info_samples, stamp_ns, maximum_skew_ns)
        if depth_sample is None or info_sample is None:
            continue
        messages = (
            color_sample.message,
            depth_sample.message,
            info_sample.message,
        )
        frame_ids = tuple(_frame_id(message) for message in messages)
        if not frame_ids[0] or len(set(frame_ids)) != 1:
            continue
        dimensions = tuple(_dimensions(message) for message in messages)
        if len(set(dimensions)) != 1:
            continue
        return RgbdSnapshot(
            captured_at_ns=stamp_ns,
            color=color_sample.message,
            depth=depth_sample.message,
            info=info_sample.message,
        )
    return None


def select_joint_positions(
    history,
    *,
    names,
    captured_at_ns,
    maximum_skew_ns=100_000_000,
):
    """Return each requested joint value nearest to the image timestamp."""

    result = {}
    for name in names:
        candidate = _nearest(history.get(name, ()), captured_at_ns, maximum_skew_ns)
        if candidate is None:
            return None
        result[name] = float(candidate.value)
    return result


class SensorSynchronizer:
    """Small bounded buffer used by the ROS callbacks in ``Vision``."""

    def __init__(
        self,
        *,
        # Wanda runs both streams at 5 Hz; aligned depth currently arrives
        # with a measured ~76 ms timestamp offset from its color frame.
        maximum_rgbd_skew_ns=100_000_000,
        # The current Wanda driver stamps joints about 182-187 ms after the
        # matched head RGB-D capture, measured on 2026-08-18.
        maximum_joint_skew_ns=250_000_000,
        frame_buffer_size=8,
        joint_buffer_size=512,
    ):
        self.maximum_rgbd_skew_ns = int(maximum_rgbd_skew_ns)
        self.maximum_joint_skew_ns = int(maximum_joint_skew_ns)
        self.colors = deque(maxlen=frame_buffer_size)
        self.depths = deque(maxlen=frame_buffer_size)
        self.infos = deque(maxlen=frame_buffer_size)
        self.joints = defaultdict(lambda: deque(maxlen=joint_buffer_size))

    def _add_message(
        self,
        target,
        message,
        arrived_at_s,
        received_at_ns=None,
    ):
        stamp_ns = message_stamp_ns(message)
        if stamp_ns <= 0 and received_at_ns is not None:
            stamp_ns = int(received_at_ns)
        if stamp_ns > 0:
            target.append(TimedMessage(stamp_ns, float(arrived_at_s), message))

    def add_color(self, message, *, arrived_at_s, received_at_ns=None):
        self._add_message(self.colors, message, arrived_at_s, received_at_ns)

    def add_depth(self, message, *, arrived_at_s, received_at_ns=None):
        self._add_message(self.depths, message, arrived_at_s, received_at_ns)

    def add_info(self, message, *, arrived_at_s, received_at_ns=None):
        self._add_message(self.infos, message, arrived_at_s, received_at_ns)

    def add_joints(self, message):
        stamp_ns = message_stamp_ns(message)
        if stamp_ns <= 0:
            return
        for name, value in zip(message.name, message.position):
            if math.isfinite(value):
                self.joints[name].append(TimedJointValue(stamp_ns, float(value)))

    def select(
        self,
        *,
        arrived_after_s,
        captured_after_ns,
        required_joints,
    ):
        snapshot = select_rgbd_snapshot(
            color_samples=self.colors,
            depth_samples=self.depths,
            info_samples=self.infos,
            arrived_after_s=arrived_after_s,
            captured_after_ns=captured_after_ns,
            maximum_skew_ns=self.maximum_rgbd_skew_ns,
        )
        if snapshot is None:
            return None
        positions = select_joint_positions(
            self.joints,
            names=required_joints,
            captured_at_ns=snapshot.captured_at_ns,
            maximum_skew_ns=self.maximum_joint_skew_ns,
        )
        if positions is None:
            return None
        return snapshot, positions
