#!/usr/bin/env python3

import argparse
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


DEFAULT_LEFT_ARM_JOINTS = [
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Realtime plot of dm_test reference, actual, and error joint trajectories."
    )
    parser.add_argument(
        "--window-sec",
        type=float,
        default=15.0,
        help="Rolling time window shown in the plot.",
    )
    parser.add_argument(
        "--update-hz",
        type=float,
        default=10.0,
        help="Plot refresh rate.",
    )
    parser.add_argument(
        "--joints",
        nargs="*",
        default=DEFAULT_LEFT_ARM_JOINTS,
        help="Joints to visualize.",
    )
    return parser.parse_args()


class TrackingPlotNode(Node):
    def __init__(self, joints, window_sec):
        super().__init__("dm_test_tracking_plot")
        self.joints = joints
        self.window_sec = window_sec
        self.lock = threading.Lock()
        self.start_time = time.time()

        self.reference_history = {
            joint: {"t": deque(), "position": deque()} for joint in joints
        }
        self.actual_history = {
            joint: {"t": deque(), "position": deque()} for joint in joints
        }
        self.error_history = {
            joint: {"t": deque(), "position": deque()} for joint in joints
        }

        self.create_subscription(
            JointState,
            "/dm_test/reference_joint_states",
            self.reference_callback,
            20,
        )
        self.create_subscription(
            JointState,
            "/dm_test/actual_joint_states",
            self.actual_callback,
            20,
        )
        self.create_subscription(
            JointState,
            "/dm_test/error_joint_states",
            self.error_callback,
            20,
        )

    def _now(self):
        return time.time() - self.start_time

    def _append_samples(self, history_map, msg):
        timestamp = self._now()
        name_to_index = {name: index for index, name in enumerate(msg.name)}

        with self.lock:
            for joint in self.joints:
                if joint not in name_to_index:
                    continue
                index = name_to_index[joint]
                if index >= len(msg.position):
                    continue

                value = msg.position[index]
                series = history_map[joint]
                series["t"].append(timestamp)
                series["position"].append(value)
                self._trim_series(series, timestamp)

    def _trim_series(self, series, current_time):
        while series["t"] and current_time - series["t"][0] > self.window_sec:
            series["t"].popleft()
            series["position"].popleft()

    def reference_callback(self, msg):
        self._append_samples(self.reference_history, msg)

    def actual_callback(self, msg):
        self._append_samples(self.actual_history, msg)

    def error_callback(self, msg):
        self._append_samples(self.error_history, msg)


def main():
    args = parse_args()
    if args.window_sec <= 0.0:
        raise ValueError("--window-sec must be > 0")
    if args.update_hz <= 0.0:
        raise ValueError("--update-hz must be > 0")

    rclpy.init()
    node = TrackingPlotNode(args.joints, args.window_sec)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    figure, axes = plt.subplots(len(args.joints), 1, figsize=(12, 2.7 * len(args.joints)), sharex=True)
    if len(args.joints) == 1:
        axes = [axes]

    reference_lines = {}
    actual_lines = {}
    error_lines = {}

    for axis, joint in zip(axes, args.joints):
        axis.set_title(joint)
        axis.set_ylabel("rad")
        axis.grid(True, alpha=0.3)
        reference_lines[joint], = axis.plot([], [], label="reference", linewidth=1.8)
        actual_lines[joint], = axis.plot([], [], label="actual", linewidth=1.4)
        error_lines[joint], = axis.plot([], [], label="error", linewidth=1.0, linestyle="--")
        axis.legend(loc="upper left")

    axes[-1].set_xlabel("time (s)")
    figure.suptitle("dm_test realtime tracking curves", fontsize=14)
    figure.tight_layout()

    def update(_frame_index):
        with node.lock:
            current_time = node._now()
            min_time = max(0.0, current_time - args.window_sec)

            for axis, joint in zip(axes, args.joints):
                ref_series = node.reference_history[joint]
                act_series = node.actual_history[joint]
                err_series = node.error_history[joint]

                reference_lines[joint].set_data(list(ref_series["t"]), list(ref_series["position"]))
                actual_lines[joint].set_data(list(act_series["t"]), list(act_series["position"]))
                error_lines[joint].set_data(list(err_series["t"]), list(err_series["position"]))

                all_values = (
                    list(ref_series["position"])
                    + list(act_series["position"])
                    + list(err_series["position"])
                )
                if all_values:
                    value_min = min(all_values)
                    value_max = max(all_values)
                    if abs(value_max - value_min) < 1e-6:
                        pad = 0.05
                    else:
                        pad = 0.1 * (value_max - value_min)
                    axis.set_ylim(value_min - pad, value_max + pad)
                axis.set_xlim(min_time, max(current_time, args.window_sec))

        return (
            list(reference_lines.values())
            + list(actual_lines.values())
            + list(error_lines.values())
        )

    animation = FuncAnimation(
        figure,
        update,
        interval=1000.0 / args.update_hz,
        blit=False,
        cache_frame_data=False,
    )
    _ = animation

    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
