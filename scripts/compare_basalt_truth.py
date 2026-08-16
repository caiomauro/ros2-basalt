#!/usr/bin/env python3

# Copyright 2026 Caio Mauro
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Caio Mauro nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import csv
import math
import os
import signal
import sys
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class BasaltTruthComparer(Node):
    def __init__(self) -> None:
        super().__init__("basalt_truth_comparer")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.truth_buffer = deque(maxlen=500)
        self.sample_count = 0
        self.sum_sq_error = 0.0
        self.sum_abs_error = 0.0
        self.max_error = 0.0
        self.last_error = None
        self.start_wall_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path(
            self.declare_parameter("output_dir", "~/.ros/basalt_wrapper")
            .get_parameter_value()
            .string_value
        ).expanduser()
        self.csv_path = str(
            output_dir / f"basalt_truth_compare_{self.start_wall_time}.csv"
        )
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "sample_index",
                "basalt_t_ns",
                "truth_t_ns",
                "truth_dt_ms",
                "basalt_x",
                "basalt_y",
                "basalt_z",
                "truth_x",
                "truth_y",
                "truth_z",
                "dx",
                "dy",
                "dz",
                "err",
            ]
        )
        self.csv_file.flush()

        self.create_subscription(
            Odometry,
            "/gz/ground_truth_odom_aligned",
            self.truth_callback,
            qos,
        )
        self.create_subscription(
            Odometry,
            "/basalt/odometry",
            self.basalt_callback,
            qos,
        )
        self.timer = self.create_timer(1.0, self.print_status)

        self.get_logger().info(
            "Comparing /basalt/odometry against /gz/ground_truth_odom_aligned"
        )
        self.get_logger().info(f"Writing CSV log to {self.csv_path}")

    def truth_callback(self, msg: Odometry) -> None:
        self.truth_buffer.append(
            (
                stamp_to_ns(msg.header.stamp),
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )
        )

    def basalt_callback(self, msg: Odometry) -> None:
        if not self.truth_buffer:
            return

        basalt_t_ns = stamp_to_ns(msg.header.stamp)
        basalt_x = msg.pose.pose.position.x
        basalt_y = msg.pose.pose.position.y
        basalt_z = msg.pose.pose.position.z

        best = min(self.truth_buffer, key=lambda sample: abs(sample[0] - basalt_t_ns))
        dt_ms = abs(best[0] - basalt_t_ns) / 1_000_000.0

        dx = basalt_x - best[1]
        dy = basalt_y - best[2]
        dz = basalt_z - best[3]
        err = math.sqrt(dx * dx + dy * dy + dz * dz)

        self.sample_count += 1
        self.last_error = (err, dx, dy, dz, dt_ms)
        self.sum_sq_error += err * err
        self.sum_abs_error += err
        self.max_error = max(self.max_error, err)
        self.csv_writer.writerow(
            [
                self.sample_count,
                basalt_t_ns,
                best[0],
                f"{dt_ms:.3f}",
                f"{basalt_x:.6f}",
                f"{basalt_y:.6f}",
                f"{basalt_z:.6f}",
                f"{best[1]:.6f}",
                f"{best[2]:.6f}",
                f"{best[3]:.6f}",
                f"{dx:.6f}",
                f"{dy:.6f}",
                f"{dz:.6f}",
                f"{err:.6f}",
            ]
        )
        if self.sample_count % 25 == 0:
            self.csv_file.flush()

    def print_status(self) -> None:
        if self.sample_count == 0 or self.last_error is None:
            self.get_logger().info("Waiting for Basalt and truth samples...")
            return

        err, dx, dy, dz, dt_ms = self.last_error
        rmse = math.sqrt(self.sum_sq_error / self.sample_count)
        mean_abs = self.sum_abs_error / self.sample_count
        self.get_logger().info(
            "samples=%d latest_err=%.3fm rmse=%.3fm mean_abs=%.3fm max=%.3fm "
            "delta_xyz=(%.3f, %.3f, %.3f)m truth_dt=%.1fms"
            % (
                self.sample_count,
                err,
                rmse,
                mean_abs,
                self.max_error,
                dx,
                dy,
                dz,
                dt_ms,
            )
        )


def main() -> int:
    rclpy.init()
    node = BasaltTruthComparer()

    def _shutdown(*_args) -> None:
        if not node.csv_file.closed:
            node.csv_file.flush()
            node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    finally:
        if not node.csv_file.closed:
            node.csv_file.flush()
            node.csv_file.close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
