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
import os
import signal
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def associate_by_time(reference_times_ns, query_times_ns, max_delta_ns):
    matched_ref = []
    matched_query = []
    matched_times = []
    j = 0
    for q_idx, q_t in enumerate(query_times_ns):
        while j + 1 < len(reference_times_ns) and reference_times_ns[j + 1] <= q_t:
            j += 1
        candidates = [j]
        if j + 1 < len(reference_times_ns):
            candidates.append(j + 1)
        best_idx = None
        best_delta = None
        for cand in candidates:
            delta = abs(reference_times_ns[cand] - q_t)
            if delta <= max_delta_ns and (best_delta is None or delta < best_delta):
                best_delta = delta
                best_idx = cand
        if best_idx is not None:
            matched_ref.append(best_idx)
            matched_query.append(q_idx)
            matched_times.append(q_t)
    return np.array(matched_ref), np.array(matched_query), np.array(matched_times)


def align_points(reference_xyz, estimate_xyz):
    ref_centroid = reference_xyz.mean(axis=0)
    est_centroid = estimate_xyz.mean(axis=0)
    ref_centered = reference_xyz - ref_centroid
    est_centered = estimate_xyz - est_centroid
    h = est_centered.T @ ref_centered
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = ref_centroid - r @ est_centroid
    aligned = (r @ estimate_xyz.T).T + t
    return aligned, r, t


def compute_metrics(reference_xyz, estimate_xyz):
    delta = estimate_xyz - reference_xyz
    err = np.linalg.norm(delta, axis=1)
    return {
        "count": len(err),
        "rmse": float(np.sqrt(np.mean(np.square(err)))),
        "mae": float(np.mean(np.abs(err))),
        "max": float(np.max(err)),
        "latest": float(err[-1]),
        "mean_dx": float(np.mean(delta[:, 0])),
        "mean_dy": float(np.mean(delta[:, 1])),
        "mean_dz": float(np.mean(delta[:, 2])),
        "rmse_x": float(np.sqrt(np.mean(np.square(delta[:, 0])))),
        "rmse_y": float(np.sqrt(np.mean(np.square(delta[:, 1])))),
        "rmse_z": float(np.sqrt(np.mean(np.square(delta[:, 2])))),
    }, delta, err


class OfflineComparer(Node):
    def __init__(self) -> None:
        super().__init__("basalt_truth_offline_comparer")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.max_association_delta_ns = int(
            self.declare_parameter("max_association_delta_ns", 50_000_000)
            .get_parameter_value()
            .integer_value
        )
        self.truth_topic = (
            self.declare_parameter(
                "truth_topic", "/gz/ground_truth_odom_raw"
            )
            .get_parameter_value()
            .string_value
        )
        self.estimate_topic = (
            self.declare_parameter("estimate_topic", "/basalt/odometry")
            .get_parameter_value()
            .string_value
        )
        self.corrected_topic = (
            self.declare_parameter("corrected_topic", "/geo/corrected_pose")
            .get_parameter_value()
            .string_value
        )

        self.truth_samples = []
        self.estimate_samples = []
        self.corrected_samples = []
        self.start_wall_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path(
            self.declare_parameter("output_dir", "~/.ros/basalt_wrapper")
            .get_parameter_value()
            .string_value
        ).expanduser()
        self.csv_path = str(
            output_dir / f"basalt_truth_compare_offline_{self.start_wall_time}.csv"
        )
        self.corrected_csv_path = str(
            output_dir
            / f"geo_corrected_truth_compare_offline_{self.start_wall_time}.csv"
        )
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)

        self.create_subscription(Odometry, self.truth_topic, self.truth_callback, qos)
        self.create_subscription(
            Odometry, self.estimate_topic, self.estimate_callback, qos
        )
        self.create_subscription(
            PoseStamped, self.corrected_topic, self.corrected_callback, qos
        )
        self.timer = self.create_timer(2.0, self.print_status)

        self.get_logger().info(
            f"Collecting {self.estimate_topic} against {self.truth_topic}"
        )
        self.get_logger().info(
            f"Also collecting {self.corrected_topic} against {self.truth_topic}"
        )
        self.get_logger().info(
            f"Will align offline with max association delta "
            f"{self.max_association_delta_ns / 1e6:.1f} ms"
        )

    def truth_callback(self, msg: Odometry) -> None:
        self.truth_samples.append(
            (
                stamp_to_ns(msg.header.stamp),
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )
        )

    def estimate_callback(self, msg: Odometry) -> None:
        self.estimate_samples.append(
            (
                stamp_to_ns(msg.header.stamp),
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )
        )

    def corrected_callback(self, msg: PoseStamped) -> None:
        self.corrected_samples.append(
            (
                stamp_to_ns(msg.header.stamp),
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            )
        )

    def print_status(self) -> None:
        self.get_logger().info(
            f"buffering estimate={len(self.estimate_samples)} "
            f"corrected={len(self.corrected_samples)} "
            f"truth={len(self.truth_samples)}"
        )

    def summarize_track(self, label, samples, csv_path) -> bool:
        if len(self.truth_samples) < 20 or len(samples) < 20:
            print(
                f"{label}: not enough samples collected "
                f"(estimate={len(samples)} truth={len(self.truth_samples)})",
                file=sys.stderr,
            )
            return False
        truth = np.array(self.truth_samples, dtype=float)
        estimate = np.array(samples, dtype=float)

        truth_idx, est_idx, matched_times = associate_by_time(
            truth[:, 0].astype(np.int64),
            estimate[:, 0].astype(np.int64),
            self.max_association_delta_ns,
        )
        if len(truth_idx) < 20:
            print(
                f"{label}: too few matched samples after timestamp association "
                f"(matched={len(truth_idx)})",
                file=sys.stderr,
            )
            return False

        truth_xyz = truth[truth_idx, 1:4]
        estimate_xyz = estimate[est_idx, 1:4]
        aligned_xyz, _, _ = align_points(truth_xyz, estimate_xyz)
        metrics, delta, err = compute_metrics(truth_xyz, aligned_xyz)

        with open(csv_path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "track",
                    "sample_index",
                    "estimate_t_ns",
                    "truth_t_ns",
                    "estimate_x",
                    "estimate_y",
                    "estimate_z",
                    "truth_x",
                    "truth_y",
                    "truth_z",
                    "aligned_x",
                    "aligned_y",
                    "aligned_z",
                    "dx",
                    "dy",
                    "dz",
                    "err",
                ]
            )
            for i, (t_idx, e_idx) in enumerate(zip(truth_idx, est_idx), start=1):
                writer.writerow(
                    [
                        label,
                        i,
                        int(estimate[e_idx, 0]),
                        int(truth[t_idx, 0]),
                        f"{estimate[e_idx, 1]:.6f}",
                        f"{estimate[e_idx, 2]:.6f}",
                        f"{estimate[e_idx, 3]:.6f}",
                        f"{truth[t_idx, 1]:.6f}",
                        f"{truth[t_idx, 2]:.6f}",
                        f"{truth[t_idx, 3]:.6f}",
                        f"{aligned_xyz[i - 1, 0]:.6f}",
                        f"{aligned_xyz[i - 1, 1]:.6f}",
                        f"{aligned_xyz[i - 1, 2]:.6f}",
                        f"{delta[i - 1, 0]:.6f}",
                        f"{delta[i - 1, 1]:.6f}",
                        f"{delta[i - 1, 2]:.6f}",
                        f"{err[i - 1]:.6f}",
                    ]
                )

        print(f"Summary view ({label}):")
        print(f"  matched samples: {metrics['count']}")
        print(f"  RMSE:            {metrics['rmse']:.6f} m")
        print(f"  mean abs error:  {metrics['mae']:.6f} m")
        print(f"  max error:       {metrics['max']:.6f} m")
        print(f"  latest error:    {metrics['latest']:.6f} m")
        print(
            f"  mean axis bias:  ({metrics['mean_dx']:.6f}, "
            f"{metrics['mean_dy']:.6f}, {metrics['mean_dz']:.6f}) m"
        )
        print(
            f"  axis RMSE:       ({metrics['rmse_x']:.6f}, "
            f"{metrics['rmse_y']:.6f}, {metrics['rmse_z']:.6f}) m"
        )
        print()
        print("Outputs:")
        print(f"  offline compare csv: {csv_path}")
        return True

    def finalize(self) -> int:
        raw_ok = self.summarize_track("raw_basalt", self.estimate_samples, self.csv_path)
        corrected_ok = self.summarize_track(
            "geo_corrected", self.corrected_samples, self.corrected_csv_path
        )
        if not raw_ok and not corrected_ok:
            print(
                "No comparable estimate tracks were available. If the stack was running, "
                "check basalt_wrapper.log for a growing pending IMU/stereo queue.",
                file=sys.stderr,
            )
            return 1
        return 0


def main() -> int:
    rclpy.init()
    node = OfflineComparer()

    exit_code = 0
    finalized = False

    def _shutdown(*_args) -> None:
        nonlocal exit_code, finalized
        if finalized:
            return
        finalized = True
        try:
            exit_code = node.finalize()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok() and not finalized:
            try:
                exit_code = node.finalize()
            except Exception:
                pass
            node.destroy_node()
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
