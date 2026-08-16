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


from pathlib import Path
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def load_tum(path: Path):
    samples = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            fields = line.strip().split()
            if len(fields) != 8:
                continue
            timestamp = float(fields[0])
            tx, ty, tz = (float(value) for value in fields[1:4])
            qx, qy, qz, qw = (float(value) for value in fields[4:8])
            samples.append((timestamp, tx, ty, tz, qx, qy, qz, qw))
    return samples


class EurocGroundTruthPublisher(Node):
    def __init__(self, tum_path: Path, frame_id: str) -> None:
        super().__init__("euroc_ground_truth_publisher")

        self._samples = load_tum(tum_path)
        if not self._samples:
            raise RuntimeError(f"no valid TUM samples found in {tum_path}")

        self._frame_id = frame_id
        self._start_time = self.get_clock().now()
        self._dataset_start = self._samples[0][0]
        self._dataset_end = self._samples[-1][0]
        self._next_index = 0
        self._last_index = 0

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._path_pub = self.create_publisher(
            PathMsg, "/euroc/ground_truth_path", latched_qos
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, "/euroc/ground_truth_pose", stream_qos
        )
        self._odom_pub = self.create_publisher(
            Odometry, "/euroc/ground_truth_odometry", stream_qos
        )

        self._path_msg = self._build_path_message()
        self._path_pub.publish(self._path_msg)

        self._timer = self.create_timer(0.02, self._publish_current_sample)
        self.get_logger().info(
            f"publishing EuRoC ground truth from {tum_path} "
            f"with {len(self._samples)} samples on /euroc/ground_truth_*"
        )

    def _build_path_message(self) -> PathMsg:
        path_msg = PathMsg()
        path_msg.header.frame_id = self._frame_id
        now = self.get_clock().now().to_msg()
        path_msg.header.stamp = now
        for sample in self._samples:
            pose = PoseStamped()
            pose.header.frame_id = self._frame_id
            pose.header.stamp = now
            pose.pose.position.x = sample[1]
            pose.pose.position.y = sample[2]
            pose.pose.position.z = sample[3]
            pose.pose.orientation.x = sample[4]
            pose.pose.orientation.y = sample[5]
            pose.pose.orientation.z = sample[6]
            pose.pose.orientation.w = sample[7]
            path_msg.poses.append(pose)
        return path_msg

    def _publish_current_sample(self) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        target_time = min(self._dataset_start + elapsed, self._dataset_end)

        while self._next_index + 1 < len(self._samples):
            next_sample_time = self._samples[self._next_index + 1][0]
            if next_sample_time > target_time:
                break
            self._next_index += 1

        self._last_index = self._next_index
        sample = self._samples[self._last_index]
        now = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self._frame_id
        pose_msg.header.stamp = now
        pose_msg.pose.position.x = sample[1]
        pose_msg.pose.position.y = sample[2]
        pose_msg.pose.position.z = sample[3]
        pose_msg.pose.orientation.x = sample[4]
        pose_msg.pose.orientation.y = sample[5]
        pose_msg.pose.orientation.z = sample[6]
        pose_msg.pose.orientation.w = sample[7]

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = "euroc_ground_truth_body"
        odom_msg.pose.pose = pose_msg.pose

        self._pose_pub.publish(pose_msg)
        self._odom_pub.publish(odom_msg)

        if target_time >= self._dataset_end:
            self._timer.cancel()
            # Keep the final pose available for any late subscribers.
            self.create_timer(
                1.0,
                lambda: (
                    self._path_pub.publish(self._path_msg),
                    self._pose_pub.publish(pose_msg),
                    self._odom_pub.publish(odom_msg),
                ),
            )


def main() -> int:
    if len(sys.argv) not in (2, 3):
        print(
            "usage: euroc_groundtruth_publisher.py <groundtruth.tum> [frame_id]",
            file=sys.stderr,
        )
        return 2

    tum_path = Path(sys.argv[1]).expanduser()
    frame_id = sys.argv[2] if len(sys.argv) == 3 else "basalt_world"
    if not tum_path.is_file():
        print(f"ground-truth TUM not found: {tum_path}", file=sys.stderr)
        return 1

    rclpy.init()
    node = None
    try:
        node = EurocGroundTruthPublisher(tum_path, frame_id)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
