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


from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def is_zero_stamp(stamp) -> bool:
    return stamp.sec == 0 and stamp.nanosec == 0


class GazeboGroundTruthOdometry(Node):
    def __init__(self) -> None:
        super().__init__("gz_ground_truth_odometry")

        self.input_topic = (
            self.declare_parameter("input_topic", "/gz_dynamic_tf")
            .get_parameter_value()
            .string_value
        )
        self.output_odom_topic = (
            self.declare_parameter("output_odom_topic", "/gz/ground_truth_odom_raw")
            .get_parameter_value()
            .string_value
        )
        self.output_path_topic = (
            self.declare_parameter("output_path_topic", "/gz/ground_truth_path_raw")
            .get_parameter_value()
            .string_value
        )
        self.child_frame_suffix = (
            self.declare_parameter("child_frame_suffix", "/link/base_link")
            .get_parameter_value()
            .string_value
        )
        self.camera_output_odom_topic = (
            self.declare_parameter(
                "camera_output_odom_topic", "/gz/downward_camera_odom_raw"
            )
            .get_parameter_value()
            .string_value
        )
        self.camera_child_frame_suffix = (
            self.declare_parameter(
                "camera_child_frame_suffix", "/link/downward_camera_link"
            )
            .get_parameter_value()
            .string_value
        )
        # ros_gz_bridge in Humble preserves Pose_V ordering but currently drops
        # Pose.name when converting to TFMessage. For this custom model the
        # model pose is entry 0 and downward_camera_link is entry 10.
        self.base_fallback_index = (
            self.declare_parameter("base_fallback_index", 0)
            .get_parameter_value()
            .integer_value
        )
        self.camera_fallback_index = (
            self.declare_parameter("camera_fallback_index", 10)
            .get_parameter_value()
            .integer_value
        )
        self._reported_unnamed_fallback = False
        self.path_frame_id = (
            self.declare_parameter("path_frame_id", "basalt_world")
            .get_parameter_value()
            .string_value
        )
        self.max_path_length = (
            self.declare_parameter("max_path_length", 10000)
            .get_parameter_value()
            .integer_value
        )
        self.publish_path = (
            self.declare_parameter("publish_path", True)
            .get_parameter_value()
            .bool_value
        )
        self.path_publish_stride = (
            self.declare_parameter("path_publish_stride", 10)
            .get_parameter_value()
            .integer_value
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, output_qos)
        self.camera_odom_pub = self.create_publisher(
            Odometry, self.camera_output_odom_topic, output_qos
        )
        self.path_pub = self.create_publisher(Path, self.output_path_topic, output_qos)
        self.sub = self.create_subscription(
            TFMessage, self.input_topic, self.callback, sensor_qos
        )
        self.path = Path()
        self.path.header.frame_id = self.path_frame_id
        self._path_counter = 0

        self.get_logger().info(
            f"Publishing {self.output_odom_topic}, {self.camera_output_odom_topic}, "
            f"and {self.output_path_topic} "
            f"from {self.input_topic}; publish_path={self.publish_path} "
            f"path_publish_stride={self.path_publish_stride}"
        )

    def callback(self, msg: TFMessage) -> None:
        if not msg.transforms:
            return

        selected = None
        selected_camera = None
        for transform in msg.transforms:
            child = transform.child_frame_id
            if child.endswith(self.child_frame_suffix) or child == "base_link":
                selected = transform
            if child.endswith(self.camera_child_frame_suffix) or child == "downward_camera_link":
                selected_camera = transform
        if selected is None:
            if all(not transform.child_frame_id for transform in msg.transforms):
                if 0 <= self.base_fallback_index < len(msg.transforms):
                    selected = msg.transforms[self.base_fallback_index]
                if 0 <= self.camera_fallback_index < len(msg.transforms):
                    selected_camera = msg.transforms[self.camera_fallback_index]
                if not self._reported_unnamed_fallback:
                    self.get_logger().info(
                        "TFMessage has unnamed transforms; using ordered Pose_V "
                        f"fallback base={self.base_fallback_index} "
                        f"camera={self.camera_fallback_index} count={len(msg.transforms)}"
                    )
                    self._reported_unnamed_fallback = True
            if selected is None:
                self.get_logger().warn(
                    f"No transform ending in {self.child_frame_suffix!r} on {self.input_topic}",
                    throttle_duration_sec=5.0,
                )
                return

        odom = self.transform_to_odometry(selected, "base_link")
        self.odom_pub.publish(odom)

        if selected_camera is not None:
            self.camera_odom_pub.publish(
                self.compose_link_odometry(
                    selected,
                    selected_camera,
                    "downward_camera_link",
                )
            )
        else:
            self.get_logger().warn(
                f"No transform ending in {self.camera_child_frame_suffix!r} on {self.input_topic}",
                throttle_duration_sec=5.0,
            )

        stamp = odom.header.stamp
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        if self.publish_path:
            self.path.header.stamp = stamp
            self.path.poses.append(pose)
            if len(self.path.poses) > self.max_path_length:
                del self.path.poses[: len(self.path.poses) - self.max_path_length]
            self._path_counter += 1
            if self._path_counter % max(1, self.path_publish_stride) == 0:
                self.path_pub.publish(self.path)

    def transform_to_odometry(self, selected, unnamed_child_frame: str) -> Odometry:
        stamp = selected.header.stamp
        if is_zero_stamp(stamp):
            stamp = self.get_clock().now().to_msg()

        transform = selected.transform
        rotation = transform.rotation
        norm = math.sqrt(
            rotation.w * rotation.w
            + rotation.x * rotation.x
            + rotation.y * rotation.y
            + rotation.z * rotation.z
        )
        if norm == 0.0:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
        else:
            qw = rotation.w / norm
            qx = rotation.x / norm
            qy = rotation.y / norm
            qz = rotation.z / norm

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.path_frame_id
        odom.child_frame_id = selected.child_frame_id or unnamed_child_frame
        odom.pose.pose.position.x = transform.translation.x
        odom.pose.pose.position.y = transform.translation.y
        odom.pose.pose.position.z = transform.translation.z
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        return odom

    @staticmethod
    def normalized_quaternion(rotation):
        norm = math.sqrt(
            rotation.w * rotation.w
            + rotation.x * rotation.x
            + rotation.y * rotation.y
            + rotation.z * rotation.z
        )
        if norm == 0.0:
            return 1.0, 0.0, 0.0, 0.0
        return (
            rotation.w / norm,
            rotation.x / norm,
            rotation.y / norm,
            rotation.z / norm,
        )

    def compose_link_odometry(self, model_pose, link_pose, child_frame: str) -> Odometry:
        """Compose a Pose_V model world pose with its link-local pose."""
        model = model_pose.transform
        link = link_pose.transform
        mw, mx, my, mz = self.normalized_quaternion(model.rotation)
        lw, lx, ly, lz = self.normalized_quaternion(link.rotation)

        # Rotate the local link translation by the model world attitude using
        # q * [0,v] * conjugate(q).
        vx, vy, vz = link.translation.x, link.translation.y, link.translation.z
        tx = 2.0 * (my * vz - mz * vy)
        ty = 2.0 * (mz * vx - mx * vz)
        tz = 2.0 * (mx * vy - my * vx)
        rx = vx + mw * tx + (my * tz - mz * ty)
        ry = vy + mw * ty + (mz * tx - mx * tz)
        rz = vz + mw * tz + (mx * ty - my * tx)

        qw = mw * lw - mx * lx - my * ly - mz * lz
        qx = mw * lx + mx * lw + my * lz - mz * ly
        qy = mw * ly - mx * lz + my * lw + mz * lx
        qz = mw * lz + mx * ly - my * lx + mz * lw
        qnorm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if qnorm > 0.0:
            qw, qx, qy, qz = qw / qnorm, qx / qnorm, qy / qnorm, qz / qnorm

        stamp = model_pose.header.stamp
        if is_zero_stamp(stamp):
            stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.path_frame_id
        odom.child_frame_id = child_frame
        odom.pose.pose.position.x = model.translation.x + rx
        odom.pose.pose.position.y = model.translation.y + ry
        odom.pose.pose.position.z = model.translation.z + rz
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        return odom


def main() -> int:
    rclpy.init()
    node = GazeboGroundTruthOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
