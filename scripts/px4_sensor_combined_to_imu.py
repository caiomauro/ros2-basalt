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


import math

import rclpy
from rclpy.executors import ExternalShutdownException
from px4_msgs.msg import SensorCombined
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class Px4RosClockMapper:
    """Preserve PX4 sample spacing while surviving a PX4 clock discontinuity."""

    def __init__(self, discontinuity_threshold_ns: int):
        self.discontinuity_threshold_ns = int(discontinuity_threshold_ns)
        self.offset_ns = None
        self.last_px4_ns = 0
        self.last_ros_receipt_ns = 0
        self.last_output_ns = 0
        self.rebase_count = 0

    def map(self, px4_ns: int, ros_receipt_ns: int):
        rebased = False
        source_dt_ns = 0
        receipt_dt_ns = 0
        old_skew_ns = 0

        if self.offset_ns is None:
            self.offset_ns = px4_ns - ros_receipt_ns
        else:
            source_dt_ns = px4_ns - self.last_px4_ns
            receipt_dt_ns = ros_receipt_ns - self.last_ros_receipt_ns
            candidate_ns = px4_ns - self.offset_ns
            old_skew_ns = candidate_ns - ros_receipt_ns

            # A PX4 SITL time jump must not be passed through to Basalt. Compare
            # source-clock progress with ROS/Gazebo-clock progress so ordinary
            # DDS batching (small receipt-time jitter) does not cause a rebase.
            clock_progress_error_ns = source_dt_ns - receipt_dt_ns
            # The observed PX4 SITL failure is a forward jump in PX4 source
            # time. A delayed ROS callback produces the opposite signature
            # (large negative progress error / skew) and must not permanently
            # shift the sensor clock just because the executor was briefly busy.
            if (
                clock_progress_error_ns > self.discontinuity_threshold_ns
                and old_skew_ns > self.discontinuity_threshold_ns
            ):
                target_ns = max(self.last_output_ns + 1, ros_receipt_ns)
                self.offset_ns = px4_ns - target_ns
                rebased = True
                self.rebase_count += 1

        output_ns = px4_ns - self.offset_ns
        if output_ns <= self.last_output_ns:
            output_ns = self.last_output_ns + 1

        self.last_px4_ns = px4_ns
        self.last_ros_receipt_ns = ros_receipt_ns
        self.last_output_ns = output_ns
        return output_ns, rebased, source_dt_ns, receipt_dt_ns, old_skew_ns


class Px4SensorCombinedToImu(Node):
    def __init__(self):
        super().__init__("px4_sensor_combined_to_imu")
        self.declare_parameter("input_topic", "/fmu/out/sensor_combined")
        self.declare_parameter("output_topic", "/imu0")
        self.declare_parameter("frame_id", "imu0")
        self.declare_parameter("use_ros_time", True)
        self.declare_parameter("gyro_variance", [1e-4, 1e-4, 1e-4])
        self.declare_parameter("accel_variance", [1e-3, 1e-3, 1e-3])
        self.declare_parameter("clock_discontinuity_threshold_sec", 0.10)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.use_ros_time = bool(self.get_parameter("use_ros_time").value)
        self.gyro_variance = list(self.get_parameter("gyro_variance").value)
        self.accel_variance = list(self.get_parameter("accel_variance").value)
        if len(self.gyro_variance) != 3 or len(self.accel_variance) != 3:
            raise ValueError("gyro_variance and accel_variance must have 3 values")
        discontinuity_threshold_sec = float(
            self.get_parameter("clock_discontinuity_threshold_sec").value
        )
        if discontinuity_threshold_sec <= 0.0:
            raise ValueError("clock_discontinuity_threshold_sec must be > 0")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.pub = self.create_publisher(
            Imu, str(self.get_parameter("output_topic").value), qos
        )
        self.sub = self.create_subscription(
            SensorCombined,
            str(self.get_parameter("input_topic").value),
            self.callback,
            qos,
        )
        self.count = 0
        self.zero_time_drops = 0
        self.clock_mapper = Px4RosClockMapper(
            round(discontinuity_threshold_sec * 1_000_000_000)
        )
        self.get_logger().info(
            f"bridging {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}; PX4 FRD accel/gyro -> ROS FLU"
        )

    def callback(self, msg: SensorCombined):
        now = self.get_clock().now()
        if self.use_ros_time and now.nanoseconds <= 0:
            self.zero_time_drops += 1
            if self.zero_time_drops <= 5 or self.zero_time_drops % 200 == 0:
                self.get_logger().warn(
                    f"waiting for non-zero /clock before publishing IMU; "
                    f"dropped={self.zero_time_drops}"
                )
            return

        out = Imu()
        out.header.frame_id = self.frame_id
        if self.use_ros_time:
            px4_stamp_ns = int(msg.timestamp) * 1000
            first_mapping = self.clock_mapper.offset_ns is None
            (
                stamp_ns,
                rebased,
                source_dt_ns,
                receipt_dt_ns,
                old_skew_ns,
            ) = self.clock_mapper.map(px4_stamp_ns, now.nanoseconds)
            if first_mapping:
                # PX4 SITL and Gazebo /clock can use different epochs. Keep a
                # constant offset between them, but preserve each PX4 sample's
                # original spacing. Receipt-time stamps collapse queued DDS
                # samples to 1 ns apart and make VIO inertial integration blow
                # up as soon as the vehicle moves.
                self.get_logger().info(
                    "initialized PX4->ROS IMU clock mapping: "
                    f"px4_minus_ros={self.clock_mapper.offset_ns}ns"
                )
            if rebased:
                self.get_logger().warn(
                    "rebased PX4->ROS IMU clock after source discontinuity: "
                    f"count={self.clock_mapper.rebase_count} "
                    f"source_dt_ns={source_dt_ns} receipt_dt_ns={receipt_dt_ns} "
                    f"old_skew_ns={old_skew_ns} "
                    f"new_px4_minus_ros={self.clock_mapper.offset_ns}ns"
                )
        else:
            stamp_ns = int(msg.timestamp) * 1000

        # Basalt requires strictly increasing IMU timestamps. Under ROS sim time,
        # back-to-back callbacks can occasionally observe the same clock tick, so
        # force monotonic progression by 1 ns when needed.
        if not self.use_ros_time:
            if stamp_ns <= self.clock_mapper.last_output_ns:
                stamp_ns = self.clock_mapper.last_output_ns + 1
            self.clock_mapper.last_output_ns = stamp_ns
        out.header.stamp.sec = stamp_ns // 1_000_000_000
        out.header.stamp.nanosec = stamp_ns % 1_000_000_000

        out.orientation_covariance[0] = -1.0
        for i in range(3):
            out.angular_velocity_covariance[i * 3 + i] = float(self.gyro_variance[i])
            out.linear_acceleration_covariance[i * 3 + i] = float(self.accel_variance[i])

        # PX4 body IMU convention is FRD. Basalt's ROS-topic path and the
        # x500_stereo_sim_calib camera extrinsics are in ROS FLU.
        gyro_frd = msg.gyro_rad
        accel_frd = msg.accelerometer_m_s2
        out.angular_velocity.x = float(gyro_frd[0])
        out.angular_velocity.y = -float(gyro_frd[1])
        out.angular_velocity.z = -float(gyro_frd[2])
        out.linear_acceleration.x = float(accel_frd[0])
        out.linear_acceleration.y = -float(accel_frd[1])
        out.linear_acceleration.z = -float(accel_frd[2])

        self.pub.publish(out)
        self.count += 1
        if self.count <= 5 or self.count % 500 == 0:
            px4_stamp_ns = int(msg.timestamp) * 1000
            if not self.use_ros_time:
                source_dt_ns = 0
            norm = math.sqrt(
                out.linear_acceleration.x**2
                + out.linear_acceleration.y**2
                + out.linear_acceleration.z**2
            )
            self.get_logger().info(
                "published imu "
                f"#{self.count} stamp={out.header.stamp.sec}.{out.header.stamp.nanosec:09d} "
                f"source_dt_ns={source_dt_ns} receipt_age_ns={now.nanoseconds - stamp_ns} "
                f"acc=[{out.linear_acceleration.x:.3f} "
                f"{out.linear_acceleration.y:.3f} {out.linear_acceleration.z:.3f}] "
                f"|acc|={norm:.3f}"
            )


def main():
    rclpy.init()
    node = Px4SensorCombinedToImu()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
