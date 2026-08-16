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


"""Use raw VIO for VTOL transition, then smoothly add geographic correction."""

from __future__ import annotations

import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VtolVehicleStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class VtolExternalVisionSelector(Node):
    """
    Prevent discrete map fixes from disturbing takeoff/front transition.

    PX4 receives raw stereo VIO until it confirms fixed-wing mode. Geographic
    position correction is then blended in with a rate limit and remains on
    through the back-transition and landing, avoiding a second frame switch.
    """

    def __init__(self) -> None:
        super().__init__("vtol_ev_selector")
        self.declare_parameter("raw_topic", "/basalt/odometry")
        self.declare_parameter("corrected_topic", "/geo/corrected_odometry")
        self.declare_parameter("output_topic", "/vtol/selected_odometry")
        self.declare_parameter("blend_time_constant_s", 3.0)
        self.declare_parameter("max_correction_slew_mps", 3.0)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("raw_topic").value),
            self._raw_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("corrected_topic").value),
            self._corrected_callback,
            10,
        )
        self.create_subscription(
            VtolVehicleStatus,
            "/fmu/out/vtol_vehicle_status",
            self._vtol_callback,
            px4_qos,
        )

        self._corrected: Odometry | None = None
        self._fixed_wing_seen = False
        self._blend = [0.0, 0.0, 0.0]
        self._last_stamp_s: float | None = None
        self._last_state: int | None = None
        self.get_logger().info(
            "EV policy: raw Basalt through front transition; slew-limited VPC after FW"
        )

    def _corrected_callback(self, msg: Odometry) -> None:
        self._corrected = msg

    def _vtol_callback(self, msg: VtolVehicleStatus) -> None:
        state = int(msg.vehicle_vtol_state)
        if state != self._last_state:
            self.get_logger().info(f"VTOL state observed: {state}")
            self._last_state = state
        if state == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW:
            if not self._fixed_wing_seen:
                self.get_logger().info(
                    "Fixed-wing confirmed: enabling smooth geographic correction"
                )
            self._fixed_wing_seen = True

    @staticmethod
    def _stamp_seconds(msg: Odometry) -> float:
        return float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)

    def _raw_callback(self, raw: Odometry) -> None:
        output = copy.deepcopy(raw)
        stamp_s = self._stamp_seconds(raw)
        dt = 0.0 if self._last_stamp_s is None else stamp_s - self._last_stamp_s
        self._last_stamp_s = stamp_s

        corrected = self._corrected
        if self._fixed_wing_seen and corrected is not None and 0.0 < dt < 0.5:
            raw_position = raw.pose.pose.position
            corrected_position = corrected.pose.pose.position
            target = (
                corrected_position.x - raw_position.x,
                corrected_position.y - raw_position.y,
                corrected_position.z - raw_position.z,
            )
            time_constant = max(
                0.1, float(self.get_parameter("blend_time_constant_s").value)
            )
            alpha = 1.0 - math.exp(-dt / time_constant)
            max_step = max(
                0.01,
                float(self.get_parameter("max_correction_slew_mps").value) * dt,
            )
            for axis in range(3):
                requested_step = alpha * (target[axis] - self._blend[axis])
                requested_step = max(-max_step, min(max_step, requested_step))
                self._blend[axis] += requested_step

        output.pose.pose.position.x += self._blend[0]
        output.pose.pose.position.y += self._blend[1]
        output.pose.pose.position.z += self._blend[2]
        self._publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = VtolExternalVisionSelector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
