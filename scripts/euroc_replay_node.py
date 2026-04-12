#!/usr/bin/env python3

from __future__ import annotations

import csv
import heapq
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
from cv_bridge import CvBridge
from rclpy.clock import ClockType
from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu


@dataclass(order=True)
class ReplayEvent:
    stamp_ns: int
    priority: int
    kind: str
    payload: Any


class EurocReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("euroc_replay_node")

        self.declare_parameter("dataset_path", "")
        self.declare_parameter("left_image_topic", "/cam0/image_raw")
        self.declare_parameter("right_image_topic", "/cam1/image_raw")
        self.declare_parameter("imu_topic", "/imu0")
        self.declare_parameter("clock_topic", "/clock")
        self.declare_parameter("publish_clock", True)
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("frame_id_left", "cam0")
        self.declare_parameter("frame_id_right", "cam1")
        self.declare_parameter("frame_id_imu", "imu0")

        dataset_path_value = (
            self.get_parameter("dataset_path").get_parameter_value().string_value.strip()
        )
        if not dataset_path_value:
            raise RuntimeError("dataset_path parameter is required")
        dataset_path = Path(dataset_path_value).expanduser()

        self.mav0_path = self._resolve_mav0_path(dataset_path)
        self.speed = self.get_parameter("speed").get_parameter_value().double_value
        if self.speed <= 0.0:
            raise RuntimeError("speed must be > 0")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        left_topic = self.get_parameter("left_image_topic").value
        right_topic = self.get_parameter("right_image_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        clock_topic = self.get_parameter("clock_topic").value

        self.left_frame_id = self.get_parameter("frame_id_left").value
        self.right_frame_id = self.get_parameter("frame_id_right").value
        self.imu_frame_id = self.get_parameter("frame_id_imu").value

        self.left_pub = self.create_publisher(Image, left_topic, sensor_qos)
        self.right_pub = self.create_publisher(Image, right_topic, sensor_qos)
        self.imu_pub = self.create_publisher(Imu, imu_topic, sensor_qos)

        self.publish_clock = self.get_parameter("publish_clock").value
        self.clock_pub = None
        if self.publish_clock:
            self.clock_pub = self.create_publisher(Clock, clock_topic, 10)

        self.bridge = CvBridge()
        self.events = self._load_events()
        if not self.events:
            raise RuntimeError(f"No replayable events found under {self.mav0_path}")

        self.first_stamp_ns = self.events[0].stamp_ns
        self.start_wall_time = time.monotonic()
        self.last_clock_stamp_ns = self.first_stamp_ns
        self.finished = False
        self.timer = self.create_timer(0.001, self._pump_events)

        self.get_logger().info(
            f"Replaying EuRoC dataset from {self.mav0_path} with "
            f"{len(self.events)} events at {self.speed:.3f}x speed"
        )

    def _resolve_mav0_path(self, dataset_path: Path) -> Path:
        if dataset_path.name == "mav0":
            mav0_path = dataset_path
        elif (dataset_path / "mav0").is_dir():
            mav0_path = dataset_path / "mav0"
        else:
            raise RuntimeError(
                f"dataset_path must point to a EuRoC sequence root or mav0 folder; got {dataset_path}"
            )

        required = ["cam0", "cam1", "imu0"]
        missing = [name for name in required if not (mav0_path / name).exists()]
        if missing:
            raise RuntimeError(
                f"Dataset at {mav0_path} is missing required entries: {', '.join(missing)}"
            )
        return mav0_path

    def _load_events(self) -> deque[ReplayEvent]:
        events: list[ReplayEvent] = []
        events.extend(self._load_camera_events("cam0", priority=1))
        events.extend(self._load_camera_events("cam1", priority=2))
        events.extend(self._load_imu_events(priority=0))
        heapq.heapify(events)
        return deque(heapq.heappop(events) for _ in range(len(events)))

    def _load_camera_events(self, camera_name: str, priority: int) -> list[ReplayEvent]:
        camera_dir = self.mav0_path / camera_name
        csv_path = camera_dir / "data.csv"
        image_dir = camera_dir / "data"
        events: list[ReplayEvent] = []

        with csv_path.open("r", newline="") as handle:
            reader = csv.reader(handle)
            for row in reader:
                if not row or row[0].startswith("#"):
                    continue
                stamp_ns = int(row[0].strip())
                image_path = image_dir / row[1].strip()
                events.append(ReplayEvent(stamp_ns, priority, camera_name, image_path))

        return events

    def _load_imu_events(self, priority: int) -> list[ReplayEvent]:
        csv_path = self.mav0_path / "imu0" / "data.csv"
        events: list[ReplayEvent] = []

        with csv_path.open("r", newline="") as handle:
            reader = csv.reader(handle)
            for row in reader:
                if not row or row[0].startswith("#"):
                    continue
                stamp_ns = int(row[0].strip())
                omega = [float(row[1]), float(row[2]), float(row[3])]
                accel = [float(row[4]), float(row[5]), float(row[6])]
                events.append(ReplayEvent(stamp_ns, priority, "imu0", (omega, accel)))

        return events

    def _pump_events(self) -> None:
        if self.finished:
            return

        elapsed_ns = int((time.monotonic() - self.start_wall_time) * self.speed * 1e9)
        target_stamp_ns = self.first_stamp_ns + elapsed_ns

        while self.events and self.events[0].stamp_ns <= target_stamp_ns:
            event = self.events.popleft()
            self.last_clock_stamp_ns = event.stamp_ns
            if event.kind == "cam0":
                self._publish_image(event.payload, self.left_frame_id, self.left_pub, event.stamp_ns)
            elif event.kind == "cam1":
                self._publish_image(event.payload, self.right_frame_id, self.right_pub, event.stamp_ns)
            else:
                self._publish_imu(event.payload, event.stamp_ns)

        if self.clock_pub is not None:
            clock_msg = Clock()
            clock_msg.clock = self._to_ros_stamp(self.last_clock_stamp_ns)
            self.clock_pub.publish(clock_msg)

        if not self.events:
            self.get_logger().info("Finished replaying EuRoC dataset")
            self.finished = True
            self.timer.cancel()
            self.create_timer(0.1, self._shutdown)

    def _publish_image(self, image_path: Path, frame_id: str, publisher, stamp_ns: int) -> None:
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"Failed to load image {image_path}")

        msg = self.bridge.cv2_to_imgmsg(image, encoding="mono8")
        msg.header.frame_id = frame_id
        msg.header.stamp = self._to_ros_stamp(stamp_ns)
        publisher.publish(msg)

    def _publish_imu(self, imu_data: tuple[list[float], list[float]], stamp_ns: int) -> None:
        omega, accel = imu_data
        msg = Imu()
        msg.header.frame_id = self.imu_frame_id
        msg.header.stamp = self._to_ros_stamp(stamp_ns)
        msg.angular_velocity.x = omega[0]
        msg.angular_velocity.y = omega[1]
        msg.angular_velocity.z = omega[2]
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        self.imu_pub.publish(msg)

    def _to_ros_stamp(self, stamp_ns: int):
        return rclpy.time.Time(nanoseconds=stamp_ns, clock_type=ClockType.ROS_TIME).to_msg()

    def _shutdown(self) -> None:
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = EurocReplayNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
