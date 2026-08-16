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

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import rclpy
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
from sensor_msgs.msg import CameraInfo, Image, Imu
import yaml


@dataclass(order=True)
class Event:
    stamp_ns: int
    priority: int
    topic: str
    msg_type: str
    payload: object


def ns_to_stamp(stamp_ns: int):
    sec = stamp_ns // 1_000_000_000
    nanosec = stamp_ns % 1_000_000_000
    return sec, nanosec


def set_header(msg, stamp_ns: int, frame_id: str) -> None:
    sec, nanosec = ns_to_stamp(stamp_ns)
    msg.header.stamp.sec = int(sec)
    msg.header.stamp.nanosec = int(nanosec)
    msg.header.frame_id = frame_id


def resolve_mav0_path(dataset_path: Path) -> Path:
    candidates: list[Path] = []
    if dataset_path.name == "mav0":
        candidates.append(dataset_path)
    if (dataset_path / "mav0").is_dir():
        candidates.append(dataset_path / "mav0")

    # Some local EuRoC copies are wrapped in an extra sequence directory, e.g.
    # <root>/V1_02_medium/V1_02_medium/mav0.
    for child in dataset_path.iterdir() if dataset_path.is_dir() else []:
        nested_mav0 = child / "mav0"
        if child.is_dir() and nested_mav0.is_dir():
            candidates.append(nested_mav0)

    seen: set[Path] = set()
    unique_candidates = []
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved not in seen:
            seen.add(resolved)
            unique_candidates.append(candidate)

    if not unique_candidates:
        raise RuntimeError(
            f"dataset_path must point to a EuRoC sequence root or mav0 folder; got {dataset_path}"
        )

    mav0 = unique_candidates[0]

    for required in ("cam0", "cam1", "imu0"):
        if not (mav0 / required).exists():
            raise RuntimeError(f"missing {required} under {mav0}")
    return mav0


def read_csv_rows(csv_path: Path) -> Iterable[list[str]]:
    with csv_path.open("r", newline="") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            yield [cell.strip() for cell in row]


def load_camera_info(sensor_yaml_path: Path, frame_id: str) -> CameraInfo:
    data = yaml.safe_load(sensor_yaml_path.read_text())

    resolution = data["resolution"]
    intrinsics = data["intrinsics"]
    distortion_coeffs = data.get("distortion_coefficients", [0.0, 0.0, 0.0, 0.0])
    distortion_model = data.get("distortion_model", "radtan")

    msg = CameraInfo()
    msg.width = int(resolution[0])
    msg.height = int(resolution[1])
    msg.distortion_model = (
        "plumb_bob" if distortion_model in ("radtan", "pinhole") else distortion_model
    )
    msg.d = [float(x) for x in distortion_coeffs]

    fx, fy, cx, cy = [float(x) for x in intrinsics[:4]]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.binning_x = 0
    msg.binning_y = 0
    set_header(msg, 0, frame_id)
    return msg


def make_image_msg(image_path: Path, stamp_ns: int, frame_id: str) -> Image:
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"failed to load image {image_path}")

    msg = Image()
    set_header(msg, stamp_ns, frame_id)
    msg.height = int(image.shape[0])
    msg.width = int(image.shape[1])
    msg.encoding = "mono8"
    msg.is_bigendian = 0
    msg.step = int(image.shape[1])
    msg.data = image.tobytes()
    return msg


def make_camera_info_msg(base_info: CameraInfo, stamp_ns: int, frame_id: str) -> CameraInfo:
    msg = CameraInfo()
    msg.height = base_info.height
    msg.width = base_info.width
    msg.distortion_model = base_info.distortion_model
    msg.d = list(base_info.d)
    msg.k = list(base_info.k)
    msg.r = list(base_info.r)
    msg.p = list(base_info.p)
    msg.binning_x = base_info.binning_x
    msg.binning_y = base_info.binning_y
    msg.roi = base_info.roi
    set_header(msg, stamp_ns, frame_id)
    return msg


def make_imu_msg(row: list[str], frame_id: str) -> tuple[int, Imu]:
    stamp_ns = int(row[0])
    msg = Imu()
    set_header(msg, stamp_ns, frame_id)
    msg.angular_velocity.x = float(row[1])
    msg.angular_velocity.y = float(row[2])
    msg.angular_velocity.z = float(row[3])
    msg.linear_acceleration.x = float(row[4])
    msg.linear_acceleration.y = float(row[5])
    msg.linear_acceleration.z = float(row[6])
    return stamp_ns, msg


def build_events(mav0_path: Path) -> list[Event]:
    cam0_info = load_camera_info(mav0_path / "cam0" / "sensor.yaml", "cam0")
    cam1_info = load_camera_info(mav0_path / "cam1" / "sensor.yaml", "cam1")

    events: list[Event] = []

    for row in read_csv_rows(mav0_path / "cam0" / "data.csv"):
        stamp_ns = int(row[0])
        image_path = mav0_path / "cam0" / "data" / row[1]
        events.append(
            Event(
                stamp_ns,
                1,
                "/cam0/camera_info",
                "sensor_msgs/msg/CameraInfo",
                make_camera_info_msg(cam0_info, stamp_ns, "cam0"),
            )
        )
        events.append(
            Event(
                stamp_ns,
                2,
                "/cam0/image_raw",
                "sensor_msgs/msg/Image",
                make_image_msg(image_path, stamp_ns, "cam0"),
            )
        )

    for row in read_csv_rows(mav0_path / "cam1" / "data.csv"):
        stamp_ns = int(row[0])
        image_path = mav0_path / "cam1" / "data" / row[1]
        events.append(
            Event(
                stamp_ns,
                3,
                "/cam1/camera_info",
                "sensor_msgs/msg/CameraInfo",
                make_camera_info_msg(cam1_info, stamp_ns, "cam1"),
            )
        )
        events.append(
            Event(
                stamp_ns,
                4,
                "/cam1/image_raw",
                "sensor_msgs/msg/Image",
                make_image_msg(image_path, stamp_ns, "cam1"),
            )
        )

    for row in read_csv_rows(mav0_path / "imu0" / "data.csv"):
        stamp_ns, imu_msg = make_imu_msg(row, "imu0")
        events.append(Event(stamp_ns, 0, "/imu0", "sensor_msgs/msg/Imu", imu_msg))

    events.sort()
    return events


def write_bag(events: list[Event], output_uri: Path, storage_id: str) -> None:
    if output_uri.exists():
        raise RuntimeError(
            f"output_uri already exists: {output_uri}. Remove it first or choose a new bag path."
        )
    output_uri.parent.mkdir(parents=True, exist_ok=True)

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(output_uri), storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    created_topics: set[str] = set()
    for event in events:
        if event.topic not in created_topics:
            writer.create_topic(make_topic_metadata(event.topic, event.msg_type))
            created_topics.add(event.topic)

    for event in events:
        writer.write(event.topic, serialize_message(event.payload), event.stamp_ns)


def make_topic_metadata(topic_name: str, msg_type: str) -> TopicMetadata:
    try:
        return TopicMetadata(
            name=topic_name,
            type=msg_type,
            serialization_format="cdr",
            offered_qos_profiles="",
        )
    except TypeError:
        try:
            return TopicMetadata(
                name=topic_name,
                type=msg_type,
                serialization_format="cdr",
            )
        except TypeError:
            return TopicMetadata(topic_name, msg_type, "cdr", "")


def main() -> None:
    rclpy.init(args=None)
    node = rclpy.create_node("euroc_to_rosbag2")
    node.declare_parameter("dataset_path", "")
    node.declare_parameter("output_uri", "")
    node.declare_parameter("storage_id", "sqlite3")

    dataset_path_value = node.get_parameter("dataset_path").value.strip()
    output_uri_value = node.get_parameter("output_uri").value.strip()
    storage_id = node.get_parameter("storage_id").value.strip()

    if not dataset_path_value:
        raise RuntimeError("dataset_path parameter is required")
    if not output_uri_value:
        raise RuntimeError("output_uri parameter is required")

    dataset_path = Path(dataset_path_value).expanduser()
    output_uri = Path(output_uri_value).expanduser()
    mav0_path = resolve_mav0_path(dataset_path)

    node.get_logger().info(f"Building offline rosbag2 from {mav0_path}")
    events = build_events(mav0_path)
    node.get_logger().info(f"Writing {len(events)} timestamp-sorted messages to {output_uri}")
    write_bag(events, output_uri, storage_id)
    node.get_logger().info("Finished writing rosbag2")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
