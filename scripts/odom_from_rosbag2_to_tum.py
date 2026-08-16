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

import argparse
from pathlib import Path

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Extract a nav_msgs/Odometry topic from a rosbag2 bag as TUM."
    )
    parser.add_argument("bag_uri", type=Path)
    parser.add_argument("topic")
    parser.add_argument("output_tum", type=Path)
    args = parser.parse_args()

    storage_options = rosbag2_py.StorageOptions(
        uri=str(args.bag_uri), storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if args.topic not in topic_types:
        available = "\n  ".join(sorted(topic_types))
        raise RuntimeError(
            f"topic {args.topic!r} not found in {args.bag_uri}. Available:\n  {available}"
        )

    msg_type = get_message(topic_types[args.topic])
    if msg_type is not Odometry:
        raise RuntimeError(
            f"topic {args.topic!r} has type {topic_types[args.topic]!r}, expected nav_msgs/msg/Odometry"
        )

    rows = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(data, msg_type)
        pose = msg.pose.pose
        rows.append(
            (
                stamp_to_seconds(msg.header.stamp),
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )

    if not rows:
        raise RuntimeError(f"no odometry samples found for {args.topic!r}")

    args.output_tum.parent.mkdir(parents=True, exist_ok=True)
    with args.output_tum.open("w", encoding="utf-8") as handle:
        handle.write("# timestamp tx ty tz qx qy qz qw\n")
        for row in rows:
            handle.write(
                f"{row[0]:.9f} {row[1]:.9f} {row[2]:.9f} {row[3]:.9f} "
                f"{row[4]:.9f} {row[5]:.9f} {row[6]:.9f} {row[7]:.9f}\n"
            )

    print(f"wrote {len(rows)} odometry samples to {args.output_tum}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
