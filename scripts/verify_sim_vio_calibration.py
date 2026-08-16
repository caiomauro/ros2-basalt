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


"""Fail fast when Gazebo stereo/IMU geometry and Basalt calibration diverge."""

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def pose_values(element):
    if element is None or not element.text:
        raise ValueError("missing SDF pose")
    values = [float(value) for value in element.text.split()]
    if len(values) < 3:
        raise ValueError(f"invalid SDF pose: {element.text!r}")
    return values + [0.0] * (6 - len(values))


def pose_xyz(element):
    values = pose_values(element)
    return values[:3]


def vector_add(a, b):
    return [a[index] + b[index] for index in range(3)]


def quat_multiply(a, b):
    """Hamilton product for quaternions in xyzw order."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def rpy_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def rotation_error_deg(actual, expected):
    an = math.sqrt(sum(value * value for value in actual))
    en = math.sqrt(sum(value * value for value in expected))
    if an <= 1e-12 or en <= 1e-12:
        return math.inf
    dot = abs(sum(a * e for a, e in zip(actual, expected)) / (an * en))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def close(label, actual, expected, *, absolute=2e-6, relative=0.01):
    tolerance = max(absolute, relative * max(abs(expected), 1e-12))
    ok = abs(actual - expected) <= tolerance
    print(
        f"  {'OK' if ok else 'FAIL'} {label}: calibration={actual:.10g} "
        f"model={expected:.10g} tolerance={tolerance:.3g}"
    )
    return ok


def sensor_noise(imu_sensor, group, axis):
    noise = imu_sensor.find(f"./imu/{group}/{axis}/noise")
    if noise is None:
        raise ValueError(f"missing {group}/{axis}/noise in IMU SDF")
    discrete_std = float(noise.findtext("stddev"))
    dynamic_std_text = noise.findtext("dynamic_bias_stddev")
    correlation_text = noise.findtext("dynamic_bias_correlation_time")
    if dynamic_std_text is None or correlation_text is None:
        bias_density = 0.0
    else:
        bias_density = math.sqrt(2.0) * float(dynamic_std_text) / math.sqrt(
            float(correlation_text)
        )
    return discrete_std, bias_density


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calib", required=True, type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--camera-model", required=True, type=Path)
    parser.add_argument("--imu-model", required=True, type=Path)
    args = parser.parse_args()

    calibration = json.loads(args.calib.read_text())["value0"]
    model_root = ET.parse(args.model).getroot()
    camera_root = ET.parse(args.camera_model).getroot()
    imu_root = ET.parse(args.imu_model).getroot()

    direct_camera_links = {
        name: model_root.find(f".//link[@name='{name}']")
        for name in ("left_camera_link", "right_camera_link")
    }
    cameras_are_direct = all(link is not None for link in direct_camera_links.values())
    if cameras_are_direct:
        camera_definition_root = model_root
        include_xyz = [0.0, 0.0, 0.0]
    else:
        include = next(
            (
                node
                for node in model_root.findall(".//include")
                if "stereo_cam" in (node.findtext("uri") or "")
            ),
            None,
        )
        if include is None:
            raise ValueError("parent model has neither direct stereo links nor stereo_cam include")
        include_xyz = pose_xyz(include.find("pose"))
        camera_definition_root = camera_root

    link_positions = {}
    camera_positions = {}
    camera_sensor_poses = {}
    camera_specs = {}
    for link_name in ("left_camera_link", "right_camera_link"):
        link = camera_definition_root.find(f".//link[@name='{link_name}']")
        if link is None:
            raise ValueError(f"camera model has no {link_name}")
        link_positions[link_name] = vector_add(
            include_xyz, pose_xyz(link.find("pose"))
        )
        sensor = link.find("./sensor[@type='camera']")
        if sensor is None:
            raise ValueError(f"camera model has no camera sensor on {link_name}")
        camera_sensor_poses[link_name] = pose_values(sensor.find("pose"))
        camera_positions[link_name] = vector_add(
            link_positions[link_name], camera_sensor_poses[link_name][:3]
        )
        image = sensor.find("./camera/image")
        if image is None:
            raise ValueError(f"camera sensor on {link_name} has no image definition")
        camera_specs[link_name] = {
            "width": int(image.findtext("width")),
            "height": int(image.findtext("height")),
            "hfov": float(sensor.findtext("./camera/horizontal_fov")),
            "rate": float(sensor.findtext("update_rate")),
        }

    ok = True
    print("Stereo extrinsics")
    for index, link_name in enumerate(("left_camera_link", "right_camera_link")):
        expected = camera_positions[link_name]
        calibrated = calibration["T_imu_cam"][index]
        for axis_index, axis in enumerate(("x", "y", "z")):
            ok &= close(
                f"cam{index}.{axis}", calibrated[f"p{axis}"], expected[axis_index]
            )

    joints = {
        joint.get("name"): pose_values(joint.find("pose"))
        for joint in model_root.findall(".//joint")
        if joint.get("name") in ("LeftCameraJoint", "RightCameraJoint")
    }
    for joint_name, link_name in (
        ("LeftCameraJoint", "left_camera_link"),
        ("RightCameraJoint", "right_camera_link"),
    ):
        if joint_name not in joints:
            print(f"  FAIL missing {joint_name}")
            ok = False
            continue
        for axis_index, axis in enumerate(("x", "y", "z")):
            ok &= close(
                f"{joint_name}.{axis}",
                joints[joint_name][axis_index],
                link_positions[link_name][axis_index],
            )

        # Gazebo's camera looks along link +X, while Basalt's pinhole camera
        # looks along +Z with +X right and +Y down.  Compose the joint's body
        # rotation with that fixed optical-frame conversion and verify the
        # complete T_imu_cam rotation.  Translation-only checks previously let
        # a bad pitch change pass silently.
        joint_pose = joints[joint_name]
        body_from_link = rpy_quaternion(*joint_pose[3:6])
        link_from_sensor = rpy_quaternion(*camera_sensor_poses[link_name][3:6])
        link_from_optical = [0.5, -0.5, 0.5, -0.5]
        expected_q = quat_multiply(
            quat_multiply(body_from_link, link_from_sensor), link_from_optical
        )
        index = 0 if joint_name == "LeftCameraJoint" else 1
        calibrated = calibration["T_imu_cam"][index]
        actual_q = [
            float(calibrated["qx"]),
            float(calibrated["qy"]),
            float(calibrated["qz"]),
            float(calibrated["qw"]),
        ]
        angle_error = rotation_error_deg(actual_q, expected_q)
        rotation_ok = angle_error <= 0.05
        print(
            f"  {'OK' if rotation_ok else 'FAIL'} {joint_name}.rotation: "
            f"angle_error={angle_error:.6f}deg tolerance=0.05deg"
        )
        ok &= rotation_ok

    baseline = math.dist(
        camera_positions["left_camera_link"], camera_positions["right_camera_link"]
    )
    fx = float(calibration["intrinsics"][0]["intrinsics"]["fx"])
    print(f"  baseline={baseline:.3f} m, disparity@80m={fx * baseline / 80:.3f} px, "
          f"disparity@100m={fx * baseline / 100:.3f} px")

    print("Stereo imaging")
    for index, link_name in enumerate(("left_camera_link", "right_camera_link")):
        spec = camera_specs[link_name]
        calibrated_resolution = calibration["resolution"][index]
        intrinsics = calibration["intrinsics"][index]["intrinsics"]
        expected_fx = spec["width"] / (2.0 * math.tan(spec["hfov"] / 2.0))
        ok &= close(
            f"cam{index}.width",
            float(calibrated_resolution[0]),
            float(spec["width"]),
            absolute=0.0,
            relative=0.0,
        )
        ok &= close(
            f"cam{index}.height",
            float(calibrated_resolution[1]),
            float(spec["height"]),
            absolute=0.0,
            relative=0.0,
        )
        ok &= close(f"cam{index}.fx_from_hfov", float(intrinsics["fx"]), expected_fx)
        ok &= close(f"cam{index}.fy_from_hfov", float(intrinsics["fy"]), expected_fx)
        ok &= close(
            f"cam{index}.cx",
            float(intrinsics["cx"]),
            0.5 * spec["width"],
        )
        ok &= close(
            f"cam{index}.cy",
            float(intrinsics["cy"]),
            0.5 * spec["height"],
        )
        print(
            f"  cam{index}: {spec['width']}x{spec['height']} @ {spec['rate']:.1f} Hz, "
            f"HFOV={math.degrees(spec['hfov']):.3f} deg"
        )

    if cameras_are_direct:
        # Gazebo may skip render deadlines in the textured world, so several
        # real exposures are offered per 20 Hz estimator slot. Each eye has an
        # independent transport worker and pairing remains timestamp-exact.
        hardware_checks = (
            ("hardware_baseline", baseline, 0.075, 1e-6),
            ("hardware_hfov_deg", math.degrees(camera_specs["left_camera_link"]["hfov"]), 86.0, 1e-6),
            ("hardware_width_per_eye", float(camera_specs["left_camera_link"]["width"]), 1280.0, 0.0),
            ("hardware_height_per_eye", float(camera_specs["left_camera_link"]["height"]), 800.0, 0.0),
            ("navigation_camera_source_rate", camera_specs["left_camera_link"]["rate"], 120.0, 0.0),
        )
        for label, actual, expected, tolerance in hardware_checks:
            check_ok = abs(actual - expected) <= tolerance
            print(
                f"  {'OK' if check_ok else 'FAIL'} {label}: "
                f"actual={actual:.10g} required={expected:.10g}"
            )
            ok &= check_ok
        right = camera_specs["right_camera_link"]
        left = camera_specs["left_camera_link"]
        stereo_spec_ok = right == left
        print(
            f"  {'OK' if stereo_spec_ok else 'FAIL'} synchronized_camera_specs: "
            f"left={left} right={right}"
        )
        ok &= stereo_spec_ok

    sensor = imu_root.find(".//sensor[@name='imu_sensor']")
    if sensor is None:
        raise ValueError("IMU model has no imu_sensor")
    update_rate = float(sensor.findtext("update_rate"))
    ok &= close("imu_update_rate", float(calibration["imu_update_rate"]), update_rate)

    print("IMU continuous-time noise densities")
    for group, calibration_key, bias_key in (
        ("linear_acceleration", "accel_noise_std", "accel_bias_std"),
        ("angular_velocity", "gyro_noise_std", "gyro_bias_std"),
    ):
        for index, axis in enumerate(("x", "y", "z")):
            discrete_std, bias_density = sensor_noise(sensor, group, axis)
            noise_density = discrete_std / math.sqrt(update_rate)
            calibrated_noise = float(calibration[calibration_key][index])
            # Basalt consumes PX4's filtered SensorCombined stream, not the
            # pristine Gazebo sensor sample. The effective estimator noise may
            # be deliberately more conservative than the physical density,
            # but it must never be smaller (overconfident).
            upper = 0.02 if group == "linear_acceleration" else 0.002
            noise_ok = noise_density <= calibrated_noise <= upper
            print(
                f"  {'OK' if noise_ok else 'FAIL'} {calibration_key}.{axis}: "
                f"calibration={calibrated_noise:.10g} physical_floor={noise_density:.10g} "
                f"estimator_ceiling={upper:.10g}"
            )
            ok &= noise_ok
            if bias_density == 0.0:
                # A simulator with no explicit Gauss-Markov bias still needs a
                # bounded estimator process-noise floor so small gravity,
                # timing, and mounting residuals remain observable instead of
                # being integrated into position drift.  Cap that floor at
                # Basalt's published EuRoC/TUM-VI defaults.
                estimator_floor_max = 0.001 if group == "linear_acceleration" else 0.0001
                calibrated_bias = float(calibration[bias_key][index])
                bias_ok = 0.0 < calibrated_bias <= estimator_floor_max
                print(
                    f"  {'OK' if bias_ok else 'FAIL'} {bias_key}.{axis}: "
                    f"calibration={calibrated_bias:.10g}, model=none, "
                    f"estimator_floor_max={estimator_floor_max:.10g}"
                )
                ok &= bias_ok
            else:
                ok &= close(
                    f"{bias_key}.{axis}",
                    float(calibration[bias_key][index]),
                    bias_density,
                )

    if not ok:
        print("Calibration verification FAILED", file=sys.stderr)
        return 1
    print("Calibration verification PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
