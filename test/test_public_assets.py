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


"""Release-facing tests for checked-in launch and configuration assets."""

import importlib.util
import json
import subprocess
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_json(path: Path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def test_example_calibrations_have_consistent_stereo_layout():
    calibration_paths = sorted((PACKAGE_ROOT / "config").glob("*_calib.json"))
    assert calibration_paths

    for path in calibration_paths:
        calibration = _load_json(path)["value0"]
        transforms = calibration["T_imu_cam"]
        intrinsics = calibration["intrinsics"]
        resolutions = calibration["resolution"]

        assert len(transforms) == len(intrinsics) == len(resolutions) == 2
        assert transforms[0] != transforms[1]
        assert calibration["imu_update_rate"] > 0.0
        for width, height in resolutions:
            assert width > 0 and height > 0
        for model in intrinsics:
            values = model["intrinsics"]
            assert values["fx"] > 0.0 and values["fy"] > 0.0


def test_example_vio_configs_have_bounded_estimator_history():
    config_paths = sorted((PACKAGE_ROOT / "config").glob("*_config.json"))
    assert config_paths

    for path in config_paths:
        config = _load_json(path)["value0"]
        assert config["config.vio_max_states"] > 0
        assert config["config.vio_max_kfs"] >= config["config.vio_max_states"]
        assert isinstance(config["config.vio_use_lm"], bool)


def test_launch_description_exposes_core_sensor_contract():
    from launch.actions import DeclareLaunchArgument

    launch_path = PACKAGE_ROOT / "launch" / "basalt_node.launch.py"
    spec = importlib.util.spec_from_file_location("basalt_wrapper_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    argument_names = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert {
        "left_image_topic",
        "right_image_topic",
        "imu_topic",
        "calib_path",
        "config_path",
        "input_mode",
        "bag_uri",
    } <= argument_names


def test_dependency_bootstrap_script_has_valid_shell_syntax():
    subprocess.run(
        ["bash", "-n", str(PACKAGE_ROOT / "scripts" / "setup_basalt.sh")],
        check=True,
    )


def test_basalt_integration_patch_is_present_and_well_formed():
    patch = PACKAGE_ROOT / "patches" / "basalt_global_position_factors.patch"
    content = patch.read_text(encoding="utf-8")
    assert content.startswith("diff --git ")
    assert "GlobalPositionMeasurement" in content
    assert "Finished VIOFilter before first IMU" in content


def test_public_files_do_not_contain_developer_home_paths():
    roots = [
        PACKAGE_ROOT / ".github",
        PACKAGE_ROOT / "config",
        PACKAGE_ROOT / "docs",
        PACKAGE_ROOT / "include",
        PACKAGE_ROOT / "launch",
        PACKAGE_ROOT / "params",
        PACKAGE_ROOT / "scripts",
        PACKAGE_ROOT / "src",
    ]
    files = [PACKAGE_ROOT / "CMakeLists.txt", PACKAGE_ROOT / "README.md"]
    for root in roots:
        files.extend(path for path in root.rglob("*") if path.is_file())

    for path in files:
        if "__pycache__" in path.parts or path.suffix == ".pyc":
            continue
        assert "/home/caio" not in path.read_text(
            encoding="utf-8", errors="ignore"
        ), path
