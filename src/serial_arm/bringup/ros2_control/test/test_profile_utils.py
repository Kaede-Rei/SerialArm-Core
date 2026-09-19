import os
from pathlib import Path
import sys

import pytest
import yaml

sys.path.append(str(Path(__file__).resolve().parents[1] / "scripts"))
import profile_utils


class CoreProfile:
    core_config_path = "/resolved/core.yaml"
    hardware_plugin = "serial_arm_hardware_fake"
    hardware_config_path = "/resolved/hardware.yaml"


def _write_profiles(path, profiles):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump({"profiles": profiles}), encoding="utf-8")


def _minimal_profile(description=None, controllers=None, moveit=None):
    profile = {
        "core": {"package": "robot_pkg", "config": "config/core.yaml"},
        "hardware": {
            "plugin": "serial_arm_hardware_fake",
            "config_package": "robot_pkg",
            "config": "config/hardware.yaml",
        },
        "description": description
        or {
            "package": "robot_pkg",
            "urdf": "model/robot.urdf",
            "ros2_control_xacro": "model/robot.ros2_control.xacro",
        },
        "controllers": controllers
        or {
            "package": "robot_pkg",
            "config": "config/ros2_controllers.yaml",
        },
    }
    if moveit is not None:
        profile["moveit"] = moveit
    return profile


def test_profile_without_moveit_loads_base_fields(monkeypatch, tmp_path):
    profiles_file = (
        tmp_path
        / "serial_arm_robot_profiles"
        / "config"
        / "robot_profiles.yaml"
    )
    _write_profiles(profiles_file, {"minimal_arm": _minimal_profile()})

    def fake_share(package):
        return str(tmp_path / package)

    def fake_prefix(package):
        return str(tmp_path / "install" / package)

    monkeypatch.setattr(profile_utils, "get_package_share_directory", fake_share)
    monkeypatch.setattr(profile_utils, "get_package_prefix", fake_prefix)
    monkeypatch.setattr(
        profile_utils,
        "load_core_profile",
        lambda robot_profile, profile_file, resource_paths: CoreProfile(),
    )

    profile = profile_utils.load_profile("minimal_arm")
    assert profile["profile_file"] == str(profiles_file)
    assert profile["core_config_path"] == "/resolved/core.yaml"
    assert profile["hardware_plugin"] == "serial_arm_hardware_fake"
    assert profile["hardware_config_path"] == "/resolved/hardware.yaml"
    assert profile["description_type"] == "urdf"
    assert profile["description_urdf_path"].endswith("robot_pkg/model/robot.urdf")
    assert profile["ros2_control_xacro_path"].endswith(
        "robot_pkg/model/robot.ros2_control.xacro"
    )
    assert profile["controllers_path"].endswith(
        "robot_pkg/config/ros2_controllers.yaml"
    )
    assert profile["controller_names"] == [
        "joint_state_broadcaster",
        "joint_trajectory_controller",
    ]
    assert profile["resource_paths"] == [
        str(tmp_path / "install" / "robot_pkg"),
        str(tmp_path / "install" / "serial_arm_hardware_fake"),
    ]
    assert "moveit_package" not in profile

    with pytest.raises(
        RuntimeError, match="Robot profile 'minimal_arm' does not define MoveIt support"
    ):
        profile_utils.require_moveit_package(profile, "minimal_arm")


def test_external_profile_file_supports_xacro_resource_paths_and_controller_spawn(
    monkeypatch, tmp_path
):
    profiles_file = tmp_path / "downstream" / "config" / "robot_profiles.yaml"
    _write_profiles(
        profiles_file,
        {
            "tomato_picker": _minimal_profile(
                description={
                    "package": "tomato_picker_description",
                    "xacro": "urdf/tomato_picker.urdf.xacro",
                    "ros2_control_xacro": "urdf/tomato_picker.ros2_control.xacro",
                },
                controllers={
                    "package": "tomato_picker_bringup",
                    "config": "config/ros2_controllers.yaml",
                    "spawn": ["joint_state_broadcaster", "tomato_arm_controller"],
                },
                moveit={"package": "tomato_picker_moveit_config"},
            )
        },
    )

    shares = tmp_path / "shares"
    prefixes = tmp_path / "prefixes"
    captured = {}

    monkeypatch.setattr(
        profile_utils,
        "get_package_share_directory",
        lambda package: str(shares / package),
    )
    monkeypatch.setattr(
        profile_utils,
        "get_package_prefix",
        lambda package: str(prefixes / package),
    )

    def fake_core(robot_profile, profile_file, resource_paths):
        captured["robot_profile"] = robot_profile
        captured["profile_file"] = profile_file
        captured["resource_paths"] = resource_paths
        return CoreProfile()

    monkeypatch.setattr(profile_utils, "load_core_profile", fake_core)

    explicit = os.pathsep.join(["/opt/custom_robot", "/srv/serial_arm_resources"])
    profile = profile_utils.load_profile(
        "tomato_picker",
        profile_file=str(profiles_file),
        resource_paths=explicit,
    )

    assert captured["robot_profile"] == "tomato_picker"
    assert captured["profile_file"] == str(profiles_file.resolve())
    assert captured["resource_paths"][:2] == [
        "/opt/custom_robot",
        "/srv/serial_arm_resources",
    ]
    assert str(prefixes / "robot_pkg") in captured["resource_paths"]
    assert str(prefixes / "serial_arm_hardware_fake") in captured["resource_paths"]
    assert str(prefixes / "tomato_picker_description") in captured["resource_paths"]
    assert str(prefixes / "tomato_picker_bringup") in captured["resource_paths"]
    assert str(prefixes / "tomato_picker_moveit_config") in captured["resource_paths"]

    assert profile["description_type"] == "xacro"
    assert profile["description_xacro_path"] == str(
        shares / "tomato_picker_description" / "urdf/tomato_picker.urdf.xacro"
    )
    assert profile["ros2_control_xacro_path"] == str(
        shares
        / "tomato_picker_description"
        / "urdf/tomato_picker.ros2_control.xacro"
    )
    assert profile["controller_names"] == [
        "joint_state_broadcaster",
        "tomato_arm_controller",
    ]
    assert profile["moveit_package"] == "tomato_picker_moveit_config"


def test_resource_paths_are_normalized_and_deduplicated():
    value = os.pathsep.join(["/opt/a", "/opt/b", "/opt/a", ""])
    assert profile_utils.normalize_resource_paths(value) == ["/opt/a", "/opt/b"]
    assert profile_utils.normalize_resource_paths(["/opt/a", "/opt/a", "/opt/c"]) == [
        "/opt/a",
        "/opt/c",
    ]


def test_external_profile_file_missing_reports_path(tmp_path):
    missing = tmp_path / "missing" / "robot_profiles.yaml"
    with pytest.raises(RuntimeError, match="Robot Profile file does not exist"):
        profile_utils.load_profile("missing", profile_file=str(missing))


def test_missing_profile_reports_file_and_available_profiles(monkeypatch, tmp_path):
    profiles_file = tmp_path / "robot_profiles.yaml"
    _write_profiles(profiles_file, {"robot_a": _minimal_profile()})

    with pytest.raises(RuntimeError) as error:
        profile_utils.load_profile("robot_b", profile_file=str(profiles_file))

    message = str(error.value)
    assert "robot_b" in message
    assert str(profiles_file.resolve()) in message
    assert "robot_a" in message


def test_invalid_controller_spawn_is_rejected(monkeypatch, tmp_path):
    profiles_file = tmp_path / "robot_profiles.yaml"
    _write_profiles(
        profiles_file,
        {
            "bad_robot": _minimal_profile(
                controllers={
                    "package": "robot_pkg",
                    "config": "config/ros2_controllers.yaml",
                    "spawn": "joint_trajectory_controller",
                }
            )
        },
    )

    monkeypatch.setattr(
        profile_utils,
        "get_package_share_directory",
        lambda package: str(tmp_path / package),
    )
    monkeypatch.setattr(
        profile_utils,
        "get_package_prefix",
        lambda package: str(tmp_path / "install" / package),
    )
    monkeypatch.setattr(
        profile_utils,
        "load_core_profile",
        lambda robot_profile, profile_file, resource_paths: CoreProfile(),
    )

    with pytest.raises(RuntimeError, match="controllers.spawn"):
        profile_utils.load_profile("bad_robot", profile_file=str(profiles_file))
