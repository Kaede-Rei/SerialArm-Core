from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import xacro

sys.path.append(str(Path(__file__).resolve().parents[1] / "scripts"))
import profile_utils


class CoreProfile:
    core_config_path = "/resolved/core.yaml"
    hardware_plugin = "serial_arm_hardware_fake"
    hardware_config_path = "/resolved/hardware.yaml"


def hardware_params(robot_profile, **overrides):
    profile = profile_utils.load_profile(robot_profile)
    mappings = {
        "config_file": profile["core_config_path"],
        "hardware_plugin": profile["hardware_plugin"],
        "hardware_config": profile["hardware_config_path"],
        "serial_port": overrides.get("serial_port", ""),
        "baudrate": overrides.get("baudrate", ""),
        "bus": overrides.get("bus", ""),
    }
    document = xacro.process_file(profile["ros2_control_xacro_path"], mappings=mappings)
    root = ET.fromstring(document.toxml())
    hardware = root.find(".//ros2_control/hardware")
    assert hardware is not None
    return {
        param.attrib["name"]: (param.text or "")
        for param in hardware.findall("param")
    }


@pytest.fixture(autouse=True)
def fake_core_profile(monkeypatch):
    monkeypatch.setattr(
        profile_utils,
        "load_core_profile",
        lambda robot_profile, profiles_file: CoreProfile(),
    )


@pytest.mark.parametrize("robot_profile", ["dm_arm_gray", "dm_arm_white"])
def test_default_xacro_does_not_emit_empty_runtime_overrides(robot_profile):
    params = hardware_params(robot_profile)

    assert params["config_file"] == "/resolved/core.yaml"
    assert params["hardware_plugin"] == "serial_arm_hardware_fake"
    assert params["hardware_config"] == "/resolved/hardware.yaml"
    assert "serial_port" not in params
    assert "baudrate" not in params
    assert "bus" not in params


@pytest.mark.parametrize("robot_profile", ["dm_arm_gray", "dm_arm_white"])
def test_serial_port_override_reaches_ros2_control_hardware(robot_profile):
    params = hardware_params(robot_profile, serial_port="/dev/ttyACM1")

    assert params["serial_port"] == "/dev/ttyACM1"
    assert "baudrate" not in params
    assert "bus" not in params


@pytest.mark.parametrize("robot_profile", ["dm_arm_gray", "dm_arm_white"])
def test_baudrate_override_reaches_ros2_control_hardware(robot_profile):
    params = hardware_params(robot_profile, baudrate="1000000")

    assert params["baudrate"] == "1000000"
    assert "serial_port" not in params
    assert "bus" not in params


@pytest.mark.parametrize("robot_profile", ["dm_arm_gray", "dm_arm_white"])
def test_bus_override_reaches_ros2_control_hardware(robot_profile):
    params = hardware_params(robot_profile, bus="main_can")

    assert params["bus"] == "main_can"
    assert "serial_port" not in params
    assert "baudrate" not in params


@pytest.mark.parametrize("robot_profile", ["dm_arm_gray", "dm_arm_white"])
def test_multiple_overrides_reach_ros2_control_hardware(robot_profile):
    params = hardware_params(
        robot_profile,
        serial_port="/dev/ttyACM1",
        baudrate="1000000",
        bus="main_can",
    )

    assert params["serial_port"] == "/dev/ttyACM1"
    assert params["baudrate"] == "1000000"
    assert params["bus"] == "main_can"
