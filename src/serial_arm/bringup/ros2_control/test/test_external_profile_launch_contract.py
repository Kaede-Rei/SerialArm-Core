from pathlib import Path


LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"


def test_all_generic_launches_accept_external_profile_arguments():
    for launch_name in ["display.launch.py", "hardware.launch.py", "moveit.launch.py"]:
        source = (LAUNCH_DIR / launch_name).read_text(encoding="utf-8")
        assert 'DeclareLaunchArgument("profile_file", default_value="")' in source
        assert 'DeclareLaunchArgument("resource_paths", default_value="")' in source
        assert 'profile_file=context.launch_configurations.get("profile_file", "")' in source or 'profile_file = context.launch_configurations.get("profile_file", "")' in source
        assert 'resource_paths=context.launch_configurations.get("resource_paths", "")' in source or 'resource_paths = context.launch_configurations.get("resource_paths", "")' in source


def test_moveit_forwards_external_profile_arguments_to_hardware_launch():
    source = (LAUNCH_DIR / "moveit.launch.py").read_text(encoding="utf-8")
    assert '"profile_file": profile_file' in source
    assert '"resource_paths": resource_paths' in source
    assert '"controller_manager_name": context.launch_configurations.get(' in source


def test_hardware_spawns_profile_defined_controllers():
    source = (LAUNCH_DIR / "hardware.launch.py").read_text(encoding="utf-8")
    assert 'for controller_name in profile["controller_names"]' in source


def test_display_supports_xacro_description():
    source = (LAUNCH_DIR / "display.launch.py").read_text(encoding="utf-8")
    assert 'profile["description_type"] == "xacro"' in source
    assert 'FindExecutable(name="xacro")' in source
