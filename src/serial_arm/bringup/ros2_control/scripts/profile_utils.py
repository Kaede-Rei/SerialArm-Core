import os
from pathlib import Path

import yaml


DEFAULT_PROFILE_PACKAGE = "serial_arm_robot_profiles"
DEFAULT_PROFILE_RELATIVE_PATH = Path("config") / "robot_profiles.yaml"
DEFAULT_CONTROLLER_SPAWN = [
    "joint_state_broadcaster",
    "joint_trajectory_controller",
]


def get_package_share_directory(package):
    from ament_index_python.packages import get_package_share_directory as resolve_share

    return resolve_share(package)


def get_package_prefix(package):
    from ament_index_python.packages import get_package_prefix as resolve_prefix

    return resolve_prefix(package)


def normalize_resource_paths(resource_paths=None):
    """Normalize launch/Python resource roots to a stable, de-duplicated list"""
    if resource_paths is None:
        return []

    if isinstance(resource_paths, (str, Path)):
        raw = str(resource_paths)
        values = raw.split(os.pathsep) if raw else []
    else:
        values = []
        for value in resource_paths:
            if value is None:
                continue
            text = str(value)
            if not text:
                continue
            values.extend(text.split(os.pathsep))

    result = []
    for value in values:
        value = value.strip()
        if not value:
            continue
        normalized = str(Path(value).expanduser())
        if normalized not in result:
            result.append(normalized)
    return result


def resolve_profiles_file(profile_file=""):
    if profile_file:
        profiles_file = Path(profile_file).expanduser()
        if not profiles_file.is_file():
            raise RuntimeError(
                f"Robot Profile file does not exist: {profiles_file}"
            )
        return profiles_file.resolve()

    return (
        Path(get_package_share_directory(DEFAULT_PROFILE_PACKAGE))
        / DEFAULT_PROFILE_RELATIVE_PATH
    )


def _load_profiles_document(profiles_file):
    try:
        with Path(profiles_file).open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
    except OSError as error:
        raise RuntimeError(
            f"failed to read Robot Profile file '{profiles_file}': {error}"
        ) from error
    except yaml.YAMLError as error:
        raise RuntimeError(
            f"failed to parse Robot Profile file '{profiles_file}': {error}"
        ) from error

    profiles = document.get("profiles")
    if not isinstance(profiles, dict):
        raise RuntimeError(
            f"Robot Profile file '{profiles_file}' must contain a 'profiles' mapping"
        )
    return profiles


def _require_mapping(profile, key, robot_profile, profiles_file):
    value = profile.get(key)
    if not isinstance(value, dict):
        raise RuntimeError(
            f"Robot profile '{robot_profile}' in '{profiles_file}' "
            f"must define '{key}'"
        )
    return value


def _profile_packages(profile):
    packages = []
    candidates = [
        (profile.get("core") or {}).get("package"),
        (profile.get("hardware") or {}).get("config_package"),
        (profile.get("hardware") or {}).get("plugin"),
        (profile.get("description") or {}).get("package"),
        (profile.get("controllers") or {}).get("package"),
        (profile.get("moveit") or {}).get("package"),
    ]
    for package in candidates:
        if isinstance(package, str) and package and package not in packages:
            packages.append(package)
    return packages


def _ament_resource_paths(profile):
    """Return install prefixes for packages that are visible in the current ROS 2 overlay"""
    paths = []
    for package in _profile_packages(profile):
        try:
            prefix = str(Path(get_package_prefix(package)))
        except Exception:
            # Core/Hardware resources may intentionally live outside ament and be
            # supplied through the explicit resource_paths launch argument
            continue
        if prefix not in paths:
            paths.append(prefix)
    return paths


def _merge_resource_paths(explicit_paths, discovered_paths):
    result = []
    for path in [*explicit_paths, *discovered_paths]:
        if path and path not in result:
            result.append(path)
    return result


def load_profile(robot_profile, profile_file="", resource_paths=None):
    profiles_file = resolve_profiles_file(profile_file)
    profiles = _load_profiles_document(profiles_file)
    if robot_profile not in profiles:
        available = ", ".join(sorted(profiles.keys())) or "<none>"
        raise RuntimeError(
            f"Robot profile '{robot_profile}' was not found in '{profiles_file}'. "
            f"Available profiles: {available}"
        )

    raw_profile = profiles[robot_profile]
    if not isinstance(raw_profile, dict):
        raise RuntimeError(
            f"Robot profile '{robot_profile}' in '{profiles_file}' must be a mapping"
        )
    profile = dict(raw_profile)

    explicit_resource_paths = normalize_resource_paths(resource_paths)
    resolved_resource_paths = _merge_resource_paths(
        explicit_resource_paths,
        _ament_resource_paths(profile),
    )

    core_profile = load_core_profile(
        robot_profile,
        str(profiles_file),
        resolved_resource_paths,
    )
    profile["profile_file"] = str(profiles_file)
    profile["resource_paths"] = resolved_resource_paths
    profile["core_config_path"] = core_profile.core_config_path
    profile["hardware_plugin"] = core_profile.hardware_plugin
    profile["hardware_config_path"] = core_profile.hardware_config_path

    description = _require_mapping(
        profile, "description", robot_profile, profiles_file
    )
    description_package = description.get("package")
    if not description_package:
        raise RuntimeError(
            f"Robot profile '{robot_profile}' in '{profiles_file}' "
            "must define description.package"
        )

    if description.get("xacro"):
        profile["description_type"] = "xacro"
        profile["description_path"] = resolve_package_path(
            description_package, description["xacro"]
        )
        profile["description_xacro_path"] = profile["description_path"]
    elif description.get("urdf"):
        profile["description_type"] = "urdf"
        profile["description_path"] = resolve_package_path(
            description_package, description["urdf"]
        )
        profile["description_urdf_path"] = profile["description_path"]
    else:
        raise RuntimeError(
            f"Robot profile '{robot_profile}' in '{profiles_file}' must define "
            "description.xacro or description.urdf"
        )

    ros2_control_xacro = description.get("ros2_control_xacro")
    if not ros2_control_xacro:
        raise RuntimeError(
            f"Robot profile '{robot_profile}' in '{profiles_file}' must define "
            "description.ros2_control_xacro"
        )
    profile["ros2_control_xacro_path"] = resolve_package_path(
        description_package, ros2_control_xacro
    )

    controllers = _require_mapping(
        profile, "controllers", robot_profile, profiles_file
    )
    controllers_package = controllers.get("package")
    controllers_config = controllers.get("config")
    if not controllers_package or not controllers_config:
        raise RuntimeError(
            f"Robot profile '{robot_profile}' in '{profiles_file}' must define "
            "controllers.package and controllers.config"
        )
    profile["controllers_path"] = resolve_package_path(
        controllers_package, controllers_config
    )

    controller_names = controllers.get("spawn", DEFAULT_CONTROLLER_SPAWN)
    if not isinstance(controller_names, list) or any(
        not isinstance(name, str) or not name.strip() for name in controller_names
    ):
        raise RuntimeError(
            f"Robot profile '{robot_profile}' in '{profiles_file}' has invalid "
            "controllers.spawn; expected a list of non-empty controller names"
        )
    profile["controller_names"] = [name.strip() for name in controller_names]

    moveit = profile.get("moveit")
    if moveit and moveit.get("package"):
        profile["moveit_package"] = moveit["package"]
    return profile


def load_core_profile(robot_profile, profiles_file, resource_paths=None):
    try:
        from serial_arm import load_robot_profile_core
    except ImportError as error:
        raise RuntimeError(
            "failed to import serial_arm Python binding required "
            f"for Core Robot Profile resolution: {error}"
        ) from error
    return load_robot_profile_core(
        robot_profile,
        profiles_file,
        normalize_resource_paths(resource_paths),
    )


def require_moveit_package(profile, robot_profile):
    moveit = profile.get("moveit")
    if not moveit or not moveit.get("package"):
        raise RuntimeError(f"Robot profile '{robot_profile}' does not define MoveIt support")
    return moveit["package"]


def resolve_package_path(package, relative_path):
    return str(Path(get_package_share_directory(package)) / relative_path)
