"""Persistent data directories for each robot Profile."""
from __future__ import annotations
from pathlib import Path
import os
import re


def profile_home(info, repository_root):
    root = Path(repository_root).resolve()
    name = str(info['profile'])
    if not name or name in ('.', '..') or any(c in name for c in '/\\\0\r\n'):
        raise ValueError('Profile 名称不适合作为存储目录')
    profile_file = Path(info['profile_file']).expanduser().resolve()
    # Bundled installed resources cannot be modified in place
    writable = os.environ.get('SERIAL_ARM_BUNDLE_PROFILE_PREFIX')
    if writable and profile_file.is_relative_to(Path(writable).resolve() / 'share'):
        return Path(writable).resolve() / 'profiles' / name
    shared = root / 'src/robot_supports/profiles'
    if profile_file.is_relative_to(shared) or ('serial_arm_robot_profiles' in profile_file.parts and profile_file.parent.name == 'config'):
        robot = root / 'src/robot_supports/robots' / name
        if robot.is_dir():
            return robot
        return shared / name
    folder = profile_file.parent.parent if profile_file.parent.name == 'config' else profile_file.parent
    # A single robot_profiles.yaml may describe several distinct robots
    # Keep their runtime records separate without changing the original YAML
    if profile_file.name == 'robot_profiles.yaml' and profile_file.is_file():
        try:
            import yaml
            entries = (yaml.safe_load(profile_file.read_text()) or {}).get('profiles') or {}
            if isinstance(entries, dict) and len(entries) > 1:
                return folder / name
        except (OSError, ValueError, TypeError):
            pass
    return folder


def category_directory(info, repository_root, category):
    if not re.fullmatch(r'[a-z][a-z0-9-]*', category):
        raise ValueError('数据分类名称无效')
    return profile_home(info, repository_root) / '.serialarm' / category
