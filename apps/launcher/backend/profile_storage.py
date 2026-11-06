"""Profile-owned data directories, independent of catalogue filename or size."""
from __future__ import annotations
from pathlib import Path
import json
import os
import re
import shutil
import uuid

import yaml


def _identity(info, repository_root):
    root = Path(repository_root).resolve()
    name = str(info['profile'])
    if not name or name in ('.', '..') or any(c in name for c in '/\\\0\r\n'):
        raise ValueError('Profile 名称不适合作为存储目录')
    profile_file = Path(info['profile_file']).expanduser().resolve()
    folder = profile_file.parent.parent if profile_file.parent.name == 'config' else profile_file.parent
    writable = os.environ.get('SERIAL_ARM_BUNDLE_PROFILE_PREFIX')
    if writable and profile_file.is_relative_to(Path(writable).resolve() / 'share'):
        home = Path(writable).resolve() / 'profiles' / name
    else:
        shared = root / 'src/robot_supports/profiles'
        if profile_file.is_relative_to(shared) or ('serial_arm_robot_profiles' in profile_file.parts and profile_file.parent.name == 'config'):
            robot = root / 'src/robot_supports/robots' / name
            home = robot if robot.is_dir() else shared / name
        else:
            # Adding a neighbour or renaming the catalogue never moves a robot.
            home = folder if folder.name == name else folder / name
    return name, profile_file, folder, home


def legacy_profile_homes(info, repository_root):
    name, profile_file, folder, home = _identity(info, repository_root)
    return list(dict.fromkeys(path for path in (folder, folder / name, profile_file.parent) if path != home))


def _metadata(path):
    try:
        return (json.loads(path.read_text(encoding='utf-8')) if path.suffix == '.json'
                else yaml.safe_load(path.read_text(encoding='utf-8'))) or {}
    except (OSError, ValueError, UnicodeError, yaml.YAMLError):
        return {}


def _owner(path):
    data = _metadata(path)
    if not isinstance(data, dict):
        return None
    scope = data.get('scope')
    return data.get('profile') or (scope.get('profile') if isinstance(scope, dict) else None)


def _same_tree(source, target):
    a = {p.relative_to(source) for p in source.rglob('*') if p.is_file()}
    b = {p.relative_to(target) for p in target.rglob('*') if p.is_file()}
    return a == b and all((source / p).read_bytes() == (target / p).read_bytes() for p in a)


def _copy_record(source, target, metadata=None):
    # Never alter historical sources or combine files from divergent tasks.
    if source.is_symlink() or any(p.is_symlink() for p in source.rglob('*')):
        return
    if target.exists():
        if target.is_dir() and _same_tree(source, target):
            return target
        index = 1
        candidate = target.with_name(target.name + f'-legacy-{index}')
        while candidate.exists():
            if candidate.is_dir() and _same_tree(source, candidate):
                return candidate
            index += 1
            candidate = target.with_name(target.name + f'-legacy-{index}')
        target = candidate
    target.parent.mkdir(parents=True, exist_ok=True)
    staging = target.with_name(target.name + '.importing')
    index = 1
    while staging.exists():
        staging = target.with_name(target.name + f'.importing-{index}')
        index += 1
    try:
        shutil.copytree(source, staging)
        if metadata is not None:
            (staging / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
        staging.rename(target)
        return target
    except (OSError, shutil.Error):
        raise ValueError(f'无法迁移历史记录 {source}；原文件保留，请检查目录权限与磁盘空间')


def _copy_trajectory(source, target, name, repository_root):
    from trajectory_library import safe_profile
    metadata = _metadata(source / 'metadata.json')
    key = metadata.get('id') if isinstance(metadata, dict) else None
    if not isinstance(key, str) or not re.fullmatch(r'[0-9a-f]{32}', key):
        return None
    collision = False
    # These are the same two roots TrajectoryLibrary.list/ensure consume.
    roots = (target.parent, Path(repository_root) / '.install' / 'teach-replay' / safe_profile(name))
    for base in roots:
        for path in base.glob('*/metadata.json'):
            other = _metadata(path)
            if not isinstance(other, dict) or other.get('id') != key or _owner(path) != name:
                continue
            if _same_tree(source, path.parent):
                return path.parent
            collision = True
    if collision:
        metadata = {**metadata, 'id': uuid.uuid4().hex,
                    'source_id': metadata.get('source_id') or key}
        return _copy_record(source, target, metadata)
    return _copy_record(source, target)


def _import_owned_records(info, repository_root, home):
    name = str(info['profile'])
    receipt_path = home / '.serialarm' / 'legacy-imports.yaml'
    receipt = _metadata(receipt_path)
    imported = receipt.get('imports') if isinstance(receipt, dict) else None
    imports = dict(imported) if isinstance(imported, dict) else {}
    changed = False
    def import_record(record, destination, category=None):
        nonlocal changed
        previous = imports.get(str(record))
        # A removed migrated trajectory was explicitly deleted by the user;
        # leaving its historical source intact must not resurrect it next time.
        if isinstance(previous, str):
            return
        copied = (_copy_trajectory(record, destination, name, repository_root)
                  if category == 'teach-replay' else _copy_record(record, destination))
        if copied is not None:
            imports[str(record)] = str(copied.relative_to(home)) if copied.is_relative_to(home) else str(copied)
            changed = True
    for legacy in legacy_profile_homes(info, repository_root):
        source = legacy / '.serialarm'
        if not source.is_dir():
            continue
        for category in ('model-calibration', 'teach-replay'):
            base = source / category
            if not base.is_dir():
                continue
            for record in base.iterdir():
                if record.is_dir() and _owner(record / 'metadata.json') == name:
                    import_record(record, home / '.serialarm' / category / record.name, category)
        for category in ('tcp-calibration', 'geometry-calibration'):
            record = source / category / name
            candidates = list(record.glob('*session.yaml')) if record.is_dir() else []
            if candidates and all(_owner(path) == name for path in candidates):
                import_record(record, home / '.serialarm' / category / name)
    if changed:
        temp = receipt_path.with_suffix('.yaml.tmp')
        temp.write_text(yaml.safe_dump({'profile': name, 'imports': imports}, allow_unicode=True), encoding='utf-8')
        temp.replace(receipt_path)


def profile_home(info, repository_root):
    _, _, _, home = _identity(info, repository_root)
    _import_owned_records(info, repository_root, home)
    return home


def category_directory(info, repository_root, category):
    if not re.fullmatch(r'[a-z][a-z0-9-]*', category):
        raise ValueError('数据分类名称无效')
    return profile_home(info, repository_root) / '.serialarm' / category
