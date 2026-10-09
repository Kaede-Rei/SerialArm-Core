"""Verify a recorded robot-model snapshot without relying on a stale absolute path"""
from __future__ import annotations

import hashlib
from pathlib import Path


def recorded_file_fingerprint(path):
    """Match the 64-bit FNV-1a variant used by C++ gravity_calibration.cpp"""
    value = 1469598103934665603
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(65536), b''):
            for byte in block:
                value = ((value ^ byte) * 1099511628211) & 0xffffffffffffffff
    return f'{value:016x}'


def matches_recorded_fingerprint(path, fingerprint):
    path = Path(path)
    if not path.is_file() or not isinstance(fingerprint, str) or not fingerprint:
        return False
    if len(fingerprint) == 16:
        return recorded_file_fingerprint(path) == fingerprint.lower()
    if len(fingerprint) == 64:
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(65536), b''):
                digest.update(block)
        return digest.hexdigest() == fingerprint.lower()
    return False


def verified_source_urdf(directory, metadata):
    """Prefer immutable task snapshot, fall back to a matching live file only"""
    directory = Path(directory).expanduser().resolve()
    expected = metadata.get('urdf_fingerprint', '')
    snapshot = directory / 'source' / 'model.urdf'
    if matches_recorded_fingerprint(snapshot, expected):
        return snapshot
    original = Path(metadata.get('urdf_path') or '/nonexistent').expanduser().resolve()
    if matches_recorded_fingerprint(original, expected):
        return original
    if snapshot.is_file():
        raise ValueError('任务自带 URDF 的指纹与记录不匹配，请检查任务快照，禁止跳过校验')
    raise ValueError('缺少与历史任务指纹匹配的 URDF，请恢复 source/model.urdf 或原始模型')


def resolve_relative_mesh(mesh_filename, snapshot, metadata):
    """Locate a referenced mesh; do not accept path escapes outside known model trees"""
    mesh = Path(mesh_filename)
    if mesh.is_absolute() or not mesh_filename or '://' in mesh_filename:
        return None
    original = Path(metadata.get('urdf_path') or '/nonexistent').expanduser().resolve()
    candidates = [snapshot.parent / mesh, original.parent / mesh]
    # Permit meshes inside each task/model tree without traversing arbitrary paths
    for allowed_root, candidate in zip((snapshot.parent.parent, original.parent.parent), candidates):
        candidate = candidate.resolve()
        if candidate.is_relative_to(allowed_root.resolve()) and candidate.is_file():
            return candidate
    return None
