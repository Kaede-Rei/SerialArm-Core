"""Read-only historical/current URDF compatibility for the calibration COM viewer

This does not authorize replay, recomputation or deploying historical parameters
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from model_calibration_source import recorded_file_fingerprint, verified_source_urdf


def _non_inertial_tree(node):
    # Compare the complete robot description, excluding only Link inertial payloads
    return (node.tag, tuple(sorted(node.attrib.items())),
            (node.text or '').strip(),
            tuple(_non_inertial_tree(child) for child in node
                  if not (node.tag == 'link' and child.tag == 'inertial')))


def _inertial(link):
    inertial = link.find('inertial')
    if inertial is None:
        return None
    origin = inertial.find('origin')
    mass = inertial.find('mass')
    tensor = inertial.find('inertia')
    if mass is None or tensor is None:
        raise ValueError('invalid inertial definition for ' + link.attrib.get('name', '?'))
    xyz = [float(x) for x in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
    m = float(mass.get('value', 'nan'))
    components = [float(tensor.get(field, 'nan')) for field in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')]
    if len(xyz) != 3 or not all(math.isfinite(v) for v in [m, *xyz, *components]):
        raise ValueError('nonfinite inertial definition for ' + link.attrib.get('name', '?'))
    return {'mass': m, 'com': xyz, 'tensor': components}


def _physics_warnings(name, obj):
    if obj is None:
        return []
    m = obj['mass']
    xx, xy, xz, yy, yz, zz = obj['tensor']
    problems = []
    if m <= 0:
        problems.append(name + ': 质量必须为正')
    # A real rigid-body rotational inertia around the COM is positive semidefinite
    if (min(xx, yy, zz) <= 0 or xx * yy - xy * xy <= 0 or
        xx * (yy * zz - yz * yz) - xy * (xy * zz - yz * xz) + xz * (xy * yz - yy * xz) <= 0):
        problems.append(name + ': 惯性张量不满足正定性，无法认定为物理有效惯量')
    # Additional necessary triangle inequalities, not a full geometry certification
    if min(yy + zz - xx, xx + zz - yy, xx + yy - zz) < -1e-10:
        problems.append(name + ': 惯性张量不满足主惯量三角不等式')
    return problems


def compare_calibration_urdfs(directory, metadata, current_urdf):
    """Only read URDFs, verify source integrity, forbid any kinematic/geometry diff"""
    source = verified_source_urdf(directory, metadata)
    current = Path(current_urdf).expanduser().resolve()
    if not current.is_file():
        raise ValueError('current visual URDF is missing')
    expected = str(metadata.get('urdf_fingerprint') or '').lower()
    actual = recorded_file_fingerprint(current)
    if current == source:
        return {'comparable': True, 'same_model': True, 'expected': expected,
                'actual': actual, 'changed_links': [], 'source_inertials': {}, 'current_inertials': {},
                'warnings': [], 'preview_only': True}
    source_robot = ET.parse(source).getroot()
    current_robot = ET.parse(current).getroot()
    if source_robot.tag != 'robot' or current_robot.tag != 'robot':
        raise ValueError('URDF robot root is missing')
    if _non_inertial_tree(source_robot) != _non_inertial_tree(current_robot):
        return {'comparable': False, 'same_model': False, 'expected': expected,
                'actual': actual, 'changed_links': [], 'source_inertials': {},
                'current_inertials': {}, 'warnings': ['机器人关节、坐标系、几何或其他非惯量内容发生变化，禁止历史候选预览'],
                'preview_only': True}
    source_links = {link.get('name'): link for link in source_robot.findall('link')}
    current_links = {link.get('name'): link for link in current_robot.findall('link')}
    if source_links.keys() != current_links.keys():
        raise ValueError('historical/current link names differ')
    source_values = {name: _inertial(link) for name, link in source_links.items()}
    current_values = {name: _inertial(link) for name, link in current_links.items()}
    changed = [name for name in source_links if source_values[name] != current_values[name]]
    warnings = []
    for name in changed:
        warnings.extend(_physics_warnings(name, current_values[name]))
        if (source_values[name] is None) != (current_values[name] is None):
            warnings.append(name + ': inertial 存在性已变化，该 Link 无法进行质心对比')
    return {'comparable': True, 'same_model': not changed, 'expected': expected,
            'actual': actual, 'changed_links': changed,
            'source_inertials': {name: {'mass': val['mass'], 'com': val['com']} for name, val in source_values.items() if val},
            'current_inertials': {name: {'mass': val['mass'], 'com': val['com']} for name, val in current_values.items() if val},
            'warnings': warnings, 'preview_only': True}
