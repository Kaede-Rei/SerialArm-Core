"""Narrow runtime-parameter persistence with conflict checks and atomic replacement"""
from __future__ import annotations

from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil

import yaml


def sha256_file(path):
    path = Path(path)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _map(joint_names, values):
    if len(joint_names) != len(values):
        raise ValueError('parameter size does not match joint count')
    return '{' + ', '.join(f'{name}: {float(value):.9g}' for name, value in zip(joint_names, values)) + '}'


def _targets(joints, runtime, gravity_scale):
    friction = runtime.get('friction') or {}
    return {
        ('model', 'gravity_scale'): _map(joints, gravity_scale),
        ('capability', 'admittance', 'observer', 'mode'): str(runtime['observer_mode']),
        ('capability', 'admittance', 'observer', 'momentum_gain'): _map(joints, runtime['momentum_gain']),
        ('capability', 'admittance', 'calibration', 'torque_bias'): _map(joints, runtime['torque_bias']),
        ('capability', 'admittance', 'calibration', 'torque_threshold'): _map(joints, runtime['torque_threshold']),
        ('capability', 'admittance', 'calibration', 'friction', 'enabled'): 'true' if friction.get('enabled') else 'false',
        ('capability', 'admittance', 'calibration', 'friction', 'velocity_transition'): f"{float(friction.get('velocity_transition', 0.03)):.9g}",
        ('capability', 'admittance', 'calibration', 'friction', 'positive_coulomb'): _map(joints, friction.get('positive_coulomb', [])),
        ('capability', 'admittance', 'calibration', 'friction', 'positive_viscous'): _map(joints, friction.get('positive_viscous', [])),
        ('capability', 'admittance', 'calibration', 'friction', 'negative_coulomb'): _map(joints, friction.get('negative_coulomb', [])),
        ('capability', 'admittance', 'calibration', 'friction', 'negative_viscous'): _map(joints, friction.get('negative_viscous', [])),
        ('capability', 'admittance', 'controller', 'mass'): _map(joints, runtime['mass']),
        ('capability', 'admittance', 'controller', 'damping'): _map(joints, runtime['damping']),
        ('capability', 'admittance', 'controller', 'stiffness'): _map(joints, runtime['stiffness']),
        ('capability', 'admittance', 'controller', 'max_delta_q'): _map(joints, runtime['max_delta_q']),
        ('capability', 'admittance', 'controller', 'max_delta_q_dot'): _map(joints, runtime['max_delta_q_dot']),
    }


def patch_text(text, targets):
    lines = text.splitlines(keepends=True)
    stack = []
    found = set()
    pattern = re.compile(r'^(\s*)([A-Za-z0-9_]+):(\s*)([^#\n]*?)(\s+#.*)?(\r?\n)?$')
    for index, line in enumerate(lines):
        match = pattern.match(line)
        if not match:
            continue
        indent = len(match.group(1).replace('\t', '    '))
        key = match.group(2)
        value = match.group(4).strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        path = tuple(item[1] for item in stack) + (key,)
        if path in targets:
            comment = match.group(5) or ''
            newline = match.group(6) or ''
            lines[index] = f"{match.group(1)}{key}:{match.group(3)}{targets[path]}{comment}{newline}"
            found.add(path)
        if not value:
            stack.append((indent, key))
    missing = set(targets) - found
    if missing:
        raise ValueError('configuration is missing writable fields: ' + ', '.join('.'.join(x) for x in sorted(missing)))
    return ''.join(lines)


def preview(path, joints, runtime, gravity_scale):
    path = Path(path).resolve()
    before = path.read_text()
    after = patch_text(before, _targets(joints, runtime, gravity_scale))
    changed = []
    for left, right in zip(before.splitlines(), after.splitlines()):
        if left != right:
            changed.append({'before': left.strip(), 'after': right.strip()})
    return {'path': str(path), 'sha256': sha256_file(path), 'changes': changed, 'changed': before != after}


def save(path, expected_sha, joints, runtime, gravity_scale):
    path = Path(path).resolve()
    current = sha256_file(path)
    if current != expected_sha:
        raise ValueError('configuration changed outside the workspace; reload before saving')
    before = path.read_text()
    after = patch_text(before, _targets(joints, runtime, gravity_scale))
    yaml.safe_load(after)
    if before == after:
        return {'path': str(path), 'sha256': current, 'backup': '', 'changed': False}
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    backup = path.with_name(path.name + '.bak-' + stamp)
    shutil.copy2(path, backup)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(after)
    try:
        yaml.safe_load(temp.read_text())
        temp.replace(path)
        yaml.safe_load(path.read_text())
    except Exception:
        temp.unlink(missing_ok=True)
        raise
    return {'path': str(path), 'sha256': sha256_file(path), 'backup': str(backup), 'changed': True}


def export_telemetry(root, events, metadata):
    base = Path(root) / '.install' / 'exports'
    base.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    path = base / f'session-{stamp}.jsonl'
    with path.open('w') as stream:
        stream.write(json.dumps({'type': 'metadata', **metadata}, ensure_ascii=False) + '\n')
        for event in events:
            stream.write(json.dumps(event, ensure_ascii=False, separators=(',', ':')) + '\n')
    return {'path': str(path), 'samples': len(events)}
