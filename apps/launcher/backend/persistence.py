"""Narrow runtime-parameter persistence with conflict checks and atomic replacement"""
from __future__ import annotations

from datetime import datetime
import hashlib
import csv
import math
import json
import os
from pathlib import Path
import re
import shutil
import difflib
import tempfile

import yaml

from model_calibration_source import matches_recorded_fingerprint, verified_source_urdf, resolve_relative_mesh


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
    """Patch existing YAML scalar leaves via parser source marks, preserving layout.

    Updates flow or block-style joint dictionaries without overwriting child
    lines, comments, anchors or unrelated settings. The complete parsed YAML
    must differ *only* at explicitly requested paths.
    """
    import copy
    from yaml.nodes import MappingNode, ScalarNode

    original = yaml.safe_load(text)
    root = yaml.compose(text)
    if not isinstance(original, dict) or not isinstance(root, MappingNode):
        raise ValueError('Core YAML 根节点必须是映射')
    expected = copy.deepcopy(original)
    edits = []
    for path, value in targets.items():
        if not isinstance(path, tuple) or not path:
            raise ValueError('无效的 Core 参数路径')
        node = root
        data = expected
        for key in path:
            if not isinstance(node, MappingNode) or not isinstance(data, dict):
                raise ValueError('Core 参数不是预期的映射: ' + '.'.join(path))
            matched = [(k, v) for k, v in node.value if isinstance(k, ScalarNode) and k.value == key]
            if len(matched) != 1 or key not in data:
                raise ValueError('Core 缺少或重复定义参数: ' + '.'.join(path))
            node = matched[0][1]
            data = data[key]
        replacement = yaml.safe_load(value) if isinstance(value, str) else value
        if isinstance(node, MappingNode):
            if not isinstance(replacement, dict) or not isinstance(data, dict):
                raise ValueError('Core 参数映射类型不匹配: ' + '.'.join(path))
            if set(data) != set(replacement):
                raise ValueError('Core 关节映射与候选不一致: ' + '.'.join(path))
            for name, new_value in replacement.items():
                children = [v for k, v in node.value if isinstance(k, ScalarNode) and k.value == name]
                if len(children) != 1 or not isinstance(children[0], ScalarNode):
                    raise ValueError('Core 关节字段无效: ' + '.'.join(path + (str(name),)))
                if not isinstance(new_value, (int, float)) or isinstance(new_value, bool) or not math.isfinite(float(new_value)):
                    raise ValueError('Core 关节候选参数非有限数值: ' + '.'.join(path + (str(name),)))
                leaf = children[0]
                # Replace the numeric token, never the preceding indentation,
                # newline, trailing comment, or following mapping entry
                raw = repr(float(new_value))
                edits.append((leaf.start_mark.index, leaf.end_mark.index, raw))
            parent = expected
            for key in path[:-1]:
                parent = parent[key]
            parent[path[-1]] = replacement
        elif isinstance(node, ScalarNode) and not isinstance(replacement, (dict, list)):
            if isinstance(replacement, float) and not math.isfinite(replacement):
                raise ValueError('Core 候选参数非有限数值: ' + '.'.join(path))
            edits.append((node.start_mark.index, node.end_mark.index, str(value)))
            parent = expected
            for key in path[:-1]:
                parent = parent[key]
            parent[path[-1]] = replacement
        else:
            raise ValueError('Core 参数类型不匹配: ' + '.'.join(path))
    edits.sort()
    for (start, end, _), (next_start, _, _) in zip(edits, edits[1:]):
        if end > next_start:
            raise ValueError('Core 参数修改区间重叠，拒绝导出')
    updated = text
    for start, end, replacement in reversed(edits):
        updated = updated[:start] + replacement + updated[end:]
    # Structural verification also catches accidental alias updates or sibling
    # changes that would still produce syntactically valid YAML
    actual = yaml.safe_load(updated)
    if actual != expected:
        raise ValueError('Core 更新后的参数超出指定范围，已拒绝写入')
    return updated


def preview(path, joints, runtime, gravity_scale):
    path = Path(path).resolve()
    before = path.read_text()
    after = patch_text(before, _targets(joints, runtime, gravity_scale))
    changed = []
    for line in difflib.ndiff(before.splitlines(), after.splitlines()):
        if line.startswith(('- ', '+ ')):
            changed.append({'before': line[2:].strip() if line[0] == '-' else '',
                            'after': line[2:].strip() if line[0] == '+' else ''})
    return {'path': str(path), 'sha256': sha256_file(path), 'changes': changed, 'changed': before != after}


def save(path, expected_sha, joints, runtime, gravity_scale, storage_home=None):
    path = Path(path).resolve()
    current = sha256_file(path)
    if current != expected_sha:
        raise ValueError('configuration changed outside the workspace; reload before saving')
    before = path.read_text()
    after = patch_text(before, _targets(joints, runtime, gravity_scale))
    yaml.safe_load(after)
    if before == after:
        return {'path': str(path), 'sha256': current, 'backup': '', 'changed': False}
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    backup_dir = Path(storage_home).resolve() / '.serialarm' / 'backups' if storage_home else path.parent
    backup_dir.mkdir(parents=True, exist_ok=True)
    backup = backup_dir / (path.name + '.bak-' + stamp)
    shutil.copy2(path, backup)
    temp = None
    replaced = False
    try:
        # Keep temp in the same filesystem and never overwrite a concurrent .tmp
        with tempfile.NamedTemporaryFile('w', dir=path.parent, prefix=path.name + '.',
                                         suffix='.tmp', encoding='utf-8', delete=False) as stream:
            temp = Path(stream.name)
            stream.write(after)
            stream.flush()
            os.fsync(stream.fileno())
        if sha256_file(path) != expected_sha:
            raise ValueError('保存期间配置已被其他进程修改，请重新加载后再保存')
        if yaml.safe_load(temp.read_text(encoding='utf-8')) != yaml.safe_load(after):
            raise ValueError('临时配置内容校验失败，原配置未修改')
        os.replace(temp, path)
        replaced = True
        if yaml.safe_load(path.read_text(encoding='utf-8')) != yaml.safe_load(after):
            raise ValueError('新配置落盘校验失败')
    except Exception:
        if replaced:
            shutil.copy2(backup, path)
        raise
    finally:
        if temp is not None:
            temp.unlink(missing_ok=True)
    return {'path': str(path), 'sha256': sha256_file(path), 'backup': str(backup), 'changed': True}


def export_telemetry(root, events, metadata):
    base = Path(root) / '.serialarm' / 'exports'
    base.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    path = base / f'session-{stamp}.jsonl'
    with path.open('w') as stream:
        stream.write(json.dumps({'type': 'metadata', **metadata}, ensure_ascii=False) + '\n')
        for event in events:
            stream.write(json.dumps(event, ensure_ascii=False, separators=(',', ':')) + '\n')
    return {'path': str(path), 'samples': len(events)}


def _load_model_calibration_task(directory):
    directory = Path(directory).resolve()
    metadata_path = directory / 'metadata.json'
    result_path = directory / 'result.json'
    frames_path = directory / 'frames.csv'
    trajectory_path = directory / 'trajectory.csv'
    for path in (metadata_path, result_path, frames_path, trajectory_path):
        if not path.is_file():
            raise ValueError(f'model calibration task file is missing: {path.name}')
    metadata = json.loads(metadata_path.read_text())
    result = json.loads(result_path.read_text())
    if result.get('task_id') != metadata.get('task_id'):
        raise ValueError('model calibration task id mismatch')
    return directory, metadata, result


def _insert_or_replace_model_scalar(text, key, value):
    lines = text.splitlines(keepends=True)
    model_indent = None
    model_start = None
    model_end = len(lines)
    key_pattern = re.compile(r'^(\s*)model:\s*(?:#.*)?(?:\r?\n)?$')
    scalar_pattern = re.compile(rf'^(\s*){re.escape(key)}:\s*([^#\n]*?)(\s+#.*)?(\r?\n)?$')
    for index, line in enumerate(lines):
        match = key_pattern.match(line)
        if match:
            model_indent = len(match.group(1).replace('\t', '    '))
            model_start = index
            break
    if model_start is None:
        raise ValueError('configuration is missing model section')
    child_indent = None
    for index in range(model_start + 1, len(lines)):
        stripped = lines[index].strip()
        if not stripped or stripped.startswith('#'):
            continue
        indent = len(lines[index]) - len(lines[index].lstrip(' '))
        if indent <= model_indent:
            model_end = index
            break
        if child_indent is None:
            child_indent = indent
        match = scalar_pattern.match(lines[index])
        if match and indent > model_indent:
            comment = match.group(3) or ''
            newline = match.group(4) or '\n'
            lines[index] = f"{match.group(1)}{key}: {value}{comment}{newline}"
            return ''.join(lines)
    indent_text = ' ' * (child_indent if child_indent is not None else model_indent + 2)
    lines.insert(model_end, f'{indent_text}{key}: {value}\n')
    return ''.join(lines)


def _candidate_threshold(candidate, joints):
    p99 = candidate.get('validation_candidate', {}).get('p99', [])
    noise = candidate.get('noise_rms', [])
    if len(p99) != len(joints) or len(noise) != len(joints):
        raise ValueError('candidate validation metrics do not match joint count')
    return [max(0.02, 1.2 * float(p99[i]), 3.0 * float(noise[i])) for i in range(len(joints))]


def _same_gravity_candidate(a, b):
    """Exact physical parameter comparison independent of JSON field ordering"""
    def moments(result):
        return {item['link_name']: tuple(float(x) for x in item['value']) for item in (result.get('first_moments') or [])}
    left, right = moments(a), moments(b)
    return bool(left) and left.keys() == right.keys() and all(
        all(math.isclose(x, y, abs_tol=1e-11, rel_tol=1e-10) for x, y in zip(left[name], right[name]))
        for name in left
    ) and all(math.isclose(float(x), float(y), abs_tol=1e-11, rel_tol=1e-10)
              for x, y in zip(a.get('torque_bias') or [], b.get('torque_bias') or [])) and \
        len(a.get('torque_bias') or []) == len(b.get('torque_bias') or []) and \
        (a.get('candidate_masses') or {}) == (b.get('candidate_masses') or {})


def _recorded_joint_bias(directory, metadata, joints):
    """Preserve the original bias only when an old offline report has none"""
    core = directory / 'source' / 'core.yaml'
    if not matches_recorded_fingerprint(core, metadata.get('core_fingerprint')):
        core = Path(metadata.get('core_config_path') or '/missing').expanduser().resolve()
    if not matches_recorded_fingerprint(core, metadata.get('core_fingerprint')):
        raise ValueError('旧离线候选缺少力矩偏置，且找不到指纹匹配的原始 Core 配置')
    cfg = yaml.safe_load(core.read_text(encoding='utf-8'))
    values = (((cfg or {}).get('capability') or {}).get('admittance') or {}).get('calibration', {}).get('torque_bias')
    if not isinstance(values, dict) or set(values) != set(joints):
        raise ValueError('原始 Core 配置的力矩偏置与受控关节不一致')
    ordered = [values[joint] for joint in joints]
    if any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in ordered):
        raise ValueError('原始 Core 配置的力矩偏置含无效数值')
    return ordered


def build_gravity_correction_payload(directory, *, allow_unvalidated=False, prefer_recomputed=False):
    directory, metadata, result = _load_model_calibration_task(directory)
    gravity = result.get('gravity_result') or {}
    if prefer_recomputed:
        recomputed = directory / 'recomputed-candidate.json'
        if not recomputed.is_file():
            raise ValueError('此任务尚未生成离线重力候选，请先执行离线重算')
        gravity = json.loads(recomputed.read_text(encoding='utf-8'))
        if (gravity.get('task_id') != metadata.get('task_id')
                or gravity.get('core_fingerprint') != metadata.get('core_fingerprint')
                or gravity.get('urdf_fingerprint') != metadata.get('urdf_fingerprint')):
            raise ValueError('离线候选与标定任务来源不匹配')
        def sha(path):
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(65536), b''):
                    digest.update(chunk)
            return digest.hexdigest()
        source = verified_source_urdf(directory, metadata)
        static_file = directory / 'accepted_static.csv'
        accepted_hash = sha(static_file) if static_file.is_file() else None
        if (not (directory / 'frames.csv').is_file()
                or gravity.get('frames_sha256') != sha(directory / 'frames.csv')
                or gravity.get('accepted_static_sha256') != accepted_hash
                or gravity.get('source_sha256') != sha(source)):
            raise ValueError('离线候选的原始数据或模型已发生变化，请重新执行离线重算')
    joints = result.get('joint_names') or metadata.get('joint_names') or []
    if not allow_unvalidated and (result.get('phase') != 'complete' or not gravity.get('static_pass')):
        raise ValueError('model calibration task has no validated static candidate')
    recomputed_used = gravity is not result.get('gravity_result')
    friction_matches = not recomputed_used or _same_gravity_candidate(gravity, result.get('gravity_result') or {})
    friction_pass = bool(result.get('friction_pass')) and friction_matches
    torque_bias = gravity.get('torque_bias')
    bias_source = 'fitted_candidate'
    if recomputed_used and torque_bias is None:
        # Historical offline fits do not save the per-joint residual bias
        # Reuse the verified original configuration without inventing a fit
        torque_bias = _recorded_joint_bias(directory, metadata, joints)
        bias_source = 'original_core_unrecalibrated'
    source_id = 'recomputed-candidate.json' if recomputed_used else 'result.json'
    lock_names = list(gravity.get('locked_links') or []) if gravity.get('identification_mode', 'global') == 'local' else []
    fitted_names = list(gravity.get('fitted_links') or [])
    if not fitted_names:
        fitted_names = [x['link_name'] for x in gravity.get('first_moments', []) if x['link_name'] not in lock_names]
    if len(fitted_names) != len(set(fitted_names)) or len(lock_names) != len(set(lock_names)) or set(lock_names) & set(fitted_names):
        raise ValueError('candidate fitted/locked Link lists overlap or contain duplicates')
    if gravity.get('identification_mode', 'global') not in ('global', 'local'):
        raise ValueError('candidate identification mode is invalid')
    if gravity.get('identification_mode', 'global') == 'global':
        if lock_names or set(fitted_names) != {item['link_name'] for item in gravity.get('first_moments') or []}:
            raise ValueError('全局辨识只能包含完整候选，不接受局部锁定或局部候选拼接')
    if gravity.get('identification_mode', 'global') == 'local':
        candidates = _identifiable_links_from_snapshot(directory, metadata)
        allowed = {item['name'] for item in candidates}
        if not allowed or set(fitted_names) | set(lock_names) != allowed or not set(fitted_names) or set(fitted_names) & set(lock_names):
            raise ValueError('local identification Link selection differs from the recorded model')
        import xml.etree.ElementTree as ET
        root = ET.parse(verified_source_urdf(directory, metadata)).getroot()
        link_map = {link.get('name'): link for link in root.findall('link')}
        candidate_moments = {item['link_name']: item['value'] for item in gravity.get('first_moments') or []}
        for name in lock_names:
            link = link_map[name]
            inertial = link.find('inertial')
            mass = float(inertial.find('mass').get('value'))
            origin = inertial.find('origin')
            com = [float(x) for x in origin.get('xyz', '0 0 0').split()] if origin is not None else [0.0]*3
            values = candidate_moments.get(name)
            if values is None or len(values) != 3 or len(com) != 3 or not all(
                math.isclose(float(value), mass*coord, abs_tol=1e-9, rel_tol=1e-9)
                for value, coord in zip(values, com)):
                raise ValueError(f'locked Link first moments were modified: {name}')
    if result.get('core_fingerprint') != metadata.get('core_fingerprint') or result.get('urdf_fingerprint') != metadata.get('urdf_fingerprint'):
        raise ValueError('task result fingerprint does not match recorded source')
    moments = gravity.get('first_moments') or []
    if not moments:
        raise ValueError('candidate contains no first moments')
    first_moments = {item['link_name']: [float(x) for x in item['value']] for item in moments}
    candidate_masses = gravity.get('candidate_masses') if recomputed_used else None
    if recomputed_used:
        if not isinstance(candidate_masses, dict) or set(candidate_masses) != set(first_moments):
            raise ValueError('离线质量参数和一阶矩 Link 映射不一致')
        candidate_masses = {name: float(mass) for name, mass in candidate_masses.items()}
        if any(not math.isfinite(mass) or mass <= 0 for mass in candidate_masses.values()):
            raise ValueError('离线候选包含无效质量参数')
    friction = result.get('friction') or {}
    payload = {
        'schema': 'serial_arm_gravity_correction',
        'task_id': result['task_id'],
        'core_fingerprint': result['core_fingerprint'],
        'urdf_fingerprint': result['urdf_fingerprint'],
        'joint_names': joints,
        'static_pass': bool(gravity.get('static_pass')),
        'candidate_review_required': not bool(gravity.get('static_pass')),
        'candidate_quality_reason': gravity.get('failure_reason', ''),
        'friction_pass': friction_pass,
        'friction_matches_gravity': friction_matches,
        'friction_status': 'validated' if friction_pass else 'requires_refit_or_review',
        'candidate_source': source_id,
        'torque_bias_source': bias_source,
        'identification_mode': gravity.get('identification_mode', 'global'),
        'locked_links': lock_names,
        'fitted_links': fitted_names,
        'original_gravity_scale': result.get('original_gravity_scale') or metadata.get('original_gravity_scale'),
        'first_moments': first_moments,
        'constraints': {},
        'fit_options': {k:v for k,v in (metadata.get('calibration_options') or {}).items() if k != 'max_com_offset_m'},
        'validation': {
            'numerical_rank': gravity.get('numerical_rank'),
            'constraints_ok': gravity.get('constraints_ok'),
            'training_original': gravity.get('training_original'),
            'training_candidate': gravity.get('training_candidate'),
            'validation_original': gravity.get('validation_original'),
            'validation_scaled': gravity.get('validation_scaled'),
            'validation_candidate': gravity.get('validation_candidate'),
            'noise_rms': gravity.get('noise_rms'),
        },
        'residual': {
            'torque_bias': torque_bias,
            'torque_threshold': _candidate_threshold(gravity, joints),
            'friction': {
                'velocity_transition': 0.03,
                'positive_coulomb': friction.get('positive_coulomb', [0.0] * len(joints)) if friction_matches else [0.0] * len(joints),
                'positive_viscous': friction.get('positive_viscous', [0.0] * len(joints)) if friction_matches else [0.0] * len(joints),
                'negative_coulomb': friction.get('negative_coulomb', [0.0] * len(joints)) if friction_matches else [0.0] * len(joints),
                'negative_viscous': friction.get('negative_viscous', [0.0] * len(joints)) if friction_matches else [0.0] * len(joints),
            },
        },
        'source': {
            'core_config_path': metadata.get('core_config_path'),
            'urdf_path': metadata.get('urdf_path'),
            'resource_paths': metadata.get('resource_paths', []),
            'units': metadata.get('units', {}),
            'mapping': metadata.get('mapping', {}),
            'load_description': metadata.get('load_description', ''),
        },
    }
    if candidate_masses is not None:
        payload['candidate_masses'] = candidate_masses
    return directory, metadata, result, payload


def save_gravity_correction(core_path, expected_sha, task_directory, storage_home=None):
    core_path = Path(core_path).resolve()
    if sha256_file(core_path) != expected_sha:
        raise ValueError('configuration changed outside the workspace; reload before saving')
    directory, metadata, result, payload = build_gravity_correction_payload(task_directory)
    if not matches_recorded_fingerprint(metadata['urdf_path'], metadata['urdf_fingerprint']):
        raise ValueError('当前 URDF 与历史任务不一致，禁止应用参数，请先恢复对应模型')
    candidate_dir = directory / 'candidate'
    candidate_dir.mkdir(parents=True, exist_ok=True)
    correction_path = candidate_dir / 'gravity-correction.yaml'
    temp_correction = correction_path.with_suffix('.yaml.tmp')
    temp_correction.write_text(yaml.safe_dump(payload, allow_unicode=True, sort_keys=False))
    yaml.safe_load(temp_correction.read_text())
    temp_correction.replace(correction_path)
    relative = Path(os.path.relpath(correction_path, core_path.parent))
    before = core_path.read_text()
    after = _insert_or_replace_model_scalar(before, 'gravity_correction_path', relative.as_posix())
    yaml.safe_load(after)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    backup_dir = Path(storage_home).resolve() / '.serialarm' / 'backups' if storage_home else core_path.parent
    backup_dir.mkdir(parents=True, exist_ok=True)
    backup = backup_dir / (core_path.name + '.bak-' + stamp)
    shutil.copy2(core_path, backup)
    temp = core_path.with_name(core_path.name + '.tmp')
    temp.write_text(after)
    try:
        yaml.safe_load(temp.read_text())
        temp.replace(core_path)
        yaml.safe_load(core_path.read_text())
    except Exception:
        temp.unlink(missing_ok=True)
        raise
    application = {
        'saved_at': datetime.now().isoformat(timespec='seconds'),
        'core_path': str(core_path),
        'core_backup': str(backup),
        'gravity_correction_path': str(correction_path),
        'core_sha256': sha256_file(core_path),
    }
    (directory / 'application.json').write_text(json.dumps(application, ensure_ascii=False, indent=2))
    return application




def _identifiable_links_from_snapshot(directory, metadata):
    """Reconstruct link ownership for old datasets without treating fixed frames as masses"""
    import xml.etree.ElementTree as ET
    try:
        source = verified_source_urdf(directory, metadata)
        root = ET.parse(source).getroot()
    except (OSError, ValueError, ET.ParseError):
        return []
    controlled = set(metadata.get('joint_names') or [])
    link_nodes = {node.get('name'): node for node in root.findall('link')}
    found = []
    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        child = joint.find('child')
        if joint_name not in controlled or child is None:
            continue
        link_name = child.get('link')
        link = link_nodes.get(link_name)
        inertial = link.find('inertial') if link is not None else None
        mass_node = inertial.find('mass') if inertial is not None else None
        if mass_node is None:
            continue
        try:
            mass = float(mass_node.get('value', ''))
        except (ValueError, TypeError):
            continue
        if not math.isfinite(mass) or mass <= 0:
            continue
        found.append({'name': link_name, 'joint': joint_name, 'mass': mass})
    return found


def load_model_calibration_summary(directory):
    """Inspect a task without changing native state or opening any hardware.

    Result files are optional: a demonstration becomes replayable as soon as
    trajectory.csv has been written. Partial recordings remain inspectable.
    """
    if not directory:
        raise ValueError('请选择标定任务文件夹')
    folder = Path(directory).expanduser().resolve()
    metadata_path = folder / 'metadata.json'
    if not metadata_path.is_file():
        raise ValueError('所选目录不是标定任务：缺少 metadata.json')
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    task_id = metadata.get('task_id')
    joint_names = metadata.get('joint_names')
    if not isinstance(task_id, str) or not task_id or not isinstance(joint_names, list) or not joint_names or not all(isinstance(j, str) and j for j in joint_names) or len(set(joint_names)) != len(joint_names):
        raise ValueError('任务元数据缺少有效任务编号或关节列表')
    result_path = folder / 'result.json'
    result = None
    if result_path.is_file():
        result = json.loads(result_path.read_text(encoding='utf-8'))
        if result.get('task_id') != task_id:
            raise ValueError('标定结果与任务记录编号不匹配')
    complete_path = folder / 'trajectory.csv'
    checkpoint_path = folder / 'trajectory.checkpoint.csv'
    trajectory_path = complete_path if complete_path.is_file() else checkpoint_path
    trajectory = {'valid': False, 'samples': 0, 'duration_s': None, 'sample_dt_s': None,
                  'source': 'trajectory.csv' if complete_path.is_file() else
                            'trajectory.checkpoint.csv' if checkpoint_path.is_file() else ''}
    if trajectory_path.is_file():
        with trajectory_path.open(encoding='utf-8', newline='') as stream:
            reader = csv.reader(stream)
            header = next(reader, [])
            if len(header) != 2 or header[0] != 'sample_dt':
                raise ValueError('示教轨迹格式错误：sample_dt')
            dt = float(header[1])
            if not math.isfinite(dt) or not 0 < dt <= 0.5:
                raise ValueError('示教轨迹采样周期无效')
            if next(reader, []) != ['index', *joint_names]:
                raise ValueError('示教轨迹关节列与任务元数据不一致')
            count = 0
            for row in reader:
                if len(row) != len(joint_names) + 1 or row[0] != str(count):
                    raise ValueError(f'示教轨迹第 {count} 帧编号或列数无效')
                if not all(math.isfinite(float(v)) for v in row[1:]):
                    raise ValueError(f'示教轨迹第 {count} 帧存在非有限关节位置')
                count += 1
                if count > 300000:
                    raise ValueError('轨迹帧数超过安全读取上限')
            if count >= 20:
                trajectory = {'valid': True, 'samples': count, 'duration_s': (count - 1) * dt, 'sample_dt_s': dt, 'source': trajectory['source']}
    phase = str(result.get('phase', '')) if result else ''
    if result and phase == 'complete':
        record_kind = 'completed'
    elif result or (folder / 'interrupted.txt').is_file() or (trajectory['valid'] and trajectory['source'] == 'trajectory.checkpoint.csv'):
        record_kind = 'interrupted'
    elif trajectory['valid']:
        record_kind = 'teaching_only'
    else:
        record_kind = 'interrupted'
    can_resume = trajectory['valid']
    files = {'metadata.json': metadata_path.is_file(), 'frames.csv': (folder/'frames.csv').is_file(),
             'accepted_static.csv': (folder/'accepted_static.csv').is_file(),
             'trajectory.csv': complete_path.is_file(),
             'trajectory.checkpoint.csv': checkpoint_path.is_file(), 'result.json': result_path.is_file()}
    note = ('已完成标定，可查看结果或从原轨迹重新进行一次采集' if record_kind == 'completed' else
            '仅有示教轨迹，可重新校验后从头开始自动采集' if record_kind == 'teaching_only' else
            '此任务未完整结束；可用轨迹可从头重新采集，不能直接接着中断点运动' if can_resume else
            '未发现可用轨迹或检查点，只能查看记录，无法恢复自动回放')
    # Stored result is never a live session; the GUI must not infer that the
    # playback is ready until the native process explicitly imports it.
    display_result = dict(result) if result else {'phase': 'recorded_only' if can_resume else 'failed', 'task_id': task_id, 'joint_names': joint_names}
    if can_resume and trajectory['source'] == 'trajectory.checkpoint.csv':
        note = '仅保存了示教检查点，属于中断片段，必须重新校验运动范围和真机起点，再人工确认回放'
    selectable_links = _identifiable_links_from_snapshot(folder, metadata)
    offline_candidate = None
    recomputed = folder / 'recomputed-candidate.json'
    if recomputed.is_file():
        candidate = json.loads(recomputed.read_text(encoding='utf-8'))
        if (isinstance(candidate, dict)
                and candidate.get('task_id') == task_id
                and candidate.get('urdf_fingerprint') == metadata.get('urdf_fingerprint')
                and candidate.get('core_fingerprint') == metadata.get('core_fingerprint')
                and isinstance(candidate.get('first_moments'), list)
                and isinstance(candidate.get('candidate_masses'), dict)):
            offline_candidate = candidate
    return {'directory': str(folder), 'metadata': metadata, 'result': display_result,
            'recomputed_candidate': offline_candidate,
            'identifiable_links': selectable_links,
            'has_result': bool(result), 'record_kind': record_kind, 'can_resume': can_resume,
            'files': files, 'trajectory': trajectory, 'note': note, 'task_id': task_id}


def list_model_calibration_records(root, limit=30, *, legacy_root=None, profile_name=None):
    """Read-only inventory, always derived from disk rather than cached GUI state"""
    root = Path(root)
    parents = [root / '.serialarm' / 'model-calibration']
    if legacy_root is not None:
        parents.append(Path(legacy_root) / '.install' / 'model-calibration')
    else:
        parents.append(root / '.install' / 'model-calibration')
    directories = [folder for parent in parents if parent.is_dir() for folder in parent.iterdir()
                   if folder.is_dir() and folder.name.startswith('model-calibration-')]
    records = []
    for folder in sorted(directories, key=lambda x: x.stat().st_mtime, reverse=True)[:limit]:
        try:
            record = load_model_calibration_summary(folder)
            recorded_profile = (record.get('metadata') or {}).get('profile')
            if profile_name and recorded_profile and recorded_profile != profile_name:
                continue
            records.append(record)
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            records.append({'directory': str(folder.resolve()), 'task_id': folder.name, 'can_resume': False,
                            'record_kind': 'invalid', 'trajectory': {'valid': False, 'samples': 0},
                            'files': {name: (folder/name).is_file() for name in
                                      ('metadata.json', 'frames.csv', 'trajectory.csv', 'trajectory.checkpoint.csv', 'result.json')},
                            'note': f'记录校验未通过: {exc}'})
    return records


def preview_gravity_correction(core_path, task_directory):
    core_path = Path(core_path).resolve()
    directory, metadata, result, payload = build_gravity_correction_payload(task_directory)
    candidate_path = directory / 'candidate' / 'gravity-correction.yaml'
    relative = Path(os.path.relpath(candidate_path, core_path.parent)).as_posix()
    before = core_path.read_text()
    after = _insert_or_replace_model_scalar(before, 'gravity_correction_path', relative)
    yaml.safe_load(after)
    changes = []
    before_lines = before.splitlines()
    after_lines = after.splitlines()
    for index in range(max(len(before_lines), len(after_lines))):
        left = before_lines[index] if index < len(before_lines) else ''
        right = after_lines[index] if index < len(after_lines) else ''
        if left != right:
            changes.append({'before': left.strip(), 'after': right.strip()})
    return {
        'path': str(core_path),
        'sha256': sha256_file(core_path),
        'candidate_path': str(candidate_path),
        'relative_candidate_path': relative,
        'changes': changes,
        'changed': before != after,
        'task_id': result.get('task_id', ''),
        'static_pass': bool(payload['static_pass']),
        'friction_pass': bool(payload['friction_pass']),
        'friction_status': payload.get('friction_status', 'requires_refit_or_review'),
        'candidate_source': payload.get('candidate_source', 'result.json'),
    }


def restore_gravity_correction(core_path, task_directory):
    core_path = Path(core_path).resolve()
    directory = Path(task_directory).resolve()
    application_path = directory / 'application.json'
    if not application_path.is_file():
        raise ValueError('model calibration application record is missing')
    application = json.loads(application_path.read_text())
    if Path(application.get('core_path', '')).resolve() != core_path:
        raise ValueError('application record belongs to another core configuration')
    if sha256_file(core_path) != application.get('core_sha256'):
        raise ValueError('configuration changed after candidate save; reload before restoring')
    backup = Path(application.get('core_backup', '')).resolve()
    if not backup.is_file():
        raise ValueError('original configuration backup is missing')
    restored = backup.read_text()
    yaml.safe_load(restored)
    temp = core_path.with_name(core_path.name + '.restore.tmp')
    temp.write_text(restored)
    try:
        yaml.safe_load(temp.read_text())
        temp.replace(core_path)
        yaml.safe_load(core_path.read_text())
    except Exception:
        temp.unlink(missing_ok=True)
        raise
    restore_record = {
        'restored_at': datetime.now().isoformat(timespec='seconds'),
        'core_path': str(core_path),
        'restored_from': str(backup),
        'core_sha256': sha256_file(core_path),
    }
    (directory / 'restore.json').write_text(json.dumps(restore_record, ensure_ascii=False, indent=2))
    return restore_record


def update_candidate_urdf_verification(target_dir, verification):
    target_dir = Path(target_dir).resolve()
    path = target_dir / 'candidate-urdf-verification.json'
    existing = {}
    if path.is_file():
        existing = json.loads(path.read_text())
    existing.update(verification)
    path.write_text(json.dumps(existing, ensure_ascii=False, indent=2))
    return existing


def export_friction_candidate(task_directory, destination=None):
    """Produce a review-only friction artifact, tied to the gravity model it was fitted against

    Unlike gravity-correction.yaml this is not a runtime application package
    """
    directory, metadata, result = _load_model_calibration_task(task_directory)
    fitted_gravity = result.get('gravity_result') or {}
    original_friction = result.get('friction') or {}
    if not original_friction:
        raise ValueError('task has no completed friction fit')
    _, _, _, active = build_gravity_correction_payload(directory, allow_unvalidated=True)
    joint_names = result.get('joint_names') or metadata.get('joint_names') or []
    if not joint_names:
        raise ValueError('friction candidate is missing sampled joints')
    verified = bool(result.get('friction_pass'))
    data = {
        'schema': 'serial_arm_friction_candidate_review',
        'task_id': result.get('task_id'),
        'core_fingerprint': metadata.get('core_fingerprint'),
        'urdf_fingerprint': metadata.get('urdf_fingerprint'),
        'joint_names': joint_names,
        'gravity_fit_source': 'result.json',
        'gravity_fit_fingerprint': hashlib.sha256(json.dumps({
            'first_moments': fitted_gravity.get('first_moments'),
            'torque_bias': fitted_gravity.get('torque_bias')}, sort_keys=True).encode()).hexdigest(),
        'matches_current_candidate': bool(active['friction_matches_gravity']),
        'friction_validation_pass': verified,
        'can_apply_directly': False,
        'warning': '人工审核文件，不可直接作为 Core 运行配置，如重算过重力模型须重新匹配摩擦残差',
        'torque_bias': fitted_gravity.get('torque_bias'),
        'positive_coulomb': original_friction.get('positive_coulomb'),
        'negative_coulomb': original_friction.get('negative_coulomb'),
        'positive_viscous': original_friction.get('positive_viscous'),
        'negative_viscous': original_friction.get('negative_viscous'),
    }
    dest = Path(destination).expanduser().resolve() if destination else directory / 'candidate' / 'friction-candidate.yaml'
    dest.parent.mkdir(parents=True, exist_ok=True)
    temporary = dest.with_suffix(dest.suffix + '.tmp')
    temporary.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
    yaml.safe_load(temporary.read_text(encoding='utf-8'))
    temporary.replace(dest)
    return {'path': str(dest), 'friction_validation_pass': verified,
            'matches_current_candidate': data['matches_current_candidate'],
            'can_apply_directly': False, 'gravity_fit_source': 'result.json'}

def export_candidate_urdf(task_directory, destination=None):
    import xml.etree.ElementTree as ET
    directory, metadata, result, payload = build_gravity_correction_payload(task_directory, allow_unvalidated=True, prefer_recomputed=False)
    source = verified_source_urdf(directory, metadata)
    target_dir = Path(destination).expanduser().resolve() / 'online' if destination else directory / 'candidate' / 'online'
    target_dir.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(source)
    root = tree.getroot()
    moments = {name: values for name, values in payload['first_moments'].items()
               if name in (payload.get('fitted_links') or payload['first_moments'].keys())}
    changed = []
    skipped_massless_links = []
    review_warnings = []
    def local_name(node):
        return node.tag.rsplit('}', 1)[-1]
    links = [node for node in root.iter() if local_name(node) == 'link']
    for link in links:
        name = link.get('name')
        if name not in moments:
            continue
        inertial = next((node for node in list(link) if local_name(node) == 'inertial'), None)
        if inertial is None:
            # A URDF frame such as tool0 is allowed to have no inertia at all
            # It is a coordinate frame, not an identifiable rigid-body COM
            skipped_massless_links.append(name)
            if any(abs(float(v)) > 1e-9 for v in moments[name]):
                review_warnings.append(f'{name}: 无惯量的坐标系存在非零拟合一阶矩，已保留原始 Link 并跳过反写，请人工核查')
            continue
        mass_node = next((node for node in list(inertial) if local_name(node) == 'mass'), None)
        if mass_node is None:
            raise ValueError(f'candidate link has no mass: {name}')
        mass = float(mass_node.get('value'))
        if not math.isfinite(mass):
            raise ValueError(f'candidate link mass is non-finite: {name}')
        if mass == 0:
            # A zero-mass inertial placeholder must remain a placeholder
            skipped_massless_links.append(name)
            if any(abs(float(v)) > 1e-9 for v in moments[name]):
                review_warnings.append(f'{name}: 零质量坐标系存在非零拟合一阶矩，已跳过反写，请人工核查')
            continue
        if not mass > 0:
            raise ValueError(f'candidate link mass is invalid: {name}')
        com = [float(value) / mass for value in moments[name]]
        if not all(math.isfinite(value) for value in com):
            raise ValueError(f'candidate link COM is non-finite: {name}')
        origin = next((node for node in list(inertial) if local_name(node) == 'origin'), None)
        if origin is None:
            namespace = inertial.tag[:-len('inertial')] if inertial.tag.endswith('inertial') else ''
            origin = ET.Element(namespace + 'origin')
            inertial.insert(0, origin)
        origin.set('xyz', ' '.join(f'{value:.12g}' for value in com))
        changed.append(name)
    if not changed:
        raise ValueError('candidate URDF did not match any calibrated link')
    resources = target_dir / 'resources'
    copied = []
    for mesh in [node for node in root.iter() if local_name(node) == 'mesh']:
        filename = mesh.get('filename', '')
        if not filename or filename.startswith('package://') or Path(filename).is_absolute():
            continue
        src = resolve_relative_mesh(filename, source, metadata)
        if src is None:
            raise ValueError(f'缺少候选 URDF 引用的网格资源: {filename}')
        resources.mkdir(parents=True, exist_ok=True)
        dst = resources / f'{len(copied):03d}-{src.name}'
        shutil.copy2(src, dst)
        mesh.set('filename', dst.relative_to(target_dir).as_posix())
        copied.append(str(dst))
    candidate = target_dir / 'candidate-online.urdf'
    temp = target_dir / 'candidate-online.urdf.tmp'
    tree.write(temp, encoding='utf-8', xml_declaration=True)
    ET.parse(temp)
    temp.replace(candidate)
    verification = {
        'source_urdf': str(source),
        'candidate_urdf': str(candidate),
        'changed_inertial_origins': changed,
        'skipped_massless_links': skipped_massless_links,
        'review_warnings': review_warnings,
        'mass_and_inertia_values_preserved': True,
        'locked_links_preserved': payload.get('locked_links', []),
        'identification_mode': payload.get('identification_mode', 'global'),
        'candidate_source': payload.get('candidate_source', 'result.json'),
        'friction_status': payload.get('friction_status', 'requires_refit_or_review'),
        'joint_geometry_modified': False,
        'copied_relative_meshes': copied,
        'scope': 'static gravity correction only; original masses and inertias unchanged',
        'review_required': True,
        'holdout_pass': payload['static_pass'],
        'quality_reason': payload['candidate_quality_reason'],
        'candidate_solver': (result.get('gravity_result') or {}).get('solver', ''),
    }
    (target_dir / 'candidate-urdf-verification.json').write_text(json.dumps(verification, ensure_ascii=False, indent=2))
    return verification
