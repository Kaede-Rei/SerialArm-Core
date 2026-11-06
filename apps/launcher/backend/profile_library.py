from __future__ import annotations

import hashlib
import json
import time
import copy
import math
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET
import zipfile
import subprocess
import sys

import yaml

from profiles import Inspector, config_values, read_yaml, text
from control_parameters import canonical_core, four_mode_core, four_mode_control_text, effective_hardware
from gravity_recovery_validation import gravity_validation_context, gravity_validation_changes, gravity_trusted


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _atomic_yaml(path, data):
    _atomic_yaml_text(path, yaml.safe_dump(data, allow_unicode=True, sort_keys=False))


def _atomic_yaml_text(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(text, encoding='utf-8')
    yaml.safe_load(temp.read_text())
    temp.replace(path)


def _safe_extract(source, destination):
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(source) as archive:
        members = archive.infolist()
        if len(members) > 10000 or sum(m.file_size for m in members) > 512 * 1024 * 1024:
            raise ValueError('description archive exceeds extraction limits')
        for member in members:
            if member.file_size > 64 * 1024 * 1024 or (member.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError('description archive contains an oversized file or symbolic link')
            target = (destination / member.filename).resolve()
            if destination not in target.parents and target != destination:
                raise ValueError('description archive contains an unsafe path')
            if member.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.open(member) as src, target.open('wb') as dst:
                    shutil.copyfileobj(src, dst)


def _rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ]


def _matmul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _origin_matrix(origin):
    r = _rpy_matrix(origin.get('rpy', [0, 0, 0])); x, y, z = origin.get('xyz', [0, 0, 0])
    return [[r[0][0],r[0][1],r[0][2],x],[r[1][0],r[1][1],r[1][2],y],[r[2][0],r[2][1],r[2][2],z],[0,0,0,1]]


def _quat_from_matrix(m):
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2; w = .25*s; x=(m[2][1]-m[1][2])/s; y=(m[0][2]-m[2][0])/s; z=(m[1][0]-m[0][1])/s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1+m[0][0]-m[1][1]-m[2][2])*2; w=(m[2][1]-m[1][2])/s; x=.25*s; y=(m[0][1]+m[1][0])/s; z=(m[0][2]+m[2][0])/s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1+m[1][1]-m[0][0]-m[2][2])*2; w=(m[0][2]-m[2][0])/s; x=(m[0][1]+m[1][0])/s; y=.25*s; z=(m[1][2]+m[2][1])/s
    else:
        s = math.sqrt(1+m[2][2]-m[0][0]-m[1][1])*2; w=(m[1][0]-m[0][1])/s; x=(m[0][2]+m[2][0])/s; y=(m[1][2]+m[2][1])/s; z=.25*s
    return [x,y,z,w]


class ProfileLibrary:
    def __init__(self, root, inspector):
        self.root = Path(root).resolve()
        self.inspector = inspector
        self.library_file = Path(os.environ.get('SERIAL_ARM_PROFILE_LIBRARY', Path.home() / '.config/serial-arm/profile-library.yaml')).expanduser()
        self.cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'serial-arm/description-preview'
        self._gravity_validation_baselines = {}

    def _library_data(self):
        if not self.library_file.is_file(): return {'profiles': {}, 'hidden_builtin': []}
        data = yaml.safe_load(self.library_file.read_text()) or {}
        if not isinstance(data, dict): raise ValueError('Profile Library YAML must contain a mapping')
        profiles = data.get('profiles') or {}
        if not isinstance(profiles, dict): raise ValueError('Profile Library profiles must be a mapping')
        hidden = data.get('hidden_builtin') or []
        if not isinstance(hidden, list): hidden = []
        return {'profiles': profiles, 'hidden_builtin': hidden}

    def _write_library(self, data):
        _atomic_yaml(self.library_file, {'profiles': data.get('profiles', {}), 'hidden_builtin': sorted(set(data.get('hidden_builtin', [])))})

    def _readiness_path(self, profile_file, profile):
        # Profile-specific state, including installed built-ins, is never shared
        # between unrelated robot identities
        from profile_storage import profile_home
        return profile_home({'profile': profile, 'profile_file': profile_file}, self.root) / '.serialarm' / 'readiness' / 'checks.yaml'

    _STAGES = ('model_check', 'bringup', 'mapping_calibration', 'geometry_calibration',
               'gravity_verification', 'dynamics_calibration', 'validation')
    _MODES = ('RIGID_HOLD', 'RIGID_TRACKING', 'COMPLIANT_HOLD',
              'COMPLIANT_DRAG', 'COMPLIANT_TRACKING', 'SERVO_TRACKING')
    _MODE_KEYS = {name: name.lower() for name in _MODES}
    _GAIN_PURPOSES = {
        'RIGID': ('刚性', ('rigid_hold', 'rigid_tracking'), ('tracking', 'hold', 'hold_compensation')),
        'COMPLIANT': ('柔性', ('compliant_hold', 'compliant_tracking'), ('tracking', 'hold')),
        'DRAG': ('拖拽', ('compliant_drag',), ('drag',)),
        'SERVO_TRACKING': ('伺服跟踪', ('servo_tracking',), ('native_tracking', 'admittance_tracking')),
    }
    _GAIN_CHECK_LABELS = {'tracking': '轨迹跟踪', 'hold': '定位保持',
        'hold_compensation': '保持补偿状态', 'drag': '拖拽稳定性',
        'native_tracking': '原生阻抗跟踪', 'admittance_tracking': '模型导纳跟踪'}

    @staticmethod
    def _native_file_fingerprint(path):
        # Match the native calibration FNV-1a implementation exactly
        hash_value = 1469598103934665603
        with Path(path).open('rb') as stream:
            while True:
                block = stream.read(4096)
                if not block: break
                for value in block:
                    hash_value = ((hash_value ^ value) * 1099511628211) & 0xffffffffffffffff
        return f'{hash_value:016x}'

    @staticmethod
    def _digest(*values):
        return hashlib.sha256(json.dumps(values, sort_keys=True, ensure_ascii=False,
                                         separators=(',', ':'), default=str).encode()).hexdigest()

    def _dependencies(self, info, core, hardware):
        core = canonical_core(core)
        model = core.get('model') or {}
        description_path = Path(info['resources'].get('description', ''))
        urdf_path = (Path(info['resources']['core']).parent / model.get('urdf_path', '')).resolve()
        def content_digest(path):
            return _sha(path) if path.is_file() and path.is_file() else ''
        # Scope hashes follow the actual resource dependencies rather than one global fingerprint
        source = self._digest(info['profile'], str(info['profile_file']),
                              content_digest(description_path), content_digest(urdf_path),
                              model.get('joint_names'), model.get('base_frame'), model.get('tool_frame'))
        calibration = core.get('calibration') or {}
        runtime = (core.get('control') or {}).get('runtime') or {}
        safety = core.get('safety_policy') or {}
        controller = (core.get('control') or {}).get('controller') or {}
        shutdown = core.get('shutdown') or {}
        capability = core.get('capability') or {}
        hardware_hash = self._digest(info.get('hardware_plugin'), effective_hardware(hardware, info.get('connection_overrides')), info.get('devices'))
        model_chain = {key: value for key, value in model.items() if key not in ('gravity_scale', 'gravity_correction_path')}
        model_hash = self._digest(source, model_chain)
        startup_hash = self._digest(model_hash, hardware_hash, calibration,
                                    safety.get('max_kp_override'), safety.get('max_kd_override'),
                                    controller.get('rigid_hold'))
        mapping_hash = self._digest(source, hardware_hash, calibration)
        geometry_hash = self._digest(source, calibration)
        correction_path = model.get('gravity_correction_path') or ''
        correction_sha = content_digest((Path(info['resources']['core']).parent / correction_path).resolve()) if correction_path else ''
        gravity_hash = self._digest(mapping_hash, model_hash, model.get('gravity'), model.get('gravity_scale'),
                                    model.get('gravity_correction_path'), correction_sha,
                                    controller.get('compliant_drag'), controller.get('rigid_hold'))
        dynamics_hash = self._digest(model_hash, mapping_hash, model.get('gravity'),
                                     controller.get('servo_tracking', controller.get('identification_tracking')), capability)
        scopes = {
            'model_check': model_hash, 'bringup': self._digest(hardware_hash, runtime, safety),
            'mapping_calibration': mapping_hash, 'geometry_calibration': geometry_hash,
            'gravity_verification': gravity_hash, 'dynamics_calibration': dynamics_hash,
            'validation': self._digest(mapping_hash, geometry_hash, safety, shutdown,
                                       controller.get('rigid_hold'), controller.get('rigid_tracking')),
            'startup_hold': startup_hash,
        }
        scopes['modes'] = {name: self._digest(model_hash, mapping_hash,
            safety.get('max_kp_override'), safety.get('max_kd_override'),
            (controller.get('servo_tracking', controller.get('identification_tracking', controller.get('rigid_tracking')))
             if name == 'SERVO_TRACKING' else controller.get(self._MODE_KEYS[name]))) for name in self._MODES}
        scopes['gain_purposes'] = {purpose: self._digest(model_hash, mapping_hash, gravity_hash,
            {key: runtime.get(key) for key in ('model_feedforward_mode', 'ctrl_frequency_hz', 'joint_acc_filter_alpha')},
            self._gain_interaction_context(core, purpose),
            safety.get('max_kp_override'), safety.get('max_kd_override'),
            safety.get('max_effort_override'),
            {name: controller.get(name) for name in gain_keys},
            (core.get('control') or {}).get('hold_compensation', {'enabled': False}) if purpose == 'RIGID' else None)
            for purpose, (_, gain_keys, _) in self._GAIN_PURPOSES.items()}
        return scopes

    def _readiness_meta(self, info, scopes=None):
        from profile_storage import legacy_profile_homes
        path = self._readiness_path(info['profile_file'], info['profile'])
        safe = ''.join(c if c.isalnum() or c in '_-' else '_' for c in str(info['profile'])) or 'profile'
        old_path = (self.library_file.parent / 'readiness' / f'{safe}.yaml') if str(info['profile_file']).startswith(str(self.root / 'src/robot_supports/profiles')) else (Path(info['profile_file']).parent / '.serialarm' / 'readiness' / f'{safe}.yaml')
        candidates = [(path, True), (old_path, True)]
        candidates += [(home / '.serialarm' / 'readiness' / 'checks.yaml', False)
                       for home in legacy_profile_homes(info, self.root)]
        meta = {}
        imported = False
        for source, named_owner in dict.fromkeys(candidates):
            try:
                record = yaml.safe_load(source.read_text()) if source.is_file() else {}
            except (OSError, UnicodeError, yaml.YAMLError):
                continue
            if not isinstance(record, dict):
                continue
            owner = record.get('profile')
            if owner and owner != info['profile']:
                continue
            if named_owner or owner == info['profile']:
                selected = record
            else:
                # Old shared files lack an owner. Import only entries whose
                # existing dependency scope proves this Profile owns them.
                selected = {}
                for category in ('stages', 'modes', 'gain_purposes'):
                    entries = record.get(category) or {}
                    for key, item in entries.items() if isinstance(entries, dict) else ():
                        expected = ((scopes or {}).get(key) if category == 'stages' else
                            (scopes or {}).get('startup_hold') if category == 'modes' and key == 'RIGID_HOLD' else
                            ((scopes or {}).get(category) or {}).get(key))
                        if expected and isinstance(item, dict) and item.get('scope') == expected:
                            selected.setdefault(category, {})[key] = item
            for category in ('stages', 'modes', 'gain_purposes'):
                entries = selected.get(category) or {}
                if isinstance(entries, dict):
                    for key, item in entries.items():
                        if source != path and key not in (meta.get(category) or {}):
                            imported = True
                        meta.setdefault(category, {}).setdefault(key, item)
            if source == path:
                for key, value in record.items():
                    if key not in ('stages', 'modes', 'gain_purposes') and key not in meta:
                        meta[key] = value
        if meta:
            meta['profile'] = info['profile']
            if imported:
                _atomic_yaml(path, meta)
        return path, meta

    def _is_official_profile(self, info):
        # Identity must come from the actual built-in profile catalogue, never
        # from a user-supplied display name or a claimed source flag
        try:
            catalogue = self.inspector.profiles({})
            return (Path(info['profile_file']).resolve() == Path(catalogue['profile_file']).resolve()
                    and info['profile'] in {item['name'] for item in catalogue['profiles']})
        except (OSError, ValueError, KeyError, TypeError):
            return False

    def _readiness(self, config):
        try:
            info = self.inspector.inspect(config)
        except Exception as error:
            return {'state': 'configuring', 'next_stage': 'profile_setup', 'checks': {},
                    'modes': {}, 'reason': str(error), 'fingerprint': ''}
        checks = {item['name']: bool(item['ok']) for item in info.get('checks', [])}
        try:
            core = canonical_core(read_yaml(info['resources']['core'])) if checks.get('core') else {}
        except (ValueError, TypeError, KeyError) as error:
            return {'state': 'configuring', 'verification_state': 'pending', 'next_stage': 'profile_setup',
                    'checks': {}, 'modes': {}, 'gain_purposes': {}, 'control_available': False,
                    'reason': str(error), 'setup_issues': [self._field_issue(str(error))],
                    'next_actions': [self._field_issue(str(error))['action']], 'fingerprint': info.get('fingerprint', '')}
        try:
            hardware = read_yaml(info['resources']['hardware']) if checks.get('hardware') else {}
        except (OSError, ValueError, yaml.YAMLError):
            hardware = {}
            checks['hardware'] = False
        model = core.get('model') or {}
        joints = model.get('joint_names') or []
        profile_setup = all(checks.get(k, False) for k in ('core', 'description', 'hardware'))
        profile_setup &= bool(info.get('hardware_plugin') and joints and model.get('base_frame') and model.get('tool_frame'))
        setup_issues = []
        if checks.get('core'):
            try:
                entry = (read_yaml(info['profile_file']).get('profiles') or {})[info['profile']]
                setup_issues = self._setup_issues(core, hardware, entry)
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
                pass
        if profile_setup and info['hardware_plugin'] == 'serial_arm_hardware_damiao':
            damiao = hardware.get('damiao') or {}
            acts = damiao.get('actuators') or {}
            buses = hardware.get('buses') or {}
            ids = [a.get('motor_id') for a in acts.values() if isinstance(a, dict)]
            profile_setup = (set(acts) == set(joints) and bool(damiao.get('bus') in buses)
                and all(isinstance(a, dict) and a.get('motor_type') and
                    isinstance(a.get('motor_id'), int) and not isinstance(a.get('motor_id'), bool)
                    and a['motor_id'] > 0 for a in acts.values()) and len(set(ids)) == len(joints))
        if setup_issues: profile_setup = False
        reason = setup_issues[0]['message'] if setup_issues else ''
        # Offline simulation needs a valid Core model, not a made-up motor type.
        core_issues = [item for item in setup_issues if item.get('group') != 'hardware']
        model_valid = bool(checks.get('core') and checks.get('description'))
        if model_valid:
            try:
                Inspector.validate_urdf((Path(info['resources']['core']).parent / model['urdf_path']).resolve(), joints)
            except (ValueError, OSError, KeyError) as error:
                model_valid = False
                core_issues.append(self._field_issue(str(error)))
        offline_ready = bool(checks.get('core') and checks.get('description') and joints and
                             model.get('base_frame') and model.get('tool_frame'))
        if offline_ready and not core_issues:
            try:
                self._validate_core_profile(core, {}, {'hardware': {'plugin': ''}}, Path(info['resources']['core']), core)
                check_model = getattr(self.inspector, 'validate_model_config', None)
                checked = check_model(config, info['resources']['core'],
                    (Path(info['resources']['core']).parent / model['urdf_path']).resolve()) if check_model else None
                if isinstance(checked, dict) and checked.get('valid') is not True:
                    model_valid = False
                    raise ValueError(checked['error'])
            except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as error:
                core_issues.append(self._field_issue(str(error)))
        offline_ready = offline_ready and not core_issues
        for issue in core_issues:
            if issue not in setup_issues: setup_issues.append(issue)
        if core_issues:
            profile_setup = False
            reason = core_issues[0]['message']
        if profile_setup and isinstance(self.inspector, Inspector):
            driver = self.inspector.hardware_driver(info['hardware_plugin'])
            if not driver['installed']:
                profile_setup = False
                missing_driver = self._field_issue('profile_entry.hardware.plugin ' + driver['message'])
                setup_issues.append(missing_driver)
                reason = missing_driver['message']
        if profile_setup:
            try:
                entry = (read_yaml(info['profile_file']).get('profiles') or {})[info['profile']]
                self._validate_core_profile(core, hardware, entry, Path(info['resources']['core']), core)
            except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as error:
                profile_setup = False
                reason = str(error)
                setup_issues.append(self._field_issue(reason))
        scopes = self._dependencies(info, core, hardware) if checks.get('core') else {}
        _, meta = self._readiness_meta(info, scopes)
        stages = meta.get('stages') or {}
        modes = meta.get('modes') or {}
        def status(item, scope):
            if not isinstance(item, dict) or item.get('result') not in ('passed', 'recorded', 'warning'):
                return 'pending'
            if item.get('result') == 'warning' and item.get('kind') != 'measured_gain_purpose':
                return 'pending'
            if item.get('scope') != scope:
                return 'stale'
            # Existing historical checklist entries are still user reports,
            # even if they were recorded while a real robot was connected.
            # Older native gravity samples include a true Machine proof; do
            # not demote them to self-check notes when upgrading the UI.
            if (item.get('kind') == 'operator_and_live' and
                    (item.get('runtime') or {}).get('sample_count', 0) >= 3):
                return 'verified'
            if item.get('result') == 'recorded' or str(item.get('kind', '')).startswith('operator_'):
                observations = item.get('observations') or {}
                if observations and all(v is True for v in observations.values()):
                    return 'recorded'
                # Saving an empty checklist must not look like an inspection
                return 'partial' if any(v is True for v in observations.values()) or str(item.get('note') or '').strip() else 'pending'
            candidate_path = item.get('candidate_path')
            if candidate_path:
                path = Path(candidate_path)
                if not path.is_file() or _sha(path) != item.get('candidate_sha256'):
                    return 'stale'
            return 'warning' if item.get('result') == 'warning' else 'verified'
        values = {
            'description': 'verified' if model_valid else 'failed',
            'profile_setup': 'verified' if profile_setup else 'pending',
        }
        # Independent evidence describes what was checked, not what may be opened
        for key in self._STAGES:
            values[key] = status(stages.get(key), scopes.get(key))
        hold = status(modes.get('RIGID_HOLD'), scopes.get('startup_hold'))
        mode_states = {}
        controller = (core.get('control') or {}).get('controller') or {}
        safety = core.get('safety_policy') or {}
        for name in self._MODES:
            key = self._MODE_KEYS[name]
            gains = (controller.get('servo_tracking', controller.get('identification_tracking', controller.get('rigid_tracking')))
                     if name == 'SERVO_TRACKING' else controller.get(key)) or {}
            valid = (bool(joints) and all(
                isinstance(gains.get(axis), dict) and set(gains[axis]) == set(joints)
                and all(isinstance(gains[axis][j], (int, float)) and not isinstance(gains[axis][j], bool)
                        and math.isfinite(gains[axis][j]) and gains[axis][j] >= 0 and
                        gains[axis][j] <= (safety.get('max_' + axis + '_override') or {}).get(j, float('inf'))
                        for j in joints) for axis in ('kp', 'kd')))
            if valid and name == 'RIGID_HOLD':
                valid = all(gains['kp'][j] > 0 and gains['kd'][j] > 0 for j in joints)
            if valid and name == 'COMPLIANT_DRAG':
                valid = all(gains['kd'][j] > 0 for j in joints)
            scope = scopes.get('startup_hold') if name == 'RIGID_HOLD' else (scopes.get('modes') or {}).get(name)
            own = status(modes.get(name), scope)
            mode_states[name] = (own if own in ('verified', 'warning', 'stale') else 'configured') if valid else 'unconfigured'
        required = ('description', 'profile_setup', *self._STAGES)
        missing = [k for k in required if values[k] != 'verified']
        stale = any(v == 'stale' for v in list(values.values()) + list(mode_states.values()))
        # Hardware use depends on structurally valid Core, URDF, hardware and Safety
        # settings, not the provenance of a Profile or optional acceptance notes
        accel = safety.get('max_acc') or {}
        safety_configured = (isinstance(accel, dict) and set(accel) == set(joints)
            and bool(joints) and all(isinstance(accel[j], (int, float))
                and not isinstance(accel[j], bool) and math.isfinite(accel[j])
                and accel[j] > 0 for j in joints))
        values['safety_policy'] = 'verified' if safety_configured and profile_setup else 'unconfigured'
        if profile_setup and not safety_configured:
            profile_setup = False
            values['profile_setup'] = 'pending'
            reason = 'Safety 必须为各关节配置正值 max_acc，才能启用真实硬件'
        verification_state = ('stale' if stale else 'verified' if not missing else
                              'partial' if any(v in ('recorded', 'verified', 'partial') for k, v in values.items() if k in self._STAGES) else 'pending')
        state = 'ready' if profile_setup else 'configuring'
        gain_purposes = self._gain_purpose_status(core, meta, scopes, mode_states)
        next_actions = [issue['action'] for issue in setup_issues]
        if not next_actions:
            next_actions = [{'page': 'workbench', 'label': '进入控制工作台'}] if profile_setup else [{'page': 'profile_setup', 'label': '补齐机器人配置'}]
        urdf_limits = {}
        if offline_ready:
            try:
                urdf = (Path(info['resources']['core']).parent / model['urdf_path']).resolve()
                joint_xml = {item.get('name'): item for item in ET.parse(urdf).getroot().findall('joint')}
                for joint in joints:
                    node = joint_xml.get(joint)
                    limit = node.find('limit') if node is not None else None
                    urdf_limits[joint] = {
                        'lower': limit.get('lower') if limit is not None else None,
                        'upper': limit.get('upper') if limit is not None else None,
                        'velocity': limit.get('velocity') if limit is not None else None,
                    }
            except (ET.ParseError, OSError, KeyError, TypeError, ValueError):
                urdf_limits = {}
        position_ranges = []
        for joint in joints:
            limits = urdf_limits.get(joint) or {}
            try: low, high = float(limits['lower']), float(limits['upper'])
            except (KeyError, TypeError, ValueError): low, high = None, None
            position_ranges.append((low, high))
        safety_summary = {
            'joint_names': joints, 'max_acc': accel if isinstance(accel, dict) else {},
            'position_margin': safety.get('position_margin'),
            'cmd_vel_scale': safety.get('cmd_vel_scale'),
            'state_vel_scale': safety.get('state_vel_scale'),
            'max_dt_s': safety.get('max_dt_s'),
            'state_timeout_s': safety.get('state_timeout_s'),
            'cmd_timeout_s': safety.get('cmd_timeout_s'),
            'require_continuous_cmd': safety.get('require_continuous_cmd'),
            'fault_recovery': dict(safety.get('fault_recovery') or {}),
            'urdf_limits': urdf_limits,
            'min_pos': [item[0] for item in position_ranges],
            'max_pos': [item[1] for item in position_ranges],
            'pos_margin': [safety.get('position_margin', 0)] * len(joints),
            'has_position_limit': [all(value is not None and math.isfinite(value) for value in item) for item in position_ranges],
        }
        runtime = ((core.get('control') or {}).get('runtime') or {})
        prerequisites = []
        def prerequisite(key, label, ok, field, group):
            prerequisites.append({'id': key, 'label': label, 'ready': bool(ok),
                'action': {'page': 'profile_setup', 'group': group, 'field': field, 'label': '配置' + label}})
        prerequisite('write_enabled', '控制写入', runtime.get('write_enabled') is True,
            'core.control.runtime.write_enabled', 'motion')
        drag = controller.get('compliant_drag') or {}
        kd = drag.get('kd') or {}
        prerequisite('drag_damping', '拖拽阻尼', bool(joints) and all(
            isinstance(kd.get(j), (int, float)) and not isinstance(kd.get(j), bool) and
            math.isfinite(kd[j]) and kd[j] > 0 for j in joints),
            'core.control.controller.compliant_drag.kd', 'motion')
        gravity = model.get('gravity')
        scales = model.get('gravity_scale') or {}
        gravity_ready = (isinstance(gravity, list) and len(gravity) == 3 and
            all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in gravity) and
            sum(v*v for v in gravity) > 1e-12 and bool(joints) and
            all(isinstance(scales.get(j), (int, float)) and not isinstance(scales.get(j), bool) and
                math.isfinite(scales[j]) and scales[j] > 0 for j in joints))
        prerequisite('gravity', '重力模型与补偿比例', gravity_ready,
            'core.model.gravity' if not isinstance(gravity, list) or len(gravity) != 3 or not gravity_ready and not any(gravity or []) else 'core.model.gravity_scale', 'model')
        prerequisite('hold_gains', '刚性保持增益', mode_states.get('RIGID_HOLD') != 'unconfigured',
            'core.control.controller.rigid_hold.kp', 'motion')
        calibration_ready = profile_setup and all(item['ready'] for item in prerequisites)
        return {'state': state, 'verification_state': verification_state, 'next_stage': 'profile_setup' if not profile_setup else 'ready',
                'checks': values, 'modes': mode_states, 'reason': reason,
                'fingerprint': info['fingerprint'], 'info': info,
                'control_available': profile_setup, 'safety_summary': safety_summary,
                'connection_readiness': {'structurally_valid': profile_setup, 'read_only_connectable': offline_ready,
                    'offline_connectable': offline_ready, 'hardware_connectable': profile_setup,
                    'offline_issues': core_issues,
                    'write_enabled': runtime.get('write_enabled') is True, 'basic_calibration_ready': calibration_ready,
                    'basic_calibration_prerequisites': prerequisites,
                    'next_actions': [item['action'] for item in prerequisites if not item['ready']]},
                'setup_issues': setup_issues, 'next_actions': next_actions, 'gain_purposes': gain_purposes,
                'source_trusted': self._is_official_profile(info),
                'evidence': {k: {'kind': v.get('kind'), 'note': v.get('note', ''),
                                 'observations': v.get('observations') or {}}
                             for k, v in stages.items() if isinstance(v, dict)}}

    @staticmethod
    def _field_issue(message):
        import re
        match = re.search(r'(?<![A-Za-z_])((?:core\.)?(?:model|control|calibration|capability|safety_policy|shutdown)\.[A-Za-z0-9_.-]+|hardware\.[A-Za-z0-9_.-]+|profile(?:_entry)?\.hardware\.plugin)', message)
        path = match.group(1) if match else ''
        if path and not path.startswith(('core.', 'hardware.', 'profile')): path = 'core.' + path
        key = path.removeprefix('core.').split('.')[0]
        group = {'model': 'model', 'control': 'motion', 'capability': 'motion',
                 'calibration': 'mapping', 'safety_policy': 'safety', 'shutdown': 'shutdown'}.get(key, 'hardware' if path.startswith(('hardware.', 'profile')) else 'model')
        return {'group': group, 'path': path, 'message': message,
                'action': {'page': 'profile_setup', 'group': group, 'field': path, 'label': '检查配置'}}

    def _gain_purpose_status(self, core, meta, scopes, mode_states):
        result = {}
        stored = meta.get('gain_purposes') or {}
        legacy = meta.get('modes') or {}
        for purpose, (label, gain_keys, check_keys) in self._GAIN_PURPOSES.items():
            configured = all(mode_states.get(key.upper()) in ('configured', 'verified', 'warning', 'stale') for key in gain_keys)
            entry = stored.get(purpose)
            if not isinstance(entry, dict): entry = None
            state = 'pending'
            if isinstance(entry, dict) and ((entry.get('kind') == 'validated_gain_purpose' and entry.get('result') == 'passed') or
                                          (entry.get('kind') == 'measured_gain_purpose' and entry.get('result') == 'warning')):
                source = Path(entry.get('candidate_path') or '/missing')
                context = entry.get('context_files')
                same_source = source.is_file() and _sha(source) == entry.get('candidate_sha256')
                same_source = same_source and isinstance(context, dict) and bool(context) and all(Path(path).is_file() and _sha(Path(path)) == digest for path, digest in context.items())
                state = ('warning' if entry.get('result') == 'warning' else 'verified') if entry.get('scope') == scopes.get('gain_purposes', {}).get(purpose) and same_source else 'stale'
                if not isinstance(context, dict) or not context: state = 'historical'
            else:
                old = [legacy.get(key.upper()) for key in gain_keys]
                if any(isinstance(item, dict) and item.get('result') in ('passed', 'recorded') for item in old):
                    state = 'historical'
                    entry = next(item for item in old if isinstance(item, dict))
            checks = [{'id': key, 'label': self._GAIN_CHECK_LABELS[key],
                       'status': state if state != 'verified' or (entry.get('checks') or {}).get(key) is True else 'pending'} for key in check_keys]
            # A historical tracking-only record is not proof of the new joint qualification
            if state == 'historical':
                for check in checks:
                    if check['id'] not in ('tracking', 'drag'): check['status'] = 'pending'
            result[purpose] = {'label': label, 'configured': configured,
                'evidence_status': state, 'checks': checks,
                'candidate_path': (entry or {}).get('candidate_path', ''), 'warnings': (entry or {}).get('warnings', []),
                'action': {'page': 'calibration', 'entry': 'gain', 'label': '执行器增益自动标定'}}
        return result

    def _builtin_readiness(self, config):
        # A bundled configuration is trusted as a source, never as proof of this machine's tests
        return self._readiness(config)

    def record_stage(self, config, stage, answers, native=None):
        """Store optional operator notes without turning them into runtime admission.

        A checklist is not a native robot measurement. Opening or recording a
        checklist must not require ACTIVE, a workbench, or prior stage results.
        Native measurement/verification tasks keep their independent safeguards.
        """
        requirements = {
            'model_check': ('chain', 'limits', 'inertial'),
            'bringup': ('connection',),
            'startup_hold': ('hold_gains', 'supported', 'emergency_stop'),
            'mapping_calibration': ('joint_ids', 'directions', 'offsets'),
            'geometry_calibration': ('frames', 'zero_pose', 'tool_pose'),
            'gravity_verification': ('distinct_poses', 'support', 'restored'),
            'validation': ('tracking', 'stopping', 'faults'),
        }
        if stage not in requirements:
            raise ValueError('当前项目不支持人工自查记录')
        if not isinstance(answers, dict):
            raise ValueError('检查内容必须是对象')
        note = str(answers.get('note') or '').strip()
        if len(note) > 600:
            raise ValueError('检查备注不能超过 600 字')
        checked = {}
        for key in requirements[stage]:
            value = answers.get(key, False)
            if not isinstance(value, bool):
                raise ValueError(f'检查项 {key} 必须为布尔值')
            checked[key] = value

        # Only the Profile and its configuration dependencies are required to
        # associate a note with the right robot; no hardware session is needed.
        info = self.inspector.inspect(config)
        core = read_yaml(info['resources']['core'])
        hardware = read_yaml(info['resources']['hardware'])
        scopes = self._dependencies(info, core, hardware)
        path, data = self._readiness_meta(info, scopes)
        data['profile'] = info['profile']
        field = 'modes' if stage == 'startup_hold' else 'stages'
        key = 'RIGID_HOLD' if stage == 'startup_hold' else stage
        previous = (data.get(field) or {}).get(key)
        # Updating optional notes must not erase independent native proof
        # that still matches the current configuration
        keep_native = (stage == 'gravity_verification' and isinstance(previous, dict)
            and previous.get('result') == 'passed'
            and previous.get('scope') == scopes[stage]
            and previous.get('kind') in ('native_gravity_completed', 'operator_and_live')
            and (previous.get('runtime') or {}).get('sample_count', 0) >= 3)
        entry = dict(previous) if keep_native else {
            'result': 'recorded', 'scope': scopes[stage], 'kind': 'operator_note',
        }
        entry.update({'note': note, 'observations': checked, 'recorded_at': time.time()})
        data.setdefault(field, {})[key] = entry
        # A real completed native gravity task may provide stronger evidence.
        # Missing, stale or unrelated sessions never block saving a note and
        # must never transform self-reported checkboxes into a native proof.
        if stage == 'gravity_verification' and native is not None:
            try:
                sample = native.request('status', {}, timeout=8)
                proof = sample.get('gravity_verification') or {}
                core_runtime = (core.get('control') or {}).get('runtime') or {}
                stored = (core.get('model') or {}).get('gravity_scale') or {}
                names = (core.get('model') or {}).get('joint_names') or []
                actual = sample.get('gravity_scale') or []
                scales_restored = (len(names) == len(actual) and bool(names) and
                    all(isinstance(stored.get(j), (int, float)) and
                        isinstance(v, (int, float)) and math.isfinite(v) and
                        abs(stored[j] - v) <= 1e-6 for j, v in zip(names, actual)))
                native_completed = (
                    sample.get('robot_state') == 'INACTIVE' and
                    proof.get('active') is False and
                    proof.get('phase') == 'completed' and
                    isinstance(proof.get('sample_count'), int) and
                    proof['sample_count'] >= 3 and
                    sample.get('joint_names') == names and
                    sample.get('model_feedforward_mode') == core_runtime.get('model_feedforward_mode') and
                    scales_restored and (sample.get('fault') or {}).get('code') in (None, '', 'NONE', 'OK'))
                if native_completed:
                    data['stages'][stage].update({
                        'result': 'passed', 'kind': 'native_gravity_completed',
                        'runtime': {'sample_count': proof['sample_count'],
                                    'robot_state': 'INACTIVE', 'joint_names': names},
                    })
                    data.setdefault('modes', {})['COMPLIANT_DRAG'] = {
                        'result': 'passed', 'scope': scopes['modes']['COMPLIANT_DRAG'],
                        'kind': 'native_gravity_completed',
                        'sample_count': proof['sample_count'],
                        'recorded_at': time.time(),
                    }
            except Exception:
                # There is no usable live native proof; keep only the note.
                pass
        _atomic_yaml(path, data)
        return self._readiness(config)

    @staticmethod
    def _gain_interaction_context(core, purpose):
        admittance = ((core.get('capability') or {}).get('admittance') or {})
        if purpose == 'SERVO_TRACKING':
            return {key: admittance.get(key) for key in ('joint_enabled', 'observer', 'calibration', 'controller')}
        if purpose == 'RIGID' and ((core.get('control') or {}).get('hold_compensation') or {}).get('enabled'):
            return {key: admittance.get(key) for key in ('observer', 'calibration')}
        return None

    def _gain_binding(self, folder, info, core, hardware, purposes):
        from model_calibration_source import matches_recorded_fingerprint
        folder = Path(folder)
        metadata_path = folder / 'metadata.json'
        metadata = json.loads(metadata_path.read_text())
        if not metadata.get('profile') or metadata['profile'] != info['profile']:
            raise ValueError('增益记录属于其他机器人或未记录机器人名称；可查看历史数据，本机验证需要重新标定')
        recorded_core = folder / 'source/core.yaml'
        if not matches_recorded_fingerprint(recorded_core, metadata.get('core_fingerprint')):
            raise ValueError('增益记录缺少可信 Core 快照；可查看历史数据，请重新标定后导入本机验证')
        original = canonical_core(read_yaml(recorded_core))
        if original.get('calibration') != core.get('calibration'):
            raise ValueError('core.calibration.joints 当前映射与增益标定时不同，请重新标定本机增益')
        for field in ('joint_names', 'base_frame', 'tool_frame', 'gravity', 'gravity_scale'):
            if (original.get('model') or {}).get(field) != core['model'].get(field):
                raise ValueError('core.model.' + field + ' 与增益标定时不同，请重新标定本机增益')
        for field in ('model_feedforward_mode', 'ctrl_frequency_hz', 'joint_acc_filter_alpha'):
            if original['control']['runtime'].get(field) != core['control']['runtime'].get(field):
                raise ValueError('core.control.runtime.' + field + ' 与增益标定时不同，请重新验证当前控制设置')
        for purpose in purposes:
            expected = self._gain_interaction_context(core, purpose)
            if expected is not None:
                recorded = ((original.get('capability') or {}).get('admittance') or {})
                if {key: recorded.get(key) for key in expected} != expected:
                    raise ValueError('core.capability.admittance 与增益验收使用的观测或导纳参数不同，请重新验证当前设置')
        files = [metadata_path, recorded_core]
        correction = (original.get('model') or {}).get('gravity_correction_path')
        current_correction = core['model'].get('gravity_correction_path')
        if bool(correction) != bool(current_correction):
            raise ValueError('core.model.gravity_correction_path 与增益标定时不同，请重新标定')
        if correction:
            snapshot = folder / 'source/gravity-correction.yaml'
            expected = metadata.get('gravity_correction_fingerprint')
            current = (Path(info['resources']['core']).parent / current_correction).resolve()
            if not matches_recorded_fingerprint(snapshot, expected) or not matches_recorded_fingerprint(current, expected):
                raise ValueError('core.model.gravity_correction_path 缺少与本机一致的标定快照，请重新标定')
            files.append(snapshot)
        recorded_hardware = folder / 'source/hardware.yaml'
        overrides = metadata.get('hardware_overrides')
        if not isinstance(overrides, dict) or not matches_recorded_fingerprint(recorded_hardware, metadata.get('hardware_fingerprint')):
            raise ValueError('历史增益记录未保存硬件配置与连接覆盖值；可查看或载入候选，请重新验证本机增益后导入')
        before_hardware = effective_hardware(read_yaml(recorded_hardware), overrides)
        current_hardware = effective_hardware(hardware, info.get('connection_overrides'))
        if before_hardware != current_hardware:
            raise ValueError('硬件配置与增益标定时不同，请检查执行器、总线及连接覆盖值后重新标定')
        plugin = (metadata.get('runtime_provenance') or {}).get('hardware_plugin_file_path')
        def plugin_name(value):
            name = Path(str(value or '')).name
            if name.startswith('lib'): name = name[3:]
            return name.split('.so', 1)[0]
        if not plugin or plugin_name(plugin) != plugin_name(info.get('hardware_plugin')):
            raise ValueError('增益标定使用的硬件驱动与当前机器人不同或未记录，请重新标定')
        files.append(recorded_hardware)
        effective = folder / 'source/hardware-effective.yaml'
        if effective.is_file() or metadata.get('hardware_effective_fingerprint'):
            if not matches_recorded_fingerprint(effective, metadata.get('hardware_effective_fingerprint')):
                raise ValueError('增益标定的实际硬件配置快照已变化，请恢复原始任务文件')
            if effective_hardware(read_yaml(effective)) != before_hardware:
                raise ValueError('实际硬件配置快照与标定连接覆盖值不一致，请检查原始任务')
            files.append(effective)
        return {str(path.resolve()): _sha(path) for path in files}

    def approve_gain_mode(self, config, mode, path, native=None, gain_purposes=None):
        from workflow_registry import prepare_gain_candidate
        from control_parameters import deployment_gain_groups, selected_gain_purposes, gain_selection_report
        from artifact_exports import gain_candidate_targets
        source = Path(path).expanduser().resolve()
        if not source.is_dir() and (not source.is_file() or source.name != 'gain-candidate.json'):
            raise ValueError('请选择标定任务中的 gain-candidate.json，或直接选择该任务目录')
        folder = source if source.is_dir() else source.parent
        requested = str(mode or '').upper()
        aliases = {key.upper(): purpose for purpose, (_, keys, _) in self._GAIN_PURPOSES.items() for key in keys}
        aliases.update({purpose: purpose for purpose in self._GAIN_PURPOSES})
        if requested not in ('', 'ALL') and requested not in aliases:
            raise ValueError('未知增益用途 / Unknown gain purpose: ' + requested)
        selection = (selected_gain_purposes(gain_purposes) if gain_purposes is not None else
                     selected_gain_purposes([aliases[requested]]) if requested not in ('', 'ALL') else None)
        candidate_info = prepare_gain_candidate(str(folder), selection)
        candidate_path = Path(candidate_info['candidate_path'])
        candidate = json.loads(candidate_path.read_text())
        groups = deployment_gain_groups(candidate, selection)
        available = {aliases[key.upper()]: key for key in groups}
        if gain_purposes is not None or requested in ('', 'ALL'):
            selected = available
        elif requested in aliases and aliases[requested] in available:
            selected = {aliases[requested]: available[aliases[requested]]}
        else:
            raise ValueError('候选未包含所选增益用途，请选择对应候选或导入候选中的全部用途')
        info = self.inspector.inspect(config)
        core = canonical_core(read_yaml(info['resources']['core']))
        hardware = read_yaml(info['resources']['hardware'])
        urdf = (Path(info['resources']['core']).parent / core['model']['urdf_path']).resolve()
        if not urdf.is_file() or candidate.get('urdf_fingerprint') != self._native_file_fingerprint(urdf):
            raise ValueError('增益验收候选与当前 URDF 不一致，请选择当前模型的标定任务')
        if candidate.get('joint_names') != core['model']['joint_names']:
            raise ValueError('增益候选的关节顺序与当前配置不一致')
        context_files = self._gain_binding(folder, info, core, hardware, selected)
        targets, _ = gain_candidate_targets(candidate, core, core['model']['joint_names'], selection)
        def same(actual, expected):
            if isinstance(expected, dict): return isinstance(actual, dict) and set(actual) == set(expected) and all(same(actual[k], v) for k, v in expected.items())
            if isinstance(expected, bool): return isinstance(actual, bool) and actual is expected
            return isinstance(actual, (float, int)) and not isinstance(actual, bool) and math.isfinite(actual) and math.isclose(actual, expected, rel_tol=1e-7, abs_tol=1e-5)
        for purpose in selected:
            keys = self._GAIN_PURPOSES[purpose][1]
            for key, encoded in targets.items():
                relevant = key[:2] == ('control', 'controller') and key[2] in keys
                relevant |= purpose == 'RIGID' and key[:2] == ('control', 'hold_compensation')
                if not relevant: continue
                actual = core
                for part in key: actual = actual.get(part) if isinstance(actual, dict) else None
                expected = yaml.safe_load(encoded)
                if key == ('control', 'hold_compensation', 'enabled') and actual is None: actual = False
                if not same(actual, expected):
                    raise ValueError('当前配置与已验收候选不一致：core.' + '.'.join(key) + '；请先应用并保存该候选，再导入验收记录')
        scopes = self._dependencies(info, core, hardware)
        meta_path, meta = self._readiness_meta(info, scopes)
        meta['profile'] = info['profile']
        for purpose, group in selected.items():
            quality = gain_selection_report(candidate, {group: groups[group]})
            checks = {name: quality['validated'] for name in self._GAIN_PURPOSES[purpose][2]}
            proof = {'result': 'passed' if quality['validated'] else 'warning',
                     'kind': 'validated_gain_purpose' if quality['validated'] else 'measured_gain_purpose',
                     'validated': quality['validated'], 'warnings': quality['warnings'],
                     'candidate_sha256': _sha(candidate_path), 'candidate_path': str(candidate_path),
                     'checks': checks, 'context_files': context_files, 'recorded_at': time.time()}
            meta.setdefault('gain_purposes', {})[purpose] = {**proof, 'scope': scopes['gain_purposes'][purpose]}
            for gain_key in self._GAIN_PURPOSES[purpose][1]:
                key = gain_key.upper()
                mode_scope = scopes['startup_hold'] if key == 'RIGID_HOLD' else scopes['modes'][key]
                meta.setdefault('modes', {})[key] = {**proof, 'scope': mode_scope}
        _atomic_yaml(meta_path, meta)
        return self._readiness(config)

    def list(self):
        data = self._library_data(); items=[]
        builtin = self.inspector.profiles({})
        for p in builtin['profiles']:
            if p['name'] in data['hidden_builtin']: continue
            config={'profile':p['name'],'profile_file':'','resource_paths':'','serial_port':'','baudrate':'','bus':''}
            items.append({'id':'builtin:'+p['name'],'profile':p['name'],'source':'builtin','profile_file':builtin['profile_file'],'resource_paths':'','readiness':self._builtin_readiness(config)})
        for key, value in sorted(data['profiles'].items()):
            if not isinstance(value, dict): continue
            config={'profile':text(value.get('profile')),'profile_file':text(value.get('profile_file')),'resource_paths':text(value.get('resource_paths')),'serial_port':'','baudrate':'','bus':''}
            items.append({'id':'user:'+key,'profile':config['profile'],'source':'external','profile_file':config['profile_file'],'resource_paths':config['resource_paths'],'readiness':self._readiness(config)})
        return {'path':str(self.library_file.resolve()),'profiles':items}

    def register(self, profile_file, profile, resource_paths='', policy='error'):
        profile_file = str(Path(profile_file).expanduser().resolve()); profile=text(profile); resource_paths=text(resource_paths)
        self.inspector.inspect({'profile':profile,'profile_file':profile_file,'resource_paths':resource_paths})
        data=self._library_data(); key=profile
        if key in data['profiles']:
            if policy == 'replace': pass
            elif policy == 'suffix':
                i=2
                while f'{profile}_{i}' in data['profiles']: i+=1
                key=f'{profile}_{i}'
            else: raise ValueError('Profile Library already contains this profile id')
        data['profiles'][key]={'profile_file':profile_file,'profile':profile,'resource_paths':resource_paths}
        self._write_library(data)
        return {'id':'user:'+key,'profile':profile,'profile_file':profile_file,'resource_paths':resource_paths}

    def remove(self, entry_id):
        data=self._library_data(); entry_id=text(entry_id)
        if entry_id.startswith('builtin:'):
            name=entry_id.split(':',1)[1]
            data['hidden_builtin'].append(name)
        elif entry_id.startswith('user:'):
            data['profiles'].pop(entry_id.split(':',1)[1], None)
        else: raise ValueError('invalid Profile Library entry')
        self._write_library(data); return {'removed':entry_id}

    def discover_profile_source(self, source):
        path=Path(source).expanduser().resolve()
        if path.is_file(): candidates=[path]
        elif path.is_dir():
            direct=path/'robot_profiles.yaml'
            if direct.is_file():
                candidates=[direct]
            else:
                candidates=[]
                ignored={'.git','build','install','log','logs','node_modules','__pycache__','.install','.cache'}
                for root, dirs, files in os.walk(path):
                    dirs[:] = [d for d in dirs if d not in ignored and not d.startswith('.')]
                    if 'robot_profiles.yaml' in files:
                        candidates.append(Path(root)/'robot_profiles.yaml')
                        if len(candidates) >= 20: break
        else: raise ValueError('Profile source does not exist')
        if not candidates: raise ValueError('robot_profiles.yaml was not found in the selected Profile package')
        result=[]
        for candidate in candidates:
            profiles=read_yaml(candidate).get('profiles') or {}
            for name in profiles:
                result.append({'profile_file':str(candidate),'profile':name,'resource_paths':str(path if path.is_dir() else candidate.parent)})
        return {'candidates':result}

    def _description_source(self, source):
        path=Path(source).expanduser().resolve()
        if path.is_file() and path.suffix.lower()=='.zip':
            stamp=hashlib.sha256((str(path)+str(path.stat().st_mtime_ns)+str(path.stat().st_size)).encode()).hexdigest()[:20]
            root=self.cache/stamp
            if not root.is_dir():
                self.cache.mkdir(parents=True, exist_ok=True)
                temporary = Path(tempfile.mkdtemp(prefix='extract-', dir=self.cache))
                try:
                    _safe_extract(path, temporary)
                    temporary.rename(root)
                finally:
                    if temporary.exists(): shutil.rmtree(temporary)
            return root
        if path.is_dir(): return path
        if path.is_file() and (path.name.endswith('.urdf') or path.name.endswith('.xacro')): return path.parent
        raise ValueError('select a Description directory, ZIP, URDF, or Xacro')

    def inspect_description(self, source, model_relative=""):
        selected = Path(source).expanduser().resolve()
        root = self._description_source(source)
        if model_relative:
            selected = (root / text(model_relative)).resolve()
            if not selected.is_relative_to(root) or not selected.is_file():
                raise ValueError('Selected model is outside the Description')
        explicit = selected.is_file() and selected.suffix.lower() in ('.urdf', '.xacro')
        package_root = None
        if explicit:
            for parent in selected.parents:
                if (parent / 'package.xml').is_file():
                    package_root = parent
                    root = parent
                    break
            package_root = package_root or selected.parent
            urdf = selected
        else:
            candidates = sorted(root.glob('**/*.urdf'))
            if not candidates:
                candidates = sorted(p for p in root.glob('**/*.xacro')
                    if not any(word in p.name.lower() for word in ('ros2_control', 'material', 'macro')))
            if not candidates:
                raise ValueError('Description contains no URDF or model Xacro')
            if len(candidates) > 1:
                return {'model_choices': [str(p.relative_to(root)) for p in candidates],
                        'source_root': str(root), 'joints': [], 'links': []}
            urdf = candidates[0]
            for parent in urdf.parents:
                if not parent.is_relative_to(root): break
                if (parent / 'package.xml').is_file():
                    package_root = parent
                    break
            package_root = package_root or urdf.parent
        metadata = package_root / 'package.xml'
        package_name = (ET.parse(metadata).getroot().findtext('name') or '').strip() if metadata.is_file() else package_root.name
        if not package_name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in package_name):
            raise ValueError('Description package name is invalid')
        original_package_root = str(package_root)
        resource_roots = [str(root), str(package_root.parent), str(package_root),
                          *self.inspector.roots(config_values({}), str(urdf))]
        if urdf.suffix.lower() == '.xacro':
            # Expand in a cache copy, preserving all relative mesh and include paths
            digest = hashlib.sha256()
            for item in sorted(package_root.rglob('*')):
                if item.is_file():
                    digest.update(str(item.relative_to(package_root)).encode())
                    if item.suffix.lower() in ('.xacro', '.urdf', '.yaml', '.xml'):
                        digest.update(item.read_bytes())
                    else:
                        digest.update(str((item.stat().st_size, item.stat().st_mtime_ns)).encode())
            cache = self.cache / ('expanded-' + digest.hexdigest())
            copied = cache / package_root.name
            relative = urdf.relative_to(package_root)
            expanded = copied / relative.with_name(relative.name + '.expanded.urdf')
            if not copied.is_dir():
                cache.mkdir(parents=True, exist_ok=True)
                shutil.copytree(package_root, copied, ignore=shutil.ignore_patterns('.git', '__pycache__', '.serialarm'))
            # The main package copy is reusable; includes in other packages can
            # change independently, so always evaluate Xacro against current resources.
            script = ('import sys,json; sys.path.insert(0,sys.argv[2]); '
                'from profiles import Inspector; import xacro,xacro.substitution_args; '
                'inspector=Inspector(sys.argv[3]); roots=json.loads(sys.argv[4]); '
                'xacro.substitution_args._eval_find=lambda package:str(inspector.share(package,roots)); '
                'print(xacro.process_file(sys.argv[1]).toxml())')
            result = subprocess.run([sys.executable, '-c', script, str(copied / relative),
                str(Path(__file__).parent), str(self.root), json.dumps(resource_roots)],
                capture_output=True, text=True, timeout=30)
            if result.returncode:
                shutil.rmtree(cache, ignore_errors=True)
                raise ValueError('Xacro expansion failed: ' + result.stderr[-1600:])
            ET.fromstring(result.stdout)
            expanded.write_text(result.stdout, encoding='utf-8')
            root, package_root, urdf = cache, copied, expanded
        raw=self.inspector._parse_urdf(urdf,[str(root),str(package_root.parent),str(package_root), *resource_roots])
        children={j['child'] for j in raw['joints']}; roots=[l['name'] for l in raw['links'] if l['name'] not in children]
        parent_joint={j['child']:j for j in raw['joints']}; by_parent={}
        for j in raw['joints']: by_parent.setdefault(j['parent'],[]).append(j)
        matrices={name:[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]] for name in roots}
        queue=list(roots); visited_frames=set()
        while queue:
            parent=queue.pop(0)
            if parent in visited_frames: continue
            visited_frames.add(parent); base=matrices[parent]
            for j in by_parent.get(parent,[]):
                matrices[j['child']]=_matmul(base,_origin_matrix(j['origin'])); queue.append(j['child'])
        frames=[]
        for link in raw['links']:
            m=matrices.get(link['name'],[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
            frames.append({'name':link['name'],'position':[m[0][3],m[1][3],m[2][3]],'quaternion':_quat_from_matrix(m)})
        leaves=[l['name'] for l in raw['links'] if l['name'] not in by_parent]
        tool=next((name for name in leaves if name.lower() in ('tool0','tool','tcp')), leaves[0] if leaves else (raw['links'][-1]['name'] if raw['links'] else ''))
        xacros=sorted(package_root.glob('**/*.xacro'))
        preview={'profile':'description-preview','core_config':'','urdf':str(urdf),'description_urdf':str(urdf),'description_mismatch':False,'native':{'config':{'urdf_path':str(urdf),'base_frame':roots[0] if roots else '','tool_frame':tool,'gravity':[0,0,-9.81],'gravity_scale':[],'joint_names':[]},'info':{},'state':{'positions':[],'frames':frames,'center_of_mass':None}},'model':raw,'mesh_files':raw['mesh_files'],'offline':True}
        model_issues = []
        try: Inspector.validate_urdf(urdf)
        except ValueError as error: model_issues.append(self._field_issue(str(error)))
        for link in raw['links']:
            for item in [*link['visuals'], *link['collisions']]:
                if item.get('error'):
                    model_issues.append(self._field_issue('model.urdf_path ' + link['name'] + ': ' + item['error']))
        return {'source_root':str(root),'resource_roots':resource_roots,'original_package_root':original_package_root,'package_root':str(package_root),'package_name':package_name,'urdf':str(urdf),'urdf_relative':str(urdf.relative_to(package_root)),'xacro':str(xacros[0]) if xacros else '','ros2_control_xacro':next((str(p) for p in xacros if 'ros2_control' in p.name),''),'roots':roots,'leaves':leaves,'links':[x['name'] for x in raw['links']],'joints':raw['joints'],'preview':preview,'fingerprint':_sha(urdf), 'model_valid': not model_issues, 'model_issues': model_issues}

    def _collect_description_resources(self, analysis, package_dir, copied):
        """Keep copied models usable after their original workspace is removed."""
        original = Path(analysis['package_root']).resolve()
        roots = [analysis['source_root'], str(original.parent), str(original), *analysis.get('resource_roots', []),
                 *self.inspector.roots(config_values({}), analysis['urdf'])]
        urdf = copied / analysis['urdf_relative']
        tree = ET.parse(urdf)
        model_changed = False
        dependencies = {}
        imported_files = {}
        def collect_file(source):
            source = source.resolve()
            if source in imported_files: return imported_files[source]
            if not source.is_file(): raise ValueError('model.urdf_path 缺少模型资源 / Missing model resource: ' + str(source))
            target = copied / 'resources/imported' / hashlib.sha256(str(source.parent).encode()).hexdigest()[:12] / source.name
            imported_files[source] = target
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            # Preserve the standard relative material/texture dependencies of OBJ and DAE assets.
            references = []
            if source.suffix.lower() in ('.obj', '.mtl'):
                body = source.read_text(errors='replace')
                for line in body.splitlines():
                    fields = line.strip().split(maxsplit=1)
                    if len(fields) == 2 and (fields[0] == 'mtllib' or fields[0].lower().startswith('map_') or fields[0] in ('bump','disp','decal')):
                        references.append(fields[1].strip())
                for reference in references:
                    replacement = collect_file(source.parent / reference)
                    body = body.replace(reference, os.path.relpath(replacement, target.parent))
                target.write_text(body)
            elif source.suffix.lower() == '.dae':
                asset_tree = ET.parse(source)
                changed = False
                for node in asset_tree.getroot().iter():
                    if node.tag.rsplit('}',1)[-1] == 'init_from' and node.text:
                        reference = node.text.strip()
                        if reference.startswith('#'): continue
                        referenced = source.parent / reference.removeprefix('file://')
                        if referenced.is_file():
                            replacement = collect_file(referenced)
                            node.text = os.path.relpath(replacement, target.parent); changed = True
                if changed: asset_tree.write(target, encoding='utf-8', xml_declaration=True)
            return target
        for mesh in tree.getroot().iter('mesh'):
            source = Path(self.inspector._resolve_mesh(mesh.get('filename'), analysis['urdf'], roots))
            if source.is_relative_to(original):
                target = copied / source.relative_to(original)
            elif source.is_relative_to(Path(analysis.get('original_package_root', original))):
                target = copied / source.relative_to(Path(analysis['original_package_root']))
            else:
                owner = next((parent for parent in source.parents if (parent / 'package.xml').is_file()), None)
                if owner:
                    name = (ET.parse(owner / 'package.xml').findtext('name') or '').strip()
                    if not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name):
                        raise ValueError('model.urdf_path 外部资源包名称无效 / Invalid dependency package name')
                    destination = package_dir / name
                    if destination.resolve().is_relative_to(owner):
                        raise ValueError('destination 输出目录不能位于模型依赖包内 / Output must be outside dependency packages')
                    if name == analysis['package_name'] or (name in dependencies and dependencies[name] != owner):
                        raise ValueError('model.urdf_path 资源包名称冲突 / Dependency package name collision: ' + name)
                    if name not in dependencies:
                        shutil.copytree(owner, destination, ignore=shutil.ignore_patterns('.git', '__pycache__', '.serialarm'))
                        dependencies[name] = owner
                    target = destination / source.relative_to(owner)
                else:
                    target = collect_file(source)
            if not target.is_file(): raise ValueError('model.urdf_path 复制后缺少资源 / Copied resource is missing: ' + str(target))
            relative = os.path.relpath(target, urdf.parent)
            if mesh.get('filename') != relative:
                mesh.set('filename', relative); model_changed = True
        if model_changed: tree.write(urdf, encoding='utf-8', xml_declaration=True)
        # Parse using generated resources only, with no fallback to the source workspace.
        parsed = self.inspector._parse_urdf(urdf, [str(package_dir)])
        problems = [item['error'] for link in parsed['links'] for item in [*link['visuals'],*link['collisions']] if item.get('error')]
        if problems: raise ValueError('model.urdf_path 生成配置缺少资源 / Generated model resources are incomplete: ' + '; '.join(problems))

    @staticmethod
    def _joint_map(names, value): return {name:value for name in names}

    def create_profile(self, options):
        source=text(options.get('source')); analysis=self.inspect_description(source, options.get('model_relative', ''))
        if analysis.get('model_choices'): raise ValueError('Select a model before creating the Profile')
        if analysis.get('model_issues'):
            raise ValueError(analysis['model_issues'][0]['message'])
        profile=text(options.get('profile'))
        if not profile or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in profile): raise ValueError('Profile id may contain letters, numbers, underscore, and hyphen only')
        destination=Path(text(options.get('destination'))).expanduser().resolve()
        package_dir=destination/profile
        source_package = Path(analysis['package_root']).resolve()
        if package_dir == source_package or package_dir.is_relative_to(source_package):
            raise ValueError('destination 输出目录不能位于源 Description 目录内，请选择其外部目录')
        if package_dir.exists() and any(package_dir.iterdir()): raise ValueError('destination Profile package already exists and is not empty')
        requested_joints = options.get('joint_names') or []
        known_joints = {j['name'] for j in analysis['joints'] if j['type'] != 'fixed'}
        if not isinstance(requested_joints, list) or not requested_joints or len(set(requested_joints)) != len(requested_joints) or not set(requested_joints).issubset(known_joints):
            raise ValueError('select unique controlled joints from the Description')
        if options.get('base_frame') not in analysis['links'] or options.get('tool_frame') not in analysis['links']:
            raise ValueError('base/tool frame must exist in the Description')
        try:
            package_dir.mkdir(parents=True,exist_ok=True)
            copied=package_dir/analysis['package_name']; shutil.copytree(analysis['package_root'],copied,dirs_exist_ok=True)
            self._collect_description_resources(analysis, package_dir, copied)
            config_dir=copied/'config'/'serialarm'; config_dir.mkdir(parents=True,exist_ok=True)
            joints=[text(x) for x in options.get('joint_names',[]) if text(x)]
            if not joints: raise ValueError('select at least one controlled joint')
            base=text(options.get('base_frame')); tool=text(options.get('tool_frame'))
            if base not in analysis['links'] or tool not in analysis['links']: raise ValueError('base/tool frame must exist in the Description')
            zeros=self._joint_map(joints,0.0); ones=self._joint_map(joints,1.0)
            limits = {item['name']: item.get('limit') or {} for item in analysis['joints']}
            park = {}
            for name in joints:
                low, high = limits[name].get('lower'), limits[name].get('upper')
                if low is not None and high is not None:
                    if float(low) > float(high): raise ValueError(f'{name} URDF 关节限位上下界错误')
                    park[name] = max(float(low), min(float(high), 0.0))
                else: park[name] = 0.0
            kp_rigid=self._joint_map(joints,20.0); kd_rigid=self._joint_map(joints,0.5); kp_soft=self._joint_map(joints,3.0); kd_soft=self._joint_map(joints,0.1)
            core={
              'model':{'urdf_path':'../../'+analysis['urdf_relative'].replace('\\','/'),'joint_names':joints,'base_frame':base,'tool_frame':tool,'gravity':[0.0,0.0,-9.81],'gravity_scale':ones},
              'calibration':{'joints':{j:{'direction':1.0,'pos_ratio':1.0,'tor_ratio':1.0,'joint_zero_offset':0.0,'actuator_zero_offset':0.0} for j in joints}},
              'control':{'runtime':{'ctrl_frequency_hz':200.0,'joint_acc_filter_alpha':0.2,'write_enabled':False,'model_feedforward_mode':'GRAVITY','tracking_impedance_mode':'COMPLIANT_TRACKING','tracking_gain_profile':'COMPLIANT'},'controller':{'allow_full_cmd':False,'rigid_hold':{'kp':kp_rigid,'kd':kd_rigid},'rigid_tracking':{'kp':kp_rigid,'kd':kd_rigid},'compliant_hold':{'kp':kp_soft,'kd':kd_soft},'compliant_drag':{'kp':zeros,'kd':zeros.copy()},'compliant_tracking':{'kp':kp_soft,'kd':kd_soft},'servo_tracking':{'kp':kp_rigid.copy(),'kd':kd_rigid.copy()}}},
              'safety_policy':{'position_margin':0.0,'cmd_vel_scale':0.25,'state_vel_scale':1.0,'max_acc':self._joint_map(joints,2.0),'max_kp_override':self._joint_map(joints,50.0),'max_kd_override':self._joint_map(joints,2.0),'max_dt_s':0.02,'state_timeout_s':0.05,'cmd_timeout_s':0.10,'require_all_actuators_online':True,'require_all_actuators_enabled':True,'reject_motor_error':True,'require_continuous_cmd':False,'fault_recovery':{'default_mode':'rigid_hold','allow_compliant_recovery':True,'require_operator_request':True,'gravity_model_validated':False,'recovery_timeout_s':30.0,'compliant_recovery':{'kp':kp_soft,'kd':kd_soft,'max_vel':self._joint_map(joints,0.5),'effort_scale':0.5}}},
              'shutdown':{'park_before_disable':True,'park_pos':park,'speed_scale':0.05,'position_tolerance':0.05,'velocity_tolerance':0.05,'settle_time_s':0.25,'relaxed_tolerance_ratio':2.0,'timeout_s':10.0},
              'capability':{'admittance':{'enabled':False,'reference_behavior':'NATIVE_IMPEDANCE','joint_enabled':{j:True for j in joints},'observer':{'mode':'MOMENTUM','momentum_gain':self._joint_map(joints,25.0)},'calibration':{'torque_bias':zeros,'torque_threshold':self._joint_map(joints,0.05),'friction':{'enabled':False,'velocity_transition':0.03,'positive_coulomb':zeros,'positive_viscous':zeros,'negative_coulomb':zeros,'negative_viscous':zeros}},'controller':{'mass':ones,'damping':self._joint_map(joints,5.5),'stiffness':self._joint_map(joints,15.0),'max_delta_q':self._joint_map(joints,0.5),'max_delta_q_dot':self._joint_map(joints,1.0)}}}
            }
            # If the Description already supplies a robot-specific Core config,
            # use it as the authoritative profile configuration. In particular,
            # do not silently replace validated joint-specific gains by the generic
            # 20/0.5 (rigid) and 3/0.1 (compliant) creation defaults.
            authored_config = copied / 'config' / 'core' / f'{profile}.yaml'
            if authored_config.is_file():
                authored_core = read_yaml(authored_config)
                authored_model = authored_core.get('model') or {}
                if (authored_model.get('joint_names') != joints or
                    authored_model.get('base_frame') != base or
                    authored_model.get('tool_frame') != tool):
                    raise ValueError('Description 自带 Core 配置的关节或坐标系与当前 Profile 不匹配，请核对后再导入')
                authored_urdf = (authored_config.parent / str(authored_model.get('urdf_path', ''))).resolve()
                expected_urdf = (copied / analysis['urdf_relative']).resolve()
                if authored_urdf != expected_urdf or not authored_urdf.is_file():
                    raise ValueError('Description 自带 Core 配置中的 URDF 路径与所选模型不匹配')
                core_config_relative = f'config/core/{profile}.yaml'
                # Normalize only the newly copied package, never its source.
                authored_config.write_text(four_mode_control_text(authored_config.read_text()), encoding='utf-8')
                # Keep an already included legacy copy consistent, for consumers
                # still accessing config/serialarm/core.yaml directly.
                legacy_copy = config_dir / 'core.yaml'
                if legacy_copy.is_file():
                    legacy_copy.write_bytes(authored_config.read_bytes())
            else:
                core_config_relative = 'config/serialarm/core.yaml'
                _atomic_yaml(config_dir/'core.yaml',four_mode_core(core))
            hardware_plugin=text(options.get('hardware_plugin'))
            if hardware_plugin and any(c in hardware_plugin for c in '\0\r\n'):
                raise ValueError('hardware_plugin contains an invalid path')
            hardware={}
            actuators=options.get('actuators') or []
            complete=hardware_plugin=='serial_arm_hardware_damiao' and len(actuators)==len(joints) and all(text(a.get('motor_type')) and int(a.get('motor_id',0))>0 for a in actuators)
            if hardware_plugin=='serial_arm_hardware_damiao':
                if len(actuators)!=len(joints):
                    actuators = list(actuators[:len(joints)]) + [{} for _ in range(max(0,len(joints)-len(actuators)))]
                bus=text(options.get('bus')) or 'main_can'; device=text(options.get('device')) or '/dev/ttyACM0'; baud=int(options.get('baudrate') or 921600)
                hardware={'buses':{bus:{'type':'can','backend':'damiao_usb2can','device':device,'baudrate':baud}},'damiao':{'bus':bus,'refresh_state_in_read':False,'feedback_timeout_s':0.05,'activation_retries':3,'startup_read_cycles':5,'stop_kp':10.0,'stop_kd':0.15,'stop_cycles':5,'actuators':{j:{'name':text(a.get('name')) or f'actuator{i+1}','motor_id':int(a.get('motor_id') or i+1),'master_id':int(a.get('master_id') or 0),'motor_type':text(a.get('motor_type'))} for i,(j,a) in enumerate(zip(joints,actuators))}}}
            _atomic_yaml(config_dir/'hardware.yaml',hardware)
            entry={'core':{'package':analysis['package_name'],'config':core_config_relative},'hardware':{'plugin':hardware_plugin,'config_package':analysis['package_name'],'config':'config/serialarm/hardware.yaml'},'description':{'package':analysis['package_name'],'urdf':analysis['urdf_relative'].replace('\\','/')}}
            if analysis['ros2_control_xacro']:
                entry['description']['ros2_control_xacro']=str(Path(analysis['ros2_control_xacro']).relative_to(analysis['package_root'])).replace('\\','/')
            profile_file=package_dir/'robot_profiles.yaml'; _atomic_yaml(profile_file,{'profiles':{profile:entry}})
            readiness=self._readiness_path(profile_file, profile); _atomic_yaml(readiness,{'profile':profile,'managed_by_launcher':True,'source_urdf_fingerprint':analysis['fingerprint'],'stages':{}})
            return {'package':str(package_dir),'profile_file':str(profile_file),'profile':profile,'resource_paths':str(package_dir),'hardware_complete':complete,'config':{'profile':profile,'profile_file':str(profile_file),'resource_paths':str(package_dir),'serial_port':'','baudrate':'','bus':''}}
        except Exception:
            if package_dir.exists(): shutil.rmtree(package_dir)
            raise

    def mark_stage(self, config, stage, confirmed=True):
        raise ValueError('已停用单击确认，请使用带观察记录和运行证据的阶段验收')

    def _profile_config_paths(self, config):
        info = self.inspector.inspect(config)
        return info, Path(info['resources']['core']), Path(info['resources']['hardware']), Path(info['profile_file'])

    _DAMIAO_MOTOR_TYPES = frozenset((
        'DM4310', 'DM4310_48V', 'DM4340', 'DM4340_48V', 'DM6006',
        'DM6248P', 'DM8006', 'DM8009', 'DM10010L', 'DM10010',
        'DMH3510', 'DMH6215', 'DMG6220', 'DMJH11',
    ))

    def _setup_issues(self, core, hardware, entry):
        # Report concrete missing/invalid fields before invoking the native driver
        issues = []
        def issue(group, path, message):
            issues.append({'group': group, 'path': path, 'message': message,
                           'action': {'page': 'profile_setup', 'group': group, 'field': path, 'label': '前往配置'}})
        names = ((core.get('model') or {}).get('joint_names') or [])
        plugin = ((entry.get('hardware') or {}).get('plugin') or '').strip()
        if not plugin:
            issue('hardware', 'profile_entry.hardware.plugin', '请选择与执行器匹配且已安装的硬件驱动')
        if plugin == 'serial_arm_hardware_damiao':
            buses = hardware.get('buses') or {}
            damiao = hardware.get('damiao') or {}
            bus_name = damiao.get('bus')
            if not bus_name or not isinstance(buses, dict) or bus_name not in buses:
                issue('hardware', 'hardware.damiao.bus', '请选择已配置的总线，或先初始化达妙硬件字段')
            else:
                selected = buses[bus_name]
                if not isinstance(selected, dict):
                    issue('hardware', f'hardware.buses.{bus_name}', '总线配置必须是对象')
                else:
                    for key, desc in (('type','总线类型'),('backend','通信后端'),('device','设备路径')):
                        if not isinstance(selected.get(key), str) or not selected.get(key).strip():
                            issue('hardware', f'hardware.buses.{bus_name}.{key}', f'请填写总线 {bus_name} 的{desc}')
                    baud = selected.get('baudrate')
                    if isinstance(baud, bool) or not isinstance(baud,(int,float)) or not math.isfinite(baud) or baud <= 0 or int(baud) != baud:
                        issue('hardware', f'hardware.buses.{bus_name}.baudrate', f'总线 {bus_name} 的波特率必须是正整数')
            actuators = damiao.get('actuators') or {}
            if not isinstance(actuators, dict):
                issue('hardware', 'hardware.damiao.actuators', '执行器映射必须是关节名称到电机配置的对象')
                actuators = {}
            for joint in names:
                actuator = actuators.get(joint)
                base = f'hardware.damiao.actuators.{joint}'
                if not isinstance(actuator, dict):
                    issue('hardware', base, f'{joint} 缺少执行器映射')
                    continue
                motor_type = actuator.get('motor_type')
                if not isinstance(motor_type, str) or not motor_type.strip():
                    issue('hardware', base+'.motor_type', f'{joint} 未选择电机型号，请在硬件与总线中选择')
                elif motor_type not in self._DAMIAO_MOTOR_TYPES:
                    issue('hardware', base+'.motor_type', f'{joint} 电机型号 {motor_type} 不受当前达妙驱动支持，请选择列表中的型号')
                for field, minval in (('motor_id',1),('master_id',0)):
                    value = actuator.get(field)
                    if isinstance(value, bool) or not isinstance(value, int) or value < minval:
                        issue('hardware', base+'.'+field, f'{joint} 的 {field} 必须是大于等于 {minval} 的整数')
            extra = set(actuators)-set(names)
            if extra: issue('hardware', 'hardware.damiao.actuators', f'执行器列表含未受控的关节 {", ".join(sorted(extra))}')
            ids = [a.get('motor_id') for j,a in actuators.items() if j in names and isinstance(a,dict) and isinstance(a.get('motor_id'),int)]
            if len(set(ids)) < len(ids): issue('hardware','hardware.damiao.actuators','同一总线上的电机 ID 不能重复')
        calibration = (core.get('calibration') or {}).get('joints') or {}
        for joint in names:
            values = calibration.get(joint) or {}
            for key in ('direction', 'pos_ratio', 'tor_ratio', 'joint_zero_offset', 'actuator_zero_offset'):
                value = values.get(key)
                valid = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                valid = valid and (value in (-1, 1) if key == 'direction' else value != 0 if key in ('pos_ratio', 'tor_ratio') else True)
                if not valid:
                    issue('mapping', f'core.calibration.joints.{joint}.{key}',
                          f'{joint} 的映射参数 {key} ' + ('必须为 1 或 -1' if key == 'direction' else '必须为非零数值' if key in ('pos_ratio', 'tor_ratio') else '必须为有限数值'))
        safety = core.get('safety_policy') or {}
        for key in ('position_margin', 'max_dt_s', 'state_timeout_s', 'cmd_timeout_s', 'cmd_vel_scale', 'state_vel_scale'):
            value = safety.get(key)
            valid = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
            valid = valid and (value >= 0 if key == 'position_margin' else value > 0)
            if key.endswith('vel_scale'): valid = valid and value <= 1
            if not valid: issue('safety', 'core.safety_policy.' + key, f'安全参数 {key} ' + ('必须在 0 到 1 之间且大于零' if key.endswith('vel_scale') else '缺失或无效'))
        acceleration = safety.get('max_acc') or {}
        if not isinstance(acceleration, dict) or set(acceleration) != set(names):
            issue('safety','core.safety_policy.max_acc','Safety 必须为全部受控关节提供最大加速度 max_acc')
        else:
            for joint in names:
                value = acceleration[joint]
                if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or value<=0:
                    issue('safety',f'core.safety_policy.max_acc.{joint}',f'{joint} 的 Safety 最大加速度必须大于零')
        return issues

    def profile_editor_load(self, config):
        info, core_path, hardware_path, profile_path = self._profile_config_paths(config)
        core = canonical_core(read_yaml(core_path))
        admittance = (core.get('capability') or {}).get('admittance')
        if isinstance(admittance, dict):
            admittance.setdefault('reference_behavior', 'MODEL_ADMITTANCE' if admittance.get('enabled') is True else 'NATIVE_IMPEDANCE')
        hardware = read_yaml(hardware_path) if hardware_path.is_file() else {}
        profile = read_yaml(profile_path)['profiles'][info['profile']]
        gravity_context = gravity_validation_context(core, core_path, hardware, (profile.get('hardware') or {}).get('plugin'))
        self._gravity_validation_baselines.setdefault((str(core_path), _sha(core_path),
            _sha(hardware_path) if hardware_path.is_file() else '', _sha(profile_path)), gravity_context)
        joints = (core.get('model') or {}).get('joint_names') or []
        try:
            urdf = (core_path.parent / core['model']['urdf_path']).resolve()
            nodes = ET.parse(urdf).getroot().findall('joint')
            urdf_effort_limits = {node.get('name'): float(node.find('limit').get('effort'))
                for node in nodes if node.find('limit') is not None and node.find('limit').get('effort')}
            urdf_joint_limits = {}
            for node in nodes:
                if node.get('name') not in joints: continue
                limit = node.find('limit')
                values = {'type': node.get('type') or ''}
                for field in ('lower', 'upper', 'velocity', 'effort'):
                    raw = limit.get(field) if limit is not None else None
                    try: values[field] = float(raw) if raw is not None else None
                    except (TypeError, ValueError): values[field] = None
                urdf_joint_limits[node.get('name')] = values
        except (OSError, ET.ParseError, KeyError, ValueError):
            urdf_effort_limits = {}
            urdf_joint_limits = {}
        return {
            'profile': info['profile'], 'core_path': str(core_path),
            'hardware_path': str(hardware_path), 'profile_path': str(profile_path),
            'urdf_path': str((core_path.parent / core['model']['urdf_path']).resolve()),
            'core_sha': _sha(core_path),
            'hardware_sha': _sha(hardware_path) if hardware_path.is_file() else '',
            'profile_sha': _sha(profile_path),
            'joint_names': joints, 'urdf_effort_limits': urdf_effort_limits,
            'urdf_joint_limits': urdf_joint_limits, 'core': core, 'hardware': hardware,
            'profile_entry': profile,
            'original_gravity_model_validated': bool(((core.get('safety_policy') or {}).get('fault_recovery') or {}).get('gravity_model_validated')),
            'hardware_plugin': (profile.get('hardware') or {}).get('plugin', ''),
            'setup_issues': self._setup_issues(core, hardware, profile),
            'managed_by_launcher': self._readiness_path(info['profile_file'], info['profile']).is_file(),
        }

    def hardware_inspect(self, config, plugin=None):
        info = self.inspector.inspect(config)
        selected = self.inspector.hardware_driver(info.get('hardware_plugin', '') if plugin is None else plugin)
        drivers = {}
        for name in ('serial_arm_hardware_damiao', selected['plugin']):
            if name:
                driver = self.inspector.hardware_driver(name)
                drivers[name] = {**driver, 'label': '达妙 / Damiao' if name == 'serial_arm_hardware_damiao' else name}
        manifest = self.inspector.manifest()
        for prefix in {manifest.get('native_prefix'), manifest.get('install_prefix')} - {None, ''}:
            for pattern in ('lib/libserial_arm_hardware_*.so', 'lib64/libserial_arm_hardware_*.so'):
                for path in Path(prefix).glob(pattern):
                    name = path.name.removeprefix('lib').removesuffix('.so')
                    drivers.setdefault(name, {**self.inspector.hardware_driver(name), 'label': name})
        return {**selected, 'drivers': list(drivers.values())}

    def hardware_import(self, config, path, plugin=None):
        source = Path(text(path)).expanduser().resolve()
        if not source.is_file(): raise ValueError('hardware 导入文件不存在 / Hardware YAML file does not exist')
        hardware = read_yaml(source)
        editor = self.profile_editor_load(config)
        editor['hardware'] = hardware
        if plugin is not None:
            editor['profile_entry'].setdefault('hardware', {})['plugin'] = text(plugin)
        editor['hardware_plugin'] = editor['profile_entry'].get('hardware', {}).get('plugin', '')
        editor['hardware_imported'] = True
        editor['hardware_import_source'] = str(source)
        editor['setup_issues'] = self._setup_issues(editor['core'], hardware, editor['profile_entry'])
        return editor

    @staticmethod
    def _finite(value, path, minimum=None, maximum=None, integer=False):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{path} 必须是有限数值')
        if integer and int(value) != value:
            raise ValueError(f'{path} 必须是整数')
        if minimum is not None and value < minimum:
            raise ValueError(f'{path} 不能小于 {minimum}')
        if maximum is not None and value > maximum:
            raise ValueError(f'{path} 不能大于 {maximum}')

    @staticmethod
    def _same_structure(original, candidate, path=''):
        # Edit existing values without silently deleting unknown driver fields
        if isinstance(original, dict):
            if not isinstance(candidate, dict) or set(original) - set(candidate):
                raise ValueError(f'{path or "root"} 缺少原有配置字段')
            for key, value in original.items():
                ProfileLibrary._same_structure(value, candidate[key], f'{path}.{key}' if path else str(key))
        elif isinstance(original, list):
            if not isinstance(candidate, list) or len(original) != len(candidate):
                raise ValueError(f'{path} 数组长度不一致')
            for i, value in enumerate(original):
                ProfileLibrary._same_structure(value, candidate[i], f'{path}[{i}]')
        elif isinstance(original, bool):
            if not isinstance(candidate, bool): raise ValueError(f'{path} 必须是布尔值')
        elif isinstance(original, (float, int)):
            ProfileLibrary._finite(candidate, path)
        elif isinstance(original, str):
            if not isinstance(candidate, str): raise ValueError(f'{path} 必须是文本')
        elif original is None:
            if isinstance(candidate, (dict, list)) or not (candidate is None or isinstance(candidate, (str, bool, int, float))):
                raise ValueError(f'{path} 请输入标量值')
            if isinstance(candidate, (int, float)) and not isinstance(candidate, bool):
                ProfileLibrary._finite(candidate, path)
        elif candidate != original:
            raise ValueError(f'{path} 不支持此参数类型')

    def _validate_core_profile(self, core, hardware, entry, core_path, old_core, allow_hardware_draft=False):
        core, old_core = canonical_core(core), canonical_core(old_core)
        names = core['model']['joint_names']
        original_names = old_core['model']['joint_names']
        if names != original_names or not isinstance(names, list) or not names or len(set(names)) != len(names):
            raise ValueError('model.joint_names 当前 Profile 的受控关节不能在参数编辑页更改，请重新建立控制链')
        def section(path):
            node = core
            for part in path.split('.'):
                if not isinstance(node, dict) or part not in node:
                    raise ValueError(f'{path} 缺少必需的配置字段')
                node = node[part]
            return node
        def scalar(path, minv=None, maxv=None, integer=False):
            self._finite(section(path), path, minv, maxv, integer)
        def joint_map(path, minv=None, maxv=None, integer=False, nonzero=False):
            values = section(path)
            if not isinstance(values, dict) or set(values) != set(names):
                raise ValueError(f'{path} 关节名称与 model.joint_names 不一致')
            for name in names:
                self._finite(values[name], path + '.' + name, minv, maxv, integer)
                if nonzero and values[name] == 0: raise ValueError(f'{path}.{name} 不能为零')
        model = core['model']
        urdf = (core_path.parent / model['urdf_path']).resolve()
        if not urdf.is_file(): raise ValueError('model.urdf_path 无法找到 URDF 文件')
        root = Inspector.validate_urdf(urdf, names)
        joints_xml = {j.get('name'): j for j in root.findall('joint')}
        links_xml = {n.get('name') for n in root.findall('link')}
        for name in names:
            if name not in joints_xml or joints_xml[name].get('type') == 'fixed':
                raise ValueError(f'model.joint_names.{name} 在 URDF 中不存在可动关节')
        for key in ('base_frame', 'tool_frame'):
            if model[key] not in links_xml:
                raise ValueError(f'model.{key} 不在 URDF link 中')
        parent_by_child = {}
        for node in root.findall('joint'):
            parent, child = node.find('parent'), node.find('child')
            if parent is not None and child is not None:
                parent_by_child[child.get('link')] = (parent.get('link'), node.get('name'))
        chain = []
        current, visited = model['tool_frame'], set()
        while current != model['base_frame'] and current not in visited:
            visited.add(current)
            parent = parent_by_child.get(current)
            if parent is None: break
            current, name = parent
            chain.append(name)
        chain.reverse()
        if current != model['base_frame'] or [name for name in chain if name in names] != names:
            raise ValueError('model.base_frame 或 model.tool_frame 与受控关节链不一致')
        gravity = model['gravity']
        if not isinstance(gravity, list) or len(gravity) != 3:
            raise ValueError('model.gravity 必须包含三个重力分量')
        for i,v in enumerate(gravity): self._finite(v, f'model.gravity[{i}]')
        joint_map('model.gravity_scale',0,2)
        if model.get('gravity_correction_path'):
            if not (core_path.parent / model['gravity_correction_path']).resolve().is_file():
                raise ValueError('model.gravity_correction_path 找不到校正文件')
        for name in names:
            path = 'calibration.joints.' + name
            data = section(path)
            for field in ('direction','pos_ratio','tor_ratio','joint_zero_offset','actuator_zero_offset'):
                self._finite(data[field], f'{path}.{field}')
            if data['direction'] not in (-1,1): raise ValueError(f'{path}.direction 只能为 -1 或 1')
            if data['pos_ratio'] == 0 or data['tor_ratio'] == 0:
                raise ValueError(f'{path} 位置和力矩比例不能为零')
        runtime = section('control.runtime')
        absent_boolean = object()
        for field in ('control.runtime.write_enabled', 'control.controller.allow_full_cmd',
                      'safety_policy.require_all_actuators_online', 'safety_policy.require_all_actuators_enabled',
                      'safety_policy.reject_motor_error', 'safety_policy.require_continuous_cmd',
                      'safety_policy.fault_recovery.allow_compliant_recovery',
                      'safety_policy.fault_recovery.require_operator_request',
                      'safety_policy.fault_recovery.gravity_model_validated', 'shutdown.park_before_disable',
                      'capability.admittance.calibration.friction.enabled'):
            value = core
            for part in field.split('.'):
                if not isinstance(value, dict) or part not in value:
                    value = absent_boolean
                    break
                value = value[part]
            if value is not absent_boolean and not isinstance(value, bool):
                raise ValueError(f'{field} 必须是布尔值')
        scalar('control.runtime.ctrl_frequency_hz',1)
        scalar('control.runtime.joint_acc_filter_alpha',0,1)
        if runtime['model_feedforward_mode'] not in ('NONE','GRAVITY','FULL_INVERSE_DYNAMICS'):
            raise ValueError('control.runtime.model_feedforward_mode 无效')
        if runtime['tracking_impedance_mode'] not in ('RIGID_TRACKING','COMPLIANT_TRACKING'):
            raise ValueError('control.runtime.tracking_impedance_mode 无效')
        if runtime.get('tracking_gain_profile') not in ('RIGID', 'COMPLIANT', 'SERVO_TRACKING'):
            raise ValueError('control.runtime.tracking_gain_profile 请选择刚性、柔性或伺服跟踪')
        controller = section('control.controller')
        for mode in ('rigid_hold','rigid_tracking','compliant_hold','compliant_drag','compliant_tracking','servo_tracking'):
            if mode == 'servo_tracking' and mode not in controller: continue
            for axis in ('kp','kd'): joint_map(f'control.controller.{mode}.{axis}',0)
        hold = core['control'].get('hold_compensation')
        if hold is not None:
            if not isinstance(hold, dict) or not isinstance(hold.get('enabled'), bool):
                raise ValueError('control.hold_compensation.enabled 必须为布尔值')
            if set(hold) - {'enabled', 'integral_gain', 'max_torque', 'max_rate',
                            'velocity_threshold', 'error_threshold', 'contact_threshold'}:
                raise ValueError('control.hold_compensation 含未知参数')
            for field in ('integral_gain', 'max_torque', 'max_rate', 'contact_threshold'):
                joint_map('control.hold_compensation.' + field, 0)
            for field in ('velocity_threshold', 'error_threshold'):
                scalar('control.hold_compensation.' + field, 1e-9)
        safety = section('safety_policy')
        for key in ('position_margin', 'max_dt_s','state_timeout_s','cmd_timeout_s'):
            scalar('safety_policy.'+key, 0 if key=='position_margin' else 1e-7)
        for key in ('cmd_vel_scale','state_vel_scale'):
            scalar('safety_policy.'+key,1e-7,1)
        for field in ('max_acc','max_effort_override','max_kp_override','max_kd_override'):
            if field in safety: joint_map('safety_policy.'+field,0)
        recovery = section('safety_policy.fault_recovery')
        if recovery['default_mode'] != 'rigid_hold':
            raise ValueError('safety_policy.fault_recovery.default_mode 仅支持 rigid_hold')
        scalar('safety_policy.fault_recovery.recovery_timeout_s',1e-7)
        for field in ('kp','kd','max_vel'):
            joint_map('safety_policy.fault_recovery.compliant_recovery.'+field,0)
        scalar('safety_policy.fault_recovery.compliant_recovery.effort_scale',0,1)
        for name in names:
            for mode in ('rigid_hold','rigid_tracking','compliant_hold','compliant_drag','compliant_tracking','servo_tracking'):
                for axis in ('kp','kd'):
                    if mode not in controller: continue
                    limit = safety.get('max_'+axis+'_override',{}).get(name)
                    if limit is not None and controller[mode][axis][name] > limit:
                        raise ValueError(f'control.controller.{mode}.{axis}.{name} 超出 safety_policy.max_{axis}_override')
            if hold is not None:
                effort = safety.get('max_effort_override', {}).get(name)
                if effort is not None and hold['max_torque'][name] > effort:
                    raise ValueError(f'control.hold_compensation.max_torque.{name} 超出总力矩预算')
        shut = section('shutdown')
        joint_map('shutdown.park_pos')
        for key in ('speed_scale','compliant_phase_ratio'):
            if key in shut: scalar('shutdown.'+key,1e-7,1)
        for key in ('position_tolerance','velocity_tolerance','settle_time_s','relaxed_tolerance_ratio','timeout_s'):
            scalar('shutdown.'+key,0 if key=='settle_time_s' else 1e-7)
        for name in names:
            lim = joints_xml[name].find('limit')
            if lim is not None and lim.get('lower') is not None and lim.get('upper') is not None:
                lo,hi = float(lim.get('lower')),float(lim.get('upper'))
                pos = shut['park_pos'][name]
                if not lo+safety['position_margin'] <= pos <= hi-safety['position_margin']:
                    raise ValueError(f'shutdown.park_pos.{name} 超出 URDF 有效限位')
        admittance = section('capability.admittance')
        if not isinstance(admittance['enabled'],bool): raise ValueError('capability.admittance.enabled 必须为布尔值')
        if admittance.get('reference_behavior', 'NATIVE_IMPEDANCE') not in ('NATIVE_IMPEDANCE', 'MODEL_ADMITTANCE'):
            raise ValueError('capability.admittance.reference_behavior 无效')
        flags = admittance['joint_enabled']
        if not isinstance(flags,dict) or set(flags)!=set(names) or any(not isinstance(v,bool) for v in flags.values()):
            raise ValueError('capability.admittance.joint_enabled 与关节名称不一致')
        if admittance['observer']['mode'] not in ('FULL_ID','MOMENTUM'):
            raise ValueError('capability.admittance.observer.mode 无效')
        joint_map('capability.admittance.observer.momentum_gain',0)
        for key in ('torque_bias','torque_threshold'):
            joint_map('capability.admittance.calibration.'+key,0 if key=='torque_threshold' else None)
        friction = admittance['calibration']['friction']
        scalar('capability.admittance.calibration.friction.velocity_transition',1e-7)
        for key in ('positive_coulomb','positive_viscous','negative_coulomb','negative_viscous'):
            joint_map('capability.admittance.calibration.friction.'+key)
        for key in ('mass','damping','stiffness','max_delta_q','max_delta_q_dot'):
            joint_map('capability.admittance.controller.'+key,1e-7 if key=='mass' else 0)
        plugin = ((entry.get('hardware') or {}).get('plugin') or '')
        if not isinstance(plugin,str): raise ValueError('profile.hardware.plugin 必须是文本')
        if plugin == 'serial_arm_hardware_damiao':
            damiao = hardware.get('damiao') or {}
            buses = hardware.get('buses') or {}
            bus = damiao.get('bus')
            if not bus or bus not in buses:
                if allow_hardware_draft:
                    return
                raise ValueError('hardware.damiao.bus 必须对应 hardware.buses 中的总线')
            for key in ('feedback_timeout_s',):
                self._finite(damiao.get(key), 'hardware.damiao.'+key,1e-7)
            for key in ('activation_retries','startup_read_cycles','stop_cycles'):
                self._finite(damiao.get(key), 'hardware.damiao.'+key,0,integer=True)
            for key in ('stop_kp','stop_kd'):
                self._finite(damiao.get(key), 'hardware.damiao.'+key,0)
            self._finite(buses[bus].get('baudrate'), f'hardware.buses.{bus}.baudrate',1,integer=True)
            if not buses[bus].get('device'): raise ValueError(f'hardware.buses.{bus}.device 不能为空')
            acts = damiao.get('actuators') or {}
            if set(acts)!=set(names) and not allow_hardware_draft:
                raise ValueError('hardware.damiao.actuators 与受控关节不一致')
            ids = set()
            for name in names:
                if name not in acts:
                    if allow_hardware_draft: continue
                    raise ValueError(f'hardware.damiao.actuators.{name} 尚未配置')
                item = acts[name]
                if not item.get('motor_type') and not allow_hardware_draft:
                    raise ValueError(f'hardware.damiao.actuators.{name}.motor_type 未选择电机型号')
                if item.get('motor_type') and item['motor_type'] not in self._DAMIAO_MOTOR_TYPES:
                    raise ValueError(f'hardware.damiao.actuators.{name}.motor_type 不支持型号 {item["motor_type"]}，请从列表选择')
                self._finite(item.get('motor_id'),f'hardware.damiao.actuators.{name}.motor_id',1,0x7ff,integer=True)
                self._finite(item.get('master_id'),f'hardware.damiao.actuators.{name}.master_id',0,0x7ff,integer=True)
                if item['master_id'] == 0 and item['motor_id'] > 0x0f:
                    raise ValueError(f'hardware.damiao.actuators.{name} requires an unambiguous feedback ID')
                bus_key = (bus,item['motor_id'])
                if bus_key in ids: raise ValueError(f'hardware.damiao.actuators.{name}.motor_id 重复')
                ids.add(bus_key)

    def _profile_editor_gravity_status(self, config, payload):
        info, core_path, hardware_path, profile_path = self._profile_config_paths(config)
        before = canonical_core(read_yaml(core_path))
        hardware = read_yaml(hardware_path) if hardware_path.is_file() else {}
        entry = read_yaml(profile_path)['profiles'][info['profile']]
        current = gravity_validation_context(before, core_path, hardware, (entry.get('hardware') or {}).get('plugin'))
        core = canonical_core(payload['core']) if isinstance(payload.get('core'), dict) else before
        submitted_entry = payload.get('profile_entry') or entry
        after = gravity_validation_context(core, core_path, payload.get('hardware', hardware),
                                           (submitted_entry.get('hardware') or {}).get('plugin'))
        baseline = self._gravity_validation_baselines.get((str(core_path), payload.get('core_sha'),
            payload.get('hardware_sha'), payload.get('profile_sha')), current)
        changed = list(dict.fromkeys(gravity_validation_changes(baseline, after) + gravity_validation_changes(current, after)))
        requested = gravity_trusted(core)
        return {'gravity_validation_changes': changed,
                'gravity_validation_reset': bool(requested and changed)}

    def profile_editor_save(self, config, payload):
        info, core_path, hw_path, profile_path = self._profile_config_paths(config)
        for path, field in ((core_path,'core_sha'),(hw_path,'hardware_sha'),(profile_path,'profile_sha')):
            current_sha = _sha(path) if path.is_file() else ''
            if current_sha != payload.get(field):
                raise ValueError(f'{path.name} 已被外部修改，请重新加载参数后再保存')
        before_core = canonical_core(read_yaml(core_path))
        before_hw = read_yaml(hw_path) if hw_path.is_file() else {}
        profile_data = read_yaml(profile_path)
        original_entry = profile_data['profiles'][info['profile']]
        core = canonical_core(payload['core']) if isinstance(payload.get('core'), dict) else payload.get('core')
        hardware = payload.get('hardware')
        entry = payload.get('profile_entry')
        if not isinstance(core,dict) or not isinstance(hardware,dict) or not isinstance(entry,dict):
            raise ValueError('core hardware profile_entry 必须是完整配置对象')
        self._same_structure(before_core,core)
        for section, allowed in {
            'model': {'gravity_correction_path'},
            'safety_policy': {'max_effort_override'},
            'shutdown': {'compliant_phase_ratio'},
        }.items():
            extra = set(core[section]) - set(before_core[section])
            if not extra.issubset(allowed):
                raise ValueError(f'{section} 不支持新增字段 {sorted(extra)}')
        for section in ('calibration','control','capability'):
            extra = set(core[section]) - set(before_core[section])
            if extra - ({'hold_compensation'} if section == 'control' else set()):
                raise ValueError(f'{section} 不允许新增未知字段')
        new_modes = set(core['control']['controller']) - set(before_core['control']['controller'])
        if not new_modes.issubset({'servo_tracking'}):
            raise ValueError('control.controller 不允许新增未知控制模式')
        gravity_status = self._profile_editor_gravity_status(config, {**payload, 'core': core})
        original_gravity_trusted = before_core['safety_policy']['fault_recovery']['gravity_model_validated']
        requested_gravity_trusted = core['safety_policy']['fault_recovery']['gravity_model_validated']
        # Explicit operator acknowledgment is required for a false -> true trust transition
        # Disabling recovery trust must always remain possible
        if requested_gravity_trusted and not original_gravity_trusted and payload.get('gravity_recovery_confirmed') is not True:
            raise ValueError('启用柔性故障恢复前必须确认重力补偿已在当前机械臂和负载上完成真机验证')
        if gravity_status['gravity_validation_reset']:
            core['safety_policy']['fault_recovery']['gravity_model_validated'] = False
        if payload.get('hardware_imported') is not True:
            self._same_structure(before_hw,hardware,'hardware')
        self._same_structure(original_entry,entry,'profile')
        if entry.get('core') != original_entry.get('core') or entry.get('description') != original_entry.get('description'):
            raise ValueError('profile.core 和 profile.description 关联不允许在参数编辑中切换，请重新导入 Profile')
        self._validate_core_profile(core,hardware,entry,core_path,before_core,allow_hardware_draft=True)
        # New saves use four purposes; a schema-only save rewrites just the
        # controller block and preserves the surrounding authored YAML.
        desired = {}
        if core != before_core:
            desired[core_path] = yaml.safe_dump(four_mode_core(core), allow_unicode=True, sort_keys=False)
        else:
            original_text = core_path.read_text(encoding='utf-8')
            modern_text = four_mode_control_text(original_text)
            if modern_text != original_text: desired[core_path] = modern_text
        if hardware != before_hw: desired[hw_path] = yaml.safe_dump(hardware, allow_unicode=True, sort_keys=False)
        next_profiles = copy.deepcopy(profile_data)
        next_profiles['profiles'][info['profile']] = entry
        if next_profiles != profile_data: desired[profile_path] = yaml.safe_dump(next_profiles, allow_unicode=True, sort_keys=False)
        if core_path in desired and core_path.parent.name == 'core' and core_path.parent.parent.name == 'config':
            compatibility = core_path.parent.parent / 'serialarm' / 'core.yaml'
            if compatibility.is_file(): desired[compatibility] = desired[core_path]
        backup = {path: path.read_bytes() if path.is_file() else None for path in desired}
        try:
            # Before touching files, validate every new YAML and stage replacements
            for path, value in desired.items():
                parsed = yaml.safe_load(value)
                if not isinstance(parsed,dict): raise ValueError(f'{path.name} 无效配置结构')
            for path,value in desired.items():
                _atomic_yaml_text(path,value)
        except Exception:
            for path, content in backup.items():
                if content is None: path.unlink(missing_ok=True)
                else:
                    tmp = path.with_suffix(path.suffix+'.rollback')
                    tmp.write_bytes(content); tmp.replace(path)
            raise
        return {**self.profile_editor_load(config), **gravity_status}
