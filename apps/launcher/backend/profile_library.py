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
from control_parameters import canonical_core, four_mode_core, four_mode_control_text, effective_hardware, interaction_configuration, complete_mapping_values, revoke_changed_interaction_validation
from gravity_recovery_validation import gravity_validation_context, gravity_validation_changes, gravity_trusted
from parameter_contract import field_constraints, constraint_errors, parking_complete, parking_plan
from commissioning import (REVIEWS as COMMISSIONING_REVIEWS, FOUNDATION, PATH_STEPS,
    OPTIONAL as COMMISSIONING_OPTIONAL, stage_key as commissioning_stage_key,
    review_requirements, task_answers, observation_confirmed, snapshot as commissioning_snapshot)


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
    _MODES = ('RIGID', 'COMPLIANT', 'DRAG', 'SERVO_TRACKING')
    _MODE_KEYS = {'RIGID': 'rigid', 'COMPLIANT': 'compliant', 'DRAG': 'drag', 'SERVO_TRACKING': 'rigid'}
    _GAIN_PURPOSES = {
        'RIGID': ('刚性', ('rigid',), ('tracking', 'hold', 'hold_compensation')),
        'COMPLIANT': ('柔性', ('compliant',), ('tracking', 'hold')),
        'DRAG': ('拖拽', ('drag',), ('drag',)),
        'SERVO_TRACKING': ('伺服跟踪', ('rigid',), ('native_tracking', 'admittance_tracking')),
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
        model_chain = {key: value for key, value in model.items() if key not in ('gravity_scale', 'gravity_correction_path', 'gravity_model_validated')}
        model_hash = self._digest(source, model_chain)
        startup_hash = self._digest(model_hash, hardware_hash, calibration,
                                    controller.get('rigid'))
        mapping_hash = self._digest(source, hardware_hash, calibration)
        geometry_hash = self._digest(source, calibration)
        gravity_hash = self._digest(gravity_validation_context(core, info['resources']['core'], hardware, info.get('hardware_plugin')))
        dynamics_hash = self._digest(model_hash, mapping_hash, model.get('gravity'),
                                     controller.get('rigid'), capability)
        scopes = {
            'model_check': model_hash, 'bringup': self._digest(hardware_hash, runtime, safety),
            'mapping_calibration': mapping_hash, 'geometry_calibration': geometry_hash,
            'gravity_verification': gravity_hash, 'dynamics_calibration': dynamics_hash,
            'validation': self._digest(mapping_hash, geometry_hash, safety, shutdown,
                                       controller.get('rigid'), controller.get('compliant')),
            'startup_hold': startup_hash,
        }
        scopes['modes'] = {name: self._digest(model_hash, mapping_hash,
            (controller.get('rigid')
             if name == 'SERVO_TRACKING' else controller.get(self._MODE_KEYS[name]))) for name in self._MODES}
        scopes['gain_purposes'] = {purpose: self._digest(model_hash, mapping_hash, gravity_hash,
            {key: runtime.get(key) for key in ('model_feedforward_mode', 'ctrl_frequency_hz', 'joint_acc_filter_alpha')},
            self._gain_interaction_context(core, purpose),
            safety.get('max_effort_override'),
            {name: controller.get(name) for name in gain_keys},
            (core.get('control') or {}).get('hold_compensation', {'enabled': False}) if purpose == 'RIGID' else None)
            for purpose, (_, gain_keys, _) in self._GAIN_PURPOSES.items()}
        # Commissioning receipts are invalidated by their own physical inputs,
        # not optional calibration work or an unrelated interaction parameter.
        try:
            tree = ET.parse(urdf_path).getroot()
            kinematics = {'links': [node.get('name') for node in tree.findall('link')],
                          'joints': [ET.tostring(node, encoding='unicode') for node in tree.findall('joint')]}
        except (OSError, ET.ParseError):
            kinematics = None
        geometry = self._digest(info['profile'], str(info['profile_file']), kinematics,
                               {key: model.get(key) for key in ('joint_names', 'base_frame', 'tool_frame')})
        hardware_mapping = {key: value for key, value in hardware.items() if key != 'buses'}
        mapping = self._digest(geometry, calibration, info.get('hardware_plugin'), hardware_mapping)
        admittance = capability.get('admittance') or {}
        observation = self._digest(gravity_hash, mapping, admittance.get('observer'), admittance.get('calibration'),
                                   {key: runtime.get(key) for key in ('ctrl_frequency_hz', 'joint_acc_filter_alpha', 'control_path')})
        response = self._digest(observation, admittance.get('joint_enabled'), admittance.get('controller'),
                                (admittance.get('validation') or {}).get('velocity_tolerance'),
                                safety.get('max_tracking_error'), safety.get('max_acc'), safety.get('max_effort_override'))
        for path in ('MIT', 'CSP'):
            hold_scope = self._digest(mapping, path, controller.get('rigid') if path == 'MIT' else safety.get('max_tracking_error'))
            domain = {'model_check': geometry, 'mapping_calibration': mapping,
                'bringup': self._digest(mapping, effective_hardware(hardware, info.get('connection_overrides'))),
                'safety_setup': self._digest(hold_scope, safety), 'startup_hold': hold_scope,
                'stop_fault_check': self._digest(hold_scope, safety.get('fault_recovery')),
                'gravity_verification': gravity_hash,
                'observation_calibration': observation, 'observation_validation': observation,
                'admittance_response': response, 'admittance_release': response,
                'effort_model_validation': self._digest(gravity_hash, mapping),
                'csp_execution_validation': hold_scope,
                'mit_drag_validation': self._digest(gravity_hash, mapping, controller.get('drag')),
                'mit_tracking_validation': hold_scope,
                'mit_mode_validation': self._digest(mapping, controller, gravity_hash),
                'csp_drag_validation': response,
                'upper_layer_validation': self._digest(hold_scope, safety, response if path == 'CSP' else controller),
                'parking_validation': self._digest(hold_scope,
                    {key: value for key, value in parking_plan(core, path).items() if key != 'enabled'},
                    controller.get('compliant') if path == 'MIT' and parking_plan(core, path).get('strategy') == 'COMPLIANT_THEN_RIGID' else None,
                    response if path == 'CSP' and parking_plan(core, path).get('strategy') == 'COMPLIANT_THEN_RIGID' else None, gravity_hash, safety,
                    {key: runtime.get(key) for key in ('ctrl_frequency_hz', 'joint_acc_filter_alpha', 'model_feedforward_mode')}),
                'gravity_correction': gravity_hash, 'dynamics_calibration': dynamics_hash,
                'geometry_calibration': self._digest(geometry, calibration),
                'position_repeatability': self._digest(hold_scope, geometry, model.get('tool_frame')),
                'absolute_accuracy': self._digest(geometry, calibration)}
            for stage, scope in domain.items():
                scopes[commissioning_stage_key(stage, path)] = scope
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
                # existing dependency scope proves this Profile owns them
                selected = {}
                for category in ('stages', 'modes', 'gain_purposes'):
                    entries = record.get(category) or {}
                    for key, item in entries.items() if isinstance(entries, dict) else ():
                        expected = ((scopes or {}).get(key) if category == 'stages' else
                            (scopes or {}).get('startup_hold') if category == 'modes' and key == 'RIGID' else
                            ((scopes or {}).get(category) or {}).get(key))
                        if expected and isinstance(item, dict) and item.get('scope') == expected:
                            selected.setdefault(category, {})[key] = item
            for category in ('stages', 'modes', 'gain_purposes', 'measurements'):
                entries = selected.get(category) or {}
                if isinstance(entries, dict):
                    for key, item in entries.items():
                        if source != path and key not in (meta.get(category) or {}):
                            imported = True
                        meta.setdefault(category, {}).setdefault(key, item)
            if source == path:
                for key, value in record.items():
                    if key not in ('stages', 'modes', 'gain_purposes', 'measurements') and key not in meta:
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
                    'modes': {}, 'modes_by_path': {'MIT': {}, 'CSP': {}}, 'reason': str(error), 'fingerprint': ''}
        checks = {item['name']: bool(item['ok']) for item in info.get('checks', [])}
        try:
            authored_core = read_yaml(info['resources']['core']) if checks.get('core') else {}
            core = canonical_core(authored_core)
        except (ValueError, TypeError, KeyError) as error:
            return {'state': 'configuring', 'verification_state': 'pending', 'next_stage': 'profile_setup',
                    'checks': {}, 'modes': {}, 'modes_by_path': {'MIT': {}, 'CSP': {}}, 'gain_purposes': {}, 'control_available': False,
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
        # Offline simulation needs a valid Core model, not a made-up motor type
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
            if isinstance(item, dict) and item.get('result') == 'failed':
                return 'failed' if item.get('scope') == scope else 'stale'
            if not isinstance(item, dict) or item.get('result') not in ('passed', 'recorded', 'warning'):
                return 'pending'
            if item.get('result') == 'warning' and item.get('kind') != 'measured_gain_purpose':
                return 'pending'
            if item.get('scope') != scope:
                return 'stale'
            # Existing historical checklist entries are still user reports,
            # even if they were recorded while a real robot was connected.
            # Older native gravity samples include a true Machine proof; do
            # not demote them to self-check notes when upgrading the UI
            if (item.get('kind') == 'operator_and_live' and
                    (item.get('runtime') or {}).get('sample_count', 0) >= 3):
                return 'verified'
            if (item.get('result') == 'recorded' and not str(item.get('kind', '')).startswith(('measured_', 'passive_')) or
                    str(item.get('kind', '')).startswith('operator_')):
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
            artifact = item.get('artifact_path')
            if artifact and (not Path(artifact).is_file() or _sha(artifact) != item.get('artifact_sha256')):
                return 'stale'
            if any(not Path(path).is_file() or _sha(path) != digest
                   for path, digest in (item.get('artifact_files') or {}).items()):
                return 'stale'
            if item.get('kind') == 'passive_response_record':
                return 'historical'
            return 'warning' if item.get('result') == 'warning' else 'partial' if item.get('result') == 'recorded' else 'verified'
        values = {
            'description': 'verified' if model_valid else 'failed',
            'profile_setup': 'verified' if profile_setup else 'pending',
        }
        # Independent evidence describes what was checked, not what may be opened
        for key in self._STAGES:
            values[key] = status(stages.get(key), scopes.get(key))
        hold = status(modes.get('RIGID'), scopes.get('startup_hold'))
        controller = (core.get('control') or {}).get('controller') or {}
        safety = core.get('safety_policy') or {}
        runtime = ((core.get('control') or {}).get('runtime') or {})
        tracking = safety.get('max_tracking_error')
        interaction = interaction_configuration(core) if profile_setup else {'observation_configured': False, 'admittance_configured': False}
        csp_ready = (profile_setup and bool(joints) and
            isinstance(tracking, dict) and set(tracking) == set(joints)
            and all(isinstance(tracking[j], (int, float)) and not isinstance(tracking[j], bool)
                    and math.isfinite(tracking[j]) and tracking[j] > 0 for j in joints))
        def configured_modes(path):
            result = {}
            for name in self._MODES:
                if path == 'CSP':
                    # MIT gain evidence does not qualify a different execution path.
                    ready = csp_ready and (name in ('RIGID', 'SERVO_TRACKING') or interaction['admittance_configured'])
                    result[name] = 'configured' if ready else 'unconfigured'
                    continue
                key = self._MODE_KEYS[name]
                gains = (controller.get('rigid')
                         if name == 'SERVO_TRACKING' else controller.get(key)) or {}
                valid = (bool(joints) and all(
                    isinstance(gains.get(axis), dict) and set(gains[axis]) == set(joints)
                    and all(isinstance(gains[axis][j], (int, float)) and not isinstance(gains[axis][j], bool)
                            and math.isfinite(gains[axis][j]) and gains[axis][j] >= 0
                            for j in joints) for axis in ('kp', 'kd')))
                if valid and name in ('RIGID', 'SERVO_TRACKING'):
                    valid = all(gains['kp'][j] > 0 and gains['kd'][j] > 0 for j in joints)
                if valid and name == 'DRAG':
                    valid = all(gains['kp'][j] == 0 and gains['kd'][j] > 0 for j in joints)
                scope = scopes.get('startup_hold') if name == 'RIGID' else (scopes.get('modes') or {}).get(name)
                own = status(modes.get(name), scope)
                result[name] = (own if own in ('verified', 'warning', 'stale') else 'configured') if valid else 'unconfigured'
            return result
        modes_by_path = {path: configured_modes(path) for path in ('MIT', 'CSP')}
        mode_states = modes_by_path.get(runtime.get('control_path', 'MIT'), {})
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
        gain_purposes = self._gain_purpose_status(core, meta, scopes, modes_by_path['MIT'])
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
            'max_tracking_error': tracking,
            'state_timeout_s': safety.get('state_timeout_s'),
            'cmd_timeout_s': safety.get('cmd_timeout_s'),
            'require_continuous_cmd': safety.get('require_continuous_cmd'),
            'fault_recovery': dict(safety.get('fault_recovery') or {}),
            'urdf_limits': urdf_limits,
            'min_pos': [item[0] for item in position_ranges],
            'max_pos': [item[1] for item in position_ranges],
            'pos_margin': [safety['position_margin'].get(name, 0) if isinstance(safety.get('position_margin'), dict)
                           else None for name in joints],
            'has_position_limit': [all(value is not None and math.isfinite(value) for value in item) for item in position_ranges],
        }
        prerequisites = []
        def prerequisite(key, label, ok, field, group):
            prerequisites.append({'id': key, 'label': label, 'ready': bool(ok),
                'action': {'page': 'profile_setup', 'group': group, 'field': field, 'label': '配置' + label}})
        prerequisite('write_enabled', '控制写入', runtime.get('write_enabled') is True,
            'core.control.runtime.write_enabled', 'motion')
        drag = controller.get('drag') or {}
        kd = drag.get('kd') or {}
        if runtime.get('control_path', 'MIT') == 'MIT':
            prerequisite('drag_damping', '拖拽阻尼', bool(joints) and all(
                isinstance(kd.get(j), (int, float)) and not isinstance(kd.get(j), bool) and
                math.isfinite(kd[j]) and kd[j] > 0 for j in joints),
                'core.control.controller.drag.kd', 'motion')
        else:
            prerequisite('observation', '外力观察参数', interaction['observation_configured'],
                'core.capability.admittance.observer.momentum_gain', 'observation')
        gravity = model.get('gravity')
        scales = model.get('gravity_scale') or {}
        gravity_ready = (isinstance(gravity, list) and len(gravity) == 3 and
            all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in gravity) and
            sum(v*v for v in gravity) > 1e-12 and bool(joints) and
            all(isinstance(scales.get(j), (int, float)) and not isinstance(scales.get(j), bool) and
                math.isfinite(scales[j]) and scales[j] > 0 for j in joints))
        prerequisite('gravity', '重力模型与补偿比例', gravity_ready,
            'core.model.gravity' if not isinstance(gravity, list) or len(gravity) != 3 or not gravity_ready and not any(gravity or []) else 'core.model.gravity_scale', 'model')
        prerequisite('hold_gains', '刚性保持增益', mode_states.get('RIGID') != 'unconfigured',
            'core.control.controller.rigid.kp', 'motion')
        calibration_ready = profile_setup and all(item['ready'] for item in prerequisites)
        selected_path = runtime.get('control_path', 'MIT')
        trusted = gravity_trusted(core)
        validation = ((core.get('capability') or {}).get('admittance') or {}).get('validation') or {}
        operation_readiness = {
            'basic_control': {'ready': profile_setup and mode_states.get('RIGID') != 'unconfigured',
                              'reason': reason or ('' if mode_states.get('RIGID') != 'unconfigured' else '请配置当前执行路径的刚性控制参数')},
            'gravity_verification': {'ready': calibration_ready,
                'reason': '' if calibration_ready else '请补齐当前路径的基础控制、模型和采样参数'},
            'drag': {'ready': profile_setup and mode_states.get('DRAG') != 'unconfigured' and
                              (trusted if selected_path == 'MIT' else all(validation.get(key) is True for key in ('observation_validated', 'response_validated'))),
                'reason': ('请先验证当前模型与负载的重力补偿' if selected_path == 'MIT' and not trusted else
                           '请先完成独立外力和有限刚度响应验收' if selected_path == 'CSP' and not all(validation.get(key) is True for key in ('observation_validated', 'response_validated')) else
                           '' if mode_states.get('DRAG') != 'unconfigured' else '请配置当前执行路径的拖拽参数')},
            'observation': {'ready': profile_setup and interaction['observation_configured'],
                'reason': '' if interaction['observation_configured'] else '请显式配置观察器与外力校准参数',
                'calibration_status': 'unverified' if interaction['observation_configured'] else 'unconfigured',
                'validated': validation.get('observation_validated') is True},
            'admittance': {'ready': csp_ready and interaction['admittance_configured'] and
                                   validation.get('observation_validated') is True and validation.get('response_validated') is True,
                'configured': csp_ready and interaction['admittance_configured'],
                'reason': '' if csp_ready and interaction['admittance_configured'] and all(validation.get(key) is True for key in ('observation_validated', 'response_validated'))
                          else '需要 CSP、跟踪误差限制、观察与 MDK 参数及独立外力和响应验收'},
        }
        parking_by_path = {}
        for path in ('MIT', 'CSP'):
            plan = parking_plan(core, path)
            strategy = plan.get('strategy', 'RIGID')
            flexible = strategy == 'COMPLIANT_THEN_RIGID'
            complete = parking_complete(core, path)
            rigid_ready = modes_by_path[path].get('RIGID') != 'unconfigured'
            flexible_ready = modes_by_path[path].get('COMPLIANT') != 'unconfigured' and (
                path == 'MIT' or all(validation.get(key) is True for key in ('observation_validated', 'response_validated')))
            configured = profile_setup and complete and rigid_ready and (not flexible or flexible_ready)
            enabled = plan.get('enabled') is True
            receipt = stages.get(commissioning_stage_key('parking_validation', path)) or {}
            validated = status(receipt, scopes.get(commissioning_stage_key('parking_validation', path))) not in ('pending', 'stale', 'failed') and all(
                (receipt.get('observations') or {}).get(key) is True for key, _ in COMMISSIONING_REVIEWS['parking_validation'][2])
            parking_by_path[path] = {'enabled': enabled, 'configured': bool(configured), 'strategy': strategy,
                'validated': validated,
                'reason': ('请在停放与退出策略中允许当前路径先停放再失能' if not enabled else
                    '请补齐停放目标、逐轴容差与时间参数' if not complete else
                    '请配置当前路径的刚性保持参数' if not rigid_ready else
                    '请配置 MIT 柔性增益' if flexible and not flexible_ready and path == 'MIT' else
                    'CSP 柔性停放需要导纳参数及外力、响应验收' if flexible and not flexible_ready else
                    '当前 Profile 配置尚未就绪' if not profile_setup else '')}
        configuration = {'source': 'saved_profile', 'core_path': info['resources']['core'],
            'control_path': selected_path, 'joint_names': joints, 'model': bool(model_valid),
            'hardware': bool(profile_setup), 'safety': bool(safety_configured and profile_setup),
            'hold': bool(joints) and mode_states.get('RIGID') in ('configured', 'verified', 'warning', 'stale'),
            'gravity': bool(gravity_ready), 'observation': bool(interaction['observation_configured']),
            'response': bool(interaction['admittance_configured']),
            'drag': bool(joints) and mode_states.get('DRAG') in ('configured', 'verified', 'warning', 'stale'),
            'mit_modes': all(modes_by_path['MIT'].get(mode) != 'unconfigured' for mode in self._MODES),
            'parking': parking_by_path[selected_path]['configured'], 'parking_by_path': parking_by_path,
            'validation': dict(validation)}
        commissioning = commissioning_snapshot(core, configuration, meta, scopes, status,
                                               gravity_validated=trusted, modes_by_path=modes_by_path)
        parking = parking_by_path[selected_path]
        operation_readiness['parking'] = {'allowed': parking['enabled'] and parking['configured'],
            'reason': parking['reason'], 'strategy': parking['strategy'], 'control_path': selected_path}
        for stage in COMMISSIONING_REVIEWS:
            key = commissioning_stage_key(stage, selected_path)
            entry = stages.get(key) or stages.get(stage)
            if stage == 'startup_hold' and not entry:
                entry = modes.get('RIGID')
                values[stage] = status(entry, scopes.get('startup_hold'))
            else:
                values[stage] = status(entry, scopes.get(key))
        required_steps = [step for group in commissioning['groups'] if group['required'] for step in group['steps']]
        verification_state = ('verified' if commissioning['completed'] else
            'stale' if any(step['evidence_status'] == 'stale' for step in required_steps) else
            'partial' if any(step['evidence_status'] in ('recorded', 'partial', 'verified') for step in required_steps) else 'pending')
        response_records = []
        for record in meta.get('response_records') or []:
            if not isinstance(record, dict): continue
            item = dict(record)
            item['status'] = status(item, scopes.get(commissioning_stage_key('admittance_response', item.get('control_path', selected_path))))
            response_records.append(item)
        return {'state': state, 'verification_state': verification_state, 'next_stage': 'profile_setup' if not profile_setup else 'ready',
                'checks': values, 'modes': mode_states, 'modes_by_path': modes_by_path, 'reason': reason,
                'gain_initialization': meta.get('gain_initialization'),
                'fingerprint': info['fingerprint'], 'info': info,
                'control_available': profile_setup, 'safety_summary': safety_summary,
                'operation_readiness': operation_readiness,
                'connection_readiness': {'structurally_valid': profile_setup, 'read_only_connectable': offline_ready,
                    'offline_connectable': offline_ready, 'hardware_connectable': profile_setup,
                    'offline_issues': core_issues,
                    'write_enabled': runtime.get('write_enabled') is True, 'basic_calibration_ready': calibration_ready,
                    'basic_calibration_prerequisites': prerequisites,
                    'next_actions': [item['action'] for item in prerequisites if not item['ready']]},
                'setup_issues': setup_issues, 'next_actions': next_actions, 'gain_purposes': gain_purposes,
                'source_trusted': self._is_official_profile(info),
                'configuration_summary': configuration, 'commissioning': commissioning,
                'review_requirements': review_requirements(selected_path, joints), 'response_records': response_records,
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

    @staticmethod
    def _matches_admittance_snapshot(core, runtime, *, response=False):
        if not isinstance(runtime, dict): return False
        names = (core.get('model') or {}).get('joint_names') or []
        admittance = (core.get('capability') or {}).get('admittance') or {}
        for section, fields in (('observer', ('momentum_gain',)),
                                ('calibration', ('torque_bias', 'torque_threshold')),
                                ('controller', ('mass', 'damping', 'stiffness', 'max_delta_q', 'max_delta_q_dot') if response else ())):
            for field in fields:
                expected = (admittance.get(section) or {}).get(field)
                if not isinstance(expected, dict) or runtime.get(field) != [expected.get(name) for name in names]:
                    return False
        friction = (admittance.get('calibration') or {}).get('friction') or {}
        actual_friction = runtime.get('friction') or {}
        if actual_friction.get('enabled', False) != friction.get('enabled', False): return False
        if friction.get('enabled'):
            for field in ('velocity_transition', 'positive_coulomb', 'positive_viscous', 'negative_coulomb', 'negative_viscous'):
                if actual_friction.get(field) != [(friction.get(field) or {}).get(name) for name in names]: return False
        if response:
            if [bool(value) for value in runtime.get('joint_enabled', [])] != [(admittance.get('joint_enabled') or {}).get(name) for name in names]:
                return False
        return True

    @staticmethod
    def _validation_text(core_text, changes):
        from persistence import patch_text
        additions = {'capability': {'admittance': {'validation': {key: False for key in changes}}}}
        expanded = complete_mapping_values(core_text, (), additions)
        return patch_text(expanded, {('capability', 'admittance', 'validation', key): 'true' if value else 'false'
                                     for key, value in changes.items()})

    def record_observation_measurement(self, config, native, native_fingerprint):
        """Record only the independent native validation, never a fitted result."""
        info = self.inspector.inspect(config)
        if native is None or native_fingerprint != info.get('fingerprint'):
            raise ValueError('独立外力验收要求与当前 Profile 配置一致的原生会话')
        core_path = Path(info['resources']['core'])
        core_text = core_path.read_text(encoding='utf-8')
        core = canonical_core(yaml.safe_load(core_text))
        status = native.request('status', {}, timeout=8)
        overrides = native.request('candidate_overrides', {}, timeout=8)
        calibration = status.get('calibration') or {}
        validation = calibration.get('validation') or {}
        if (info.get('write_enabled') is not True or status.get('hardware_connected') is not True or
                status.get('control_path') != 'CSP' or core['control']['runtime']['control_path'] != 'CSP' or
                (calibration.get('kind') != 'validation' and calibration.get('completed_kind') != 'validation') or
                calibration.get('phase') not in ('complete', 'completed') or
                validation.get('independent') is not True):
            raise ValueError('请先在真实硬件上完成独立静态验证；拟合和人工自查不授予外力质量许可')
        raw = native.request('calibration_samples', {}, timeout=15)
        count = len((core.get('model') or {}).get('joint_names') or [])
        period = 1.0 / core['control']['runtime']['ctrl_frequency_hz']
        maximum_sample_gap = core['safety_policy']['state_timeout_s']
        poses = raw.get('poses') or []
        raw_complete = len(poses) >= 5
        for pose in poses:
            samples = pose.get('samples') or []
            times = [sample.get('time_s') for sample in samples]
            if (len(times) < 3 or any(not isinstance(value, (int, float)) or not math.isfinite(value) for value in times) or
                    any(b <= a or b - a > maximum_sample_gap for a, b in zip(times, times[1:])) or
                    times[-1] - times[0] < 15 - 2 * period or
                    any(not isinstance(sample.get(field), list) or len(sample[field]) != count or
                        any(not isinstance(value, (int, float)) or not math.isfinite(value) for value in sample[field])
                        for sample in samples for field in ('position', 'gravity', 'measured_torque'))):
                raw_complete = False
        passed = validation.get('pass')
        qualified = (validation.get('validated') is True and validation.get('pose_count', 0) >= 5 and
                     validation.get('static_window_s', 0) >= 15 and isinstance(passed, list) and len(passed) == count and
                     all(value in (True, 1) for value in passed) and raw_complete and
                     not any(overrides.get(kind + '_applied') for kind in ('gain', 'gravity', 'dynamics')) and
                     status.get('joint_names') == core['model']['joint_names'] and
                     status.get('gravity_scale') == [core['model']['gravity_scale'][name] for name in core['model']['joint_names']] and
                     self._matches_admittance_snapshot(core, validation.get('scope_config')))
        reason = validation.get('reason') or ''
        if not raw_complete:
            reason = '原始采样未覆盖 5 个独立姿态、每姿态 15 s 连续有效数据，请检查采样间隔和反馈超时'
        elif any(overrides.get(kind + '_applied') for kind in ('gain', 'gravity', 'dynamics')):
            reason = '临时模型或增益候选不能授予已保存配置的质量许可'
        elif status.get('joint_names') != core['model']['joint_names']:
            reason = '实测关节与已保存模型不一致'
        elif (status.get('gravity_scale') != [core['model']['gravity_scale'][name] for name in core['model']['joint_names']] or
              not self._matches_admittance_snapshot(core, validation.get('scope_config'))):
            reason = '实测模型或外力估计参数与已保存配置不一致'
        hardware = read_yaml(info['resources']['hardware'])
        scopes = self._dependencies(info, core, hardware)
        key = commissioning_stage_key('observation_validation', 'CSP')
        path, meta = self._readiness_meta(info, scopes)
        artifact = path.parent / ('observation-' + str(time.time_ns()) + '.json')
        document = {'profile': info['profile'], 'control_path': 'CSP', 'scope': scopes[key],
                    'recorded_at': time.time(), 'validation': validation, 'raw': raw}
        _atomic_yaml_text(artifact, json.dumps(document, ensure_ascii=False, allow_nan=False))
        previous = (meta.get('measurements') or {}).get(key)
        if previous: meta.setdefault('measurement_history', []).append({'stage': key, **previous})
        receipt = {'kind': 'measured_observation_validation', 'result': 'passed' if qualified else 'failed',
            'scope': scopes[key], 'recorded_at': document['recorded_at'], 'control_path': 'CSP',
            'artifact_path': str(artifact), 'artifact_sha256': _sha(artifact),
            'summary': {**validation, 'validated': qualified, 'direction_check': 'operator_report_separate', 'reason': reason}}
        meta.setdefault('measurements', {})[key] = receipt
        _atomic_yaml(path, meta)
        if _sha(core_path) != hashlib.sha256(core_text.encode()).hexdigest():
            raise ValueError('保存验收期间配置已变化，原始记录已保留，请重新验证')
        # Static measurements cannot establish whether a human applied load in
        # the intended direction or whether the mechanism was contact-free.
        permitted = qualified and observation_confirmed(meta.get('stages') or {}, scopes)
        changes = {'observation_validated': permitted, 'response_validated': False}
        current = ((core.get('capability') or {}).get('admittance') or {}).get('validation') or {}
        updates = {key: value for key, value in changes.items() if (current.get(key) is True) != value}
        if updates: _atomic_yaml_text(core_path, self._validation_text(core_text, changes))
        result = self._readiness(config)
        result.update(measurement=receipt, requires_reconnect=bool(updates), observation_validated=permitted)
        self._inspection_result(result, 'observation_validation', updates)
        return result

    @staticmethod
    def _observation_permission(meta, scopes):
        key = commissioning_stage_key('observation_validation', 'CSP')
        receipt = (meta.get('measurements') or {}).get(key) or {}
        artifact = Path(receipt.get('artifact_path') or '/missing')
        return (receipt.get('kind') == 'measured_observation_validation' and receipt.get('result') == 'passed' and
                receipt.get('scope') == scopes.get(key) and (receipt.get('summary') or {}).get('validated') is True and
                artifact.is_file() and _sha(artifact) == receipt.get('artifact_sha256') and
                observation_confirmed(meta.get('stages') or {}, scopes))

    @staticmethod
    def _inspection_result(readiness, task, changes=None):
        step = next((item for group in (readiness.get('commissioning') or {}).get('groups', [])
                     for item in group['steps'] if item['id'] == task or task in item['review_stages']), None)
        readiness['inspection_result'] = {'task_id': step['id'] if step else task,
            'status': step['evidence_status'] if step else 'partial', 'completed': bool(step and step['completed']),
            'remaining': step['blockers'] if step else [], 'permission_updates': changes or {}}

    def record_response_measurement(self, config, window, export_path, *, native=None, native_fingerprint=None):
        from interaction_response import validate_admittance_response, native_response_binding
        info = self.inspector.inspect(config)
        core_path = Path(info['resources']['core'])
        core_text = core_path.read_text(encoding='utf-8')
        core = canonical_core(yaml.safe_load(core_text))
        hardware = read_yaml(info['resources']['hardware'])
        scopes = self._dependencies(info, core, hardware)
        control_path = window.get('behavior', window.get('control_path', 'MIT'))
        key = commissioning_stage_key('admittance_response', control_path)
        binding_error = ''
        if (window.get('profile') != info['profile'] or window.get('joint_names') != core['model']['joint_names'] or
                Path(window.get('core') or '/missing').resolve() != core_path.resolve()):
            binding_error = '记录不属于当前 Profile，仅保留历史数据'
        elif native is None or native_fingerprint != info.get('fingerprint'):
            binding_error = '当前配置与原生会话未绑定，仅保留历史数据'
        elif not self._matches_admittance_snapshot(core, window.get('parameters'), response=True):
            binding_error = '试验参数与已保存参数不一致，请先保存并重新验证'
        elif window.get('gravity_scale') != [core['model']['gravity_scale'][name] for name in core['model']['joint_names']]:
            binding_error = '试验重力模型比例与已保存配置不一致'
        else:
            overrides = native.request('candidate_overrides', {}, timeout=8)
            binding_error = ('临时模型或增益候选不能授予已保存配置的质量许可'
                if any(overrides.get(kind + '_applied') for kind in ('gain', 'gravity', 'dynamics')) else
                native_response_binding(window, native.telemetry(), native.status()))
        quality = validate_admittance_response(window, core)
        quality['binding_error'] = binding_error
        path, meta = self._readiness_meta(info, scopes)
        artifact = Path(export_path).resolve()
        receipt = {'id': artifact.stem, 'kind': 'passive_response_record' if binding_error else 'measured_admittance_response',
            'result': 'recorded' if binding_error else 'passed' if quality.get('passed_axes') else 'failed',
            'scope': scopes.get(key, ''), 'recorded_at': time.time(), 'control_path': control_path,
            'artifact_path': str(artifact), 'path': str(artifact), 'artifact_sha256': _sha(artifact),
            'sample_count': len(window.get('samples') or []), 'summary': quality}
        meta.setdefault('response_records', []).append(receipt)
        qualified = False
        if not binding_error:
            previous = (meta.get('measurements') or {}).get(key) or {}
            covered, files = set(), {}
            if (quality.get('eligible') and previous.get('scope') == scopes.get(key) and
                    all(Path(name).is_file() and _sha(name) == digest for name, digest in (previous.get('artifact_files') or {}).items())):
                covered.update((previous.get('summary') or {}).get('passed_axes') or [])
                files.update(previous.get('artifact_files') or {})
            covered.update(quality.get('passed_axes') or [])
            if quality.get('passed_axes'): files[str(artifact)] = _sha(artifact)
            qualified = bool(quality.get('eligible') and quality.get('required_axes') and set(quality['required_axes']) <= covered)
            receipt['summary'] = {**quality, 'window_passed_axes': quality.get('passed_axes', []),
                                  'passed_axes': sorted(covered), 'validated': qualified}
            receipt['artifact_files'] = files
            receipt['result'] = 'passed' if qualified else 'recorded' if covered else 'failed'
            if previous: meta.setdefault('measurement_history', []).append({'stage': key, **previous})
            meta.setdefault('measurements', {})[key] = receipt
        _atomic_yaml(path, meta)
        if not binding_error and _sha(core_path) == hashlib.sha256(core_text.encode()).hexdigest():
            validation = ((core.get('capability') or {}).get('admittance') or {}).get('validation') or {}
            qualified = qualified and validation.get('observation_validated') is True
            _atomic_yaml_text(core_path, self._validation_text(core_text, {'response_validated': qualified}))
        return {'receipt': receipt, 'requires_reconnect': not bool(binding_error), 'response_validated': qualified}

    def record_stage(self, config, stage, answers, native=None, native_fingerprint=None):
        """Save scoped inspections without requiring an active hardware session.

        Measured permission combines valid native proof with its required
        operator confirmations; a checklist alone cannot supply that proof.
        """
        if not isinstance(answers, dict):
            raise ValueError('检查内容必须是对象')
        info = self.inspector.inspect(config)
        core = read_yaml(info['resources']['core'])
        selected_path = ((core.get('control') or {}).get('runtime') or {}).get('control_path', 'MIT')
        records = task_answers(stage, answers, selected_path, (core.get('model') or {}).get('joint_names') or [])
        if records is not None:
            hardware = read_yaml(info['resources']['hardware'])
            scopes = self._dependencies(info, core, hardware)
            _, meta = self._readiness_meta(info, scopes)
            changed, reconnect = {}, False
            for source, fields in records.items():
                key = commissioning_stage_key(source, selected_path)
                previous = (meta.get('stages') or {}).get(key) or {}
                # Retain old fields only while their physical scope still fits.
                existing = previous.get('observations') or {} if previous.get('scope') == scopes.get(key) else {}
                result = self.record_stage(config, source, {**existing, **fields, 'note': answers.get('note', '')}, native, native_fingerprint)
                changed.update((result.get('inspection_result') or {}).get('permission_updates') or {})
                reconnect |= result.get('requires_reconnect') is True
            result = self._readiness(config)
            result['requires_reconnect'] = reconnect
            self._inspection_result(result, stage, changed)
            return result
        requirements = {key: tuple(item[0] for item in review[2]) for key, review in COMMISSIONING_REVIEWS.items()}
        requirements['startup_hold'] += tuple('joint:' + name for name in (core.get('model') or {}).get('joint_names') or [])
        if stage not in requirements:
            raise ValueError('当前项目不支持人工自查记录')
        note = str(answers.get('note') or '').strip()
        if len(note) > 600:
            raise ValueError('检查备注不能超过 600 字')
        checked = {}
        for key in requirements[stage]:
            value = answers.get(key, answers.get('bidirectional') is True if stage == 'startup_hold' and key.startswith('joint:') else False)
            if not isinstance(value, bool):
                raise ValueError(f'检查项 {key} 必须为布尔值')
            checked[key] = value

        # Only the Profile and its configuration dependencies are required to
        # associate a note with the right robot; no hardware session is needed
        info = self.inspector.inspect(config)
        core_path = Path(info['resources']['core'])
        core_text = core_path.read_text(encoding='utf-8')
        core = yaml.safe_load(core_text)
        core_sha = hashlib.sha256(core_text.encode('utf-8')).hexdigest()
        hardware = read_yaml(info['resources']['hardware'])
        scopes = self._dependencies(info, core, hardware)
        path, data = self._readiness_meta(info, scopes)
        data['profile'] = info['profile']
        # Notes and configured MIT gains have independent lifetimes. A startup
        # checklist must never replace a measured gain receipt in modes.RIGID.
        field = 'stages'
        selected_path = ((core.get('control') or {}).get('runtime') or {}).get('control_path', 'MIT')
        key = commissioning_stage_key(stage, selected_path)
        scope = scopes.get(key, scopes.get(stage))
        previous = (data.get(field) or {}).get(key)
        # Updating optional notes must not erase independent native proof
        # that still matches the current configuration
        keep_native = (stage == 'gravity_verification' and isinstance(previous, dict)
            and previous.get('result') == 'passed'
            and previous.get('scope') == scope
            and previous.get('kind') in ('native_gravity_completed', 'operator_and_live')
            and (previous.get('runtime') or {}).get('sample_count', 0) >= 3)
        entry = dict(previous) if keep_native else {
            'result': 'recorded', 'scope': scope, 'kind': 'operator_note',
        }
        if isinstance(previous, dict) and previous.get('scope') != scope:
            data.setdefault('stage_history', []).append({'stage': key, **previous})
        entry.update({'note': note, 'observations': checked, 'recorded_at': time.time(), 'control_path': selected_path})
        data.setdefault(field, {})[key] = entry
        # A real completed native gravity task may provide stronger evidence.
        # Missing, stale or unrelated sessions never block saving a note and
        # must never transform self-reported checkboxes into a native proof
        accept_gravity = False
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
                    native_fingerprint is not None and info.get('fingerprint') == native_fingerprint and
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
                    if sample.get('control_path', 'MIT') == 'MIT':
                        data.setdefault('modes', {})['DRAG'] = {
                            'result': 'passed', 'scope': scopes['modes']['DRAG'],
                            'kind': 'native_gravity_completed',
                            'sample_count': proof['sample_count'],
                            'recorded_at': time.time(),
                        }
                    accept_gravity = (all(checked.values()) and native_fingerprint is not None and
                                      info.get('fingerprint') == native_fingerprint)
            except Exception:
                # There is no usable live native proof; keep only the note
                pass
        _atomic_yaml(path, data)
        permission_changes = {}
        if stage in ('observation_validation', 'observation_calibration', 'effort_model_validation') and selected_path == 'CSP':
            permitted = self._observation_permission(data, scopes)
            current = ((core.get('capability') or {}).get('admittance') or {}).get('validation') or {}
            changes = {'observation_validated': permitted,
                       'response_validated': current.get('response_validated') is True and permitted}
            permission_changes = {key: value for key, value in changes.items() if (current.get(key) is True) != value}
            if permission_changes:
                if _sha(core_path) != core_sha:
                    raise ValueError('保存检查期间配置已变化，记录已保留，请重新检查')
                _atomic_yaml_text(core_path, self._validation_text(core_text, changes))
        trust_saved = accept_gravity and not gravity_trusted(core)
        if trust_saved:
            # The dedicated task measured this bound model; the final operator
            # acknowledgment persists only its trust flag. Loading it into the
            # control session remains an explicit reconnect.
            if _sha(core_path) != core_sha:
                raise ValueError('验证期间模型配置已变化，请重新连接并验证后再确认')
            from persistence import patch_text
            trusted_text = complete_mapping_values(core_text, ('model',), {'gravity_model_validated': False})
            trusted_text = patch_text(trusted_text, {('model', 'gravity_model_validated'): 'true'})
            _atomic_yaml_text(core_path, trusted_text)
        readiness = self._readiness(config)
        if trust_saved: permission_changes['gravity_model_validated'] = True
        readiness.update(gravity_validation_saved=trust_saved, requires_reconnect=trust_saved or bool(permission_changes))
        self._inspection_result(readiness, stage, permission_changes)
        return readiness

    @staticmethod
    def _gain_interaction_context(core, purpose):
        # Receipt scopes retain their existing saved-configuration semantics;
        # actual measured context is checked separately by _gain_binding
        admittance = ((core.get('capability') or {}).get('admittance') or {})
        if purpose == 'SERVO_TRACKING':
            return {key: admittance.get(key) for key in ('joint_enabled', 'observer', 'calibration', 'controller')}
        if purpose == 'RIGID' and ((core.get('control') or {}).get('hold_compensation') or {}).get('enabled'):
            return {key: admittance.get(key) for key in ('observer', 'calibration')}
        return None

    def _gain_binding(self, folder, info, core, hardware, purposes):
        from gain_context import verify_gain_context
        files = verify_gain_context(folder, info, core, hardware, purposes)
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
        targets, _ = gain_candidate_targets(candidate, core, core['model']['joint_names'], selection, hardware)
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
                mode_scope = scopes['startup_hold'] if key == 'RIGID' else scopes['modes'][key]
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
            # change independently, so always evaluate Xacro against current resources
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
        """Keep copied models usable after their original workspace is removed"""
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
            # Preserve the standard relative material/texture dependencies of OBJ and DAE assets
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
        # Parse using generated resources only, with no fallback to the source workspace
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
            # Zero placeholders are replaced by model-derived per-axis suggestions below
            kp_rigid=zeros.copy(); kd_rigid=zeros.copy(); kp_soft=zeros.copy(); kd_soft=zeros.copy()
            core={
              'model':{'urdf_path':'../../'+analysis['urdf_relative'].replace('\\','/'),'joint_names':joints,'base_frame':base,'tool_frame':tool,'gravity':[0.0,0.0,-9.81],'gravity_scale':ones,'gravity_model_validated':False},
              'calibration':{'joints':{j:{'direction':1.0,'pos_ratio':1.0,'tor_ratio':1.0,'joint_zero_offset':0.0,'actuator_zero_offset':0.0} for j in joints}},
              'control':{'runtime':{'ctrl_frequency_hz':200.0,'joint_acc_filter_alpha':0.2,'write_enabled':False,'model_feedforward_mode':'GRAVITY','control_path':'MIT'},'controller':{'allow_full_cmd':False,'rigid':{'kp':kp_rigid,'kd':kd_rigid},'compliant':{'kp':kp_soft,'kd':kd_soft},'drag':{'kp':zeros,'kd':zeros.copy()}}},
              'safety_policy':{'position_margin':zeros.copy(),'cmd_vel_scale':0.25,'state_vel_scale':1.0,'max_acc':self._joint_map(joints,2.0),'max_dt_s':0.02,'state_timeout_s':0.05,'cmd_timeout_s':0.10,'require_all_actuators_online':True,'require_all_actuators_enabled':True,'reject_motor_error':True,'require_continuous_cmd':False,'fault_recovery':{'default_mode':'rigid_hold','allow_drag_recovery':True,'require_operator_request':True,'recovery_timeout_s':30.0,'velocity_tolerance':self._joint_map(joints,.05)}},
              'shutdown': {'paths': {path: {'enabled': False, 'strategy': 'RIGID',
                  'park_pos': self._joint_map(joints, 0.0), 'speed_scale': 0.05,
                  'position_tolerance': self._joint_map(joints, .05), 'velocity_tolerance': self._joint_map(joints, .05),
                  'settle_time_s': 0.25, 'timeout_s': 10.0} for path in ('MIT', 'CSP')}},

            }
            # If the Description already supplies a robot-specific Core config,
            # preserve its gains; new automatic calibration independently derives
            # its model seed and never treats those gains as measured baseline
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
                # Normalize only the newly copied package, never its source
                authored_config.write_text(four_mode_control_text(authored_config.read_text()), encoding='utf-8')
                # Keep an already included legacy copy consistent, for consumers
                # still accessing config/serialarm/core.yaml directly
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
                hardware={'buses':{bus:{'type':'can','backend':'damiao_usb2can','device':device,'baudrate':baud}},'damiao':{'bus':bus,'refresh_state_in_read':False,'feedback_timeout_s':0.05,'activation_retries':3,'startup_read_cycles':5,'actuators':{j:{'name':text(a.get('name')) or f'actuator{i+1}','motor_id':int(a.get('motor_id') or i+1),'master_id':int(a.get('master_id') or 0),'motor_type':text(a.get('motor_type'))} for i,(j,a) in enumerate(zip(joints,actuators))}}}
            _atomic_yaml(config_dir/'hardware.yaml',hardware)
            initialization = None
            if not authored_config.is_file():
                from gain_initialization import initialize_profile_gains, MODEL_GAIN_REFERENCE_COMMENT
                initialization = initialize_profile_gains(self.inspector, config_dir/'core.yaml', core,
                    copied/analysis['urdf_relative'], hardware, hardware_plugin, apply_generated=True)
                _atomic_yaml_text(config_dir/'core.yaml', MODEL_GAIN_REFERENCE_COMMENT +
                    yaml.safe_dump(four_mode_core(core), allow_unicode=True, sort_keys=False))
            else:
                from gain_initialization import initialize_profile_gains
                initialization = initialize_profile_gains(self.inspector, authored_config,
                    canonical_core(read_yaml(authored_config)), copied/analysis['urdf_relative'], hardware, hardware_plugin)
                initialization['existing_configuration_preserved'] = True
            if initialization:
                (config_dir/'gain-initialization.json').write_text(json.dumps(initialization,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            entry={'core':{'package':analysis['package_name'],'config':core_config_relative},'hardware':{'plugin':hardware_plugin,'config_package':analysis['package_name'],'config':'config/serialarm/hardware.yaml'},'description':{'package':analysis['package_name'],'urdf':analysis['urdf_relative'].replace('\\','/')}}
            if analysis['ros2_control_xacro']:
                entry['description']['ros2_control_xacro']=str(Path(analysis['ros2_control_xacro']).relative_to(analysis['package_root'])).replace('\\','/')
            profile_file=package_dir/'robot_profiles.yaml'; _atomic_yaml(profile_file,{'profiles':{profile:entry}})
            readiness=self._readiness_path(profile_file, profile); _atomic_yaml(readiness,{'profile':profile,'managed_by_launcher':True,'source_urdf_fingerprint':analysis['fingerprint'],'stages':{},'gain_initialization':initialization})
            return {'package':str(package_dir),'profile_file':str(profile_file),'profile':profile,'resource_paths':str(package_dir),'hardware_complete':complete,'gain_initialization':initialization,'config':{'profile':profile,'profile_file':str(profile_file),'resource_paths':str(package_dir),'serial_port':'','baudrate':'','bus':''}}
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
                valid = valid and (value in (-1, 1) if key == 'direction' else value > 0 if key in ('pos_ratio', 'tor_ratio') else True)
                if not valid:
                    issue('mapping', f'core.calibration.joints.{joint}.{key}',
                          f'{joint} 的映射参数 {key} ' + ('必须为 1 或 -1' if key == 'direction' else '必须为正数，方向由 direction 指定' if key in ('pos_ratio', 'tor_ratio') else '必须为有限数值'))
        safety = core.get('safety_policy') or {}
        for key in ('max_dt_s', 'state_timeout_s', 'cmd_timeout_s', 'cmd_vel_scale', 'state_vel_scale'):
            value = safety.get(key)
            valid = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
            valid = valid and value > 0
            if key == 'cmd_vel_scale': valid = valid and value <= 1
            if not valid: issue('safety', 'core.safety_policy.' + key, f'安全参数 {key} 缺失或无效')
        for key in ('position_margin', 'fault_recovery.velocity_tolerance'):
            node = safety
            for part in key.split('.'): node = node.get(part, {}) if isinstance(node, dict) else {}
            if not isinstance(node, dict) or set(node) != set(names):
                issue('safety', 'core.safety_policy.' + key, '请为每个受控关节填写 ' + key)
        for path, reason in constraint_errors(core, hardware, entry):
            group = self._field_issue(path)['group']
            issue(group, path, reason)
        acceleration = safety.get('max_acc') or {}
        if not isinstance(acceleration, dict) or set(acceleration) != set(names):
            issue('safety','core.safety_policy.max_acc','Safety 必须为全部受控关节提供最大加速度 max_acc')
        else:
            for joint in names:
                value = acceleration[joint]
                if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or value<=0:
                    issue('safety',f'core.safety_policy.max_acc.{joint}',f'{joint} 的 Safety 最大加速度必须大于零')
        if ((core.get('control') or {}).get('runtime') or {}).get('control_path') == 'CSP':
            bounds = safety.get('max_tracking_error')
            for joint in names:
                value = bounds.get(joint) if isinstance(bounds, dict) else None
                if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                    issue('safety', f'core.safety_policy.max_tracking_error.{joint}',
                          f'CSP 需要为 {joint} 填写大于零的最大跟踪误差 max_tracking_error')
        return issues

    def profile_editor_load(self, config):
        info, core_path, hardware_path, profile_path = self._profile_config_paths(config)
        core = canonical_core(read_yaml(core_path))
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
            'field_constraints': field_constraints(core, hardware, profile, urdf_joint_limits),
            'original_gravity_model_validated': gravity_trusted(core),
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
    def _finite(value, path, minimum=None, maximum=None, integer=False, positive=False):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{path} 必须是有限数值')
        if integer and int(value) != value:
            raise ValueError(f'{path} 必须是整数')
        if minimum is not None and value < minimum:
            raise ValueError(f'{path} 不能小于 {minimum}')
        if maximum is not None and value > maximum:
            raise ValueError(f'{path} 不能大于 {maximum}')
        if positive and value <= 0:
            raise ValueError(f'{path} 必须大于 0')

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
        def scalar(path, minv=None, maxv=None, integer=False, positive=False):
            self._finite(section(path), path, minv, maxv, integer, positive)
        def joint_map(path, minv=None, maxv=None, integer=False, nonzero=False, positive=False):
            values = section(path)
            if not isinstance(values, dict) or set(values) != set(names):
                raise ValueError(f'{path} 关节名称与 model.joint_names 不一致')
            for name in names:
                self._finite(values[name], path + '.' + name, minv, maxv, integer, positive)
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
        if current != model['base_frame'] or {name for name in chain if name in names} != set(names):
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
            if data['pos_ratio'] <= 0 or data['tor_ratio'] <= 0:
                raise ValueError(f'{path} 位置和力矩比例必须为正数，方向由 direction 指定')
        runtime = section('control.runtime')
        absent_boolean = object()
        for field in ('control.runtime.write_enabled', 'control.controller.allow_full_cmd',
                      'safety_policy.require_all_actuators_online', 'safety_policy.require_all_actuators_enabled',
                      'safety_policy.reject_motor_error', 'safety_policy.require_continuous_cmd',
                      'safety_policy.fault_recovery.allow_drag_recovery',
                      'safety_policy.fault_recovery.require_operator_request',
                      'model.gravity_model_validated',
                      *(f'shutdown.paths.{path}.{key}' for path in ('MIT', 'CSP') for key in ('enabled',)),
                      'capability.admittance.calibration.friction.enabled'):
            value = core
            for part in field.split('.'):
                if not isinstance(value, dict) or part not in value:
                    value = absent_boolean
                    break
                value = value[part]
            if value is not absent_boolean and not isinstance(value, bool):
                raise ValueError(f'{field} 必须是布尔值')
        scalar('control.runtime.ctrl_frequency_hz',positive=True)
        scalar('control.runtime.joint_acc_filter_alpha',0,1)
        if runtime['model_feedforward_mode'] not in ('NONE','GRAVITY','FULL_INVERSE_DYNAMICS'):
            raise ValueError('control.runtime.model_feedforward_mode 无效')
        if runtime['control_path'] not in ('MIT', 'CSP'):
            raise ValueError('control.runtime.control_path 无效')
        controller = core.get('control', {}).get('controller') or {}
        for mode in ('rigid', 'compliant', 'drag'):
            if mode in controller:
                for axis in ('kp','kd'): joint_map(f'control.controller.{mode}.{axis}',0)
        # A disabled observation session needs no synthesized MIT PD gains.
        # Real activation independently requires a complete positive RIGID set.
        if 'drag' in controller and any(controller['drag']['kp'][name] != 0 for name in names):
            raise ValueError('拖拽模式 Kp 必须为零')
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
                joint_map('control.hold_compensation.' + field, positive=True)
        safety = section('safety_policy')
        if runtime['control_path'] == 'CSP':
            bounds = safety.get('max_tracking_error')
            missing = [name for name in names if not isinstance(bounds, dict) or name not in bounds]
            if missing:
                raise ValueError('CSP 缺少 safety_policy.max_tracking_error，请填写关节的最大跟踪误差: ' + ', '.join(missing))
        joint_map('safety_policy.position_margin',0)
        for key in ('max_dt_s','state_timeout_s','cmd_timeout_s'):
            scalar('safety_policy.'+key,positive=True)
        scalar('safety_policy.cmd_vel_scale',maxv=1,positive=True)
        scalar('safety_policy.state_vel_scale',positive=True)
        for field in ('max_acc','max_tracking_error','max_effort_override'):
            if field in safety: joint_map('safety_policy.'+field,positive=True)
        recovery = section('safety_policy.fault_recovery')
        if recovery['default_mode'] != 'rigid_hold':
            raise ValueError('safety_policy.fault_recovery.default_mode 仅支持 rigid_hold')
        scalar('safety_policy.fault_recovery.recovery_timeout_s',positive=True)
        joint_map('safety_policy.fault_recovery.velocity_tolerance',positive=True)
        if not isinstance(recovery.get('allow_drag_recovery'), bool):
            raise ValueError('fault_recovery.allow_drag_recovery 必须为布尔值')
        if recovery.get('require_operator_request') is not True:
            raise ValueError('fault_recovery.require_operator_request 必须为 true')
        from gain_initialization import protocol_gain_limits
        protocol = protocol_gain_limits(core, hardware, ((entry.get('hardware') or {}).get('plugin') or ''))
        for name in names:
            if name in protocol:
                for mode in ('rigid', 'compliant', 'drag'):
                    for term in ('kp', 'kd'):
                        if mode in controller and controller[mode][term][name] > protocol[name][term + '_max']:
                            raise ValueError(f'control.controller.{mode}.{term}.{name} 超出映射后的硬件协议范围')
            if hold is not None:
                effort = safety.get('max_effort_override', {}).get(name)
                if effort is not None and hold['max_torque'][name] > effort:
                    raise ValueError(f'control.hold_compensation.max_torque.{name} 超出总力矩预算')
        shut = core.get('shutdown') or {}
        if not isinstance(shut, dict) or set(shut) - {'paths'}:
            raise ValueError('shutdown 仅支持 paths.MIT 与 paths.CSP')
        plans = shut.get('paths', {})
        if not isinstance(plans, dict) or set(plans) - {'MIT', 'CSP'}:
            raise ValueError('shutdown.paths 仅支持 MIT 与 CSP')
        allowed = {'enabled', 'strategy', 'park_pos', 'speed_scale',
                   'position_tolerance', 'velocity_tolerance', 'settle_time_s', 'timeout_s'}
        for path, plan in plans.items():
            prefix = 'shutdown.paths.' + path + '.'
            if not isinstance(plan, dict) or set(plan) - allowed:
                raise ValueError(prefix[:-1] + ' 含未知停放字段')
            if plan.get('strategy', 'RIGID') not in ('RIGID', 'COMPLIANT_THEN_RIGID'):
                raise ValueError(prefix + 'strategy 必须为 RIGID 或 COMPLIANT_THEN_RIGID')
            enabled = plan.get('enabled') is True
            if enabled and not plan.get('park_pos'):
                plan['park_pos'] = dict.fromkeys(names, 0.0)
            if enabled and not parking_complete(core, path):
                raise ValueError(prefix[:-1] + ' 停放需要完整目标、逐轴容差、速度和时间参数')
            for key in ('park_pos', 'position_tolerance', 'velocity_tolerance'):
                if plan.get(key) or enabled: joint_map(prefix + key, positive=key != 'park_pos')
                elif key in plan and not isinstance(plan[key], dict):
                    raise ValueError(prefix + key + ' 必须为逐关节映射')
            if 'speed_scale' in plan: scalar(prefix + 'speed_scale', maxv=1, positive=True)
            if 'settle_time_s' in plan: scalar(prefix + 'settle_time_s', 0)
            if 'timeout_s' in plan: scalar(prefix + 'timeout_s', positive=True)
        for name in names:
            lim = joints_xml[name].find('limit')
            if lim is not None and lim.get('lower') is not None and lim.get('upper') is not None:
                lo, hi = float(lim.get('lower')), float(lim.get('upper'))
                margin = safety['position_margin'][name]
                if lo + margin >= hi - margin:
                    raise ValueError(f'safety_policy.position_margin.{name} 压空 URDF 有效行程')
                for path, plan in plans.items():
                    pos = (plan.get('park_pos') or {}).get(name)
                    if plan.get('enabled') is True and pos is not None and not lo + margin <= pos <= hi - margin:
                        raise ValueError(f'shutdown.paths.{path}.park_pos.{name} 超出 URDF 有效限位')
        for path, reason in constraint_errors(core, hardware, entry):
            raise ValueError(f'{path} {reason}')
        interaction = interaction_configuration(core)
        if hold and hold.get('enabled') and not interaction['observation_configured']:
            raise ValueError('刚性定位积分补偿需要 capability.admittance.observer 和 calibration')
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
                self._finite(damiao.get(key), 'hardware.damiao.'+key,positive=True)
            removed = set(damiao) & {'stop_kp', 'stop_kd', 'stop_cycles'}
            if removed: raise ValueError('hardware.damiao 含已移除字段 ' + ', '.join(sorted(removed)))
            for key in ('activation_retries','startup_read_cycles'):
                self._finite(damiao.get(key), 'hardware.damiao.'+key,1,integer=True)
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
        before_structure, next_structure = copy.deepcopy(before_core), copy.deepcopy(core)
        # Optional purposes can be deliberately added or removed. Their own
        # validators below enforce complete observation and MDK pairs.
        for document in (before_structure, next_structure):
            document.pop('capability', None)
            document['model'].pop('gravity_model_validated', None)
            document['control'].pop('controller', None)
            document.pop('shutdown', None)
        self._same_structure(before_structure,next_structure)
        for section, allowed in {
            'model': {'gravity_correction_path', 'gravity_model_validated'},
            'safety_policy': {'max_effort_override', 'max_tracking_error'},
            'shutdown': {'paths'},
        }.items():
            extra = set(core.get(section, {})) - set(before_core.get(section, {}))
            if not extra.issubset(allowed):
                raise ValueError(f'{section} 不支持新增字段 {sorted(extra)}')
        for section in ('calibration','control','capability'):
            extra = set(core.get(section, {})) - set(before_core.get(section, {}))
            if extra - ({'hold_compensation', 'controller'} if section == 'control' else {'admittance'} if section == 'capability' else set()):
                raise ValueError(f'{section} 不允许新增未知字段')
        new_modes = set(core['control'].get('controller', {})) - {'allow_full_cmd', 'rigid', 'compliant', 'drag'}
        if new_modes:
            raise ValueError('control.controller 不允许新增未知控制模式')
        gravity_status = self._profile_editor_gravity_status(config, {**payload, 'core': core})
        original_gravity_trusted = gravity_trusted(before_core)
        requested_gravity_trusted = gravity_trusted(core)
        # Explicit operator acknowledgment is required for a false -> true trust transition
        # Disabling recovery trust must always remain possible
        if requested_gravity_trusted and not original_gravity_trusted and payload.get('gravity_confirmed') is not True:
            raise ValueError('启用拖拽前必须确认重力模型已在当前机械臂和负载上完成验证')
        if gravity_status['gravity_validation_reset']:
            core['model']['gravity_model_validated'] = False
        original_quality = ((before_core.get('capability') or {}).get('admittance') or {}).get('validation') or {}
        requested_quality = ((core.get('capability') or {}).get('admittance') or {}).get('validation') or {}
        if any(value is True and original_quality.get(key) is not True for key, value in requested_quality.items()):
            raise ValueError('外力与导纳质量许可只能由独立测量验收保存，不能在参数编辑中手动开启')
        interaction_reset = revoke_changed_interaction_validation(before_core, core)
        if gravity_status['gravity_validation_changes'] or hardware != before_hw or entry.get('hardware') != original_entry.get('hardware'):
            for key in ('observation_validated', 'response_validated'):
                if requested_quality.get(key) is True:
                    requested_quality[key] = False
                    interaction_reset.append(key)
        if payload.get('hardware_imported') is not True:
            self._same_structure(before_hw,hardware,'hardware')
        self._same_structure(original_entry,entry,'profile')
        if entry.get('core') != original_entry.get('core') or entry.get('description') != original_entry.get('description'):
            raise ValueError('profile.core 和 profile.description 关联不允许在参数编辑中切换，请重新导入 Profile')
        readiness_path = self._readiness_path(info['profile_file'], info['profile'])
        readiness_meta = read_yaml(readiness_path) if readiness_path.is_file() else {}
        pending_model = readiness_meta.get('gain_initialization') or {}
        generated_roles = ('rigid', 'compliant')
        controller = core['control'].get('controller') or {}
        empty_generated_gains = all(value == 0 for role in generated_roles for term in ('kp','kd')
            for value in controller.get(role, {}).get(term, {}).values())
        model_axes = pending_model.get('axes') or []
        model_gains = {term:{axis.get('joint'):axis.get(term) for axis in model_axes} for term in ('kp','kd')}
        generated_controller = pending_model.get('generated_controller') or {}
        unchanged_model_gains = (all(controller.get(role, {}).get(term) == pair.get(term)
            for role,pair in generated_controller.items() for term in ('kp','kd')) if generated_controller else
            bool(model_axes) and all(controller.get(role, {}).get(term) == model_gains[term]
                for role in generated_roles for term in ('kp','kd')))
        model_context_changed = (hardware != before_hw or entry.get('hardware') != original_entry.get('hardware') or
            any(core.get(section) != before_core.get(section) for section in ('model','calibration','safety_policy')) or
            core['control']['runtime'] != before_core['control']['runtime'])
        initialization = None
        if (core['control']['runtime']['control_path'] == 'MIT' and not pending_model.get('existing_configuration_preserved') and
            (pending_model.get('configuration_applied') is False and empty_generated_gains or
             pending_model.get('configuration_applied') is True and unchanged_model_gains and model_context_changed) and
            (entry.get('hardware') or {}).get('plugin') == 'serial_arm_hardware_damiao'):
            from gain_initialization import initialize_profile_gains
            initialization = initialize_profile_gains(self.inspector, core_path, core,
                (core_path.parent / core['model']['urdf_path']).resolve(), hardware,
                entry['hardware']['plugin'], apply_generated=True)
        self._validate_core_profile(core,hardware,entry,core_path,before_core,allow_hardware_draft=True)
        # New saves use four purposes; a schema-only save rewrites just the
        # controller block and preserves the surrounding authored YAML
        desired = {}
        if initialization:
            desired[readiness_path] = yaml.safe_dump({**readiness_meta,'gain_initialization':initialization},allow_unicode=True,sort_keys=False)
            receipt = core_path.parent.parent / 'serialarm' / 'gain-initialization.json' if core_path.parent.name == 'core' else core_path.parent / 'gain-initialization.json'
            desired[receipt] = json.dumps(initialization,ensure_ascii=False,indent=2)+'\n'
        if core != before_core:
            desired[core_path] = yaml.safe_dump(four_mode_core(core), allow_unicode=True, sort_keys=False)
            if initialization and initialization.get('reference_only'):
                from gain_initialization import MODEL_GAIN_REFERENCE_COMMENT
                desired[core_path] = MODEL_GAIN_REFERENCE_COMMENT + desired[core_path]
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
        return {**self.profile_editor_load(config), **gravity_status,
                'interaction_validation_reset': list(dict.fromkeys(interaction_reset))}
