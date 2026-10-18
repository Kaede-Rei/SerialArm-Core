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

import yaml

from profiles import config_values, read_yaml, text


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _atomic_yaml(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    yaml.safe_load(temp.read_text())
    temp.replace(path)


def _safe_extract(source, destination):
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(source) as archive:
        for member in archive.infolist():
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
              'COMPLIANT_DRAG', 'COMPLIANT_TRACKING', 'IDENTIFICATION_TRACKING')
    _MODE_KEYS = {name: name.lower() for name in _MODES}

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
        hardware_hash = self._digest(info.get('hardware_plugin'), hardware, info.get('devices'))
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
                                     controller.get('identification_tracking'), capability)
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
            controller.get(self._MODE_KEYS[name])) for name in self._MODES}
        return scopes

    def _readiness_meta(self, info):
        path = self._readiness_path(info['profile_file'], info['profile'])
        # Existing verification notes remain readable after switching to
        # the Profile-local directory. New writes always use the new location
        safe = ''.join(c if c.isalnum() or c in '_-' else '_' for c in str(info['profile'])) or 'profile'
        old_path = (self.library_file.parent / 'readiness' / f'{safe}.yaml') if str(info['profile_file']).startswith(str(self.root / 'src/robot_supports/profiles')) else (Path(info['profile_file']).parent / '.serialarm' / 'readiness' / f'{safe}.yaml')
        source = path if path.is_file() else old_path
        try:
            meta = yaml.safe_load(source.read_text()) if source.is_file() else {}
        except (OSError, yaml.YAMLError):
            meta = {}
        return path, meta if isinstance(meta, dict) else {}

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
        core = read_yaml(info['resources']['core']) if checks.get('core') else {}
        hardware = read_yaml(info['resources']['hardware']) if checks.get('hardware') else {}
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
        if profile_setup:
            try:
                entry = (read_yaml(info['profile_file']).get('profiles') or {})[info['profile']]
                self._validate_core_profile(core, hardware, entry, Path(info['resources']['core']), core)
            except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as error:
                profile_setup = False
                reason = str(error)
        scopes = self._dependencies(info, core, hardware) if checks.get('core') else {}
        _, meta = self._readiness_meta(info)
        stages = meta.get('stages') or {}
        modes = meta.get('modes') or {}
        def status(item, scope):
            if not isinstance(item, dict) or item.get('result') not in ('passed', 'recorded'):
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
            return 'verified'
        values = {
            'description': 'verified' if checks.get('description') and checks.get('core') else 'failed',
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
            gains = controller.get(key) or {}
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
            mode_states[name] = (own if own in ('verified', 'stale') else 'configured') if valid else 'unconfigured'
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
        state = 'ready' if profile_setup else 'configuring'
        urdf_limits = {}
        if profile_setup:
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
        }
        return {'state': state, 'next_stage': 'profile_setup' if not profile_setup else 'ready',
                'checks': values, 'modes': mode_states, 'reason': reason,
                'fingerprint': info['fingerprint'], 'info': info,
                'control_available': profile_setup, 'safety_summary': safety_summary,
                'setup_issues': setup_issues,
                'source_trusted': self._is_official_profile(info),
                'evidence': {k: {'kind': v.get('kind'), 'note': v.get('note', ''),
                                 'observations': v.get('observations') or {}}
                             for k, v in stages.items() if isinstance(v, dict)}}

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
        path, data = self._readiness_meta(info)
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

    def approve_gain_mode(self, config, mode, path, native=None):
        if mode not in ('RIGID_TRACKING', 'COMPLIANT_TRACKING'):
            raise ValueError('该模式尚无可用的独立增益验收流程')
        current = self._readiness(config)
        info = self.inspector.inspect(config)
        from workflow_registry import prepare_gain_candidate
        candidate_info = prepare_gain_candidate(str(Path(path).expanduser().resolve().parent))
        candidate_path = Path(candidate_info['candidate_path'])
        candidate = json.loads(candidate_path.read_text())
        targets = candidate.get('targets') or {}
        target = mode.lower()
        selected = targets.get(target) if candidate.get('category') == 'all' else candidate if candidate.get('category') == target else None
        if not isinstance(selected, dict) or selected.get('validated', True) is not True:
            raise ValueError('候选中没有经独立验证的对应控制模式')
        core = read_yaml(info['resources']['core']); hw = read_yaml(info['resources']['hardware'])
        urdf_path = (Path(info['resources']['core']).parent / core['model']['urdf_path']).resolve()
        if not urdf_path.is_file() or candidate.get('urdf_fingerprint') != self._native_file_fingerprint(urdf_path):
            raise ValueError('增益验收候选与当前 URDF 不一致')
        joints = core['model']['joint_names']
        gains = core['control']['controller'][target]
        for axis in ('kp', 'kd'):
            vector = selected.get(axis)
            if not isinstance(vector, list) or len(vector) != len(joints):
                raise ValueError('候选参数的关节维度不一致')
            if any(abs(gains[axis][joint] - float(value)) > 1e-5 for joint, value in zip(joints, vector)):
                raise ValueError('当前 Core 增益与通过验证的候选不一致，请先保存已验证增益')
        scopes = self._dependencies(info, core, hw)
        meta_path, meta = self._readiness_meta(info)
        meta.setdefault('modes', {})[mode] = {
            'result': 'passed', 'scope': scopes['modes'][mode], 'kind': 'validated_gain_candidate',
            'candidate_sha256': _sha(candidate_path), 'candidate_path': str(candidate_path), 'note': '增益候选已通过独立验证且与当前 Core 增益一致',
            'recorded_at': time.time(),
        }
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
            if not root.is_dir(): _safe_extract(path, root)
            return root
        if path.is_dir(): return path
        if path.is_file() and (path.name.endswith('.urdf') or path.name.endswith('.xacro')): return path.parent
        raise ValueError('select a Description directory, ZIP, URDF, or Xacro')

    def inspect_description(self, source):
        root=self._description_source(source)
        packages=[]
        for package_xml in sorted(root.glob('**/package.xml'))[:30]:
            try:
                node=ET.parse(package_xml).getroot(); name=(node.findtext('name') or '').strip()
            except ET.ParseError: continue
            urdfs=[p for p in package_xml.parent.glob('**/*.urdf') if not p.name.endswith('.urdf.xacro')]
            if urdfs: packages.append((package_xml.parent,name,urdfs))
        if not packages:
            urdfs=list(root.glob('**/*.urdf'))
            if not urdfs: raise ValueError('Description contains no URDF')
            package_root=urdfs[0].parent; package_name=package_root.name; packages=[(package_root,package_name,urdfs)]
        package_root, package_name, urdfs=packages[0]
        urdf=urdfs[0]
        raw=self.inspector._parse_urdf(urdf,[str(root),str(package_root.parent),str(package_root)])
        children={j['child'] for j in raw['joints']}; roots=[l['name'] for l in raw['links'] if l['name'] not in children]
        parent_joint={j['child']:j for j in raw['joints']}; by_parent={}
        for j in raw['joints']: by_parent.setdefault(j['parent'],[]).append(j)
        matrices={name:[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]] for name in roots}
        queue=list(roots)
        while queue:
            parent=queue.pop(0); base=matrices[parent]
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
        return {'source_root':str(root),'package_root':str(package_root),'package_name':package_name,'urdf':str(urdf),'urdf_relative':str(urdf.relative_to(package_root)),'xacro':str(xacros[0]) if xacros else '','ros2_control_xacro':next((str(p) for p in xacros if 'ros2_control' in p.name),''),'roots':roots,'leaves':leaves,'links':[x['name'] for x in raw['links']],'joints':raw['joints'],'preview':preview,'fingerprint':_sha(urdf)}

    @staticmethod
    def _joint_map(names, value): return {name:value for name in names}

    def create_profile(self, options):
        source=text(options.get('source')); analysis=self.inspect_description(source)
        profile=text(options.get('profile'))
        if not profile or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in profile): raise ValueError('Profile id may contain letters, numbers, underscore, and hyphen only')
        destination=Path(text(options.get('destination'))).expanduser().resolve()
        package_dir=destination/profile
        if package_dir.exists() and any(package_dir.iterdir()): raise ValueError('destination Profile package already exists and is not empty')
        package_dir.mkdir(parents=True,exist_ok=True)
        copied=package_dir/analysis['package_name']; shutil.copytree(analysis['package_root'],copied,dirs_exist_ok=True)
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
          'control':{'runtime':{'ctrl_frequency_hz':200.0,'joint_acc_filter_alpha':0.2,'write_enabled':False,'model_feedforward_mode':'GRAVITY','tracking_impedance_mode':'COMPLIANT_TRACKING'},'controller':{'allow_full_cmd':False,'rigid_hold':{'kp':kp_rigid,'kd':kd_rigid},'rigid_tracking':{'kp':kp_rigid,'kd':kd_rigid},'compliant_hold':{'kp':kp_soft,'kd':kd_soft},'compliant_drag':{'kp':zeros,'kd':zeros.copy()},'compliant_tracking':{'kp':kp_soft,'kd':kd_soft},'identification_tracking':{'kp':kp_rigid.copy(),'kd':kd_rigid.copy()}}},
          'safety_policy':{'position_margin':0.0,'cmd_vel_scale':0.25,'state_vel_scale':1.0,'max_acc':self._joint_map(joints,2.0),'max_kp_override':self._joint_map(joints,50.0),'max_kd_override':self._joint_map(joints,2.0),'max_dt_s':0.02,'state_timeout_s':0.05,'cmd_timeout_s':0.10,'require_all_actuators_online':True,'require_all_actuators_enabled':True,'reject_motor_error':True,'require_continuous_cmd':False,'fault_recovery':{'default_mode':'rigid_hold','allow_compliant_recovery':True,'require_operator_request':True,'gravity_model_validated':False,'recovery_timeout_s':30.0,'compliant_recovery':{'kp':kp_soft,'kd':kd_soft,'max_vel':self._joint_map(joints,0.5),'effort_scale':0.5}}},
          'shutdown':{'park_before_disable':True,'park_pos':park,'speed_scale':0.05,'position_tolerance':0.05,'velocity_tolerance':0.05,'settle_time_s':0.25,'relaxed_tolerance_ratio':2.0,'timeout_s':10.0},
          'capability':{'admittance':{'enabled':False,'joint_enabled':{j:True for j in joints},'observer':{'mode':'MOMENTUM','momentum_gain':self._joint_map(joints,25.0)},'calibration':{'torque_bias':zeros,'torque_threshold':self._joint_map(joints,0.05),'friction':{'enabled':False,'velocity_transition':0.03,'positive_coulomb':zeros,'positive_viscous':zeros,'negative_coulomb':zeros,'negative_viscous':zeros}},'controller':{'mass':ones,'damping':self._joint_map(joints,5.5),'stiffness':self._joint_map(joints,15.0),'max_delta_q':self._joint_map(joints,0.5),'max_delta_q_dot':self._joint_map(joints,1.0)}}}
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
            # Keep an already included legacy copy consistent, for consumers
            # still accessing config/serialarm/core.yaml directly.
            legacy_copy = config_dir / 'core.yaml'
            if legacy_copy.is_file():
                legacy_copy.write_bytes(authored_config.read_bytes())
        else:
            core_config_relative = 'config/serialarm/core.yaml'
            _atomic_yaml(config_dir/'core.yaml',core)
        hardware_plugin=text(options.get('hardware_plugin'))
        if hardware_plugin not in ('', 'serial_arm_hardware_damiao'):
            raise ValueError('hardware_plugin 目前仅支持未配置或 serial_arm_hardware_damiao')
        hardware={}
        actuators=options.get('actuators') or []
        complete=hardware_plugin=='serial_arm_hardware_damiao' and len(actuators)==len(joints) and all(text(a.get('motor_type')) and int(a.get('motor_id',0))>0 for a in actuators)
        if hardware_plugin=='serial_arm_hardware_damiao':
            if len(actuators)!=len(joints):
                actuators = list(actuators[:len(joints)]) + [{} for _ in range(max(0,len(joints)-len(actuators)))]
            bus=text(options.get('bus')) or 'main_can'; device=text(options.get('device')) or '/dev/ttyACM0'; baud=int(options.get('baudrate') or 921600)
            hardware={'buses':{bus:{'type':'can','backend':'damiao_usb2can','device':device,'baudrate':baud}},'damiao':{'bus':bus,'refresh_state_in_read':False,'feedback_timeout_s':0.05,'activation_retries':3,'startup_read_cycles':5,'stop_kp':10.0,'stop_kd':0.15,'stop_cycles':5,'actuators':{j:{'name':text(a.get('name')) or f'actuator{i+1}','motor_id':int(a.get('motor_id') or i+1),'master_id':int(a.get('master_id') or 0),'motor_type':text(a.get('motor_type'))} for i,(j,a) in enumerate(zip(joints,actuators))}}}
        _atomic_yaml(config_dir/'hardware.yaml',hardware)
        entry={'core':{'package':analysis['package_name'],'config':core_config_relative},'hardware':{'plugin':hardware_plugin if hardware_plugin=='serial_arm_hardware_damiao' else '','config_package':analysis['package_name'],'config':'config/serialarm/hardware.yaml'},'description':{'package':analysis['package_name'],'urdf':analysis['urdf_relative'].replace('\\','/')}}
        if analysis['ros2_control_xacro']:
            entry['description']['ros2_control_xacro']=str(Path(analysis['ros2_control_xacro']).relative_to(analysis['package_root'])).replace('\\','/')
        profile_file=package_dir/'robot_profiles.yaml'; _atomic_yaml(profile_file,{'profiles':{profile:entry}})
        readiness=self._readiness_path(profile_file, profile); _atomic_yaml(readiness,{'profile':profile,'managed_by_launcher':True,'source_urdf_fingerprint':analysis['fingerprint'],'stages':{}})
        return {'package':str(package_dir),'profile_file':str(profile_file),'profile':profile,'resource_paths':str(package_dir),'hardware_complete':complete,'config':{'profile':profile,'profile_file':str(profile_file),'resource_paths':str(package_dir),'serial_port':'','baudrate':'','bus':''}}

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
            issues.append({'group': group, 'path': path, 'message': message})
        names = ((core.get('model') or {}).get('joint_names') or [])
        plugin = ((entry.get('hardware') or {}).get('plugin') or '').strip()
        if not plugin:
            issue('hardware', 'profile_entry.hardware.plugin', '请选择硬件驱动，达妙电机请选择 serial_arm_hardware_damiao')
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
        safety = core.get('safety_policy') or {}
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
        core = read_yaml(core_path)
        hardware = read_yaml(hardware_path) if hardware_path.is_file() else {}
        profile = read_yaml(profile_path)['profiles'][info['profile']]
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
        elif candidate != original:
            raise ValueError(f'{path} 不支持此参数类型')

    def _validate_core_profile(self, core, hardware, entry, core_path, old_core, allow_hardware_draft=False):
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
        try:
            root = ET.parse(urdf).getroot()
        except ET.ParseError as error:
            raise ValueError(f'model.urdf_path URDF 解析失败 {error}') from error
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
        scalar('control.runtime.ctrl_frequency_hz',1)
        scalar('control.runtime.joint_acc_filter_alpha',0,1)
        if runtime['model_feedforward_mode'] not in ('NONE','GRAVITY','FULL_INVERSE_DYNAMICS'):
            raise ValueError('control.runtime.model_feedforward_mode 无效')
        if runtime['tracking_impedance_mode'] not in ('RIGID_TRACKING','COMPLIANT_TRACKING'):
            raise ValueError('control.runtime.tracking_impedance_mode 无效')
        controller = section('control.controller')
        for mode in ('rigid_hold','rigid_tracking','compliant_hold','compliant_drag','compliant_tracking','identification_tracking'):
            if mode == 'identification_tracking' and mode not in controller: continue
            for axis in ('kp','kd'): joint_map(f'control.controller.{mode}.{axis}',0)
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
            for mode in ('rigid_hold','rigid_tracking','compliant_hold','compliant_drag','compliant_tracking','identification_tracking'):
                for axis in ('kp','kd'):
                    if mode not in controller: continue
                    limit = safety.get('max_'+axis+'_override',{}).get(name)
                    if limit is not None and controller[mode][axis][name] > limit:
                        raise ValueError(f'control.controller.{mode}.{axis}.{name} 超出 safety_policy.max_{axis}_override')
        shut = section('shutdown')
        joint_map('shutdown.park_pos')
        for key in ('speed_scale','compliant_phase_ratio'):
            if key in shut: scalar('shutdown.'+key,1e-7,1)
        for key in ('position_tolerance','velocity_tolerance','settle_time_s','relaxed_tolerance_ratio','timeout_s'):
            scalar('shutdown.'+key,1e-7)
        for name in names:
            lim = joints_xml[name].find('limit')
            if lim is not None and lim.get('lower') is not None and lim.get('upper') is not None:
                lo,hi = float(lim.get('lower')),float(lim.get('upper'))
                pos = shut['park_pos'][name]
                if not lo+safety['position_margin'] <= pos <= hi-safety['position_margin']:
                    raise ValueError(f'shutdown.park_pos.{name} 超出 URDF 有效限位')
        admittance = section('capability.admittance')
        if not isinstance(admittance['enabled'],bool): raise ValueError('capability.admittance.enabled 必须为布尔值')
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
                self._finite(item.get('motor_id'),f'hardware.damiao.actuators.{name}.motor_id',1,integer=True)
                self._finite(item.get('master_id'),f'hardware.damiao.actuators.{name}.master_id',0,integer=True)
                bus_key = (bus,item['motor_id'])
                if bus_key in ids: raise ValueError(f'hardware.damiao.actuators.{name}.motor_id 重复')
                ids.add(bus_key)

    def profile_editor_save(self, config, payload):
        info, core_path, hw_path, profile_path = self._profile_config_paths(config)
        for path, field in ((core_path,'core_sha'),(hw_path,'hardware_sha'),(profile_path,'profile_sha')):
            current_sha = _sha(path) if path.is_file() else ''
            if current_sha != payload.get(field):
                raise ValueError(f'{path.name} 已被外部修改，请重新加载参数后再保存')
        before_core = read_yaml(core_path)
        before_hw = read_yaml(hw_path) if hw_path.is_file() else {}
        profile_data = read_yaml(profile_path)
        original_entry = profile_data['profiles'][info['profile']]
        core = payload.get('core')
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
            if set(core[section]) != set(before_core[section]):
                raise ValueError(f'{section} 不允许新增未知字段')
        new_modes = set(core['control']['controller']) - set(before_core['control']['controller'])
        if not new_modes.issubset({'identification_tracking'}):
            raise ValueError('control.controller 不允许新增未知控制模式')
        if (core['capability']['admittance']['enabled'] and
            not before_core['capability']['admittance']['enabled']):
            raise ValueError('capability.admittance.enabled 需要后续标定和验证后才能启用')
        original_gravity_trusted = before_core['safety_policy']['fault_recovery']['gravity_model_validated']
        requested_gravity_trusted = core['safety_policy']['fault_recovery']['gravity_model_validated']
        # Explicit operator acknowledgment is required for a false -> true trust transition
        # Disabling recovery trust must always remain possible
        if requested_gravity_trusted and not original_gravity_trusted and payload.get('gravity_recovery_confirmed') is not True:
            raise ValueError('启用柔性故障恢复前必须确认重力补偿已在当前机械臂和负载上完成真机验证')
        self._same_structure(before_hw,hardware,'hardware')
        self._same_structure(original_entry,entry,'profile')
        if entry.get('core') != original_entry.get('core') or entry.get('description') != original_entry.get('description'):
            raise ValueError('profile.core 和 profile.description 关联不允许在参数编辑中切换，请重新导入 Profile')
        self._validate_core_profile(core,hardware,entry,core_path,before_core,allow_hardware_draft=True)
        # A user-owned Profile is a single authoritative configuration; preserve
        # exact existing bytes on no-op saves so comments and verified values stay intact
        desired = {}
        if core != before_core: desired[core_path] = yaml.safe_dump(core, allow_unicode=True, sort_keys=False)
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
                _atomic_yaml(path,yaml.safe_load(value))
        except Exception:
            for path, content in backup.items():
                if content is None: path.unlink(missing_ok=True)
                else:
                    tmp = path.with_suffix(path.suffix+'.rollback')
                    tmp.write_bytes(content); tmp.replace(path)
            raise
        return self.profile_editor_load(config)
