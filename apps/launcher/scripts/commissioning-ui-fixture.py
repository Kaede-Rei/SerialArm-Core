#!/usr/bin/env python3
"""Exercise production readiness APIs inside a temporary Profile, without devices."""
from copy import deepcopy
from pathlib import Path
import hashlib
import json
import sys
import tempfile
import shutil
import time

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
sys.path.insert(0, str(ROOT / 'tools'))
from control_parameters import canonical_core
from profile_library import ProfileLibrary
from parameter_contract import field_constraints
from check_backend_parameters import BackendParameters, runtime_values

home = Path(sys.argv[1]).resolve()
if not home.is_relative_to(Path(tempfile.gettempdir()).resolve()):
    raise ValueError('The commissioning UI fixture may write only inside the temporary directory')
home.mkdir(parents=True, exist_ok=True)
operation = sys.argv[2]
params = json.load(sys.stdin) if not sys.stdin.isatty() else {}
source = ROOT / 'src/robot_supports/robots/dm_arm/description/config/core/white.yaml'
core_path, hardware_path, profile_path = (home / name for name in ('core.yaml', 'hardware.yaml', 'robot_profiles.yaml'))
if operation == 'initialize' or not core_path.is_file():
    shutil.rmtree(home / 'review' / '.serialarm', ignore_errors=True)
    core = canonical_core(yaml.safe_load(source.read_text()))
    core['model']['urdf_path'] = str((source.parent / core['model']['urdf_path']).resolve())
    core['model']['gravity_model_validated'] = False
    core['control']['runtime']['write_enabled'] = True
    core['safety_policy']['max_tracking_error'] = dict.fromkeys(core['model']['joint_names'], .2)
    core.setdefault('capability', {}).setdefault('admittance', {}).setdefault('validation', {}).update({'observation_validated': False, 'response_validated': False})
    core_path.write_text(yaml.safe_dump(core, sort_keys=False))
    hardware_path.write_text((source.parent.parent / 'hardware.yaml').read_text())
    profile_path.write_text(yaml.safe_dump({'profiles': {'review': {'hardware': {'plugin': 'test_backend'}}}}))
core = canonical_core(yaml.safe_load(core_path.read_text()))
names = core['model']['joint_names']
config = {'profile': 'review', 'profile_file': str(profile_path)}

class Inspector:
    def inspect(self, _config):
        return {'profile': 'review', 'profile_file': str(profile_path), 'hardware_plugin': 'test_backend',
                'resources': {'core': str(core_path), 'hardware': str(hardware_path), 'description': core['model']['urdf_path']},
                'checks': [{'name': key, 'ok': True} for key in ('core', 'hardware', 'description')],
                'fingerprint': hashlib.sha256(core_path.read_bytes()).hexdigest(), 'write_enabled': True, 'devices': []}
    def validate_model_config(self, *args): return {'valid': True}
    def profiles(self, _config): return {'profile_file': str(profile_path)}

inspector = Inspector()
library = ProfileLibrary(home, inspector)
if operation == 'readiness_record':
    result = library.record_stage(config, params['stage'], params['answers'])
elif operation == 'select_path':
    core['control']['runtime']['control_path'] = params['path']
    core_path.write_text(yaml.safe_dump(core, sort_keys=False))
    result = library._readiness(config)
elif operation == 'observation':
    validation = {'independent': True, 'static_window_s': 15., 'pose_count': 5, 'friction_required': False,
                  'validated': params.get('passed', True), 'pass': [1] * len(names), 'scope_config': runtime_values(core)}
    if not params.get('passed', True): validation['pass'][0] = 0
    status = {'hardware_connected': True, 'control_path': 'CSP', 'joint_names': names,
              'gravity_scale': [core['model']['gravity_scale'][name] for name in names],
              'calibration': {'completed_kind': 'validation', 'kind': 'none', 'phase': 'complete', 'validation': validation}}
    interval = core['safety_policy']['state_timeout_s'] / 2
    raw = {'static_window_s': 15., 'poses': [{'samples': [dict(time_s=frame * interval,
               position=[float(pose)] * len(names), gravity=[0.] * len(names), measured_torque=[0.] * len(names))
               for frame in range(int(15. / interval) + 1)]} for pose in range(5)]}
    class Native:
        def request(self, method, *args, **kwargs):
            return deepcopy(raw if method == 'calibration_samples' else {} if method == 'candidate_overrides' else status)
    result = library.record_observation_measurement(config, Native(), inspector.inspect(config)['fingerprint'])
elif operation == 'response':
    case = BackendParameters(methodName='runTest')
    case.home, case.core_path, case.names, case.core, case.info = home, core_path, names, core, inspector.inspect(config)
    case.library = library
    _, window, native = case.response_fixture(moved_axis=params.get('axis', 0), active_axes=(0, 1))
    case.info = inspector.inspect(config)
    result = case.record_response(window, native, 'response-' + str(time.time_ns()) + '.json')
    result['readiness'] = library._readiness(config)
elif operation == 'profile_editor_load':
    result = {'profile': 'review', 'core': core, 'hardware': yaml.safe_load(hardware_path.read_text()),
              'field_constraints': field_constraints(core, yaml.safe_load(hardware_path.read_text())), 'joint_names': names, 'profile_entry': {'hardware': {'plugin': 'test_backend'}},
              'core_sha': hashlib.sha256(core_path.read_bytes()).hexdigest(), 'hardware_sha': hashlib.sha256(hardware_path.read_bytes()).hexdigest(),
              'profile_sha': hashlib.sha256(profile_path.read_bytes()).hexdigest(), 'core_path': str(core_path), 'hardware_path': str(hardware_path)}
else:
    result = library._readiness(config)
if operation == 'initialize': result = {'config': config, 'readiness': result}
print(json.dumps(result, ensure_ascii=False, allow_nan=False))
