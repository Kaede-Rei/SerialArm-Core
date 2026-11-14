#!/usr/bin/env python3
"""Exercise the current controller layout and gain-candidate deployment."""
from pathlib import Path
from copy import deepcopy
import sys
import yaml

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'apps/launcher/backend'))
from artifact_exports import gain_candidate_targets, _replace_core
from control_parameters import canonical_core, deployment_gain_groups, four_mode_control_text
from gain_context import recorded_gain_core
from persistence import _runtime_text
from profile_library import ProfileLibrary


def rejected(call, field):
    try:
        call()
    except ValueError as error:
        assert field in str(error), str(error)
    else:
        raise AssertionError('expected rejection: ' + field)


library = ProfileLibrary(root, None)

for path in (root / 'src/robot_supports/robots/dm_arm/description/config/core').glob('*.yaml'):
    text = path.read_text()
    config = canonical_core(yaml.safe_load(text))
    assert yaml.safe_load(four_mode_control_text(text)) == yaml.safe_load(text)
    assert set(config['control']['controller']) == {'allow_full_cmd', 'rigid', 'compliant', 'drag'}
    assert 'enabled' not in config['capability']['admittance']
    names = config['model']['joint_names']
    library._validate_core_profile(config, {}, {'hardware': {'plugin': ''}}, path, config)
    csp = deepcopy(config)
    csp['control']['runtime']['control_path'] = 'CSP'
    rejected(lambda: library._validate_core_profile(csp, {}, {'hardware': {'plugin': ''}}, path, config),
             'safety_policy.max_tracking_error')
    csp['safety_policy']['max_tracking_error'] = dict.fromkeys(names, .2)
    library._validate_core_profile(csp, {}, {'hardware': {'plugin': ''}}, path, config)
    del csp['safety_policy']['max_tracking_error'][names[-1]]
    rejected(lambda: library._validate_core_profile(csp, {}, {'hardware': {'plugin': ''}}, path, config), names[-1])
    obsolete = deepcopy(config)
    obsolete['capability']['admittance']['enabled'] = False
    rejected(lambda: canonical_core(obsolete), 'enabled')
    rejected(lambda: four_mode_control_text(yaml.safe_dump(obsolete)), 'enabled')
    obsolete = deepcopy(config)
    obsolete['control']['runtime']['tracking_gain_profile'] = 'COMPLIANT'
    rejected(lambda: canonical_core(obsolete), 'tracking_gain_profile')
    obsolete = deepcopy(config)
    obsolete['capability']['admittance']['observer']['mode'] = 'MOMENTUM'
    rejected(lambda: canonical_core(obsolete), 'momentum_gain')
    admittance = config['capability']['admittance']
    runtime = {'enabled': True, 'joint_enabled': [admittance['joint_enabled'][name] for name in names]}
    for section in ('observer', 'calibration', 'controller'):
        for key, value in admittance[section].items():
            runtime[key] = ([value[name] for name in names] if isinstance(value, dict) and set(value) == set(names)
                            else {term: [values[name] for name in names] if isinstance(values, dict) else values
                                  for term, values in value.items()} if isinstance(value, dict) else value)
    persisted = yaml.safe_load(_runtime_text(text, names, runtime, [config['model']['gravity_scale'][name] for name in names]))
    assert 'enabled' not in persisted['capability']['admittance']
    restored = recorded_gain_core(config, {'joint_names': names}, {'control_context': {
        'runtime': config['control']['runtime'], 'admittance': {'enabled': True}}})
    assert 'enabled' not in canonical_core(restored)['capability']['admittance']
    candidate = {'category': 'rigid', 'joint_names': names,
                 'gains': {'kp': [2.] * len(names), 'kd': [.2] * len(names)},
                 'validated': True, 'hold_verification': {'validated': True}}
    assert set(deployment_gain_groups(candidate)) == {'rigid'}
    targets, compensation = gain_candidate_targets(candidate, config, names)
    updated = yaml.safe_load(_replace_core(text, targets, hold_compensation=compensation))
    assert updated['control']['controller']['rigid']['kp'] == dict.fromkeys(names, 2.)
    assert updated['control']['controller']['compliant'] == config['control']['controller']['compliant']
    assert set(updated['control']['controller']) == {'allow_full_cmd', 'rigid', 'compliant', 'drag'}
print('interaction profile and gain persistence passed')
