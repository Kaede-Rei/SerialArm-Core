#!/usr/bin/env python3
"""Repeatable read-only URDF/trajectory design check; never loads a motor backend."""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'apps/launcher/backend'))
from gain_initialization import initialize_profile_gains, model_gain_limits
from control_parameters import canonical_core


def run(probe, profile, task):
    core_path = profile/'tomato_picker_description/config/core/tomato_picker.yaml'
    model = profile/'tomato_picker_description/urdf/tomato_picker.urdf'
    hardware = yaml.safe_load((profile/'tomato_picker_description/config/serialarm/hardware.yaml').read_text())
    core = canonical_core(yaml.safe_load(core_path.read_text()))
    before = deepcopy(core)
    class Inspector:
        def model_probe(self): return str(probe)
    local = initialize_profile_gains(Inspector(), core_path, core, model, hardware, 'serial_arm_hardware_damiao')
    assert core == before, 'model preview overwrote an operational/recovery baseline'
    assert not local['configuration_applied'] and local['runtime_baseline_unchanged']
    limits, positions, _ = model_gain_limits(core, model, hardware, 'serial_arm_hardware_damiao')
    config = {'model': {**core['model'], 'urdf_path': str(model.resolve())}}
    def inspect(rows, trajectory=None):
        with tempfile.TemporaryDirectory(prefix='serialarm-physical-model-') as temporary:
            path = Path(temporary)/'model.yaml';path.write_text(yaml.safe_dump(config))
            args = [str(probe), '--config', str(path), '--positions', ','.join(map(str, positions)),
                    '--gain-limits-json', json.dumps(rows)]
            if trajectory: args += ['--gain-path-csv', str(trajectory)]
            process = subprocess.run(args, text=True, capture_output=True, timeout=30, check=False)
        payload = json.loads(process.stdout)
        assert process.returncode == 0 and payload['ok'], payload
        return payload['gain_initialization']
    changed = deepcopy(limits)
    for row in changed:
        row['max_velocity'] *= .5;row['max_acceleration'] *= .1;row['effort_budget'] *= .8
    constrained = inspect(changed)
    assert [(a['unconstrained_kp'], a['unconstrained_kd']) for a in local['axes']] == [
        (a['unconstrained_kp'], a['unconstrained_kd']) for a in constrained['axes']], 'human velocity/acceleration/effort limits became physical model priors'
    original_limits = inspect(limits)
    assert local['axes'] == original_limits['axes'], 'same model/design/protocol is not reproducible'
    path_result = inspect(limits, task/'trajectory.csv')
    assert path_result['path_samples'] > 100 and path_result['task_demand_source'] == 'provided_demonstration_path'
    assert not path_result['measured'], 'offline design became an empirical validation'
    for result in (local, path_result):
        assert min(result['nominal_mode_damping_ratio']) >= .7*(1-1e-6)
        for axis, constraint in zip(result['axes'], limits):
            assert axis['kp'] > 0 and axis['kd'] >= 0
            assert axis['kp']*axis['reference_error_bound_rad']+axis['kd']*axis['velocity_error_bound_rad'] <= constraint['effort_budget']+1e-8
            for term in ('kp', 'kd'):
                quantum = constraint['protocol_'+term+'_quantum']
                assert math.isclose(axis[term]/quantum, round(axis[term]/quantum), abs_tol=1e-6)
    axes = {a['joint']: a for a in local['axes']}
    assert axes['joint3']['unconstrained_kp'] > axes['joint6']['unconstrained_kp'], 'same recovery design lost the physical inertia scale'
    def compact(result):
        return {'source': result['source'], 'scope': result['scope'], 'measured': result['measured'],
                'axes': [{key: row[key] for key in ('joint', 'inertia_max', 'unconstrained_kp', 'unconstrained_kd',
                    'pre_modal_kp', 'pre_modal_kd', 'kp', 'kd', 'reference_error_bound_rad', 'velocity_error_bound_rad', 'constraints')}
                    for row in result['axes']], 'modal_adjustments': result['modal_adjustments']}
    return {'checks': ['runtime_and_recovery_unchanged', 'human_velocity_acceleration_effort_not_physical_priors', 'deterministic_model_design',
                       'real_historical_path', 'nominal_coupled_damping', 'joint_effort_constraints', 'protocol_grid'],
            'recovery_design': compact(local), 'historical_path_design': compact(path_result)}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True, help='tomato_picker package in a disposable copied workspace')
    parser.add_argument('--task', type=Path, required=True, help='read-only task containing the original trajectory.csv')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = run(args.probe.resolve(), args.profile.resolve(), args.task.resolve())
    encoded = json.dumps(report, ensure_ascii=False, indent=2)+'\n'
    if args.output: args.output.write_text(encoded)
    print(encoded)
