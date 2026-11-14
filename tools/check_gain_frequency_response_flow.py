#!/usr/bin/env python3
"""Exercise isolated native frequency-response inputs using only write-disabled Mock.

Mock follows references and has no torque-to-motion dynamics. Therefore valid
frequency plants/candidates are forbidden; this checks the actual experiment,
masking, evidence persistence and stop paths, not physical tuning acceptance.
The complete case collects every admitted interior input before stopping the
worker; the partial case stops its second input. Both retain the earlier
complete 1x baseline and do not claim full-purpose calibration acceptance.
"""
from __future__ import annotations

import argparse
import csv
import json
import hashlib
import math
import re
import shutil
from pathlib import Path
import sys
import tempfile
import time

from check_adaptive_gain_flow import Native, ROOT, prepare_config
from check_gain_taught_path_flow import prepare_source, read_rows


def isolated_kind(status):
    kind = status.get('gain_probe_kind', '')
    return (kind if status.get('gain_probe_stage') == 'frequency_response_identification'
            and re.fullmatch(r'native-axis-\d+-controller-\d+(?:-input-supplement)?', kind) else None)


def baseline_evidence(folder, names, expected=None):
    path = folder / 'gain-trajectory-native-frozen.csv'
    rows = read_rows(path)
    with (folder / 'trajectory.csv').open() as stream:
        trajectory = csv.reader(stream)
        sample_dt = float(next(trajectory)[1])
        next(trajectory)
        duration = sample_dt * (sum(1 for _ in trajectory)-1)
    for name in names:
        joint = [row for row in rows if row['joint'] == name]
        assert len(joint) >= 100 and max(float(row['time_s']) for row in joint) >= duration+1.5-.08, name
        assert all(row['actual_gains_known'] == '1' for row in joint), name
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if expected is not None:
        assert digest == expected, 'Optional identification rewrote the complete baseline evidence'
    return digest, duration


def saved_stop(native, task, deadline):
    native.request('model_calibration_cancel', {'task_id': task['task_id']})
    folder = Path(task['directory'])
    stop_deadline = min(deadline, time.monotonic() + 10)
    while time.monotonic() < stop_deadline:
        status = native.request('model_calibration_status')
        if status['phase'] in ('cancelled', 'complete', 'failed'):
            try:
                result = json.loads((folder / 'result.json').read_text())
                candidate = json.loads((folder / 'gain-candidate.json').read_text())
            except (FileNotFoundError, json.JSONDecodeError):
                time.sleep(.03)
                continue
            if result.get('phase') == status['phase'] and result.get('termination') == status.get('termination'):
                assert candidate['termination'] == status['termination'], status
                assert status['termination']['kind'] != 'execution_fault', status
                assert candidate['validated'] is False
                assert native.request('status')['robot_state'] == 'ACTIVE'
                return status, candidate
        time.sleep(.03)
    raise AssertionError('The real worker did not save a consistent normal stop')


def verify_isolated(folder, parameters, names, complete):
    axis = parameters['isolated_axis_index']
    assert isinstance(axis, int) and 0 <= axis < len(names), parameters
    assert parameters['independent_disturbance'] is True and parameters['hold_path'] is False, parameters
    assert parameters['reference_schedule'] == 'control_cycle_monotonic', parameters
    assert parameters['other_axes_execution'] == 'fixed_reference_same_complete_gains', parameters
    assert parameters['disturbance_axis_eligible'] == [k == axis for k in range(len(names))], parameters
    assert math.isclose(parameters['independent_input_baseline_s'], .25), parameters
    assert all(plan['amplitude_rad'] == 0 and plan['reference_bias_rad'] == 0
               for k, plan in enumerate(parameters['plans']) if k != axis), parameters
    rows = read_rows(folder / parameters['record_path'])
    count = 0
    elapsed = max(float(row['time_s']) for row in rows)
    if complete:
        assert elapsed >= parameters['duration_s'] + 1.5 - .08, parameters
    else:
        assert .25 < elapsed < parameters['duration_s'] + 1.5 - .1, (elapsed, parameters)
    for k, joint in enumerate(names):
        rr = [row for row in rows if row['joint'] == joint]
        assert len(rr) >= (100 if complete else 20), joint
        assert all(int(row['limited']) == 0 for row in rr), joint
        assert all(abs(float(row['reference']) - parameters['center_rad'][k]) < 1e-9
                   and float(row['reference_velocity']) == 0 for row in rr), joint
        stable = [row for row in rr if .2 <= float(row['time_s']) < parameters['duration_s'] - .45]
        assert stable and all(int(row['actual_gains_known']) == 1
                   and abs(float(row['actual_kp']) - parameters['kp'][k]) < 1e-4
                   and abs(float(row['actual_kd']) - parameters['kd'][k]) < 1e-4 for row in stable), joint
        fresh = [row for row in rr if int(row['new_measurement'])]
        stamps = [int(row['feedback_sequence']) for row in fresh]
        assert stamps and all(a < b for a, b in zip(stamps, stamps[1:])), joint
        assert all(math.isfinite(float(row[key])) for row in rr for key in ('command_time_s', 'feedback_time_s')), joint
        assert all(-.002 <= float(row['command_time_s'])-float(row['feedback_time_s']) < .06 for row in fresh), joint
        baseline = [row for row in rr if float(row['time_s']) < .2]
        assert len(baseline) >= 20 and all(abs(float(row['independent_disturbance_nm'])) < 1e-12 for row in baseline), joint
        peak = max(abs(float(row['independent_disturbance_nm'])) for row in rr)
        if k == axis:
            assert peak > 0, joint
            count += sum(int(row['new_measurement']) for row in stable)
            assert parameters['independent_input_axes'][k]['bidirectional'], parameters
        else:
            assert peak == 0, joint
        # Mock has no torque response; a fixed reference cannot provide a
        # physical transfer function just because it produces many samples.
        assert max(float(row['actual']) for row in rr)-min(float(row['actual']) for row in rr) < 1e-8, joint
    return {'axis_index': axis, 'record_path': parameters['record_path'],
            'fresh_window_rows': count, 'completed': complete, 'elapsed_s': elapsed}


def fixture(terminal, directory, deadline, partial):
    directory.mkdir()
    config, names = prepare_config(directory)
    storage = directory / 'storage'
    storage.mkdir()
    native = Native(terminal, config, storage)
    try:
        native.request('activate')
        native.ready()
        # Reuse the actual taught-path flow fixture: its complete reference
        # baseline has observable timing error in Mock, so optional tuning is
        # needed. No fake torque dynamics or successful fit is injected.
        prepare_source(native, directory / 'source', names, 25)
        native.ready()
        task = native.request('model_calibration_import_trajectory', {'directory': str(directory / 'source'), 'pose_budget': 5})
        native.ready()
        native.request('model_calibration_start', {'task_id': task['task_id'], 'workflow': 'gain',
            'gain_mode': 'all', 'gain_tuning_mode': 'quick', 'released': True,
            'gain_trajectory_confirmed': True, 'gain_motion_range_confirmed': True})
        folder = Path(task['directory'])
        saw_isolated = False
        first_isolated_kind = None
        current_isolated_kind = None
        baseline_digest = None
        observed_kinds = []
        while time.monotonic() < deadline:
            status = native.request('model_calibration_status')
            assert native.request('status')['robot_state'] == 'ACTIVE', status
            kind = isolated_kind(status)
            if kind:
                if not saw_isolated:
                    baseline_digest, _ = baseline_evidence(folder, names)
                    first_isolated_kind = kind
                saw_isolated = True
                current_isolated_kind = kind
                if kind not in observed_kinds:
                    observed_kinds.append(kind)
            # A complete first traversal is required before any local input.
            # It is not evidence that optional identification was skipped.
            if partial and kind and kind != first_isolated_kind and status.get('gain_probe_elapsed_s', 0) > .65:
                break
            # All interior-axis inputs have finished when the actual worker
            # resumes its discovery path; no physical Mock fit may be adopted.
            if not partial and saw_isolated and status.get('gain_probe_stage') == 'trajectory_discovery':
                break
            assert status['phase'] not in ('complete', 'cancelled', 'failed'), status
            time.sleep(.03)
        else:
            raise AssertionError('The actual interior frequency worker did not reach its input stop point within the 180 s outer budget')
        assert saw_isolated and baseline_digest
        status, candidate = saved_stop(native, task, deadline)
        assert status['phase'] == 'cancelled' and candidate['termination']['kind'] == 'cancelled', status
        assert candidate['termination']['reason'] == 'stop_requested', candidate['termination']
        baseline_evidence(folder, names, baseline_digest)
        selected = candidate.get('targets', {}).get('rigid', candidate)
        validation = selected['native_validation']
        assert validation['completed'] and validation['available'] and validation['independent'], validation
        assert validation['record_path'] == 'gain-trajectory-native-frozen.csv' and validation['playback_rate'] == 1, validation
        assert candidate['available'] and candidate['review_ready'] and not candidate['validated'], candidate
        assert candidate['seed_source'] == candidate['model_initialization']['source'] and not candidate['model_initialization']['measured'], candidate
        assert selected['kp'] == candidate['seed_kp'] and selected['kd'] == candidate['seed_kd'], selected
        assert candidate['budget_source_duration_s'] == 25 and candidate['target_budget_s'] == 450 and candidate['time_budget_s'] == 600, candidate
        assert candidate['actual_elapsed_s'] < candidate['time_budget_s'] and not candidate['budget_exhausted'], candidate
        assert not candidate['response_guard_events'], candidate['response_guard_events']
        experiments = candidate['experiments']
        parameters = [item for item in experiments if item.get('kind') == 'short_experiment_parameters'
                      and item.get('isolated_axis_index') is not None]
        assert parameters, 'No actual isolated controller input was recorded'
        complete = []
        interrupted = []
        for item in parameters:
            raw = folder / item['record_path']
            assert raw.is_file(), item
            rows = read_rows(raw)
            completed = max(float(row['time_s']) for row in rows) >= item['duration_s'] + 1.5 - .08
            (complete if completed else interrupted).append(verify_isolated(folder, item, names, completed))
        assert complete, 'Stopping optional identification erased the first completed input'
        measurements = [item for item in experiments if item.get('kind') == 'mit_closed_loop_controller_experiment']
        plans = [item for item in experiments if item.get('kind') == 'mit_frequency_experiment_plan']
        proposals = [item for item in candidate['candidate_comparisons'] if item.get('kind') == 'mit_indirect_frequency_autotune']
        assert plans, 'The actual measured timing/model frequency plan was not recorded'
        for plan in plans:
            assert plan['urdf_local_inertia_kg_m2'] > 0 and plan['measured_feedback_period_s'] > 0, plan
            assert 0 <= plan['measured_feedback_age_s'] < .06 and plan['allocated_measurement_budget_s'] > 0, plan
            if plan['eligible']:
                assert plan['selected_center_hz'] > 0 and plan['probe_duration_s'] > 0, plan
                assert plan['complete_controller_set_cost_s'] <= plan['allocated_measurement_budget_s'] + 1e-6, plan
        for item in measurements:
            assert item['controller_index'] == 0 and not item['valid'] and item['missing_evidence'], item
            assert 'not_reconstructed' in item['input_definition'] and 'command_write' in item['time_definition'] and 'feedback_receive' in item['time_definition'], item
            assert any(value['record_path'] == item['record_path'] for value in complete), item
        assert all(not item['identified'] and not item['independent_gain_holdout_verified']
                   and not item['candidate_generated'] and not item['validated'] for item in proposals), proposals
        assert not any(item.get('kind') == 'full_path_empirical_PD_comparison'
                       for item in candidate['candidate_comparisons']), candidate['candidate_comparisons']
        if partial:
            assert len(parameters) == 2 and len(complete) == 1 and len(interrupted) == 1, (complete, interrupted)
            assert parameters[-1]['label'] == current_isolated_kind and current_isolated_kind != first_isolated_kind, parameters
        else:
            assert not interrupted and measurements and proposals, (interrupted, measurements, proposals)
            expected_axes = {names.index(item['joint']) for item in plans if item['eligible']}
            assert {item['axis_index'] for item in complete} == expected_axes, (complete, plans)
            assert {names.index(item['joint']) for item in measurements} == expected_axes, measurements
            assert {names.index(item['joint']) for item in proposals} == expected_axes, proposals
        frames = read_rows(folder / 'frames.csv')
        first_motion = next(row for row in frames if any(abs(float(row['ref_q:' + name])) > 1e-6 for name in names))
        assert first_motion['gain_stage'] == 'native-frozen', first_motion['gain_stage']
        assert not (folder / 'gain-trajectory-native-autotuned-frozen.csv').exists(), 'A torque-insensitive Mock generated a candidate validation'
        return {'partial_stop': partial, 'baseline_retained_before_optional_inputs': True,
                'baseline_record_path': validation['record_path'], 'completed_isolated_inputs': complete,
                'interrupted_isolated_inputs': interrupted, 'recorded_controller_estimates': len(measurements),
                'recorded_frequency_plans': len(plans), 'recorded_frequency_designs': len(proposals),
                'mock_frequency_plants_valid': False, 'available': candidate['available'],
                'worker_phase': status['phase'], 'termination': status['termination']}
    except Exception:
        evidence = Path(tempfile.mkdtemp(prefix='serialarm-isolated-frequency-failed-')) / 'evidence'
        shutil.copytree(directory, evidence)
        print('Retained failed native frequency fixture: ' + str(evidence), file=sys.stderr)
        print(native.diagnostics(), file=sys.stderr)
        raise
    finally:
        native.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terminal', type=Path, default=ROOT / 'build/unified/standalone-dm_arm-t1-p1-m0/core/serial_arm_terminal')
    args = parser.parse_args()
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix='serialarm-isolated-frequency-flow-') as folder:
        root = Path(folder)
        results = [fixture(args.terminal.resolve(), root / label, started + 180, partial)
                   for label, partial in (('complete', False), ('partial', True))]
    print(json.dumps({'offline_mock_only': True, 'elapsed_s': time.monotonic() - started, 'cases': results}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
