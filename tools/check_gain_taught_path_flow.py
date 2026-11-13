#!/usr/bin/env python3
"""Exercise moving taught-path adaptation and final frozen recording in native Mock.

Only temporary source records and write-disabled Mock configurations are used.
Mock follows position commands and ignores physical PD/torque. Its local
prediction fit or submitted PD change is never proof of a physical input
response or improvement. Sampled MIT response is exercised by the separate
C++ plant tests. This test verifies the actual native scheduling,
full-cycle evidence, frozen complete group, budgets, stop and application flow.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
from pathlib import Path
import sys
import tempfile
import time

import numpy as np

from check_adaptive_gain_flow import Native, ROOT, prepare_config, synthetic_record


def read_rows(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows and all(None not in row and None not in row.values() for row in rows), path
    return rows


def prepare_source(native, folder, names, recorded_t):
    synthetic_record(native, folder, names, recorded_t)
    # A smooth four-second one-way path beginning at its lower observed
    # endpoint. It has no per-joint tuning rule; the correlated pair verifies
    # that discovery follows the supplied path instead of a pose-box search.
    with (folder / 'trajectory.csv').open('w') as stream:
        stream.write('sample_dt,0.02\nindex,' + ','.join(names) + '\n')
        amplitudes = (1.0, .30, .34, .40, .36, .30)
        for index in range(201):
            u = index / 200
            q = [.5 * amplitudes[axis] * (1-math.cos(math.pi * u))
                 for axis in range(len(names))]
            q[1] = amplitudes[1] * u*u
            q[3] = .4 * q[0]
            q[4] = amplitudes[4] * (u+.08*math.sin(2*math.pi*u))
            q[5] = amplitudes[5] * u*u*(3-2*u)
            stream.write(str(index) + ',' + ','.join(format(value, '.17g') for value in q) + '\n')


def final_record(native, task, deadline):
    folder = Path(task['directory'])
    while time.monotonic() < deadline:
        status = native.request('model_calibration_status')
        if status['phase'] in ('complete', 'failed', 'cancelled'):
            try:
                candidate = json.loads((folder / 'gain-candidate.json').read_text())
                result = json.loads((folder / 'result.json').read_text())
            except (FileNotFoundError, json.JSONDecodeError):
                time.sleep(.03)
                continue
            if result.get('termination') == candidate.get('termination') == status.get('termination'):
                return candidate, result, status
        time.sleep(.03)
    raise AssertionError('Native did not persist its concrete final task and candidate within the fixture deadline')


def assert_nominal_path_basis(candidate, names):
    records = [item for item in candidate['experiments'] if item.get('kind') == 'model_initialization_conditions']
    assert len(records) == 1, records
    record = records[0]
    model = record['ideal_modal_model']
    basis = 'time_weighted_mean_of_URDF_path_mass_matrices'
    assert not record['measured'] and not model['measured'] and model['posture_rad'] is None, record
    assert record['damping_posture_aggregation'] == model['mass_matrix_basis'] == basis, record
    times = np.asarray(record['inertia_sample_times_s'], dtype=float)
    weights = np.asarray(record['inertia_time_weights'], dtype=float)
    assert len(times) > 1 and times.shape == weights.shape and np.all(np.isfinite(times))
    assert np.all(np.diff(times) > 0) and np.all(np.isfinite(weights)) and np.all(weights > 0)
    expected_weights = np.zeros_like(times)
    half_intervals = .5*np.diff(times)/(times[-1]-times[0])
    expected_weights[:-1] += half_intervals
    expected_weights[1:] += half_intervals
    np.testing.assert_allclose(weights, expected_weights, rtol=1e-10, atol=1e-12)
    assert math.isclose(float(np.sum(weights)), 1, abs_tol=1e-12)
    mass = np.asarray(model['mass_matrix_kg_m2'], dtype=float)
    damping = np.asarray(model['local_full_damping_matrix'], dtype=float)
    kp = np.asarray(record['damping_stiffness_basis_kp'], dtype=float)
    assert mass.shape == damping.shape == (len(names), len(names)) and kp.shape == (len(names),)
    assert np.all(np.isfinite(mass)) and np.all(np.isfinite(damping)) and np.all(kp > 0)
    np.testing.assert_allclose(mass, mass.T, rtol=1e-10, atol=1e-12)
    np.testing.assert_allclose(damping, damping.T, rtol=1e-10, atol=1e-12)
    np.linalg.cholesky(mass)
    np.testing.assert_allclose(kp, candidate['model_initialization']['kp'], rtol=1e-10, atol=1e-12)
    np.testing.assert_allclose(kp, candidate['seed_kp'], rtol=1e-10, atol=1e-12)
    np.testing.assert_allclose(kp, model['local_design_kp'], rtol=1e-10, atol=1e-12)
    np.testing.assert_allclose(np.diag(damping), record['damping_before_protocol_kd'], rtol=1e-10, atol=1e-12)
    # A DD design on the recorded M and final complete K obeys this identity;
    # it detects damping formed from another posture or pre-constraint K.
    zeta = record['design_parameters']['damping_ratio']
    np.testing.assert_allclose(damping @ np.linalg.solve(mass, damping), 4*zeta*zeta*np.diag(kp),
                               rtol=2e-7, atol=1e-9)
    minimum = np.asarray(candidate['model_initialization']['inertia_min'])
    maximum = np.asarray(candidate['model_initialization']['inertia_max'])
    assert np.all(np.diag(mass) >= minimum-1e-10) and np.all(np.diag(mass) <= maximum+1e-10)
    return {'basis': basis, 'inertia_samples': len(times), 'time_weights_sum': float(np.sum(weights)),
            'final_complete_kp_basis': True, 'full_damping_identity_verified': True, 'measured': False}


def assert_reference_only(value):
    assert value['reference_only'] is True and value['accuracy_thresholds_enabled'] is False, value
    if 'acceptance_scope' in value:
        assert value['acceptance_scope'] == 'measured_response_reference_only_v1', value


def assert_anchor_visits(folder, candidate, names, partial):
    with (folder / 'trajectory.csv').open() as stream:
        reader = csv.reader(stream)
        sample_dt = float(next(reader)[1])
        next(reader)
        positions = np.asarray([[float(value) for value in row[1:]] for row in reader])
    lower, upper = np.min(positions, axis=0), np.max(positions, axis=0)
    span = upper-lower
    first_axis_direction = 1 if positions[-1, 0] > positions[0, 0] else -1
    assert np.all(span > .05), 'The fixture must offer real interior room on every axis'
    experiments = candidate['experiments']
    visits = [item for item in experiments if item.get('kind') == 'taught_measurement_anchor_visit']
    assert visits, 'The real worker never visited an interior taught pose before purpose measurements'
    completed = [item for item in visits if item['completed']]
    paired, measured_records, drag_preparations = {}, [], []
    active_visit = None
    for item in experiments:
        if item.get('kind') == 'taught_measurement_anchor_visit':
            assert item['arrival_error_gate'] is False and item['reference_only'] is True, item
            source, destination = item['source_index_from'], item['source_index_to']
            assert 0 <= source < len(positions) and 0 <= destination < len(positions) and source != destination, item
            np.testing.assert_allclose(item['reference_rad'], positions[destination], atol=1e-10, rtol=0)
            assert item['planned_cost_s'] > 0 and item['elapsed_s'] >= 0, item
            record = item['record_path']
            assert record.startswith('gain-anchor-') and ('purpose-visit' in record or 'purpose-return' in record), item
            label = record.split('gain-anchor-', 1)[1].split('-purpose-', 1)[0]
            if 'purpose-visit' in record:
                assert 0 < destination < len(positions)-1, 'The worker substituted an endpoint for the planned interior'
                assert np.all((positions[destination]-lower)/span > .05) and np.all((upper-positions[destination])/span > .05), item
                paired[label] = {'visit': item}
                active_visit = item if item['completed'] else None
            else:
                assert label in paired and 'visit' in paired[label], item
                visit = paired[label]['visit']
                assert source == visit['source_index_to'] and destination == visit['source_index_from'], 'Return jumped to another occurrence of the same pose'
                paired[label]['return'] = item
                active_visit = None
            if not item['completed']:
                assert partial, 'Complete fixture unexpectedly abandoned a planned source-path visit'
                continue
            assert item['elapsed_s']+.02 >= item['planned_cost_s'], 'The worker declared arrival before its time map completed'
            rows = read_rows(folder / record)
            groups = [[row for row in rows if row['joint'] == name] for name in names]
            assert min(map(len, groups)) >= 100 and len(set(map(len, groups))) == 1, record
            source_range = sorted((source, destination))
            previous_progress = float(source)
            direction = 1 if destination > source else -1
            for sample in list(zip(*groups))[::31]:
                q = np.array([float(row['reference']) for row in sample])
                assert all(float(row['independent_disturbance_nm']) == 0 for row in sample), 'Anchor travel injected a stationary probe'
                # The fixture's first coordinate is monotonic. Invert that
                # original Hermite coordinate to recover the actual source
                # branch, then check every other coordinate on that branch.
                segment = min(max(int(np.searchsorted(first_axis_direction*positions[:, 0],
                    first_axis_direction*q[0], side='right'))-1, 0), len(positions)-2)
                def original(u):
                    left = positions[segment]-positions[segment-1] if segment else np.zeros(len(names))
                    middle = positions[segment+1]-positions[segment]
                    right = positions[segment+2]-positions[segment+1] if segment+2 < len(positions) else np.zeros(len(names))
                    def slope(a, b):
                        result = np.zeros(len(names))
                        mask = a*b > 0
                        result[mask] = 2*a[mask]*b[mask]/(a[mask]+b[mask])
                        return result
                    m0, m1 = slope(left, middle), slope(middle, right)
                    return ((2*u**3-3*u*u+1)*positions[segment] + (u**3-2*u*u+u)*m0
                            + (-2*u**3+3*u*u)*positions[segment+1] + (u**3-u*u)*m1)
                lo, hi = 0., 1.
                for _ in range(32):
                    midpoint = .5*(lo+hi)
                    if first_axis_direction*original(midpoint)[0] < first_axis_direction*q[0]:
                        lo = midpoint
                    else:
                        hi = midpoint
                u = .5*(lo+hi)
                progress = segment+u
                assert source_range[0]-1e-6 <= progress <= source_range[1]+1e-6, (record, progress, source_range)
                assert direction*(progress-previous_progress) >= -1e-6, 'Anchor traversal reversed its source path branch'
                np.testing.assert_allclose(q, original(u), atol=1e-7, rtol=0,
                                           err_msg='Anchor travel took a straight joint-space shortcut')
                previous_progress = progress
        elif item.get('kind') == 'short_experiment_parameters' and item.get('independent_disturbance'):
            assert active_visit is not None, (item['record_path'], 'Default purpose disturbance ran outside an interior visit')
            center = np.asarray(item['center_rad'])
            assert np.all((center-lower)/span > .05) and np.all((upper-center)/span > .05), (item['record_path'], center)
            # This fixture tracks the path directly; every axis has room for a
            # two-sided test. Do not generalize its success to physical PD.
            assert all(item['disturbance_axis_eligible']) and all(axis['bidirectional'] for axis in item['independent_input_axes']), item
            measured_records.append(item['record_path'])
        elif item.get('kind') == 'drag_excitation_parameters':
            assert active_visit is not None, 'Drag preparation fell back to the boundary endpoint'
            center = np.asarray(item['center_rad'])
            assert np.all((center-lower)/span > .05) and np.all((upper-center)/span > .05), item
            drag_preparations.append(item)
    primary = 'drag' if candidate['category'] == 'compliant_drag' else 'native'
    assert primary in paired and {'visit', 'return'} <= paired[primary].keys(), paired
    assert paired[primary]['visit']['completed'] and paired[primary]['return']['completed'], paired[primary]
    if not partial and candidate['category'] == 'all':
        assert 'compliant' in paired and {'visit', 'return'} <= paired['compliant'].keys(), paired
        assert len(visits) == 4 and all(item['completed'] for item in visits), visits
    elif not partial:
        assert len(visits) == 2 and all(item['completed'] for item in visits), visits
    measured_visit_cost = sum(item['elapsed_s'] for item in visits)
    assert 0 < measured_visit_cost <= candidate['actual_elapsed_s'], 'Visit wall time was omitted from the automatic-stage budget'
    assert candidate['budget_source_duration_s'] == 25, 'Interior retiming replaced trusted original T'
    assert measured_records or (primary == 'drag' and drag_preparations), 'Interior travel did not produce a real purpose measurement'
    return {'visits': [{key: item[key] for key in ('record_path', 'source_index_from', 'source_index_to',
                      'planned_cost_s', 'elapsed_s', 'completed')} for item in visits],
            'actual_visit_cost_s': measured_visit_cost, 'original_T_unchanged': True,
            'source_duration_s': sample_dt*(len(positions)-1),
            'interior_disturbance_records': measured_records, 'arrival_error_gate': False}


def assert_online_adaptation(folder, candidate, names, partial):
    comparisons = candidate['candidate_comparisons']
    assert not any(item.get('kind') in ('offline_closed_loop_candidate', 'offline_output_error_model',
                   'full_path_local_candidate', 'full_path_empirical_PD_comparison') for item in comparisons), comparisons
    assert not list(folder.glob('*comparison-return*.csv')), 'A return-only full traversal consumed the learning budget'
    assert not list(folder.glob('*autotuned-frozen*.csv')), 'Detached constant-gain polling re-entered the path workflow'
    assert not list(folder.glob('servo-probe-*-establish-*.csv')), 'Stationary startup probing replaced taught-path motion'
    updates = [item for item in comparisons if item.get('kind') == 'online_trajectory_PD_update']
    adaptive_files = sorted(folder.glob('gain-trajectory-*-online-*.csv'))
    assert adaptive_files and updates, 'The actual moving path never reached online model/update processing'
    with (folder / 'trajectory.csv').open() as stream:
        source = csv.reader(stream)
        sample_dt = float(next(source)[1])
        next(source)
        positions = [[float(value) for value in row[1:]] for row in source]
    domain = [(min(q[axis] for q in positions), max(q[axis] for q in positions)) for axis in range(len(names))]
    duration = sample_dt*(len(positions)-1)
    records, changed_records = [], []
    for path in adaptive_files:
        rows = read_rows(path)
        groups = [[row for row in rows if row['joint'] == name] for name in names]
        assert min(map(len, groups)) >= 100 and len(set(map(len, groups))) == 1, path
        events = [item for item in updates if item['source_record_path'] == path.name]
        assert events, (path, 'No online diagnostic accompanies the recorded learning traversal')
        changed = False
        for axis, values in enumerate(groups):
            assert all(row['actual_gains_known'] == '1' for row in values), names[axis]
            assert all(math.isfinite(float(row[key])) for row in values for key in
                       ('reference', 'reference_velocity', 'actual', 'velocity', 'actual_kp', 'actual_kd',
                        'command_time_s', 'feedback_time_s', 'independent_disturbance_nm')), names[axis]
            stamps = [int(row['feedback_sequence']) for row in values if row['new_measurement'] == '1']
            assert len(stamps) >= 100 and all(a < b for a, b in zip(stamps, stamps[1:])), names[axis]
            assert all(domain[axis][0]-1e-8 <= float(row['reference']) <= domain[axis][1]+1e-8 for row in values), (names[axis], domain[axis])
            assert max(abs(float(row['reference_velocity'])) for row in values) > 1e-5, (names[axis], 'No moving reference')
            assert max(abs(float(row['actual'])) for row in values) > 1e-5, (names[axis], 'Mock did not execute the path')
            clocked = [row for row in values if row['reference_sample_time_known'] == '1']
            clocks = [float(row['reference_sample_time_s']) for row in clocked]
            assert len(clocked) >= 100 and all(a < b for a, b in zip(clocks, clocks[1:])), names[axis]
            assert all(abs(float(row['reference_sample_time_s'])-float(row['command_time_s'])) < .06 for row in clocked), names[axis]
            actual_codes = {(row['actual_kp'], row['actual_kd']) for row in values if float(row['time_s']) < duration}
            changed |= len(actual_codes) > 1
        assert all(abs(float(values[3]['reference'])-.4*float(values[0]['reference'])) < 1e-7
                   for values in zip(*groups)), 'The moving reference departed from the supplied correlated taught path'
        assert max(abs(float(row['independent_disturbance_nm'])) for row in rows) > 0, (path, 'No independent known excitation was executed during path learning')
        for event in events:
            assert_reference_only(event)
            assert event['method'] == 'indirect_local_IV_ARX_projected_PD', event
            assert 0 < event['time_s'] < duration, 'Model revisions happened only after motion completed'
            assert len(event['models']) == len(event['reasons']) == len(names), event
            assert all(model.get('reason') for model in event['models']), event
            assert len(event['previous_kp']) == len(event['previous_kd']) == len(event['proposed_kp']) == len(event['proposed_kd']) == len(names)
            if event['installed']:
                assert event['candidate_generated'] and event['admission_allowed'], event
                for axis, values in enumerate(groups):
                    if event['previous_kp'][axis] == event['proposed_kp'][axis] and event['previous_kd'][axis] == event['proposed_kd'][axis]:
                        continue
                    assert event['models'][axis]['valid'], event
                    subsequent = [row for row in values if event['time_s'] <= float(row['time_s']) < duration]
                    assert any(math.isclose(float(row['actual_kp']), event['proposed_kp'][axis], abs_tol=1e-6)
                               and math.isclose(float(row['actual_kd']), event['proposed_kd'][axis], abs_tol=1e-6)
                               for row in subsequent), 'An installed gain revision never reached actual command samples during motion'
            else:
                assert not event['candidate_generated'] or not event['admission_allowed'], event
        if not changed:
            assert not any(event['installed'] for event in events), 'Mock recorded unchanged gains but claimed a successful moving update'
            assert any(not model['valid'] for event in events for model in event['models']), 'Unchanged Mock requires concrete insufficient-response evidence'
        else:
            assert any(event['installed'] for event in events), 'Gains changed without an admitted online update record'
            changed_records.append(path.name)
        records.append({'path': path.name, 'rows': len(rows), 'online_windows': len(events), 'actual_gains_changed': changed})
    frames = read_rows(folder / 'frames.csv')
    first_motion = next(row for row in frames if any(abs(float(row['ref_q:' + name])) > 1e-6 for name in names))
    assert 'online' in first_motion['gain_stage'], first_motion['gain_stage']
    assert not any('comparison-return' in row['gain_stage'] or 'measurement-entry' in row['gain_stage']
                   or 'establish' in row['gain_stage'] for row in frames)
    assert not any(item.get('kind') == 'mit_closed_loop_controller_experiment' for item in candidate['experiments']), 'The path workflow fell back to stationary multi-controller identification'
    if not partial:
        assert len(adaptive_files) == 2, 'Four-use quick mode must learn once for native and once for compliant before freezing each group'
    return {'records': records, 'actual_gains_changed_records': changed_records,
            'first_motion_stage': first_motion['gain_stage'], 'physical_identification_is_not_assumed_from_Mock': True}


def assert_frozen_group(folder, selected, names):
    validation = selected['native_validation']
    assert validation['completed'] and validation['available'] and validation['independent'], validation
    assert_reference_only(validation)
    assert validation['method'] == 'demonstrated_trajectory_tracking', validation
    assert validation['online_adaptation'] is False, 'A varying-controller learning pass became the final frozen record'
    assert validation['reference_schedule'] == 'control_cycle_monotonic' and validation['playback_rate'] == 1, validation
    assert validation['validated'] is False, validation
    rows = read_rows(folder / validation['record_path'])
    counts = {}
    for axis, name in enumerate(names):
        values = [row for row in rows if row['joint'] == name]
        assert len(values) >= 100 and max(float(row['time_s']) for row in values) >= validation['duration_s'], (name, validation)
        assert all(row['actual_gains_known'] == '1' for row in values), name
        assert all(math.isclose(float(row['actual_kp']), selected['kp'][axis], abs_tol=1e-6)
                   and math.isclose(float(row['actual_kd']), selected['kd'][axis], abs_tol=1e-6) for row in values), name
        metric = validation['metrics'][axis]
        assert metric['measurement_samples'] >= 100 and metric['endpoint_window_available'], metric
        assert metric['endpoint_passed'] is False, metric
        endpoint = [r for r in values if metric['endpoint_window_start_s'] <= float(r['time_s']) <= metric['endpoint_window_end_s']+1e-9]
        raw_peak = max(abs(float(r['reference'])-float(r['actual'])) for r in endpoint)
        assert math.isclose(raw_peak, metric['endpoint_peak_error_rad'], abs_tol=1e-9), metric
        assert all(float(row['independent_disturbance_nm']) == 0 for row in values), 'Final frozen recording must exclude identification excitation'
        counts[name] = len(values)
    return {'record_path': validation['record_path'], 'rows_by_joint': counts, 'frozen_complete_group': True}


def assert_frozen(folder, candidate, names):
    targets = candidate.get('targets', {})
    selected = targets.get('servo_tracking', candidate)
    validation = selected['native_validation']
    native_group = assert_frozen_group(folder, selected, names)
    compliant_group = assert_frozen_group(folder, targets['compliant_tracking'], names) if targets else None
    independent = [item for item in candidate['experiments'] if item.get('kind') == 'independent_torque_disturbance']
    assert independent, [item.get('kind') for item in candidate['experiments']]
    one_sided_axes = set()
    for experiment in independent:
        torque_rows = read_rows(folder / experiment['record_path'])
        assert max(abs(float(row['independent_disturbance_nm'])) for row in torque_rows) > 0, experiment
        parameters = next(item for item in candidate['experiments'] if item.get('kind') == 'short_experiment_parameters'
                          and item.get('record_path') == experiment['record_path'])
        assert parameters['independent_disturbance'] and math.isclose(parameters['independent_input_baseline_s'], .25), parameters
        assert len(parameters['independent_input_axes']) == len(names), parameters
        for axis, name in enumerate(names):
            axis_rows = [row for row in torque_rows if row['joint'] == name]
            values = [float(row['reference']) for row in axis_rows]
            assert max(values)-min(values) < 1e-7, (name, experiment)
            baseline = [row for row in axis_rows if float(row['time_s']) < .2]
            assert len(baseline) >= 20 and all(abs(float(row['independent_disturbance_nm'])) < 1e-10 for row in baseline), (name, 'No actual unloaded input baseline')
            direction = parameters['independent_input_axes'][axis]
            if parameters['disturbance_axis_eligible'][axis] and not direction['bidirectional']:
                assert direction['direction'] in (-1, 1) and direction['validation_scope'] in (
                    'positive_only_known_joint_torque', 'negative_only_known_joint_torque'), direction
                applied = [direction['direction']*float(row['independent_disturbance_nm']) for row in axis_rows]
                assert min(applied) >= -1e-10 and max(applied) > 0, (name, direction, min(applied), max(applied))
                one_sided_axes.add(name)
        models = [axis for axis in experiment['axes'] if 'disturbance_valid' in axis]
        assert len(models) == len(names) and not all(axis['disturbance_valid'] for axis in models), models
    assert_reference_only(candidate)
    assert not candidate['validated'], 'Recorded references must not be serialized as achieved physical acceptance'
    assert not selected['disturbance_verification']['validated'], selected['disturbance_verification']
    assert candidate['available'] and candidate['review_ready'] and candidate['measurement_complete'], 'Complete frozen tracking parameters must remain available with independent-disturbance warnings'
    assert not selected['qualification_complete'], 'Missing independent-disturbance evidence cannot become complete purpose qualification'
    if targets:
        assert set(targets) == {'rigid_tracking', 'compliant_tracking', 'servo_tracking', 'compliant_drag'}, targets.keys()
        assert targets['rigid_tracking']['native_validation']['record_path'] == validation['record_path'], 'Rigid and servo duplicated the shared native frozen record'
        assert all(target['available'] for target in targets.values()), 'Completed purpose parameter groups were erased after missing independent-input evidence'
        assert all(target['validated'] is False for target in targets.values()), 'A reference record became an automatic purpose pass'
        assert all(value == 0 for value in targets['compliant_drag']['kp']), targets['compliant_drag']
        rigid = targets['rigid_tracking']
        assert rigid['hold_disturbance_verification']['record_path'] == rigid['disturbance_verification']['record_path'], 'Rigid hold re-injected an already measured same-condition purpose input'
        compliant = targets['compliant_tracking']
        assert compliant['hold_disturbance_verification']['record_path'] == compliant['disturbance_verification']['record_path'], 'Compliant hold duplicated the same-condition purpose input'
        shared = [item for item in candidate['candidate_comparisons'] + candidate['experiments']
                  if item.get('kind') == 'shared_disturbance_evidence']
        assert shared, 'Shared purpose evidence was not recorded as reused evidence'
        known_records = {item['record_path']: item for item in independent}
        assert all(item['record_path'] in known_records for item in shared), shared
    return {'native': native_group, 'compliant': compliant_group,
            'frozen_complete_group': True, 'independent_disturbance_records': [item['record_path'] for item in independent],
            'one_sided_disturbance_axes': sorted(one_sided_axes), 'unloaded_input_baseline_s': .25,
            'parameter_measurement_complete': candidate['measurement_complete'],
            'qualification_complete': selected['qualification_complete'], 'review_ready': candidate['review_ready'],
            'validated': candidate['validated']}


def run_case(terminal, kind='complete', old_gain_scale=1.0, timeout_s=660):
    native = None
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix='serialarm-taught-path-') as temporary:
        directory = Path(temporary)
        config, names = prepare_config(directory, old_gain_scale)
        # Import retimes geometry against the active limits, ignoring the
        # synthetic CSV clock. Give this temporary Mock enough moving samples
        # for a local model window; original recorded T stays 25 seconds.
        document = json.loads(config.read_text())
        document['safety_policy']['cmd_vel_scale'] = min(document['safety_policy']['cmd_vel_scale'], .03)
        document['safety_policy']['max_acc'] = {name: min(value, .6)
            for name, value in document['safety_policy']['max_acc'].items()}
        recovery = document['safety_policy']['fault_recovery']['compliant_recovery']
        recovery['max_vel'] = {name: min(value, .03) for name, value in recovery['max_vel'].items()}
        config.write_text(json.dumps(document, indent=2))
        before_config = config.read_text()
        storage = directory / 'storage'
        storage.mkdir()
        try:
            native = Native(terminal, config, storage)
            original_gains = native.request('get_admittance')['execution_gains']
            native.request('activate')
            native.ready()
            prepare_source(native, directory / 'source', names, 25)
            native.ready()
            task = native.request('model_calibration_import_trajectory', {'directory': str(directory / 'source'), 'pose_budget': 5})
            # Stop all-use quick mode after native freezing, during its later
            # compliant learning. A fine single-use task now freezes last and
            # completes immediately; do not invent an idle cancellation window.
            mode, depth = ('servo_tracking', 'fine') if kind == 'fine' else ('all', 'quick')
            hard_limit = 375 if depth == 'quick' else 600
            plan = native.request('model_calibration_gain_plan', {'gain_mode': mode, 'gain_tuning_mode': depth})
            assert plan['valid'] and plan['budget_source_duration_s'] == 25 and plan['target_budget_s'] == 300 and plan['hard_limit_s'] == hard_limit, plan
            assert plan['max_path_passes'] == (4 if depth == 'quick' else 3), plan
            native.ready()
            native.request('model_calibration_start', {'task_id': task['task_id'], 'workflow': 'gain', 'gain_mode': mode,
                'gain_tuning_mode': depth, 'released': True, 'gain_trajectory_confirmed': True, 'gain_motion_range_confirmed': True,
                'gain_requirements': {'peak_error_rad': -1, 'rms_error_rad': 'ignored historical target',
                                      'hold_error_rad': 0, 'settling_time_s': -10}})
            stages = []
            deadline = started + timeout_s
            while time.monotonic() < deadline:
                status = native.request('model_calibration_status')
                assert native.request('status')['robot_state'] == 'ACTIVE', status
                assert_reference_only(status)
                stage = status.get('gain_probe_stage')
                if status.get('gain_probe_kind', '').startswith('native-online-'):
                    assert not status['gain_available'], 'A varying-controller learning pass became applicable before frozen recording'
                if stage and stage not in stages:
                    stages.append(stage)
                assert stage not in ('measurement_center_entry', 'motion_establishment'), status
                if (kind == 'cancel' and status.get('gain_path_pass_current', 0) >= 3
                        and 'compliant-purpose-visit' in status.get('gain_probe_kind', '')
                        and stage == 'trajectory_measurement_visit'):
                    native.request('model_calibration_cancel', {'task_id': task['task_id']})
                    break
                if status['phase'] in ('complete', 'failed', 'cancelled'):
                    break
                time.sleep(.03)
            else:
                raise AssertionError('The actual taught-path worker did not reach the requested fixture exit')
            candidate, result, status = final_record(native, task, deadline)
            folder = Path(task['directory'])
            assert candidate['termination'] == result['termination'] == status['termination']
            assert candidate['category'] == ('all' if mode == 'all' else 'servo_tracking'), candidate['category']
            assert candidate['seed_source'].startswith('urdf') and candidate['seed_source'] == candidate['model_initialization']['source'], candidate['seed_source']
            assert not candidate['model_initialization']['measured'] and not candidate['task_model_demand']['executed']
            nominal_basis = assert_nominal_path_basis(candidate, names)
            assert candidate['budget_source_duration_s'] == 25 and candidate['target_budget_s'] == 300 and candidate['time_budget_s'] == hard_limit
            assert candidate['budget_source_task_id'] == 'offline-original-25s'
            requirements = candidate['gain_requirements']
            assert requirements['acceptance_scope'] == 'measured_response_reference_only_v1', requirements
            assert requirements['accuracy_thresholds_enabled'] is False and requirements['legacy_accuracy_fields_ignored'] is True, requirements
            assert not {'peak_error_rad', 'rms_error_rad', 'hold_error_rad', 'settling_time_s'} & requirements.keys(), 'Ignored historical precision fields re-entered the recorded scoring contract'
            assert candidate['actual_elapsed_s'] <= hard_limit+1, 'Original wall budget was exceeded'
            assert candidate['response_guard_events'] == []
            learning = assert_online_adaptation(folder, candidate, names, kind == 'cancel')
            anchors = assert_anchor_visits(folder, candidate, names, kind == 'cancel')
            if kind == 'cancel':
                assert status['phase'] == 'cancelled' and candidate['termination']['kind'] == 'cancelled', status
                assert candidate['termination']['reason'] == 'stop_requested' and not candidate['termination']['fault_interrupted'], candidate['termination']
                native_target = candidate['targets']['servo_tracking']
                assert native_target['native_validation']['completed'] and native_target['native_validation']['available'], 'Cancelling a later purpose measurement erased the completed native frozen group'
                assert candidate['available'] and candidate['review_ready'] and not candidate['validated'], candidate
                assert not candidate['targets']['compliant_tracking']['available'], 'An interrupted compliant group became applicable without its frozen traversal'
                frozen = assert_frozen_group(folder, native_target, names)
                frozen['frozen_group_retained_after_cancel'] = True
            else:
                assert status['phase'] == 'complete' and candidate['termination']['kind'] in ('measured_reference_complete', 'insufficient_measurement'), candidate['termination']
                assert not candidate['termination']['fault_interrupted'], candidate['termination']
                frozen = assert_frozen(folder, candidate, names)
                if kind == 'fine':
                    assert not candidate.get('targets'), 'Single-use fine tuning serialized or overwrote other purposes'
                    assert candidate['path_passes'] == 3 and candidate['path_pass_limit'] == 3, candidate
                    assert len(list(folder.glob('gain-trajectory-native-online-*.csv'))) == 2
                    assert not list(folder.glob('gain-trajectory-compliant-*.csv')), 'Fine servo tuning unexpectedly executed another purpose'
                else:
                    assert candidate['path_passes'] == 4 and candidate['path_pass_limit'] == 4, candidate
            assert native.request('get_admittance')['execution_gains'] == original_gains, 'The worker retained temporary calibration gains after exit'
            assert config.read_text() == before_config, 'The worker modified its input Profile'
            native.request('deactivate')
            applied = native.request('candidate_gain_apply', {'path': str(folder / 'gain-candidate.json'), 'gain_purposes': ['servo_tracking']})
            assert applied['gain_purposes'] == ['servo_tracking'], applied
            now = native.request('get_admittance')['execution_gains']
            target = candidate.get('targets', {}).get('servo_tracking', candidate)
            assert now['servo_tracking'] == {'kp': target['kp'], 'kd': target['kd']}, now
            assert {key: value for key, value in now.items() if key != 'servo_tracking'} == {
                key: value for key, value in original_gains.items() if key != 'servo_tracking'}, now
            native.request('candidate_gain_restore')
            assert native.request('get_admittance')['execution_gains'] == original_gains
            assert config.read_text() == before_config, 'Temporary candidate application wrote the Profile'
            return {'offline_mock_only': True, 'kind': kind, 'elapsed_s': time.monotonic()-started,
                    'actual_elapsed_s': candidate['actual_elapsed_s'],
                    'old_gain_scale': old_gain_scale, 'old_servo_gains': original_gains['servo_tracking'],
                    'seed_source': candidate['seed_source'], 'seed_kp': candidate['seed_kp'], 'seed_kd': candidate['seed_kd'],
                    'nominal_path_basis': nominal_basis,
                    'recorded_T_s': candidate['budget_source_duration_s'], 'hard_limit_s': candidate['time_budget_s'],
                    'observed_stages': stages, 'online_learning': learning, 'anchor_visits': anchors, 'frozen': frozen,
                    'legacy_accuracy_targets_ignored': True,
                    'termination': candidate['termination'], 'available': candidate['available'],
                    'validated': candidate['validated'], 'physical_stiffness_qualification': False}
        except Exception:
            evidence = Path(tempfile.mkdtemp(prefix='serialarm-taught-path-failed-')) / 'evidence'
            shutil.copytree(directory, evidence)
            print('Retained failed native fixture: ' + str(evidence), file=sys.stderr)
            if native:
                print(native.diagnostics(), file=sys.stderr)
            raise
        finally:
            if native:
                native.close()


def run_cold_drag_case(terminal, timeout_s=180):
    """Drag-only fine starts near, but not exactly on, its taught endpoint."""
    native = None
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix='serialarm-cold-drag-') as temporary:
        directory = Path(temporary)
        config, names = prepare_config(directory)
        before_config = config.read_text()
        storage = directory / 'storage'
        storage.mkdir()
        try:
            native = Native(terminal, config, storage)
            original_gains = native.request('get_admittance')['execution_gains']
            native.request('activate')
            native.ready()
            source = directory / 'source'
            prepare_source(native, source, names, 25)
            # All joints use the same geometric offset. This reproduces a
            # valid nearby measured start without an exact endpoint credential.
            lines = (source / 'trajectory.csv').read_text().splitlines()
            shifted = lines[:2]
            for line in lines[2:]:
                fields = line.split(',')
                shifted.append(fields[0]+','+','.join(format(float(value)+.005, '.17g') for value in fields[1:]))
            (source / 'trajectory.csv').write_text('\n'.join(shifted)+'\n')
            task = native.request('model_calibration_import_trajectory', {'directory': str(source), 'pose_budget': 5})
            plan = native.request('model_calibration_gain_plan', {'gain_mode': 'drag', 'gain_tuning_mode': 'fine'})
            assert plan['valid'] and plan['budget_source_duration_s'] == 25 and plan['target_budget_s'] == 300 and plan['hard_limit_s'] == 600, plan
            native.ready()
            native.request('model_calibration_start', {'task_id': task['task_id'], 'workflow': 'gain',
                'gain_mode': 'drag', 'gain_tuning_mode': 'fine', 'released': True,
                'gain_trajectory_confirmed': True, 'gain_motion_range_confirmed': True})
            deadline = started+timeout_s
            while time.monotonic() < deadline:
                status = native.request('model_calibration_status')
                assert native.request('status')['robot_state'] == 'ACTIVE', status
                if status['phase'] in ('complete', 'failed', 'cancelled'):
                    break
                time.sleep(.03)
            else:
                raise AssertionError('Cold-start drag-only measurement did not finish')
            candidate, result, status = final_record(native, task, deadline)
            assert status['phase'] == 'complete' and candidate['termination']['kind'] == 'measured_reference_complete', candidate['termination']
            assert not candidate['termination']['fault_interrupted'] and candidate['termination'] == result['termination']
            assert candidate['category'] == 'compliant_drag' and not candidate.get('targets'), candidate.get('category')
            assert_reference_only(candidate)
            assert candidate['available'] and candidate['review_ready'] and not candidate['validated']
            assert candidate['path_passes'] == 0, 'Drag-only measurement invented a full tracking traversal'
            assert candidate['budget_source_duration_s'] == 25 and candidate['target_budget_s'] == 300 and candidate['time_budget_s'] == 600
            assert candidate['actual_elapsed_s'] <= 601 and candidate['budget_source_task_id'] == 'offline-original-25s'
            assert all(value == 0 for value in candidate['kp']) and all(math.isfinite(value) and value >= 0 for value in candidate['kd']), candidate
            folder = Path(task['directory'])
            assert not list(folder.glob('gain-trajectory-*-online-*.csv')) and not list(folder.glob('gain-trajectory-*-frozen.csv'))
            anchors = assert_anchor_visits(folder, candidate, names, False)
            assert len(anchors['visits']) == 2 and all('drag-purpose-' in item['record_path'] for item in anchors['visits'])
            handoffs = [item for item in candidate['experiments']
                        if item.get('kind') == 'feedback_bounded_endpoint_transition'
                        and 'drag-purpose-visit-handoff' in item.get('record_path', '')]
            assert len(handoffs) == 1 and handoffs[0]['completed'], 'Nearby cold start did not use the existing measured endpoint bridge'
            assert handoffs[0]['reference_continuation'] is False, 'A cold start invented a completed source-knot credential'
            np.testing.assert_allclose(handoffs[0]['initial_actual_rad'], np.zeros(len(names)), atol=1e-8, rtol=0)
            np.testing.assert_allclose(handoffs[0]['target_rad'], np.full(len(names), .005), atol=1e-8, rtol=0)
            assert candidate['drag_validation']['completed'] and candidate['drag_validation']['available']
            drag_rows = read_rows(Path(candidate['drag_validation']['record_path']))
            assert all(any(row['joint'] == name for row in drag_rows) for name in names)
            assert native.request('get_admittance')['execution_gains'] == original_gains and config.read_text() == before_config
            native.request('deactivate')
            applied = native.request('candidate_gain_apply', {'path': str(folder / 'gain-candidate.json'), 'gain_purposes': ['compliant_drag']})
            assert applied['gain_purposes'] == ['compliant_drag'], applied
            now = native.request('get_admittance')['execution_gains']
            assert now['compliant_drag'] == {'kp': candidate['kp'], 'kd': candidate['kd']}, (applied, now)
            assert {key: value for key, value in now.items() if key != 'compliant_drag'} == {
                key: value for key, value in original_gains.items() if key != 'compliant_drag'}, now
            native.request('candidate_gain_restore')
            assert native.request('get_admittance')['execution_gains'] == original_gains and config.read_text() == before_config
            return {'offline_mock_only': True, 'kind': 'drag', 'elapsed_s': time.monotonic()-started,
                    'actual_elapsed_s': candidate['actual_elapsed_s'],
                    'recorded_T_s': 25, 'hard_limit_s': 600, 'endpoint_offset_rad': .005,
                    'anchor_visits': anchors, 'cold_handoff_record': handoffs[0]['record_path'],
                    'path_passes': candidate['path_passes'], 'kp': candidate['kp'], 'kd': candidate['kd'],
                    'available': candidate['available'], 'validated': candidate['validated'],
                    'only_selected_purpose_applied_and_restored': True, 'termination': candidate['termination']}
        except Exception:
            evidence = Path(tempfile.mkdtemp(prefix='serialarm-cold-drag-failed-')) / 'evidence'
            shutil.copytree(directory, evidence)
            print('Retained failed cold-drag fixture: '+str(evidence), file=sys.stderr)
            if native:
                print(native.diagnostics(), file=sys.stderr)
            raise
        finally:
            if native:
                native.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terminal', type=Path, default=ROOT / 'build/unified/standalone-dm_arm-t1-p1-m0/core/serial_arm_terminal')
    parser.add_argument('--cases', choices=('all', 'complete', 'cancel', 'fine', 'drag'), default='all')
    args = parser.parse_args()
    kinds = ('complete', 'cancel', 'fine', 'drag') if args.cases == 'all' else (args.cases,)
    print(json.dumps({'scope': 'real_native_write_disabled_mock_not_physical_acceptance',
                      'cases': [run_cold_drag_case(args.terminal.resolve()) if kind == 'drag'
                                else run_case(args.terminal.resolve(), kind) for kind in kinds]}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
