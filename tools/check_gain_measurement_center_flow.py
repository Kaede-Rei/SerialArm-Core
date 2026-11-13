#!/usr/bin/env python3
"""Check real native measurement-region entry using write-disabled Mock only.

The temporary URDF declares two taught endpoints as hard lower limits. Mock
does not simulate a mechanical stop or friction; the C++ plant regression
tests those dynamics. This checks actual IPC, path references, changed-center
short measurements and persisted cancellation/budget evidence.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import yaml

from check_adaptive_gain_flow import Native, ROOT, prepare_config, synthetic_record


def prepare(directory, terminal, recorded_t):
    directory.mkdir()
    config, names = prepare_config(directory)
    document = yaml.safe_load(config.read_text())
    model = ET.parse(document['model']['urdf_path'])
    for joint in model.getroot().findall('joint'):
        if joint.get('name') in names[1:3]:
            joint.find('limit').set('lower', '0')
    urdf = directory / 'declared-endpoint.urdf'
    model.write(urdf, encoding='utf-8', xml_declaration=True)
    document['model']['urdf_path'] = str(urdf)
    document['safety_policy']['position_margin'] = 0
    config.write_text(json.dumps(document))
    storage = directory / 'storage'
    storage.mkdir()
    native = Native(terminal, config, storage)
    try:
        native.request('activate')
        native.ready()
        synthetic_record(native, directory / 'source', names, recorded_t)
        with (directory / 'source' / 'trajectory.csv').open('w') as stream:
            stream.write('sample_dt,0.05\nindex,' + ','.join(names) + '\n')
            for index in range(501):
                u = index / 500
                q = [(.045 * (1 - math.cos(4 * math.pi * u)) if axis in (1, 2)
                      else .045 * math.sin(4 * math.pi * u))
                     + .004 * math.sin((axis + 2) * 2 * math.pi * u) for axis in range(len(names))]
                q[1], q[2] = abs(q[1]), abs(q[2])
                q[3] = .4 * q[0]
                stream.write(str(index) + ',' + ','.join(format(value, '.17g') for value in q) + '\n')
        native.ready()
        task = native.request('model_calibration_import_trajectory', {'directory': str(directory / 'source'), 'pose_budget': 5})
        native.ready()
        native.request('model_calibration_start', {'task_id': task['task_id'], 'workflow': 'gain', 'gain_mode': 'all',
                       'gain_tuning_mode': 'quick', 'released': True,
                       'gain_trajectory_confirmed': True, 'gain_motion_range_confirmed': True})
        return native, task, names
    except Exception:
        print(native.diagnostics(), file=sys.stderr)
        native.close()
        raise


def saved(native, task, deadline, cancel):
    if cancel:
        native.request('model_calibration_cancel', {'task_id': task['task_id']})
    folder = Path(task['directory'])
    while time.monotonic() < deadline:
        status = native.request('model_calibration_status')
        if status['phase'] in ('cancelled', 'complete', 'failed'):
            try:
                candidate = json.loads((folder / 'gain-candidate.json').read_text())
                result = json.loads((folder / 'result.json').read_text())
            except (FileNotFoundError, json.JSONDecodeError):
                time.sleep(.03)
                continue
            if result.get('termination') == candidate.get('termination') == status.get('termination'):
                assert native.request('status')['robot_state'] == 'ACTIVE', status
                assert not candidate['validated'], candidate
                assert status['termination']['kind'] != 'execution_fault', status
                return candidate, status
        time.sleep(.03)
    raise AssertionError('The real worker did not persist its concrete exit evidence')


def correlated_reference(folder, experiment, names):
    with (folder / experiment['record_path']).open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows and all(None not in row and None not in row.values() for row in rows), experiment
    joints = [[row for row in rows if row['joint'] == name] for name in names]
    assert min(map(len, joints)) >= 10 and len(set(map(len, joints))) == 1, experiment
    assert all(int(row['limited']) == 0 for row in rows), experiment
    for values in zip(*joints):
        q = [float(row['reference']) for row in values]
        assert q[1] >= 0 and q[2] >= 0, q
        assert abs(q[3] - .4 * q[0]) < 1e-7, q
    return joints


def check_resolution_record(experiment, joints, initial_noise, names):
    n = len(names)
    fields = ('actual_rad', 'reference_rad', 'reference_error_rad', 'observed_position_step_rad',
              'feedback_lead_rad', 'position_uncertainty_rad', 'position_reserve_source')
    assert all(len(experiment[field]) == n for field in fields), experiment
    assert experiment['unknown_resolution_guard_fraction'] == .25, experiment
    assert experiment['position_step_scope'] == 'minimum_fresh_raw_position_step_not_declared_codec_resolution', experiment
    learned = []
    for axis, rr in enumerate(joints):
        actual = experiment['actual_rad'][axis]
        reference = experiment['reference_rad'][axis]
        error = experiment['reference_error_rad'][axis]
        step = experiment['observed_position_step_rad'][axis]
        lead = experiment['feedback_lead_rad'][axis]
        reserve = experiment['position_uncertainty_rad'][axis]
        assert all(math.isfinite(value) for value in (actual, reference, error, step, lead, reserve)), experiment
        assert math.isclose(error, reference - actual, rel_tol=1e-12, abs_tol=1e-15), experiment
        assert abs(error) <= lead + 1e-12 and 0 < reserve < lead, experiment
        # Diagnostics describe the raw feedback consumed by the worker before
        # submitting its next reference. The final execution cycle can already
        # have newer feedback; compare the recorded prefix actually consumed.
        matching = [index for index, row in enumerate(rr) if float(row['actual']) == actual]
        assert matching, (names[axis], actual)
        accepted = []
        last_sequence = 0
        for row in rr[:matching[-1] + 1]:
            sequence = int(row['feedback_sequence'])
            if int(row['new_measurement']) and sequence > last_sequence:
                last_sequence = sequence
                accepted.append(float(row['actual']))
        positions = sorted(set(accepted))
        gaps = [b - a for a, b in zip(positions, positions[1:]) if b - a > 1e-9]
        stream_step = min(gaps) if gaps else 0
        earlier = initial_noise['axes'][axis]['effective_encoder_step_rad']
        current_candidates = [value for value in (stream_step, earlier) if value > 0]
        if current_candidates:
            # The runtime retains earlier observed floors. With continuous
            # Mock positions, inserting a sub-1e-9 neighbor can remove an old
            # batch-adjacent gap; that must not erase valid earlier evidence.
            assert 0 < step <= min(current_candidates) + 1e-15, (names[axis], step, current_candidates)
        if step > 0:
            supported = earlier > 0 and math.isclose(step, earlier, rel_tol=1e-8, abs_tol=1e-15)
            tolerance = max(1e-15, step * 1e-8)
            for value in positions:
                index = bisect.bisect_left(positions, value + step - tolerance)
                if index < len(positions) and abs(positions[index] - value - step) <= tolerance:
                    supported = True
                    break
            assert supported, (names[axis], 'Recorded step has no actual raw-position evidence', step)
        else:
            assert not current_candidates, (names[axis], 'Observed resolution was lost', current_candidates)
        rms = initial_noise['axes'][axis]['noise_rms_rad']
        if step > 0:
            expected_reserve = max(3 * step, 6 * rms, 1e-6)
            expected_source = 'observed_position_step_and_stationary_noise_reserve'
            if earlier == 0:
                learned.append(axis)
        else:
            expected_reserve = max(.25 * lead, 6 * rms)
            expected_source = 'unidentified_position_step_internal_guard_buffer'
        assert math.isclose(reserve, expected_reserve, rel_tol=1e-8, abs_tol=1e-15), experiment
        assert experiment['position_reserve_source'][axis] == expected_source, experiment
    assert learned, 'Actual entry feedback did not establish any previously unknown position step'
    return learned


def run_case(terminal, directory, deadline, kind):
    native = None
    try:
        native, task, names = prepare(directory, terminal, 1 if kind == 'budget' else 25)
        if kind == 'budget':
            candidate, status = saved(native, task, deadline, False)
            assert 'budget' in status['termination']['kind'] + ':' + status['termination']['reason'], status
            assert candidate['budget_source_duration_s'] == 1 and candidate['time_budget_s'] == 15, candidate
            assert status['gain_elapsed_s'] <= 15, status
            assert candidate['available'] and not candidate['validated'] and not candidate['qualification_complete'], status
            assert candidate['review_required'] is True, status
            concrete = [warning for warning in candidate.get('warnings', [])
                        if 'measurement_center_entry_insufficient_reserved_budget' in warning]
            assert concrete, candidate.get('warnings')
            # The expensive positioning/identification does not fit this
            # trusted T=1 budget. It is an advisory skip; the smaller complete
            # native trajectory still fits and must remain an actual measured
            # candidate rather than being discarded with the skipped stage.
            stages = candidate['targets']
            native_trial = stages['servo_tracking']['native_validation']
            assert native_trial['completed'] and native_trial['available'] and native_trial['record_path'], native_trial
            assert stages['servo_tracking']['available'] and not stages['servo_tracking']['validated'], stages['servo_tracking']['warnings']
            assert not stages['servo_tracking']['disturbance_verification']['available'], stages['servo_tracking']['warnings']
            assert stages['servo_tracking']['kp'] == stages['rigid_tracking']['kp'] and stages['servo_tracking']['kd'] == stages['rigid_tracking']['kd']
            assert stages['rigid_tracking']['native_validation']['record_path'] == native_trial['record_path']
            with (Path(task['directory']) / native_trial['record_path']).open() as stream:
                rows = list(csv.DictReader(stream))
            counts = {}
            for joint in names:
                values = [row for row in rows if row['joint'] == joint]
                assert len(values) >= 100 and all(int(row['limited']) == 0 for row in values), joint
                assert max(float(row['time_s']) for row in values) >= native_trial['duration_s'], joint
                counts[joint] = len(values)
            assert not any(item.get('kind') == 'demonstrated_measurement_center_entry' and item.get('completed')
                           for item in candidate['experiments']), 'The skipped entry must not acquire fabricated completed evidence'
            return {'case': kind, 'termination': status['termination'], 'elapsed_s': status['gain_elapsed_s'],
                    'recorded_T_s': 1, 'hard_limit_s': 15, 'validated': False,
                    'available': candidate['available'], 'review_required': candidate['review_required'],
                    'retained_complete_trajectory': native_trial['record_path'], 'samples_by_joint': counts,
                    'concrete_budget_skip_evidence': concrete}
        saw_entry = False
        while time.monotonic() < deadline:
            status = native.request('model_calibration_status')
            assert native.request('status')['robot_state'] == 'ACTIVE', status
            probe = status.get('gain_probe_kind', '')
            if probe.endswith('-measurement-entry'):
                saw_entry = True
                if kind == 'stop' and status.get('gain_probe_elapsed_s', 0) > .2:
                    break
            if kind == 'measurement' and saw_entry and probe.startswith('native-establish-') and status.get('gain_probe_elapsed_s', 0) > .3:
                break
            assert status['phase'] not in ('cancelled', 'complete', 'failed'), status
            time.sleep(.03)
        else:
            raise AssertionError('The real worker never entered the expected demonstrated-path stage')
        candidate, status = saved(native, task, min(deadline, time.monotonic() + 10), True)
        entries = [item for item in candidate['experiments'] if item.get('kind') == 'demonstrated_measurement_center_entry'
                   and item.get('direction') == 'toward_measurement_region']
        assert saw_entry and entries, candidate.get('experiments')
        experiment = entries[-1]
        assert experiment['counted_in_budget'] and experiment['validation_scope'] == 'positioning_only_not_identification_or_purpose_qualification', experiment
        assert experiment['source_indices'][0] is None, experiment
        source_indices = [value for value in experiment['source_indices'] if value is not None]
        assert all(b <= a for a, b in zip(source_indices, source_indices[1:])), source_indices
        folder = Path(task['directory'])
        joints = correlated_reference(folder, experiment, names)
        noise_records = [item for item in candidate['experiments']
                         if item.get('stage') == 'trajectory_noise' and item.get('completed') and 'axes' in item]
        assert noise_records and len(noise_records[0]['axes']) == len(names), candidate['experiments']
        learned = check_resolution_record(experiment, joints, noise_records[0], names)
        for axis, rr in enumerate(joints):
            assert abs(float(rr[0]['reference']) - experiment['fresh_origin_rad'][axis]) < 1e-8, (axis, rr[0], experiment)
        assert status['termination']['kind'] == 'cancelled' and status['termination']['reason'] == 'stop_requested', status
        if kind == 'stop':
            assert not experiment['completed'], experiment
            assert 'interrupted' in experiment['reason'], experiment
            return {'case': kind, 'completed': False, 'recorded_rows': len(joints[0]),
                    'learned_position_step_axes': learned, 'observed_position_step_rad': experiment['observed_position_step_rad'],
                    'position_reserve_source': experiment['position_reserve_source'], 'termination': status['termination']}
        assert experiment['completed'] and experiment['selected_center_rad'][2] > .005, experiment
        for axis, rr in enumerate(joints):
            assert abs(float(rr[-1]['reference']) - experiment['selected_center_rad'][axis]) < 1e-8, (axis, rr[-1], experiment)
            assert float(rr[-1]['reference_velocity']) == 0, (axis, rr[-1])
        with (folder / 'trajectory.csv').open() as stream:
            original = csv.reader(stream)
            next(original)
            next(original)
            demonstrated = [[float(value) for value in row[1:]] for row in original]
        selected = demonstrated[experiment['target_source_index']]
        assert all(abs(a - b) < 1e-10 for a, b in zip(experiment['selected_center_rad'], selected)), experiment
        short = next(item for item in candidate['experiments'] if item.get('kind') == 'short_experiment_parameters'
                     and item.get('label', '').startswith('native-establish-'))
        assert len(noise_records) >= 2, noise_records
        at_center = noise_records[-1]
        retained = []
        for axis in range(len(names)):
            measured = at_center['axes'][axis]['encoder_step_rad']
            effective = at_center['axes'][axis]['effective_encoder_step_rad']
            previous = experiment['observed_position_step_rad'][axis]
            assert at_center['axes'][axis]['encoder_step_observed'] == (effective > 0), at_center
            expected = min(previous, measured) if previous > 0 and measured > 0 else max(previous, measured)
            assert math.isclose(effective, expected, rel_tol=1e-8, abs_tol=1e-15), (axis, previous, measured, effective)
            if previous > 0 and measured == 0:
                assert effective == previous, (axis, previous, effective)
                retained.append(axis)
            assert math.isclose(short['plans'][axis]['encoder_step_rad'], effective, rel_tol=1e-8, abs_tol=1e-15), short
            assert math.isclose(short['plans'][axis]['noise_rms_rad'], at_center['axes'][axis]['noise_rms_rad'], rel_tol=1e-8, abs_tol=1e-15), short
        assert retained, 'Fixture did not exercise preserving known resolution through a constant zero-code noise window'
        assert short['center_rad'][2] > .005 and abs(short['center_rad'][2] - experiment['fresh_origin_rad'][2]) > .005, short
        assert all(abs(short['center_rad'][i] - float(joints[i][-1]['actual'])) < .003 for i in range(len(names))), short
        with (folder / short['record_path']).open() as stream:
            short_rows = list(csv.DictReader(stream))
        assert len(short_rows) >= 20 * len(names), short
        return {'case': kind, 'completed': True, 'recorded_entry_rows': len(joints[0]),
                'fresh_origin_rad': experiment['fresh_origin_rad'], 'selected_center_rad': experiment['selected_center_rad'],
                'actual_short_center_rad': short['center_rad'], 'short_recorded_rows': len(short_rows),
                'learned_position_step_axes': learned, 'observed_position_step_rad': experiment['observed_position_step_rad'],
                'position_reserve_source': experiment['position_reserve_source'], 'quiet_window_retained_step_axes': retained,
                'source_indices': experiment['source_indices'], 'termination': status['termination']}
    except Exception:
        if native:
            print(native.diagnostics(), file=sys.stderr)
        raise
    finally:
        if native:
            native.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terminal', type=Path, default=ROOT / 'build/unified/standalone-dm_arm-t1-p1-m0/core/serial_arm_terminal')
    args = parser.parse_args()
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix='serialarm-measurement-center-flow-') as folder:
        root = Path(folder)
        cases = [run_case(args.terminal.resolve(), root / kind, started + 90, kind)
                 for kind in ('measurement', 'stop', 'budget')]
    print(json.dumps({'offline_mock_only': True, 'elapsed_s': time.monotonic() - started, 'cases': cases,
                     'scope': 'native IPC, actual emitted references and persisted evidence; physical stop/friction covered separately by C++ plant'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
