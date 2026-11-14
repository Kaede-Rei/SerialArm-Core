#!/usr/bin/env python3
"""Check advisory gain-stage failures through the real write-disabled Mock IPC.

A tiny demonstrated range on one axis cannot support the required local
identification/release. Full trajectories and hold windows are still executable
on this checked path. The worker must measure those remaining contexts and
retain the honest missing-evidence warning for final user review.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import time

from check_adaptive_gain_flow import Native, ROOT, prepare_config, synthetic_record


def reference_kinematics(folder, trial, joint, values, axis):
    """Check actual captured references against their source sampling timeline."""
    with (folder / 'trajectory.csv').open() as stream:
        reader = csv.reader(stream)
        source_dt = float(next(reader)[1])
        next(reader)
        positions = [float(row[axis + 1]) for row in reader]
    if trial['reverse']:
        positions.reverse()
    rate = trial['playback_rate']
    tangents = [0.0]
    for left, center, right in zip(positions, positions[1:], positions[2:]):
        a, b = center-left, right-center
        tangents.append(2*a*b/(a+b) if a*b > 0 else 0.0)
    tangents.append(0.0)
    acceleration_bound = 0.0
    for p0, p1, m0, m1 in zip(positions, positions[1:], tangents, tangents[1:]):
        displacement = p1-p0
        acceleration_bound = max(acceleration_bound,
            abs(6*displacement-4*m0-2*m1), abs(-6*displacement+2*m0+4*m1))
    acceleration_bound *= (rate/source_dt)**2
    def source_at(sample_time):
        progress = min(len(positions)-1, max(0.0, sample_time*rate/source_dt))
        lower = min(int(progress), len(positions)-2)
        u = progress-lower
        p0, p1, m0, m1 = positions[lower], positions[lower+1], tangents[lower], tangents[lower+1]
        q = (2*u**3-3*u*u+1)*p0+(u**3-2*u*u+u)*m0+(-2*u**3+3*u*u)*p1+(u**3-u*u)*m1
        velocity = ((6*u*u-6*u)*p0+(3*u*u-4*u+1)*m0+(-6*u*u+6*u)*p1+(3*u*u-2*u)*m1)*rate/source_dt
        return q, velocity if sample_time < trial['duration_s'] else 0.0
    previous = None
    worst_mismatch = worst_bound = 0.0
    for row in values:
        assert row['reference_sample_time_known'] == '1', (joint, row)
        sample_time = float(row['reference_sample_time_s'])
        assert math.isfinite(sample_time) and sample_time <= float(row['command_time_s'])+1e-9, (joint, row)
        q, velocity = float(row['reference']), float(row['reference_velocity'])
        expected_q, expected_velocity = source_at(sample_time)
        assert math.isclose(q, expected_q, abs_tol=1e-10), (joint, sample_time, q, expected_q)
        assert math.isclose(velocity, expected_velocity, abs_tol=1e-9), (joint, sample_time, velocity, expected_velocity)
        if previous is not None:
            dt = sample_time-previous[0]
            assert dt > 0, (joint, previous, sample_time)
            delta = q-previous[1]
            integral = .5*(velocity+previous[2])*dt
            # With |dv/dt| <= A, trapezoidal velocity integration differs
            # from displacement by at most A*dt^2/4, including knot changes.
            # This bound comes from this saved source, not the 0.1° target.
            bound = .25*acceleration_bound*dt*dt+1e-10
            mismatch = abs(delta-integral)
            assert mismatch <= bound, (joint, sample_time, dt, delta, integral, mismatch, bound)
            worst_mismatch = max(worst_mismatch, mismatch)
            worst_bound = max(worst_bound, bound)
        previous = sample_time, q, velocity
    return {'samples': len(values), 'source_acceleration_bound_rad_s2': acceleration_bound,
            'maximum_position_velocity_integral_mismatch_rad': worst_mismatch,
            'maximum_source_acceleration_tolerance_rad': worst_bound,
            'source_time_and_position_velocity_consistent': True}


def measured_csv(folder, trial, names):
    assert trial['completed'] and trial['available'], trial
    assert trial['record_path'], trial
    with (folder / trial['record_path']).open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows and all(None not in row and None not in row.values() for row in rows), trial
    samples, kinematics = {}, {}
    for axis, joint in enumerate(names):
        values = [row for row in rows if row['joint'] == joint]
        assert len(values) >= 100, (joint, trial)
        assert max(float(row['time_s']) for row in values) >= trial['duration_s'], (joint, trial)
        assert all(int(row['limited']) == 0 for row in values), (joint, trial)
        assert all(math.isfinite(float(row[key])) for row in values
                   for key in ('reference', 'reference_velocity', 'actual', 'velocity')), joint
        samples[joint] = len(values)
        kinematics[joint] = reference_kinematics(folder, trial, joint, values, axis)
    return {'record_path': trial['record_path'], 'samples_by_joint': samples,
            'validated': trial['validated'], 'real_cycle_reference_kinematics': kinematics}


def run(terminal):
    native = None
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix='serialarm-gain-stage-continuation-') as temporary:
        directory = Path(temporary)
        config, names = prepare_config(directory)
        config_before = config.read_text()
        storage = directory / 'storage'
        storage.mkdir()
        try:
            native = Native(terminal, config, storage)
            native.request('activate')
            native.ready()
            synthetic_record(native, directory / 'source', names, 25)
            with (directory / 'source' / 'trajectory.csv').open('w') as stream:
                stream.write('sample_dt,0.02\nindex,' + ','.join(names) + '\n')
                for index in range(101):
                    u = index / 100
                    # This is a range insufficiency, not a declared physical
                    # hard stop, a Safety exception or a joint-specific rule.
                    q = [(.00002 * math.sin(2 * math.pi * u) if axis == 4
                          else .015 * math.sin(2 * math.pi * u) + .004 * math.sin((axis + 2) * 2 * math.pi * u))
                         for axis in range(len(names))]
                    stream.write(str(index) + ',' + ','.join(format(value, '.17g') for value in q) + '\n')
            native.ready()
            task = native.request('model_calibration_import_trajectory',
                                  {'directory': str(directory / 'source'), 'pose_budget': 5})
            native.ready()
            native.request('model_calibration_start', {
                'task_id': task['task_id'], 'workflow': 'gain', 'gain_mode': 'all',
                'gain_tuning_mode': 'quick', 'released': True,
                'gain_trajectory_confirmed': True, 'gain_motion_range_confirmed': True})
            folder = Path(task['directory'])
            observed = []
            deadline = started + 120
            while time.monotonic() < deadline:
                status = native.request('model_calibration_status')
                assert native.request('status')['robot_state'] == 'ACTIVE', status
                probe = status.get('gain_probe_kind', '')
                if probe and probe not in observed:
                    observed.append(probe)
                if status['phase'] in ('complete', 'failed', 'cancelled'):
                    try:
                        candidate = json.loads((folder / 'gain-candidate.json').read_text())
                        result = json.loads((folder / 'result.json').read_text())
                    except (FileNotFoundError, json.JSONDecodeError):
                        time.sleep(.03)
                        continue
                    if result.get('termination') == candidate.get('termination') == status.get('termination'):
                        break
                time.sleep(.03)
            else:
                raise AssertionError('The advisory-failure fixture did not reach its final review within 120 seconds')
            assert status['phase'] == result['phase'] == 'failed', status
            assert status['termination']['kind'] not in ('execution_fault', 'cancelled', 'budget_exhausted'), status
            assert not candidate['available'] and not candidate['validated'] and not candidate['qualification_complete'], {
                'termination': candidate['termination'], 'available': candidate['available'],
                'targets': {key: {'available': stage['available'], 'warnings': stage['warnings']}
                            for key, stage in candidate['targets'].items()}}
            assert candidate['warnings'], candidate
            assert candidate['review_ready'] is False and candidate['measurement_complete'] is False and candidate['review_required'] is False, candidate
            assert status['gain_available'] is False and status['progress'] < 1, status
            assert candidate['nonfatal_stage_warnings'] > 0, candidate['termination']
            assert candidate['execution_policy'] == 'continue_available_measurements_until_stop_fault_or_original_budget'
            warning_records = [item for item in candidate['experiments'] if item.get('kind') == 'measurement_warning']
            assert warning_records and len(warning_records) == candidate['nonfatal_stage_warnings'], warning_records
            assert any(item['phase'] == 'native-identification' and 'plan unavailable' in item['reason']
                       for item in warning_records), warning_records
            assert all(item['continued'] and not item['completed'] and not item['validated']
                       and len(item['kp']) == len(item['kd']) == len(names) for item in warning_records), warning_records
            assert any('plan unavailable' in warning or 'short response identification insufficient' in warning
                       for warning in candidate['warnings']), candidate['warnings']
            targets = candidate['targets']
            assert set(targets) == {'rigid', 'compliant', 'drag'}, targets
            measured = {}
            for purpose in ('rigid', 'compliant'):
                stage = targets[purpose]
                measured[purpose] = measured_csv(folder, stage['native_validation'], names)
                assert not stage['validated'] and not stage['qualification_complete'], stage
                assert not stage['disturbance_verification']['available'], stage
                assert stage['warnings'], stage
                if purpose in ('rigid', 'compliant'):
                    hold = stage['hold_verification']
                    assert hold['completed'] and hold['available'] and hold['samples'] >= 30, hold
                    assert hold['record_path'] and (folder / hold['record_path']).is_file(), hold
            assert measured['compliant']['record_path'] != measured['rigid']['record_path'], measured
            drag = targets['drag']
            assert not drag['validated'] and not drag['drag_validation']['available'], drag
            assert any('insufficient safe joint range' in warning or 'range_insufficient' in warning
                       for warning in drag['warnings']), drag
            assert all('compliant_drag:' not in warning and not warning.startswith('drag-')
                       for warning in targets['compliant']['warnings']), targets['compliant']['warnings']
            assert all('not_started' not in warning and warning != 'measurement_in_progress'
                       for stage in targets.values() for warning in stage.get('warnings', [])), targets
            assert config.read_text() == config_before, 'The automatic worker modified its input configuration'
            return {'offline_mock_only': True, 'elapsed_s': time.monotonic() - started,
                    'first_identification_incomplete': True, 'later_complete_trajectory_measurements': measured,
                    'later_hold_purposes': ['rigid', 'compliant'],
                    'drag_attempted_with_honest_range_warning': True,
                    'purpose_warnings_isolated': True,
                    'observed_stages': observed, 'phase': status['phase'],
                    'termination': status['termination'], 'available': candidate['available'],
                    'validated': candidate['validated'], 'qualification_complete': candidate['qualification_complete'],
                    'review_required': candidate['review_required'],
                    'nonfatal_stage_warnings': candidate['nonfatal_stage_warnings'],
                    'warnings': candidate['warnings']}
        except Exception:
            if native:
                print(native.diagnostics(), file=sys.stderr)
            raise
        finally:
            if native:
                native.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terminal', type=Path,
                        default=ROOT / 'build/unified/standalone-dm_arm-t1-p1-m0/core/serial_arm_terminal')
    args = parser.parse_args()
    print(json.dumps(run(args.terminal.resolve()), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
