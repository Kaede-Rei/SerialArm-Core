#!/usr/bin/env python3
"""Exercise the native HOLD disturbance worker without a hardware backend.

The temporary Core disables writes and Native receives no hardware plugin.
This checks fixed HOLD execution, recording and cancellation, not physical
stiffness: the position-following Mock has no torque-to-motion dynamics.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

from check_adaptive_gain_flow import Native, ROOT, prepare_config, synthetic_record


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def run(terminal):
    checks = []
    with tempfile.TemporaryDirectory(prefix='serialarm-hold-disturbance-') as folder:
        directory = Path(folder)
        config, names = prepare_config(directory)
        storage = directory / 'storage'
        storage.mkdir()
        native = Native(terminal, config, storage)
        started = time.monotonic()
        try:
            native.request('activate')
            native.ready()
            checks.append('actual_native_mock_with_writes_disabled_and_no_hardware_plugin')
            synthetic_record(native, directory / 'source', names, 25)
            # A gentle path isolates HOLD execution from full-path tuning; T
            # remains the same original 25 s, with unchanged goals and Safety
            source_path = directory / 'source' / 'trajectory.csv'
            with source_path.open() as stream:
                path_rows = list(csv.reader(stream))
            for row in path_rows[2:]:
                row[1:] = [format(.1*float(value), '.17g') for value in row[1:]]
            with source_path.open('w') as stream:
                csv.writer(stream, lineterminator='\n').writerows(path_rows)
            native.ready()
            task = native.request('model_calibration_import_trajectory', {
                'directory': str(directory / 'source'), 'pose_budget': 5})
            native.ready()
            native.request('model_calibration_start', {
                'task_id': task['task_id'], 'workflow': 'gain', 'gain_mode': 'rigid',
                'gain_tuning_mode': 'fine', 'released': True,
                'gain_trajectory_confirmed': True, 'gain_motion_range_confirmed': True})
            task_path = Path(task['directory'])
            name = 'servo-probe-rigid_tracking-hold-disturbance.csv'
            target = task_path / name
            # A full original HOLD window is sufficient. Supplementation may
            # then continue, so cancel through the normal worker rather than
            # spending its remaining budget on a torque-insensitive Mock.
            deadline = time.monotonic() + 300
            previous_stage = None
            complete_rows = []
            while time.monotonic() < deadline:
                status = native.request('status')
                assert status['robot_state'] == 'ACTIVE', status
                calibration = native.request('model_calibration_status')
                stage = (calibration.get('gain_probe_kind'), calibration.get('gain_probe_message'))
                if stage != previous_stage:
                    print('Offline HOLD check: '+str(stage[0])+' '+str(stage[1]), file=sys.stderr)
                    previous_stage = stage
                assert calibration.get('error') != 'short response input rejected', calibration
                if target.exists() and target.stat().st_size > 1000:
                    rows = read_rows(target)
                    # The writer can be observed between opening and flushing.
                    if rows and all(None not in row and None not in row.values() for row in rows):
                        duration = (calibration.get('gain_probe_duration_s', 7)
                                    if calibration.get('gain_probe_kind', '').startswith('rigid_tracking-hold-disturbance') else 7)
                        if all(len(axis_rows := [row for row in rows if row['joint'] == joint]) >= 100
                               and max(float(row['time_s']) for row in axis_rows) >= duration-.08 for joint in names):
                            complete_rows = rows
                            break
                assert calibration['phase'] not in ('failed', 'cancelled', 'complete'), calibration
                time.sleep(.05)
            assert complete_rows, 'The real worker did not complete a HOLD disturbance window within 300 s'
            checks.append('hold_disturbance_produced_a_complete_nonempty_full_cycle_CSV')

            native.request('model_calibration_cancel', {'task_id': task['task_id']})
            stop_deadline = time.monotonic() + 10
            while time.monotonic() < stop_deadline:
                calibration = native.request('model_calibration_status')
                if calibration['phase'] in ('failed', 'cancelled', 'complete'):
                    # Terminal phase becomes visible before recorder.stop and
                    # the final candidate rewrite. result.json is written last
                    try:
                        saved = json.loads((task_path / 'result.json').read_text())
                    except (FileNotFoundError, json.JSONDecodeError):
                        saved = {}
                    if saved.get('phase') == calibration['phase'] and saved.get('termination') == calibration.get('termination'):
                        break
                time.sleep(.03)
            else:
                raise AssertionError('Worker did not finish recording and persist its terminal result')
            assert calibration['phase'] in ('cancelled', 'complete'), calibration
            assert calibration.get('error') != 'short response input rejected', calibration
            assert native.request('status')['robot_state'] == 'ACTIVE'
            candidate = json.loads((task_path / 'gain-candidate.json').read_text())
            assert candidate['termination'] == calibration['termination'], (candidate['termination'], calibration['termination'])
            assert candidate['termination']['kind'] != 'execution_fault', candidate['termination']
            assert not candidate['termination']['fault_interrupted'], candidate['termination']
            if calibration['phase'] == 'cancelled':
                assert candidate['termination']['kind'] == 'cancelled', candidate['termination']
                assert candidate['termination']['reason'] == 'stop_requested', candidate['termination']
            checks.append('normal_worker_cancellation_or_completion_preserves_active_hold_and_matching_termination')

            parameters = next(item for item in candidate['experiments']
                              if item.get('record_path') == name and item.get('kind') == 'short_experiment_parameters')
            assert parameters['hold_path'] and parameters['independent_disturbance'], parameters
            assert parameters['reference_execution'] == 'fixed_hold_target', parameters
            assert parameters['center_source'] == 'preserved_nominal_hold_target', parameters
            center = parameters['center_rad']
            assert len(center) == len(names), parameters
            eligible = parameters['disturbance_axis_eligible']
            assert len(eligible) == len(names) and any(eligible), parameters
            measured = {}
            for axis, joint in enumerate(names):
                rows = [row for row in complete_rows if row['joint'] == joint]
                assert len(rows) >= 100 and sum(int(row['new_measurement']) for row in rows) >= 100, joint
                assert max(float(row['time_s']) for row in rows) >= parameters['duration_s']+1.5-.08, joint
                assert all(row['actual_gains_known'] == '1' for row in rows), joint
                assert all(math.isfinite(float(value)) for row in rows for key, value in row.items() if key != 'joint'), joint
                assert max(abs(float(row['reference'])-center[axis]) for row in rows) < 1e-9, joint
                assert max(abs(float(row['reference_velocity'])) for row in rows) < 1e-9, joint
                input_peak = max(abs(float(row['independent_disturbance_nm'])) for row in rows)
                assert (input_peak > 0 if eligible[axis] else input_peak == 0), (joint, eligible[axis], input_peak)
                assert not any(int(row['limited']) for row in rows), joint
                measured[joint] = {'samples':len(rows), 'fresh_samples':sum(int(row['new_measurement']) for row in rows),
                    'reference_rad':center[axis],
                    'actual_span_rad':max(float(row['actual']) for row in rows)-min(float(row['actual']) for row in rows),
                    'independent_input_peak_nm':input_peak, 'disturbance_axis_eligible':eligible[axis]}
            checks.append('recorded_fixed_hold_target_zero_reference_velocity_and_independent_torque_only_for_eligible_axes')

            frame_count = 0
            input_peak = 0
            with (task_path / 'frames.csv').open() as stream:
                for row in csv.DictReader(stream):
                    if row['gain_stage'] != 'rigid_tracking-hold-disturbance':
                        continue
                    frame_count += 1
                    assert row['impedance_mode'] == 'RIGID', row['impedance_mode']
                    for axis, joint in enumerate(names):
                        assert abs(float(row['nominal_q:'+joint])-center[axis]) < 1e-9, joint
                        assert abs(float(row['accepted_q:'+joint])-center[axis]) < 1e-9, joint
                        assert abs(float(row['accepted_dq:'+joint])) < 1e-9, joint
                        input_peak = max(input_peak, abs(float(row['disturbance:'+joint])))
            assert frame_count >= 100 and input_peak > 0, (frame_count, input_peak)
            checks.append('full_cycle_frames_confirm_HOLD_nominal_and_accepted_target_with_applied_disturbance')
            experiment = next(item for item in candidate['experiments']
                              if item.get('record_path') == name and item.get('kind') == 'independent_torque_disturbance')
            models = [axis for axis in experiment['axes'] if 'disturbance_valid' in axis]
            assert len(models) == len(names) and not all(axis['disturbance_valid'] for axis in models), models
            assert not candidate['validated'], candidate['validated']
            rigid = candidate.get('targets', {}).get('rigid', candidate)
            assert rigid['native_validation']['available'] and rigid['hold_verification']['available'], rigid
            assert rigid['available'] and not candidate['available'] and not candidate['review_ready'], rigid
            assert rigid['qualification_complete'] is False and not rigid['validated'], rigid
            assert candidate['budget_source_duration_s'] == 25 and candidate['time_budget_s'] == 600, candidate
            checks.append('torque_insensitive_mock_does_not_pass_physical_disturbance_or_purpose_qualification')
            checks.append('completed_native_and_hold_parameters_remain_available_despite_missing_disturbance_qualification')
            return {'status':'passed', 'checks':checks, 'elapsed_wall_s':time.monotonic()-started,
                    'worker_phase':calibration['phase'], 'termination':candidate['termination'],
                    'recorded_T_s':25, 'hard_budget_s':600, 'full_cycle_hold_frames':frame_count,
                    'hold_disturbance_csv':measured,
                    'scope':'actual_native_mock_worker_no_hardware_or_physical_acceptance'}
        except Exception:
            print(native.diagnostics(), file=sys.stderr)
            raise
        finally:
            native.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terminal', type=Path,
                        default=ROOT / 'build/unified/standalone-dm_arm-t1-p1-m0/core/serial_arm_terminal')
    args = parser.parse_args()
    terminal = args.terminal.resolve()
    if not terminal.is_file() or not os.access(terminal, os.X_OK):
        parser.error('--terminal must be an executable file')
    print(json.dumps(run(terminal), indent=2))
