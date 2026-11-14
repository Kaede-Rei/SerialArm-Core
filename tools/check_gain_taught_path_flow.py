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
import shlex
import shutil
from pathlib import Path
import sys
import subprocess
import tempfile
import time
import zipfile

import numpy as np

from check_adaptive_gain_flow import Native, ROOT, prepare_config, synthetic_record


def displaced_zero_stiffness_terminal(terminal, directory):
    """Build the real worker with only its memory bus changed, inside /tmp.

    The normal Mock copies its position command even when Kp is zero. It
    cannot reproduce the changed resting position that exposed the real
    HOLD-to-tracking resume failure. This fixture moves every axis by the
    same 4 mrad on entry to zero stiffness, then keeps that measured position
    until positive stiffness returns. A requested single read step then makes
    the purpose callback return the existing measured-HOLD
    state explicitly, so a perfect Mock source-knot match cannot skip the
    branch under test. It deliberately models no PD response.
    Production sources, runtime configurations and installed binaries remain
    untouched; the actual Terminal/Robot/ControlRuntime code is compiled as is.
    """
    build = terminal.parent
    cmake = build / 'CMakeFiles/serial_arm_terminal.dir'
    flags = (cmake / 'flags.make').read_text()
    def values(name):
        line = next(line for line in flags.splitlines() if line.startswith(name+' ='))
        return shlex.split(line.split('=', 1)[1])
    source = ROOT / 'src/serial_arm/core/app/serial_arm_terminal.cpp'
    original = source.read_text()
    start = original.index('class MockMotorBus final')
    end = original.index('class TerminalApp', start)
    memory = original[start:end]
    old = '''        state_.pos = cmd.pos;
        state_.vel = cmd.vel;
        state_.tor = cmd.tor;'''
    new = '''        const bool zero_stiffness = !cmd.kp.empty() &&
            std::all_of(cmd.kp.begin(),cmd.kp.end(),[](double value){return std::abs(value)<1e-12;});
        if(zero_stiffness) {
            if(!zero_stiffness_active_) {
                state_.pos=cmd.pos;
                for(auto& value:state_.pos)value+=.004;
            }
            state_.vel.assign(cmd.vel.size(),0);
        } else {
            state_.pos=cmd.pos;
            state_.vel=cmd.vel;
        }
        zero_stiffness_active_=zero_stiffness;
        state_.tor=cmd.tor;'''
    assert memory.count(old) == 1, 'The in-memory fixture no longer matches the production Mock write method'
    memory = memory.replace(old, new).replace('    bool active_{ false };',
        '    bool active_{ false };\n    bool zero_stiffness_active_{false};')
    memory=memory.replace('public:\n','public:\n    inline static bool fixture_step_requested_{false};\n',1)
    read_marker='''        if(!connected_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
        const auto stamp='''
    assert memory.count(read_marker)==1
    memory=memory.replace(read_marker,'''        if(!connected_) return tl::make_unexpected(MotorBusErr::NOT_CONNECTED);
        if(fixture_step_requested_) {for(auto& value:state_.pos)value+=.004;fixture_step_requested_=false;}
        const auto stamp=''')
    assert memory.count('bool zero_stiffness_active_') == 1
    copied = directory / 'serial_arm_terminal_displaced_mock.cpp'
    derived = original[:start]+memory+original[end:]
    callback = '''                    measure(original_index,remaining_path);
                    JointVector accepted;'''
    injected = '''                    measure(original_index,remaining_path);
                    // Focused state-machine fixture: use the real P0 and
                    // demonstrated-release paths, omitting unrelated tuning polls.
                    (void)measure;
                    JointImpedanceGains damping=gains;std::fill(damping.kp.begin(),damping.kp.end(),0);
                    {
                        std::unique_lock<std::mutex> guard(mutex_);
                        if(!runtime_.set_impedance_mode(robot_,JointImpedanceMode::DRAG) ||
                           !robot_.set_execution_gain_profile(JointGainProfile::DRAG) ||
                           !robot_.set_calibration_tracking_gains(damping,JointImpedanceMode::DRAG))
                            throw std::runtime_error("fixture_P0_setup_failed");
                        const auto until=Robot::Clock::now()+std::chrono::seconds(2);
                        while(Robot::Clock::now()<until) {
                            const auto cursor=cycle_counter_;
                            cycle_cv_.wait_for(guard,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor;});
                            if(last_output_ && std::all_of(last_output_->joint_cmd.kp.begin(),last_output_->joint_cmd.kp.end(),[](double p){return std::abs(p)<1e-10;}))break;
                        }
                    }
                    DragTrial release_fixture(n);
                    gain_drag_taught_release(release_fixture,damping,gains,reserved_s+remaining_path,label+"-resume-fixture");
                    {
                        std::unique_lock<std::mutex> guard(mutex_);
                        MockMotorBus::fixture_step_requested_=true;
                        const auto fresh_cursor=cycle_counter_;
                        if(!cycle_cv_.wait_for(guard,std::chrono::milliseconds(60),[&]{return cycle_counter_>fresh_cursor;}))
                            throw std::runtime_error("fixture_fresh_feedback_cycle_missing");
                        gain_hold_for_assessment_locked(gains,true);
                        const auto cursor=cycle_counter_;
                        if(!cycle_cv_.wait_for(guard,std::chrono::milliseconds(60),[&]{return cycle_counter_>cursor;}))
                            throw std::runtime_error("fixture_measured_HOLD_cycle_missing");
                    }
                    JointVector accepted;'''
    assert derived.count(callback) == 1, 'The focused callback boundary changed'
    derived = derived.replace(callback, injected)
    local_poll = '''                            run_measurement("compliant-local-correction",stage.gains,[&]{gain_refine_taught_segment(stage.gains,JointGainProfile::COMPLIANT,stage.native,verification_reserve(),"compliant",anchor);});'''
    assert derived.count(local_poll)==1
    derived=derived.replace(local_poll,'                            (void)anchor; // This focused callback measures its real hold only')
    soft_work='''                            soft_purpose_work();pending_anchor_return_s=0;'''
    assert derived.count(soft_work)==1
    derived=derived.replace(soft_work,'''                            tune_hold(stage,true);pending_anchor_return_s=0;''')
    copied.write_text(derived)
    output = directory / 'serial_arm_terminal'
    obj = directory / 'serial_arm_terminal.o'
    log = directory / 'compile.log'
    with log.open('w') as stream:
        fixture_flags = [flag for flag in values('CXX_FLAGS') if not flag.startswith('-O')]+['-O0']
        subprocess.run(['/usr/bin/c++', *values('CXX_DEFINES'), *values('CXX_INCLUDES'),
                        *fixture_flags, '-c', str(copied), '-o', str(obj)],
                       cwd=build, stdout=stream, stderr=subprocess.STDOUT, check=True)
        link = shlex.split((cmake / 'link.txt').read_text())
        link[link.index('CMakeFiles/serial_arm_terminal.dir/app/serial_arm_terminal.cpp.o')] = str(obj)
        link[link.index('-o')+1] = str(output)
        subprocess.run(link, cwd=build, stdout=stream, stderr=subprocess.STDOUT, check=True)
    assert source.read_text() == original, 'The native source changed while compiling the focused fixture'
    return output


def read_rows(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows and all(None not in row and None not in row.values() for row in rows), path
    return rows


def cycle_groups(rows, names):
    """Align actual per-axis records by the original control-cycle clock."""
    grouped = [[row for row in rows if row['joint'] == name] for name in names]
    assert min(map(len, grouped)) >= 30 and len(set(map(len, grouped))) == 1
    for sample in zip(*grouped):
        clocks = [float(row['command_time_s']) for row in sample]
        assert max(clocks)-min(clocks) < 1e-8, 'Joint records no longer describe the same command cycle'
        assert all(row['actual_gains_known'] == '1' for row in sample)
        assert all(math.isfinite(float(row[key])) for row in sample for key in (
            'time_s', 'actual', 'velocity', 'actual_kp', 'actual_kd', 'independent_disturbance_nm', 'feedback_time_s'))
    return grouped


def assert_isolated_pulses(rows, names, plans, kp, kd):
    grouped = cycle_groups(rows, names)
    assert plans and all(plan['scheduled'] for plan in plans), plans
    scheduled_axes = {plan['axis_index'] for plan in plans}
    assert scheduled_axes == set(range(len(names))), 'The roomy Mock fixture did not measure every axis'
    previous_end = -math.inf
    for plan in plans:
        axis = plan['axis_index']
        assert plan['joint'] == names[axis] and plan['direction'] in (-1, 1), plan
        assert previous_end <= plan['start_s'] and plan['start_s'] < plan['release_s'] < plan['end_s'], plan
        previous_end = plan['end_s']
    active_counts, active_axes = [], set()
    for sample in zip(*grouped):
        active = [axis for axis, row in enumerate(sample) if abs(float(row['independent_disturbance_nm'])) > 1e-10]
        assert len(active) <= 1, 'Purpose input simultaneously excited several mechanically coupled axes'
        active_counts.append(len(active));active_axes.update(active)
        for axis in active:
            row = sample[axis]
            windows = [plan for plan in plans if plan['axis_index'] == axis
                       and plan['start_s']-.02 <= float(row['time_s']) <= plan['release_s']+.02]
            assert len(windows) == 1, (names[axis], row, windows)
            plan = windows[0]
            actual_input = float(row['independent_disturbance_nm'])
            assert abs(actual_input) <= abs(plan['amplitude_nm'])+1e-6, (names[axis], row, plan)
            assert plan['direction']*actual_input >= -1e-10, (names[axis], row, plan)
            # This is the actual emitted whole group, not a concatenation of
            # preferred axis values from separate controller experiments.
            np.testing.assert_allclose([float(value['actual_kp']) for value in sample], kp, rtol=0, atol=1e-6)
            np.testing.assert_allclose([float(value['actual_kd']) for value in sample], kd, rtol=0, atol=1e-6)
    assert active_axes == scheduled_axes, (active_axes, scheduled_axes)
    for plan in plans:
        axis = plan['axis_index']
        values = grouped[axis]
        loaded = [row for row in values if plan['start_s'] <= float(row['time_s']) < plan['release_s']]
        released = [row for row in values if plan['release_s']+.02 <= float(row['time_s']) <= plan['end_s']]
        assert len(loaded) >= 20 and len(released) >= 20, (names[axis], plan)
        assert max(abs(float(row['independent_disturbance_nm'])) for row in loaded) > 1e-10
        assert all(abs(float(row['independent_disturbance_nm'])) < 1e-10 for row in released)
        assert sum(row['new_measurement'] == '1' for row in loaded+released) >= 30
    return {'scheduled_axes': [names[axis] for axis in sorted(scheduled_axes)],
            'executed_windows': len(plans), 'maximum_simultaneous_input_axes': max(active_counts)}


def assert_drag_zero_stiffness(folder, selected, names, candidate=None):
    validation = selected['drag_validation']
    assert validation['completed'] and validation['available'], validation
    assert all(value == 0 for value in selected['gains']['kp'])
    parameters = next(item for item in (candidate or selected)['experiments']
                      if item.get('kind') == 'drag_excitation_parameters'
                      and Path(item['record_path']).name == Path(validation['record_path']).name)
    assert parameters['actual_gains_ready_s'] <= parameters['quiet_start_s'] < parameters['quiet_end_s'], parameters
    assert parameters['quiet_end_s']-parameters['quiet_start_s'] >= .699, parameters
    rows = read_rows(folder / validation['record_path'])
    groups = cycle_groups(rows, names)
    assert all('phase' in row and 'released' in row for row in rows), 'Drag evidence omitted actual zero-stiffness phase timing'
    quiet_end = -math.inf
    first_input = math.inf
    for axis, values in enumerate(groups):
        quiet = [row for row in values if row['phase'] == 'quiet'
                 and parameters['quiet_start_s'] <= float(row['time_s']) <= parameters['quiet_end_s']]
        assert len(quiet) >= 30 and float(quiet[-1]['time_s'])-float(quiet[0]['time_s']) >= .58, (names[axis], 'No complete zero-input quiet window')
        assert all(abs(float(row['independent_disturbance_nm'])) < 1e-10 for row in quiet)
        quiet_end = max(quiet_end, float(quiet[-1]['time_s']))
        actual_zero = [row for row in values if row['phase'] in ('quiet', 'input', 'release')]
        assert actual_zero, (names[axis], 'No actual damping-only measurement')
        assert all(abs(float(row['actual_kp'])) < 1e-10 and math.isclose(float(row['actual_kd']), selected['gains']['kd'][axis], abs_tol=1e-6)
                   for row in actual_zero), 'Drag scope started while a positive-P handover was still executing'
        for row in values:
            if abs(float(row['independent_disturbance_nm'])) > 1e-10:
                first_input = min(first_input, float(row['time_s']))
                assert row['phase'] == 'input', row
            if row['phase'] == 'release':
                assert abs(float(row['independent_disturbance_nm'])) < 1e-10, row
                assert (row['released'] == '1') == (row['active_axis'] == names[axis]), row
    assert first_input > quiet_end, 'Optional release preparation displaced the basic quiet result'
    for sample in zip(*groups):
        assert sum(abs(float(row['independent_disturbance_nm'])) > 1e-10 for row in sample) <= 1, 'Drag preparation correlated several axes'
    axes = validation['axes']
    assert len(axes) == len(names) and all(axis['quiet_stability_measured'] for axis in axes)
    events = (candidate or selected)['experiments']
    plans = [item for item in events if item.get('kind') == 'drag_input_plan'
             and Path(item['record_path']).name == Path(validation['record_path']).name]
    assert {item['joint'] for item in plans} == set(names), 'Drag input preparation omitted a joint and its missing-evidence reason'
    for plan in plans:
        assert plan['reason'] and plan['input_ceiling_nm'] >= 0 and plan['torque_quantum_nm'] >= 0, plan
        if plan['scheduled']:
            assert 0 < plan['input_nm'] <= plan['input_ceiling_nm']+1e-10 and plan['duration_s'] > 0, plan
        if plan['prior_valid']:
            assert plan['prior_source'] and plan['prior_input_nm'] > 0, plan
    supplements = [item for item in events if item.get('kind') == 'drag_input_supplement_decision'
                   and Path(item['record_path']).name == Path(validation['record_path']).name]
    assert all(item['reason'] and item['next_input_nm'] >= 0 for item in supplements), supplements
    # Independent torque does not move this Mock. A later explicitly recorded
    # taught-motion release can establish motion before P0, whereas an input
    # pulse alone cannot manufacture that missing response.
    assert all(not axis['release_decay_measured'] or axis.get('demonstrated_motion_preparation')
               for axis in axes), 'The position-following Mock manufactured a pulse-driven zero-P release response'
    assert selected['available'], 'Unmeasured optional release erased measured quiet damping'
    return {'record_path': validation['record_path'], 'actual_zero_stiffness_before_quiet': True,
            'quiet_result_retained_without_physical_release': True,
            'quiet_start_s': parameters['quiet_start_s'], 'quiet_end_s': parameters['quiet_end_s'],
            'actual_gains_ready_s': parameters['actual_gains_ready_s'],
            'input_plans': plans, 'input_supplements': supplements,
            'maximum_simultaneous_input_axes': 1 if math.isfinite(first_input) else 0}


def assert_exported_evidence(directory, folder, selected_purpose):
    """Exercise both real exports and the desktop file list on temporary data."""
    sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
    from artifact_exports import export_artifact
    from workflow_registry import export_record, load
    expected = {path.name: path.read_bytes() for pattern in ('gain-anchor-*.csv', 'gain-purpose-*.csv', 'drag-decay-*.csv', 'drag-taught-release-*.csv', 'gain-trajectory-local-*.csv', 'gain-trajectory-*-passing-purpose.csv', 'gain-transition-*-passing-resume.csv')
                for path in folder.glob(pattern)}
    assert expected and any(name.startswith(('gain-purpose-', 'drag-decay-', 'drag-taught-release-')) for name in expected), 'The worker did not save its actual purpose measurements'
    listed = load(str(folder), 'gain')['files']
    assert all(listed.get(name) is True for name in expected), ('Desktop evidence list lost a purpose record', listed.keys())
    destination = directory / 'artifact-export'
    destination.mkdir()
    artifact = export_artifact(str(directory), str(folder), 'gain', destination=str(destination), gain_purposes=[selected_purpose])
    record = export_record(str(directory), str(folder), 'gain')
    for exported, prefix in ((artifact, 'evidence/'), (record, '')):
        with zipfile.ZipFile(exported['path']) as archive:
            for name, original in expected.items():
                assert archive.read(prefix+name) == original, ('Export omitted or changed actual measurement', name)
    assert all((folder / name).read_bytes() == original for name, original in expected.items()), 'Export rewrote the task evidence'
    return {'files': sorted(expected), 'artifact_and_record_export_verified': True}


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


def assert_passing_purpose_path(folder, candidate, names, partial, positions, sample_dt):
    """A normal traversal pauses its source clock, measures, then resumes."""
    events = [item for item in candidate['experiments'] if item.get('kind') == 'taught_path_purpose_dwell']
    assert events, 'The normal path never reached its purpose-measurement dwell'
    assert not [item for item in candidate['experiments']
                if item.get('kind') == 'taught_measurement_anchor_visit'
                and any(part in item.get('record_path', '') for part in ('-purpose-visit', '-purpose-return'))], 'Normal purpose measurements still require separate endpoint visits'
    duration = sample_dt*(len(positions)-1)
    reports = []
    for event in events:
        assert event['method'] == 'purpose_measurement_during_normal_taught_path_traversal', event
        assert event['arrival_error_gate'] is False and event['exact_anchor_arrival_required'] is False, event
        assert event['validated'] is False and event['playback_rate'] == 1, event
        assert abs(event['source_duration_s']-duration) < 1e-8
        anchor = event['source_index']
        assert 0 < anchor < len(positions)-1, 'Normal traversal stopped at a boundary endpoint for purpose testing'
        np.testing.assert_allclose(event['reference_rad'], positions[anchor], atol=1e-10, rtol=0)
        timeline = positions[::-1] if event['reverse'] else positions
        rows = read_rows(folder/event['record_path'])
        groups = [[row for row in rows if row['joint'] == name] for name in names]
        assert min(map(len, groups)) >= 30 and len(set(map(len, groups))) == 1, event['record_path']
        clock = np.array([float(row['reference_sample_time_s']) for row in groups[0]])
        source = np.array([float(row['trajectory_source_time_s']) for row in groups[0]])
        assert np.all(np.diff(clock) > 0) and np.all(np.diff(source) >= -1e-9)
        assert source.min() >= 0 and source.max() <= duration+1e-8
        for sample in list(zip(*groups))[::17]:
            source_s = float(sample[0]['trajectory_source_time_s'])
            index = min(int(source_s/sample_dt), len(timeline)-2)
            u = min(1., max(0., source_s/sample_dt-index))
            left = timeline[index]-timeline[index-1] if index else np.zeros(len(names))
            middle = timeline[index+1]-timeline[index]
            right = timeline[index+2]-timeline[index+1] if index+2 < len(timeline) else np.zeros(len(names))
            def slope(a, b):
                result = np.zeros(len(names))
                mask = a*b > 0
                result[mask] = 2*a[mask]*b[mask]/(a[mask]+b[mask])
                return result
            expected = ((2*u**3-3*u*u+1)*timeline[index]+(u**3-2*u*u+u)*slope(left,middle)
                        +(-2*u**3+3*u*u)*timeline[index+1]+(u**3-u*u)*slope(middle,right))
            np.testing.assert_allclose([float(row['reference']) for row in sample], expected, atol=1e-7, rtol=0,
                                       err_msg='Passing-purpose travel left the original Hermite branch')
            assert all(row['reference_sample_time_known'] == '1' for row in sample)
            assert all(float(row['independent_disturbance_nm']) == 0 for row in sample), 'Dwell input was relabelled as moving-path evidence'
            assert all(abs(float(row['command_time_s'])-float(row['reference_sample_time_s'])) < .08 for row in sample), 'Source clock replaced the real command/sample clock'
        assert event['pause_begin_s'] > 0, 'No normal-path motion preceded the purpose measurement'
        before = np.flatnonzero(clock <= event['pause_begin_s']+.02)
        assert len(before) > 10
        if event['completed']:
            assert event['resumed'] and event['pause_end_s'] > event['pause_begin_s'], event
            assert event['measurement_dwell_s'] > 0
            assert event['elapsed_s'] >= duration+event['measurement_dwell_s']-.1, 'Dwell wall time disappeared from cost'
            after = np.flatnonzero(clock >= event['pause_end_s']-.02)
            assert len(after) > 10 and duration-source[-1] < .03
            gap = int(np.argmax(np.diff(clock)))
            assert clock[gap+1]-clock[gap] >= event['pause_end_s']-event['pause_begin_s']-.05, 'Main CSV concealed the separate purpose-measurement capture gap'
            source_anchor = (len(positions)-1-anchor if event['reverse'] else anchor)*sample_dt
            assert abs(source[gap]-source_anchor) < .02 and abs(source[gap+1]-source_anchor) < .02, 'Resume skipped unmeasured source-path movement'
            assert source[gap+1]-source[gap] < .02
            np.testing.assert_allclose([float(group[-1]['reference']) for group in groups], timeline[-1], atol=1e-7, rtol=0)
        else:
            assert partial and not event['resumed'], 'Only the explicit cancel fixture may interrupt a dwell'
        reports.append({key:event[key] for key in ('record_path','source_index','reverse','completed','resumed',
                                                  'elapsed_s','measurement_dwell_s','pause_begin_s','pause_end_s')})
    if not partial:
        assert len(events) == (2 if candidate['category'] == 'all' else 1)
        assert all(event['completed'] for event in events)
    else:
        assert events[0]['completed'] and 'native-' in events[0]['record_path']
        assert len(events) == 2 and not events[1]['completed'] and 'compliant-' in events[1]['record_path']
    cost = sum(event['elapsed_s'] for event in events)
    assert 0 < cost <= candidate['actual_elapsed_s']
    return {'passing_measurements':reports,'actual_passing_cost_s':cost,'original_T_unchanged':True,
            'source_duration_s':duration,'arrival_error_gate':False,'main_capture_keeps_real_dwell_gap':True}


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
    visits = [item for item in experiments if item.get('kind') == 'taught_measurement_anchor_visit'
              and '-local-' not in item.get('record_path', '')]
    dwells = [(index,item) for index,item in enumerate(experiments) if item.get('kind') == 'taught_path_purpose_dwell']
    assert visits or dwells, 'The real worker never reached an interior taught pose before purpose measurements'
    completed = [item for item in visits if item['completed']]
    paired, measured_records, drag_preparations, approaches = {}, [], [], {}
    active_visit = None
    for event_index, item in enumerate(experiments):
        if dwells:
            active_visit = next((event for index,event in dwells if index >= event_index), None)
        if item.get('kind') == 'taught_measurement_anchor_visit':
            if '-local-' in item.get('record_path', ''):
                continue
            assert item['arrival_error_gate'] is False and item['reference_only'] is True, item
            source, destination = item['source_index_from'], item['source_index_to']
            assert 0 <= source < len(positions) and 0 <= destination < len(positions) and source != destination, item
            np.testing.assert_allclose(item['reference_rad'], positions[destination], atol=1e-10, rtol=0)
            assert item['planned_cost_s'] > 0 and item['elapsed_s'] >= 0, item
            record = item['record_path']
            assert record.startswith('gain-anchor-') and any(part in record for part in ('purpose-visit','purpose-return','-approach-','-release-')), item
            label = record.split('gain-anchor-', 1)[1].split('-purpose-', 1)[0]
            if '-approach-' in record:
                label=record.split('-approach-',1)[0]
                assert active_visit is not None, 'Matched approach ran outside an interior measurement dwell'
                anchor=active_visit.get('source_index',active_visit.get('source_index_to'))
                assert abs(source-destination)*sample_dt <= .5+sample_dt+1e-8, 'Matched approach took a remote detour'
                if record.endswith('-approach-out.csv'):
                    assert source == anchor and destination != anchor
                    approaches[label]={'out':item}
                else:
                    assert record.endswith('-approach-in.csv') and label in approaches
                    outbound=approaches[label]['out']
                    assert source == outbound['source_index_to'] and destination == anchor, 'Matched approach returned to a different source occurrence'
                    approaches[label]['in']=item
            elif '-release-' in record:
                assert active_visit is not None, 'Demonstrated release left the current interior measurement dwell'
                anchor=active_visit.get('source_index',active_visit.get('source_index_to'))
                assert max(abs(source-anchor),abs(destination-anchor))*sample_dt <= 4+sample_dt, item
            elif 'purpose-visit' in record:
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
            assert item['center_source'] == 'completed_taught_anchor_nominal_reference_not_actual_rezero', item
            np.testing.assert_allclose(center, active_visit['reference_rad'], atol=1e-10, rtol=0,
                                       err_msg='Purpose pulse silently replaced the taught target with actual lagging feedback')
            assert len(item['entry_actual_rad']) == len(names) and all(math.isfinite(value) for value in item['entry_actual_rad']), item
            pulse_rows = read_rows(folder / item['record_path'])
            for axis, name in enumerate(names):
                np.testing.assert_allclose([float(row['reference']) for row in pulse_rows if row['joint'] == name],
                                           center[axis], atol=1e-9, rtol=0,
                                           err_msg='Actual purpose commands re-zeroed the nominal target')
            assert np.all((center-lower)/span > .05) and np.all((upper-center)/span > .05), (item['record_path'], center)
            prior_event = next(event for event in experiments if event.get('kind') == 'purpose_input_prior'
                               and event.get('record_path') == item['record_path'])
            priors = prior_event['input_priors']
            assert len(priors) == len(names), 'Purpose input scale omitted a joint'
            for axis, prior in enumerate(priors):
                assert prior['joint'] == names[axis] and prior['reason'], prior
                if not prior['valid']:
                    continue
                assert prior['seed_nm'] > 0 and prior['source'].startswith('gain-trajectory-'), prior
                source_rows = [row for row in read_rows(folder / prior['source'])
                               if row['joint'] == names[axis] and row['new_measurement'] == '1'
                               and row['actual_gains_known'] == '1' and row['limited'] == '0']
                assert len(source_rows) >= 8
                measured_input = max(abs(float(row['independent_disturbance_nm'])) for row in source_rows)
                pd_peak = max(abs(float(row['actual_kp'])*(float(row['reference'])-float(row['actual']))
                                 +float(row['actual_kd'])*(float(row['reference_velocity'])-float(row['velocity'])))
                              for row in source_rows)
                assert prior['independent_peak_nm'] <= measured_input+1e-8, 'Purpose prior invented independent-input evidence'
                assert prior['low_speed_pd_level_nm'] <= pd_peak+1e-8, 'Purpose prior invented low-speed PD evidence'
                assert prior['seed_nm'] <= max(measured_input,pd_peak)+1e-8, 'Purpose seed scale is not supported by its recorded source'
            assert item['waveform'] == 'isolated_smooth_pulse' and item['isolation'] == 'sequential_single_axis', item
            assert all(axis['scheduled'] for axis in item['pulse_axes']), 'The roomy Mock fixture unexpectedly skipped an axis'
            measured_records.append(item['record_path'])
        elif item.get('kind') == 'hold_parameters':
            assert active_visit is not None, 'Default hold measurement ran outside its completed interior visit'
            nominal = np.asarray(active_visit['reference_rad'])
            assert item['target_source'] == 'completed_taught_anchor_nominal_reference_not_actual_rezero', item
            np.testing.assert_allclose(item['target_pos'], nominal, atol=1e-10, rtol=0)
            assert len(item['entry_actual_pos']) == len(names) and all(math.isfinite(value) for value in item['entry_actual_pos'])
            hold_rows = read_rows(folder / item['record_path'])
            for axis, name in enumerate(names):
                values = [row for row in hold_rows if row['joint'] == name]
                np.testing.assert_allclose([float(row['reference']) for row in values], nominal[axis], atol=1e-9, rtol=0,
                                           err_msg='Hold captured actual position instead of the emitted taught anchor')
                final = [row for row in values if row['final_window'] == '1']
                assert len(final) >= 30 and all(row['actual_gains_known'] == '1' for row in final), item
                np.testing.assert_allclose([float(row['kp']) for row in final], item['kp'][axis], atol=1e-6, rtol=0)
                np.testing.assert_allclose([float(row['kd']) for row in final], item['kd'][axis], atol=1e-6, rtol=0)
        elif item.get('kind') == 'drag_excitation_parameters':
            assert active_visit is not None, 'Drag preparation fell back to the boundary endpoint'
            center = np.asarray(item['center_rad'])
            assert np.all((center-lower)/span > .05) and np.all((upper-center)/span > .05), item
            drag_preparations.append(item)
    if dwells:
        passing = assert_passing_purpose_path(folder, candidate, names, partial, positions, sample_dt)
        assert measured_records, 'A passing-purpose dwell did not save its actual measurements'
        passing['interior_disturbance_records'] = measured_records
        assert all(set(pair) == {'out','in'} for pair in approaches.values()), 'A matched approach did not return to its nominal anchor'
        passing['matched_approaches'] = len(approaches)
        return passing
    primary = 'drag' if candidate['category'] == 'drag' else 'native'
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


def assert_budget_reservations(candidate, partial):
    events = [item for item in candidate['experiments']
              if item.get('kind') == 'gain_stage_budget_reservation']
    if candidate['category'] == 'all':
        compliant = [item for item in events if item['purpose'] == 'compliant']
        assert compliant, 'The four-use worker did not record the later compliant measurement reservation'
        for event in compliant:
            assert event['scope'] == 'remaining_purposes_and_frozen_groups', event
            assert 0 < event['future_measurements_reserved_s'] < candidate['time_budget_s'], event
        if not partial:
            skipped = [item for item in candidate['experiments']
                       if item.get('kind') == 'interior_purpose_measurement_skipped']
            assert not skipped, ('The roomy four-use fixture spent another purpose\'s measurement reserve', skipped)
    return {'stages': events, 'original_T_s': candidate['budget_source_duration_s'],
            'target_budget_s': candidate['target_budget_s'], 'hard_limit_s': candidate['time_budget_s']}


def assert_local_comparisons(folder, candidate, names):
    plans = [item for item in candidate['experiments'] if item.get('kind') == 'taught_local_correction_plan']
    comparisons = [item for item in candidate['candidate_comparisons']
                   if item.get('kind') == 'taught_local_complete_group_comparison']
    reports = []
    for plan in plans:
        dwell = next((item for item in candidate['experiments']
                      if item.get('kind') == 'taught_path_purpose_dwell'
                      and item['record_path'] == 'gain-trajectory-'+plan['purpose']+'-passing-purpose.csv'), None)
        if dwell:
            assert max(abs(plan[key]-dwell['source_index'])*plan['source_dt_s']
                       for key in ('source_index_from','source_index_to')) <= 4+plan['source_dt_s'], 'The passing-purpose local poll detoured to a remote source segment'
    for comparison in comparisons:
        plan = next(item for item in plans if item['purpose'] == comparison['purpose'])
        assert comparison['model_identified'] is False and comparison['reference_only'] is True
        assert plan['model_identified'] is False and plan['complete_comparison_cost_s'] > 0
        assert plan['complete_comparison_cost_s'] < candidate['time_budget_s']
        for field in ('source_index_from', 'source_index_to', 'reverse', 'playback_rate'):
            assert comparison[field] == plan[field], (field, comparison, plan)
        assert comparison['reverse'] == (comparison['source_index_from'] > comparison['source_index_to'])
        assert comparison['feedforward_scope'] and comparison['load_scope'] and comparison['tail_s'] > 0
        previous = np.array(comparison['previous_kp']+comparison['previous_kd'])
        proposed = np.array(comparison['proposed_kp']+comparison['proposed_kd'])
        assert np.count_nonzero(abs(previous-proposed) > 1e-9) == 1, 'The empirical poll changed several coordinates'
        pair = []
        for path_key, prefix in (('baseline_record_path', 'previous'), ('record_path', 'proposed')):
            path = comparison[path_key]
            assert path.startswith('gain-trajectory-local-'), path
            groups = cycle_groups(read_rows(folder / path), names)
            for axis, values in enumerate(groups):
                assert all(abs(float(row['independent_disturbance_nm'])) < 1e-10 for row in values)
                np.testing.assert_allclose([float(row['actual_kp']) for row in values], comparison[prefix+'_kp'][axis], atol=1e-6, rtol=0)
                np.testing.assert_allclose([float(row['actual_kd']) for row in values], comparison[prefix+'_kd'][axis], atol=1e-6, rtol=0)
            pair.append(groups)
        # Compare the two actual emitted reference schedules at common times,
        # rather than accepting matching labels on a forward and reverse pass.
        for axis in range(len(names)):
            before, after = pair[0][axis], pair[1][axis]
            before_t = np.array([float(row['reference_sample_time_s']) for row in before])
            after_t = np.array([float(row['reference_sample_time_s']) for row in after])
            common = np.arange(max(before_t[0], after_t[0]), min(before_t[-1], after_t[-1]), .02)
            assert len(common) >= 20
            for key, atol in (('reference', 2e-4), ('reference_velocity', .003)):
                a = np.interp(common, before_t, [float(row[key]) for row in before])
                b = np.interp(common, after_t, [float(row[key]) for row in after])
                # Near a stopped segment boundary, two asynchronous sample
                # grids cannot reconstruct a curved velocity ramp to a fixed
                # scalar tolerance. Bound only that interpolation uncertainty
                # by the actual bracketing command changes; a plateau's bound
                # remains zero, so a slower/different path still fails.
                if key == 'reference_velocity':
                    def bracket_span(stamps, values):
                        upper = np.clip(np.searchsorted(stamps, common), 1, len(stamps)-1)
                        data = np.asarray(values)
                        return abs(data[upper]-data[upper-1])
                    uncertainty = .5*(bracket_span(before_t, [float(row[key]) for row in before])
                                      +bracket_span(after_t, [float(row[key]) for row in after]))
                    assert np.all(abs(a-b) <= atol+uncertainty), 'The local poll changed actual reference velocity beyond recorded sampling resolution'
                else:
                    np.testing.assert_allclose(a, b, rtol=0, atol=atol,
                        err_msg='The local poll changed actual reference direction, speed or comparison window')
            assert all(math.isfinite(float(row['feedforward_nm'])) for row in before+after)
        assert len(comparison['baseline_axes']) == len(comparison['candidate_axes']) == len(names)
        reports.append({'baseline': comparison['baseline_record_path'], 'compared': comparison['record_path'],
                        'kept': comparison['kept'], 'changed_joint': comparison['changed_joint'],
                        'coordinate': comparison['coordinate'], 'model_identified': False,
                        'comparison_reason': comparison.get('comparison_reason', comparison['reason'])})
    return {'plans': len(plans), 'measured_comparisons': reports,
            'scope': 'actual_native_scheduling_and_recording_not_physical_improvement'}


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
                    model = event['models'][axis]
                    assert model['valid'] or model.get('response_establishment_trial'), event
                    if model.get('response_establishment_trial'):
                        assert not model['valid'], 'An empirical response trial became a successful model fit'
                        assert abs(model['measured_error_bias_rad']) > 0
                        assert event['proposed_kd'][axis] == event['previous_kd'][axis], 'Static response establishment changed damping without dynamic evidence'
                        assert event['previous_kp'][axis] < event['proposed_kp'][axis] <= model['unresolved_trial_kp_ceiling']+1e-8
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
    first_motion = next(row for row in frames if row['phase']=='gain_adaptation'
                        and any(abs(float(row['ref_dq:' + name])) > 1e-6 for name in names))
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
        assert all(math.isclose(float(row['actual_kp']), selected['gains']['kp'][axis], abs_tol=1e-6)
                   and math.isclose(float(row['actual_kd']), selected['gains']['kd'][axis], abs_tol=1e-6) for row in values), name
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
    selected = targets.get('rigid', candidate)
    validation = selected['native_validation']
    native_group = assert_frozen_group(folder, selected, names)
    compliant_group = assert_frozen_group(folder, targets['compliant'], names) if targets else None
    independent = [item for item in candidate['experiments'] if item.get('kind') == 'independent_torque_disturbance']
    assert independent, [item.get('kind') for item in candidate['experiments']]
    pulse_records = []
    for experiment in independent:
        torque_rows = read_rows(folder / experiment['record_path'])
        assert max(abs(float(row['independent_disturbance_nm'])) for row in torque_rows) > 0, experiment
        parameters = next(item for item in candidate['experiments'] if item.get('kind') == 'short_experiment_parameters'
                          and item.get('record_path') == experiment['record_path'])
        assert parameters['independent_disturbance'] and parameters['waveform'] == 'isolated_smooth_pulse', parameters
        assert parameters['isolation'] == 'sequential_single_axis', parameters
        assert parameters['other_axes_execution'] == 'fixed_reference_same_complete_gains', parameters
        assert experiment['method'] == 'isolated_smooth_torque_pulse' and experiment['model_identified'] is False, experiment
        assert experiment['record_path'].startswith('gain-purpose-'), experiment
        pulse = assert_isolated_pulses(torque_rows, names, parameters['pulse_axes'], parameters['kp'], parameters['kd'])
        for axis, name in enumerate(names):
            axis_rows = [row for row in torque_rows if row['joint'] == name]
            values = [float(row['reference']) for row in axis_rows]
            assert max(values)-min(values) < 1e-7, (name, experiment)
            baseline = [row for row in axis_rows if float(row['time_s']) < .2]
            assert len(baseline) >= 20 and all(abs(float(row['independent_disturbance_nm'])) < 1e-10 for row in baseline), (name, 'No actual unloaded input baseline')
        pulse_records.append({'record_path': experiment['record_path'], **pulse})
        assert len(experiment['axes']) == len(names), 'A single-axis window erased the other axes from the direct measurement'
    assert_reference_only(candidate)
    assert not candidate['validated'], 'Recorded references must not be serialized as achieved physical acceptance'
    assert not selected['disturbance_verification']['validated'], selected['disturbance_verification']
    assert candidate['available'] and candidate['review_ready'] and candidate['measurement_complete'], 'Complete frozen tracking parameters must remain available with independent-disturbance warnings'
    direct = selected['disturbance_verification']
    assert direct['method'] == 'isolated_smooth_torque_pulse' and direct['model_identified'] is False, direct
    assert 'direct_measurement_complete' in direct and len(direct['axes']) == len(names), direct
    for target in (targets.values() if targets else [selected]):
        evidence=target.get('disturbance_verification',{})
        if evidence.get('method') != 'isolated_smooth_torque_pulse':
            continue
        axes=evidence['axes']
        resolved_complete=len(axes)==len(names) and all(axis['direct_response_complete'] and axis['measurement_complete'] for axis in axes)
        assert evidence['direct_measurement_complete'] == resolved_complete, 'Complete input windows became complete physical response evidence'
        if not resolved_complete:
            assert not target['qualification_complete'], 'Unresolved purpose response became complete purpose qualification'
        assert target['available'], 'Missing direct-response evidence erased a completed parameter group'
    if not direct['direct_measurement_complete']:
        assert not selected['qualification_complete'], 'Missing direct purpose evidence became complete purpose qualification'
    if targets:
        assert set(targets) == {'rigid', 'compliant', 'drag'}, targets.keys()
        assert all(target['available'] for target in targets.values()), 'Completed purpose parameter groups were erased after missing independent-input evidence'
        assert all(target['validated'] is False for target in targets.values()), 'A reference record became an automatic purpose pass'
        assert all(value == 0 for value in targets['drag']['gains']['kp']), targets['drag']
        drag = assert_drag_zero_stiffness(folder, targets['drag'], names, candidate)
        rigid = targets['rigid']
        assert rigid['hold_disturbance_verification']['record_path'] == rigid['disturbance_verification']['record_path'], 'Rigid hold re-injected an already measured same-condition purpose input'
        compliant = targets['compliant']
        assert compliant['hold_disturbance_verification']['record_path'] == compliant['disturbance_verification']['record_path'], 'Compliant hold duplicated the same-condition purpose input'
        shared = [item for item in candidate['candidate_comparisons'] + candidate['experiments']
                  if item.get('kind') == 'shared_disturbance_evidence']
        assert shared, 'Shared purpose evidence was not recorded as reused evidence'
        known_records = {item['record_path']: item for item in independent}
        assert all(item['record_path'] in known_records for item in shared), shared
    return {'native': native_group, 'compliant': compliant_group,
            'drag': drag if targets else None,
            'frozen_complete_group': True, 'independent_disturbance_records': [item['record_path'] for item in independent],
            'isolated_pulses': pulse_records, 'direct_measurement_complete': direct['direct_measurement_complete'],
            'parameter_model_identified': direct['model_identified'],
            'parameter_measurement_complete': candidate['measurement_complete'],
            'qualification_complete': selected['qualification_complete'], 'review_ready': candidate['review_ready'],
            'validated': candidate['validated']}


def assert_displaced_passing_resume(folder, candidate, names):
    """Require the real P0 -> assessment HOLD -> soft bridge failure branch."""
    experiments = candidate['experiments']
    bridge = [event for event in experiments
              if event.get('kind') == 'feedback_bounded_endpoint_transition'
              and event.get('record_path') == 'gain-transition-compliant-passing-resume.csv']
    assert len(bridge) == 1 and bridge[0]['completed'], 'The displaced P0 fixture never completed its soft resume bridge'
    bridge = bridge[0]
    difference = np.asarray(bridge['initial_reference_rad'])-np.asarray(bridge['target_rad'])
    assert np.max(abs(difference)) > 1e-8, 'The fixture accidentally restored the nominal target before its resume test'
    assert bridge['reference_continuation'] and bridge['elapsed_s'] >= .5
    frames = read_rows(folder/'frames.csv')
    transition = [row for row in frames if row['gain_stage'] == 'compliant-passing-resume-endpoint-transition']
    assert transition and all(row['impedance_mode'] == 'COMPLIANT' for row in transition), \
        'Resume submitted a reference while the actual runtime was still in assessment HOLD or DRAG'
    assert any(all(float(row['cmd_kp:'+name]) > 0 for name in names) for row in transition), \
        'The soft bridge never installed positive tracking feedback gains'
    selected = candidate.get('targets', {}).get('compliant', candidate)
    for key in ('kp','kd'):
        np.testing.assert_allclose(bridge[key], selected['gains'][key], atol=1e-8, rtol=0)
    before = frames[:frames.index(transition[0])]
    assert any(row['impedance_mode'] == 'DRAG'
               and all(abs(float(row['cmd_kp:'+name])) < 1e-10 for name in names) for row in before), \
        'The regression never entered actual P0 before resuming soft tracking'
    passing = next(event for event in experiments if event.get('kind') == 'taught_path_purpose_dwell'
                   and event['record_path'] == 'gain-trajectory-compliant-passing-purpose.csv')
    assert passing['completed'] and passing['resumed']
    assert selected['native_validation']['completed']
    release_events = [event for event in experiments if event.get('kind') == 'drag_demonstrated_motion_release']
    assert release_events, 'Unresolved torque-pulse release never used the nearby moving taught segment'
    release_records = []
    for event in release_events:
        rows = read_rows(folder/event['record_path'])
        groups = cycle_groups(rows, names)
        assert event['source_index_from'] < event['source_index_release'] and event['preparation_duration_s'] > 0
        if event['domain_stopped']:
            assert not any(axis['measurement_complete'] for axis in event['axes']), 'Domain-limited preparation became complete release evidence'
            release_records.append({'record_path':event['record_path'], 'domain_stopped':True, 'axes':event['axes']})
            continue
        assert event['actual_zero_stiffness_s'] is not None, event
        for group in groups:
            before_zero = [row for row in group if float(row['time_s']) < event['actual_zero_stiffness_s']]
            after_zero = [row for row in group if float(row['time_s']) >= event['actual_zero_stiffness_s']+.02]
            assert len(before_zero) >= 20 and len(after_zero) >= 20
            assert any(float(row['actual_kp']) > 0 for row in before_zero)
            assert all(abs(float(row['actual_kp'])) < 1e-10 for row in after_zero)
            assert all(abs(float(row['independent_disturbance_nm'])) < 1e-10 for row in group)
        release_records.append({'record_path':event['record_path'], 'actual_zero_stiffness_s':event['actual_zero_stiffness_s'],
                                'preparation_duration_s':event['preparation_duration_s'], 'axes':event['axes']})
    return {'zero_stiffness_feedback_step_rad':.004, 'one_requested_fresh_feedback_step_rad':.004,
            'callback_scope':'P0_then_real_taught_release_then_fresh_measured_HOLD_not_full_purpose_search',
            'same_rule_for_every_axis':True,
            'resume_bridge':bridge['record_path'], 'bridge_initial_offset_rad':difference.tolist(),
            'bridge_runtime_mode':'COMPLIANT', 'compliant_passing_resumed':True,
            'compliant_frozen_completed':True, 'taught_motion_release':release_records,
            'scope':'native_mode_handoff_and_evidence_not_physical_gain_performance'}


def run_focused_resume_case(terminal, timeout_s=180):
    """Exercise the actual resume branch and final anchor without full searches."""
    native = None
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix='serialarm-focused-resume-') as temporary:
        directory=Path(temporary)
        config,names=prepare_config(directory)
        document=json.loads(config.read_text())
        document['safety_policy']['cmd_vel_scale']=min(document['safety_policy']['cmd_vel_scale'],.03)
        document['safety_policy']['max_acc']={name:min(value,.6) for name,value in document['safety_policy']['max_acc'].items()}
        recovery=document['safety_policy']['fault_recovery']['compliant_recovery']
        recovery['max_vel']={name:min(value,.03) for name,value in recovery['max_vel'].items()}
        config.write_text(json.dumps(document,indent=2))
        before=config.read_bytes()
        storage=directory/'storage';storage.mkdir()
        try:
            native=Native(terminal,config,storage)
            native.request('activate');native.ready()
            prepare_source(native,directory/'source',names,25)
            task=native.request('model_calibration_import_trajectory',{'directory':str(directory/'source'),'pose_budget':5})
            plan=native.request('model_calibration_gain_plan',{'gain_mode':'compliant','gain_tuning_mode':'fine'})
            assert plan['target_budget_s']==300 and plan['hard_limit_s']==375 and plan['budget_source_duration_s']==25,plan
            native.request('model_calibration_start',{'task_id':task['task_id'],'workflow':'gain','gain_mode':'compliant',
                'gain_tuning_mode':'fine','released':True,'gain_trajectory_confirmed':True,'gain_motion_range_confirmed':True})
            deadline=started+timeout_s
            while time.monotonic()<deadline:
                status=native.request('model_calibration_status')
                assert native.request('status')['robot_state']=='ACTIVE',status
                if status['phase'] in ('complete','failed','cancelled'):break
                time.sleep(.03)
            else:raise AssertionError('Focused resume worker did not finish')
            candidate,result,status=final_record(native,task,deadline)
            folder=Path(task['directory'])
            assert status['phase']=='complete' and not candidate['termination']['fault_interrupted'],candidate['termination']
            assert candidate['category']=='compliant' and not candidate.get('targets')
            assert candidate['budget_source_duration_s']==25 and candidate['actual_elapsed_s']<=375
            assert candidate['termination']==result['termination']==status['termination']
            assert not candidate['response_guard_events']
            resume=assert_displaced_passing_resume(folder,candidate,names)
            frozen=assert_frozen_group(folder,candidate,names)
            with (folder/'trajectory.csv').open() as stream:
                source=csv.reader(stream);dt=float(next(source)[1]);next(source)
                positions=np.asarray([[float(value) for value in row[1:]] for row in source])
            passing=assert_passing_purpose_path(folder,candidate,names,False,positions,dt)
            dwell=next(event for event in candidate['experiments'] if event.get('kind')=='taught_path_purpose_dwell')
            assert len(dwell['nearby_reference_span_rad'])==len(dwell['axis_response_weights'])==len(names)
            assert all(math.isfinite(value) and value>=0 for value in dwell['axis_response_weights'])
            exports=assert_exported_evidence(directory,folder,'compliant')
            assert config.read_bytes()==before,'The fixture changed its input configuration'
            retained=Path(tempfile.mkdtemp(prefix='serialarm-focused-resume-result-'))/'evidence'
            shutil.copytree(directory,retained)
            return {'kind':'focused_resume','offline_mock_only':True,'compiler_optimization':'O0_only_temporary_fixture',
                'actual_elapsed_s':candidate['actual_elapsed_s'],'recorded_T_s':25,'target_budget_s':300,'hard_limit_s':375,
                'termination':candidate['termination'],'resume':resume,'frozen':frozen,'passing':passing,
                'anchor':{key:dwell[key] for key in ('source_index','nearby_response_motion_score','nearby_reference_span_rad','axis_response_weights')},
                'exports':exports,'retained_evidence':str(retained),'source_profile_unchanged':True,
                'scope':'actual_mode_handoff_anchor_resume_frozen_and_exports_not_full_purpose_tuning'}
        except Exception:
            retained=Path(tempfile.mkdtemp(prefix='serialarm-focused-resume-failed-'))/'evidence'
            shutil.copytree(directory,retained)
            print('Retained focused native fixture: '+str(retained),file=sys.stderr)
            if native:print(native.diagnostics(),file=sys.stderr)
            raise
        finally:
            if native:native.close()


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
            mode, depth = ('rigid', 'fine') if kind == 'fine' else ('all', 'quick')
            target_budget, hard_limit = (450, 600) if mode == 'all' else (300, 375)
            plan = native.request('model_calibration_gain_plan', {'gain_mode': mode, 'gain_tuning_mode': depth})
            assert plan['valid'] and plan['budget_source_duration_s'] == 25 and plan['target_budget_s'] == target_budget and plan['hard_limit_s'] == hard_limit, plan
            assert plan['max_path_passes'] == (6 if depth == 'quick' else 4), plan
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
                if (kind == 'cancel' and status.get('gain_path_pass_current', 0) >= 4
                        and status.get('gain_probe_kind', '').startswith('compliant-disturbance')
                        and stage == 'disturbance_identification'):
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
            assert candidate['category'] == ('all' if mode == 'all' else 'rigid'), candidate['category']
            assert candidate['seed_source'].startswith('urdf') and candidate['seed_source'] == candidate['model_initialization']['source'], candidate['seed_source']
            assert not candidate['model_initialization']['measured'] and not candidate['task_model_demand']['executed']
            nominal_basis = assert_nominal_path_basis(candidate, names)
            assert candidate['budget_source_duration_s'] == 25 and candidate['target_budget_s'] == target_budget and candidate['time_budget_s'] == hard_limit
            assert candidate['budget_source_task_id'] == 'offline-original-25s'
            requirements = candidate['gain_requirements']
            assert requirements['acceptance_scope'] == 'measured_response_reference_only_v1', requirements
            assert requirements['accuracy_thresholds_enabled'] is False and requirements['legacy_accuracy_fields_ignored'] is True, requirements
            assert not {'peak_error_rad', 'rms_error_rad', 'hold_error_rad', 'settling_time_s'} & requirements.keys(), 'Ignored historical precision fields re-entered the recorded scoring contract'
            assert candidate['actual_elapsed_s'] <= hard_limit+1, 'Original wall budget was exceeded'
            assert candidate['response_guard_events'] == []
            learning = assert_online_adaptation(folder, candidate, names, kind == 'cancel')
            anchors = assert_anchor_visits(folder, candidate, names, kind == 'cancel')
            reservations = assert_budget_reservations(candidate, kind == 'cancel')
            if kind == 'cancel':
                assert status['phase'] == 'cancelled' and candidate['termination']['kind'] == 'cancelled', status
                assert candidate['termination']['reason'] == 'stop_requested' and not candidate['termination']['fault_interrupted'], candidate['termination']
                native_target = candidate['targets']['rigid']
                assert native_target['native_validation']['completed'] and native_target['native_validation']['available'], 'Cancelling a later purpose measurement erased the completed native frozen group'
                assert candidate['available'] and candidate['review_ready'] and not candidate['validated'], candidate
                assert not candidate['targets']['compliant']['available'], 'An interrupted compliant group became applicable without its frozen traversal'
                frozen = assert_frozen_group(folder, native_target, names)
                frozen['frozen_group_retained_after_cancel'] = True
            else:
                assert status['phase'] == 'complete' and candidate['termination']['kind'] in ('measured_reference_complete', 'insufficient_measurement'), candidate['termination']
                assert not candidate['termination']['fault_interrupted'], candidate['termination']
                frozen = assert_frozen(folder, candidate, names)
                if kind == 'fine':
                    assert not candidate.get('targets'), 'Single-use fine tuning serialized or overwrote other purposes'
                    assert candidate['path_passes'] == 4 and candidate['path_pass_limit'] == 4, candidate
                    assert len(list(folder.glob('gain-trajectory-native-online-*.csv'))) == 2
                    assert not list(folder.glob('gain-trajectory-compliant-*.csv')), 'Fine servo tuning unexpectedly executed another purpose'
                else:
                    assert candidate['path_passes'] == 6 and candidate['path_pass_limit'] == 6, candidate
            assert native.request('get_admittance')['execution_gains'] == original_gains, 'The worker retained temporary calibration gains after exit'
            assert config.read_text() == before_config, 'The worker modified its input Profile'
            native.request('deactivate')
            applied = native.request('candidate_gain_apply', {'path': str(folder / 'gain-candidate.json'), 'gain_purposes': ['rigid']})
            assert applied['gain_purposes'] == ['rigid'], applied
            now = native.request('get_admittance')['execution_gains']
            target = candidate.get('targets', {}).get('rigid', candidate)
            assert now['rigid'] == {'kp': target['gains']['kp'], 'kd': target['gains']['kd']}, now
            assert {key: value for key, value in now.items() if key != 'rigid'} == {
                key: value for key, value in original_gains.items() if key != 'rigid'}, now
            native.request('candidate_gain_restore')
            assert native.request('get_admittance')['execution_gains'] == original_gains
            assert config.read_text() == before_config, 'Temporary candidate application wrote the Profile'
            local = assert_local_comparisons(folder, candidate, names)
            exports = assert_exported_evidence(directory, folder, 'rigid')
            assert config.read_text() == before_config, 'Artifact export changed the active configuration'
            return {'offline_mock_only': True, 'kind': kind, 'elapsed_s': time.monotonic()-started,
                    'actual_elapsed_s': candidate['actual_elapsed_s'],
                    'old_gain_scale': old_gain_scale, 'old_servo_gains': original_gains['rigid'],
                    'seed_source': candidate['seed_source'], 'seed_kp': candidate['seed_kp'], 'seed_kd': candidate['seed_kd'],
                    'nominal_path_basis': nominal_basis,
                    'recorded_T_s': candidate['budget_source_duration_s'], 'target_budget_s': candidate['target_budget_s'],
                    'hard_limit_s': candidate['time_budget_s'],
                    'observed_stages': stages, 'online_learning': learning, 'anchor_visits': anchors,
                    'budget_reservations': reservations, 'local_comparisons': local, 'frozen': frozen, 'exports': exports,
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
            assert plan['valid'] and plan['budget_source_duration_s'] == 25 and plan['target_budget_s'] == 300 and plan['hard_limit_s'] == 375, plan
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
            assert candidate['category'] == 'drag' and not candidate.get('targets'), candidate.get('category')
            assert_reference_only(candidate)
            assert candidate['available'] and candidate['review_ready'] and not candidate['validated']
            assert candidate['path_passes'] == 0, 'Drag-only measurement invented a full tracking traversal'
            assert candidate['budget_source_duration_s'] == 25 and candidate['target_budget_s'] == 300 and candidate['time_budget_s'] == 375
            assert candidate['actual_elapsed_s'] <= 376 and candidate['budget_source_task_id'] == 'offline-original-25s'
            assert all(value == 0 for value in candidate['gains']['kp']) and all(math.isfinite(value) and value >= 0 for value in candidate['gains']['kd']), candidate
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
            drag = assert_drag_zero_stiffness(folder, candidate, names)
            assert native.request('get_admittance')['execution_gains'] == original_gains and config.read_text() == before_config
            native.request('deactivate')
            applied = native.request('candidate_gain_apply', {'path': str(folder / 'gain-candidate.json'), 'gain_purposes': ['drag']})
            assert applied['gain_purposes'] == ['drag'], applied
            now = native.request('get_admittance')['execution_gains']
            assert now['drag'] == {'kp': candidate['gains']['kp'], 'kd': candidate['gains']['kd']}, (applied, now)
            assert {key: value for key, value in now.items() if key != 'drag'} == {
                key: value for key, value in original_gains.items() if key != 'drag'}, now
            native.request('candidate_gain_restore')
            assert native.request('get_admittance')['execution_gains'] == original_gains and config.read_text() == before_config
            exports = assert_exported_evidence(directory, folder, 'drag')
            assert config.read_text() == before_config
            return {'offline_mock_only': True, 'kind': 'drag', 'elapsed_s': time.monotonic()-started,
                    'actual_elapsed_s': candidate['actual_elapsed_s'],
                    'recorded_T_s': 25, 'target_budget_s': 300, 'hard_limit_s': 375, 'endpoint_offset_rad': .005,
                    'anchor_visits': anchors, 'cold_handoff_record': handoffs[0]['record_path'], 'exports': exports,
                    'path_passes': candidate['path_passes'], 'kp': candidate['gains']['kp'], 'kd': candidate['gains']['kd'], 'drag': drag,
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
    parser.add_argument('--cases', choices=('all', 'complete', 'cancel', 'fine', 'drag', 'resume'), default='all')
    args = parser.parse_args()
    kinds = ('complete', 'cancel', 'fine', 'drag') if args.cases == 'all' else (args.cases,)
    if args.cases == 'resume':
        with tempfile.TemporaryDirectory(prefix='serialarm-displaced-native-build-') as temporary:
            terminal = displaced_zero_stiffness_terminal(args.terminal.resolve(), Path(temporary))
            cases = [run_focused_resume_case(terminal)]
    else:
        cases = [run_cold_drag_case(args.terminal.resolve()) if kind == 'drag'
                 else run_case(args.terminal.resolve(), kind) for kind in kinds]
    print(json.dumps({'scope':'real_native_write_disabled_mock_not_physical_acceptance','cases':cases}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
