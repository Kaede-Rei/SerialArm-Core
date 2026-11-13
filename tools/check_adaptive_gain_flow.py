#!/usr/bin/env python3
"""Exercise the real native gain IPC with temporary, hardware-free fixtures.

No hardware plugin is passed and the temporary Core explicitly disables writes.
The synthetic candidate tests application compatibility, not physical acceptance.
Run under .install/run with the project's GUI Python environment (PyYAML).
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import threading
import time
import traceback

import yaml


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / 'src/robot_supports/robots/dm_arm/description/config/core/white.yaml'


class Native:
    def __init__(self, terminal: Path, config: Path, storage: Path):
        # The only accepted configuration was generated below, inside our
        # temporary workspace. The process has no hardware plugin argument.
        document = yaml.safe_load(config.read_text())
        if document['control']['runtime']['write_enabled'] is not False:
            raise ValueError('Offline flow check requires write_enabled=false')
        self.log = tempfile.TemporaryFile(mode='w+t')
        env = os.environ.copy()
        env['SERIAL_ARM_PROFILE_STORAGE'] = str(storage)
        env['LD_LIBRARY_PATH'] = str(terminal.parent) + (':' + env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
        self.process = subprocess.Popen(
            [str(terminal), '--machine', '--config', str(config)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log,
            text=True, bufsize=1, env=env, cwd=storage.parent)
        self.replies: queue.Queue = queue.Queue()
        self.next_id = 0

        def read():
            for line in self.process.stdout:
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    self.replies.put({'error': 'Invalid native JSON: ' + line[:300]})
                    continue
                if 'id' in value:
                    self.replies.put(value)
            self.replies.put({'error': 'Native process exited before replying'})

        self.reader = threading.Thread(target=read, daemon=True)
        self.reader.start()

    def request(self, method: str, params: dict | None = None, timeout: float = 20):
        self.next_id += 1
        self.process.stdin.write(json.dumps({'id': self.next_id, 'method': method, 'params': params or {}}) + '\n')
        self.process.stdin.flush()
        reply = self.replies.get(timeout=timeout)
        if 'error' in reply:
            raise RuntimeError(f'{method}: {reply["error"]}')
        if reply.get('id') != self.next_id:
            raise RuntimeError('Native reply ID does not match request')
        return reply['result']

    def ready(self):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            status = self.request('status')
            if status.get('write_enabled') is not False:
                raise RuntimeError('Native did not confirm the offline mock backend')
            if status['robot_state'] == 'FAULT':
                raise RuntimeError('Mock fault during setup: ' + json.dumps(status.get('fault')))
            if status['robot_state'] == 'ACTIVE' and status.get('valid') and status.get('feedback_age_ms', 1000) < 30:
                return status
            time.sleep(.02)
        raise RuntimeError('Mock did not publish fresh ACTIVE feedback')

    def close(self):
        if self.process.poll() is None:
            try:
                self.request('deactivate', timeout=5)
            except (RuntimeError, BrokenPipeError, queue.Empty):
                pass
            self.process.stdin.close()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=5)
        self.reader.join(timeout=1)

    def diagnostics(self):
        self.log.seek(0)
        return self.log.read()[-6000:]


def prepare_config(directory: Path, old_gain_scale: float = 1) -> tuple[Path, list[str]]:
    config = yaml.safe_load(EXAMPLE.read_text())
    config['model']['urdf_path'] = str((EXAMPLE.parent / config['model']['urdf_path']).resolve())
    config['control']['runtime']['write_enabled'] = False
    config['capability']['admittance']['enabled'] = False
    config['shutdown']['park_before_disable'] = False
    def scale_gains(value):
        if not isinstance(value, dict):
            return
        for key, item in value.items():
            if key in ('kp', 'kd') and isinstance(item, dict):
                for joint in item:
                    item[joint] *= old_gain_scale
            else:
                scale_gains(item)
    scale_gains(config['control']['controller'])
    path = directory / 'offline-core.yaml'
    # JSON is a YAML subset, and avoids changing the original example/Profile.
    path.write_text(json.dumps(config, indent=2))
    return path, config['model']['joint_names']


def synthetic_record(native: Native, folder: Path, names: list[str], recorded_duration_s: float = 25) -> Path:
    # Obtain real source snapshots and fingerprints from the native recorder.
    # The script does not implement a parallel fingerprint algorithm.
    initial = native.request('model_calibration_teach_begin', {'load_description': 'offline synthetic fixture'})
    native.request('model_calibration_cancel', {'task_id': initial['task_id']})
    recorded = Path(initial['directory'])
    meta = json.loads((recorded / 'metadata.json').read_text())
    folder.mkdir()
    meta['task_id'] = f'offline-original-{recorded_duration_s:g}s'
    meta['gain_budget_source_duration_s'] = recorded_duration_s
    meta['gain_budget_source'] = 'original_recorded_samples'
    meta['gain_budget_source_task_id'] = meta['task_id']
    (folder / 'metadata.json').write_text(json.dumps(meta))
    (folder / 'result.json').write_text(json.dumps({'task_id': meta['task_id'], 'recorded_duration_s': recorded_duration_s}))
    with (folder / 'trajectory.csv').open('w') as stream:
        # A stored/imported path may already have been retimed independently of T.
        stream.write('sample_dt,0.05\nindex,' + ','.join(names) + '\n')
        for index in range(501):
            u = index / 500
            q = [(.45 * (1 - math.cos(4 * math.pi * u)) if axis in (1, 2)
                  else .45 * math.sin(4 * math.pi * u))
                 + .04 * math.sin((axis + 2) * 2 * math.pi * u) for axis in range(len(names))]
            stream.write(str(index) + ',' + ','.join(format(v, '.17g') for v in q) + '\n')
    return recorded


def assert_budget(plan: dict, tuning_mode: str):
    assert plan['valid'], plan
    assert plan['gain_tuning_mode'] == tuning_mode, plan
    assert plan['budget_source_duration_s'] == 25, plan
    assert plan['target_budget_s'] == (450 if tuning_mode == 'quick' else 300), plan
    assert plan['hard_limit_s'] == (600 if tuning_mode == 'quick' else 375), plan


def application_fixture(folder: Path, meta: dict, original_gains: dict, names: list[str]) -> tuple[Path, dict]:
    gains = copy.deepcopy(original_gains['servo_tracking'])
    gains['kp'] = [value * .8 for value in gains['kp']]
    gains['kd'] = [value * .9 for value in gains['kd']]
    metric = {'samples': 100, 'measurement_samples': 100, 'rms_error_rad': .02,
              'peak_error_rad': .04, 'velocity_rms_rad_s': .02, 'settling_s': 1, 'tail_error_rad': .01}
    data = {'schema': 'serial_arm_gain_candidate', 'task_id': meta['task_id'],
            'urdf_fingerprint': meta['urdf_fingerprint'], 'joint_names': names,
            'category': 'servo_tracking', 'available': False, 'validated': False,
            'validation_scope': 'independent_demonstration_trajectory',
            'review_ready': False, 'measurement_complete': False, 'qualification_complete': False,
            'warnings': ['synthetic_application_compatibility_fixture_not_a_physical_measurement'],
            'kp': gains['kp'], 'kd': gains['kd'], 'admittance_required': False,
            'admittance_validation': {'required': False, 'status': 'not_requested', 'validated': False},
            'native_validation': {'completed': True, 'available': True, 'validated': False,
                                  'independent': True, 'duration_s': 1, 'measured_duration_s': 1,
                                  'metrics': [copy.deepcopy(metric) for _ in names]},
            'disturbance_verification': {'available': False, 'completed': False, 'validated': False}}
    path = folder / 'gain-candidate.json'
    path.write_text(json.dumps(data))
    return path, gains


def run(terminal: Path) -> dict:
    checks = []
    native = None
    with tempfile.TemporaryDirectory(prefix='serial-arm-adaptive-flow-') as temporary:
        directory = Path(temporary)
        config, names = prepare_config(directory)
        storage = directory / 'storage'
        storage.mkdir()
        try:
            native = Native(terminal, config, storage)
            before = native.request('get_admittance')['execution_gains']
            native.request('activate')
            native.ready()
            checks.append('real_native_mock_no_hardware_plugin_and_writes_disabled')
            source = directory / 'source-task'
            recorded = synthetic_record(native, source, names)
            native.ready()
            imported = native.request('model_calibration_import_trajectory', {'directory': str(source), 'pose_budget': 5})
            quick = native.request('model_calibration_gain_plan', {'gain_mode': 'all', 'gain_tuning_mode': 'quick'})
            fine = native.request('model_calibration_gain_plan', {'gain_mode': 'servo_tracking', 'gain_tuning_mode': 'fine'})
            assert_budget(quick, 'quick')
            assert_budget(fine, 'fine')
            checks.append('four_use_18T_target_24T_hard_and_single_use_12T_target_15T_hard')
            assert quick['budget_source'] == 'original_recorded_samples', quick
            assert quick['budget_source_task_id'] == 'offline-original-25s', quick
            assert not math.isclose(quick['trajectory_duration_s'], 25, abs_tol=.01), quick
            checks.append('retimed_path_duration_does_not_replace_trusted_recorded_T')
            native.request('model_calibration_cancel', {'task_id': imported['task_id']})
            native.ready()
            repeated = native.request('model_calibration_import_trajectory', {'directory': imported['directory'], 'pose_budget': 5})
            repeated_plan = native.request('model_calibration_gain_plan', {'gain_mode': 'all', 'gain_tuning_mode': 'quick'})
            assert_budget(repeated_plan, 'quick')
            assert repeated_plan['budget_source_task_id'] == quick['budget_source_task_id'], repeated_plan
            checks.append('repeated_import_preserves_original_T_and_origin')

            # Exercise the actual worker entry and stop path, without waiting for
            # a synthetic Mock to claim physical tuning success.
            native.ready()
            native.request('model_calibration_start', {
                'task_id': repeated['task_id'], 'workflow': 'gain', 'gain_mode': 'servo_tracking',
                'gain_tuning_mode': 'fine', 'released': True, 'gain_trajectory_confirmed': True,
                'gain_motion_range_confirmed': True,
                'gain_requirements': {'peak_error_rad': -100, 'rms_error_rad': 'obsolete target',
                                      'hold_error_rad': 0, 'settling_time_s': -1}})
            running = native.request('model_calibration_status')
            assert running['reference_only'] is True and running['accuracy_thresholds_enabled'] is False, running
            native.request('model_calibration_cancel', {'task_id': repeated['task_id']})
            stop_deadline = time.monotonic() + 8
            stopped = {}
            while time.monotonic() < stop_deadline:
                stopped = native.request('model_calibration_status')
                if stopped['phase'] in ('cancelled', 'complete', 'failed'):
                    break
                time.sleep(.02)
            assert stopped.get('phase') == 'cancelled', stopped
            assert not stopped.get('gain_available', False), stopped
            checks.append('real_worker_user_stop_exits_without_promoting_unmeasured_results')
            checks.append('historical_accuracy_fields_ignored_in_real_start_without_rejection_or_automatic_scoring')

            native.request('deactivate')
            original_meta = json.loads((recorded / 'metadata.json').read_text())
            candidate, selected = application_fixture(recorded, original_meta, before, names)
            applied = native.request('candidate_gain_apply', {'path': str(candidate), 'gain_purposes': ['servo_tracking']})
            assert applied['gain_purposes'] == ['servo_tracking'], applied
            after = native.request('get_admittance')['execution_gains']
            assert after['servo_tracking'] == selected, after
            assert {k: v for k, v in before.items() if k != 'servo_tracking'} == {
                k: v for k, v in after.items() if k != 'servo_tracking'}, after
            checks.append('available_warning_single_servo_application_preserves_other_five_gain_groups')
            checks.append('complete_tracking_parameters_apply_with_missing_disturbance_and_historical_aggregate_flags')
            native.request('candidate_gain_restore')
            assert native.request('get_admittance')['execution_gains'] == before
            checks.append('candidate_restore_recovers_complete_original_gain_state')
            return {'status': 'passed', 'checks': checks, 'quick_plan': quick, 'fine_plan': fine,
                    'scope': 'real_native_mock_IPC_with_synthetic_records_no_hardware_acceptance'}
        except Exception:
            if native:
                print(native.diagnostics(), file=sys.stderr)
            raise
        finally:
            if native:
                native.close()


def completed_taught_path_probes(terminal: Path, old_gain_scale: float) -> dict:
    """Finish actual moving path adaptation and frozen reference recording."""
    from check_gain_taught_path_flow import run_case
    return run_case(terminal, kind='complete', old_gain_scale=old_gain_scale)


def budget_limited_exit(terminal: Path) -> dict:
    """A tiny synthetic T must not start a six-second noise trial or pass."""
    native = None
    with tempfile.TemporaryDirectory(prefix='serial-arm-adaptive-budget-') as temporary:
        directory = Path(temporary)
        config, names = prepare_config(directory)
        storage = directory / 'storage'
        storage.mkdir()
        try:
            native = Native(terminal, config, storage)
            native.request('activate')
            native.ready()
            source = directory / 'short-source'
            synthetic_record(native, source, names, .2)
            native.ready()
            imported = native.request('model_calibration_import_trajectory', {'directory': str(source), 'pose_budget': 5})
            native.ready()
            started = time.monotonic()
            plan=native.request('model_calibration_gain_plan', {'gain_mode':'all','gain_tuning_mode':'quick'})
            assert not plan['valid'] and not plan['fits'], plan
            try:
                native.request('model_calibration_start', {
                    'task_id': imported['task_id'], 'workflow': 'gain', 'gain_mode': 'all',
                    'gain_tuning_mode': 'quick', 'released': True, 'gain_trajectory_confirmed': True,
                    'gain_motion_range_confirmed': True})
            except RuntimeError as error:
                assert 'time_budget_insufficient_for_complete_measurement' in str(error), str(error)
            else:
                raise AssertionError('An unfinishable four-use plan was started')
            status=native.request('model_calibration_status')
            assert status['phase']=='waiting_replay_confirmation' and not status['gain_available'], status
            assert not (Path(imported['directory'])/'gain-candidate.json').exists(), status
            return {'status': 'passed', 'recorded_T_s': .2, 'hard_limit_s': plan['hard_limit_s'],
                    'actual_elapsed_s': time.monotonic()-started, 'available': False, 'validated': False,
                    'scope': 'synthetic_short_budget_refuses_unfinishable_measurements_before_start'}
        finally:
            if native:
                native.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terminal', type=Path, required=True, help='Freshly built serial_arm_terminal executable')
    parser.add_argument('--full-probes', action='store_true', help='Complete moving taught-path adaptation and frozen records with two old gain configurations')
    args = parser.parse_args()
    terminal = args.terminal.resolve()
    if not terminal.is_file() or not os.access(terminal, os.X_OK):
        parser.error('--terminal must be an executable file')
    try:
        report = run(terminal)
        report['budget_limited_exit'] = budget_limited_exit(terminal)
        report['checks'].append('real_worker_refuses_unfinishable_trial_and_budget_exit_cannot_pass')
        if args.full_probes:
            probes = [completed_taught_path_probes(terminal, scale) for scale in (1, .25)]
            assert probes[0]['old_servo_gains'] != probes[1]['old_servo_gains']
            assert probes[0]['seed_source'] == probes[1]['seed_source']
            assert probes[0]['seed_kp'] == probes[1]['seed_kp']
            assert probes[0]['seed_kd'] == probes[1]['seed_kd']
            assert probes[0]['recorded_T_s'] == probes[1]['recorded_T_s'] == 25
            report['checks'].extend(['completed_real_native_moving_adaptation_frozen_path_and_disturbance_have_full_cycle_evidence',
                                     'online_models_record_applied_gain_updates_or_concrete_insufficient_evidence',
                                     'torque_insensitive_mock_reports_insufficient_disturbance_evidence_and_reference_only_results',
                                     'different_old_gains_produce_identical_native_model_seed_and_T'])
            report['completed_taught_path_probes'] = probes
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    except (AssertionError, RuntimeError, OSError, ValueError, queue.Empty) as error:
        print(f'Adaptive gain flow check failed: {error}', file=sys.stderr)
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
