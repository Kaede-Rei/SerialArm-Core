#!/usr/bin/env python3
"""Check explicit parking and desktop exit policy without hardware access."""
import argparse
from contextlib import redirect_stdout
from copy import deepcopy
import importlib.util
import io
import json
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
from bridge import parking_configured, prepare_normal_disconnect, verify_machine_admission

spec = importlib.util.spec_from_file_location('parking_python_terminal', ROOT / 'src/serial_arm/core/app/serial_arm_terminal.py')
terminal_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(terminal_module)


def readiness():
    return {'control_available': True, 'configuration_summary': {'parking_by_path': {'MIT': {'enabled': True, 'configured': True, 'validated': True, 'strategy': 'RIGID'}}},
            'modes_by_path': {'MIT': {'RIGID': 'configured'}},
            'commissioning': {'groups': [{'steps': [{'id': 'parking_validation',
                'control_path': 'MIT', 'completed': True}]}]}}


class NativeStub:
    def __init__(self, state='ACTIVE', path='MIT', park_result='INACTIVE'):
        self.state, self.calls, self.path, self.park_result = state, [], path, park_result

    def request(self, method, *args, **kwargs):
        self.calls.append(method)
        if method == 'status':
            return {'robot_state': self.state, 'control_path': self.path,
                    'valid': True, 'feedback_age_ms': 0., 'sequence': 1}
        if method == 'park':
            self.state = self.park_result
            return True
        raise AssertionError(method)


class ExitPolicy(unittest.TestCase):
    def test_explicit_configuration_is_separate_from_acceptance(self):
        ready = readiness()
        ready['configuration_summary']['parking_by_path']['MIT']['validated'] = False
        self.assertTrue(parking_configured(ready))
        ready['configuration_summary']['parking_by_path']['MIT']['enabled'] = False
        self.assertFalse(parking_configured(ready))

    def test_disabled_or_unconfigured_parking_does_not_move(self):
        for mutate in (lambda r: r['configuration_summary']['parking_by_path']['MIT'].update(enabled=False),
                       lambda r: r['configuration_summary']['parking_by_path']['MIT'].update(configured=False),
                       lambda r: r.update(control_available=False),
                       lambda r: r['configuration_summary']['parking_by_path'].clear()):
            ready = readiness()
            mutate(ready)
            native = NativeStub()
            with self.assertRaisesRegex(ValueError, '请先显式失能'):
                prepare_normal_disconnect(native, ready)
            self.assertEqual(native.calls, ['status'])

    def test_enabled_parking_precedes_disconnect_without_an_extra_receipt(self):
        ready = readiness()
        ready['configuration_summary']['parking_by_path']['MIT']['validated'] = False
        native = NativeStub()
        prepare_normal_disconnect(native, ready)
        self.assertEqual(native.calls, ['status', 'park', 'status'])

    def test_unfinished_parking_does_not_allow_disconnect(self):
        for state in ('ACTIVE', 'FAULT'):
            native = NativeStub(park_result=state)
            with self.assertRaisesRegex(ValueError, '未确认完成'):
                prepare_normal_disconnect(native, readiness())
            self.assertEqual(native.calls, ['status', 'park', 'status'])

    def test_fault_or_unknown_disconnect_keeps_connection(self):
        for state in ('FAULT', 'OFFLINE', None):
            native = NativeStub(state=state)
            with self.assertRaisesRegex(ValueError, '请先显式失能'):
                prepare_normal_disconnect(native, readiness())
            self.assertEqual(native.calls, ['status'])

    def test_inactive_disconnect_needs_no_parking_plan(self):
        native = NativeStub(state='INACTIVE')
        prepare_normal_disconnect(native, {})
        self.assertEqual(native.calls, ['status'])

    def test_disconnect_uses_actual_path_permission(self):
        ready = readiness()
        ready['configuration_summary']['parking_by_path']['MIT']['enabled'] = False
        ready['configuration_summary']['parking_by_path']['CSP'] = {'enabled': True, 'configured': True}
        native = NativeStub(path='CSP')
        prepare_normal_disconnect(native, ready)
        self.assertEqual(native.calls, ['status', 'park', 'status'])
        native = NativeStub(path='MIT')
        with self.assertRaises(ValueError):
            prepare_normal_disconnect(native, ready)
        self.assertEqual(native.calls, ['status'])

    def test_explicit_park_uses_the_actual_path_permission(self):
        ready = readiness()
        ready['info'] = {'write_enabled': True}
        ready['configuration_summary']['parking_by_path']['MIT']['enabled'] = False
        ready['configuration_summary']['parking_by_path']['CSP'] = {
            'enabled': True, 'configured': True, 'strategy': 'RIGID'}

        class Admission:
            def _readiness(self, config): return ready

        native = NativeStub(path='CSP')
        verify_machine_admission(Admission(), {}, 'park', {}, native)
        self.assertEqual(native.calls, ['status'])
        native = NativeStub(path='MIT')
        with self.assertRaises(ValueError):
            verify_machine_admission(Admission(), {}, 'park', {}, native)
        self.assertEqual(native.calls, ['status'])

    def test_compliant_csp_parking_cannot_use_a_temporary_response_test(self):
        ready = readiness()
        ready['info'] = {'write_enabled': True}
        ready['configuration_summary']['parking_by_path']['CSP'] = {
            'enabled': True, 'configured': True, 'strategy': 'COMPLIANT_THEN_RIGID'}

        class Admission:
            def _readiness(self, config): return ready

        class Native:
            def request(self, method, *args, **kwargs):
                if method == 'status':
                    return {'robot_state': 'ACTIVE', 'control_path': 'CSP', 'valid': True,
                            'feedback_age_ms': 0., 'sequence': 1}
                if method == 'get_admittance':
                    return {'validation': {'observation_validated': True,
                            'response_validated': False, 'test_active': True}}
                raise AssertionError(method)

        with self.assertRaisesRegex(ValueError, '临时有限刚度'):
            verify_machine_admission(Admission(), {}, 'park', {}, Native())


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class SessionStub:
    def __init__(self, clock, *, stale=False, interrupted=False, delayed_reference=False, compliant_offset=False):
        self.clock, self.stale, self.interrupted = clock, stale, interrupted
        self.delayed_reference, self.compliant_offset = delayed_reference, compliant_offset
        self.state, self.running, self.sequence, self.calls = 'ACTIVE', True, 0, []
        self.mode, self.mode_changes, self.motion_id = 'RIGID', [], 0

    @property
    def snapshot(self):
        self.sequence += 0 if self.stale else 1
        linear = .002 if self.clock.now < .08 else .0005
        if self.compliant_offset:
            linear = .0015 if self.mode == 'COMPLIANT' else .0005
        if self.interrupted:
            self.state, self.running = 'FAULT', False
        return SimpleNamespace(valid=True, sequence=self.sequence, last_error='',
            motion=SimpleNamespace(id=self.motion_id, reference_complete=bool(self.motion_id) and
                                   (not self.delayed_reference or self.clock.now >= .14)),
            cycle=SimpleNamespace(interaction_mode=self.mode, joint_state=SimpleNamespace(pos=[.02, linear], vel=[0, 0])))

    def set_impedance_mode(self, mode):
        self.calls.append(('mode', mode))
        self.mode_changes.append((mode, self.clock.now))
        self.mode = mode

    def move_to(self, target, scale):
        self.calls.append(('move', list(target)))
        self.motion_id += 1
        return self.motion_id

    def hold_current(self):
        self.calls.append(('hold', self.clock.now))

    def stop(self):
        self.calls.append(('stop', self.clock.now))
        self.state, self.running = 'INACTIVE', False


class PythonParking(unittest.TestCase):
    def run_park(self, *, enabled=True, strategy='RIGID', path='MIT', verified=True, **options):
        clock = Clock()
        terminal = terminal_module.Terminal.__new__(terminal_module.Terminal)
        terminal.serial_arm_module = SimpleNamespace(RobotState=SimpleNamespace(ACTIVE='ACTIVE', INACTIVE='INACTIVE', FAULT='FAULT'),
            JointImpedanceMode=SimpleNamespace(RIGID='RIGID', COMPLIANT='COMPLIANT'),
            ParkingStrategy=SimpleNamespace(RIGID='RIGID', COMPLIANT_THEN_RIGID='COMPLIANT_THEN_RIGID'),
            ControlPath=SimpleNamespace(MIT='MIT', CSP='CSP'))
        terminal.joint_names = ['rotary', 'linear']
        parking = SimpleNamespace(enabled=enabled, strategy=strategy, park_pos=[0, 0],
            position_tolerance=[.03, .001], velocity_tolerance=[.05, .001],
            speed_scale=.1, settle_time_s=.04, timeout_s=.5)
        terminal.session = SessionStub(clock, **options)
        terminal.session.config = SimpleNamespace(runtime=SimpleNamespace(control_path=path),
            shutdown=SimpleNamespace(for_path=lambda actual: parking if actual == path else None),
            capability=SimpleNamespace(admittance=SimpleNamespace(validation=SimpleNamespace(
                observation_validated=verified, response_validated=verified))))
        with patch.object(terminal_module, 'time', clock), redirect_stdout(io.StringIO()):
            result = terminal.park_and_stop()
        return terminal, result

    def test_explicit_park_ignores_exit_switch_and_uses_each_joint_tolerance(self):
        terminal, result = self.run_park()
        self.assertTrue(result)
        self.assertEqual(terminal.session.calls[0], ('mode', 'RIGID'))
        self.assertGreaterEqual(next(time for name, time in terminal.session.calls if name == 'stop'), .12)

    def test_two_stages_preserve_goal_and_require_reference_then_measured_settling(self):
        terminal, result = self.run_park(strategy='COMPLIANT_THEN_RIGID', delayed_reference=True, compliant_offset=True)
        self.assertTrue(result)
        changes = terminal.session.mode_changes
        self.assertEqual([mode for mode, _ in changes], ['COMPLIANT', 'RIGID'])
        self.assertGreaterEqual(changes[1][1], .18)
        stopped = next(value for key, value in terminal.session.calls if key == 'stop')
        self.assertGreaterEqual(stopped - changes[1][1], .04)
        self.assertEqual([value for key, value in terminal.session.calls if key == 'move'], [[0, 0]])
        self.assertFalse(any(key == 'hold' for key, _ in terminal.session.calls))

    def test_disabled_actual_path_and_unvalidated_csp_do_not_start_or_fallback(self):
        for options in ({'enabled': False}, {'enabled': False, 'path': 'CSP'},
                        {'path': 'CSP', 'strategy': 'COMPLIANT_THEN_RIGID', 'verified': False}):
            terminal, result = self.run_park(**options)
            self.assertFalse(result)
            self.assertEqual(terminal.session.calls, [])

    def test_stale_feedback_times_out_and_interrupted_parking_never_reports_completion(self):
        for options in ({'stale': True}, {'interrupted': True}):
            terminal, result = self.run_park(**options)
            self.assertFalse(result)
            self.assertFalse(any(name == 'stop' for name, _ in terminal.session.calls))

    def test_ordinary_exit_does_not_stop_or_move_an_active_robot(self):
        terminal, _ = self.run_park(stale=True)
        before = deepcopy(terminal.session.calls)
        with redirect_stdout(io.StringIO()):
            self.assertFalse(terminal.safe_exit())
        self.assertEqual(terminal.session.calls, before)


def check_native(terminal):
    from check_adaptive_gain_flow import Native, prepare_config
    cases = [(path, strategy, enabled, accepted) for path in ('MIT', 'CSP')
             for strategy, enabled, accepted in (('RIGID', True, True), ('COMPLIANT_THEN_RIGID', True, True),
                                                  ('RIGID', False, True))]
    cases += [('CSP', 'COMPLIANT_THEN_RIGID', True, False)]
    for path, strategy, enabled, accepted in cases:
        with tempfile.TemporaryDirectory(prefix='serialarm-park-exit-') as temp:
            directory = Path(temp)
            config, names = prepare_config(directory)
            document = json.loads(config.read_text())
            document['control']['runtime']['control_path'] = path
            document['model']['gravity_scale'] = dict.fromkeys(names, 0)
            document['control']['runtime']['model_feedforward_mode'] = 'NONE'
            document['safety_policy']['max_tracking_error'] = dict.fromkeys(names, .5)
            # A stationary CSP fixture checks strategy permission without attributing
            # unmodeled Mock inertia to a real admittance response experiment
            goal = 0 if path == 'CSP' and strategy == 'COMPLIANT_THEN_RIGID' else .01
            document['shutdown']['paths'][path].update(enabled=enabled, strategy=strategy,
                settle_time_s=.03, timeout_s=2,
                park_pos=dict.fromkeys(names, goal) if enabled else {},
                position_tolerance=dict.fromkeys(names, .001), velocity_tolerance=dict.fromkeys(names, .05))
            document['shutdown']['paths']['CSP' if path == 'MIT' else 'MIT']['enabled'] = True
            if path == 'CSP':
                document['capability']['admittance']['validation'].update(
                    observation_validated=True, response_validated=accepted)
            config.write_text(json.dumps(document))
            native = Native(terminal, config, directory / 'storage')
            try:
                if not enabled:
                    try:
                        native.request('park')
                    except RuntimeError as error:
                        assert 'disabled' in str(error), error
                    else:
                        raise AssertionError('disabled INACTIVE parking was reported successful')
                native.request('activate')
                native.ready()
                try:
                    native.request('shutdown')
                except RuntimeError as error:
                    assert 'INACTIVE' in str(error), error
                else:
                    raise AssertionError('ACTIVE native shutdown was accepted')
                assert native.request('status')['robot_state'] == 'ACTIVE'
                if enabled and accepted:
                    assert native.request('park') is True
                    assert native.request('status')['robot_state'] == 'INACTIVE'
                    if strategy == 'COMPLIANT_THEN_RIGID':
                        assert '交接刚性' in native.diagnostics()
                else:
                    try:
                        native.request('park')
                    except RuntimeError as error:
                        assert ('disabled' if not enabled else 'ADMITTANCE_NOT_VALIDATED') in str(error), error
                    else:
                        raise AssertionError('disabled or unaccepted compliant parking was accepted')
                    status = native.request('status')
                    assert status['robot_state'] == 'ACTIVE' and status['impedance_mode'] == 'RIGID', status
                    native.request('deactivate')
                assert native.request('shutdown') is True
                native.process.wait(timeout=3)
                assert native.process.returncode == 0
            finally:
                native.close()
    print('PASS: both parking paths/strategies, disabled plans, CSP permission and INACTIVE-only exit')


def check_reference(core_build):
    build = core_build.resolve()
    link = shlex.split((build / 'CMakeFiles/serial_arm_terminal.dir/link.txt').read_text())
    with tempfile.TemporaryDirectory(prefix='serialarm-park-reference-') as temp:
        directory = Path(temp)
        obj, executable = directory / 'parking.o', directory / 'parking'
        includes = [ROOT / 'src/serial_arm/core/include', ROOT / 'src/serial_arm/core/third_lib/tl/include', Path('/usr/include/eigen3')]
        subprocess.run([link[0], '-std=c++17', '-O1', *[f'-I{path}' for path in includes], '-c',
                        str(ROOT / 'tools/park_reference_test.cpp'), '-o', str(obj)], check=True)
        index = next(i for i, value in enumerate(link) if value.endswith('serial_arm_terminal.cpp.o'))
        link[index] = str(obj)
        link[link.index('-o') + 1] = str(executable)
        subprocess.run(link, cwd=build, check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--terminal', type=Path)
    parser.add_argument('--core-build', type=Path)
    args = parser.parse_args()
    suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(test) for test in (ExitPolicy, PythonParking))
    if not unittest.TextTestRunner(verbosity=1).run(suite).wasSuccessful():
        raise SystemExit(1)
    if args.terminal:
        check_native(args.terminal.resolve())
    if args.core_build:
        check_reference(args.core_build)
