from pathlib import Path
import io
import json
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from bridge import verify_machine_admission
import bridge


class Library:
    def _readiness(self, config):
        return {'info': {'write_enabled': True}, 'control_available': True,
                'modes': {'RIGID_HOLD': 'verified', 'RIGID_TRACKING': 'verified'}}


class Native:
    def __init__(self, state='INACTIVE', enabled=True):
        self.status = {'robot_state': state, 'valid': True, 'sequence': 1,
                       'feedback_age_ms': 2, 'impedance_mode': 'RIGID_HOLD',
                       'reference_behavior': 'MODEL_ADMITTANCE'}
        self.enabled = enabled
    def request(self, method, params, **kwargs):
        return self.status if method == 'status' else {'enabled': self.enabled}


class ReferenceAdmission(unittest.TestCase):
    def check(self, native, method, params=None):
        return verify_machine_admission(Library(), {}, method, params or {}, native)

    def test_strategy_change_does_not_bypass_enabled_or_park_requirement(self):
        self.check(Native(), 'set_reference_behavior', {'behavior': 'MODEL_ADMITTANCE'})
        self.check(Native(enabled=False), 'set_reference_behavior', {'behavior': 'NATIVE_IMPEDANCE'})
        with self.assertRaisesRegex(ValueError, '未启用'):
            self.check(Native(enabled=False), 'set_reference_behavior', {'behavior': 'MODEL_ADMITTANCE'})
        with self.assertRaisesRegex(ValueError, '停放'):
            self.check(Native('ACTIVE'), 'set_reference_behavior', {'behavior': 'NATIVE_IMPEDANCE'})
        with self.assertRaisesRegex(ValueError, '参考策略'):
            self.check(Native(), 'set_reference_behavior', {'behavior': 'implicit'})

    def test_tuning_is_parked_or_explicit_active_admittance(self):
        parked = Native(); parked.status['valid'] = False
        self.check(parked, 'set_admittance', {'stiffness': [0]})
        active = Native('ACTIVE'); self.check(active, 'set_admittance', {'damping': [0]})
        active.status['reference_behavior'] = 'NATIVE_IMPEDANCE'
        with self.assertRaisesRegex(ValueError, 'MODEL_ADMITTANCE'):
            self.check(active, 'set_admittance')

    def test_active_stale_feedback_drag_and_calibration_still_reject_tuning(self):
        for key, value in [('feedback_age_ms', 500), ('impedance_mode', 'COMPLIANT_DRAG'),
                           ('model_calibration', {'phase': 'replaying'})]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                native = Native('ACTIVE'); native.status[key] = value
                self.check(native, 'set_admittance')

    def test_strategy_change_during_task_is_rejected(self):
        native = Native(); native.status['teach_replay'] = {'phase': 'recording'}
        with self.assertRaisesRegex(ValueError, '任务'):
            self.check(native, 'set_reference_behavior', {'behavior': 'NATIVE_IMPEDANCE'})

    def test_replay_uses_the_requested_gain_purpose_and_strategy(self):
        library = mock.Mock()
        library._readiness.return_value = {'info': {'write_enabled': True}, 'control_available': True,
            'modes': {'SERVO_TRACKING': 'configured', 'COMPLIANT_DRAG': 'configured'}}
        native = Native('ACTIVE')
        verify_machine_admission(library, {}, 'teach_replay_play', dict(mode='RIGID_TRACKING',
            gain_profile='SERVO_TRACKING', reference_behavior='NATIVE_IMPEDANCE'), native)
        verify_machine_admission(library, {}, 'teach_replay_play', dict(mode='RIGID_TRACKING',
            gain_profile='SERVO_TRACKING', reference_behavior='MODEL_ADMITTANCE'), native)
        for purpose, behavior in [('RIGID', 'MODEL_ADMITTANCE'), ('DRAG', 'NATIVE_IMPEDANCE'),
                                  ('', 'NATIVE_IMPEDANCE'), ('SERVO_TRACKING', '')]:
            with self.subTest(purpose=purpose, behavior=behavior), self.assertRaises(ValueError):
                verify_machine_admission(library, {}, 'teach_replay_play', dict(mode='RIGID_TRACKING',
                    gain_profile=purpose, reference_behavior=behavior), native)
        with self.assertRaisesRegex(ValueError, '启用'):
            verify_machine_admission(library, {}, 'teach_replay_play', dict(mode='RIGID_TRACKING',
                gain_profile='SERVO_TRACKING', reference_behavior='MODEL_ADMITTANCE'), Native('ACTIVE', enabled=False))

    def test_drag_preparation_accepts_full_id_without_bypassing_gain_and_feedback(self):
        library = mock.Mock()
        library._readiness.return_value = {'info': {'write_enabled': True}, 'control_available': True,
            'modes': {'COMPLIANT_DRAG': 'configured'}}
        native = Native('ACTIVE')
        native.status.update(model_feedforward_mode='FULL_INVERSE_DYNAMICS', gravity_scale=[1.])
        verify_machine_admission(library, {}, 'teach_replay_record_begin', {}, native)
        verify_machine_admission(library, {}, 'set_impedance_mode',
                                 {'mode': 'COMPLIANT_DRAG', 'gain_profile': 'DRAG'}, native)
        native.status['feedback_age_ms'] = 300
        with self.assertRaisesRegex(ValueError, '真实反馈'):
            verify_machine_admission(library, {}, 'teach_replay_record_begin', {}, native)

    def test_servo_motion_uses_servo_readiness_instead_of_rigid_gain_readiness(self):
        library = mock.Mock()
        library._readiness.return_value = {'info': {'write_enabled': True}, 'control_available': True,
            'modes': {'SERVO_TRACKING': 'configured'}}
        native = Native('ACTIVE')
        native.status.update(gain_profile='SERVO_TRACKING', joint={'pos': [0.]})
        verify_machine_admission(library, {}, 'move_to', {'positions': [.1]}, native)
        library._readiness.return_value['modes']['SERVO_TRACKING'] = 'unconfigured'
        with self.assertRaisesRegex(ValueError, 'SERVO_TRACKING'):
            verify_machine_admission(library, {}, 'move_to', {'positions': [.1]}, native)

    def test_large_response_export_preserves_samples_and_analysis_without_unrelated_session_data(self):
        class Stub:
            def __init__(self, *args):
                pass
            def close(self):
                pass
            def status(self):
                return {'state': 'idle'}
            def telemetry(self):
                return [{'event': 'telemetry', 'data': {'wrong_session': True}}]
        names = ['a', 'b', 'c', 'd', 'e', 'f']
        zero = [0.] * len(names)
        keys = ('pos', 'vel', 'acc', 'nominal', 'nominal_vel', 'nominal_acc',
                'virtual', 'virtual_vel', 'virtual_acc', 'accepted', 'accepted_vel', 'accepted_acc',
                'measured_torque', 'pd_torque', 'model_feedforward', 'predicted_total_torque',
                'effort_limit', 'kp', 'kd', 'external')
        samples = [dict(time_s=i * .05, sequence=i, observer_valid=True, new_measurement=True,
                        reference_limited=False, torque_limited=False,
                        **{key: zero[:] for key in keys}) for i in range(1500)]
        window = dict(recording=False, joint_names=names, source='passive_native_cycle_snapshots',
                      release_time_s=None, samples=samples, session_id='source-session', impedance_mode='RIGID_TRACKING')
        export = json.dumps({'id': 1, 'method': 'workbench_export', 'params': {'profile': 'source', 'response_window': window}}) + '\n'
        ordinary = json.dumps({'id': 2, 'method': 'status', 'params': {'padding': 'x' * (1024 * 1024)}}) + '\n'
        self.assertGreater(len(export), 1024 * 1024)
        messages = []
        with tempfile.TemporaryDirectory() as folder, mock.patch.multiple(
                bridge, ROOT=Path(folder), Inspector=Stub, Supervisor=Stub, NativeSession=Stub,
                ModelRuntime=Stub, ProfileLibrary=Stub, TcpCalibration=Stub, emit=messages.append), \
                mock.patch.object(sys, 'stdin', io.StringIO(export + ordinary)), \
                mock.patch.object(sys, 'stderr', io.StringIO()), mock.patch.object(bridge.signal, 'signal'):
            bridge.main()
            result = next(item['result'] for item in messages if item.get('id') == 1)
            self.assertEqual(result['samples'], len(samples))
            self.assertEqual(result['interaction_response']['sample_count'], len(samples))
            self.assertTrue(all(not item['identified'] for item in result['interaction_response']['joints']))
            lines = Path(result['path']).read_text().splitlines()
            metadata = json.loads(lines[0])
            self.assertEqual(metadata['response_window']['session_id'], 'source-session')
            self.assertNotIn('samples', metadata['response_window'])
            self.assertIn('execution_dynamics', metadata['response_window'])
            self.assertEqual(json.loads(lines[1])['event'], 'response_snapshot')
            self.assertNotIn('wrong_session', ''.join(lines))
            rejected = next(item['error'] for item in messages if item.get('id') == 2)
            self.assertIn('request too large', rejected['technical'])


if __name__ == '__main__':
    unittest.main()
