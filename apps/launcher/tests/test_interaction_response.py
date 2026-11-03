from pathlib import Path
import json
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from interaction_response import analyze_interaction_response, verify_return_response


def record():
    rng = np.random.default_rng(52)
    dt, n, delay = .01, 1600, 3
    u = np.repeat(rng.uniform(-.2, .2, 80), 20)
    y = np.zeros(n)
    radius, angle = np.exp(-4 * dt), 8 * dt
    a1, a2 = 2 * radius * np.cos(angle), -radius ** 2
    b = 1 - a1 - a2
    for k in range(delay + 2, n):
        y[k] = a1 * y[k - 1] + a2 * y[k - 2] + b * u[k - delay]
    return {'joint_names': ['joint1'], 'impedance_mode': 'RIGID_TRACKING',
            'samples': [{'time_s': k * dt, 'accepted': [float(u[k])], 'pos': [float(y[k])],
                         'external': [0], 'effort_limit': [10], 'torque_limited': False}
                        for k in range(n)]}


class InteractionResponse(unittest.TestCase):
    def test_identifies_known_delay_poles_and_held_out_response(self):
        result = analyze_interaction_response(record())
        joint = result['joints'][0]
        self.assertTrue(joint['identified'], joint['reason'])
        self.assertAlmostEqual(joint['delay_s'], .03, places=6)
        self.assertAlmostEqual(joint['natural_frequency_hz'], np.hypot(4, 8) / (2 * np.pi), places=5)
        self.assertAlmostEqual(joint['damping_ratio'], 4 / np.hypot(4, 8), places=5)
        self.assertAlmostEqual(joint['static_gain'], 1, places=6)
        self.assertLess(joint['validation_nrmse'], 1e-6)
        self.assertGreater(joint['bandwidth_hz'], 1)
        json.dumps(result, allow_nan=False)

    def test_static_or_saturated_windows_do_not_invent_bandwidth(self):
        for mode in ('static', 'saturated', 'contact', 'missing'):
            window = record()
            if mode == 'static':
                for sample in window['samples']: sample['accepted'] = [0]
            elif mode == 'saturated': window['samples'][10]['torque_limited'] = True
            elif mode == 'contact': window['samples'][10]['external'] = [1]
            else: window['samples'][10]['pos'] = [None]
            with self.subTest(mode=mode):
                joint = analyze_interaction_response(window)['joints'][0]
                self.assertFalse(joint['identified'])
                self.assertIsNone(joint['bandwidth_hz'])
                self.assertTrue(joint['reason'])

    def test_invalid_or_discontinuous_clock_is_rejected(self):
        for offset in (0, -.01, .5):
            window = record()
            window['samples'][30]['time_s'] = window['samples'][29]['time_s'] + offset
            self.assertFalse(analyze_interaction_response(window)['joints'][0]['identified'])

    def test_short_record_and_drag_are_reported_without_motion(self):
        window = record(); window['samples'] = window['samples'][:80]
        self.assertFalse(analyze_interaction_response(window)['joints'][0]['identified'])

    def test_return_to_original_target_keeps_external_measurement_independent(self):
        window = {'joint_names': ['a'], 'release_time_s': .5, 'summary': {'tolerance': .001},
                  'samples': [dict(time_s=k * .01, nominal=[.2], pos=[.2 + (.02 if k < 75 else .0002)],
                                   new_measurement=True) for k in range(151)]}
        value = verify_return_response(window)
        self.assertTrue(value['joint_pass'])
        self.assertIsNone(value['external_tcp_pass'])
        window['external_tcp_measurement'] = dict(source='manual_external_measurement',
                                                  displacement_m=[.005, 0, 0], tolerance_m=.001)
        value = verify_return_response(window)
        self.assertTrue(value['joint_pass'])
        self.assertFalse(value['external_tcp_pass'])
        window['samples'][-1]['nominal'] = [.205]
        self.assertFalse(verify_return_response(window)['joint_pass'])
        self.assertFalse(verify_return_response(window)['target_unchanged'])

    def test_return_requires_release_dwell_and_rejects_fk_as_external_evidence(self):
        self.assertIsNone(verify_return_response(record())['joint_pass'])
        window = record()
        window['external_tcp_measurement'] = dict(source='forward_kinematics',
                                                  displacement_m=[0, 0, 0], tolerance_m=.001)
        with self.assertRaises(ValueError): verify_return_response(window)
        window = record(); window['impedance_mode'] = 'COMPLIANT_DRAG'
        self.assertFalse(analyze_interaction_response(window)['joints'][0]['identified'])

    def test_return_uses_each_joint_unit_and_tolerance(self):
        window = dict(joint_names=['rotation', 'slide'], joint_units=['rad', 'm'],
                      release_time_s=.2, return_tolerance=[.002, .0005],
                      samples=[dict(time_s=k * .01, nominal=[.1, .02], pos=[.101, .0208],
                                    new_measurement=True) for k in range(101)])
        result = verify_return_response(window)
        self.assertFalse(result['joint_pass'])
        self.assertIsNone(result['maximum_error_rad'])
        self.assertTrue(result['joint_errors'][0]['passed'])
        self.assertFalse(result['joint_errors'][1]['passed'])
        self.assertEqual(result['joint_errors'][1]['unit'], 'm')
        window = record()
        for sample in window['samples']: sample['new_measurement'] = False
        self.assertFalse(analyze_interaction_response(window)['joints'][0]['identified'])


if __name__ == '__main__':
    unittest.main()
