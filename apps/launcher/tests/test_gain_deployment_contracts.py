from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
import artifact_exports as exports


class GainDeploymentContracts(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.dest = self.root / 'artifact'; self.dest.mkdir()
        self.core = self.root / 'core.yaml'
        gains = dict(kp={'axis': 20.}, kd={'axis': 4.})
        self.cfg = dict(model={'joint_names': ['axis']}, control={'controller': {
            key: deepcopy(gains) for key in ('rigid_hold', 'rigid_tracking', 'compliant_hold',
                'compliant_tracking', 'compliant_drag', 'servo_tracking')},
            'hold_compensation': {'enabled': True, 'integral_gain': {'axis': .15},
                'max_torque': {'axis': .2}, 'max_rate': {'axis': .05},
                'contact_threshold': {'axis': .3}, 'velocity_threshold': .02, 'error_threshold': .03}})
        self.metadata = dict(task_id='task', urdf_fingerprint='model', joint_names=['axis'])
        self.candidate = dict(schema='serial_arm_gain_candidate', **self.metadata,
            category='rigid_tracking', validated=True, kp=[25.], kd=[5.],
            hold_gains=dict(kp=[30.], kd=[6.]), hold_verification={'validated': True})

    def export(self, candidate=None, *, flow=False):
        self.core.write_text('# retained comment\n' + yaml.safe_dump(self.cfg, default_flow_style=flow))
        (self.root / 'gain-candidate.json').write_text(json.dumps(candidate or self.candidate))
        with patch.object(exports, '_snapshot', return_value=(self.root, self.metadata,
                self.root / 'robot.urdf', self.core, self.cfg)):
            result = exports._gain_export({'directory': str(self.root)}, self.dest)
        self.assertTrue(result['validated'])
        output = (self.dest / 'core.yaml').read_text()
        self.assertTrue(output.startswith('# retained comment\n'))
        return yaml.safe_load(output)

    def test_baseline_candidate_disables_previous_compensation(self):
        for metadata in (None, {'enabled': False}):
            with self.subTest(metadata=metadata):
                candidate = deepcopy(self.candidate)
                if metadata is not None: candidate['hold_compensation'] = metadata
                result = self.export(candidate)
                self.assertFalse(result['control']['hold_compensation']['enabled'])
                self.assertEqual(result['control']['controller']['rigid_hold']['kp']['axis'], 30.)
                self.assertEqual(result['control']['hold_compensation']['max_torque']['axis'], .2)

    def test_verified_compensation_and_partial_config_deploy_as_one_state(self):
        candidate = deepcopy(self.candidate)
        candidate['hold_verification']['validated'] = False
        candidate['hold_compensation_verification'] = {'validated': True}
        candidate['hold_compensation'] = dict(enabled=True, integral_gain=[.1], max_torque=[.12],
            max_rate=[.02], contact_threshold=[.2], velocity_threshold=.01, error_threshold=.02)
        for flow in (False, True):
            self.cfg['control']['hold_compensation'] = {'enabled': False}
            result = self.export(candidate, flow=flow)
            hold = result['control']['hold_compensation']
            self.assertTrue(hold['enabled'])
            self.assertEqual(hold['max_torque'], {'axis': .12})
            self.assertEqual(hold['error_threshold'], .02)

    def test_compensation_state_requires_the_corresponding_verification(self):
        candidate = deepcopy(self.candidate)
        candidate['hold_compensation'] = {'enabled': True}
        with self.assertRaisesRegex(ValueError, '独立保持验收'): self.export(candidate)
        candidate['hold_compensation']['enabled'] = False
        candidate['hold_verification']['validated'] = False
        candidate['hold_compensation_verification'] = {'validated': True}
        with self.assertRaisesRegex(ValueError, '独立保持验收'): self.export(candidate)

    def test_standalone_compliant_uses_the_deployed_rigid_reference(self):
        candidate = {**self.candidate, 'category': 'compliant_tracking', 'kp': [8.], 'kd': [2.],
            'rigid_reference_kp': [20.], 'rigid_reference_kd': [4.],
            'compliant_separation_validated': True, 'tested_speed_fraction': .2}
        result = self.export(candidate)
        self.assertEqual(result['control']['controller']['compliant_tracking']['kp'], {'axis': 8.})
        candidate['rigid_reference_kp'] = [30.]
        with self.assertRaisesRegex(ValueError, '刚性参考不一致'): self.export(candidate)


if __name__ == '__main__':
    unittest.main()
