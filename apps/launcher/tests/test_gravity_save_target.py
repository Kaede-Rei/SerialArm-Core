"""Historical calibration must match the selected deployment configuration."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from model_calibration_source import recorded_file_fingerprint
from persistence import preview_gravity_correction, save_gravity_correction, sha256_file


class GravitySaveTargetTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.task = self.root / 'task'
        self.task.mkdir()
        self.model = self.root / 'source.urdf'
        self.model.write_text('''<robot name="fixture"><link name="base"/>
<link name="tip"><inertial><origin xyz="0.1 0 0"/><mass value="1"/>
<inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
</inertial></link><joint name="axis" type="revolute"><parent link="base"/>
<child link="tip"/><axis xyz="0 1 0"/>
<limit lower="-1" upper="1" effort="10" velocity="1"/></joint></robot>''')
        self.metadata = dict(task_id='source-task', joint_names=['axis'],
                             core_fingerprint='source-core', urdf_path=str(self.model),
                             urdf_fingerprint=recorded_file_fingerprint(self.model))
        gravity = dict(static_pass=True, torque_bias=[0.],
                       torque_bias_convention='model_minus_measured',
                       first_moments=[dict(link_name='tip', value=[.12, 0., 0.])],
                       validation_candidate=dict(p99=[.01]), noise_rms=[.001])
        result = dict(task_id='source-task', phase='complete', joint_names=['axis'],
                      core_fingerprint='source-core', urdf_fingerprint=self.metadata['urdf_fingerprint'],
                      gravity_result=gravity, friction_pass=False)
        self.write_metadata()
        (self.task / 'result.json').write_text(json.dumps(result))
        (self.task / 'frames.csv').write_text('monotonic_ns,phase,valid\n')
        (self.task / 'trajectory.csv').write_text('monotonic_ns,q:axis\n')
        self.core = self.root / 'selected.yaml'
        self.core.write_text('model:\n  urdf_path: source.urdf\n  joint_names: [axis]\n'
                             'control:\n  retained_setting: keep\n')

    def write_metadata(self):
        (self.task / 'metadata.json').write_text(json.dumps(self.metadata))

    def assert_rejected_without_writes(self):
        before = self.core.read_bytes()
        for action in (lambda: preview_gravity_correction(self.core, self.task),
                       lambda: save_gravity_correction(self.core, sha256_file(self.core), self.task)):
            with self.assertRaises(ValueError):
                action()
            self.assertEqual(self.core.read_bytes(), before)
            self.assertFalse((self.task / 'candidate').exists())
            self.assertFalse((self.task / 'application.json').exists())
            self.assertEqual(list(self.root.glob('*.bak-*')), [])

    def test_other_model_is_rejected_even_when_original_model_still_matches(self):
        other = self.root / 'other.urdf'
        other.write_text(self.model.read_text().replace('value="1"', 'value="2"'))
        self.core.write_text(self.core.read_text().replace('source.urdf', 'other.urdf'))
        self.assert_rejected_without_writes()

    def test_wrong_joint_mapping_is_rejected(self):
        self.core.write_text(self.core.read_text().replace('[axis]', '[other]'))
        self.assert_rejected_without_writes()

    def test_reordered_joints_are_rejected(self):
        self.metadata['joint_names'] = ['axis', 'other']
        self.write_metadata()
        self.core.write_text(self.core.read_text().replace('[axis]', '[other, axis]'))
        self.assert_rejected_without_writes()

    def test_missing_selected_model_is_rejected(self):
        self.core.write_text(self.core.read_text().replace('source.urdf', 'missing.urdf'))
        self.assert_rejected_without_writes()

    def test_relocated_matching_model_can_be_saved_and_preserves_other_settings(self):
        relocated = self.root / 'relocated'
        relocated.mkdir()
        (relocated / 'robot.urdf').write_bytes(self.model.read_bytes())
        self.core.write_text(self.core.read_text().replace('source.urdf', 'relocated/robot.urdf'))
        self.model.unlink()
        preview = preview_gravity_correction(self.core, self.task)
        saved = save_gravity_correction(self.core, preview['sha256'], self.task)
        config = yaml.safe_load(self.core.read_text())
        correction = yaml.safe_load(Path(saved['gravity_correction_path']).read_text())
        self.assertEqual(config['control'], {'retained_setting': 'keep'})
        self.assertEqual(config['model']['urdf_path'], 'relocated/robot.urdf')
        self.assertEqual(correction['urdf_fingerprint'], recorded_file_fingerprint(relocated / 'robot.urdf'))
        self.assertEqual((self.core.parent / config['model']['gravity_correction_path']).resolve(),
                         Path(saved['gravity_correction_path']))

    def test_absolute_matching_model_can_be_saved(self):
        self.core.write_text(self.core.read_text().replace('source.urdf', str(self.model)))
        preview = preview_gravity_correction(self.core, self.task)
        self.assertTrue(save_gravity_correction(self.core, preview['sha256'], self.task))


if __name__ == '__main__':
    unittest.main()
