"""Regression coverage for inertia-less tool frames and unsampled URDF branches"""
import json
import pathlib
import sys
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'backend'))
from full_inertial import _articulated_mapping, _fit_regressor, _sampling_map, export_full_inertial_candidate
from persistence import export_candidate_urdf
from model_calibration_source import recorded_file_fingerprint


class Joint:
    def __init__(self, idx):
        self.nq = self.nv = 1
        self.idx_q = self.idx_v = idx


class Inertia:
    def __init__(self, mass):
        self.mass = float(mass)
        self.lever = np.zeros(3)
        self.inertia = np.eye(3)

    def toDynamicParameters(self):
        return np.array([self.mass, 0, 0, 0, 1, 0, 1, 0, 0, 1], dtype=float)


class Model:
    def __init__(self):
        # Deliberately permuted order: sample joint1, joint2 but not joint7
        self.names = ['universe', 'joint1', 'joint7', 'joint2']
        self.njoints = len(self.names)
        self.nq = self.nv = 3
        self.joints = [None, Joint(0), Joint(1), Joint(2)]
        self.inertias = [None, Inertia(2), Inertia(3), Inertia(4)]

    def createData(self):
        return object()


ROBOT = '''<robot name="test">
<link name="base"/>
<link name="link1"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
<link name="link2"><inertial><origin xyz="0 0 0"/><mass value="4"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
<link name="tool0"/>
<link name="link7"><inertial><origin xyz="0 0 0"/><mass value="3"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
<joint name="joint1" type="revolute"><parent link="base"/><child link="link1"/></joint>
<joint name="joint2" type="revolute"><parent link="link1"/><child link="link2"/></joint>
<joint name="tool0_joint" type="fixed"><parent link="link2"/><child link="tool0"/></joint>
<joint name="joint7" type="revolute"><parent link="link2"/><child link="link7"/></joint>
</robot>'''


class Tool0CandidateTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.task = pathlib.Path(self.tmp.name) / 'task'
        source = self.task / 'source'
        source.mkdir(parents=True)
        self.urdf = source / 'model.urdf'
        self.urdf.write_text(ROBOT)
        self.metadata = {'urdf_path': str(self.urdf), 'urdf_fingerprint': recorded_file_fingerprint(self.urdf),
                         'joint_names': ['joint1', 'joint2'], 'task_id': 'sample'}

    def test_gravity_candidate_skips_inertia_less_tool_frame(self):
        payload = {'first_moments': {'link1': [0.2, 0, 0], 'tool0': [0, 0, 0]},
                   'static_pass': False, 'candidate_quality_reason': 'unvalidated'}
        with patch('persistence.build_gravity_correction_payload',
                   return_value=(self.task, self.metadata, {'gravity_result': {}}, payload)):
            result = export_candidate_urdf(self.task)
        tree = ET.parse(result['candidate_urdf']).getroot()
        self.assertEqual(result['skipped_massless_links'], ['tool0'])
        self.assertIsNone(tree.find("./link[@name='tool0']/inertial"))
        self.assertEqual(tree.find("./link[@name='link1']/inertial/origin").get('xyz'), '0.1 0 0')
        self.assertEqual(tree.find("./link[@name='link7']/inertial/mass").get('value'), '3')

    def test_nonzero_tool_frame_candidate_is_reported_not_silently_written(self):
        payload = {'first_moments': {'link1': [0.2, 0, 0], 'tool0': [1.2, 0, 0]},
                   'static_pass': False, 'candidate_quality_reason': 'unvalidated'}
        with patch('persistence.build_gravity_correction_payload',
                   return_value=(self.task, self.metadata, {'gravity_result': {}}, payload)):
            result = export_candidate_urdf(self.task)
        self.assertEqual(len(result['review_warnings']), 1)
        self.assertIn('tool0', result['review_warnings'][0])

    def test_mapping_of_measured_subset_ignores_massless_tool_and_preserves_extra_joint(self):
        mapped, unchanged = _articulated_mapping(ET.fromstring(ROBOT), Model(), ['joint1', 'joint2'])
        self.assertEqual([(i, name) for i, name, _ in mapped], [(1, 'link1'), (3, 'link2')])
        self.assertEqual(unchanged, ['link7'])
        self.assertEqual(_sampling_map(Model(), mapped), ([0, 2], [0, 2],
                         list(range(0, 10)) + list(range(20, 30))))

    def test_unsampled_massive_fixed_attachment_is_rejected(self):
        source = ROBOT.replace('<link name="tool0"/>',
                               '<link name="tool0"><inertial><mass value="2"/></inertial></link>')
        with self.assertRaisesRegex(ValueError, '固定附属'):
            _articulated_mapping(ET.fromstring(source), Model(), ['joint1', 'joint2'])

    def test_regressor_keeps_auxiliary_contribution_as_fixed_prior(self):
        model = Model()
        mapped, _ = _articulated_mapping(ET.fromstring(ROBOT), model, ['joint1', 'joint2'])
        calls = []
        def regression(model, data, q, v, a):
            calls.append((q.copy(),v.copy(),a.copy()))
            Y = np.zeros((3, 30))
            Y[0, 0] = 1 + q[0]
            Y[2, 20] = 1 + q[2]
            Y[0, 10] = 0.25  # joint7 inertia contributes to sampled joint1 torque
            Y[2, 10] = 0.5
            return Y
        fakepin = types.SimpleNamespace(neutral=lambda m: np.zeros(m.nq), computeJointTorqueRegressor=regression)
        samples = []
        for direction in ('static_reverse','friction_forward_fast'):
            for i in range(80):
                q=np.array([i/100.0,-i/110.0]);vel=np.array([0.2,0.15]);acc=np.array([0.01,0.01])
                Y=regression(model, None, np.array([q[0],0.0,q[1]]),np.array([vel[0],0,vel[1]]),np.array([acc[0],0,acc[1]]))
                tau=(Y@np.concatenate([it.toDynamicParameters() for it in model.inertias[1:]]))[[0,2]]
                samples.append((direction,q,vel,acc,tau))
        result = _fit_regressor(samples, model, fakepin, mapped)
        self.assertEqual(result['candidate'].shape, (20,))
        self.assertEqual(result['degrees'], 20)
        self.assertTrue(np.isfinite(result['candidate']).all())
        self.assertTrue(all(q[1] == 0 and v[1] == 0 and a[1] == 0 for q,v,a in calls))
        # Source auxiliary mass stays outside fitted parameters
        self.assertEqual(result['full_model_prior'][10], 3)

    def test_full_inertial_export_keeps_tool0_and_auxiliary_joint_unchanged(self):
        self.task.joinpath('metadata.json').write_text(json.dumps(self.metadata))
        self.task.joinpath('result.json').write_text(json.dumps({'task_id':'sample'}))
        self.task.joinpath('frames.csv').write_text('not used')
        model = Model()
        fakepin = types.SimpleNamespace(buildModelFromUrdf=lambda path: model)
        values = np.concatenate([model.inertias[1].toDynamicParameters(), model.inertias[3].toDynamicParameters()])
        fit = {'candidate':values,'rank':7,'degrees':20}
        review = ((values, 1.0, 1.0), 1.2, [])
        rows = [('static_reverse', None, None, None, None)] * 120
        with patch.dict(sys.modules, {'pinocchio': fakepin}), \
             patch('full_inertial._extract_rows', return_value=rows), \
             patch('full_inertial._fit_regressor', return_value=fit), \
             patch('full_inertial._review_candidate', return_value=review):
            report = export_full_inertial_candidate(self.task)
        root = ET.parse(report['candidate_urdf']).getroot()
        self.assertIsNone(root.find("./link[@name='tool0']/inertial"))
        self.assertEqual(root.find("./link[@name='link7']/inertial/mass").get('value'), '3')
        self.assertEqual(report['unmeasured_joints_assumed_neutral'], ['joint7'])
        self.assertIn('joint7', ' '.join(report['review_warnings']))
        self.assertFalse(report['all_parameters_identifiable'])


if __name__ == '__main__':
    unittest.main()
