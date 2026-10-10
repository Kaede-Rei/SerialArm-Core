"""Local calibration provenance and friction source safety regressions"""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from model_calibration_source import recorded_file_fingerprint
from persistence import (build_gravity_correction_payload, export_candidate_urdf,
                         export_friction_candidate, load_model_calibration_summary)


class LocalProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.task = Path(self.temp.name) / 'dataset'
        (self.task / 'source').mkdir(parents=True)
        urdf = '''<robot name="example"><link name="base_link"/>
        <link name="Link2"><inertial><origin xyz="0.1 0 0"/><mass value="1"/>
        <inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial></link>
        <link name="Link6"><inertial><origin xyz="0.01 0.02 0"/><mass value="0.8"/>
        <inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial></link>
        <link name="tool0"/>
        <joint name="joint2" type="revolute"><parent link="base_link"/><child link="Link2"/></joint>
        <joint name="joint6" type="revolute"><parent link="Link2"/><child link="Link6"/></joint>
        <joint name="tool0_joint" type="fixed"><parent link="Link6"/><child link="tool0"/></joint></robot>'''
        source = self.task / 'source/model.urdf'
        source.write_text(urdf)
        self.metadata = {'task_id': 'sample', 'urdf_path': str(source),
                         'urdf_fingerprint': recorded_file_fingerprint(source),
                         'core_fingerprint': 'config-test', 'joint_names': ['joint2','joint6']}
        (self.task / 'metadata.json').write_text(json.dumps(self.metadata))
        original = {'status': 'passed', 'static_pass': True, 'numerical_rank': 2,
                    'first_moments': [{'link_name': 'Link2','value':[.1,0,0]},
                                      {'link_name':'Link6','value':[.008,.016,0]}],
                    'torque_bias':[.01,-.01],
                    'noise_rms':[.005,.005],
                    'validation_candidate':{'p99':[.02,.02]}}
        self.result = {'task_id':'sample', 'phase':'complete', 'joint_names':['joint2','joint6'],
                       'core_fingerprint':'config-test', 'urdf_fingerprint':self.metadata['urdf_fingerprint'],
                       'friction_pass': True, 'gravity_result':original,
                       'friction':{x:[0.05,0.06] for x in ('positive_coulomb','negative_coulomb','positive_viscous','negative_viscous')}}
        (self.task/'result.json').write_text(json.dumps(self.result))
        (self.task/'frames.csv').write_text('frames\n')
        (self.task/'trajectory.csv').write_text('sample_dt,0.01\nindex,joint2,joint6\n' + ''.join(f'{i},0,0\n' for i in range(25)))

    def local(self, *,tamper=False):
        candidate = dict(self.result['gravity_result'])
        candidate['identification_mode']='local'
        candidate['locked_links']=['Link2']
        candidate['fitted_links']=['Link6']
        candidate['first_moments'] = [{'link_name':'Link2','value':[.2 if tamper else .1,0,0]},
                                     {'link_name':'Link6','value':[.012,.016,0]}]
        (self.task/'recomputed-candidate.json').write_text(json.dumps(candidate))

    def test_historical_link_discovery_skips_massless_frame(self):
        result = load_model_calibration_summary(self.task)
        self.assertEqual([x['name'] for x in result['identifiable_links']], ['Link2','Link6'])
        self.assertEqual([x['joint'] for x in result['identifiable_links']], ['joint2','joint6'])

    def test_local_export_only_changes_unlocked_link_and_stales_friction(self):
        self.local()
        payload = build_gravity_correction_payload(self.task, allow_unvalidated=True)[3]
        self.assertEqual(payload['locked_links'], ['Link2'])
        self.assertEqual(payload['fitted_links'], ['Link6'])
        self.assertFalse(payload['friction_pass'])
        self.assertEqual(payload['candidate_source'], 'recomputed-candidate.json')
        path = export_candidate_urdf(self.task)
        self.assertEqual(path['changed_inertial_origins'], ['Link6'])
        root = ET.parse(path['candidate_urdf']).getroot()
        self.assertEqual(root.find("./link[@name='Link2']/inertial/origin").get('xyz'), '0.1 0 0')
        self.assertEqual(root.find("./link[@name='Link6']/inertial/origin").get('xyz'), '0.015 0.02 0')
        friction = export_friction_candidate(self.task)
        self.assertFalse(friction['matches_current_candidate'])
        self.assertFalse(friction['can_apply_directly'])

    def test_locked_link_tampering_is_rejected(self):
        self.local(tamper=True)
        with self.assertRaisesRegex(ValueError, 'locked Link'):
            export_candidate_urdf(self.task)

    def test_original_friction_export_is_separate_and_review_only(self):
        friction = export_friction_candidate(self.task)
        self.assertTrue(friction['matches_current_candidate'])
        self.assertTrue(friction['friction_validation_pass'])
        self.assertFalse(friction['can_apply_directly'])
        self.assertTrue(Path(friction['path']).is_file())

    def test_unrecorded_link_rejected(self):
        self.local()
        p = self.task/'recomputed-candidate.json'
        data=json.loads(p.read_text())
        data['locked_links']=['tool0']
        p.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'Link selection'):
            export_candidate_urdf(self.task)


if __name__=='__main__':
    unittest.main()
