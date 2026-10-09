"""Offline URDF export and native-extension isolation regressions"""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from model_calibration_source import (matches_recorded_fingerprint,
                                      recorded_file_fingerprint, verified_source_urdf)
from persistence import export_candidate_urdf
from bridge import run_full_inertial_isolated


class CandidateExportResilience(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name) / 'task'
        (self.directory / 'source').mkdir(parents=True)
        self.urdf = self.directory / 'source/model.urdf'
        self.urdf.write_text('<robot name="x"><link name="link1"><inertial>'
                             '<origin xyz="0 0 0"/><mass value="2"/>'
                             '<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>'
                             '</inertial></link></robot>')
        self.metadata = {'urdf_path': str(Path(self.tmp.name) / 'vanished.urdf'),
                         'urdf_fingerprint': recorded_file_fingerprint(self.urdf)}

    def test_existing_cpp_fingerprint_is_not_sha256(self):
        self.assertEqual(len(self.metadata['urdf_fingerprint']), 16)
        self.assertTrue(matches_recorded_fingerprint(self.urdf, self.metadata['urdf_fingerprint']))
        self.assertEqual(verified_source_urdf(self.directory, self.metadata), self.urdf)

    def test_changed_snapshot_rejected_even_when_live_path_missing(self):
        self.urdf.write_text('<robot name="altered"/>')
        with self.assertRaisesRegex(ValueError, '指纹'):
            verified_source_urdf(self.directory, self.metadata)

    def test_sha256_backwards_compatibility(self):
        import hashlib
        sha = hashlib.sha256(self.urdf.read_bytes()).hexdigest()
        self.assertTrue(matches_recorded_fingerprint(self.urdf, sha))

    def test_candidate_export_uses_original_record_snapshot_after_move(self):
        payload = {'first_moments': {'link1': [0.8, -0.4, 0.2]},
                   'static_pass': False, 'candidate_quality_reason': 'holdout_worse'}
        with patch('persistence.build_gravity_correction_payload', return_value=(self.directory,self.metadata,{'gravity_result':{}},payload)):
            info = export_candidate_urdf(self.directory)
        from xml.etree import ElementTree as ET
        root = ET.parse(info['candidate_urdf']).getroot()
        self.assertEqual(root.find('./link/inertial/origin').get('xyz'), '0.4 -0.2 0.1')
        self.assertTrue(info['review_required'])
        self.assertFalse(info['holdout_pass'])
        self.assertEqual(Path(info['source_urdf']), self.urdf)

    def test_full_inertial_native_crash_is_contained_and_explained(self):
        import subprocess
        with patch('bridge.subprocess.run', return_value=subprocess.CompletedProcess([], -11, '', 'Segmentation fault')):
            with self.assertRaisesRegex(ValueError, 'Backend 未受影响'):
                run_full_inertial_isolated(self.directory)

    def test_numpy_abi_mismatch_produces_actionable_error(self):
        import subprocess
        completed = subprocess.CompletedProcess([], -11, '', 'AttributeError: _ARRAY_API not found')
        with patch('bridge.subprocess.run', return_value=completed):
            with self.assertRaisesRegex(ValueError, 'numpy<2'):
                run_full_inertial_isolated(self.directory)

    def test_worker_success_result_returns_json(self):
        import subprocess
        completed = subprocess.CompletedProcess([], 0, '{"candidate_urdf": "/tmp/candidate.urdf"}', '')
        with patch('bridge.subprocess.run', return_value=completed) as call:
            self.assertEqual(run_full_inertial_isolated(self.directory)['candidate_urdf'], '/tmp/candidate.urdf')
            self.assertEqual(call.call_args.kwargs['env']['PYTHONNOUSERSITE'], '1')


if __name__ == '__main__':
    unittest.main()
