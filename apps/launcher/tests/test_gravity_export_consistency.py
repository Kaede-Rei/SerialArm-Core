"""Exercise real artifact copying/hashing with an isolated fake native worker.

No Pinocchio, hardware, repository data, or deployment directory is required.
Only the source-snapshot lookup, payload builder, and native subprocess are
replaced; report bytes, XML/YAML parsing, hashing, copying, and ZIP assembly
use the production implementation against temporary fixture files.
"""
from contextlib import contextmanager
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock
import zipfile

import yaml

BACKEND = next(parent / 'apps' / 'launcher' / 'backend'
               for parent in Path(__file__).resolve().parents
               if (parent / 'apps' / 'launcher' / 'backend' / 'gravity_validation.py').is_file())
source_dependencies = ModuleType('model_calibration_source')
for name in ('matches_recorded_fingerprint', 'verified_source_urdf', 'resolve_relative_mesh'):
    setattr(source_dependencies, name, mock.Mock())
registry_dependency = ModuleType('workflow_registry')
registry_dependency.load = mock.Mock()
spec = importlib.util.spec_from_file_location('export_consistency_persistence', BACKEND / 'persistence.py')
persistence = importlib.util.module_from_spec(spec)
with mock.patch.dict(sys.modules, {'model_calibration_source': source_dependencies}):
    spec.loader.exec_module(persistence)
spec = importlib.util.spec_from_file_location('artifact_export_consistency_under_test', BACKEND / 'artifact_exports.py')
exports = importlib.util.module_from_spec(spec)
with mock.patch.dict(sys.modules, {'model_calibration_source': source_dependencies,
                                  'persistence': persistence,
                                  'workflow_registry': registry_dependency}):
    spec.loader.exec_module(exports)


def digest(data):
    return hashlib.sha256(data).hexdigest()


class GravityExportConsistencyTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.folder = self.root / 'immutable-task'
        self.folder.mkdir()
        self.dest = self.root / 'artifact'
        self.dest.mkdir()
        self.core = self.folder / 'core.yaml'
        self.core.write_text('''model:
  urdf_path: source.urdf
  gravity_scale: {axis: 1.7}
capability:
  admittance:
    calibration:
      torque_bias: {axis: 9.0}
      torque_threshold: {axis: 8.0}
      friction:
        enabled: true
        velocity_transition: 0.2
        positive_coulomb: {axis: 1.0}
        positive_viscous: {axis: 2.0}
        negative_coulomb: {axis: 3.0}
        negative_viscous: {axis: 4.0}
control:
  retained_setting: keep-me
''', encoding='utf-8')
        self.source_model = self.folder / 'source.urdf'
        self.source_model.write_bytes(b'<robot name="source"/>')
        self.record = {'directory': str(self.folder), 'task_id': 'task-1'}
        self.metadata = {'joint_names': ['axis'], 'task_id': 'task-1'}
        self.report = {'candidate_run_id': 'run-A', 'fit_objective': 'urdf',
                       'deployment_objective': 'raw_urdf', 'torque_bias': [.3],
                       'static_pass': False, 'validation_conclusion': 'degraded'}
        # Noncanonical whitespace must survive export unchanged.
        self.report_bytes = (json.dumps(self.report, indent=4) + '\n').encode()
        (self.folder / 'recomputed-candidate.json').write_bytes(self.report_bytes)
        self.payload = {
            'candidate_source': 'recomputed-candidate.json',
            'candidate_run_id': 'run-A',
            'candidate_report_sha256': digest(self.report_bytes),
            'static_pass': False, 'friction_pass': False, 'friction_matches_gravity': False,
            'torque_bias_source': 'fitted_candidate',
            'residual': {'torque_bias': [.3], 'torque_threshold': [.06], 'friction': {
                'velocity_transition': .03,
                'positive_coulomb': [0.], 'positive_viscous': [0.],
                'negative_coulomb': [0.], 'negative_viscous': [0.],
            }},
        }
        self.urdf_bytes = b'<?xml version="1.0"?><robot name="candidate"><link name="tip"/></robot>\n'
        self.worker_folder = None

    def worker(self, argv, *, capture_output, text, timeout, env,
               changed_candidate=False, bad_report_receipt=False, bad_urdf_receipt=False):
        self.assertEqual(argv[2], str(self.folder))
        self.assertEqual(argv[4], 'legacy_gravity_export')
        self.assertEqual(json.loads(argv[5]), {'candidate_path':str(self.folder / 'recomputed-candidate.json')})
        self.assertEqual(env['PYTHONNOUSERSITE'], '1')
        self.worker_folder = Path(argv[3]) / 'offline'
        self.worker_folder.mkdir()
        urdf = self.worker_folder / 'candidate-offline.urdf'
        urdf.write_bytes(self.urdf_bytes)
        report_bytes = self.report_bytes
        if changed_candidate:
            # Deterministically model another recompute replacing latest.json
            # after the parent built its payload but before the worker read it.
            report_bytes = json.dumps({**self.report, 'candidate_run_id': 'run-B',
                                      'torque_bias': [-.7]}).encode()
        (self.worker_folder / 'recomputed-candidate.json').write_bytes(report_bytes)
        resources = self.worker_folder / 'resources'
        resources.mkdir()
        (resources / 'fixture.mesh').write_bytes(b'fixture resource bytes\n')
        verification = {
            'candidate_urdf': str(urdf), 'candidate_source': 'recomputed-candidate.json',
            'candidate_run_id': 'run-B' if changed_candidate else 'run-A',
            'candidate_report_sha256': '0'*64 if bad_report_receipt else digest(report_bytes),
            'candidate_urdf_sha256': '0'*64 if bad_urdf_receipt else digest(self.urdf_bytes),
            'holdout_pass': False,
        }
        (self.worker_folder / 'candidate-urdf-verification.json').write_text(json.dumps(verification))
        return SimpleNamespace(returncode=0, stdout=json.dumps(verification), stderr='')

    @contextmanager
    def isolated_export(self, **worker_options):
        def run(argv, **kwargs):
            return self.worker(argv, **kwargs, **worker_options)
        with mock.patch.object(exports, '_snapshot', return_value=(
                self.folder, self.metadata, self.source_model, self.core,
                yaml.safe_load(self.core.read_text()))), \
             mock.patch.object(exports, '_gravity_merge_source', return_value=(self.core, 'task_snapshot')), \
             mock.patch.object(exports, 'build_gravity_correction_payload',
                               return_value=(self.folder, self.metadata, {}, self.payload)), \
             mock.patch.object(exports, 'load', return_value=self.record), \
             mock.patch('subprocess.run', side_effect=run) as worker:
            yield worker

    def assert_manifest(self, get_bytes):
        report_bytes = get_bytes('identification-report.json')
        self.assertEqual(report_bytes, self.report_bytes)
        self.assertEqual(get_bytes('robot.urdf'), self.urdf_bytes)
        verification = json.loads(get_bytes('candidate-verification.json'))
        self.assertEqual(verification['candidate_urdf'], 'robot.urdf')
        self.assertEqual(verification['candidate_source'], 'identification-report.json')
        self.assertEqual(verification['candidate_report_sha256'], digest(report_bytes))
        self.assertEqual(verification['candidate_urdf_sha256'], digest(get_bytes('robot.urdf')))
        self.assertEqual(verification['candidate_run_id'], 'run-A')
        self.assertFalse(verification['holdout_pass'])
        config = yaml.safe_load(get_bytes('core.yaml'))
        self.assertEqual(config['model']['urdf_path'], 'robot.urdf')
        self.assertEqual(config['model']['gravity_scale'], {'axis': 1.0})
        calibration = config['capability']['admittance']['calibration']
        self.assertEqual(calibration['torque_bias'], {'axis': .3})
        self.assertEqual(calibration['torque_threshold'], {'axis': .06})
        self.assertFalse(calibration['friction']['enabled'])
        self.assertEqual(config['control']['retained_setting'], 'keep-me')

    def test_offline_export_copies_exact_report_and_relocatable_manifest(self):
        original_core = self.core.read_bytes()
        with self.isolated_export() as worker:
            result = exports._gravity_export(self.record, self.dest, 'offline')
        worker.assert_called_once()
        self.assert_manifest(lambda name: (self.dest / name).read_bytes())
        self.assertFalse(self.worker_folder.exists())  # all remaining paths must be package-local
        self.assertEqual((self.dest / 'resources/fixture.mesh').read_bytes(), b'fixture resource bytes\n')
        self.assertEqual(self.core.read_bytes(), original_core)
        self.assertFalse(result['static_pass'])
        self.assertFalse(result['validated'])
        self.assertTrue(result['review_required'])

    def test_zip_carries_report_and_hashes_for_its_actual_files(self):
        with self.isolated_export():
            result = exports.export_artifact(self.root, self.folder, 'gravity',
                                             destination=self.dest, candidate_source='offline')
        with zipfile.ZipFile(result['path']) as archive:
            self.assert_manifest(archive.read)
            self.assertIn('resources/fixture.mesh', archive.namelist())
            self.assertFalse(any(name.startswith('/') for name in archive.namelist()))

    def test_concurrent_candidate_replacement_rejects_mixed_zip(self):
        with self.isolated_export(changed_candidate=True):
            with self.assertRaisesRegex(ValueError, '离线候选发生变化'):
                exports.export_artifact(self.root, self.folder, 'gravity',
                                        destination=self.dest, candidate_source='offline')
        self.assertFalse(list(self.dest.glob('*.zip')))

    def test_worker_report_receipt_mismatch_is_rejected(self):
        with self.isolated_export(bad_report_receipt=True):
            with self.assertRaisesRegex(ValueError, '导出校验记录不一致'):
                exports._gravity_export(self.record, self.dest, 'offline')
        self.assertFalse((self.dest / 'core.yaml').exists())

    def test_worker_urdf_receipt_mismatch_is_rejected(self):
        with self.isolated_export(bad_urdf_receipt=True):
            with self.assertRaisesRegex(ValueError, '导出校验记录不一致'):
                exports._gravity_export(self.record, self.dest, 'offline')
        self.assertFalse((self.dest / 'core.yaml').exists())


if __name__ == '__main__':
    unittest.main()
