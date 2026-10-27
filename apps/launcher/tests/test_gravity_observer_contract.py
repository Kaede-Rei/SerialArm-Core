"""Keep URDF acceptance and the bias-subtracted observer dead zone separate."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType
import unittest
from unittest import mock

BACKEND = next(parent / 'apps' / 'launcher' / 'backend'
               for parent in Path(__file__).resolve().parents
               if (parent / 'apps' / 'launcher' / 'backend' / 'gravity_validation.py').is_file())
dependencies = ModuleType('model_calibration_source')
for name in ('matches_recorded_fingerprint', 'verified_source_urdf', 'resolve_relative_mesh'):
    setattr(dependencies, name, mock.Mock())
spec = importlib.util.spec_from_file_location('persistence_observer_contract_under_test', BACKEND / 'persistence.py')
persistence = importlib.util.module_from_spec(spec)
with mock.patch.dict(sys.modules, {'model_calibration_source': dependencies}):
    spec.loader.exec_module(persistence)


def candidate():
    # A constant residual of +2 N m is removed by the observer, while the
    # raw URDF still has that error and must retain its failed acceptance.
    return {
        'static_pass': False,
        'failure_reason': 'raw URDF prediction degraded',
        'identification_mode': 'global',
        'first_moments': [{'link_name': 'tip', 'value': [0., 0., .1]}],
        'fitted_links': ['tip'],
        'torque_bias': [2.], 'torque_bias_convention': 'model_minus_measured',
        'fitted_torque_bias': [0.],
        'candidate_run_id': 'fixed-test-run', 'fit_objective': 'urdf',
        'deployment_objective': 'raw_urdf',
        'noise_rms': [.02],
        'validation_candidate': {'rms': [2.], 'p99': [2.1], 'max': [2.1]},
        'gravity_validation': {
            'schema': 'serial_arm_gravity_validation', 'schema_version': 1,
            'status': 'degraded', 'static_pass': False,
            'raw_urdf': {'validation': {'candidate': {'p99': [2.1]}}},
            'bias_corrected': {'validation': {'candidate': {'p99': [.01]}}},
            'training_bias': {'candidate': [2.], 'convention': 'model_minus_measured'},
        },
    }


class GravityObserverContractTests(unittest.TestCase):
    def test_corrected_dead_zone_does_not_include_already_removed_bias(self):
        value = candidate()
        original = copy.deepcopy(value)
        self.assertAlmostEqual(persistence._candidate_threshold(value, ['axis'])[0], .06)
        self.assertEqual(value, original)
        self.assertEqual(value['validation_candidate']['p99'], [2.1])
        self.assertFalse(value['static_pass'])

    def test_payload_retains_observer_bias_and_failed_raw_gate(self):
        gravity = candidate()
        metadata = {'joint_names': ['axis'], 'core_fingerprint': 'core',
                    'urdf_fingerprint': 'model'}
        result = {'task_id': 'test', 'joint_names': ['axis'], 'phase': 'complete',
                  'core_fingerprint': 'core', 'urdf_fingerprint': 'model',
                  'gravity_result': gravity}
        with mock.patch.object(persistence, '_load_model_calibration_task',
                               return_value=(Path('/unused'), metadata, result)):
            _, _, _, payload = persistence.build_gravity_correction_payload('/unused', allow_unvalidated=True)
            self.assertFalse(payload['static_pass'])
            self.assertEqual(payload['residual']['torque_bias'], [2.])
            self.assertEqual(payload['residual']['torque_threshold'], [.06])
            self.assertEqual(payload['validation']['validation_candidate']['p99'], [2.1])
            self.assertEqual(payload['torque_bias_role'], 'observer_residual_offset_not_gravity_feedforward')
            self.assertIn('bias_corrected', payload['torque_threshold_source'])
            self.assertEqual(payload['fit_objective'], 'urdf')
            self.assertEqual(payload['deployment_objective'], 'raw_urdf')
            with self.assertRaisesRegex(ValueError, 'validated static candidate'):
                persistence.build_gravity_correction_payload('/unused')

    def test_offline_payload_hash_binds_exact_parsed_report_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            (folder / 'source').mkdir()
            (folder / 'frames.csv').write_bytes(b'frames\n')
            source = folder / 'source/model.urdf'
            source.write_bytes(b'<robot name="fixture"/>')
            metadata = {'joint_names': ['axis'], 'core_fingerprint': 'core',
                        'urdf_fingerprint': 'model', 'task_id': 'test'}
            result = {**metadata, 'phase': 'complete', 'gravity_result': candidate()}
            gravity = candidate()
            gravity.update(task_id='test', core_fingerprint='core', urdf_fingerprint='model',
                           frames_sha256=hashlib.sha256((folder / 'frames.csv').read_bytes()).hexdigest(),
                           accepted_static_sha256=None,
                           source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                           candidate_masses={'tip': 1.})
            # Deliberately noncanonical formatting: hash must describe the
            # report bytes, not json.dumps of the reconstructed dictionary.
            raw = (json.dumps(gravity, indent=4) + '\n').encode()
            (folder / 'recomputed-candidate.json').write_bytes(raw)
            with mock.patch.object(persistence, '_load_model_calibration_task',
                                   return_value=(folder, metadata, result)), \
                 mock.patch.object(persistence, 'verified_source_urdf', return_value=source):
                _, _, _, payload = persistence.build_gravity_correction_payload(
                    folder, allow_unvalidated=True, prefer_recomputed=True)
            self.assertEqual(payload['candidate_report_sha256'], hashlib.sha256(raw).hexdigest())
            self.assertEqual(payload['candidate_run_id'], 'fixed-test-run')
            self.assertEqual(payload['fit_objective'], 'urdf')
            self.assertEqual(payload['deployment_objective'], 'raw_urdf')
            self.assertEqual(payload['residual']['torque_bias'], [2.])

    def test_legacy_records_keep_existing_threshold_semantics(self):
        legacy = {'validation_candidate': {'p99': [.08, .01]}, 'noise_rms': [.02, .04]}
        self.assertEqual(persistence._candidate_threshold(legacy, ['a', 'b']), [.096, .12])

    def test_new_report_without_corrected_metrics_does_not_fall_back_to_raw(self):
        for invalid in (None, {}, {'validation': None}, {'validation': {'candidate': []}}):
            value = candidate()
            value['gravity_validation']['bias_corrected'] = invalid
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                persistence._candidate_threshold(value, ['axis'])

    def test_new_report_rejects_unknown_schema(self):
        value = candidate()
        value['gravity_validation']['schema'] = 'unknown'
        with self.assertRaises(ValueError):
            persistence._candidate_threshold(value, ['axis'])

    def test_invalid_scale_or_p99_cannot_create_invalid_dead_zone(self):
        for invalid in (float('nan'), float('inf'), -.1, True, '0.1'):
            for field in ('noise', 'p99'):
                value = candidate()
                if field == 'noise': value['noise_rms'] = [invalid]
                else: value['gravity_validation']['bias_corrected']['validation']['candidate']['p99'] = [invalid]
                with self.subTest(field=field, invalid=invalid), self.assertRaises(ValueError):
                    persistence._candidate_threshold(value, ['axis'])

    def test_wrong_joint_count_rejected_for_new_and_legacy_reports(self):
        value = candidate()
        with self.assertRaises(ValueError):
            persistence._candidate_threshold(value, ['a', 'b'])
        value.pop('gravity_validation')
        with self.assertRaises(ValueError):
            persistence._candidate_threshold(value, ['a', 'b'])


if __name__ == '__main__':
    unittest.main()
