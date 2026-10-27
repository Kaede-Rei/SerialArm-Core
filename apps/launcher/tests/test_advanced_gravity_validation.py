"""Check static advanced identification's scoring contract without native code.

The synthetic regressor has four mass/first-moment columns but only one
supported direction.  Pinocchio and the solver are replaced at the boundary;
the real shared evaluator decides the reported improvement.
"""
import importlib.util
import hashlib
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

import numpy as np

BACKEND = next(parent / 'apps' / 'launcher' / 'backend'
               for parent in Path(__file__).resolve().parents
               if (parent / 'apps' / 'launcher' / 'backend' / 'gravity_validation.py').is_file())
sys.path.insert(0, str(BACKEND))
fake_dependencies = ModuleType('full_inertial')
for name in ('_prepare_inputs', '_articulated_mapping', '_sampling_map', '_extract_rows',
             '_assign_configuration', '_fit_regressor', '_dynamic_parameters_to_urdf_fields',
             'physical_inertia'):
    setattr(fake_dependencies, name, mock.Mock())
spec = importlib.util.spec_from_file_location('advanced_gravity_validation_under_test', BACKEND / 'advanced_identification.py')
advanced = importlib.util.module_from_spec(spec)
with mock.patch.dict(sys.modules, {'full_inertial': fake_dependencies}):
    spec.loader.exec_module(advanced)


class AdvancedGravityValidationTests(unittest.TestCase):
    def fit(self, offset=0., hold_count=2, prior_preferred=False):
        model = SimpleNamespace(
            njoints=2, nv=1, names=['universe', 'axis'], createData=lambda: object(),
            inertias=[None, SimpleNamespace(toDynamicParameters=lambda: np.r_[1., np.zeros(9)])])
        def regressor(model, data, state, velocity, acceleration):
            y = np.zeros((1, 10)); y[0, 1] = state[0]
            return y
        pin = SimpleNamespace(neutral=lambda model: np.zeros(1), computeJointTorqueRegressor=regressor)
        poses = [(index, index >= 4, np.array([float(index+1)]),
                  np.array([.1*(index+1)+offset]), None)
                 for index in range(4+hold_count)]
        fake_solver = ModuleType('gravity_mass_com')
        fake_solver.solve_static_updates = mock.Mock(return_value={
            'delta': np.array([0., 0. if prior_preferred else .1, 0., 0.]),
            'torque_bias': np.array([-offset]), 'rank': 1, 'regularization': .01,
            'prior_preferred': prior_preferred, 'singular_values': [1., 0., 0., 0.],
            'solver': 'synthetic_one_supported_direction'})
        def assign(model, state, indices, values):
            state[indices] = values
        with mock.patch.object(advanced, '_pose_groups', return_value=poses), \
             mock.patch.object(advanced, '_sampling_map', return_value=([0], [0], list(range(10)))), \
             mock.patch.object(advanced, '_assign_configuration', side_effect=assign), \
             mock.patch.dict(sys.modules, {'gravity_mass_com': fake_solver}):
            result = advanced._fit_static('unused', model, pin, [(1, 'link1', None)], 'mass_com', set())
        self.assertEqual(fake_solver.solve_static_updates.call_args.kwargs['objective'], 'bias_separated')
        return result

    def test_static_candidate_uses_raw_scores_and_nonzero_supported_rank(self):
        fit = self.fit()
        self.assertLess(fit['rank'], fit['columns'])
        self.assertTrue(fit['passed'])
        self.assertEqual(fit['fit_objective'], 'bias_separated')
        self.assertEqual(fit['deployment_objective'], 'raw_urdf')
        self.assertAlmostEqual(fit['candidate_rms'], 0.)
        self.assertEqual(fit['gravity_validation']['status'], 'improved')

    def test_bias_corrected_only_improvement_does_not_pass(self):
        fit = self.fit(offset=-2.)
        self.assertFalse(fit['passed'])
        self.assertGreater(fit['candidate_rms'], fit['baseline'])
        self.assertEqual(fit['gravity_validation']['bias_corrected']['comparison']['direction'], 'improved')
        self.assertEqual(fit['gravity_validation']['status'], 'degraded')

    def test_single_holdout_does_not_pass(self):
        fit = self.fit(hold_count=1)
        self.assertFalse(fit['passed'])
        self.assertEqual(fit['gravity_validation']['status'], 'insufficient_evidence')

    def test_prior_retained_is_explicit(self):
        fit = self.fit(prior_preferred=True)
        self.assertFalse(fit['passed'])
        self.assertEqual(fit['gravity_validation']['status'], 'prior_retained')

    def test_source_hashes_cover_samples_and_model_and_detect_changes(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            model = folder / 'model.urdf'; model.write_text('<robot name="test"/>')
            frames = folder / 'frames.csv'; frames.write_text('raw\n')
            accepted = folder / 'accepted_static.csv'; accepted.write_text('accepted\n')
            original = {path: path.read_bytes() for path in (model, frames, accepted)}
            hashes = advanced._identification_source_hashes(folder, model)
            for key, path in [('source_sha256', model), ('frames_sha256', frames),
                              ('accepted_static_sha256', accepted)]:
                self.assertEqual(hashes[key], hashlib.sha256(original[path]).hexdigest())
            advanced._verify_identification_sources(folder, model, hashes)
            for path in (model, frames, accepted):
                with self.subTest(changed=path.name):
                    path.write_bytes(original[path] + b'changed')
                    with self.assertRaisesRegex(ValueError, '发生变化'):
                        advanced._verify_identification_sources(folder, model, hashes)
                    path.write_bytes(original[path])
            accepted.unlink()
            with self.assertRaisesRegex(ValueError, '发生变化'):
                advanced._verify_identification_sources(folder, model, hashes)
            missing = advanced._identification_source_hashes(folder, model)
            self.assertIsNone(missing['accepted_static_sha256'])
            accepted.write_bytes(original[accepted])
            with self.assertRaisesRegex(ValueError, '发生变化'):
                advanced._verify_identification_sources(folder, model, missing)

    def test_advanced_run_rejects_input_mutation_during_fit_before_export(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary) / 'task'; folder.mkdir()
            snapshot = folder / 'model.urdf'
            snapshot.write_text('<robot name="test"><link name="link1"/></robot>')
            (folder / 'frames.csv').write_text('raw\n')
            accepted = folder / 'accepted_static.csv'; accepted.write_text('accepted\n')
            destination = Path(temporary) / 'output'
            fake_fit = dict(warnings=[], rank=1, columns=4, baseline=1., candidate_rms=.5,
                            passed=True, train_groups=4, holdout_groups=2, regularization=0.,
                            solver='synthetic', torque_bias_convention='model_minus_measured',
                            prior_preferred=False, singular_values=[1.], fit_objective='bias_separated',
                            deployment_objective='raw_urdf', gravity_validation={'status': 'improved'})
            def mutate(*args):
                accepted.write_text('changed during fit\n')
                return fake_fit
            fake_pin = ModuleType('pinocchio'); fake_pin.buildModelFromUrdf = lambda path: object()
            dependencies = ModuleType('full_inertial')
            dependencies._review_candidate = mock.Mock()
            dependencies._extract_rows = mock.Mock()
            dependencies._apply_recorded_gravity = lambda model, metadata: model
            source_module = ModuleType('model_calibration_source')
            source_module.resolve_relative_mesh = mock.Mock()
            with mock.patch.dict(sys.modules, {'pinocchio': fake_pin, 'full_inertial': dependencies,
                                               'model_calibration_source': source_module}), \
                 mock.patch.object(advanced, '_prepare_inputs', return_value=(
                     folder, {'joint_names': ['axis']}, {}, snapshot, None)), \
                 mock.patch.object(advanced, '_articulated_mapping', return_value=([(1, 'link1', None)], [])), \
                 mock.patch.object(advanced, '_fit_static', side_effect=mutate), \
                 mock.patch.object(advanced, '_write_static_candidate'):
                with self.assertRaisesRegex(ValueError, '发生变化'):
                    advanced.run_advanced(folder, 'mass_com', destination)
            self.assertFalse(destination.exists())


if __name__ == '__main__':
    unittest.main()
