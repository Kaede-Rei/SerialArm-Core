"""Regression checks for the physical joint-identification objective."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
import joint_identification as joint


class JointRegularizationTests(unittest.TestCase):
    def test_locked_and_full_models_use_same_per_coordinate_penalty(self):
        # Identity pseudo-inertias for two links; the second has a small,
        # physically feasible change while the first has no measurement signal.
        prior = np.tile([1., 0, 0, 0, 2., 0, 2., 0, 0, 2.], 2)
        engine = type('Model', (), {'prior': prior})()
        target = np.linspace(.01, .1, 10)
        z = {'tau': np.zeros((1, 1))}

        def system(_z, _statics, basis, _free, T, _method):
            n = T.shape[1]
            A = np.eye(n + 3)
            b = np.r_[np.zeros(n - 10), target, np.zeros(3)]
            raw = dict(beta=np.zeros(basis.shape[1] + 3), sigma=np.ones(1),
                       equation_rank=n, equation_columns=n, singular_values=np.ones(n))
            return A, b, np.ones(3), raw

        with patch.object(joint, '_system', side_effect=system):
            global_fit, _, global_info = joint.constrained_fit(
                z, [], engine, np.arange(20), np.eye(20), 'WLS', 10.)
            local_fit, _, local_info = joint.constrained_fit(
                z, [], engine, np.arange(10, 20), np.eye(10), 'WLS', 10.)

        np.testing.assert_allclose(global_fit, local_fit, atol=1e-6)
        self.assertEqual(global_info['regularization_normalization_dimension'], 20)
        self.assertEqual(local_info['regularization_normalization_dimension'], 20)
        self.assertEqual(global_info['regularization_per_coordinate'], .5)
        self.assertEqual(local_info['regularization_per_coordinate'], .5)

    def test_motion_folds_spread_each_phase_and_keep_blocks_whole(self):
        train = {'segment': np.repeat(np.arange(10), 2),
                 'phase': np.repeat(['forward'] * 4 + ['reverse'] * 3 + ['fast'] * 3, 2),
                 'moving': np.ones((20, 2), dtype=bool)}
        folds = joint._motion_cv_groups(train, 3)
        self.assertEqual(sorted(group for fold in folds for group in fold), list(range(10)))
        for fold in folds:
            phases = set(train['phase'][np.isin(train['segment'], fold)])
            self.assertEqual(phases, {'forward', 'reverse', 'fast'})

    def test_motion_folds_balance_phases_with_fewer_blocks_than_folds(self):
        train = {'segment': np.arange(6),
                 'phase': np.array(['forward'] * 2 + ['reverse'] * 2 + ['fast'] * 2),
                 'moving': np.ones((6, 2), dtype=bool)}
        folds = joint._motion_cv_groups(train, 3)
        self.assertEqual([len(fold) for fold in folds], [2, 2, 2])
        self.assertEqual(sorted(group for fold in folds for group in fold), list(range(6)))

    def test_motion_folds_rebalance_axis_excitation_without_splitting_blocks(self):
        # The original round-robin gives folds [0,3], [1,4], [2,5],
        # leaving the first and last fold without one of the two axes.
        moving = np.repeat(np.array([[1, 0], [1, 0], [0, 1],
                                     [1, 0], [0, 1], [0, 1]], dtype=bool), 10, axis=0)
        train = {'segment': np.repeat(np.arange(6), 10),
                 'phase': np.repeat(['forward'] * 6, 10), 'moving': moving}
        folds = joint._motion_cv_groups(train, 3)
        self.assertEqual(sorted(group for fold in folds for group in fold), list(range(6)))
        for fold in folds:
            held = np.isin(train['segment'], fold)
            self.assertTrue((train['moving'][held].sum(axis=0) >= 5).all())
            self.assertTrue((train['moving'][~held].sum(axis=0) >= 10).all())

    def test_joint_fit_runs_physical_solver_and_cross_validation(self):
        rng = np.random.default_rng(71)
        prior = np.array([1., 0, 0, 0, 2., 0, 2., 0, 0, 2.])
        engine = type('Model', (), {'prior': prior, 'cols': np.arange(10)})()
        free = np.arange(10)
        basis = np.eye(10)
        T, _, _ = joint.physical_coordinates(prior, free)
        truth = T @ np.linspace(.01, .04, 10)
        design = rng.normal(size=(90, 1, 13))
        tau = np.einsum('nji,i->nj', design[:, :, :10], truth)
        train = {'X': design, 'tau': tau, 'baseline': np.zeros((90, 1)),
                 'moving': np.ones((90, 1), dtype=bool),
                 'v': rng.normal(size=(90, 1)),
                 'segment': np.repeat(np.arange(6), 15),
                 'phase': np.repeat(['forward'] * 3 + ['reverse'] * 3, 15)}
        statics = []
        for _ in range(6):
            Y = rng.normal(size=(1, 10))
            statics.append({'Y': Y, 'tau': Y @ truth, 'baseline': np.zeros(1),
                            'uncertainty': np.ones(1) * .1})

        candidate, fit, info = joint.fit_joint(train, statics, engine, free, basis)
        self.assertLess(np.linalg.norm(candidate - prior - truth), np.linalg.norm(truth))
        self.assertEqual(info['solver_status'], 'optimal')
        self.assertEqual(info['folds'], 3)
        self.assertEqual(info['motion_cv_stratification'], 'phase_and_axis_balanced_whole_blocks')
        self.assertEqual(info['regularization_normalization_dimension'], 10)
        self.assertTrue(np.isfinite(fit['beta']).all())


if __name__ == '__main__':
    unittest.main()
