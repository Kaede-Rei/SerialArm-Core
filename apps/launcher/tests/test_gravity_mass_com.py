"""Numerical contracts for deployment-aware static gravity identification."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
import gravity_mass_com as gm


class StaticGravityObjectives(unittest.TestCase):
    def setUp(self):
        self.rng = np.random.default_rng(3617)

    def test_invalid_objective_is_rejected(self):
        for objective in ('raw', '', None):
            with self.subTest(objective=objective), self.assertRaises(ValueError):
                gm.solve_static_updates(np.ones((4, 2, 1)), np.zeros((4, 2)), objective=objective)

    def test_default_retains_bias_separated_contract(self):
        A = self.rng.normal(size=(9, 3, 4))
        b = self.rng.normal(size=(9, 3))
        default = gm.solve_static_updates(A, b)
        explicit = gm.solve_static_updates(A, b, objective='bias_separated')
        np.testing.assert_array_equal(default['delta'], explicit['delta'])
        self.assertEqual(default['cross_validation_scores'], explicit['cross_validation_scores'])
        self.assertEqual(default['fit_objective'], 'bias_separated')
        self.assertEqual(default['selection_metric'], 'leave_one_pose_out_bias_separated_mse')

    def test_constant_offset_has_different_meaning_in_the_two_objectives(self):
        A = np.ones((7, 2, 1))
        b = np.full((7, 2), .2)
        separate = gm.solve_static_updates(A, b, objective='bias_separated')
        raw = gm.solve_static_updates(A, b, objective='urdf')
        self.assertEqual(separate['rank'], 0)
        self.assertTrue(separate['prior_preferred'])
        np.testing.assert_array_equal(separate['delta'], [0.])
        np.testing.assert_allclose(separate['torque_bias'], [-.2, -.2], atol=1e-15)
        np.testing.assert_allclose(raw['delta'], [.2], atol=1e-12)
        np.testing.assert_array_equal(raw['torque_bias'], [0., 0.])
        self.assertEqual(raw['selection_metric'], 'leave_one_pose_out_raw_urdf_mse')

    def test_bias_separated_update_is_invariant_to_joint_constant_shifts(self):
        A = self.rng.normal(size=(10, 3, 4))
        b = np.einsum('pji,i->pj', A, [.01, -.02, .005, .007])
        base = gm.solve_static_updates(A, b)
        shifted = gm.solve_static_updates(A, b + [1., -2., 3.])
        np.testing.assert_allclose(base['delta'], shifted['delta'], atol=1e-12, rtol=0)
        np.testing.assert_allclose(shifted['torque_bias'], base['torque_bias'] - [1., -2., 3.], atol=1e-12)

    def test_known_local_mass_and_first_moment_recovery_keeps_locks(self):
        # A linear-regression fixture checks the free-block contract without
        # requiring a robot library in the launcher's Python unit tests.
        Y = np.zeros((18, 3, 20))
        Y[:, :, :4] = self.rng.normal(size=(18, 3, 4))
        Y[:, :, 10:14] = self.rng.normal(size=(18, 3, 4))
        prior = np.zeros(20)
        prior[0], prior[10] = 1.2, .8
        prior[1:4], prior[11:14] = [.01, .02, -.01], [.001, -.003, .02]
        # COM-frame tensors are outside the static optimization variables.
        prior[4:10] = [.1, .02, .2, .01, .03, .3]
        prior[14:20] = [.02, .001, .03, .002, .003, .04]
        delta = np.array([.015, -.0007, .0015, -.002])
        fixed = self.rng.normal(size=(18, 3))*.03
        mapped = [(1, 'Link1', None), (2, 'Link6', None)]
        for objective in ('bias_separated', 'urdf'):
            with self.subTest(objective=objective):
                intercept = np.array([.03, -.01, .02]) if objective == 'bias_separated' else np.zeros(3)
                tau = np.einsum('pji,i->pj', Y, prior)+fixed+np.einsum('pji,i->pj', Y[:, :, 10:14], delta)+intercept
                rows = [(a, f, t) for a, f, t in zip(Y, fixed, tau)]
                fit = gm.fit_static_mass_com(prior, rows, mapped, {'Link1'}, objective=objective)
                np.testing.assert_allclose(fit['candidate'][10:14], prior[10:14]+delta, atol=1e-11, rtol=0)
                np.testing.assert_array_equal(fit['candidate'][:10], prior[:10])
                np.testing.assert_array_equal(fit['candidate'][14:20], prior[14:20])
                self.assertEqual(fit['active_indices'], [10, 11, 12, 13])
                self.assertEqual(fit['rank'], 4)
                np.testing.assert_allclose(fit['torque_bias'], -intercept, atol=1e-11, rtol=0)
                self.assertEqual(fit['solver'], 'static_raw_svd_ridge_pose_cv' if objective == 'urdf' else 'static_centered_svd_ridge_pose_cv')

    def test_external_holdout_targets_do_not_change_training_fit(self):
        A = self.rng.normal(size=(12, 3, 4))
        b = self.rng.normal(size=(12, 3))
        training = np.array([0, 1, 3, 4, 6, 7, 8, 10])
        heldout = np.array([2, 5, 9, 11])
        for objective in ('bias_separated', 'urdf'):
            clean = gm.solve_static_updates(A[training], b[training], objective=objective)
            changed = b.copy()
            changed[heldout] += 1e8
            poison = gm.solve_static_updates(A[training], changed[training], objective=objective)
            np.testing.assert_array_equal(clean['delta'], poison['delta'])
            self.assertEqual(clean['cross_validation_scores'], poison['cross_validation_scores'])

    def test_each_cv_fold_rebuilds_scaling_and_bias_from_its_training_rows(self):
        A = self.rng.normal(size=(7, 3, 4))
        A[-1] *= 30.  # Full-dataset scaling would visibly contaminate this fold.
        b = self.rng.normal(size=(7, 3))
        penalty = 1.
        for objective in ('bias_separated', 'urdf'):
            errors = []
            for held in range(len(A)):
                keep = np.arange(len(A)) != held
                train_a, train_b = A[keep], b[keep]
                centered = train_a-train_a.mean(axis=0) if objective == 'bias_separated' else train_a
                scale = np.linalg.norm(centered.reshape(-1, 4), axis=0)
                # Independent augmented primal system, with an unpenalized
                # joint intercept only in the bias-separated objective.
                X = train_a.reshape(-1, 4)/scale
                bias_cols = 3 if objective == 'bias_separated' else 0
                if bias_cols:
                    X = np.column_stack([X, np.tile(np.eye(3), (len(train_a), 1))])
                reg = np.diag(np.r_[np.full(4, np.sqrt(penalty)), np.zeros(bias_cols)])
                coefficients = np.linalg.lstsq(np.vstack([X, reg]), np.r_[train_b.reshape(-1), np.zeros(X.shape[1])], rcond=None)[0]
                delta = coefficients[:4]/scale
                offset = coefficients[4:] if bias_cols else np.zeros(3)
                errors.append(np.mean((b[held]-A[held]@delta-offset)**2))
            fit = gm.solve_static_updates(A, b, objective=objective)
            i = fit['cross_validation_penalties'].index(penalty)
            self.assertAlmostEqual(fit['cross_validation_scores'][i], float(np.mean(errors)), places=10)

    def test_unobservable_columns_stay_at_prior(self):
        A = self.rng.normal(size=(10, 3, 3))
        A[:, :, 2] = 0.
        b = np.einsum('pji,i->pj', A, [.02, -.03, 100.])
        for objective in ('bias_separated', 'urdf'):
            fit = gm.solve_static_updates(A, b, objective=objective)
            self.assertEqual(fit['rank'], 2)
            self.assertEqual(fit['delta'][2], 0.)

    def test_rank_zero_and_exact_prior_remain_no_update(self):
        for objective in ('bias_separated', 'urdf'):
            for A, b in [(np.zeros((5, 3, 4)), np.ones((5, 3))),
                         (self.rng.normal(size=(8, 3, 4)), np.zeros((8, 3)))]:
                fit = gm.solve_static_updates(A, b, objective=objective)
                self.assertTrue(fit['prior_preferred'])
                self.assertIsNone(fit['regularization'])
                np.testing.assert_array_equal(fit['delta'], np.zeros(4))

    def test_zero_and_weak_ridge_are_available_without_losing_prior(self):
        A = self.rng.normal(size=(8, 3, 4))
        fit = gm.solve_static_updates(A, np.zeros((8, 3)))
        self.assertEqual(fit['cross_validation_penalties'], [None, 0., 1e-6, 1e-4, 1e-3, .01, .1, 1., 10., 100., 1000.])


if __name__ == '__main__':
    unittest.main()
