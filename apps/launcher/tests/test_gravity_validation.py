"""Semantic regression tests for the hardware-free gravity evaluation contract."""
from pathlib import Path
import json
import sys
import unittest

import numpy as np

BACKEND = next(parent / 'apps' / 'launcher' / 'backend'
               for parent in Path(__file__).resolve().parents
               if (parent / 'apps' / 'launcher' / 'backend' / 'gravity_validation.py').is_file())
sys.path.insert(0, str(BACKEND))
from gravity_validation import evaluate_gravity_candidate


class GravityValidationTests(unittest.TestCase):
    def evaluate(self, train_baseline, train_candidate, train_measured,
                 hold_baseline, hold_candidate, hold_measured, **kwargs):
        settings = dict(candidate_changed=True, prior_preferred=False, rank=1)
        settings.update(kwargs)
        return evaluate_gravity_candidate(
            train_baseline, train_candidate, train_measured,
            hold_baseline, hold_candidate, hold_measured, **settings)

    def simple(self, before=1.0, after=0.5, train_count=3, hold_count=2, joints=1, **kwargs):
        tb = np.full((train_count, joints), before)
        tc = np.full((train_count, joints), after)
        hb = np.full((hold_count, joints), before)
        hc = np.full((hold_count, joints), after)
        return self.evaluate(tb, tc, np.zeros_like(tb), hb, hc, np.zeros_like(hb), **kwargs)

    def test_bias_improvement_cannot_hide_raw_regression(self):
        train_q = np.array([1., 2., 3.])[:, None]
        hold_q = np.array([4., 5.])[:, None]
        # Candidate recovers configuration variation but increases its absolute
        # error against the measured torque; the constant offset is not URDF.
        result = self.evaluate(
            np.zeros_like(train_q), .1 * train_q, .1 * train_q - 2.,
            np.zeros_like(hold_q), .1 * hold_q, .1 * hold_q - 2.)
        self.assertEqual(result['status'], 'degraded')
        self.assertFalse(result['static_pass'])
        self.assertEqual(result['bias_corrected']['comparison']['direction'], 'improved')
        self.assertAlmostEqual(result['raw_urdf']['validation']['candidate']['aggregate_rms_nm'], 2.)
        self.assertAlmostEqual(result['bias_corrected']['validation']['candidate']['aggregate_rms_nm'], 0.)
        self.assertIn('bias_corrected_improvement_does_not_certify_raw_urdf', result['reasons'])

    def test_unchanged_prior_is_not_an_improvement(self):
        result = self.simple(before=1., after=1., candidate_changed=False, prior_preferred=True)
        self.assertEqual(result['status'], 'prior_retained')
        self.assertFalse(result['static_pass'])
        self.assertEqual(result['raw_urdf']['comparison']['direction'], 'equivalent')

    def test_changed_model_with_identical_predictions_is_equivalent(self):
        result = self.simple(before=1., after=1.)
        self.assertEqual(result['status'], 'equivalent')
        self.assertFalse(result['static_pass'])

    def test_below_minimum_improvement_does_not_pass(self):
        result = self.simple(before=1., after=.995)
        self.assertEqual(result['status'], 'equivalent')
        self.assertEqual(result['raw_urdf']['comparison']['required_improvement_nm'], .01)
        self.assertFalse(result['static_pass'])

    def test_holdout_perturbation_never_changes_training_bias(self):
        tb = np.arange(6., dtype=float).reshape(3, 2)
        tc = tb + np.array([.2, -.3])
        measured = tb - np.array([.5, -.8])
        hb = np.ones((2, 2)); hc = hb + .1
        first = self.evaluate(tb, tc, measured, hb, hc, np.zeros_like(hb))
        second = self.evaluate(tb, tc, measured, hb, hc, np.full_like(hb, 100.))
        self.assertEqual(first['training_bias'], second['training_bias'])
        self.assertEqual(first['bias_corrected']['training'], second['bias_corrected']['training'])
        self.assertNotEqual(first['raw_urdf']['validation'], second['raw_urdf']['validation'])

    def test_supported_rank_one_does_not_require_all_physical_parameters(self):
        # A rank-one observable update can improve predictions at six joints;
        # no full-column-rank requirement is added by the evaluator.
        result = self.simple(rank=1, joints=6)
        self.assertEqual(result['status'], 'improved')
        self.assertTrue(result['static_pass'])
        self.assertEqual(result['supported_rank'], 1)
        self.assertFalse(result['evidence']['automatic_application_allowed'])
        self.assertFalse(result['evidence']['validated_on_hardware'])

    def test_rank_zero_does_not_pass_even_with_better_supplied_predictions(self):
        result = self.simple(rank=0)
        self.assertEqual(result['status'], 'insufficient_evidence')
        self.assertFalse(result['static_pass'])
        self.assertIn('no_supported_parameter_direction', result['reasons'])

    def test_large_joint_regression_blocks_aggregate_improvement(self):
        tb = np.tile([1., .01], (3, 1)); tc = np.tile([.1, .1], (3, 1))
        hb = tb[:2]; hc = tc[:2]
        result = self.evaluate(tb, tc, np.zeros_like(tb), hb, hc, np.zeros_like(hb))
        self.assertTrue(result['raw_urdf']['comparison']['strict_improvement'])
        self.assertEqual(result['raw_urdf']['comparison']['regressed_joint_indices'], [1])
        self.assertEqual(result['status'], 'degraded')
        self.assertFalse(result['static_pass'])

    def test_per_joint_tolerance_is_configurable(self):
        tb = np.tile([1., .01], (3, 1)); tc = np.tile([.1, .1], (3, 1))
        result = self.evaluate(tb, tc, np.zeros_like(tb), tb[:2], tc[:2], np.zeros_like(tb[:2]),
                               per_joint_regression_tolerance_nm=[.02, .1])
        self.assertEqual(result['status'], 'improved')
        self.assertEqual(result['thresholds']['per_joint_regression_tolerance_nm'], [.02, .1])

    def test_one_holdout_is_insufficient_but_direction_is_retained(self):
        result = self.simple(hold_count=1)
        self.assertEqual(result['status'], 'insufficient_evidence')
        self.assertEqual(result['raw_urdf']['comparison']['direction'], 'improved')
        self.assertFalse(result['static_pass'])

    def test_two_holdouts_are_always_labeled_limited_same_session_evidence(self):
        result = self.simple(hold_count=2)
        self.assertEqual(result['status'], 'improved')
        self.assertEqual(result['evidence']['scope'], 'same_session_pose_holdout_limited_evidence')
        self.assertFalse(result['evidence']['cross_session_validation'])
        self.assertFalse(result['evidence']['holdout_independence_verified'])

    def test_empty_training_and_holdout_are_explicit_not_nan(self):
        result = self.simple(train_count=0, hold_count=0)
        self.assertEqual(result['status'], 'insufficient_evidence')
        self.assertIsNone(result['bias_corrected'])
        self.assertIsNone(result['raw_urdf']['validation']['baseline'])
        json.dumps(result, allow_nan=False)

    def test_inconsistent_prior_metadata_cannot_pass(self):
        result = self.simple(candidate_changed=True, prior_preferred=True)
        self.assertEqual(result['status'], 'insufficient_evidence')
        self.assertFalse(result['static_pass'])
        self.assertIn('prior_preferred_but_parameters_changed', result['reasons'])

    def test_zero_baseline_does_not_produce_infinite_relative_improvement(self):
        result = self.simple(before=0., after=.5)
        self.assertEqual(result['status'], 'degraded')
        self.assertIsNone(result['raw_urdf']['comparison']['relative_improvement'])
        json.dumps(result, allow_nan=False)

    def test_finite_residuals_that_overflow_squared_metrics_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'non-finite derived'):
            self.simple(before=1e155, after=2e155)

    def test_subtraction_overflow_from_finite_inputs_is_rejected(self):
        train = np.full((3, 1), 1e308); hold = train[:2]
        with self.assertRaisesRegex(ValueError, 'non-finite derived'):
            self.evaluate(train, train, -train, hold, hold, -hold)

    def test_overflowed_engineering_threshold_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'non-finite derived'):
            self.simple(before=10., after=1., relative_improvement=1e308)

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            self.simple(rank=-1)
        with self.assertRaises(ValueError):
            self.simple(after=float('nan'))
        with self.assertRaises(ValueError):
            self.simple(per_joint_regression_tolerance_nm=-.01)
        with self.assertRaises(ValueError):
            self.simple(relative_improvement=-.01)
        with self.assertRaises(ValueError):
            self.simple(joints=2, per_joint_regression_tolerance_nm=[.01, .02, .03])
        with self.assertRaises(ValueError):
            self.simple(min_validation_poses=0)


if __name__ == '__main__':
    unittest.main()
