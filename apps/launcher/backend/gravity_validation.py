"""Offline gravity-candidate evaluation with explicit deployment semantics

Rows are stationary pose aggregates and columns are joints, all in N m.  This
module does not fit inertial parameters, select hyperparameters, or operate a
robot.  Its two scores answer different questions

* raw_urdf: prediction minus measured torque, without a fitted offset
* bias_corrected: the same residual minus a per-joint offset estimated only
  from training poses and then frozen for the held-out poses

Torque feedback is not independent gravity truth.  An offline pass is evidence
for a candidate on this particular holdout, not certification of compensation
or physical safety on hardware.  The improvement and regression thresholds
below are engineering decision thresholds, not estimates of sensor noise
"""
from __future__ import annotations

from functools import wraps
import numpy as np


def _require_finite_result(value):
    if isinstance(value, dict):
        for item in value.values():
            _require_finite_result(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _require_finite_result(item)
    elif isinstance(value, (float, np.floating)) and not np.isfinite(value):
        raise ValueError('gravity evaluation produced a non-finite derived metric')


def _finite_evaluation(function):
    """Fail closed on overflow even when every supplied input is finite"""
    @wraps(function)
    def checked(*args, **kwargs):
        try:
            with np.errstate(over='raise', invalid='raise', divide='raise'):
                result = function(*args, **kwargs)
        except (FloatingPointError, OverflowError) as exc:
            raise ValueError('gravity evaluation produced non-finite derived residuals or metrics') from exc
        # Python float operations (for example a relative threshold product)
        # do not obey NumPy errstate; reject them before serializing a report
        _require_finite_result(result)
        return result
    return checked


def _pose_arrays(baseline, candidate, measured, label):
    values = [np.asarray(item, dtype=float) for item in (baseline, candidate, measured)]
    if any(item.ndim != 2 for item in values):
        raise ValueError(f'{label} arrays must have shape (poses, joints)')
    if any(item.shape != values[0].shape for item in values[1:]):
        raise ValueError(f'{label} prediction and measurement shapes differ')
    if values[0].shape[1] < 1 or any(not np.isfinite(item).all() for item in values):
        raise ValueError(f'{label} arrays need at least one joint and finite values')
    return tuple(values)


def _nonnegative_number(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite nonnegative number') from exc
    if not np.isfinite(number) or number < 0:
        raise ValueError(f'{name} must be a finite nonnegative number')
    return number


def _integer(value, name, minimum):
    if (isinstance(value, (bool, np.bool_))
            or not isinstance(value, (int, np.integer)) or value < minimum):
        raise ValueError(f'{name} must be an integer >= {minimum}')
    return int(value)


def _metrics(error):
    if len(error) == 0:
        return None
    return {
        'rms': np.sqrt(np.mean(error**2, axis=0)).tolist(),
        'p99': np.percentile(np.abs(error), 99, axis=0).tolist(),
        'max': np.max(np.abs(error), axis=0).tolist(),
        'aggregate_rms_nm': float(np.sqrt(np.mean(error**2))),
        'mean_residual_nm': np.mean(error, axis=0).tolist(),
    }


def _comparison(before, after, absolute_improvement_nm, relative_improvement,
                axis_tolerance):
    if before is None or after is None:
        return {
            'direction': 'unavailable', 'strict_improvement': False,
            'per_joint_non_regression': False,
            'required_improvement_nm': None, 'improvement_nm': None,
            'relative_improvement': None, 'per_joint_improvement_nm': None,
            'regressed_joint_indices': [],
        }
    original = before['aggregate_rms_nm']
    updated = after['aggregate_rms_nm']
    threshold = max(absolute_improvement_nm, relative_improvement * original)
    improvement = original - updated
    per_joint = np.asarray(before['rms']) - np.asarray(after['rms'])
    regressed = np.flatnonzero(-per_joint > axis_tolerance).tolist()
    # Strict means beyond the declared minimum, not just numerical inequality
    strict = improvement > threshold
    if regressed or improvement < -threshold:
        direction = 'degraded'
    elif strict:
        direction = 'improved'
    else:
        direction = 'equivalent'
    return {
        'direction': direction,
        'strict_improvement': bool(strict),
        'per_joint_non_regression': not regressed,
        'required_improvement_nm': threshold,
        'improvement_nm': improvement,
        'relative_improvement': improvement / original if original > 0 else None,
        'per_joint_improvement_nm': per_joint.tolist(),
        'regressed_joint_indices': regressed,
    }


@_finite_evaluation
def evaluate_gravity_candidate(
    training_baseline, training_candidate, training_measured,
    validation_baseline, validation_candidate, validation_measured, *,
    candidate_changed, prior_preferred, rank,
    absolute_improvement_nm=0.001, relative_improvement=0.01,
    per_joint_regression_tolerance_nm=0.02,
    min_training_poses=3, min_validation_poses=2,
):
    """Evaluate already-fixed baseline/candidate predictions on disjoint poses

    All six arrays use shape ``(pose_count, joint_count)``; different training
    and validation counts are allowed.  Callers must provide disjoint pose
    groups and identify the model, data, and solver versions in their report.
    The evaluator cannot establish independence from predictions alone

    The returned ``status`` is one of prior_retained, improved, equivalent,
    degraded, or insufficient_evidence.  ``static_pass`` is an *offline* raw
    prediction-improvement gate.  It requires a changed, non-prior candidate,
    rank > 0, sufficient pose counts, a strict aggregate RMS improvement, and
    no per-joint regression beyond the supplied tolerance.  Rank need not
    equal the number of physical parameters; callers must preserve unsupported
    directions and separately check parameter/export physical admissibility

    Two validation poses meet the default calculation gate but still provide
    limited same-session evidence.  No number of same-session poses makes this
    function authorize hardware use.  The bias sign is model-minus-measured
    """
    train = _pose_arrays(training_baseline, training_candidate, training_measured, 'training')
    hold = _pose_arrays(validation_baseline, validation_candidate, validation_measured, 'validation')
    if train[0].shape[1] != hold[0].shape[1]:
        raise ValueError('training and validation joint counts differ')
    if not isinstance(candidate_changed, (bool, np.bool_)):
        raise ValueError('candidate_changed must be a boolean')
    if not isinstance(prior_preferred, (bool, np.bool_)):
        raise ValueError('prior_preferred must be a boolean')
    rank = _integer(rank, 'rank', 0)
    min_training_poses = _integer(min_training_poses, 'min_training_poses', 1)
    min_validation_poses = _integer(min_validation_poses, 'min_validation_poses', 1)
    absolute_improvement_nm = _nonnegative_number(absolute_improvement_nm, 'absolute_improvement_nm')
    relative_improvement = _nonnegative_number(relative_improvement, 'relative_improvement')
    joints = train[0].shape[1]
    try:
        tolerance = np.broadcast_to(np.asarray(per_joint_regression_tolerance_nm, float), (joints,))
    except (ValueError, TypeError) as exc:
        raise ValueError('per_joint_regression_tolerance_nm must be scalar or one value per joint') from exc
    if not np.isfinite(tolerance).all() or np.any(tolerance < 0):
        raise ValueError('per_joint_regression_tolerance_nm must be finite and nonnegative')

    raw_train = (train[0] - train[2], train[1] - train[2])
    raw_hold = (hold[0] - hold[2], hold[1] - hold[2])
    # No validation measurements may enter this estimate
    biases = tuple(error.mean(axis=0) for error in raw_train) if len(train[0]) else None

    def report(errors_train, errors_hold):
        result = {
            'training': {'baseline': _metrics(errors_train[0]), 'candidate': _metrics(errors_train[1])},
            'validation': {'baseline': _metrics(errors_hold[0]), 'candidate': _metrics(errors_hold[1])},
        }
        result['comparison'] = _comparison(
            result['validation']['baseline'], result['validation']['candidate'],
            absolute_improvement_nm, relative_improvement, tolerance)
        return result

    raw_report = report(raw_train, raw_hold)
    corrected_report = None
    if biases is not None:
        corrected_report = report(
            tuple(error - bias for error, bias in zip(raw_train, biases)),
            tuple(error - bias for error, bias in zip(raw_hold, biases)),
        )
    reasons = []
    if rank == 0:
        reasons.append('no_supported_parameter_direction')
    if len(train[0]) < min_training_poses:
        reasons.append('insufficient_training_poses')
    if len(hold[0]) < min_validation_poses:
        reasons.append('insufficient_validation_poses')
    if prior_preferred and candidate_changed:
        reasons.append('prior_preferred_but_parameters_changed')
    sufficient = not reasons
    if prior_preferred and candidate_changed:
        status = 'insufficient_evidence'
    elif prior_preferred or not candidate_changed:
        status = 'prior_retained'
        reasons.append('no_parameter_improvement_claim')
    elif not sufficient:
        status = 'insufficient_evidence'
    else:
        status = raw_report['comparison']['direction']
    if status == 'equivalent':
        reasons.append('raw_improvement_below_engineering_threshold')
    elif status == 'degraded':
        reasons.append('raw_aggregate_or_per_joint_regression')
    if corrected_report and corrected_report['comparison']['direction'] == 'improved' and status != 'improved':
        reasons.append('bias_corrected_improvement_does_not_certify_raw_urdf')

    return {
        'schema': 'serial_arm_gravity_validation', 'schema_version': 1,
        'status': status, 'static_pass': status == 'improved',
        'candidate_changed': bool(candidate_changed), 'prior_preferred': bool(prior_preferred),
        'supported_rank': rank, 'reasons': reasons,
        'raw_urdf': raw_report, 'bias_corrected': corrected_report,
        'training_bias': {
            'convention': 'model_minus_measured',
            'estimation': 'per_joint_training_pose_mean_frozen_on_validation',
            'baseline': biases[0].tolist() if biases is not None else None,
            'candidate': biases[1].tolist() if biases is not None else None,
            'used_for_raw_urdf_score': False,
        },
        'thresholds': {
            'meaning': 'engineering_decision_thresholds_not_measurement_noise',
            'absolute_improvement_nm': absolute_improvement_nm,
            'relative_improvement': relative_improvement,
            'improvement_rule': 'baseline_rms_minus_candidate_rms > max(absolute, relative * baseline_rms)',
            'per_joint_regression_tolerance_nm': tolerance.tolist(),
            'min_training_poses': min_training_poses,
            'min_validation_poses': min_validation_poses,
        },
        'evidence': {
            'scope': 'same_session_pose_holdout_limited_evidence',
            'training_pose_count': len(train[0]), 'validation_pose_count': len(hold[0]),
            'joint_count': joints, 'minimum_pose_count_gate_met': (
                len(train[0]) >= min_training_poses and len(hold[0]) >= min_validation_poses),
            'holdout_independence_verified': False,
            'independence_requirement': 'caller_must_verify_disjoint_pose_groups',
            'cross_session_validation': False, 'validated_on_hardware': False,
            'automatic_application_allowed': False,
            'physical_admissibility_checked': False,
            'measurement_reference': 'recorded_torque_feedback_not_independent_gravity_truth',
            'metric_weighting': 'equal_pose_equal_joint',
            'p99_scope': 'pose_aggregate_errors_not_individual_frame_errors',
        },
    }
