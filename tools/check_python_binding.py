#!/usr/bin/env python3
"""Hardware-free installed Python binding check"""

import numpy as np
from serial_arm import ActuatorState, DynamicsState, JointCtrlCmd, JointState, RobotCycleOutput


def check_vector(owner, field, values):
    expected = np.asarray(values, dtype=np.float64)
    setattr(owner, field, expected)
    actual = getattr(owner, field)
    assert isinstance(actual, np.ndarray), (type(owner).__name__, field)
    assert actual.shape == expected.shape, (type(owner).__name__, field, actual.shape)
    assert actual.dtype == np.dtype('float64'), (type(owner).__name__, field, actual.dtype)
    assert actual.strides == (8,), (type(owner).__name__, field, actual.strides)
    np.testing.assert_array_equal(actual, expected, err_msg=f'{type(owner).__name__}.{field}')

    # Repeated reads should own their data instead of exposing mutable C++ state
    actual[0] = -999
    np.testing.assert_array_equal(getattr(owner, field), expected)


def main():
    feedback=ActuatorState()
    feedback.feedback_received_ns=[12345678901234567]
    feedback.feedback_timing_valid=[1]
    assert feedback.feedback_received_ns==[12345678901234567]
    assert feedback.feedback_timing_valid==[1]
    cycle=RobotCycleOutput()
    assert hasattr(cycle,'joint_acc_valid') and hasattr(cycle,'feedback_age_ms')
    assert cycle.command_written_ns == 0
    assert cycle.hold_integrator_freeze_reasons == []
    assert isinstance(cycle.calibration_disturbance, np.ndarray)
    assert cycle.calibration_disturbance.size == 0
    assert isinstance(cycle.requested_kp, np.ndarray) and cycle.requested_kp.size == 0
    assert isinstance(cycle.requested_kd, np.ndarray) and cycle.requested_kd.size == 0
    for name, value in [('command_written_ns', 123), ('hold_integrator_freeze_reasons', ['moving']),
                        ('calibration_disturbance', [0.01]), ('requested_kp', [1.0]), ('requested_kd', [0.1])]:
        try:
            setattr(cycle, name, value)
        except AttributeError:
            pass
        else:
            raise AssertionError(f'{name} must be read-only telemetry')
    state = JointState()
    for field, begin in (('pos', 1), ('vel', 11), ('tor', 21)):
        check_vector(state, field, np.arange(begin, begin + 6, dtype=np.float64))

    for field, begin in (('pos', 31), ('vel', 41), ('tor', 51)):
        check_vector(ActuatorState(), field, np.arange(begin, begin + 6, dtype=np.float64))

    ctrl = JointCtrlCmd()
    for field, begin in (('pos', 61), ('vel', 71), ('tor', 81), ('kp', 91), ('kd', 101)):
        check_vector(ctrl, field, np.arange(begin, begin + 6, dtype=np.float64))

    # A non-contiguous input is accepted by the forcecast C++ binding
    check_vector(state, 'pos', np.arange(2.0, 14.0, dtype=np.float64)[::2])

    pose = DynamicsState().tool_pose
    assert pose.shape == (4, 4), pose.shape
    assert pose.dtype == np.dtype('float64'), pose.dtype
    assert pose.strides == (32, 8), pose.strides
    np.testing.assert_array_equal(pose, np.eye(4))

    print('PASS: SerialArm Python/NumPy binding arrays and tool pose')


if __name__ == '__main__':
    main()
