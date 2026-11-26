import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dynamic_identification import preprocess


class ConstantRegressor:
    cols = np.array([0])
    theta = np.array([2.0])

    def Y(self, q, v, a):
        return np.ones((1, 1))


def observations(sample_rates, torque_frequency=None):
    segments, times, positions, torques = [], [], [], []
    start_index = 0
    for index, fs in enumerate(sample_rates):
        t = index * 12.0 + np.arange(int(fs * 10)) / fs
        q = 0.2 * t
        tau = np.full_like(t, 2.0)
        if torque_frequency is not None:
            tau += 0.5 * np.sin(2 * np.pi * torque_frequency * t)
        segments.append(dict(id=index, lo=start_index, hi=start_index + len(t),
                             phase='friction_forward_slow'))
        start_index += len(t)
        times.extend(t)
        positions.extend(q)
        torques.extend(tau)
    return dict(names=['joint1'], t=np.asarray(times), q=np.asarray(positions)[:, None],
                tau=np.asarray(torques)[:, None], segments=segments)


class DynamicSamplingTest(unittest.TestCase):
    def test_uneven_source_rates_share_exact_20_hz_grid(self):
        data = observations([50.0 - 1e-9, 50.0 + 1e-9, 100.0])
        result = preprocess(data, ConstantRegressor(), np.array([0]), np.ones((1, 1)))
        for segment in data['segments']:
            selected = result['segment'] == segment['id']
            time = result['time'][selected]
            np.testing.assert_allclose(np.diff(time), 0.05, rtol=0, atol=1e-12)
            self.assertGreater(time[0], data['t'][segment['lo']] + 1.5)
            self.assertLess(time[-1], data['t'][segment['hi'] - 1] - 1.5)
        np.testing.assert_allclose(result['baseline'], result['tau'], atol=1e-12)
        np.testing.assert_allclose(result['X'][:, 0, 0], 1.0, atol=1e-12)
        np.testing.assert_allclose(result['q'][:, 0], 0.2 * result['time'], atol=1e-4)
        np.testing.assert_array_equal(result['moving'], np.abs(result['v']) > 0.03)

    def test_antialias_filter_suppresses_out_of_band_torque(self):
        data = observations([80.0], torque_frequency=15.0)
        result = preprocess(data, ConstantRegressor(), np.array([0]), np.ones((1, 1)))
        self.assertLess(np.max(np.abs(result['tau'] - 2.0)), 0.001)


if __name__ == '__main__':
    unittest.main()
