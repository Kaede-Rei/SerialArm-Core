"""Regression tests for independently filtered, phase-stratified blocks."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'apps/launcher/backend'))
import dynamic_identification as dynamic


class TestMotionSplit(unittest.TestCase):
    def test_balanced_blocks_and_stratified_motion(self):
        # Four unrelated synthetic phases, each 14 seconds at 50 Hz; a
        # static interval must never enter either motion partition.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'metadata.json').write_text(json.dumps({'joint_names': ['j']}))
            fields = ['monotonic_ns', 'phase', 'valid', 'feedback_age_ms',
                      'q:j', 'dq:j', 'tau:j']
            phases = ['static_forward'] + list(dynamic.PHASES[2:])
            with (root / 'frames.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for p, phase in enumerate(phases):
                    for i in range(700):
                        index = p * 700 + i
                        t = index * .02
                        writer.writerow({'monotonic_ns': index * 20_000_000,
                                         'phase': phase, 'valid': '1', 'feedback_age_ms': '0',
                                         'q:j': np.sin(t), 'dq:j': np.cos(t), 'tau:j': np.sin(t)})
            observations = dynamic.load_observations(root)
            for phase in phases:
                blocks = [s for s in observations['segments'] if s['phase'] == phase]
                self.assertEqual(len(blocks), 2)
                self.assertEqual([s['hi'] - s['lo'] for s in blocks], [350, 350])

            class Engine:
                cols = [0, 1, 2, 3]
                theta = np.zeros(4)

                @staticmethod
                def Y(q, v, a):
                    return np.array([[1., q[0], v[0], a[0]]])

            z = dynamic._motion_only(dynamic.preprocess(observations, Engine(),
                                                       np.arange(4), np.eye(4)))
            train, held = dynamic._disjoint_split(z)
            self.assertFalse(set(train['segment']) & set(held['segment']))
            self.assertFalse(any(p.startswith('static') for p in train['phase']))
            coverage = dynamic._motion_coverage(train, held)
            self.assertTrue(coverage['all_motion_phases_in_both'])
            for entry in coverage['phases'].values():
                self.assertEqual(entry['training']['blocks'], 1)
                self.assertEqual(entry['validation']['blocks'], 1)
                self.assertGreater(entry['training']['moving_equations_per_joint'][0], 30)
                self.assertGreater(entry['validation']['moving_equations_per_joint'][0], 30)

    def test_single_block_phase_is_not_claimed_as_held_out(self):
        phase = dynamic.PHASES[2:]
        z = {'segment': np.arange(5),
             'phase': np.array([phase[0], phase[0], phase[1], phase[2], phase[3]]),
             'moving': np.ones((5, 1), dtype=bool)}
        train, held = dynamic._disjoint_split(z)
        coverage = dynamic._motion_coverage(train, held)
        self.assertFalse(coverage['all_motion_phases_in_both'])
        self.assertEqual(coverage['phases'][phase[1]]['validation']['blocks'], 0)
        self.assertEqual(coverage['phases'][phase[1]]['training']['blocks'], 1)

    def test_blocks_do_not_cross_invalid_phase_or_time_gaps(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'metadata.json').write_text(json.dumps({'joint_names': ['j']}))
            fields = ['monotonic_ns', 'phase', 'valid', 'feedback_age_ms',
                      'q:j', 'dq:j', 'tau:j']
            with (root / 'frames.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for i in range(320):
                    writer.writerow({'monotonic_ns': i * 20_000_000 + (100_000_000 if i >= 240 else 0),
                                     'phase': dynamic.PHASES[2 if i < 160 else 3],
                                     'valid': '0' if i == 80 else '1',
                                     'feedback_age_ms': '0', 'q:j': '0', 'dq:j': '0', 'tau:j': '0'})
            d = dynamic.load_observations(root)
            self.assertEqual([(s['lo'], s['hi']) for s in d['segments']],
                             [(0, 80), (81, 160), (160, 240), (240, 320)])
            for seg in d['segments']:
                lo, hi = seg['lo'], seg['hi']
                self.assertTrue(d['valid'][lo:hi].all())
                self.assertTrue((d['phase'][lo:hi] == seg['phase']).all())
                self.assertTrue((np.diff(d['t'][lo:hi]) <= .025).all())

    def test_no_splittable_phase_requires_more_data(self):
        z = {'segment': np.arange(4), 'phase': np.array(dynamic.PHASES[2:]),
             'moving': np.ones((4, 1), dtype=bool)}
        with self.assertRaisesRegex(ValueError, '无法分层留出'):
            dynamic._disjoint_split(z)

    def test_split_prefers_motion_in_both_partitions_over_last_block(self):
        phases = dynamic.PHASES[2:]
        # The last block in the first phase is stationary, though two other
        # blocks can supply motion to both sides. In the second phase only one
        # block moves, so it must be retained for training.
        block_phases = [phases[0]] * 3 + [phases[1]] * 2 + [phases[2]] * 2 + [phases[3]] * 2
        moving_blocks = [True, True, False, True, False, True, True, True, True]
        z = {'segment': np.repeat(np.arange(len(block_phases)), 10),
             'phase': np.repeat(block_phases, 10),
             'moving': np.repeat(np.array(moving_blocks)[:, None], 10, axis=0)}
        train, held = dynamic._disjoint_split(z)
        held_segments = set(held['segment'])
        self.assertEqual(len(held_segments & {0, 1}), 1)
        self.assertNotIn(2, held_segments)
        self.assertNotIn(3, held_segments)
        self.assertIn(4, held_segments)
        coverage = dynamic._motion_coverage(train, held)
        self.assertGreater(coverage['phases'][phases[0]]['training']['moving_equations_per_joint'][0], 0)
        self.assertGreater(coverage['phases'][phases[0]]['validation']['moving_equations_per_joint'][0], 0)
        self.assertEqual(coverage['phases'][phases[1]]['validation']['moving_equations_per_joint'][0], 0)


if __name__ == '__main__':
    unittest.main()
