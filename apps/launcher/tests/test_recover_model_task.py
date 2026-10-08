"""Regression tests for task-folder inspection without result.json."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backend"))
from persistence import load_model_calibration_summary


class RecoverModelTaskTests(unittest.TestCase):
    def make_record(self, root, trajectory=True, result=None):
        folder = Path(root) / "model-calibration-sample"
        folder.mkdir()
        (folder / "metadata.json").write_text(json.dumps({"task_id":"model-calibration-sample", "joint_names":["joint1","joint2"], "calibration_options":{"pose_budget":8}}))
        (folder / "frames.csv").write_text("header\n")
        if trajectory:
            (folder / "trajectory.csv").write_text('sample_dt,0.01\nindex,joint1,joint2\n' + ''.join(f'{i},{i*0.002},{-i*0.002}\n' for i in range(25)))
        if result is not None:
            (folder / "result.json").write_text(json.dumps({"task_id":"model-calibration-sample", "phase":result}))
        return folder

    def test_recorded_demo_without_result_is_recoverable(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = self.make_record(temp)
            original = (folder / 'trajectory.csv').read_bytes()
            found = load_model_calibration_summary(folder)
            self.assertEqual(found['record_kind'], 'teaching_only')
            self.assertEqual(found['result']['phase'], 'recorded_only')
            self.assertTrue(found['can_resume'])
            self.assertFalse(found['has_result'])
            self.assertEqual(found['trajectory']['samples'], 25)
            self.assertAlmostEqual(found['trajectory']['duration_s'], 0.24)
            self.assertEqual(original, (folder / 'trajectory.csv').read_bytes())
            self.assertFalse((folder / 'result.json').exists())

    def test_interrupted_record_can_restart_if_trajectory_complete(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = self.make_record(temp, result='failed')
            found = load_model_calibration_summary(folder)
            self.assertEqual(found['record_kind'], 'interrupted')
            self.assertTrue(found['can_resume'])
            self.assertEqual(found['result']['phase'], 'failed')

    def test_fault_interrupted_demo_is_recoverable_without_result(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = self.make_record(temp)
            (folder / 'interrupted.txt').write_text('Teaching interrupted by robot fault')
            found = load_model_calibration_summary(folder)
            self.assertEqual(found['record_kind'], 'interrupted')
            self.assertTrue(found['can_resume'])
            self.assertFalse(found['has_result'])
            self.assertEqual(found['result']['phase'], 'recorded_only')

    def test_completed_record_remains_available(self):
        with tempfile.TemporaryDirectory() as temp:
            found = load_model_calibration_summary(self.make_record(temp, result='complete'))
            self.assertEqual(found['record_kind'], 'completed')
            self.assertTrue(found['has_result'])
            self.assertTrue(found['can_resume'])

    def test_partial_task_without_trajectory_is_view_only(self):
        with tempfile.TemporaryDirectory() as temp:
            found = load_model_calibration_summary(self.make_record(temp, trajectory=False))
            self.assertEqual(found['record_kind'], 'interrupted')
            self.assertFalse(found['can_resume'])

    def test_corrupt_trajectory_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = self.make_record(temp)
            (folder/'trajectory.csv').write_text('sample_dt,0.01\nindex,joint2,joint1\n0,0,0\n1,0,0\n')
            with self.assertRaisesRegex(ValueError, '关节列'):
                load_model_calibration_summary(folder)

    def test_nonfinite_trajectory_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = self.make_record(temp)
            (folder/'trajectory.csv').write_text('sample_dt,0.01\nindex,joint1,joint2\n0,0,nan\n1,0,0\n')
            with self.assertRaisesRegex(ValueError, '非有限'):
                load_model_calibration_summary(folder)

    def test_result_id_mismatch_is_not_silently_treated_as_demo(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = self.make_record(temp, result='complete')
            (folder/'result.json').write_text('{"task_id":"another", "phase":"complete"}')
            with self.assertRaisesRegex(ValueError, '编号不匹配'):
                load_model_calibration_summary(folder)


if __name__ == '__main__': unittest.main()
