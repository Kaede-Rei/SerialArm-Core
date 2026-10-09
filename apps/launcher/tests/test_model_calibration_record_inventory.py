"""Record inventory reads current disk state, including fault checkpoints"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from persistence import load_model_calibration_summary, list_model_calibration_records


class ModelCalibrationRecordInventoryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.folder = self.root / '.install/model-calibration/model-calibration-123456'
        self.folder.mkdir(parents=True)
        (self.folder / 'metadata.json').write_text(json.dumps({
            'task_id': 'model-calibration-123456', 'joint_names': ['joint1', 'joint2']
        }), encoding='utf-8')

    def write_track(self, name):
        lines = ['sample_dt,0.02', 'index,joint1,joint2']
        lines += [f'{i},{0.001*i},{-0.001*i}' for i in range(22)]
        (self.folder / name).write_text('\n'.join(lines) + '\n', encoding='utf-8')

    def test_existing_track_recovers_without_restart(self):
        self.assertFalse(load_model_calibration_summary(self.folder)['can_resume'])
        self.write_track('trajectory.csv')
        entry = load_model_calibration_summary(self.folder)
        self.assertTrue(entry['can_resume'])
        self.assertEqual(entry['trajectory']['source'], 'trajectory.csv')
        self.assertTrue(entry['files']['trajectory.csv'])
        self.assertTrue(list_model_calibration_records(self.root)[0]['can_resume'])

    def test_partial_checkpoint_is_identified_separately(self):
        self.write_track('trajectory.checkpoint.csv')
        entry = load_model_calibration_summary(self.folder)
        self.assertTrue(entry['can_resume'])
        self.assertEqual(entry['record_kind'], 'interrupted')
        self.assertEqual(entry['trajectory']['source'], 'trajectory.checkpoint.csv')
        self.assertFalse(entry['files']['trajectory.csv'])

    def test_complete_record_preferred_over_checkpoint(self):
        self.write_track('trajectory.checkpoint.csv')
        self.write_track('trajectory.csv')
        entry = load_model_calibration_summary(self.folder)
        self.assertEqual(entry['trajectory']['source'], 'trajectory.csv')

    def test_malformed_checkpoint_not_marked_as_replayable(self):
        (self.folder / 'trajectory.checkpoint.csv').write_text('invalid data')
        records = list_model_calibration_records(self.root)
        self.assertEqual(len(records), 1)
        self.assertFalse(records[0]['can_resume'])
        self.assertEqual(records[0]['record_kind'], 'invalid')


if __name__ == '__main__':
    unittest.main()
