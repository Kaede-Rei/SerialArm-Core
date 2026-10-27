from pathlib import Path
import csv
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest


TOOL = Path(__file__).resolve().parents[3] / 'tools/audit_calibration_kinematics.py'


class AuditCliTests(unittest.TestCase):
    def fixture(self, root, age=2):
        path = root / 'frames.csv'
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['monotonic_ns', 'q:joint1', 'dq:joint1', 'phase', 'valid', 'feedback_age_ms'])
            for i in range(3001):
                t = i * .01
                writer.writerow([1000000000+i*10000000, .4*math.sin(.8*t),
                                 .32*math.cos(.8*t)/1.25-.002, 'static_reverse', 1, age])
        return path

    def run_cli(self, *args):
        return subprocess.run([sys.executable, '-B', str(TOOL), *map(str,args)],
                              env={**os.environ, 'PYTHONDONTWRITEBYTECODE':'1'},
                              text=True, capture_output=True, timeout=30)

    def test_stdout_report_preserves_source_and_reports_scale(self):
        with tempfile.TemporaryDirectory() as d:
            path=self.fixture(Path(d)); before=path.read_bytes()
            run=self.run_cli(path)
            self.assertEqual(run.returncode,0,run.stderr)
            result=json.loads(run.stdout)[0]
            self.assertTrue(result['source_unchanged'])
            self.assertEqual(result['audit']['status'],'requires_review')
            self.assertAlmostEqual(result['audit']['joints'][0]['scale'],1.25,places=4)
            self.assertEqual(path.read_bytes(),before)

    def test_existing_output_and_source_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); source=self.fixture(root); output=root/'report.json'
            output.write_text('preserve-me')
            for target in (source,output):
                before=target.read_bytes(); run=self.run_cli(source,'--output',target)
                self.assertNotEqual(run.returncode,0)
                self.assertEqual(target.read_bytes(),before)

    def test_negative_feedback_age_is_excluded(self):
        with tempfile.TemporaryDirectory() as d:
            source=self.fixture(Path(d),age=-1);run=self.run_cli(source)
            self.assertEqual(run.returncode,0,run.stderr)
            report=json.loads(run.stdout)[0]['audit']
            self.assertEqual(report['used_windows'],0)
            self.assertEqual(report['status'],'limited_excitation')


if __name__ == '__main__':
    unittest.main()
